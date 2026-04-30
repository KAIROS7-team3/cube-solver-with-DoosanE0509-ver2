"""Gemini VLA 백엔드 (1차 시각 추출 전담).

역할:
- RGB 1장을 Gemini에 전달해 큐브 상단 중심 픽셀(`pixel_uv`)을 얻는다.
- 윗면 정보 `top_color`, `top_face_9`를 함께 추출/검증한다.

비역할:
- depth 샘플링, camera/base 3D 좌표 계산은 하지 않는다.
  (해당 책임은 `opencv_depth_backend.py`가 담당)
"""

import json
import os
from pathlib import Path
from typing import Any, Dict, List, Tuple

import numpy as np

from .base import CubeDetector

_DEFAULT_DETECT_FALLBACK = (
    'Return JSON only: {"point": [y_norm, x_norm], "top_color": "W", "top_face_9": "WWWWWWWWW"}; '
    "point 0-1000; top_color in W,R,G,Y,O,B; top_face_9 exactly 9 letters W,R,G,Y,O,B only; "
    "top_face_9 row-major on visible U face: rows left-to-right, top-to-bottom "
    "(cells 123 / 456 / 789 → indices 0..8); top_face_9[4] must equal top_color (center)."
)
_DEFAULT_STATE_FALLBACK = (
    'Return JSON with "faces" (U,R,F,D,L,B nine chars each) and "state_54". '
    "Letters W,R,G,Y,O,B only."
)


class GeminiVLADetector(CubeDetector):
    """Gemini 기반 2D/U면 시각 정보 추출기.

    - `detect()`: 단일 RGB 입력에서 `pixel_uv`, `top_color`, `top_face_9`를 반환한다.
    - `detect_state()`: 5면 이미지(B,R,F,L,D)로 전체 면 상태를 추정한다.
    - depth 기반 3D 좌표 계산은 수행하지 않는다.
    """

    _DETECT_PROMPT = Path(__file__).resolve().parent.parent / "prompts" / "gemini_detect_prompt.txt"
    _STATE_PROMPT = Path(__file__).resolve().parent.parent / "prompts" / "gemini_state_prompt.txt"

    def __init__(self, min_confidence: float = 0.5, depth_unit_scale: float = 0.001) -> None:
        self._min_confidence = float(min_confidence)
        _ = depth_unit_scale

    def detect(self, rgb: np.ndarray, depth: np.ndarray, K: np.ndarray) -> Tuple[Any, float]:
        self._validate_inputs(rgb, depth, K)

        prompt = self._read_prompt_file(self._DETECT_PROMPT, _DEFAULT_DETECT_FALLBACK)
        raw = self._call_gemini_json([rgb], prompt)
        det = self._parse_detect_response(raw)
        self._validate_detection(det)

        confidence = float(det.get("confidence", 1.0))
        if confidence < self._min_confidence:
            raise RuntimeError(
                f"confidence too low: {confidence:.3f} < {self._min_confidence:.3f}"
            )

        h, w = rgb.shape[:2]
        u = (float(det["x_norm"]) / 1000.0) * float(w)
        v = (float(det["y_norm"]) / 1000.0) * float(h)
        stub = {
            "pixel_uv": (u, v),
            "top_color": det["top_color"],
            "top_face_9": det["top_face_9"],
            "source": "gemini_vla",
        }
        return stub, confidence

    def detect_state(self, images_rgb: List[np.ndarray]) -> Dict[str, Any]:
        if len(images_rgb) != 5:
            raise ValueError("detect_state expects 5 RGB images [B,R,F,L,D].")
        for idx, img in enumerate(images_rgb):
            if img is None or img.ndim != 3 or img.shape[2] != 3:
                raise ValueError(f"images_rgb[{idx}] must be HxWx3 RGB")

        prompt = self._read_prompt_file(self._STATE_PROMPT, _DEFAULT_STATE_FALLBACK)
        raw = self._call_gemini_json(images_rgb, prompt)
        return self._parse_state_response(raw)

    def _validate_inputs(self, rgb: np.ndarray, depth: np.ndarray, K: np.ndarray) -> None:
        if rgb is None or depth is None or K is None:
            raise ValueError("rgb, depth, K must not be None")
        if rgb.ndim != 3 or rgb.shape[2] != 3:
            raise ValueError("rgb must be HxWx3")
        if depth.ndim != 2:
            raise ValueError("depth must be HxW")
        if K.shape != (3, 3):
            raise ValueError("K must be 3x3")

    @staticmethod
    def _read_prompt_file(path: Path, fallback: str) -> str:
        try:
            text = path.read_text(encoding="utf-8").strip()
            if text:
                return text
        except OSError:
            pass
        return fallback

    def _call_gemini_json(self, rgb_images: List[np.ndarray], prompt: str) -> str:
        self._load_dotenv_if_available()
        api_key = os.getenv("GEMINI_API_KEY", "").strip()
        if not api_key:
            raise RuntimeError("GEMINI_API_KEY is not set.")

        model_name = os.getenv("GEMINI_MODEL", "gemini-3.1-pro-preview").strip()

        try:
            from google import genai  # type: ignore[import-not-found]
            from google.genai import types  # type: ignore[import-not-found]
        except Exception as exc:
            raise RuntimeError("Install google-genai: pip install google-genai") from exc

        parts: List[Any] = [prompt]
        for img in rgb_images:
            blob, mime = self._encode_image_for_gemini(img)
            parts.append(types.Part.from_bytes(data=blob, mime_type=mime))

        client = genai.Client(api_key=api_key)
        response = client.models.generate_content(
            model=model_name,
            contents=parts,
            config=types.GenerateContentConfig(
                response_mime_type="application/json",
                temperature=0.0,
            ),
        )
        text = (response.text or "").strip()
        if not text:
            raise RuntimeError("Empty Gemini response")
        return text

    @staticmethod
    def _load_dotenv_if_available() -> None:
        try:
            from dotenv import load_dotenv  # type: ignore

            load_dotenv()
        except Exception:
            pass

    def _encode_image_for_gemini(self, rgb: np.ndarray) -> Tuple[bytes, str]:
        try:
            import cv2

            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            ok, buf = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
            if ok:
                return buf.tobytes(), "image/jpeg"
        except Exception:
            pass

        try:
            from io import BytesIO
            from PIL import Image

            img = Image.fromarray(rgb)
            bio = BytesIO()
            img.save(bio, format="JPEG", quality=95)
            return bio.getvalue(), "image/jpeg"
        except Exception as exc:
            raise RuntimeError("Need opencv-python or pillow for JPEG encoding") from exc

    def _parse_detect_response(self, raw_output: str) -> Dict[str, Any]:
        try:
            data = json.loads(raw_output)
        except json.JSONDecodeError as exc:
            raise ValueError(f"Not valid JSON: {raw_output[:200]}") from exc

        if "point" not in data:
            raise ValueError(f"Missing 'point': {data}")
        point = data["point"]
        if not isinstance(point, (list, tuple)) or len(point) != 2:
            raise ValueError(f"Bad 'point': {point}")
        top_color = str(data.get("top_color", "")).strip().upper()
        if top_color not in {"W", "R", "G", "Y", "O", "B"}:
            raise ValueError(f"Bad 'top_color': {data.get('top_color')}")
        top_face_9 = "".join(
            ch for ch in str(data.get("top_face_9", "")).upper() if not ch.isspace()
        )
        if len(top_face_9) != 9:
            raise ValueError(f"Bad 'top_face_9' length: {top_face_9}")
        if any(ch not in {"W", "R", "G", "Y", "O", "B"} for ch in top_face_9):
            raise ValueError(f"Bad 'top_face_9' chars: {top_face_9}")
        if top_face_9[4] != top_color:
            raise ValueError(
                f"top_color and top_face_9 center mismatch: {top_color} vs {top_face_9[4]}"
            )

        return {
            "x_norm": float(point[1]),
            "y_norm": float(point[0]),
            "confidence": float(data.get("confidence", 1.0)),
            "top_color": top_color,
            "top_face_9": top_face_9,
        }

    def _parse_state_response(self, raw_output: str) -> Dict[str, Any]:
        try:
            data = json.loads(raw_output)
        except json.JSONDecodeError as exc:
            raise ValueError(f"Not valid JSON: {raw_output[:200]}") from exc

        if "faces" not in data or "state_54" not in data:
            raise ValueError("JSON must have 'faces' and 'state_54'")

        faces = data["faces"]
        if not isinstance(faces, dict):
            raise ValueError("'faces' must be an object")

        required = ["U", "R", "F", "D", "L", "B"]
        allowed = set("WRGYOB")
        out_faces: Dict[str, str] = {}
        for f in required:
            raw_val = faces.get(f, "")
            s_raw = str(raw_val)
            s = "".join(ch for ch in s_raw if not ch.isspace())
            if len(s) != 9:
                raise ValueError(
                    f"faces.{f} must be 9 sticker letters (after removing whitespace); "
                    f"raw_len={len(s_raw)} norm_len={len(s)}: {repr(s_raw)[:220]}"
                )
            if any(ch not in allowed for ch in s):
                bad_positions = [(i, repr(ch)) for i, ch in enumerate(s) if ch not in allowed]
                raise ValueError(
                    f"faces.{f} invalid chars: repr={repr(s)[:220]} bad={bad_positions[:12]}"
                )
            out_faces[f] = s

        state_raw = str(data["state_54"])
        state_54 = "".join(ch for ch in state_raw if not ch.isspace())
        if len(state_54) != 54:
            raise ValueError(
                f"state_54 must be 54 chars (after removing whitespace); "
                f"raw_len={len(state_raw)} norm_len={len(state_54)}: {repr(state_raw)[:140]}"
            )
        if any(ch not in allowed for ch in state_54):
            raise ValueError("state_54 invalid chars")

        return {"faces": out_faces, "state_54": state_54}

    def _validate_detection(self, det: Dict[str, float]) -> None:
        if not (0.0 <= det["confidence"] <= 1.0):
            raise ValueError(f"confidence out of range: {det['confidence']}")
        if not (0.0 <= det["x_norm"] <= 1000.0) or not (0.0 <= det["y_norm"] <= 1000.0):
            raise ValueError(f"norm out of range: {det['x_norm']}, {det['y_norm']}")

