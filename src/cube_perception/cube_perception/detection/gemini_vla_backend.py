import json
import os
from pathlib import Path
from typing import Any, Dict, List, Tuple

import numpy as np

from .base import CubeDetector

_DEFAULT_DETECT_FALLBACK = (
    'Return JSON only: {"point": [y_norm, x_norm], "top_color": "W"}; '
    "point normalized 0-1000 and top_color in W,R,G,Y,O,B."
)
_DEFAULT_STATE_FALLBACK = (
    'Return JSON with "faces" (U,R,F,D,L,B nine chars each) and "state_54". '
    "Letters W,R,G,Y,O,B only."
)


class GeminiVLADetector(CubeDetector):
    """Gemini 기반 큐브 감지·상태 추정."""

    _DETECT_PROMPT = Path(__file__).resolve().parent.parent / "prompts" / "gemini_detect_prompt.txt"
    _STATE_PROMPT = Path(__file__).resolve().parent.parent / "prompts" / "gemini_state_prompt.txt"

    def __init__(self, min_confidence: float = 0.5, depth_unit_scale: float = 0.001) -> None:
        self._min_confidence = float(min_confidence)
        self._depth_unit_scale = float(depth_unit_scale)

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
        cam_xyz = self._pixel_to_camera_xyz(u, v, depth, K)

        stub = {
            "camera_xyz": cam_xyz,
            "pixel_uv": (u, v),
            "top_color": det["top_color"],
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

        model_name = os.getenv("GEMINI_MODEL", "gemini-robotics-er-1.6-preview").strip()

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
                thinking_config=types.ThinkingConfig(thinking_budget=0),
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

    def _parse_detect_response(self, raw_output: str) -> Dict[str, float]:
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

        return {
            "x_norm": float(point[1]),
            "y_norm": float(point[0]),
            "confidence": float(data.get("confidence", 1.0)),
            "top_color": top_color,
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
            s = str(faces.get(f, ""))
            if len(s) != 9:
                raise ValueError(f"faces.{f} must be 9 chars")
            if any(ch not in allowed for ch in s):
                raise ValueError(f"faces.{f} invalid chars: {s}")
            out_faces[f] = s

        state_54 = str(data["state_54"])
        if len(state_54) != 54:
            raise ValueError("state_54 must be 54 chars")
        if any(ch not in allowed for ch in state_54):
            raise ValueError("state_54 invalid chars")

        return {"faces": out_faces, "state_54": state_54}

    def _validate_detection(self, det: Dict[str, float]) -> None:
        if not (0.0 <= det["confidence"] <= 1.0):
            raise ValueError(f"confidence out of range: {det['confidence']}")
        if not (0.0 <= det["x_norm"] <= 1000.0) or not (0.0 <= det["y_norm"] <= 1000.0):
            raise ValueError(f"norm out of range: {det['x_norm']}, {det['y_norm']}")

    def _pixel_to_camera_xyz(
        self,
        u: float,
        v: float,
        depth: np.ndarray,
        K: np.ndarray,
    ) -> Tuple[float, float, float]:
        h, w = depth.shape
        ui = int(round(u))
        vi = int(round(v))
        if ui < 0 or ui >= w or vi < 0 or vi >= h:
            raise ValueError(f"pixel ({u}, {v}) out of bounds for ({w}x{h})")

        z_raw = float(depth[vi, ui])
        z = z_raw * self._depth_unit_scale
        if not np.isfinite(z) or z <= 0.0:
            raise ValueError(f"invalid depth at ({ui},{vi}): raw={z_raw}, scaled={z}")

        fx = float(K[0, 0])
        fy = float(K[1, 1])
        cx = float(K[0, 2])
        cy = float(K[1, 2])
        if fx == 0.0 or fy == 0.0:
            raise ValueError("invalid intrinsics fx/fy")

        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        return (x, y, z)
