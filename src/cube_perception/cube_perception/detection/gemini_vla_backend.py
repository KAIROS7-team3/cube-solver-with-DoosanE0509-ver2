import json
import os
from pathlib import Path
from typing import Any, Dict, List, Tuple

import numpy as np

from .base import CubeDetector


class GeminiVLADetector(CubeDetector):
    """Gemini 기반 큐브 감지/상태 추정 백엔드."""

    _DETECT_PROMPT_FILE = (
        Path(__file__).resolve().parent.parent / "prompts" / "gemini_detect_prompt.txt"
    )
    _STATE_PROMPT_FILE = (
        Path(__file__).resolve().parent.parent / "prompts" / "gemini_state_prompt.txt"
    )

    def __init__(self, min_confidence: float = 0.5, depth_unit_scale: float = 0.001) -> None:
        # TODO(tuning): 감지 최소 신뢰도 임계값
        self._min_confidence = float(min_confidence)
        # TODO(tuning): depth 단위 스케일 (mm->m: 0.001, 이미 m면 1.0)
        self._depth_unit_scale = float(depth_unit_scale)

    def detect(self, rgb: np.ndarray, depth: np.ndarray, K: np.ndarray) -> Tuple[Any, float]:
        self._validate_inputs(rgb, depth, K)

        prompt = self._load_prompt(
            self._DETECT_PROMPT_FILE,
            (
                "You are a cube detector. Return JSON only. "
                'Output format must be exactly: {"point": [y, x]}. '
                "The point must be normalized to 0-1000 as [y, x]. "
                "Assume the cube is parallel to the camera plane. "
                "Do not return any other keys."
            ),
        )

        raw = self._call_gemini_json([rgb], prompt)
        det = self._parse_detect_response(raw)
        self._validate_detection(det)

        confidence = float(det.get("confidence", 1.0))
        if confidence < self._min_confidence:
            raise RuntimeError(
                f"Gemini detection confidence too low: {confidence:.3f} < {self._min_confidence:.3f}"
            )

        h, w = rgb.shape[:2]
        u = (float(det["x_norm"]) / 1000.0) * float(w)
        v = (float(det["y_norm"]) / 1000.0) * float(h)
        point_3d = self._pixel_to_camera_xyz(u=u, v=v, depth=depth, K=K)

        pose_stub = {
            "camera_xyz": point_3d,
            "pixel_uv": (u, v),
            "source": "gemini_vla",
        }
        return pose_stub, confidence

    def detect_state(self, images_rgb: List[np.ndarray]) -> Dict[str, Any]:
        """상태 추정 입력 5장([B,R,F,L,D])으로 cube state를 추정한다."""
        if len(images_rgb) != 5:
            raise ValueError("detect_state expects exactly 5 images in order [B, R, F, L, D].")
        for idx, img in enumerate(images_rgb):
            if img is None or img.ndim != 3 or img.shape[2] != 3:
                raise ValueError(f"images_rgb[{idx}] must be HxWx3 RGB image")

        prompt = self._load_prompt(
            self._STATE_PROMPT_FILE,
            (
                "You are a Rubik's cube state estimator. Return JSON only. "
                'Output format: {"faces": {...}, "state_54": "..."}. '
                "Use only W,R,G,Y,O,B and URFDLB face order."
            ),
        )
        raw = self._call_gemini_json(images_rgb, prompt)
        return self._parse_state_response(raw)

    def _validate_inputs(self, rgb: np.ndarray, depth: np.ndarray, K: np.ndarray) -> None:
        if rgb is None or depth is None or K is None:
            raise ValueError("rgb, depth, K must not be None")
        if rgb.ndim != 3 or rgb.shape[2] != 3:
            raise ValueError("rgb must be HxWx3 array")
        if depth.ndim != 2:
            raise ValueError("depth must be HxW array")
        if K.shape != (3, 3):
            raise ValueError("K must be a 3x3 intrinsic matrix")

    def _load_prompt(self, prompt_file: Path, fallback: str) -> str:
        # TODO(tuning): prompts/*.txt 문구 튜닝
        try:
            prompt = prompt_file.read_text(encoding="utf-8").strip()
            if prompt:
                return prompt
        except Exception:
            pass
        return fallback

    def _call_gemini_json(self, rgb_images: List[np.ndarray], prompt: str) -> str:
        self._load_dotenv_if_available()
        api_key = os.getenv("GEMINI_API_KEY", "").strip()
        if not api_key:
            raise RuntimeError("GEMINI_API_KEY is not set. Set env var or .env and retry.")

        model_name = os.getenv("GEMINI_MODEL", "gemini-robotics-er-1.6-preview").strip()

        try:
            from google import genai  # type: ignore[import-not-found]
            from google.genai import types  # type: ignore[import-not-found]
        except Exception as exc:
            raise RuntimeError(
                "google-genai package is required. Install with: pip install google-genai"
            ) from exc

        parts: List[Any] = [prompt]
        for img in rgb_images:
            image_bytes, mime_type = self._encode_image_for_gemini(img)
            parts.append(types.Part.from_bytes(data=image_bytes, mime_type=mime_type))

        client = genai.Client(api_key=api_key)
        response = client.models.generate_content(
            model=model_name,
            contents=parts,
            config=types.GenerateContentConfig(
                response_mime_type="application/json",
                # TODO(tuning): 안정성 우선(0.0). 필요시 증가.
                temperature=0.0,
                # TODO(tuning): 지연/정확도 트레이드오프
                thinking_config=types.ThinkingConfig(thinking_budget=0),
            ),
        )

        text = (response.text or "").strip()
        if not text:
            raise RuntimeError("Gemini response text is empty")
        return text

    def _load_dotenv_if_available(self) -> None:
        try:
            from dotenv import load_dotenv  # type: ignore

            load_dotenv()
        except Exception:
            pass

    def _encode_image_for_gemini(self, rgb: np.ndarray) -> Tuple[bytes, str]:
        # 1) OpenCV 우선 인코딩
        try:
            import cv2

            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            # TODO(tuning): JPEG 품질(80~95)
            ok, buf = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
            if ok:
                return buf.tobytes(), "image/jpeg"
        except Exception:
            pass

        # 2) Pillow fallback
        try:
            from io import BytesIO
            from PIL import Image

            img = Image.fromarray(rgb)
            bio = BytesIO()
            img.save(bio, format="JPEG", quality=95)
            return bio.getvalue(), "image/jpeg"
        except Exception as exc:
            raise RuntimeError(
                "Failed to encode image. Install opencv-python or pillow."
            ) from exc

    def _parse_detect_response(self, raw_output: str) -> Dict[str, float]:
        try:
            data = json.loads(raw_output)
        except json.JSONDecodeError as exc:
            raise ValueError(f"Gemini detect output is not valid JSON: {raw_output}") from exc

        if "point" not in data:
            raise ValueError(f"detect response must include 'point': {data}")
        point = data["point"]
        if not isinstance(point, (list, tuple)) or len(point) != 2:
            raise ValueError(f"Invalid point format: {point}")

        return {
            "x_norm": float(point[1]),
            "y_norm": float(point[0]),
            "confidence": float(data.get("confidence", 1.0)),
        }

    def _parse_state_response(self, raw_output: str) -> Dict[str, Any]:
        try:
            data = json.loads(raw_output)
        except json.JSONDecodeError as exc:
            raise ValueError(f"Gemini state output is not valid JSON: {raw_output}") from exc

        if "faces" not in data or "state_54" not in data:
            raise ValueError("State JSON must include 'faces' and 'state_54'")

        faces = data["faces"]
        if not isinstance(faces, dict):
            raise ValueError("'faces' must be an object")

        required_faces = ["U", "R", "F", "D", "L", "B"]
        allowed = set("WRGYOB")
        normalized_faces: Dict[str, str] = {}
        for f in required_faces:
            s = str(faces.get(f, ""))
            if len(s) != 9:
                raise ValueError(f"faces.{f} must be 9 chars")
            if any(ch not in allowed for ch in s):
                raise ValueError(f"faces.{f} contains invalid letters: {s}")
            normalized_faces[f] = s

        state_54 = str(data["state_54"])
        if len(state_54) != 54:
            raise ValueError("state_54 must be 54 chars")
        if any(ch not in allowed for ch in state_54):
            raise ValueError(f"state_54 contains invalid letters: {state_54}")

        return {"faces": normalized_faces, "state_54": state_54}

    def _validate_detection(self, det: Dict[str, float]) -> None:
        if not (0.0 <= det["confidence"] <= 1.0):
            raise ValueError(f"confidence out of range: {det['confidence']}")
        if not (0.0 <= det["x_norm"] <= 1000.0):
            raise ValueError(f"x_norm out of range: {det['x_norm']}")
        if not (0.0 <= det["y_norm"] <= 1000.0):
            raise ValueError(f"y_norm out of range: {det['y_norm']}")

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
            raise ValueError(f"pixel out of image bounds: ({u}, {v}) for ({w}, {h})")

        z_raw = float(depth[vi, ui])
        z = z_raw * self._depth_unit_scale
        if not np.isfinite(z) or z <= 0.0:
            raise ValueError(f"invalid depth value at ({ui}, {vi}): raw={z_raw}, scaled={z}")

        fx = float(K[0, 0])
        fy = float(K[1, 1])
        cx = float(K[0, 2])
        cy = float(K[1, 2])
        if fx == 0.0 or fy == 0.0:
            raise ValueError("invalid intrinsic matrix: fx/fy must be non-zero")

        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        return (x, y, z)
