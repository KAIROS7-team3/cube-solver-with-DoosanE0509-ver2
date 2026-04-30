"""Depth/3D 계산 전담 백엔드.

역할:
- `pixel_uv` 주변 depth를 안정적으로 샘플링한다.
- `depth + K`로 camera 좌표를 복원한다.
- `T_base_cam`으로 base 좌표를 계산한다.

입력:
- `pixel_uv`, `depth(HxW)`, `K(3x3)`, `T_base_cam(4x4)`

출력:
- `depth_m`, `camera_xyz`, `base_xyz`
"""

from __future__ import annotations

from typing import Any, Dict, Tuple

import numpy as np

from .base import CubeDetector


class OpenCVDepthDetector(CubeDetector):
    """Depth 샘플링과 3D/base 복원 전용 계산기.

    - 주 진입점은 `compute_from_pixel()`이다.
    - `pixel_uv` 주변 depth를 샘플링하고 `K`로 camera 좌표를 계산한다.
    - `T_base_cam`을 적용해 base 좌표까지 반환한다.
    """

    def __init__(self, depth_unit_scale: float = 0.001, sample_radius_px: int = 2) -> None:
        self._depth_unit_scale = float(depth_unit_scale)
        self._sample_radius_px = max(0, int(sample_radius_px))

    def detect(self, rgb: np.ndarray, depth: np.ndarray, K: np.ndarray) -> Tuple[Any, float]:
        _ = rgb
        _ = depth
        _ = K
        raise NotImplementedError(
            "OpenCVDepthDetector.detect is not used. "
            "Use compute_from_pixel(pixel_uv, depth, K, T_base_cam)."
        )

    def compute_from_pixel(
        self,
        pixel_uv: Tuple[float, float],
        depth: np.ndarray,
        K: np.ndarray,
        T_base_cam: np.ndarray,
    ) -> Dict[str, Any]:
        if depth is None or K is None or T_base_cam is None:
            raise ValueError("depth, K, T_base_cam must not be None")
        if depth.ndim != 2:
            raise ValueError("depth must be HxW")
        if K.shape != (3, 3):
            raise ValueError("K must be 3x3")
        if T_base_cam.shape != (4, 4):
            raise ValueError("T_base_cam must be 4x4")

        u = float(pixel_uv[0])
        v = float(pixel_uv[1])
        ui = int(round(u))
        vi = int(round(v))
        h, w = depth.shape
        if ui < 0 or ui >= w or vi < 0 or vi >= h:
            raise ValueError(f"pixel out of range: ({u}, {v}) for depth {w}x{h}")

        depth_m = self._sample_depth_m(depth, ui, vi)
        camera_xyz = self._pixel_to_camera_xyz(u, v, depth_m, K)
        base_xyz = self._camera_to_base(camera_xyz, T_base_cam)
        return {
            "pixel_uv": (u, v),
            "depth_m": depth_m,
            "camera_xyz": camera_xyz,
            "base_xyz": base_xyz,
            "source": "opencv_depth",
        }

    def _sample_depth_m(self, depth: np.ndarray, ui: int, vi: int) -> float:
        r = self._sample_radius_px
        x0 = max(0, ui - r)
        x1 = min(depth.shape[1], ui + r + 1)
        y0 = max(0, vi - r)
        y1 = min(depth.shape[0], vi + r + 1)
        roi = np.asarray(depth[y0:y1, x0:x1], dtype=np.float64)
        valid = roi[np.isfinite(roi) & (roi > 0.0)]
        if valid.size == 0:
            raise ValueError(f"invalid depth around ({ui}, {vi})")
        return float(np.median(valid) * self._depth_unit_scale)

    @staticmethod
    def _pixel_to_camera_xyz(
        u: float,
        v: float,
        depth_m: float,
        K: np.ndarray,
    ) -> Tuple[float, float, float]:
        fx = float(K[0, 0])
        fy = float(K[1, 1])
        cx = float(K[0, 2])
        cy = float(K[1, 2])
        if fx <= 0.0 or fy <= 0.0:
            raise ValueError("invalid camera intrinsics fx/fy")
        x = (u - cx) * depth_m / fx
        y = (v - cy) * depth_m / fy
        z = depth_m
        return (x, y, z)

    @staticmethod
    def _camera_to_base(
        camera_xyz: Tuple[float, float, float],
        T_base_cam: np.ndarray,
    ) -> Tuple[float, float, float]:
        p_cam_h = np.array([camera_xyz[0], camera_xyz[1], camera_xyz[2], 1.0], dtype=np.float64)
        p_base_h = T_base_cam @ p_cam_h
        return (float(p_base_h[0]), float(p_base_h[1]), float(p_base_h[2]))
