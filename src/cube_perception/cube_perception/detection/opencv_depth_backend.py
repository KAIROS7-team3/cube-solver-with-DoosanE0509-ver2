"""로컬 CV 백엔드 (미구현 시 Gemini 없이 테스트할 자리)."""

from typing import Any, Tuple

import numpy as np

from .base import CubeDetector


class OpenCVDepthDetector(CubeDetector):
    """RGB·Depth·K 로 큐브 후보 추정 — 추후 contour/ROI 등 붙일 예정."""

    def detect(
        self,
        rgb: np.ndarray,
        depth: np.ndarray,
        K: np.ndarray,
    ) -> Tuple[Any, float]:
        if rgb is None or depth is None or K is None:
            raise ValueError("rgb, depth, K must not be None")
        if rgb.ndim != 3 or rgb.shape[2] != 3:
            raise ValueError("rgb must be HxWx3")
        if depth.ndim != 2:
            raise ValueError("depth must be HxW")
        if K.shape != (3, 3):
            raise ValueError("K must be 3x3")

        raise NotImplementedError("OpenCVDepthDetector.detect not implemented yet")
