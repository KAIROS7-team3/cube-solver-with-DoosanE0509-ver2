from typing import Any, Tuple

import numpy as np

from .base import CubeDetector


class OpenCVDepthDetector(CubeDetector):
    """OpenCV + depth 기반 큐브 감지 백엔드 스켈레톤."""

    def detect(
        self,
        rgb: np.ndarray,
        depth: np.ndarray,
        K: np.ndarray,
    ) -> Tuple[Any, float]:
        if rgb is None or depth is None or K is None:
            raise ValueError("rgb, depth, K must not be None")

        if rgb.ndim != 3 or rgb.shape[2] != 3:
            raise ValueError("rgb must be HxWx3 array")
        if depth.ndim != 2:
            raise ValueError("depth must be HxW array")
        if K.shape != (3, 3):
            raise ValueError("K must be a 3x3 intrinsic matrix")

        # TODO: 실제 OpenCV 기반 큐브 pose 추정 로직 구현
        # 반환 타입은 (PoseStamped, confidence)를 따라야 한다.
        raise NotImplementedError("OpenCVDepthDetector.detect is not implemented yet")
