"""Detection 계층 공통 인터페이스.

이 파일은 `cube_perception`의 감지 계층 계약을 정의한다.
- `GeminiVLADetector`: 2D 위치 + U면 시각 정보 추출
- `OpenCVDepthDetector`: depth 기반 3D/base 좌표 계산

주의:
- 현재 리팩터링 구조에서 `detect()`는 주로 Gemini 경로에서 사용된다.
- OpenCV depth 계산은 `compute_from_pixel()`로 분리되어 호출된다.
"""

from abc import ABC, abstractmethod
from typing import Any, Tuple

import numpy as np


class CubeDetector(ABC):
    """감지 백엔드 공통 인터페이스."""

    COLOR_MAP = {
        "W": "White",
        "Y": "Yellow",
        "R": "Red",
        "O": "Orange",
        "G": "Green",
        "B": "Blue",
    }

    @abstractmethod
    def detect(
        self,
        rgb: np.ndarray,
        depth: np.ndarray,
        K: np.ndarray,
    ) -> Tuple[Any, float]:
        """(pose_stub, confidence). pose_stub은 노드에서 PoseStamped 로 변환."""
        ...

    def validate_color_string(self, color_str: str) -> bool:
        return len(color_str) == 9 and all(c in self.COLOR_MAP for c in color_str)
