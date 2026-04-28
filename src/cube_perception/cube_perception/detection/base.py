from abc import ABC, abstractmethod
from typing import Any, Tuple

import numpy as np


class CubeDetector(ABC):
    """큐브 감지 백엔드(OpenCV, VLA 등)의 공통 인터페이스."""

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
        """
        큐브 pose와 confidence를 반환한다.

        Returns:
            tuple: (pose, confidence)
                - pose: geometry_msgs.msg.PoseStamped (런타임에서 사용)
                - confidence: 0.0 ~ 1.0
        """
        raise NotImplementedError

    def validate_color_string(self, color_str: str) -> bool:
        """9자리 색상 문자열이 유효한 색상 코드인지 검증."""
        return len(color_str) == 9 and all(c in self.COLOR_MAP for c in color_str)