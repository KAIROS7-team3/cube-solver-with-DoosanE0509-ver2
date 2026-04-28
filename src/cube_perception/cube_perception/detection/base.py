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
