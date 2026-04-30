import json
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String


class DebugViewerNode(Node):
    """실시간 카메라 + 디버그 1x6 썸네일 표시 GUI."""

    def __init__(self) -> None:
        super().__init__("cube_perception_debug_viewer")
        self._bridge = CvBridge()

        self.declare_parameter("color_image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("window_name", "cube_perception_debug")
        self._window_name = self.get_parameter("window_name").get_parameter_value().string_value

        self._latest_color: Optional[np.ndarray] = None
        self._first_api: Optional[np.ndarray] = None
        self._faces: Dict[str, Optional[np.ndarray]] = {"B": None, "R": None, "F": None, "L": None, "D": None}
        self._u_face_9: Optional[str] = None
        self._u_top_color: Optional[str] = None

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        color_topic = self.get_parameter("color_image_topic").get_parameter_value().string_value
        self.create_subscription(Image, color_topic, self._color_cb, qos)
        self.create_subscription(String, "/cube_perception/u_face_cache", self._u_cache_cb, 10)
        self.create_subscription(Image, "/cube_perception/debug/first_api_image", self._first_cb, 10)
        self.create_subscription(Image, "/cube_perception/debug/face_B", self._face_cb_factory("B"), 10)
        self.create_subscription(Image, "/cube_perception/debug/face_R", self._face_cb_factory("R"), 10)
        self.create_subscription(Image, "/cube_perception/debug/face_F", self._face_cb_factory("F"), 10)
        self.create_subscription(Image, "/cube_perception/debug/face_L", self._face_cb_factory("L"), 10)
        self.create_subscription(Image, "/cube_perception/debug/face_D", self._face_cb_factory("D"), 10)

        self.create_timer(0.05, self._render)  # 20 FPS
        self.get_logger().info("debug_viewer_node started")

    def _to_bgr(self, msg: Image) -> np.ndarray:
        if msg.encoding == "bgr8":
            return self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        if msg.encoding == "rgb8":
            rgb = self._bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
            return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        return self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def _color_cb(self, msg: Image) -> None:
        try:
            self._latest_color = self._to_bgr(msg)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning("color decode failed: %s", exc)

    def _first_cb(self, msg: Image) -> None:
        try:
            self._first_api = self._to_bgr(msg)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning("first_api decode failed: %s", exc)

    def _face_cb_factory(self, face: str):
        def _cb(msg: Image) -> None:
            try:
                self._faces[face] = self._to_bgr(msg)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warning("face %s decode failed: %s", face, exc)

        return _cb

    def _u_cache_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            top_face_9 = str(data.get("top_face_9", "")).strip().upper()
            top_color = str(data.get("top_color", "")).strip().upper()
            if len(top_face_9) == 9 and all(c in {"W", "R", "G", "Y", "O", "B"} for c in top_face_9):
                self._u_face_9 = top_face_9
            if top_color in {"W", "R", "G", "Y", "O", "B"}:
                self._u_top_color = top_color
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning("u cache decode failed: %s", exc)

    def _thumb_or_blank(self, img: Optional[np.ndarray], label: str, size: Tuple[int, int]) -> np.ndarray:
        w, h = size
        if img is None:
            canvas = np.zeros((h, w, 3), dtype=np.uint8)
            cv2.putText(canvas, f"{label}: N/A", (10, h // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 2)
            return canvas
        out = cv2.resize(img, (w, h), interpolation=cv2.INTER_AREA)
        cv2.rectangle(out, (0, 0), (w - 1, h - 1), (80, 80, 80), 1)
        cv2.putText(out, label, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        return out

    def _render(self) -> None:
        if self._latest_color is None:
            return

        top_h = 520
        top_w = 960
        top = cv2.resize(self._latest_color, (top_w, top_h), interpolation=cv2.INTER_AREA)
        cv2.putText(top, "LIVE CAMERA", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        u_text = self._u_face_9 if self._u_face_9 is not None else "N/A"
        c_text = self._u_top_color if self._u_top_color is not None else "N/A"
        cv2.putText(
            top,
            f"U cache: {u_text} (center={c_text})",
            (10, 65),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (0, 255, 255),
            2,
        )

        thumb_w, thumb_h = 160, 120
        thumbs: List[np.ndarray] = [
            self._thumb_or_blank(self._first_api, "1st API", (thumb_w, thumb_h)),
            self._thumb_or_blank(self._faces["B"], "B", (thumb_w, thumb_h)),
            self._thumb_or_blank(self._faces["R"], "R", (thumb_w, thumb_h)),
            self._thumb_or_blank(self._faces["F"], "F", (thumb_w, thumb_h)),
            self._thumb_or_blank(self._faces["L"], "L", (thumb_w, thumb_h)),
            self._thumb_or_blank(self._faces["D"], "D", (thumb_w, thumb_h)),
        ]
        bottom = np.hstack(thumbs)
        panel = np.vstack([top, bottom])

        cv2.imshow(self._window_name, panel)
        cv2.waitKey(1)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = DebugViewerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
