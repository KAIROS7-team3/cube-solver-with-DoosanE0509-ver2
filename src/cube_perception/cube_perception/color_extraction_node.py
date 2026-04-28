from typing import Dict, List, Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from .detection.gemini_vla_backend import GeminiVLADetector

try:
    from cube_perception.srv import ExtractFace
except ImportError:
    ExtractFace = None


class ColorExtractionNode(Node):
    VALID_FACES = {"U", "D", "R", "L", "F", "B"}
    STATE_IMAGE_ORDER = ["B", "R", "F", "L", "D"]

    def __init__(self) -> None:
        super().__init__("color_extraction_node")
        self._bridge = CvBridge()
        self._latest_bgr: Optional[np.ndarray] = None
        self._latest_faces: Optional[Dict[str, str]] = None
        self._captured_face_images_rgb: Dict[str, np.ndarray] = {}
        self._state_dirty = True

        self._state_detector = GeminiVLADetector(min_confidence=0.0, depth_unit_scale=1.0)
        self.declare_parameter("state_retry_count", 2)

        self.declare_parameter("color_image_topic", "camera/color/image_raw")
        color_topic = self.get_parameter("color_image_topic").get_parameter_value().string_value
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, color_topic, self._color_cb, qos)
        self._debug_face_pubs = {
            "B": self.create_publisher(Image, "/cube_perception/debug/face_B", 10),
            "R": self.create_publisher(Image, "/cube_perception/debug/face_R", 10),
            "F": self.create_publisher(Image, "/cube_perception/debug/face_F", 10),
            "L": self.create_publisher(Image, "/cube_perception/debug/face_L", 10),
            "D": self.create_publisher(Image, "/cube_perception/debug/face_D", 10),
        }

        self._extract_service = None
        if ExtractFace is None:
            self.get_logger().warning(
                "ExtractFace 미생성 — 패키지 빌드 후 다시 실행하세요."
            )
        else:
            self._extract_service = self.create_service(
                ExtractFace, "~/extract_face", self._extract_cb
            )
        self.get_logger().info("color_extraction_node started")

    def _color_cb(self, msg: Image) -> None:
        try:
            self._latest_bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning("color_cb failed: %s", exc)

    def _capture_frame(self) -> np.ndarray:
        if self._latest_bgr is None:
            raise RuntimeError("No color frame received yet")
        return self._latest_bgr.copy()

    def _capture_face_image(self, face: str) -> None:
        bgr = self._capture_frame()
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        self._captured_face_images_rgb[face] = rgb
        pub = self._debug_face_pubs.get(face)
        if pub is not None:
            try:
                pub.publish(self._bridge.cv2_to_imgmsg(bgr, encoding="bgr8"))
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warning("debug %s publish failed: %s", face, exc)
        self._state_dirty = True

    def _state_inputs_ready(self) -> bool:
        return all(face in self._captured_face_images_rgb for face in self.STATE_IMAGE_ORDER)

    def _ordered_state_images(self) -> List[np.ndarray]:
        if not self._state_inputs_ready():
            missing = [f for f in self.STATE_IMAGE_ORDER if f not in self._captured_face_images_rgb]
            raise RuntimeError(f"missing faces: {missing}")
        return [self._captured_face_images_rgb[f] for f in self.STATE_IMAGE_ORDER]

    def _run_state_estimation(self) -> None:
        retries = max(int(self.get_parameter("state_retry_count").get_parameter_value().integer_value), 0)
        last_exc: Optional[Exception] = None

        for _ in range(retries + 1):
            try:
                images = self._ordered_state_images()
                out = self._state_detector.detect_state(images)
                faces = out["faces"]
                state_54 = out["state_54"]
                for face in self.VALID_FACES:
                    val = str(faces[face])
                    if len(val) != 9:
                        raise ValueError(f"faces.{face} length invalid: {val}")
                if len(state_54) != 54:
                    raise ValueError(f"state_54 length invalid: {len(state_54)}")
                self._latest_faces = faces
                self._state_dirty = False
                return
            except Exception as exc:  # noqa: BLE001
                last_exc = exc

        raise RuntimeError(f"state estimation failed after retries: {last_exc}") from last_exc

    def _extract_cb(self, request, response):
        face = request.face.strip().upper()
        if face not in self.VALID_FACES:
            response.success = False
            response.colors_9 = ""
            response.message = (
                f"Invalid face '{request.face}'. Expected one of {sorted(self.VALID_FACES)}"
            )
            return response

        try:
            if face in self.STATE_IMAGE_ORDER:
                self._capture_face_image(face)

            if self._state_inputs_ready() and (self._latest_faces is None or self._state_dirty):
                self._run_state_estimation()

            if self._latest_faces is None:
                missing = [f for f in self.STATE_IMAGE_ORDER if f not in self._captured_face_images_rgb]
                raise RuntimeError(
                    f"Need captures for {self.STATE_IMAGE_ORDER} (any order). missing={missing}"
                )

            response.colors_9 = self._latest_faces[face]
            response.success = True
            response.message = "ok"
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.colors_9 = ""
            response.message = f"extract failed: {exc}"
        return response


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = ColorExtractionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
