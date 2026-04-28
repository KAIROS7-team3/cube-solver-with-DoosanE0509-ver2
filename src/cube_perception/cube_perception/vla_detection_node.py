from __future__ import annotations

from typing import Any, List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header

from .detection.base import CubeDetector
from .detection.gemini_vla_backend import GeminiVLADetector
from .detection.opencv_depth_backend import OpenCVDepthDetector

try:
    from cube_perception.srv import DetectCubePose
except ImportError:
    DetectCubePose = None


def _T_base_from_param(row_major_16: List[float]) -> np.ndarray:
    if len(row_major_16) != 16:
        raise ValueError("T_base_cam must have exactly 16 elements (row-major 4x4).")
    return np.array(row_major_16, dtype=np.float64).reshape(4, 4)


def _camera_point_to_base(T_base_cam: np.ndarray, p_cam: Tuple[float, float, float]) -> np.ndarray:
    p_h = np.array([p_cam[0], p_cam[1], p_cam[2], 1.0], dtype=np.float64)
    p_base = T_base_cam @ p_h
    return p_base[:3]


def _K_from_camera_info(msg: CameraInfo) -> np.ndarray:
    k = list(msg.k)
    if len(k) < 9:
        raise ValueError("CameraInfo.k must have at least 9 elements")
    return np.array(
        [[k[0], k[1], k[2]], [k[3], k[4], k[5]], [k[6], k[7], k[8]]], dtype=np.float64
    )


def _quat_wxyz() -> Tuple[float, float, float, float]:
    return (1.0, 0.0, 0.0, 0.0)


class VlaDetectionNode(Node):
    """카메라 구독 → 백엔드 `camera_xyz` → `T_base_cam` 으로 `base` 프레임 PoseStamped 발행."""

    def __init__(self) -> None:
        super().__init__("vla_detection_node")
        self._bridge = CvBridge()

        self.declare_parameter("detector_backend", "opencv")
        self.declare_parameter("min_confidence", 0.5)
        self.declare_parameter("depth_unit_scale", 0.001)
        self.declare_parameter("color_image_topic", "camera/color/image_raw")
        self.declare_parameter("depth_image_topic", "camera/depth/image_raw")
        self.declare_parameter("camera_info_topic", "camera/color/camera_info")
        self.declare_parameter(
            "T_base_cam",
            [
                1.0,
                0.0,
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
                0.0,
                0.0,
                0.0,
                1.0,
            ],
        )
        self.declare_parameter("output_frame_id", "base_link")

        backend = (
            self.get_parameter("detector_backend")
            .get_parameter_value()
            .string_value.strip()
            .lower()
        )
        min_c = float(self.get_parameter("min_confidence").get_parameter_value().double_value)
        depth_scale = float(
            self.get_parameter("depth_unit_scale").get_parameter_value().double_value
        )
        self._T_base_cam = _T_base_from_param(
            list(self.get_parameter("T_base_cam").get_parameter_value().double_array_value)
        )
        self._output_frame = (
            self.get_parameter("output_frame_id").get_parameter_value().string_value
        )

        self._detector: CubeDetector = self._load_backend(
            backend, min_confidence=min_c, depth_unit_scale=depth_scale
        )

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self._latest_bgr: Optional[np.ndarray] = None
        self._latest_depth: Optional[np.ndarray] = None
        self._K: Optional[np.ndarray] = None
        self._last_color_stamp: Any = None  # sensor_msgs/Time or None
        self._latest_top_color: Optional[str] = None
        self._debug_first_api_pub = self.create_publisher(
            Image, "/cube_perception/debug/first_api_image", 10
        )

        color_topic = self.get_parameter("color_image_topic").get_parameter_value().string_value
        depth_topic = self.get_parameter("depth_image_topic").get_parameter_value().string_value
        info_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value

        self.create_subscription(Image, color_topic, self._color_cb, qos)
        self.create_subscription(Image, depth_topic, self._depth_cb, qos)
        self.create_subscription(CameraInfo, info_topic, self._info_cb, 10)

        self._detect_service: Any = None
        if DetectCubePose is None:
            self.get_logger().warning(
                "DetectCubePose 타입 로드 실패. colcon build 후 srv 생성 여부 확인."
            )
        else:
            self._detect_service = self.create_service(
                DetectCubePose, "detect_cube_pose", self._detect_cb
            )

        self.get_logger().info(
            f"vla_detection_node started, backend={self._detector.__class__.__name__}, "
            f"output_frame={self._output_frame}"
        )

    def _load_backend(
        self, name: str, min_confidence: float, depth_unit_scale: float
    ) -> CubeDetector:
        if name == "opencv":
            return OpenCVDepthDetector()
        if name == "gemini":
            return GeminiVLADetector(
                min_confidence=min_confidence,
                depth_unit_scale=depth_unit_scale,
            )
        raise ValueError(
            f"Unsupported detector_backend '{name}'. Supported: opencv, gemini"
        )

    def _color_cb(self, msg: Image) -> None:
        try:
            self._latest_bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self._last_color_stamp = msg.header.stamp
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning("color_cb: %s", exc)

    def _depth_cb(self, msg: Image) -> None:
        try:
            enc = msg.encoding
            if enc in ("16UC1", "mono16"):
                self._latest_depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding=enc)
            else:
                self.get_logger().warning("depth encoding %s; expected 16UC1/mono16", enc)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning("depth_cb: %s", exc)

    def _info_cb(self, msg: CameraInfo) -> None:
        try:
            self._K = _K_from_camera_info(msg)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning("info_cb: %s", exc)

    def _detect_cb(self, request, response) -> Any:
        _ = request.hint
        if self._latest_bgr is None or self._latest_depth is None or self._K is None:
            response.success = False
            response.message = "Camera frames or camera_info not ready."
            response.confidence = 0.0
            return response

        bgr = self._latest_bgr.copy()
        depth = self._latest_depth.copy()
        K = self._K.copy()
        try:
            self._debug_first_api_pub.publish(self._bridge.cv2_to_imgmsg(bgr, encoding="bgr8"))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning("debug first_api publish failed: %s", exc)
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        if depth.shape[:2] != rgb.shape[:2]:
            response.success = False
            response.message = (
                f"depth/color 크기 불일치: depth {depth.shape} vs color {rgb.shape[:2]}"
            )
            response.confidence = 0.0
            return response

        try:
            stub, conf = self._detector.detect(rgb, depth, K)
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = f"detect failed: {exc}"
            response.confidence = 0.0
            return response

        cam_xyz: Optional[Tuple[float, float, float]] = None
        if isinstance(stub, dict) and "camera_xyz" in stub:
            t = stub["camera_xyz"]
            cam_xyz = (float(t[0]), float(t[1]), float(t[2]))
            top_color = stub.get("top_color")
            if isinstance(top_color, str) and top_color in {"W", "R", "G", "Y", "O", "B"}:
                self._latest_top_color = top_color
                self.get_logger().info(f"cached top_color={self._latest_top_color}")
        else:
            response.success = False
            response.message = "Backend did not return camera_xyz in stub dict."
            response.confidence = float(conf) if conf is not None else 0.0
            return response

        p_base = _camera_point_to_base(self._T_base_cam, cam_xyz)
        qw, qx, qy, qz = _quat_wxyz()

        pose = PoseStamped()
        pose.header = Header()
        if self._last_color_stamp is not None:
            pose.header.stamp = self._last_color_stamp
        else:
            pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self._output_frame
        pose.pose = Pose(
            position=Point(x=float(p_base[0]), y=float(p_base[1]), z=float(p_base[2])),
            orientation=Quaternion(x=qx, y=qy, z=qz, w=qw),
        )

        response.pose = pose
        response.confidence = float(conf)
        response.success = True
        response.message = "ok"
        return response


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[Node] = None
    try:
        node = VlaDetectionNode()
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
