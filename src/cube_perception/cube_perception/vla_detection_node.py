from __future__ import annotations

"""DetectCubePose 서비스 오케스트레이션 노드.

처리 흐름:
1) RGB/depth/K 수집
2) `GeminiVLADetector.detect()`로 `pixel_uv`, `top_face_9` 획득
3) `OpenCVDepthDetector.compute_from_pixel()`로 3D/base 좌표 계산
4) `PoseStamped(frame_id=base_link)` 조립 후 서비스 응답

부가 기능:
- 첫 호출의 U면 정보(`top_face_9`)를 `/cube_perception/u_face_cache`로 publish
  하여 `color_extraction_node`가 재사용할 수 있게 한다.
"""

import json
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
from std_msgs.msg import String

from .detection.gemini_vla_backend import GeminiVLADetector
from .detection.opencv_depth_backend import OpenCVDepthDetector

try:
    from cube_interfaces.srv import DetectCubePose
except ImportError:
    DetectCubePose = None


def _T_base_from_param(row_major_16: List[float]) -> np.ndarray:
    if len(row_major_16) != 16:
        raise ValueError("T_base_cam must have exactly 16 elements (row-major 4x4).")
    return np.array(row_major_16, dtype=np.float64).reshape(4, 4)


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
    """`DetectCubePose` 서비스 오케스트레이터.

    - Gemini 백엔드에서 `pixel_uv`, `top_face_9`를 얻는다.
    - Depth 백엔드에서 `base_xyz`를 계산한다.
    - 최종 `PoseStamped(base_link)`를 조립해 서비스 응답으로 반환한다.
    - U면 캐시(`top_face_9`)를 토픽으로 publish하여 색 추출 노드와 공유한다.
    """

    def __init__(self) -> None:
        super().__init__("vla_detection_node")
        self._bridge = CvBridge()

        self.declare_parameter("detector_backend", "gemini_depth_split")
        self.declare_parameter("min_confidence", 0.5)
        self.declare_parameter("depth_unit_scale", 0.001)
        self.declare_parameter("depth_sample_radius_px", 2)
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

        min_c = float(self.get_parameter("min_confidence").get_parameter_value().double_value)
        depth_scale = float(
            self.get_parameter("depth_unit_scale").get_parameter_value().double_value
        )
        depth_sample_radius = int(
            self.get_parameter("depth_sample_radius_px").get_parameter_value().integer_value
        )
        self._T_base_cam = _T_base_from_param(
            list(self.get_parameter("T_base_cam").get_parameter_value().double_array_value)
        )
        self._output_frame = (
            self.get_parameter("output_frame_id").get_parameter_value().string_value
        )

        self._gemini_detector = GeminiVLADetector(
            min_confidence=min_c,
            depth_unit_scale=depth_scale,
        )
        self._depth_backend = OpenCVDepthDetector(
            depth_unit_scale=depth_scale,
            sample_radius_px=depth_sample_radius,
        )

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self._latest_bgr: Optional[np.ndarray] = None
        self._latest_depth: Optional[np.ndarray] = None
        self._K: Optional[np.ndarray] = None
        self._last_color_stamp: Any = None  # sensor_msgs/Time or None
        self._latest_top_color: Optional[str] = None
        self._latest_top_face_9: Optional[str] = None
        self._u_face_cache_pub = self.create_publisher(String, "/cube_perception/u_face_cache", 10)
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

        self.get_logger().info(f"vla_detection_node started, output_frame={self._output_frame}")

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
            stub, conf = self._gemini_detector.detect(rgb, depth, K)
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = f"detect failed: {exc}"
            response.confidence = 0.0
            return response

        if not isinstance(stub, dict) or "pixel_uv" not in stub:
            response.success = False
            response.message = "Gemini backend did not return pixel_uv."
            response.confidence = float(conf) if conf is not None else 0.0
            return response

        pixel_uv = stub["pixel_uv"]
        top_color = stub.get("top_color")
        top_face_9 = stub.get("top_face_9")
        if isinstance(top_color, str):
            self._latest_top_color = top_color
        if isinstance(top_face_9, str):
            self._latest_top_face_9 = top_face_9
            self._publish_u_face_cache(top_face_9, top_color)

        try:
            depth_out = self._depth_backend.compute_from_pixel(
                pixel_uv=(float(pixel_uv[0]), float(pixel_uv[1])),
                depth=depth,
                K=K,
                T_base_cam=self._T_base_cam,
            )
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = f"depth backend failed: {exc}"
            response.confidence = 0.0
            return response

        p_base = depth_out["base_xyz"]
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

    def _publish_u_face_cache(self, top_face_9: str, top_color: Optional[str]) -> None:
        msg = String()
        msg.data = json.dumps(
            {
                "face": "U",
                "top_face_9": top_face_9,
                "top_color": top_color or "",
            }
        )
        self._u_face_cache_pub.publish(msg)


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
