"""ExtractFace/GetCubeState 서비스 노드.

외부 계약:
- `ExtractFace(face)` 요청은 B/R/F/L/D 캡처 ACK 용도로 사용한다.
- `GetCubeState()` 요청으로 최종 54자 상태를 반환한다.

내부 역할:
- U면은 `vla_detection_node`의 `/cube_perception/u_face_cache`를 캐시한다.
- B/R/F/L/D 캡처만 수행한다. Gemini 2차는 `GetCubeState` 호출 시에만 실행한다.
- 최종 상태는 U 캐시 + 5면 추정 결과를 결합해 생성한다.
"""

import json
from typing import Dict, List, Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .detection.gemini_vla_backend import GeminiVLADetector

try:
    from cube_interfaces.srv import ExtractFace
    from cube_interfaces.srv import GetCubeState
except ImportError:
    ExtractFace = None
    GetCubeState = None


class ColorExtractionNode(Node):
    """`ExtractFace`(캡처) + `GetCubeState`(상태 반환) 게이트웨이.

    - ExtractFace: B/R/F/L/D 캡처 ACK 전용 (Gemini 2차는 호출하지 않음).
    - GetCubeState: U 캐시 + Gemini 2차 추정 결과를 합쳐 54자를 반환.
    """

    VALID_FACES = {"U", "D", "R", "L", "F", "B"}
    CAPTURE_FACE_ORDER = ["B", "R", "F", "L", "D"]
    STATE_54_ORDER = ["U", "R", "F", "D", "L", "B"]
    FACE_REMAP_FOR_STATE = {
        # row-major(1..9) -> 요구 면 순서
        "U": [6, 3, 0, 7, 4, 1, 8, 5, 2],  # 7,4,1 / 8,5,2 / 9,6,3
        "D": [6, 3, 0, 7, 4, 1, 8, 5, 2],  # 7,4,1 / 8,5,2 / 9,6,3
        "B": [2, 5, 8, 1, 4, 7, 0, 3, 6],  # 3,6,9 / 2,5,8 / 1,4,7
        "R": [2, 5, 8, 1, 4, 7, 0, 3, 6],  # 3,6,9 / 2,5,8 / 1,4,7
        "F": [2, 5, 8, 1, 4, 7, 0, 3, 6],  # 3,6,9 / 2,5,8 / 1,4,7
        "L": [2, 5, 8, 1, 4, 7, 0, 3, 6],  # 3,6,9 / 2,5,8 / 1,4,7
    }

    def __init__(self) -> None:
        super().__init__("color_extraction_node")
        self._bridge = CvBridge()
        self._latest_bgr: Optional[np.ndarray] = None
        self._latest_faces_row_major: Optional[Dict[str, str]] = None
        self._latest_state_54: Optional[str] = None
        self._latest_u_face_9: Optional[str] = None
        self._captured_face_images_rgb: Dict[str, np.ndarray] = {}
        self._state_dirty = True

        self._state_detector = GeminiVLADetector(min_confidence=0.0, depth_unit_scale=1.0)
        self.declare_parameter("state_retry_count", 2)

        self.declare_parameter("color_image_topic", "camera/color/image_raw")
        color_topic = self.get_parameter("color_image_topic").get_parameter_value().string_value
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, color_topic, self._color_cb, qos)
        self.create_subscription(String, "/cube_perception/u_face_cache", self._u_face_cache_cb, 10)
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
        self._get_state_service = None
        if GetCubeState is None:
            self.get_logger().warning(
                "GetCubeState 미생성 — colcon build 후 다시 실행하세요."
            )
        else:
            self._get_state_service = self.create_service(
                GetCubeState, "~/get_cube_state", self._get_cube_state_cb
            )
        self._state_raw_pub = self.create_publisher(String, "/cube/state_raw", 10)
        self.get_logger().info("color_extraction_node started")

    def _color_cb(self, msg: Image) -> None:
        try:
            self._latest_bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning("color_cb failed: %s", exc)

    def _u_face_cache_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            top_face_9 = "".join(
                c for c in str(data.get("top_face_9", "")).upper() if not c.isspace()
            )
            if len(top_face_9) == 9 and all(c in {"W", "R", "G", "Y", "O", "B"} for c in top_face_9):
                self._latest_u_face_9 = top_face_9
                self.get_logger().info("cached U face from first detection")
                if self._latest_faces_row_major is not None and self.VALID_FACES <= set(
                    self._latest_faces_row_major
                ):
                    self._latest_faces_row_major["U"] = top_face_9
                    faces_state = self._build_state_faces(self._latest_faces_row_major)
                    self._latest_state_54 = self._compose_state_54(faces_state)
                    self._publish_state_raw(self._latest_state_54)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning("u_face_cache parse failed: %s", exc)

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
        return all(face in self._captured_face_images_rgb for face in self.CAPTURE_FACE_ORDER)

    def _ordered_state_images(self) -> List[np.ndarray]:
        if not self._state_inputs_ready():
            missing = [f for f in self.CAPTURE_FACE_ORDER if f not in self._captured_face_images_rgb]
            raise RuntimeError(f"missing faces: {missing}")
        return [self._captured_face_images_rgb[f] for f in self.CAPTURE_FACE_ORDER]

    def _compose_state_54(self, faces: Dict[str, str]) -> str:
        return "".join(str(faces[k]) for k in self.STATE_54_ORDER)

    def _remap_face_for_state(self, face: str, colors_9: str) -> str:
        if len(colors_9) != 9:
            raise ValueError(f"{face} face length must be 9: {colors_9}")
        remap = self.FACE_REMAP_FOR_STATE[face]
        # 현장 피드백: 리매핑 결과의 9글자 순서를 뒤집어야 기대값과 일치.
        return "".join(colors_9[idx] for idx in remap)[::-1]

    def _build_state_faces(self, faces_row_major: Dict[str, str]) -> Dict[str, str]:
        return {
            face: self._remap_face_for_state(face, faces_row_major[face])
            for face in self.VALID_FACES
        }

    def _run_state_estimation(self) -> None:
        retries = max(int(self.get_parameter("state_retry_count").get_parameter_value().integer_value), 0)
        last_exc: Optional[Exception] = None

        for _ in range(retries + 1):
            try:
                images = self._ordered_state_images()
                out = self._state_detector.detect_state(images)
                faces = out["faces"]
                for face in self.VALID_FACES:
                    val = str(faces[face])
                    if len(val) != 9:
                        raise ValueError(f"faces.{face} length invalid: {val}")
                if self._latest_u_face_9 is not None:
                    faces["U"] = self._latest_u_face_9
                faces_rm: Dict[str, str] = {}
                for face in self.VALID_FACES:
                    s = "".join(ch for ch in str(faces[face]) if not ch.isspace())
                    if len(s) != 9:
                        raise ValueError(f"faces.{face} length invalid after norm: {repr(s)}")
                    faces_rm[face] = s
                faces_state = self._build_state_faces(faces_rm)
                state_54 = self._compose_state_54(faces_state)
                if len(state_54) != 54:
                    raise ValueError(f"state_54 length invalid: {len(state_54)}")
                self._latest_faces_row_major = dict(faces_rm)
                self._latest_state_54 = state_54
                self._state_dirty = False
                self._publish_state_raw(state_54)
                return
            except Exception as exc:  # noqa: BLE001
                last_exc = exc

        raise RuntimeError(f"state estimation failed after retries: {last_exc}") from last_exc

    def _publish_state_raw(self, state_54: str) -> None:
        msg = String()
        msg.data = state_54
        self._state_raw_pub.publish(msg)

    def _extract_cb(self, request, response):
        face = request.face.strip().upper()
        if face not in self.CAPTURE_FACE_ORDER:
            response.success = False
            response.colors_9 = ""
            response.message = (
                "ExtractFace supports capture-only faces: "
                f"{self.CAPTURE_FACE_ORDER}. "
                "U is provided by DetectCubePose cache and GetCubeState."
            )
            return response

        try:
            self._capture_face_image(face)
            captured_count = len(
                [f for f in self.CAPTURE_FACE_ORDER if f in self._captured_face_images_rgb]
            )

            if self._state_inputs_ready():
                response.message = (
                    f"captured {face} ({captured_count}/5), all_captured — call GetCubeState for state_54"
                )
            else:
                response.message = f"captured {face} ({captured_count}/5)"
            response.colors_9 = ""
            response.success = True
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.colors_9 = ""
            response.message = f"extract failed: {exc}"
        return response

    def _get_cube_state_cb(self, _request, response):
        try:
            if self._latest_u_face_9 is None:
                raise RuntimeError("U cache is empty. Call DetectCubePose first.")

            if not self._state_inputs_ready():
                missing = [f for f in self.CAPTURE_FACE_ORDER if f not in self._captured_face_images_rgb]
                raise RuntimeError(f"Need captures for {self.CAPTURE_FACE_ORDER}. missing={missing}")

            if self._latest_faces_row_major is None or self._state_dirty or self._latest_state_54 is None:
                self._run_state_estimation()

            if self._latest_faces_row_major is None or self._latest_state_54 is None:
                raise RuntimeError("state cache is unavailable")

            response.state_54 = self._latest_state_54
            response.faces_json = json.dumps(self._latest_faces_row_major, sort_keys=True)
            response.success = True
            response.message = "ok"
        except Exception as exc:  # noqa: BLE001
            response.state_54 = ""
            response.faces_json = ""
            response.success = False
            response.message = f"get_cube_state failed: {exc}"
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
