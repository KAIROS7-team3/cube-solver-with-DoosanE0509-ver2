from typing import Dict, Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from PyQt5 import QtCore, QtGui, QtWidgets

from cube_perception.srv import DetectCubePose, ExtractFace


class ServiceTesterNode(Node):
    """GUI에서 쓰는 ROS 통신 전용 노드."""

    def __init__(self, color_image_topic: str) -> None:
        super().__init__("cube_perception_service_tester_gui")
        self._bridge = CvBridge()
        self._latest_color: Optional[np.ndarray] = None
        self._first_api: Optional[np.ndarray] = None
        self._faces: Dict[str, Optional[np.ndarray]] = {"B": None, "R": None, "F": None, "L": None, "D": None}

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, color_image_topic, self._color_cb, qos)
        self.create_subscription(Image, "/cube_perception/debug/first_api_image", self._first_cb, 10)
        self.create_subscription(Image, "/cube_perception/debug/face_B", self._face_cb_factory("B"), 10)
        self.create_subscription(Image, "/cube_perception/debug/face_R", self._face_cb_factory("R"), 10)
        self.create_subscription(Image, "/cube_perception/debug/face_F", self._face_cb_factory("F"), 10)
        self.create_subscription(Image, "/cube_perception/debug/face_L", self._face_cb_factory("L"), 10)
        self.create_subscription(Image, "/cube_perception/debug/face_D", self._face_cb_factory("D"), 10)

        self.detect_client = self.create_client(DetectCubePose, "/detect_cube_pose")
        self.extract_client = self.create_client(ExtractFace, "/color_extraction_node/extract_face")

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
        except Exception:
            pass

    def _first_cb(self, msg: Image) -> None:
        try:
            self._first_api = self._to_bgr(msg)
        except Exception:
            pass

    def _face_cb_factory(self, face: str):
        def _cb(msg: Image) -> None:
            try:
                self._faces[face] = self._to_bgr(msg)
            except Exception:
                pass

        return _cb


class ServiceTesterWindow(QtWidgets.QWidget):
    def __init__(self, node: ServiceTesterNode) -> None:
        super().__init__()
        self.node = node
        self.setWindowTitle("cube_perception Service Tester (PyQt)")
        self.resize(1100, 820)

        root = QtWidgets.QVBoxLayout(self)

        self.live_label = QtWidgets.QLabel("LIVE CAMERA")
        self.live_label.setAlignment(QtCore.Qt.AlignCenter)
        self.live_label.setMinimumSize(960, 540)
        self.live_label.setStyleSheet("background: #111; color: #ddd;")
        root.addWidget(self.live_label)

        thumbs_layout = QtWidgets.QHBoxLayout()
        self.thumb_labels: Dict[str, QtWidgets.QLabel] = {}
        for key in ["1st API", "B", "R", "F", "L", "D"]:
            lbl = QtWidgets.QLabel(f"{key}\nN/A")
            lbl.setAlignment(QtCore.Qt.AlignCenter)
            lbl.setFixedSize(160, 120)
            lbl.setStyleSheet("background: #222; color: #ddd; border: 1px solid #555;")
            thumbs_layout.addWidget(lbl)
            self.thumb_labels[key] = lbl
        root.addLayout(thumbs_layout)

        btn_row = QtWidgets.QHBoxLayout()
        self.btn_detect = QtWidgets.QPushButton("DetectCubePose 호출")
        self.btn_detect.clicked.connect(self.on_detect_clicked)
        btn_row.addWidget(self.btn_detect)

        for face in ["B", "R", "F", "L", "D", "U"]:
            b = QtWidgets.QPushButton(f"ExtractFace {face}")
            b.clicked.connect(lambda _checked=False, f=face: self.on_extract_clicked(f))
            btn_row.addWidget(b)
        root.addLayout(btn_row)

        self.status_label = QtWidgets.QLabel("상태: 준비")
        root.addWidget(self.status_label)

        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(300)
        root.addWidget(self.log, 1)

        self._ui_timer = QtCore.QTimer(self)
        self._ui_timer.timeout.connect(self.refresh_images)
        self._ui_timer.start(100)

        self._spin_timer = QtCore.QTimer(self)
        self._spin_timer.timeout.connect(self.spin_ros_once)
        self._spin_timer.start(20)

    def log_line(self, text: str) -> None:
        self.log.appendPlainText(text)

    def spin_ros_once(self) -> None:
        rclpy.spin_once(self.node, timeout_sec=0.0)

    def _to_qpixmap(self, bgr: np.ndarray, size: QtCore.QSize) -> QtGui.QPixmap:
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        h, w, _ = rgb.shape
        qimg = QtGui.QImage(rgb.data, w, h, 3 * w, QtGui.QImage.Format_RGB888)
        pm = QtGui.QPixmap.fromImage(qimg)
        return pm.scaled(size, QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation)

    def _set_thumb(self, key: str, img: Optional[np.ndarray]) -> None:
        lbl = self.thumb_labels[key]
        if img is None:
            lbl.setText(f"{key}\nN/A")
            lbl.setPixmap(QtGui.QPixmap())
            return
        lbl.setText("")
        lbl.setPixmap(self._to_qpixmap(img, lbl.size()))

    def refresh_images(self) -> None:
        if self.node._latest_color is not None:
            self.live_label.setText("")
            self.live_label.setPixmap(self._to_qpixmap(self.node._latest_color, self.live_label.size()))
        else:
            self.live_label.setText("LIVE CAMERA\n(no frame)")

        self._set_thumb("1st API", self.node._first_api)
        self._set_thumb("B", self.node._faces["B"])
        self._set_thumb("R", self.node._faces["R"])
        self._set_thumb("F", self.node._faces["F"])
        self._set_thumb("L", self.node._faces["L"])
        self._set_thumb("D", self.node._faces["D"])

    def on_detect_clicked(self) -> None:
        if not self.node.detect_client.wait_for_service(timeout_sec=0.2):
            self.status_label.setText("상태: /detect_cube_pose 서비스 없음")
            self.log_line("[Detect] service unavailable")
            return
        req = DetectCubePose.Request()
        req.hint = ""
        future = self.node.detect_client.call_async(req)
        future.add_done_callback(self._on_detect_done)
        self.status_label.setText("상태: DetectCubePose 요청 중...")
        self.log_line("[Detect] request sent")

    def _on_detect_done(self, future) -> None:
        try:
            resp = future.result()
            self.status_label.setText(f"상태: Detect done (success={resp.success})")
            self.log_line(
                f"[Detect] success={resp.success} conf={resp.confidence:.3f} msg={resp.message}"
            )
        except Exception as exc:  # noqa: BLE001
            self.status_label.setText("상태: Detect 실패")
            self.log_line(f"[Detect] exception: {exc}")

    def on_extract_clicked(self, face: str) -> None:
        if not self.node.extract_client.wait_for_service(timeout_sec=0.2):
            self.status_label.setText("상태: /extract_face 서비스 없음")
            self.log_line("[Extract] service unavailable")
            return
        req = ExtractFace.Request()
        req.face = face
        future = self.node.extract_client.call_async(req)
        future.add_done_callback(lambda f: self._on_extract_done(face, f))
        self.status_label.setText(f"상태: ExtractFace({face}) 요청 중...")
        self.log_line(f"[Extract-{face}] request sent")

    def _on_extract_done(self, face: str, future) -> None:
        try:
            resp = future.result()
            self.status_label.setText(f"상태: Extract {face} done (success={resp.success})")
            self.log_line(
                f"[Extract-{face}] success={resp.success} colors_9='{resp.colors_9}' msg={resp.message}"
            )
        except Exception as exc:  # noqa: BLE001
            self.status_label.setText(f"상태: Extract {face} 실패")
            self.log_line(f"[Extract-{face}] exception: {exc}")


def main() -> None:
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--color-image-topic",
        default="/camera/camera/color/image_raw",
        help="Live camera image topic",
    )
    args, unknown = parser.parse_known_args()

    rclpy.init(args=unknown)
    node = ServiceTesterNode(color_image_topic=args.color_image_topic)

    app = QtWidgets.QApplication([])
    win = ServiceTesterWindow(node)
    win.show()
    app.exec_()

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
