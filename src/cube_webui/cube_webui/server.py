"""FastAPI + rclpy single-process Web UI for the cube solver.

Subscribes to orchestrator topics + a camera image topic, exposes:
  GET  /                       - index.html
  GET  /static/*               - static assets
  GET  /api/snapshot           - current full state (JSON)
  WS   /ws/events              - live state pushes (JSON)
  POST /api/start_scan         - {} -> /orchestrator/start_scan  (blocks until done)
  POST /api/start_solve        - {"solution": "..."} -> /orchestrator/start_solve
  POST /api/start_run          - {} -> /orchestrator/start_run  (fire-and-forget)
  POST /api/cancel             - {} -> /orchestrator/cancel
  POST /api/emergency_stop     - {} -> /orchestrator/emergency_stop  (즉시 모션 정지)
  POST /api/token              - {"token": "U|R'|F2|..."} -> ExecuteSolveToken
  GET  /stream/camera.mjpg     - multipart MJPEG live stream
"""
from __future__ import annotations

import asyncio
import json
import threading
import time
from collections import deque
from pathlib import Path
from typing import Any, Optional

import cv2  # type: ignore
import rclpy
import uvicorn
from cv_bridge import CvBridge
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

from cube_interfaces.srv import StartRun, StartScan, StartSolve

try:
    from cube_interfaces.action import ExecuteSolveToken
    _HAS_EXECUTE_TOKEN = True
except ImportError:
    ExecuteSolveToken = None  # type: ignore
    _HAS_EXECUTE_TOKEN = False


STATIC_DIR = Path(__file__).parent / "static"


class WebUIBridge(Node):
    """ROS-side state holder + service/action proxy, spun on a worker thread."""

    def __init__(
        self,
        camera_topic: str,
        loop: asyncio.AbstractEventLoop,
        broadcast_coro,
        fault_history_size: int = 50,
    ) -> None:
        super().__init__("cube_webui_bridge")
        self._loop = loop
        self._broadcast = broadcast_coro
        self._cb_group = ReentrantCallbackGroup()
        self._bridge = CvBridge()

        # mutable state
        self._lock = threading.Lock()
        self._latest_state: str = "UNKNOWN"
        self._latest_solution: str = ""
        self._latest_current_token: str = ""
        self._face_colors: dict[str, str] = {}
        self._latest_detection: Optional[dict[str, Any]] = None
        self._latest_fault: str = ""
        self._fault_history: deque[dict[str, Any]] = deque(maxlen=fault_history_size)
        self._latest_jpeg: Optional[bytes] = None
        self._latest_jpeg_lock = threading.Lock()

        # subscribers
        self.create_subscription(
            String, "/orchestrator/state", self._on_state, 10,
            callback_group=self._cb_group,
        )
        self.create_subscription(
            String, "/orchestrator/solution", self._on_solution, 10,
            callback_group=self._cb_group,
        )
        self.create_subscription(
            String, "/orchestrator/current_token", self._on_current_token, 10,
            callback_group=self._cb_group,
        )
        self.create_subscription(
            String, "/orchestrator/face_colors", self._on_face_colors, 10,
            callback_group=self._cb_group,
        )
        self.create_subscription(
            PoseStamped, "/orchestrator/last_detection", self._on_detection, 10,
            callback_group=self._cb_group,
        )
        self.create_subscription(
            String, "/orchestrator/fault", self._on_fault, 10,
            callback_group=self._cb_group,
        )
        self.create_subscription(
            Image, camera_topic, self._on_image, 1,
            callback_group=self._cb_group,
        )

        # service clients
        self._scan_cli = self.create_client(
            StartScan, "/orchestrator/start_scan", callback_group=self._cb_group
        )
        self._solve_cli = self.create_client(
            StartSolve, "/orchestrator/start_solve", callback_group=self._cb_group
        )
        self._run_cli = self.create_client(
            StartRun, "/orchestrator/start_run", callback_group=self._cb_group
        )
        self._cancel_cli = self.create_client(
            Trigger, "/orchestrator/cancel", callback_group=self._cb_group
        )
        self._estop_cli = self.create_client(
            Trigger, "/orchestrator/emergency_stop", callback_group=self._cb_group
        )
        self._token_act = (
            ActionClient(
                self,
                ExecuteSolveToken,
                "/robot/execute_solve_token",
                callback_group=self._cb_group,
            )
            if _HAS_EXECUTE_TOKEN
            else None
        )

    # ---- subscribers ----
    def _on_state(self, msg: String) -> None:
        with self._lock:
            self._latest_state = msg.data
        self._push({"type": "state", "data": msg.data})

    def _on_solution(self, msg: String) -> None:
        with self._lock:
            self._latest_solution = msg.data
        tokens = [t for t in msg.data.split() if t]
        self._push({"type": "solution", "data": tokens})

    def _on_current_token(self, msg: String) -> None:
        with self._lock:
            self._latest_current_token = msg.data
        self._push({"type": "current_token", "data": msg.data})

    def _on_face_colors(self, msg: String) -> None:
        if ":" not in msg.data:
            return
        face, colors = msg.data.split(":", 1)
        with self._lock:
            self._face_colors[face] = colors
        self._push({"type": "face_colors", "face": face, "colors": colors})

    def _on_detection(self, msg: PoseStamped) -> None:
        info = {
            "frame_id": msg.header.frame_id,
            "x": float(msg.pose.position.x),
            "y": float(msg.pose.position.y),
            "z": float(msg.pose.position.z),
            "qx": float(msg.pose.orientation.x),
            "qy": float(msg.pose.orientation.y),
            "qz": float(msg.pose.orientation.z),
            "qw": float(msg.pose.orientation.w),
            "stamp_sec": int(msg.header.stamp.sec),
        }
        with self._lock:
            self._latest_detection = info
        self._push({"type": "last_detection", "data": info})

    def _on_fault(self, msg: String) -> None:
        entry = {"reason": msg.data, "ts": time.time()}
        with self._lock:
            self._latest_fault = msg.data
            self._fault_history.append(entry)
        self._push({"type": "fault", "data": entry})

    def _on_image(self, msg: Image) -> None:
        try:
            cv_img = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            return
        ok, buf = cv2.imencode(".jpg", cv_img, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
        if not ok:
            return
        with self._latest_jpeg_lock:
            self._latest_jpeg = buf.tobytes()

    # ---- broadcast ----
    def _push(self, payload: dict[str, Any]) -> None:
        try:
            asyncio.run_coroutine_threadsafe(self._broadcast(payload), self._loop)
        except Exception:
            pass

    # ---- snapshot ----
    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return {
                "state": self._latest_state,
                "solution": [t for t in self._latest_solution.split() if t],
                "current_token": self._latest_current_token,
                "face_colors": dict(self._face_colors),
                "last_detection": self._latest_detection,
                "fault": self._latest_fault,
                "fault_history": list(self._fault_history),
            }

    def latest_jpeg(self) -> Optional[bytes]:
        with self._latest_jpeg_lock:
            return self._latest_jpeg

    # ---- proxies ----
    def call_scan(self, timeout: float = 600.0) -> dict[str, Any]:
        if not self._scan_cli.wait_for_service(timeout_sec=5.0):
            return {"success": False, "message": "start_scan service unavailable"}
        future = self._scan_cli.call_async(StartScan.Request())
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout=timeout):
            return {"success": False, "message": "start_scan timeout"}
        res = future.result()
        return {
            "success": bool(res.success),
            "message": res.message,
            "state_54": res.state_54,
            "solution": res.solution,
        }

    def call_solve(self, solution: str = "", timeout: float = 600.0) -> dict[str, Any]:
        if not self._solve_cli.wait_for_service(timeout_sec=5.0):
            return {"success": False, "message": "start_solve service unavailable"}
        req = StartSolve.Request()
        req.solution = solution
        future = self._solve_cli.call_async(req)
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout=timeout):
            return {"success": False, "message": "start_solve timeout"}
        res = future.result()
        return {"success": bool(res.success), "message": res.message}

    def call_run(self, timeout: float = 5.0) -> dict[str, Any]:
        if not self._run_cli.wait_for_service(timeout_sec=timeout):
            return {"success": False, "message": "start_run service unavailable"}
        future = self._run_cli.call_async(StartRun.Request())
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout=timeout):
            return {"success": False, "message": "start_run timeout"}
        res = future.result()
        return {"success": bool(res.success), "message": res.message}

    def call_cancel(self, timeout: float = 5.0) -> dict[str, Any]:
        if not self._cancel_cli.wait_for_service(timeout_sec=timeout):
            return {"success": False, "message": "cancel service unavailable"}
        future = self._cancel_cli.call_async(Trigger.Request())
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout=timeout):
            return {"success": False, "message": "cancel timeout"}
        res = future.result()
        return {"success": bool(res.success), "message": res.message}

    def call_emergency_stop(self, timeout: float = 3.0) -> dict[str, Any]:
        if not self._estop_cli.wait_for_service(timeout_sec=timeout):
            return {"success": False, "message": "emergency_stop service unavailable"}
        future = self._estop_cli.call_async(Trigger.Request())
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout=timeout):
            return {"success": False, "message": "emergency_stop timeout"}
        res = future.result()
        return {"success": bool(res.success), "message": res.message}

    def call_token(self, token: str, timeout: float = 60.0) -> dict[str, Any]:
        if self._token_act is None:
            return {"success": False, "message": "ExecuteSolveToken action not available"}
        if not self._token_act.wait_for_server(timeout_sec=5.0):
            return {"success": False, "message": "execute_solve_token unavailable"}
        goal = ExecuteSolveToken.Goal()
        goal.token = token
        send_future = self._token_act.send_goal_async(goal)
        sent = threading.Event()
        send_future.add_done_callback(lambda _f: sent.set())
        if not sent.wait(timeout=5.0):
            return {"success": False, "message": "send_goal timeout"}
        gh = send_future.result()
        if gh is None or not gh.accepted:
            return {"success": False, "message": "goal rejected"}
        result_future = gh.get_result_async()
        finished = threading.Event()
        result_future.add_done_callback(lambda _f: finished.set())
        if not finished.wait(timeout=timeout):
            gh.cancel_goal_async()
            return {"success": False, "message": "result timeout, cancelled"}
        wrapper = result_future.result()
        if wrapper is None:
            return {"success": False, "message": "no result"}
        result = wrapper.result
        return {
            "success": bool(getattr(result, "success", False)),
            "message": getattr(result, "message", ""),
        }


# ----------------------------------------------------------------------
# FastAPI app + state plumbing
# ----------------------------------------------------------------------
class WebSocketHub:
    def __init__(self) -> None:
        self._clients: set[WebSocket] = set()
        self._lock = asyncio.Lock()

    async def add(self, ws: WebSocket) -> None:
        async with self._lock:
            self._clients.add(ws)

    async def remove(self, ws: WebSocket) -> None:
        async with self._lock:
            self._clients.discard(ws)

    async def broadcast(self, payload: dict[str, Any]) -> None:
        if not self._clients:
            return
        text = json.dumps(payload)
        async with self._lock:
            dead: list[WebSocket] = []
            for ws in self._clients:
                try:
                    await ws.send_text(text)
                except Exception:
                    dead.append(ws)
            for ws in dead:
                self._clients.discard(ws)


def build_app(bridge: WebUIBridge, hub: WebSocketHub) -> FastAPI:
    app = FastAPI(title="cube_webui")

    if STATIC_DIR.exists():
        app.mount("/static", StaticFiles(directory=str(STATIC_DIR)), name="static")

    @app.get("/")
    def index():
        idx = STATIC_DIR / "index.html"
        if not idx.exists():
            return JSONResponse({"error": "index.html missing"}, status_code=500)
        return FileResponse(str(idx))

    @app.get("/api/snapshot")
    def snapshot():
        return JSONResponse(bridge.snapshot())

    @app.post("/api/start_scan")
    async def start_scan():
        loop = asyncio.get_event_loop()
        result = await loop.run_in_executor(None, bridge.call_scan)
        return JSONResponse(result)

    @app.post("/api/start_solve")
    async def start_solve(payload: dict[str, Any] | None = None):
        solution = str((payload or {}).get("solution", "")).strip()
        loop = asyncio.get_event_loop()
        result = await loop.run_in_executor(None, bridge.call_solve, solution)
        return JSONResponse(result)

    @app.post("/api/start_run")
    async def start_run():
        loop = asyncio.get_event_loop()
        result = await loop.run_in_executor(None, bridge.call_run)
        return JSONResponse(result)

    @app.post("/api/cancel")
    async def cancel():
        loop = asyncio.get_event_loop()
        result = await loop.run_in_executor(None, bridge.call_cancel)
        return JSONResponse(result)

    @app.post("/api/emergency_stop")
    async def emergency_stop():
        loop = asyncio.get_event_loop()
        result = await loop.run_in_executor(None, bridge.call_emergency_stop)
        return JSONResponse(result)

    @app.post("/api/token")
    async def token(payload: dict[str, Any]):
        tok = str((payload or {}).get("token", "")).strip()
        if not tok:
            return JSONResponse(
                {"success": False, "message": "empty token"}, status_code=400
            )
        loop = asyncio.get_event_loop()
        result = await loop.run_in_executor(None, bridge.call_token, tok)
        return JSONResponse(result)

    @app.websocket("/ws/events")
    async def ws_events(ws: WebSocket):
        await ws.accept()
        await hub.add(ws)
        try:
            await ws.send_text(
                json.dumps({"type": "snapshot", "data": bridge.snapshot()})
            )
            while True:
                await ws.receive_text()
        except WebSocketDisconnect:
            pass
        except Exception:
            pass
        finally:
            await hub.remove(ws)

    @app.get("/stream/camera.mjpg")
    def camera_stream():
        boundary = b"--frame"

        async def gen():
            while True:
                frame = bridge.latest_jpeg()
                if frame is None:
                    await asyncio.sleep(0.1)
                    continue
                chunk = (
                    boundary
                    + b"\r\nContent-Type: image/jpeg\r\nContent-Length: "
                    + str(len(frame)).encode()
                    + b"\r\n\r\n"
                    + frame
                    + b"\r\n"
                )
                yield chunk
                await asyncio.sleep(0.1)

        return StreamingResponse(
            gen(), media_type="multipart/x-mixed-replace; boundary=frame"
        )

    return app


# ----------------------------------------------------------------------
# Entry point
# ----------------------------------------------------------------------
def _spin_ros(executor: MultiThreadedExecutor) -> None:
    try:
        executor.spin()
    except Exception:
        pass


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    hub = WebSocketHub()

    bridge_node = WebUIBridge(
        camera_topic="/camera/camera/color/image_raw",
        loop=loop,
        broadcast_coro=hub.broadcast,
    )

    bridge_node.declare_parameter("camera_topic", "/camera/camera/color/image_raw")
    bridge_node.declare_parameter("host", "0.0.0.0")
    bridge_node.declare_parameter("port", 8080)
    host = str(bridge_node.get_parameter("host").value)
    port = int(bridge_node.get_parameter("port").value)

    executor = MultiThreadedExecutor()
    executor.add_node(bridge_node)
    ros_thread = threading.Thread(target=_spin_ros, args=(executor,), daemon=True)
    ros_thread.start()

    app = build_app(bridge_node, hub)
    config = uvicorn.Config(
        app, host=host, port=port, loop="asyncio", log_level="info"
    )
    server = uvicorn.Server(config)
    try:
        loop.run_until_complete(server.serve())
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        bridge_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
