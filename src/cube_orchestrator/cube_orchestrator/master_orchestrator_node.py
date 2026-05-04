"""Cube solver orchestrator.

두 가지 분기로 동작:
  [분기 1] start_scan  : DETECT_CUBE → PICKUP×4 → ROTATE×6 + ExtractFace×5 → SOLVE
                         결과(state_54, solution)를 응답으로 반환 + 노드 내부에 캐시
  [분기 2] start_solve : PLACE×2 → EXECUTE_TOKEN×N → GO_HOME
                         req.solution 이 비어 있으면 캐시된 solution 사용

  [호환]   start_run   : 분기 1 → 분기 2 순차 실행 (기존 동작 유지)

ExtractFace returns capture status only; the assembled 54-char state is fetched
once via GetCubeState before solving. Any sub-step failure transitions to FAULT.
"""
from __future__ import annotations

import threading
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from cube_interfaces.srv import StartRun, StartScan, StartSolve

try:
    import kociemba  # type: ignore
except ImportError:  # pragma: no cover
    kociemba = None

from cube_interfaces.srv import DetectCubePose, ExtractFace, GetCubeState
from cube_interfaces.action import (
    ExecuteSolveToken,
    GoHome,
    PickupCube,
    PlaceOnJig,
    RotateCubeForFace,
)


PICKUP_STEPS = ("RELEASE_HOME", "DESCEND", "GRIP", "LIFT")
PLACE_STEPS = ("APPROACH", "RELEASE")
ROTATE_SEQUENCE = ("B", "R", "F", "L", "D", "B_AGAIN")
CAPTURE_FACES = {"B", "R", "F", "L", "D"}

# start_scan / start_solve 서비스 콜백이 블로킹 대기할 최대 시간 (초)
SCAN_CALLBACK_TIMEOUT_SEC  = 600.0
SOLVE_CALLBACK_TIMEOUT_SEC = 600.0


class MasterOrchestratorNode(Node):
    """Top-level FSM node for the cube solver workflow."""

    def __init__(self) -> None:
        super().__init__("master_orchestrator_node")

        # ---- parameters ----
        self.declare_parameter("service_timeout_sec", 10.0)
        self.declare_parameter("action_timeout_sec", 120.0)
        self.declare_parameter("perceive_action_timeout_sec", 30.0)

        self._svc_timeout = float(self.get_parameter("service_timeout_sec").value)
        self._act_timeout = float(self.get_parameter("action_timeout_sec").value)
        self._perceive_act_timeout = float(
            self.get_parameter("perceive_action_timeout_sec").value
        )

        # ---- runtime state ----
        self._cb_group = ReentrantCallbackGroup()
        self._state = "IDLE"
        self._busy_lock = threading.Lock()
        self._busy = False
        self._cancel_event = threading.Event()
        self._cube_pose: Optional[PoseStamped] = None
        self._current_goal_handle = None
        self._goal_handle_lock = threading.Lock()

        # 캐시: 분기 1 결과를 분기 2가 재사용
        self._cached_state_54: str = ""
        self._cached_solution: Optional[list[str]] = None

        # 워커 → 콜백 결과 전달 (scan/solve 모드의 블로킹 대기용)
        self._pending_mode: str = "run"
        self._pending_inject_tokens: Optional[list[str]] = None
        self._start_event = threading.Event()
        self._result_event = threading.Event()
        self._result_ok: bool = False
        self._result_state_54: str = ""
        self._result_solution_str: str = ""

        # ---- publishers ----
        self._state_pub = self.create_publisher(String, "/orchestrator/state", 10)
        self._solution_pub = self.create_publisher(String, "/orchestrator/solution", 10)
        self._current_token_pub = self.create_publisher(
            String, "/orchestrator/current_token", 10
        )
        self._face_colors_pub = self.create_publisher(
            String, "/orchestrator/face_colors", 10
        )
        self._last_detection_pub = self.create_publisher(
            PoseStamped, "/orchestrator/last_detection", 10
        )
        self._fault_pub = self.create_publisher(String, "/orchestrator/fault", 10)

        # ---- services ----
        self._start_run_srv = self.create_service(
            StartRun, "/orchestrator/start_run",
            self._start_run_cb, callback_group=self._cb_group,
        )
        self._start_scan_srv = self.create_service(
            StartScan, "/orchestrator/start_scan",
            self._start_scan_cb, callback_group=self._cb_group,
        )
        self._start_solve_srv = self.create_service(
            StartSolve, "/orchestrator/start_solve",
            self._start_solve_cb, callback_group=self._cb_group,
        )
        self._cancel_srv = self.create_service(
            Trigger, "/orchestrator/cancel",
            self._cancel_cb, callback_group=self._cb_group,
        )

        # ---- service clients (cube_perception) ----
        self._detect_cli = self.create_client(
            DetectCubePose, "/cube_perception/detect_cube_pose",
            callback_group=self._cb_group,
        )
        self._extract_cli = self.create_client(
            ExtractFace, "/cube_perception/extract_face",
            callback_group=self._cb_group,
        )
        self._get_state_cli = self.create_client(
            GetCubeState, "/cube_perception/get_cube_state",
            callback_group=self._cb_group,
        )

        # ---- action clients (cube_robot_action) ----
        self._pickup_act = ActionClient(
            self, PickupCube, "/robot/pickup_cube", callback_group=self._cb_group
        )
        self._place_act = ActionClient(
            self, PlaceOnJig, "/robot/place_on_jig", callback_group=self._cb_group
        )
        self._rotate_act = ActionClient(
            self, RotateCubeForFace, "/robot/rotate_cube_for_face",
            callback_group=self._cb_group,
        )
        self._token_act = ActionClient(
            self, ExecuteSolveToken, "/robot/execute_solve_token",
            callback_group=self._cb_group,
        )
        self._home_act = ActionClient(
            self, GoHome, "/robot/go_home", callback_group=self._cb_group
        )

        # ---- worker thread ----
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

        self._pub_state("IDLE")
        self.get_logger().info("master_orchestrator_node ready")

    # ------------------------------------------------------------------
    # Service callbacks
    # ------------------------------------------------------------------
    def _start_run_cb(
        self, req: StartRun.Request, res: StartRun.Response
    ) -> StartRun.Response:
        """기존 동작 유지: 분기 1 + 분기 2를 순차 실행. fire-and-forget."""
        with self._busy_lock:
            if self._busy:
                res.success = False
                res.message = "orchestrator busy"
                return res
            self._busy = True
        self._cancel_event.clear()
        self._pending_mode = "run"
        self._start_event.set()
        res.success = True
        res.message = "started"
        self.get_logger().info("start_run: started")
        return res

    def _start_scan_cb(
        self, req: StartScan.Request, res: StartScan.Response
    ) -> StartScan.Response:
        """분기 1 실행 후 블로킹 대기. 응답에 state_54 + solution 포함."""
        with self._busy_lock:
            if self._busy:
                res.success = False
                res.message = "orchestrator busy"
                return res
            self._busy = True
        self._cancel_event.clear()
        self._pending_mode = "scan"
        self._result_event.clear()
        self._start_event.set()

        if not self._result_event.wait(timeout=SCAN_CALLBACK_TIMEOUT_SEC):
            res.success = False
            res.message = "scan timeout"
            return res

        res.success = self._result_ok
        res.message = "scan complete" if self._result_ok else "scan failed"
        res.state_54 = self._result_state_54
        res.solution = self._result_solution_str
        return res

    def _start_solve_cb(
        self, req: StartSolve.Request, res: StartSolve.Response
    ) -> StartSolve.Response:
        """분기 2 실행 후 블로킹 대기. req.solution이 비어 있으면 캐시 사용."""
        tokens: Optional[list[str]] = None
        if req.solution.strip():
            tokens = [t for t in req.solution.strip().split() if t]
        elif self._cached_solution is not None:
            tokens = self._cached_solution
        else:
            res.success = False
            res.message = "no solution: provide solution or call start_scan first"
            return res

        with self._busy_lock:
            if self._busy:
                res.success = False
                res.message = "orchestrator busy"
                return res
            self._busy = True
        self._cancel_event.clear()
        self._pending_mode = "solve"
        self._pending_inject_tokens = tokens
        self._result_event.clear()
        self._start_event.set()

        if not self._result_event.wait(timeout=SOLVE_CALLBACK_TIMEOUT_SEC):
            res.success = False
            res.message = "solve timeout"
            return res

        res.success = self._result_ok
        res.message = "solve complete" if self._result_ok else "solve failed"
        return res

    def _cancel_cb(self, req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        if not self._busy:
            res.success = False
            res.message = "no active run"
            return res
        self._cancel_event.set()
        with self._goal_handle_lock:
            handle = self._current_goal_handle
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception as exc:  # pragma: no cover
                self.get_logger().warn(f"cancel_goal_async raised: {exc}")
        self.get_logger().warn("cancel requested")
        res.success = True
        res.message = "cancel requested"
        return res

    # ------------------------------------------------------------------
    # Worker loop
    # ------------------------------------------------------------------
    def _worker_loop(self) -> None:
        while rclpy.ok():
            if not self._start_event.wait(timeout=0.5):
                continue
            self._start_event.clear()
            mode = self._pending_mode
            try:
                if mode == "scan":
                    self._worker_scan()
                elif mode == "solve":
                    self._worker_solve()
                else:
                    self._worker_run()
            except Exception as exc:  # pragma: no cover
                self.get_logger().error(f"unexpected FSM exception: {exc}")
                self._handle_fault(f"exception: {exc}")
                self._result_event.set()  # unblock waiting callback
            finally:
                with self._busy_lock:
                    self._busy = False
                self._cube_pose = None
                self._pending_inject_tokens = None

    def _worker_scan(self) -> None:
        """분기 1: 스캔 + 솔버. 결과를 캐시하고 result_event로 콜백에 전달."""
        self._result_ok = False
        self._result_state_54 = ""
        self._result_solution_str = ""
        result = self._run_scan()
        if result is not None:
            state_54, solution_str, tokens = result
            self._cached_state_54 = state_54
            self._cached_solution = tokens
            self._result_ok = True
            self._result_state_54 = state_54
            self._result_solution_str = solution_str
        self._result_event.set()

    def _worker_solve(self) -> None:
        """분기 2: 풀이 실행. 결과를 result_event로 콜백에 전달."""
        tokens = self._pending_inject_tokens
        self._result_ok = False
        if tokens is None:
            self._handle_fault("no solution tokens in worker_solve")
        else:
            self._result_ok = self._run_solve_execute(tokens)
        self._result_event.set()

    def _worker_run(self) -> None:
        """기존 start_run 동작: 분기 1 + 분기 2 순차 실행."""
        result = self._run_scan()
        if result is None:
            return
        _, _, tokens = result
        self._cached_solution = tokens
        self._run_solve_execute(tokens)

    # ------------------------------------------------------------------
    # FSM: 분기 1 — scan
    # ------------------------------------------------------------------
    def _run_scan(self) -> Optional[tuple[str, str, list[str]]]:
        """DETECT → PICKUP → ROTATE+EXTRACT → SOLVE.
        Returns (state_54, solution_str, tokens) or None on failure."""
        self._transition("DETECT_CUBE")
        if not self._run_detect():
            self._handle_fault("DetectCubePose failed")
            return None
        if self._cancel_event.is_set():
            self._handle_fault("cancelled")
            return None

        self._transition("PICKUP")
        if not self._run_pickup():
            self._handle_fault("PickupCube failed")
            return None
        if self._cancel_event.is_set():
            self._handle_fault("cancelled")
            return None

        for step in ROTATE_SEQUENCE:
            self._transition(f"ROTATE({step})")
            if not self._run_rotate(step):
                self._handle_fault(f"rotate {step} failed")
                return None
            if self._cancel_event.is_set():
                self._handle_fault("cancelled")
                return None
            if step not in CAPTURE_FACES:
                continue
            self._transition(f"PERCEIVE_FACE({step})")
            if not self._run_extract(step):
                self._handle_fault(f"perceive {step} failed")
                return None
            if self._cancel_event.is_set():
                self._handle_fault("cancelled")
                return None

        self._transition("SOLVE")
        tokens, state_54 = self._run_solve()
        if tokens is None:
            self._handle_fault("kociemba.solve failed")
            return None
        solution_str = " ".join(tokens)
        self._publish_string(self._solution_pub, solution_str)
        self.get_logger().info("scan branch complete")
        return (state_54, solution_str, tokens)

    # ------------------------------------------------------------------
    # FSM: 분기 2 — solve execute
    # ------------------------------------------------------------------
    def _run_solve_execute(self, tokens: list[str]) -> bool:
        """PLACE → TOKEN×N → HOME. Returns True on success."""
        self._transition("PLACE_ON_JIG")
        if not self._run_place():
            self._handle_fault("PlaceOnJig failed")
            return False
        if self._cancel_event.is_set():
            self._handle_fault("cancelled")
            return False

        for i, token in enumerate(tokens):
            self._publish_string(
                self._current_token_pub, f"{i + 1}/{len(tokens)}:{token}"
            )
            self._transition(f"EXECUTE_TOKEN({i + 1}/{len(tokens)}:{token})")
            if not self._run_token(token):
                self._handle_fault(f"token {token} failed")
                return False
            if self._cancel_event.is_set():
                self._handle_fault("cancelled")
                return False

        self._publish_string(self._current_token_pub, "")
        self._transition("GO_HOME")
        if not self._run_home():
            self._handle_fault("GoHome failed")
            return False

        self._transition("DONE")
        self._transition("IDLE")
        return True

    # ------------------------------------------------------------------
    # FSM step implementations
    # ------------------------------------------------------------------
    def _run_detect(self) -> bool:
        req = DetectCubePose.Request()
        req.hint = "workspace"
        res = self._call_service(self._detect_cli, req, self._svc_timeout)
        if res is None or not res.success:
            msg = getattr(res, "message", "no response") if res else "timeout"
            self.get_logger().error(f"DetectCubePose failed: {msg}")
            return False
        self._cube_pose = res.pose
        try:
            self._last_detection_pub.publish(res.pose)
        except Exception:
            pass
        self.get_logger().info(
            f"cube detected (confidence={res.confidence:.2f}) at "
            f"{res.pose.pose.position.x:.3f}, {res.pose.pose.position.y:.3f}, "
            f"{res.pose.pose.position.z:.3f}"
        )
        return True

    def _run_pickup(self) -> bool:
        if self._cube_pose is None:
            self.get_logger().error("no cube pose available for pickup")
            return False
        for step in PICKUP_STEPS:
            self._transition(f"PICKUP({step})")
            goal = PickupCube.Goal()
            goal.step = step
            goal.cube_pose = self._cube_pose
            if not self._call_action(
                self._pickup_act, goal, self._act_timeout,
                label=f"PickupCube({step})",
            ):
                return False
            if self._cancel_event.is_set():
                return False
        return True

    def _run_place(self) -> bool:
        for step in PLACE_STEPS:
            self._transition(f"PLACE_ON_JIG({step})")
            goal = PlaceOnJig.Goal()
            goal.step = step
            if not self._call_action(
                self._place_act, goal, self._act_timeout,
                label=f"PlaceOnJig({step})",
            ):
                return False
            if self._cancel_event.is_set():
                return False
        return True

    def _run_rotate(self, next_face: str) -> bool:
        goal = RotateCubeForFace.Goal()
        goal.next_face = next_face
        return self._call_action(
            self._rotate_act, goal, self._perceive_act_timeout,
            label=f"RotateCubeForFace({next_face})",
        )

    def _run_extract(self, face: str) -> bool:
        req = ExtractFace.Request()
        req.face = face
        res = self._call_service(self._extract_cli, req, self._svc_timeout)
        if res is None or not res.success:
            msg = getattr(res, "message", "no response") if res else "timeout"
            self.get_logger().error(f"ExtractFace({face}) failed: {msg}")
            return False
        self._publish_string(self._face_colors_pub, f"{face}:captured")
        self.get_logger().info(f"face {face} captured")
        return True

    def _run_solve(self) -> tuple[Optional[list[str]], str]:
        """GetCubeState → kociemba. Returns (tokens, state_54); tokens=None on failure."""
        res = self._call_service(
            self._get_state_cli, GetCubeState.Request(), self._svc_timeout
        )
        if res is None or not res.success:
            msg = getattr(res, "message", "no response") if res else "timeout"
            self.get_logger().error(f"GetCubeState failed: {msg}")
            return None, ""
        urfdlb = res.state_54
        if len(urfdlb) != 54:
            self.get_logger().error(
                f"GetCubeState bad length {len(urfdlb)}: {urfdlb!r}"
            )
            return None, ""
        self.get_logger().info(f"state_raw: {urfdlb}")

        if kociemba is None:
            self.get_logger().error("kociemba module not available")
            return None, urfdlb
        try:
            solution = str(kociemba.solve(urfdlb)).strip()
        except Exception as exc:
            self.get_logger().error(f"kociemba.solve failed: {exc}")
            return None, urfdlb
        tokens = [tok for tok in solution.split() if tok]
        self.get_logger().info(f"solution ({len(tokens)} moves): {solution}")
        return tokens, urfdlb

    def _run_token(self, token: str) -> bool:
        goal = ExecuteSolveToken.Goal()
        goal.token = token
        return self._call_action(
            self._token_act, goal, self._act_timeout,
            label=f"ExecuteSolveToken({token})",
            feedback_cb=self._on_token_feedback,
        )

    def _run_home(self) -> bool:
        return self._call_action(
            self._home_act, GoHome.Goal(), self._act_timeout, label="GoHome"
        )

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _call_service(self, client, request, timeout_sec: float):
        if not client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error(f"service unavailable: {client.srv_name}")
            return None
        future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout=timeout_sec):
            self.get_logger().error(
                f"service timeout ({timeout_sec}s): {client.srv_name}"
            )
            return None
        return future.result()

    def _call_action(
        self,
        action_client: ActionClient,
        goal,
        timeout_sec: float,
        label: str,
        feedback_cb=None,
    ) -> bool:
        if not action_client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().error(f"action server unavailable: {label}")
            return False

        send_future = action_client.send_goal_async(goal, feedback_callback=feedback_cb)
        sent = threading.Event()
        send_future.add_done_callback(lambda _f: sent.set())
        if not sent.wait(timeout=timeout_sec):
            self.get_logger().error(f"{label} send_goal timeout")
            return False
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f"{label} goal rejected")
            return False

        with self._goal_handle_lock:
            self._current_goal_handle = goal_handle
        try:
            result_future = goal_handle.get_result_async()
            finished = threading.Event()
            result_future.add_done_callback(lambda _f: finished.set())
            if not finished.wait(timeout=timeout_sec):
                self.get_logger().error(f"{label} result timeout, cancelling")
                goal_handle.cancel_goal_async()
                return False
        finally:
            with self._goal_handle_lock:
                self._current_goal_handle = None

        wrapper = result_future.result()
        if wrapper is None:
            self.get_logger().error(f"{label} returned no result")
            return False
        result = wrapper.result
        success = bool(getattr(result, "success", False))
        if not success:
            msg = getattr(result, "message", "")
            self.get_logger().error(f"{label} failed: {msg}")
        return success

    def _on_token_feedback(self, feedback_msg) -> None:
        fb = feedback_msg.feedback
        self.get_logger().debug(
            f"token feedback stage={fb.stage} progress={fb.progress:.2f}"
        )

    # ------------------------------------------------------------------
    # State publishing
    # ------------------------------------------------------------------
    def _transition(self, new_state: str) -> None:
        self._state = new_state
        self._pub_state(new_state)

    def _pub_state(self, state: str) -> None:
        self._publish_string(self._state_pub, state)
        self.get_logger().info(f"[state] {state}")

    def _publish_string(self, publisher, text: str) -> None:
        msg = String()
        msg.data = text
        publisher.publish(msg)

    def _handle_fault(self, reason: str) -> None:
        self.get_logger().error(f"FAULT: {reason}")
        self._publish_string(self._fault_pub, reason)
        self._transition("FAULT")
        self._transition("IDLE")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MasterOrchestratorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
