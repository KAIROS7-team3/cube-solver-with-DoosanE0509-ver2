"""
robot_action_server_node.py
────────────────────────────
명세서(cube_solver_node_spec) 3.1 기준 구현.

시나리오: 명령 → 동작 → 완료 리퀘스트 → 확인 → 다음 명령 패턴 반복
오케스트레이터가 단계별로 액션을 호출하며 각 액션은
완료 Result를 반환하여 다음 단계 진행 여부를 오케스트레이터가 결정한다.

액션 서버 목록 (명세서 3.1):
  - ExecuteSolveToken  : 솔빙 토큰 1개 실행
  - PickupCube         : step 기반 4단계 픽업
  - PlaceOnJig         : step 기반 2단계 안치
  - GoHome             : INITIAL_STATE 복귀
  - RotateCubeForFace  : 면별 단일 동작 스캔
"""

import time

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from cube_interfaces.action import (
    ExecuteSolveToken, PickupCube, PlaceOnJig, GoHome, RotateCubeForFace,
)
from dsr_msgs2.srv import (
    MoveLine, MoveJoint, MoveStop,
    ConfigCreateTcp, SetCurrentTcp, GetCurrentTcp,
)
from dsr_msgs2.srv import DrlStart
from cube_interfaces.action import SafeGrasp

from .motion_library import (
    Step, StepKind,
    PULSE_OPEN, PULSE_CUBE,
    TCP_NAME,
    JOINT_HOME, ZIG_HOME,
    token_sequence, VALID_TOKENS,
    pickup_step_seq, PICKUP_STEPS,
    rotate_face_seq, SCAN_FACES,
    place_step_seq, PLACE_STEPS,
    VEL_X, VEL_R, ACC_X, ACC_R, VEL_J, ACC_J,
)

Z_SAFE_LIMIT: float = 10.0
DR_BASE       = 0
DR_MV_MOD_ABS = 0
DR_MV_MOD_REL = 1

GRIP_GOAL_CURRENT: int     = 500
GRIP_THRESHOLD_CLOSE: int  = 300
GRIP_THRESHOLD_OPEN: int   = 9999
GRIP_TIMEOUT_SEC: float    = 8.0


class RobotActionServerNode(Node):
    """
    팔+그리퍼 합성 동작 액션 서버 노드. (명세서 3.1)
    각 액션은 오케스트레이터의 명령에 의해 단계별로 호출되며
    완료 Result를 반환하여 다음 단계로의 진행을 오케스트레이터가 결정한다.
    """

    def __init__(self):
        super().__init__('robot_action_server_node')
        self.cb_group = ReentrantCallbackGroup()

        self.declare_parameter('robot_ns', '')
        ns = self.get_parameter('robot_ns').get_parameter_value().string_value
        _p = f'/{ns}' if ns else ''

        self.movel_cli   = self.create_client(MoveLine,  f'{_p}/motion/move_line',        callback_group=self.cb_group)
        self.movej_cli   = self.create_client(MoveJoint, f'{_p}/motion/move_joint',       callback_group=self.cb_group)
        self.stop_cli    = self.create_client(MoveStop,  f'{_p}/motion/move_stop',        callback_group=self.cb_group)
        self.cfg_tcp_cli = self.create_client(ConfigCreateTcp, f'{_p}/tcp/config_create_tcp', callback_group=self.cb_group)
        self.set_tcp_cli = self.create_client(SetCurrentTcp,   f'{_p}/tcp/set_current_tcp',   callback_group=self.cb_group)
        self.get_tcp_cli = self.create_client(GetCurrentTcp,   f'{_p}/tcp/get_current_tcp',   callback_group=self.cb_group)
        self.drl_cli     = self.create_client(DrlStart,  f'{_p}/drl/drl_start',           callback_group=self.cb_group)

        self.grip_cli = ActionClient(self, SafeGrasp, '/gripper/safe_grasp', callback_group=self.cb_group)

        self._wait_for_services()

        # 5개 액션 서버 등록 (명세서 3.1)
        ActionServer(self, ExecuteSolveToken,  '/robot/execute_solve_token',
                     execute_callback=self._exec_token_cb,  goal_callback=self._goal_cb, cancel_callback=self._cancel_cb, callback_group=self.cb_group)
        ActionServer(self, PickupCube,         '/robot/pickup_cube',
                     execute_callback=self._pickup_cb,      goal_callback=self._goal_cb, cancel_callback=self._cancel_cb, callback_group=self.cb_group)
        ActionServer(self, PlaceOnJig,         '/robot/place_on_jig',
                     execute_callback=self._place_cb,       goal_callback=self._goal_cb, cancel_callback=self._cancel_cb, callback_group=self.cb_group)
        ActionServer(self, GoHome,             '/robot/go_home',
                     execute_callback=self._home_cb,        goal_callback=self._goal_cb, cancel_callback=self._cancel_cb, callback_group=self.cb_group)
        ActionServer(self, RotateCubeForFace,  '/robot/rotate_cube_for_face',
                     execute_callback=self._rotate_face_cb, goal_callback=self._goal_cb, cancel_callback=self._cancel_cb, callback_group=self.cb_group)

        self._tcp_ready = False
        self._tcp_timer = self.create_timer(1.5, self._init_tcp_sync)

        self.get_logger().info('✅ RobotActionServerNode 초기화 완료')
        self.get_logger().info(f'   유효 토큰: {VALID_TOKENS}')

    def _wait_for_services(self):
        for cli, name in [
            (self.movel_cli,   'move_line'), (self.movej_cli, 'move_joint'),
            (self.stop_cli,    'move_stop'), (self.drl_cli,   'drl_start'),
        ]:
            if not cli.wait_for_service(timeout_sec=15.0):
                self.get_logger().error(f'❌ {name} 없음')
            else:
                self.get_logger().info(f'✅ {name} 연결됨')
        if not self.grip_cli.wait_for_server(timeout_sec=15.0):
            self.get_logger().error('❌ /gripper/safe_grasp 액션 서버 없음')
        else:
            self.get_logger().info('✅ /gripper/safe_grasp 연결됨')

    def _init_tcp_sync(self):
        self._tcp_timer.cancel()
        try:
            if self.drl_cli.service_is_ready():
                req = DrlStart.Request()
                req.robot_system = 0
                req.code = f'set_tcp("{TCP_NAME}")'
                self.drl_cli.call_async(req)
                self.get_logger().info(f'✅ TCP 설정: {TCP_NAME}')
        except Exception as e:
            self.get_logger().warn(f'⚠️ TCP 예외: {e}')
        self._tcp_ready = True
        self.get_logger().info('🟢 초기화 완료 — 동작 명령 수신 가능')

    async def _stop_robot(self):
        if self.stop_cli.service_is_ready():
            req = MoveStop.Request()
            req.stop_mode = 1
            await self.stop_cli.call_async(req)
            self.get_logger().warn('🛑 즉시 정지')

    def _goal_cb(self, goal_request):
        if not self._tcp_ready:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        self.get_logger().warn('🚨 취소 요청')
        return CancelResponse.ACCEPT

    def _feedback(self, gh, stage: str, progress: float = 0.0):
        try:
            fb = gh.action_type.Feedback()
            if hasattr(fb, 'stage'):   fb.stage    = stage
            if hasattr(fb, 'progress'): fb.progress = float(progress)
            gh.publish_feedback(fb)
        except Exception:
            pass

    # ─────────────────────────────────────────────────────
    # _run_sequence: Step 리스트 순회 (명세서 3.1)
    # ─────────────────────────────────────────────────────
    async def _run_sequence(self, gh, steps: list):
        for i, step in enumerate(steps):
            if gh.is_cancel_requested:
                await self._stop_robot()
                gh.canceled()
                return None
            self.get_logger().info(f'  [{i+1}/{len(steps)}] {step.kind.name}')
            ok = await self._exec_step(step)
            if gh.is_cancel_requested:
                await self._stop_robot()
                gh.canceled()
                return None
            if not ok:
                await self._stop_robot()
                gh.abort()
                return False
        return True

    # ─────────────────────────────────────────────────────
    # ExecuteSolveToken 골 콜백 (명세서 3.1 _exec_token_cb)
    # ─────────────────────────────────────────────────────
    async def _exec_token_cb(self, gh):
        token = gh.request.token
        self.get_logger().info(f'▶ ExecuteSolveToken: {token}')
        result = ExecuteSolveToken.Result()
        if token not in VALID_TOKENS:
            result.success = False
            result.message = f'알 수 없는 토큰: {token}'
            gh.abort()
            return result

        self._feedback(gh, 'approach', 0.0)
        steps = [JOINT_HOME()] + token_sequence(token) + [JOINT_HOME()]
        ok = await self._run_sequence(gh, steps)

        if ok is None:
            result.success, result.message = False, f'{token} 취소됨'
        elif not ok:
            result.success, result.message = False, f'{token} 실패'
        else:
            gh.succeed()
            result.success, result.message = True, f'{token} 완료'
            self._feedback(gh, 'retreat', 1.0)
        return result

    # ─────────────────────────────────────────────────────
    # PickupCube 골 콜백 (명세서 3.1 _pickup_cb)
    # step: RELEASE_HOME | DESCEND | GRIP | LIFT
    # DRL 동작 1~6 분배
    # ─────────────────────────────────────────────────────
    async def _pickup_cb(self, gh):
        step = gh.request.step
        self.get_logger().info(f'▶ PickupCube step: {step}')
        result = PickupCube.Result()

        if step not in PICKUP_STEPS:
            result.success = False
            result.message = f'알 수 없는 step: {step}'
            gh.abort()
            return result

        # cube_pose XY 좌표 추출 (m → mm 변환)
        # cube_pose가 (0,0)이면 ZIG Home 기준 동작 (임시 픽업)
        # cube_pose XY 값이 있으면 워크스테이션 랜덤 위치 픽업
        cube_x, cube_y = None, None
        try:
            pose = gh.request.cube_pose.pose
            if pose.position.x != 0.0 or pose.position.y != 0.0:
                cube_x = pose.position.x * 1000.0  # m → mm
                cube_y = pose.position.y * 1000.0  # m → mm
                self.get_logger().info(
                    f'  cube_pose 수신: x={cube_x:.1f}mm y={cube_y:.1f}mm'
                    f' → 워크스테이션 랜덤 위치 픽업')
            else:
                self.get_logger().info(
                    '  cube_pose 없음(0,0) → ZIG Home 기준 임시 픽업')
        except Exception as e:
            self.get_logger().warn(f'  cube_pose 파싱 실패({e}) → ZIG Home 기준')

        self._feedback(gh, step.lower(), 0.0)
        steps = pickup_step_seq(step, cube_x, cube_y)
        ok = await self._run_sequence(gh, steps)

        if ok is None:
            result.success, result.message = False, f'PickupCube {step} 취소됨'
        elif not ok:
            result.success, result.message = False, f'PickupCube {step} 실패'
        else:
            gh.succeed()
            result.success, result.message = True, f'PickupCube {step} 완료'
            self._feedback(gh, step.lower(), 1.0)
        return result

    # ─────────────────────────────────────────────────────
    # PlaceOnJig 골 콜백 (명세서 3.1 _place_cb)
    # step: APPROACH | RELEASE
    # DRL 동작 14~16 분배
    # ─────────────────────────────────────────────────────
    async def _place_cb(self, gh):
        step = gh.request.step
        self.get_logger().info(f'▶ PlaceOnJig step: {step}')
        result = PlaceOnJig.Result()

        if step not in PLACE_STEPS:
            result.success = False
            result.message = f'알 수 없는 step: {step}'
            gh.abort()
            return result

        self._feedback(gh, step.lower(), 0.0)
        steps = place_step_seq(step)
        ok = await self._run_sequence(gh, steps)

        if ok is None:
            result.success, result.message = False, f'PlaceOnJig {step} 취소됨'
        elif not ok:
            result.success, result.message = False, f'PlaceOnJig {step} 실패'
        else:
            gh.succeed()
            result.success, result.message = True, f'PlaceOnJig {step} 완료'
            self._feedback(gh, step.lower(), 1.0)
        return result

    # ─────────────────────────────────────────────────────
    # GoHome 골 콜백 (명세서 3.1 _home_cb)
    # ─────────────────────────────────────────────────────
    async def _home_cb(self, gh):
        self.get_logger().info('▶ GoHome')
        result = GoHome.Result()
        self._feedback(gh, 'moving', 0.0)
        ok = await self._run_sequence(gh, [JOINT_HOME(), ZIG_HOME(), JOINT_HOME()])
        if ok is None:
            result.success, result.message = False, 'GoHome 취소됨'
        elif not ok:
            result.success, result.message = False, 'GoHome 실패'
        else:
            gh.succeed()
            result.success, result.message = True, 'GoHome 완료'
            self._feedback(gh, 'done', 1.0)
        return result

    # ─────────────────────────────────────────────────────
    # RotateCubeForFace 골 콜백 (명세서 3.1 _rotate_face_cb)
    # next_face: B | R | F | L | D | B_AGAIN
    # DRL 동작 7~13 분배
    # ─────────────────────────────────────────────────────
    async def _rotate_face_cb(self, gh):
        next_face = gh.request.next_face
        self.get_logger().info(f'▶ RotateCubeForFace: {next_face}')
        result = RotateCubeForFace.Result()

        if next_face not in SCAN_FACES:
            result.success = False
            gh.abort()
            return result

        self._feedback(gh, 'moving', 0.0)
        steps = rotate_face_seq(next_face)
        ok = await self._run_sequence(gh, steps)

        if ok is None:
            result.success = False
        elif not ok:
            result.success = False
        else:
            gh.succeed()
            result.success = True
            self._feedback(gh, 'done', 1.0)
        return result

    # ─────────────────────────────────────────────────────
    # Step 실행
    # ─────────────────────────────────────────────────────
    async def _exec_step(self, step: Step) -> bool:
        if step.kind == StepKind.MOVE_L_ABS:
            return await self._movel(step, DR_MV_MOD_ABS)
        elif step.kind == StepKind.MOVE_L_REL:
            return await self._movel(step, DR_MV_MOD_REL)
        elif step.kind in (StepKind.MOVE_J_ABS, StepKind.MOVE_J_REL):
            return await self._movej(step)
        elif step.kind == StepKind.GRIP:
            return await self._grip(step)
        elif step.kind == StepKind.WAIT:
            time.sleep(step.sec or 0.5)
            return True
        return False

    async def _movel(self, step: Step, mode: int) -> bool:
        pos = step.pose
        if mode == DR_MV_MOD_ABS and pos[2] < Z_SAFE_LIMIT:
            self.get_logger().error(f'🚨 Z 안전 한계 초과: {pos[2]:.1f}mm')
            return False
        req = MoveLine.Request()
        req.pos        = [float(p) for p in pos]
        req.vel        = [step.vel or VEL_X, step.vel or VEL_R]
        req.acc        = [step.acc or ACC_X, step.acc or ACC_R]
        req.time       = 0.0
        req.radius     = 0.0
        req.ref        = DR_BASE
        req.mode       = mode
        req.blend_type = 0
        req.sync_type  = 0
        res = await self.movel_cli.call_async(req)
        time.sleep(0.3)
        return bool(res and res.success)

    async def _movej(self, step: Step) -> bool:
        req = MoveJoint.Request()
        req.pos        = [float(j) for j in step.pose]
        req.vel        = step.vel or VEL_J
        req.acc        = step.acc or ACC_J
        req.time       = 0.0
        req.radius     = 0.0
        req.mode       = DR_MV_MOD_ABS if step.kind == StepKind.MOVE_J_ABS else DR_MV_MOD_REL
        req.blend_type = 0
        req.sync_type  = 0
        res = await self.movej_cli.call_async(req)
        time.sleep(0.3)
        return bool(res and res.success)

    async def _grip(self, step: Step) -> bool:
        pulse = step.pulse
        is_open = (pulse == PULSE_OPEN)
        goal = SafeGrasp.Goal()
        goal.target_position  = int(pulse)
        goal.goal_current     = GRIP_GOAL_CURRENT
        goal.current_threshold = GRIP_THRESHOLD_OPEN if is_open else GRIP_THRESHOLD_CLOSE
        goal.timeout_sec      = GRIP_TIMEOUT_SEC
        self.get_logger().info(f'  🤖 gripper {"OPEN" if is_open else "CLOSE"} (pulse={pulse})')
        gh = await self.grip_cli.send_goal_async(goal)
        if not gh.accepted:
            self.get_logger().error('  ❌ gripper goal rejected')
            return False
        result_wrap = await gh.get_result_async()
        res = result_wrap.result
        if not res.success:
            self.get_logger().error(f'  ❌ gripper failed: {res.message}')
            return False
        self.get_logger().info(
            f'  ✅ gripper done: pos={res.final_position} cur={res.final_current} grasped={res.grasped}'
        )
        return True


def main(args=None):
    rclpy.init(args=args)
    node = RobotActionServerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('종료')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
