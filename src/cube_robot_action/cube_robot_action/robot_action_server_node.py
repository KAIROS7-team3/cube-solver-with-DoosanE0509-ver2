"""
robot_action_server_node.py
────────────────────────────
변경사항:
  - ensure_future 제거 → 직접 await (_run_sequence)
  - 전체 실행 흐름: Home → 동작시퀀스 → Home 복귀
  - 인터럽트는 각 스텝 완료 후 즉시 체크
  - TCP fire-and-forget 방식 (데드락 방지)
"""

import asyncio
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from cube_interfaces.action import CubeMove
from dsr_msgs2.srv import (
    MoveLine, MoveJoint, MoveStop,
    ConfigCreateTcp, SetCurrentTcp, GetCurrentTcp,
)

from .motion_library import (
    Step, StepKind,
    PULSE_OPEN, PULSE_CUBE,
    TCP_NAME, TCP_POS,
    JOINT_HOME,
    token_sequence, VALID_TOKENS,
    VEL_X, VEL_R, ACC_X, ACC_R, VEL_J, ACC_J,
)

Z_SAFE_LIMIT: float = 10.0
DR_BASE       = 0
DR_MV_MOD_ABS = 0
DR_MV_MOD_REL = 1

# ── Home Step은 motion_library의 JOINT_HOME() 사용 ─────────


class RobotActionServerNode(Node):

    def __init__(self):
        super().__init__('robot_action_server_node')
        self.cb_group = ReentrantCallbackGroup()
        # TCP 서비스 클라이언트들 아래에 추가
        from dsr_msgs2.srv import DrlStart
        self.drl_cli = self.create_client(
            DrlStart, '/drl/drl_start',
            callback_group=self.cb_group
        )

        # ── 네임스페이스 파라미터 ─────────────────────────
        # rviz 모드:    --ros-args -p robot_ns:=dsr01
        # moveit 모드:  파라미터 없음 (기본값 '')
        self.declare_parameter('robot_ns', '')
        ns = self.get_parameter('robot_ns').get_parameter_value().string_value
        _p = f'/{ns}' if ns else ''
        self.get_logger().info(f'   서비스 prefix: "{_p}" (빈 문자열=moveit모드)')

        self.movel_cli = self.create_client(
            MoveLine,  f'{_p}/motion/move_line',
            callback_group=self.cb_group)
        self.movej_cli = self.create_client(
            MoveJoint, f'{_p}/motion/move_joint',
            callback_group=self.cb_group)
        self.stop_cli = self.create_client(
            MoveStop,  f'{_p}/motion/move_stop',
            callback_group=self.cb_group)
        self.cfg_tcp_cli = self.create_client(
            ConfigCreateTcp, f'{_p}/tcp/config_create_tcp',
            callback_group=self.cb_group)
        self.set_tcp_cli = self.create_client(
            SetCurrentTcp,   f'{_p}/tcp/set_current_tcp',
            callback_group=self.cb_group)
        self.get_tcp_cli = self.create_client(
            GetCurrentTcp,   f'{_p}/tcp/get_current_tcp',
            callback_group=self.cb_group)

        self._wait_for_services()

        self._action_server = ActionServer(
            self, CubeMove, 'cube_move',
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
            callback_group=self.cb_group)

        self._tcp_ready = False
        self._tcp_timer = self.create_timer(1.5, self._init_tcp_sync)

        self.get_logger().info('✅ RobotActionServerNode 초기화 완료')
        self.get_logger().info(f'   유효 토큰: {VALID_TOKENS}')
        self.get_logger().info('   TCP 초기화 대기 중...')

    # ─────────────────────────────────────────────────────
    # 서비스 연결 대기
    # ─────────────────────────────────────────────────────
    def _wait_for_services(self):
        for cli, name in [
            (self.movel_cli,   '/motion/move_line'),
            (self.movej_cli,   '/motion/move_joint'),
            (self.stop_cli,    '/motion/move_stop'),
            (self.cfg_tcp_cli, '/tcp/config_create_tcp'),
            (self.set_tcp_cli, '/tcp/set_current_tcp'),
            (self.get_tcp_cli, '/tcp/get_current_tcp'),
            (self.drl_cli,     '/drl/drl_start'),
        ]:
            if not cli.wait_for_service(timeout_sec=5.0):
                self.get_logger().error(f'❌ {name} 서비스 없음')
            else:
                self.get_logger().info(f'✅ {name} 연결됨')

    # ─────────────────────────────────────────────────────
    # TCP 초기화 — fire-and-forget (데드락 방지)
    # ─────────────────────────────────────────────────────
    def _init_tcp_sync(self):
        """TCP 설정 — fire-and-forget (데드락 방지)"""
        self._tcp_timer.cancel()
        try:
            if self.drl_cli.service_is_ready():
                from dsr_msgs2.srv import DrlStart
                req = DrlStart.Request()
                req.robot_system = 0
                req.code = 'set_tcp("GripperDA_v1")'
                self.drl_cli.call_async(req)
                self.get_logger().info('✅ TCP 설정 완료: GripperDA_v1 (Z=160mm)')
            else:
                self.get_logger().warn('⚠️  drl_start 서비스 없음 — 스킵')
        except Exception as e:
            self.get_logger().warn(f'⚠️  TCP 초기화 예외 (무시): {e}')

        self._tcp_ready = True
        self.get_logger().info('🟢 초기화 완료 — 동작 명령 수신 가능')

    # ─────────────────────────────────────────────────────
    # 즉시 정지
    # ─────────────────────────────────────────────────────
    async def _stop_robot(self):
        if self.stop_cli.service_is_ready():
            req = MoveStop.Request()
            req.stop_mode = 1
            await self.stop_cli.call_async(req)
            self.get_logger().warn('🛑 로봇 즉시 정지 완료')
        else:
            self.get_logger().error(
                '❌ move_stop 없음 — 티치 펜던트 비상 정지 필요!')

    # ─────────────────────────────────────────────────────
    # Action 콜백
    # ─────────────────────────────────────────────────────
    def goal_cb(self, goal_request):
        token = goal_request.move_token
        if not self._tcp_ready:
            self.get_logger().error('❌ TCP 초기화 미완료 — Goal 거부')
            return GoalResponse.REJECT
        if token not in VALID_TOKENS:
            self.get_logger().warn(f'⚠️  알 수 없는 토큰: {token!r}')
            return GoalResponse.REJECT
        self.get_logger().info(f'📥 Goal 수락: {token}')
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle):
        self.get_logger().warn('🚨 취소 요청 수신')
        return CancelResponse.ACCEPT

    async def execute_cb(self, goal_handle):
        token    = goal_handle.request.move_token
        feedback = CubeMove.Feedback()
        result   = CubeMove.Result()
        self.get_logger().info(f'▶ 실행 시작: {token}')

        # ── 전체 시퀀스: [Home] + 동작 + [Home 복귀] ──────
        token_steps  = token_sequence(token)
        full_steps   = [JOINT_HOME()] + token_steps + [JOINT_HOME()]
        total        = len(full_steps)

        ok = await self._run_sequence(
            goal_handle, full_steps, feedback, total
        )

        if ok is None:
            result.success        = False
            result.message        = f'{token} 취소됨'
            result.steps_executed = feedback.steps_done
            return result
        if not ok:
            result.success        = False
            result.message        = f'{token} 실패 @ {feedback.current_step}'
            result.steps_executed = feedback.steps_done
            return result

        goal_handle.succeed()
        result.success        = True
        result.message        = f'{token} 완료'
        result.steps_executed = total
        self.get_logger().info(f'✅ {token} 완료')
        return result

    # ─────────────────────────────────────────────────────
    # 시퀀스 실행 — 직접 await (ensure_future 제거)
    # 반환값: True=성공 / False=실패 / None=취소
    # ─────────────────────────────────────────────────────
    async def _run_sequence(self, goal_handle, steps, feedback, total):
        for i, step in enumerate(steps):

            # 스텝 전 취소 확인
            if goal_handle.is_cancel_requested:
                self.get_logger().warn('🛑 취소 확인 → 즉시 정지')
                await self._stop_robot()
                goal_handle.canceled()
                return None

            # 피드백 발행
            feedback.current_step = step.name
            feedback.steps_done   = i
            feedback.steps_total  = total
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(f'  [{i+1}/{total}] {step.name}')

            # 직접 await — event loop 문제 없음
            step_ok = await self._exec_step(step)

            # 스텝 완료 후 취소 확인
            if goal_handle.is_cancel_requested:
                self.get_logger().warn(f'🛑 스텝 후 취소: {step.name}')
                await self._stop_robot()
                goal_handle.canceled()
                return None

            if not step_ok:
                self.get_logger().error(f'❌ Step 실패: {step.name}')
                await self._stop_robot()
                goal_handle.abort()
                return False

        return True

    # ─────────────────────────────────────────────────────
    # 단일 Step 실행
    # ─────────────────────────────────────────────────────
    async def _exec_step(self, step: Step) -> bool:
        if step.kind == StepKind.MOVE_L_ABS:
            return await self._movel(step, mode=DR_MV_MOD_ABS)
        elif step.kind == StepKind.MOVE_L_REL:
            return await self._movel(step, mode=DR_MV_MOD_REL)
        elif step.kind == StepKind.MOVE_J_ABS:
            return await self._movej(step)
        elif step.kind == StepKind.MOVE_J_REL:
            return await self._movej(step)
        elif step.kind == StepKind.GRIP:
            return await self._grip(step)
        elif step.kind == StepKind.WAIT:
            time.sleep(step.sec or 0.5)
            return True
        else:
            self.get_logger().error(f'알 수 없는 StepKind: {step.kind}')
            return False

    # ─────────────────────────────────────────────────────
    # movel
    # ─────────────────────────────────────────────────────
    async def _movel(self, step: Step, mode: int) -> bool:
        pos = step.pose
        if mode == DR_MV_MOD_ABS and pos[2] < Z_SAFE_LIMIT:
            self.get_logger().error(
                f'🚨 Z 안전 한계 초과! Z={pos[2]:.1f}mm → 차단: {step.name}')
            return False

        req = MoveLine.Request()
        req.pos        = [float(p) for p in pos]
        req.vel        = [step.vel or VEL_X, step.acc or VEL_R]
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

    # ─────────────────────────────────────────────────────
    # movej — HOME_STEP은 절대값으로 처리
    # ─────────────────────────────────────────────────────
    async def _movej(self, step: Step) -> bool:
        req = MoveJoint.Request()
        req.pos        = [float(j) for j in step.pose]
        req.vel        = step.vel or VEL_J
        req.acc        = step.acc or ACC_J
        req.time       = 0.0
        req.radius     = 0.0
        # MOVE_J_ABS면 절대 이동, MOVE_J_REL이면 상대 이동
        req.mode       = DR_MV_MOD_ABS if step.kind == StepKind.MOVE_J_ABS else DR_MV_MOD_REL
        req.blend_type = 0
        req.sync_type  = 0
        res = await self.movej_cli.call_async(req)
        time.sleep(0.3)
        return bool(res and res.success)

    # ─────────────────────────────────────────────────────
    # 그리퍼
    # ─────────────────────────────────────────────────────
    async def _grip(self, step: Step) -> bool:
        pulse = step.pulse
        label = 'OPEN' if pulse == PULSE_OPEN else 'CLOSE'
        self.get_logger().info(f'  🤖 gripper {label} (pulse={pulse})')
        time.sleep(0.5)
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