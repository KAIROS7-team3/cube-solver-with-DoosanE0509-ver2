"""
robot_action_server_node.py — 최종 완성본
변경사항:
  - _init_tcp_timer_cb + async _init_tcp 제거
  - _init_tcp_sync (동기 타이머 콜백) 으로 교체
  - ConfigCreateTcp weight 필드 제거 (srv에 없음)
  - TCP_WEIGHT_KG import 제거
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
    token_sequence, VALID_TOKENS,
    VEL_X, VEL_R, ACC_X, ACC_R, VEL_J, ACC_J,
)

Z_SAFE_LIMIT: float = 10.0
DR_BASE       = 0
DR_MV_MOD_ABS = 0
DR_MV_MOD_REL = 1


class RobotActionServerNode(Node):

    def __init__(self):
        super().__init__('robot_action_server_node')
        self.cb_group = ReentrantCallbackGroup()

        self.movel_cli = self.create_client(
            MoveLine,  '/motion/move_line',
            callback_group=self.cb_group)
        self.movej_cli = self.create_client(
            MoveJoint, '/motion/move_joint',
            callback_group=self.cb_group)
        self.stop_cli = self.create_client(
            MoveStop,  '/motion/move_stop',
            callback_group=self.cb_group)
        self.cfg_tcp_cli = self.create_client(
            ConfigCreateTcp, '/tcp/config_create_tcp',
            callback_group=self.cb_group)
        self.set_tcp_cli = self.create_client(
            SetCurrentTcp,   '/tcp/set_current_tcp',
            callback_group=self.cb_group)
        self.get_tcp_cli = self.create_client(
            GetCurrentTcp,   '/tcp/get_current_tcp',
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

    def _wait_for_services(self):
        for cli, name in [
            (self.movel_cli,   '/motion/move_line'),
            (self.movej_cli,   '/motion/move_joint'),
            (self.stop_cli,    '/motion/move_stop'),
            (self.cfg_tcp_cli, '/tcp/config_create_tcp'),
            (self.set_tcp_cli, '/tcp/set_current_tcp'),
            (self.get_tcp_cli, '/tcp/get_current_tcp'),
        ]:
            if not cli.wait_for_service(timeout_sec=5.0):
                self.get_logger().error(f'❌ {name} 서비스 없음')
            else:
                self.get_logger().info(f'✅ {name} 연결됨')

    def _init_tcp_sync(self):
        """동기 타이머 콜백 — TCP 등록/설정 후 타이머 취소."""
        self._tcp_timer.cancel()

        # 1) TCP 프로파일 등록 (name, pos 필드만 존재)
        if self.cfg_tcp_cli.service_is_ready():
            req = ConfigCreateTcp.Request()
            req.name = TCP_NAME
            req.pos  = [float(v) for v in TCP_POS]
            future = self.cfg_tcp_cli.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            res = future.result()
            if res and res.success:
                self.get_logger().info(
                    f'✅ TCP 프로파일 등록: {TCP_NAME} (Z={TCP_POS[2]}mm)')
            else:
                self.get_logger().warn(
                    '⚠️  TCP 등록 실패 (이미 존재하면 무시)')
        else:
            self.get_logger().error('❌ config_create_tcp 서비스 없음')

        # 2) 현재 TCP 설정
        if self.set_tcp_cli.service_is_ready():
            req = SetCurrentTcp.Request()
            req.name = TCP_NAME
            future = self.set_tcp_cli.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            res = future.result()
            if res and res.success:
                self.get_logger().info(f'✅ 현재 TCP 설정: {TCP_NAME}')
                self._tcp_ready = True
            else:
                self.get_logger().error('❌ TCP 설정 실패')
                return
        else:
            self.get_logger().error('❌ set_current_tcp 서비스 없음')
            return

        # 3) 설정 확인
        if self.get_tcp_cli.service_is_ready():
            future = self.get_tcp_cli.call_async(GetCurrentTcp.Request())
            rclpy.spin_until_future_complete(self, future)
            res = future.result()
            self.get_logger().info(f'📍 현재 TCP 확인: {res.info}')

        self.get_logger().info('🟢 TCP 초기화 완료 — 동작 명령 수신 가능')

    async def _stop_robot(self):
        if self.stop_cli.service_is_ready():
            req = MoveStop.Request()
            req.stop_mode = 1
            await self.stop_cli.call_async(req)
            self.get_logger().warn('🛑 로봇 즉시 정지 완료')
        else:
            self.get_logger().error(
                '❌ move_stop 없음 — 티치 펜던트 비상 정지 필요!')

    def goal_cb(self, goal_request):
        token = goal_request.move_token
        if not self._tcp_ready:
            self.get_logger().error(
                '❌ TCP 초기화 미완료 — Goal 거부. 잠시 후 다시 시도하세요.')
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

        steps = token_sequence(token)
        total = len(steps)
        ok    = await self._run_sequence(goal_handle, steps, feedback, total)

        if ok is None:
            result.success = False
            result.message = f'{token} 취소됨'
            result.steps_executed = feedback.steps_done
            return result
        if not ok:
            result.success = False
            result.message = f'{token} 실패 @ {feedback.current_step}'
            result.steps_executed = feedback.steps_done
            return result

        goal_handle.succeed()
        result.success = True
        result.message = f'{token} 완료'
        result.steps_executed = total
        self.get_logger().info(f'✅ {token} 완료')
        return result

    async def _run_sequence(self, goal_handle, steps, feedback, total):
        for i, step in enumerate(steps):
            if goal_handle.is_cancel_requested:
                self.get_logger().warn('🛑 취소 확인 → 즉시 정지')
                await self._stop_robot()
                goal_handle.canceled()
                return None

            feedback.current_step = step.name
            feedback.steps_done   = i
            feedback.steps_total  = total
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(f'  [{i+1}/{total}] {step.name}')

            coro_task = asyncio.ensure_future(self._exec_step(step))
            while not coro_task.done():
                if goal_handle.is_cancel_requested:
                    self.get_logger().warn(f'🛑 실행 중 취소: {step.name}')
                    coro_task.cancel()
                    await self._stop_robot()
                    goal_handle.canceled()
                    return None
                await asyncio.sleep(0.05)

            step_ok = coro_task.result() if not coro_task.cancelled() else False
            if not step_ok:
                self.get_logger().error(f'❌ Step 실패: {step.name}')
                await self._stop_robot()
                goal_handle.abort()
                return False

        return True

    async def _exec_step(self, step: Step) -> bool:
        if step.kind == StepKind.MOVE_L_ABS:
            return await self._movel(step, mode=DR_MV_MOD_ABS)
        elif step.kind == StepKind.MOVE_L_REL:
            return await self._movel(step, mode=DR_MV_MOD_REL)
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

    async def _movej(self, step: Step) -> bool:
        req = MoveJoint.Request()
        req.pos        = [float(j) for j in step.pose]
        req.vel        = step.vel or VEL_J
        req.acc        = step.acc or ACC_J
        req.time       = 0.0
        req.radius     = 0.0
        req.mode       = DR_MV_MOD_REL
        req.blend_type = 0
        req.sync_type  = 0
        res = await self.movej_cli.call_async(req)
        time.sleep(0.3)
        return bool(res and res.success)

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
