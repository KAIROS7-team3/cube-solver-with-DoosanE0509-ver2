"""
action_client_node.py
──────────────────────
변경사항:
  - 단일 토큰 입력 외에 솔루션 문자열 입력 추가
    예: "F R' U2 B D'" → 순서대로 서버에 전송
  - 각 토큰 실행이 완료된 후 다음 토큰 전송 (순차 실행)
  - Ctrl+C / stop 즉시 정지 유지
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from cube_interfaces.action import CubeMove
from dsr_msgs2.srv import MoveStop
import threading
import sys
import signal


class CubeMoveActionClient(Node):
    def __init__(self):
        super().__init__('cube_move_action_client')

        self._action_client = ActionClient(self, CubeMove, 'cube_move')
        self.stop_cli = self.create_client(MoveStop, '/motion/move_stop')

        self._current_goal_handle: ClientGoalHandle = None
        self._is_running  = False
        self._token_queue = []   # 솔루션 문자열 사용 시 큐

        signal.signal(signal.SIGINT, self._sigint_handler)
        self.get_logger().info('✅ CubeMoveActionClient 초기화 완료')

    # ─────────────────────────────────────────────────────
    # Ctrl+C 핸들러
    # ─────────────────────────────────────────────────────
    def _sigint_handler(self, sig, frame):
        print('\n')
        self.get_logger().warn('🚨 긴급 정지 요청 (S)')
        self._emergency_stop()
        rclpy.shutdown()
        sys.exit(0)

    # ─────────────────────────────────────────────────────
    # 긴급 정지
    # ─────────────────────────────────────────────────────
    def _emergency_stop(self):
        self._token_queue.clear()   # 대기 중인 토큰 전부 취소
        self.get_logger().warn('🛑 로봇 즉시 정지 명령 전송')
        if self.stop_cli.service_is_ready():
            req = MoveStop.Request()
            req.stop_mode = 1
            self.stop_cli.call_async(req)
            self.get_logger().warn('✅ move_stop 전송 완료')
        else:
            self.get_logger().error(
                '❌ /motion/move_stop 없음 — 티치 펜던트 비상 정지 필요!')
        if self._current_goal_handle is not None and self._is_running:
            self.get_logger().warn('🛑 Action Cancel 전송 중...')
            cancel_future = self._current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._cancel_done_cb)
        self._is_running = False

    def _cancel_done_cb(self, future):
        self.get_logger().warn('✅ Action 취소 완료')

    # ─────────────────────────────────────────────────────
    # Goal 전송 (단일 토큰)
    # ─────────────────────────────────────────────────────
    def send_goal(self, move_token: str):
        self.get_logger().info('서버 대기 중...')
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('❌ Action 서버 없음')
            self._prompt_next()
            return

        goal = CubeMove.Goal()
        goal.move_token = move_token
        self.get_logger().info(f'📤 Goal 전송: {move_token}')
        self.get_logger().warn(
            '🚨 긴급 정지: Ctrl+C 또는 "stop" 입력'
        )

        self._is_running = True
        self._send_goal_future = self._action_client.send_goal_async(
            goal, feedback_callback=self.feedback_cb
        )
        self._send_goal_future.add_done_callback(self.goal_response_cb)

    # ─────────────────────────────────────────────────────
    # Goal 수락 콜백
    # ─────────────────────────────────────────────────────
    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('❌ Goal 거부됨')
            self._is_running = False
            self._token_queue.clear()
            self._prompt_next()
            return

        self._current_goal_handle = goal_handle
        self.get_logger().info('✅ Goal 수락됨 — 실행 중...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_cb)

    # ─────────────────────────────────────────────────────
    # 피드백 콜백
    # ─────────────────────────────────────────────────────
    def feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'  📊 [{fb.steps_done}/{fb.steps_total}] {fb.current_step}'
        )

    # ─────────────────────────────────────────────────────
    # 결과 콜백 — 큐에 다음 토큰 있으면 자동 실행
    # ─────────────────────────────────────────────────────
    def result_cb(self, future):
        self._is_running = False
        self._current_goal_handle = None

        from action_msgs.msg import GoalStatus
        status = future.result().status
        result = future.result().result

        if status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('🛑 동작이 취소됐어요')
            self._token_queue.clear()
        elif result.success:
            self.get_logger().info(
                f'🎉 완료: {result.message} ({result.steps_executed}스텝)'
            )
        else:
            self.get_logger().error(f'❌ 실패: {result.message}')
            self._token_queue.clear()

        # 큐에 다음 토큰이 있으면 바로 실행
        if self._token_queue:
            next_token = self._token_queue.pop(0)
            remaining  = len(self._token_queue)
            self.get_logger().info(
                f'▶ 다음 토큰 실행: {next_token} '
                f'(남은 {remaining}개)'
            )
            self.send_goal(next_token)
        else:
            self._prompt_next()

    # ─────────────────────────────────────────────────────
    # 사용자 입력
    # ─────────────────────────────────────────────────────
    def _prompt_next(self):
        print('\n' + '='*45)
        print('단일 동작:  F  F\'  F2  B  B\'  B2')
        print('            U  U\'  U2  D  D\'  D2')
        print('            R  R\'  R2  L  L\'  L2')
        print('            P (Perception)')
        print('솔루션 입력: solution')
        print('  예) solution: F R\' U2 B D\' L2')
        print('긴급 정지:  stop')
        print('종료:       quit')
        print('='*45)

        cmd = input('입력 >> ').strip()

        if cmd == 'quit':
            self.get_logger().info('종료')
            rclpy.shutdown()
        elif cmd == 'stop':
            self._emergency_stop()
            self._prompt_next()
        elif cmd == 'solution':
            sol = input('솔루션 문자열 >> ').strip()
            tokens = sol.split()
            if not tokens:
                self.get_logger().warn('⚠️  빈 솔루션')
                self._prompt_next()
                return
            self.get_logger().info(
                f'📋 솔루션 {len(tokens)}개 토큰: {tokens}'
            )
            # 첫 토큰 실행, 나머지는 큐에 저장
            self._token_queue = tokens[1:]
            self.send_goal(tokens[0])
        else:
            self.send_goal(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = CubeMoveActionClient()

    def input_thread():
        node._prompt_next()

    thread = threading.Thread(target=input_thread, daemon=True)
    thread.start()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()