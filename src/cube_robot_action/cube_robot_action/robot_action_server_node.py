import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup

from cube_interfaces.action import CubeMove
from dsr_msgs2.srv import MoveLine, MoveJoint

import time

# ═══════════════════════════════════════════════
# 공통 상수 (cube_motion_preamble.drl 기준)
# ═══════════════════════════════════════════════
VEL_X = 10.0   # 직선 이동 속도 (mm/sec) — movel 의 선속도
VEL_R = 10.0   # 회전 이동 속도 (deg/sec) — movel 의 각속도
ACC_X = 5.0   # 직선 이동 가속도 (mm/sec²)
ACC_R = 5.0   # 회전 이동 가속도 (deg/sec²)
VEL_J = 10.0   # joint 이동 속도 (deg/sec) — movej 전용
ACC_J = 5.0   # joint 이동 가속도 (deg/sec²) — movej 전용

TIME_ML = 0.0  # movel 동작 완료까지 강제 시간 (sec)
               # 0.0 이면 vel/acc 기준으로 자동 계산
TIME_MJ = 0.0  # movej 동작 완료까지 강제 시간 (sec)
               # 0.0 이면 vel/acc 기준으로 자동 계산

# 그리퍼 TCP 오프셋 (link_6 → 그리퍼 끝단, mm 단위)
# 실측 후 아래 값 교체 필요
TCP_Z_OFFSET = 160.0  # ← 실측값으로 교체

GRIP_RELEASE_STROKE = 100
GRIP_GRAB_STROKE    = 420

DR_BASE        = 0
DR_MV_MOD_ABS  = 0
DR_MV_MOD_REL  = 1

POS_INITIAL = [373.030, 0.000, 74.660 + TCP_Z_OFFSET, 22.85, -180.0, 22.85]

# 워크스테이션 큐브 파지 Z 고정값 (큐브인식모션테스트.tw 실측값)
# 워크스테이션 바닥이 평평하므로 XY 위치와 무관하게 항상 동일한 Z에서 파지
WORKSTATION_GRIP_Z = 13.71 + TCP_Z_OFFSET  # mm (TCP 오프셋 반영)
POS_L_DROP      = [373.030,  0.000, 48.530, 90.0,    90.0,  90.0 ]
POS_R_DROP      = [373.030,  0.000, 48.530, 90.0,   -90.0,  90.0 ]
POS_READY_D_LOW = [373.010,  0.000, 20.000, 22.85, -180.0,  22.85]
POS_READY_D_UP  = [373.020,  0.000, 27.730,  6.68, -180.0,   6.68]


class RobotActionServerNode(Node):
    def __init__(self):
        super().__init__('robot_action_server_node')
        self.cb_group = ReentrantCallbackGroup()

        # ── Doosan 서비스 클라이언트 ──
        self.movel_cli = self.create_client(
            MoveLine, '/motion/move_line',
            callback_group=self.cb_group
        )
        self.movej_cli = self.create_client(
            MoveJoint, '/motion/move_joint',
            callback_group=self.cb_group
        )

        # 서비스 대기
        self._wait_for_services()

        # ── CubeMove Action 서버 ──
        self._action_server = ActionServer(
            self,
            CubeMove,
            'cube_move',
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
            callback_group=self.cb_group
        )

        # ── 동작 시퀀스 맵 ──
        self.sequence_map = {
            'F'  : self.seq_F,
            "F'" : self.seq_F_prime,
            'F2' : self.seq_F2,
            'R'  : self.seq_R,
            "R'" : self.seq_R_prime,
            'R2' : self.seq_R2,
            'U'  : self.seq_U,
            "U'" : self.seq_U_prime,
            'U2' : self.seq_U2,
            'D'  : self.seq_D,
            "D'" : self.seq_D_prime,
            'D2' : self.seq_D2,
            'L'  : self.seq_L,
            "L'" : self.seq_L_prime,
            'L2' : self.seq_L2,
            'B'  : self.seq_B,
            "B'" : self.seq_B_prime,
            'B2' : self.seq_B2,
            'P'  : self.seq_Perception,
        }

        self.get_logger().info('✅ RobotActionServerNode 초기화 완료')

    # ─────────────────────────────────────────
    # 서비스 대기
    # ─────────────────────────────────────────
    def _wait_for_services(self):
        for cli, name in [
            (self.movel_cli, '/motion/move_line'),
            (self.movej_cli, '/motion/move_joint'),
        ]:
            timeout = 5.0
            if not cli.wait_for_service(timeout_sec=timeout):
                self.get_logger().error(f'❌ {name} 서비스 없음 — 로봇 연결 확인 필요')
            else:
                self.get_logger().info(f'✅ {name} 연결됨')

    # ─────────────────────────────────────────
    # Action 콜백
    # ─────────────────────────────────────────
    def goal_cb(self, goal_request):
        token = goal_request.move_token
        if token not in self.sequence_map:
            self.get_logger().warn(f'⚠️ 알 수 없는 동작: {token}')
            return GoalResponse.REJECT
        self.get_logger().info(f'📥 Goal 수락: {token}')
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle):
        self.get_logger().info('🛑 취소 요청 수신')
        return CancelResponse.ACCEPT

    async def execute_cb(self, goal_handle):
        token = goal_handle.request.move_token
        self.get_logger().info(f'▶ 실행 시작: {token}')

        feedback = CubeMove.Feedback()
        result   = CubeMove.Result()

        seq_func = self.sequence_map[token]
        steps    = seq_func()  # 시퀀스 리스트 반환
        total    = len(steps)

        for i, (step_name, coro) in enumerate(steps):
            # 취소 확인
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = '취소됨'
                return result

            # 피드백 발행
            feedback.current_step = step_name
            feedback.steps_done   = i
            feedback.steps_total  = total
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(f'  [{i+1}/{total}] {step_name}')

            # 실제 실행
            ok = await coro
            if not ok:
                goal_handle.abort()
                result.success        = False
                result.message        = f'{step_name} 실패'
                result.steps_executed = i
                return result

        goal_handle.succeed()
        result.success        = True
        result.message        = f'{token} 완료'
        result.steps_executed = total
        self.get_logger().info(f'✅ {token} 완료')
        return result

    # ─────────────────────────────────────────
    # 기본 이동 함수
    # ─────────────────────────────────────────

    async def movel_abs(self, pos):
        # Z축 안전 체크
        # 안전 Z축 하한선 (mm) — 이 값 이하로 내려가면 차단
        Z_SAFE_LIMIT = 10.0        
        if pos[2] < Z_SAFE_LIMIT:
            self.get_logger().error(
                f'🚨 안전 한계 초과! Z={pos[2]}mm '
                f'(최소 {Z_SAFE_LIMIT}mm) — 동작 차단'
            )
            return False
        req = MoveLine.Request()
        req.pos        = [float(p) for p in pos]
        req.vel        = [VEL_X, VEL_R]
        req.acc        = [ACC_X, ACC_R]
        req.time       = TIME_ML
        req.radius     = 0.0
        req.ref        = DR_BASE
        req.mode       = DR_MV_MOD_ABS
        req.blend_type = 0
        req.sync_type  = 0
        res = await self.movel_cli.call_async(req)
        time.sleep(0.3)
        return res.success if res else False

    async def movel_rel(self, pos):
        req = MoveLine.Request()
        req.pos        = [float(p) for p in pos]
        req.vel        = [VEL_X, VEL_R]
        req.acc        = [ACC_X, ACC_R]
        req.time       = TIME_ML
        req.radius     = 0.0
        req.ref        = DR_BASE
        req.mode       = DR_MV_MOD_REL
        req.blend_type = 0
        req.sync_type  = 0
        res = await self.movel_cli.call_async(req)
        time.sleep(0.3)
        return res.success if res else False

    async def movej_rel(self, joints):
        req = MoveJoint.Request()
        req.pos        = [float(j) for j in joints]
        req.vel        = VEL_J
        req.acc        = ACC_J
        req.time       = TIME_MJ
        req.radius     = 0.0
        req.mode       = DR_MV_MOD_REL
        req.blend_type = 0
        req.sync_type  = 0
        res = await self.movej_cli.call_async(req)
        time.sleep(0.3)
        return res.success if res else False

    async def gripper_release(self):
        # 가상 모드: 실제 serial 없으므로 로그만
        self.get_logger().info('  🤖 gripper_release (가상)')
        time.sleep(0.5)
        return True

    async def gripper_grap(self):
        self.get_logger().info('  🤖 gripper_grap (가상)')
        time.sleep(0.5)
        return True

    # ═══════════════════════════════════════════════
    # 19개 시퀀스 정의
    # 각 함수는 (step_name, coroutine) 리스트를 반환
    # ═══════════════════════════════════════════════

    def seq_U(self):
        return [
            ('gripper_release',          self.gripper_release()),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
            ('movel_rel Z-40',           self.movel_rel([0,0,-40,0,0,0])),
            ('gripper_grap 1',           self.gripper_grap()),
            ('gripper_grap 2',           self.gripper_grap()),
            ('movej_rel J6+90',          self.movej_rel([0,0,0,0,0,90])),
            ('gripper_release 1',        self.gripper_release()),
            ('gripper_release 2',        self.gripper_release()),
            ('movel_rel Z+40',           self.movel_rel([0,0,40,0,0,0])),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
        ]

    def seq_U_prime(self):
        return [
            ('gripper_release',          self.gripper_release()),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
            ('movel_rel Z-40',           self.movel_rel([0,0,-40,0,0,0])),
            ('gripper_grap 1',           self.gripper_grap()),
            ('gripper_grap 2',           self.gripper_grap()),
            ('movej_rel J6-90',          self.movej_rel([0,0,0,0,0,-90])),
            ('gripper_release 1',        self.gripper_release()),
            ('gripper_release 2',        self.gripper_release()),
            ('movel_rel Z+40',           self.movel_rel([0,0,40,0,0,0])),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
        ]

    def seq_U2(self):
        return [
            ('gripper_release',          self.gripper_release()),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
            ('movel_rel Z-40',           self.movel_rel([0,0,-40,0,0,0])),
            ('gripper_grap 1',           self.gripper_grap()),
            ('gripper_grap 2',           self.gripper_grap()),
            ('movej_rel J6+90 (1)',      self.movej_rel([0,0,0,0,0,90])),
            ('movej_rel J6+90 (2)',      self.movej_rel([0,0,0,0,0,90])),
            ('gripper_release 1',        self.gripper_release()),
            ('gripper_release 2',        self.gripper_release()),
            ('movel_rel Z+40',           self.movel_rel([0,0,40,0,0,0])),
            ('movej_rel J6-90 (1)',      self.movej_rel([0,0,0,0,0,-90])),
            ('movej_rel J6-90 (2)',      self.movej_rel([0,0,0,0,0,-90])),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
        ]

    def seq_D(self):
        return [
            ('gripper_release',          self.gripper_release()),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
            ('movel_abs POS_READY_D_LOW',self.movel_abs(POS_READY_D_LOW)),
            ('gripper_grap 1',           self.gripper_grap()),
            ('gripper_grap 2',           self.gripper_grap()),
            ('movel_abs POS_READY_D_UP', self.movel_abs(POS_READY_D_UP)),
            ('movej_rel J6+90',          self.movej_rel([0,0,0,0,0,90])),
            ('movel_rel Z+40',           self.movel_rel([0,0,40,0,0,0])),
            ('movej_rel J6-90',          self.movej_rel([0,0,0,0,0,-90])),
            ('movel_rel Z-40',           self.movel_rel([0,0,-40,0,0,0])),
            ('gripper_release',          self.gripper_release()),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
        ]

    def seq_D_prime(self):
        return [
            ('gripper_release',          self.gripper_release()),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
            ('movel_abs POS_READY_D_LOW',self.movel_abs(POS_READY_D_LOW)),
            ('gripper_grap 1',           self.gripper_grap()),
            ('gripper_grap 2',           self.gripper_grap()),
            ('movel_abs POS_READY_D_UP', self.movel_abs(POS_READY_D_UP)),
            ('movej_rel J6-90',          self.movej_rel([0,0,0,0,0,-90])),
            ('movel_rel Z+40',           self.movel_rel([0,0,40,0,0,0])),
            ('movej_rel J6+90',          self.movej_rel([0,0,0,0,0,90])),
            ('movel_rel Z-40',           self.movel_rel([0,0,-40,0,0,0])),
            ('gripper_release',          self.gripper_release()),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
        ]

    def seq_D2(self):
        return [
            ('gripper_release',          self.gripper_release()),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
            ('movel_abs POS_READY_D_LOW',self.movel_abs(POS_READY_D_LOW)),
            ('gripper_grap 1',           self.gripper_grap()),
            ('gripper_grap 2',           self.gripper_grap()),
            ('movel_abs POS_READY_D_UP', self.movel_abs(POS_READY_D_UP)),
            ('movej_rel J6+90 (1)',      self.movej_rel([0,0,0,0,0,90])),
            ('movej_rel J6+90 (2)',      self.movej_rel([0,0,0,0,0,90])),
            ('movel_rel Z+40',           self.movel_rel([0,0,40,0,0,0])),
            ('movej_rel J6-90 (1)',      self.movej_rel([0,0,0,0,0,-90])),
            ('movej_rel J6-90 (2)',      self.movej_rel([0,0,0,0,0,-90])),
            ('movel_rel Z-40',           self.movel_rel([0,0,-40,0,0,0])),
            ('gripper_release',          self.gripper_release()),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
        ]

    def seq_R(self):
        return [
            ('gripper_release',          self.gripper_release()),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
            ('movel_rel Z-40',           self.movel_rel([0,0,-40,0,0,0])),
            ('gripper_grap 1',           self.gripper_grap()),
            ('gripper_grap 2',           self.gripper_grap()),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
            ('movel_abs POS_R_DROP',     self.movel_abs(POS_R_DROP)),
            ('gripper_release 1',        self.gripper_release()),
            ('gripper_release 2',        self.gripper_release()),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
            ('movel_rel Z-40',           self.movel_rel([0,0,-40,0,0,0])),
            ('gripper_grap 1',           self.gripper_grap()),
            ('gripper_grap 2',           self.gripper_grap()),
            ('movej_rel J6+90',          self.movej_rel([0,0,0,0,0,90])),
            ('gripper_release 1',        self.gripper_release()),
            ('gripper_release 2',        self.gripper_release()),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
            ('movel_rel Z-40',           self.movel_rel([0,0,-40,0,0,0])),
            ('gripper_grap 1',           self.gripper_grap()),
            ('gripper_grap 2',           self.gripper_grap()),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
            ('movel_abs POS_L_DROP',     self.movel_abs(POS_L_DROP)),
            ('gripper_release',          self.gripper_release()),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
        ]

    def seq_R_prime(self):
        return [
            ('gripper_release',          self.gripper_release()),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
            ('movel_rel Z-40',           self.movel_rel([0,0,-40,0,0,0])),
            ('gripper_grap 1',           self.gripper_grap()),
            ('gripper_grap 2',           self.gripper_grap()),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
            ('movel_abs POS_R_DROP',     self.movel_abs(POS_R_DROP)),
            ('gripper_release 1',        self.gripper_release()),
            ('gripper_release 2',        self.gripper_release()),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
            ('movel_rel Z-40',           self.movel_rel([0,0,-40,0,0,0])),
            ('gripper_grap 1',           self.gripper_grap()),
            ('gripper_grap 2',           self.gripper_grap()),
            ('movej_rel J6-90',          self.movej_rel([0,0,0,0,0,-90])),
            ('gripper_release 1',        self.gripper_release()),
            ('gripper_release 2',        self.gripper_release()),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
            ('movel_rel Z-40',           self.movel_rel([0,0,-40,0,0,0])),
            ('gripper_grap 1',           self.gripper_grap()),
            ('gripper_grap 2',           self.gripper_grap()),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
            ('movel_abs POS_L_DROP',     self.movel_abs(POS_L_DROP)),
            ('gripper_release',          self.gripper_release()),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
        ]

    def seq_R2(self):
        return [
            ('gripper_release',          self.gripper_release()),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
            ('movel_rel Z-40',           self.movel_rel([0,0,-40,0,0,0])),
            ('gripper_grap 1',           self.gripper_grap()),
            ('gripper_grap 2',           self.gripper_grap()),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
            ('movel_abs POS_R_DROP',     self.movel_abs(POS_R_DROP)),
            ('gripper_release 1',        self.gripper_release()),
            ('gripper_release 2',        self.gripper_release()),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
            ('movel_rel Z-40',           self.movel_rel([0,0,-40,0,0,0])),
            ('gripper_grap 1',           self.gripper_grap()),
            ('gripper_grap 2',           self.gripper_grap()),
            ('movej_rel J6+90 (1)',      self.movej_rel([0,0,0,0,0,90])),
            ('movej_rel J6+90 (2)',      self.movej_rel([0,0,0,0,0,90])),
            ('gripper_release 1',        self.gripper_release()),
            ('gripper_release 2',        self.gripper_release()),
            ('movel_rel Z+40',           self.movel_rel([0,0,40,0,0,0])),
            ('movej_rel J6-90 (1)',      self.movej_rel([0,0,0,0,0,-90])),
            ('movej_rel J6-90 (2)',      self.movej_rel([0,0,0,0,0,-90])),
            ('movel_rel Z-40',           self.movel_rel([0,0,-40,0,0,0])),
            ('gripper_grap 1',           self.gripper_grap()),
            ('gripper_grap 2',           self.gripper_grap()),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
            ('movel_abs POS_L_DROP',     self.movel_abs(POS_L_DROP)),
            ('gripper_release',          self.gripper_release()),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
        ]

    def seq_L(self):
        return [
            ('gripper_release',          self.gripper_release()),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
            ('movel_rel Z-40',           self.movel_rel([0,0,-40,0,0,0])),
            ('gripper_grap 1',           self.gripper_grap()),
            ('gripper_grap 2',           self.gripper_grap()),
            ('movel_rel Z+40',           self.movel_rel([0,0,40,0,0,0])),
            ('movel_abs POS_L_DROP',     self.movel_abs(POS_L_DROP)),
            ('gripper_release 1',        self.gripper_release()),
            ('gripper_release 2',        self.gripper_release()),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
            ('movel_rel Z-40',           self.movel_rel([0,0,-40,0,0,0])),
            ('gripper_grap 1',           self.gripper_grap()),
            ('gripper_grap 2',           self.gripper_grap()),
            ('movej_rel J6+90',          self.movej_rel([0,0,0,0,0,90])),
            ('gripper_release 1',        self.gripper_release()),
            ('gripper_release 2',        self.gripper_release()),
            ('movel_rel Z+40',           self.movel_rel([0,0,40,0,0,0])),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
            ('movel_rel Z-40',           self.movel_rel([0,0,-40,0,0,0])),
            ('gripper_grap 1',           self.gripper_grap()),
            ('gripper_grap 2',           self.gripper_grap()),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
            ('movel_abs POS_R_DROP',     self.movel_abs(POS_R_DROP)),
            ('gripper_release',          self.gripper_release()),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
        ]

    def seq_L_prime(self):
        return [
            ('gripper_release',          self.gripper_release()),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
            ('movel_rel Z-40',           self.movel_rel([0,0,-40,0,0,0])),
            ('gripper_grap 1',           self.gripper_grap()),
            ('gripper_grap 2',           self.gripper_grap()),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
            ('movel_abs POS_L_DROP',     self.movel_abs(POS_L_DROP)),
            ('gripper_release 1',        self.gripper_release()),
            ('gripper_release 2',        self.gripper_release()),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
            ('movel_rel Z-40',           self.movel_rel([0,0,-40,0,0,0])),
            ('gripper_grap 1',           self.gripper_grap()),
            ('gripper_grap 2',           self.gripper_grap()),
            ('movej_rel J6-90',          self.movej_rel([0,0,0,0,0,-90])),
            ('gripper_release 1',        self.gripper_release()),
            ('gripper_release 2',        self.gripper_release()),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
            ('movel_rel Z-40',           self.movel_rel([0,0,-40,0,0,0])),
            ('gripper_grap 1',           self.gripper_grap()),
            ('gripper_grap 2',           self.gripper_grap()),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
            ('movel_abs POS_R_DROP',     self.movel_abs(POS_R_DROP)),
            ('gripper_release',          self.gripper_release()),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
        ]

    def seq_L2(self):
        return [
            ('gripper_release',          self.gripper_release()),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
            ('movel_rel Z-40',           self.movel_rel([0,0,-40,0,0,0])),
            ('gripper_grap 1',           self.gripper_grap()),
            ('gripper_grap 2',           self.gripper_grap()),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
            ('movel_abs POS_L_DROP',     self.movel_abs(POS_L_DROP)),
            ('gripper_release 1',        self.gripper_release()),
            ('gripper_release 2',        self.gripper_release()),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
            ('movel_rel Z-40',           self.movel_rel([0,0,-40,0,0,0])),
            ('gripper_grap 1',           self.gripper_grap()),
            ('gripper_grap 2',           self.gripper_grap()),
            ('movej_rel J6+90 (1)',      self.movej_rel([0,0,0,0,0,90])),
            ('movej_rel J6+90 (2)',      self.movej_rel([0,0,0,0,0,90])),
            ('gripper_release 1',        self.gripper_release()),
            ('gripper_release 2',        self.gripper_release()),
            ('movel_rel Z+40',           self.movel_rel([0,0,40,0,0,0])),
            ('movej_rel J6-90 (1)',      self.movej_rel([0,0,0,0,0,-90])),
            ('movej_rel J6-90 (2)',      self.movej_rel([0,0,0,0,0,-90])),
            ('movel_rel Z-40',           self.movel_rel([0,0,-40,0,0,0])),
            ('gripper_grap 1',           self.gripper_grap()),
            ('gripper_grap 2',           self.gripper_grap()),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
            ('movel_abs POS_R_DROP',     self.movel_abs(POS_R_DROP)),
            ('gripper_release',          self.gripper_release()),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
        ]

    def seq_F(self):
        return [
            ('gripper_release',          self.gripper_release()),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
            ('movej_rel J6+90',          self.movej_rel([0,0,0,0,0,90])),
            ('movel_rel Z-40',           self.movel_rel([0,0,-40,0,0,0])),
            ('gripper_grap 1',           self.gripper_grap()),
            ('gripper_grap 2',           self.gripper_grap()),
            ('movel_rel Z+40',           self.movel_rel([0,0,40,0,0,0])),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
            ('movel_abs POS_R_DROP',     self.movel_abs(POS_R_DROP)),
            ('gripper_release 1',        self.gripper_release()),
            ('gripper_release 2',        self.gripper_release()),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
            ('movel_rel Z-40',           self.movel_rel([0,0,-40,0,0,0])),
            ('gripper_grap 1',           self.gripper_grap()),
            ('gripper_grap 2',           self.gripper_grap()),
            ('movej_rel J6+90',          self.movej_rel([0,0,0,0,0,90])),
            ('gripper_release 1',        self.gripper_release()),
            ('gripper_release 2',        self.gripper_release()),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
            ('movel_abs POS_R_DROP',     self.movel_abs(POS_R_DROP)),
            ('gripper_grap 1',           self.gripper_grap()),
            ('gripper_grap 2',           self.gripper_grap()),
            ('movel_rel Z+40',           self.movel_rel([0,0,40,0,0,0])),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
            ('movej_rel J6+90',          self.movej_rel([0,0,0,0,0,90])),
            ('movel_rel X-17 Z-15',      self.movel_rel([-17,0,-15,0,0,0])),
            ('movel_rel Z-10',           self.movel_rel([0,0,-10,0,0,0])),
            ('gripper_release',          self.gripper_release()),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
        ]

    def seq_F_prime(self):
        return [
            ('gripper_release',          self.gripper_release()),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
            ('movej_rel J6+90',          self.movej_rel([0,0,0,0,0,90])),
            ('movel_rel Z-40',           self.movel_rel([0,0,-40,0,0,0])),
            ('gripper_grap 1',           self.gripper_grap()),
            ('gripper_grap 2',           self.gripper_grap()),
            ('movel_rel Z+40',           self.movel_rel([0,0,40,0,0,0])),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
            ('movel_abs POS_R_DROP',     self.movel_abs(POS_R_DROP)),
            ('gripper_release 1',        self.gripper_release()),
            ('gripper_release 2',        self.gripper_release()),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
            ('movel_rel Z-40',           self.movel_rel([0,0,-40,0,0,0])),
            ('gripper_grap 1',           self.gripper_grap()),
            ('gripper_grap 2',           self.gripper_grap()),
            ('movej_rel J6-90',          self.movej_rel([0,0,0,0,0,-90])),
            ('gripper_release 1',        self.gripper_release()),
            ('gripper_release 2',        self.gripper_release()),
            ('movel_rel Z+40',           self.movel_rel([0,0,40,0,0,0])),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
            ('movel_abs POS_R_DROP',     self.movel_abs(POS_R_DROP)),
            ('gripper_grap 1',           self.gripper_grap()),
            ('gripper_grap 2',           self.gripper_grap()),
            ('movel_rel Z+40',           self.movel_rel([0,0,40,0,0,0])),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
            ('movej_rel J6+90',          self.movej_rel([0,0,0,0,0,90])),
            ('movel_rel X-17 Z-15',      self.movel_rel([-17,0,-15,0,0,0])),
            ('movel_rel Z-10',           self.movel_rel([0,0,-10,0,0,0])),
            ('gripper_release',          self.gripper_release()),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
        ]

    def seq_F2(self):
        return [
            ('gripper_release',          self.gripper_release()),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
            ('movej_rel J6+90',          self.movej_rel([0,0,0,0,0,90])),
            ('movel_rel Z-40',           self.movel_rel([0,0,-40,0,0,0])),
            ('gripper_grap 1',           self.gripper_grap()),
            ('gripper_grap 2',           self.gripper_grap()),
            ('movel_rel Z+40',           self.movel_rel([0,0,40,0,0,0])),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
            ('movel_abs POS_R_DROP',     self.movel_abs(POS_R_DROP)),
            ('gripper_release 1',        self.gripper_release()),
            ('gripper_release 2',        self.gripper_release()),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
            ('movel_rel Z-40',           self.movel_rel([0,0,-40,0,0,0])),
            ('gripper_grap 1',           self.gripper_grap()),
            ('gripper_grap 2',           self.gripper_grap()),
            ('movej_rel J6+90 (1)',      self.movej_rel([0,0,0,0,0,90])),
            ('movej_rel J6+90 (2)',      self.movej_rel([0,0,0,0,0,90])),
            ('gripper_release 1',        self.gripper_release()),
            ('gripper_release 2',        self.gripper_release()),
            ('movel_rel Z+40',           self.movel_rel([0,0,40,0,0,0])),
            ('movej_rel J6-90 (1)',      self.movej_rel([0,0,0,0,0,-90])),
            ('movej_rel J6-90 (2)',      self.movej_rel([0,0,0,0,0,-90])),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
            ('movel_abs POS_R_DROP',     self.movel_abs(POS_R_DROP)),
            ('gripper_grap 1',           self.gripper_grap()),
            ('gripper_grap 2',           self.gripper_grap()),
            ('movel_rel Z+40',           self.movel_rel([0,0,40,0,0,0])),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
            ('movej_rel J6+90',          self.movej_rel([0,0,0,0,0,90])),
            ('movel_rel X-17 Z-15',      self.movel_rel([-17,0,-15,0,0,0])),
            ('movel_rel Z-10',           self.movel_rel([0,0,-10,0,0,0])),
            ('gripper_release',          self.gripper_release()),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
        ]

    def seq_B(self):
        return [
            ('gripper_release',          self.gripper_release()),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
            ('movej_rel J6+90',          self.movej_rel([0,0,0,0,0,90])),
            ('movel_rel Z-40',           self.movel_rel([0,0,-40,0,0,0])),
            ('gripper_grap 1',           self.gripper_grap()),
            ('gripper_grap 2',           self.gripper_grap()),
            ('movel_rel Z+40',           self.movel_rel([0,0,40,0,0,0])),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
            ('movel_abs POS_L_DROP',     self.movel_abs(POS_L_DROP)),
            ('gripper_release 1',        self.gripper_release()),
            ('gripper_release 2',        self.gripper_release()),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
            ('movel_rel Z-40',           self.movel_rel([0,0,-40,0,0,0])),
            ('gripper_grap 1',           self.gripper_grap()),
            ('gripper_grap 2',           self.gripper_grap()),
            ('movej_rel J6+90',          self.movej_rel([0,0,0,0,0,90])),
            ('gripper_release 1',        self.gripper_release()),
            ('gripper_release 2',        self.gripper_release()),
            ('movel_rel Z+40',           self.movel_rel([0,0,40,0,0,0])),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
            ('movel_abs POS_L_DROP',     self.movel_abs(POS_L_DROP)),
            ('gripper_grap 1',           self.gripper_grap()),
            ('gripper_grap 2',           self.gripper_grap()),
            ('movel_rel Z+40',           self.movel_rel([0,0,40,0,0,0])),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
            ('movej_rel J6+90',          self.movej_rel([0,0,0,0,0,90])),
            ('movel_rel X+17 Z-15',      self.movel_rel([17,0,-15,0,0,0])),
            ('movel_rel Z-10',           self.movel_rel([0,0,-10,0,0,0])),
            ('gripper_release',          self.gripper_release()),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
        ]

    def seq_B_prime(self):
        return [
            ('gripper_release',          self.gripper_release()),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
            ('movej_rel J6+90',          self.movej_rel([0,0,0,0,0,90])),
            ('movel_rel Z-40',           self.movel_rel([0,0,-40,0,0,0])),
            ('gripper_grap 1',           self.gripper_grap()),
            ('gripper_grap 2',           self.gripper_grap()),
            ('movel_rel Z+40',           self.movel_rel([0,0,40,0,0,0])),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
            ('movel_abs POS_L_DROP',     self.movel_abs(POS_L_DROP)),
            ('gripper_release 1',        self.gripper_release()),
            ('gripper_release 2',        self.gripper_release()),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
            ('movel_rel Z-40',           self.movel_rel([0,0,-40,0,0,0])),
            ('gripper_grap 1',           self.gripper_grap()),
            ('gripper_grap 2',           self.gripper_grap()),
            ('movej_rel J6-90',          self.movej_rel([0,0,0,0,0,-90])),
            ('gripper_release 1',        self.gripper_release()),
            ('gripper_release 2',        self.gripper_release()),
            ('movel_rel Z+40',           self.movel_rel([0,0,40,0,0,0])),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
            ('movel_abs POS_L_DROP',     self.movel_abs(POS_L_DROP)),
            ('gripper_grap 1',           self.gripper_grap()),
            ('gripper_grap 2',           self.gripper_grap()),
            ('movel_rel Z+40',           self.movel_rel([0,0,40,0,0,0])),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
            ('movej_rel J6+90',          self.movej_rel([0,0,0,0,0,90])),
            ('movel_rel X+17 Z-15',      self.movel_rel([17,0,-15,0,0,0])),
            ('movel_rel Z-10',           self.movel_rel([0,0,-10,0,0,0])),
            ('gripper_release',          self.gripper_release()),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
        ]

    def seq_B2(self):
        return [
            ('gripper_release',          self.gripper_release()),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
            ('movej_rel J6+90',          self.movej_rel([0,0,0,0,0,90])),
            ('movel_rel Z-40',           self.movel_rel([0,0,-40,0,0,0])),
            ('gripper_grap 1',           self.gripper_grap()),
            ('gripper_grap 2',           self.gripper_grap()),
            ('movel_rel Z+40',           self.movel_rel([0,0,40,0,0,0])),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
            ('movel_abs POS_L_DROP',     self.movel_abs(POS_L_DROP)),
            ('gripper_release 1',        self.gripper_release()),
            ('gripper_release 2',        self.gripper_release()),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
            ('movel_rel Z-40',           self.movel_rel([0,0,-40,0,0,0])),
            ('gripper_grap 1',           self.gripper_grap()),
            ('gripper_grap 2',           self.gripper_grap()),
            ('movej_rel J6+90 (1)',      self.movej_rel([0,0,0,0,0,90])),
            ('movej_rel J6+90 (2)',      self.movej_rel([0,0,0,0,0,90])),
            ('gripper_release 1',        self.gripper_release()),
            ('gripper_release 2',        self.gripper_release()),
            ('movel_rel Z+40',           self.movel_rel([0,0,40,0,0,0])),
            ('movej_rel J6-90 (1)',      self.movej_rel([0,0,0,0,0,-90])),
            ('movej_rel J6-90 (2)',      self.movej_rel([0,0,0,0,0,-90])),
            ('movel_abs POS_L_DROP',     self.movel_abs(POS_L_DROP)),
            ('gripper_grap 1',           self.gripper_grap()),
            ('gripper_grap 2',           self.gripper_grap()),
            ('movel_rel Z+40',           self.movel_rel([0,0,40,0,0,0])),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
            ('movej_rel J6+90',          self.movej_rel([0,0,0,0,0,90])),
            ('movel_rel X+17 Z-15',      self.movel_rel([17,0,-15,0,0,0])),
            ('movel_rel Z-10',           self.movel_rel([0,0,-10,0,0,0])),
            ('gripper_release',          self.gripper_release()),
            ('movel_abs POS_INITIAL',    self.movel_abs(POS_INITIAL)),
        ]

    def seq_PickupFromWorkstation(self, cube_x: float, cube_y: float):
        """
        워크스테이션 랜덤 위치 큐브 픽업 시퀀스.

        cube_x, cube_y: cube_perception DetectCubePose가 반환한 XY 좌표 (mm)

        동작 순서:
          1) 그리퍼 개방
          2) [cube_x, cube_y, POS_INITIAL Z] 수평 이동
          3) Z 하강 → WORKSTATION_GRIP_Z(13.71mm + TCP_OFFSET)까지
             하강량 = -(POS_INITIAL[2] - WORKSTATION_GRIP_Z)
          4) 큐브 파지
          5) Z +40mm 상승

        TODO: 현재 approach Z = POS_INITIAL[2] 고정값
              차후 개선 방향:
                - approach Z = cube_pose.z + SAFE_OFFSET(30mm 이상)으로 동적 계산
                - 객체 높이에 관계없이 범용 파지 가능하도록 보완
        """
        approach_z  = POS_INITIAL[2]
        descend_rel = -(approach_z - WORKSTATION_GRIP_Z)  # -60.95mm
        orient      = POS_INITIAL[3:]

        return [
            ('gripper_release',
                self.gripper_release()),
            ('movel_abs approach_xy',
                self.movel_abs([cube_x, cube_y, approach_z] + orient)),
            ('movel_rel descend_z',
                self.movel_rel([0, 0, descend_rel, 0, 0, 0])),
            ('gripper_grap',
                self.gripper_grap()),
            ('movel_rel lift_z',
                self.movel_rel([0, 0, 40, 0, 0, 0])),
        ]

    def seq_Perception(self):
        POS_CAMERA_R_DROP = [373.03, 12.36, 48.53, 90.0, -90.0, 90.0]
        POS_CAMERA_L_DROP = [373.03, -12.96, 48.53, 90.0, 90.0, 90.0]
        return [
            ('gripper_release',              self.gripper_release()),
            ('movel_abs POS_INITIAL',        self.movel_abs(POS_INITIAL)),
            ('movel_rel Z-30',               self.movel_rel([0,0,-30,0,0,0])),
            ('gripper_grap',                 self.gripper_grap()),
            ('movel_abs POS_INITIAL',        self.movel_abs(POS_INITIAL)),
            ('movel_rel Y-293',              self.movel_rel([0,-293.33,0,0,0,0])),
            ('movel_rel Z-57',               self.movel_rel([0,0,-57.78,0,0,0])),
            ('movej_rel J6+90 (1)',          self.movej_rel([0,0,0,0,0,90])),
            ('movej_rel J6+90 (2)',          self.movej_rel([0,0,0,0,0,90])),
            ('movej_rel J6+90 (3)',          self.movej_rel([0,0,0,0,0,90])),
            ('movej_rel J6-90 (1)',          self.movej_rel([0,0,0,0,0,-90])),
            ('movej_rel J6-90 (2)',          self.movej_rel([0,0,0,0,0,-90])),
            ('movej_rel J6-90 (3)',          self.movej_rel([0,0,0,0,0,-90])),
            ('movel_rel Z+57',               self.movel_rel([0,0,57.78,0,0,0])),
            ('movel_abs POS_INITIAL',        self.movel_abs(POS_INITIAL)),
            ('movel_abs POS_CAMERA_R_DROP',  self.movel_abs(POS_CAMERA_R_DROP)),
            ('gripper_release',              self.gripper_release()),
            ('movel_abs POS_INITIAL',        self.movel_abs(POS_INITIAL)),
            ('movel_rel Z-30',               self.movel_rel([0,0,-30,0,0,0])),
            ('gripper_grap',                 self.gripper_grap()),
            ('movel_abs POS_INITIAL',        self.movel_abs(POS_INITIAL)),
            ('movel_rel Y-293',              self.movel_rel([0,-293.33,0,0,0,0])),
            ('movel_rel Z-57',               self.movel_rel([0,0,-57.78,0,0,0])),
            ('movej_rel J6+90 (1)',          self.movej_rel([0,0,0,0,0,90])),
            ('movej_rel J6+90 (2)',          self.movej_rel([0,0,0,0,0,90])),
            ('movej_rel J6-90 (1)',          self.movej_rel([0,0,0,0,0,-90])),
            ('movej_rel J6-90 (2)',          self.movej_rel([0,0,0,0,0,-90])),
            ('movel_rel Z+57',               self.movel_rel([0,0,57.78,0,0,0])),
            ('movel_abs POS_INITIAL',        self.movel_abs(POS_INITIAL)),
            ('movel_abs POS_CAMERA_L_DROP',  self.movel_abs(POS_CAMERA_L_DROP)),
            ('gripper_release',              self.gripper_release()),
            ('movel_abs POS_INITIAL',        self.movel_abs(POS_INITIAL)),
        ]


def main(args=None):
    rclpy.init(args=args)
    node = RobotActionServerNode()
    executor = rclpy.executors.MultiThreadedExecutor()
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
