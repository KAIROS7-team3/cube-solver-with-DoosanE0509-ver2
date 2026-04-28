"""
motion_library.py
─────────────────
변경사항:
  - HOME (movel_abs INITIAL_STATE) → ZIG_HOME 으로 이름 변경
  - JOINT_HOME 신규 추가: movej 절대값 [0,0,90,0,90,0]
    → robot_action_server_node의 execute_cb에서
      전체 시퀀스 앞뒤에 JOINT_HOME을 붙여 사용
  - 모든 시퀀스 내 HOME() → ZIG_HOME() 으로 교체
  - StepKind.MOVE_J_ABS 추가 (JOINT_HOME 절대 이동용)
"""

from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional


# ═══════════════════════════════════════════════════════════
# StepKind 정의
# ═══════════════════════════════════════════════════════════

class StepKind(Enum):
    MOVE_L_ABS = auto()   # movel — 절대 좌표 (base 기준)
    MOVE_L_REL = auto()   # movel — 상대 변위
    MOVE_J_ABS = auto()   # movej — 관절 절대값 (deg)  ← 신규
    MOVE_J_REL = auto()   # movej — 관절 상대 회전 (deg)
    GRIP       = auto()   # 그리퍼 pulse 설정
    WAIT       = auto()   # 대기 (sec)


# ═══════════════════════════════════════════════════════════
# Step 데이터클래스
# ═══════════════════════════════════════════════════════════

@dataclass
class Step:
    name:  str
    kind:  StepKind
    pose:  Optional[list] = None    # [x,y,z,rx,ry,rz] mm/deg
    vel:   Optional[float] = None
    acc:   Optional[float] = None
    pulse: Optional[int]   = None   # 그리퍼 PULSE 상수
    sec:   Optional[float] = None   # WAIT 시간


# ═══════════════════════════════════════════════════════════
# 속도 / 가속도 기본값
# ═══════════════════════════════════════════════════════════

VEL_X: float = 20.0   # mm/sec
VEL_R: float = 20.0   # deg/sec
ACC_X: float = 15.0    # mm/sec²
ACC_R: float = 15.0    # deg/sec²
VEL_J: float = 20.0   # deg/sec
ACC_J: float = 15.0    # deg/sec²

# ═══════════════════════════════════════════════════════════
# TCP 설정값 (참조용 — 서비스 등록은 server_node에서)
# ═══════════════════════════════════════════════════════════

TCP_NAME:      str   = 'gripper_tcp'
TCP_POS:       list  = [0.0, 0.0, 160.0, 0.0, 0.0, 0.0]
TCP_WEIGHT_KG: float = 0.5

# ═══════════════════════════════════════════════════════════
# 그리퍼 PULSE 상수
# ═══════════════════════════════════════════════════════════

PULSE_OPEN:   int = 200
PULSE_CUBE:   int = 420
PULSE_ROTATE: int = 420
PULSE_REPOSE: int = 420

# ═══════════════════════════════════════════════════════════
# 웨이포인트 상수 (TCP 오프셋 없음 — 컨트롤러가 처리)
# ═══════════════════════════════════════════════════════════

# 원래 DRL 좌표값으로 복원
INITIAL_STATE: list = [373.030,   0.000,  74.660, 22.85, -180.0,  22.85]
L_DROP_POS:    list = [373.030,   0.000,  48.530, 90.0,    90.0,  90.0 ]
R_DROP_POS:    list = [373.030,   0.000,  48.530, 90.0,   -90.0,  90.0 ]
READY_D_LOW:   list = [373.010,   0.000,  20.000, 22.85, -180.0,  22.85]
READY_D_UP:    list = [373.020,   0.000,  27.730,  6.68, -180.0,   6.68]
CAM_R_DROP:    list = [373.030,  12.360,  48.530, 90.0,   -90.0,  90.0 ]
CAM_L_DROP:    list = [373.030, -12.960,  48.530, 90.0,    90.0,  90.0 ]

# Joint Home 자세 (절대값 deg)
JOINT_HOME_POS: list = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]

# 상대 이동 상수
Z_40_DOWN:   list = [0,       0,     -40,    0, 0, 0]
Z_40_UP:     list = [0,       0,      40,    0, 0, 0]
Z_30_DOWN:   list = [0,       0,     -30,    0, 0, 0]
Z_57_DOWN:   list = [0,       0,    -57.78,  0, 0, 0]
Z_57_UP:     list = [0,       0,     57.78,  0, 0, 0]
Z_10_DOWN:   list = [0,       0,     -10,    0, 0, 0]
Y_293_NEG:   list = [0,   -293.33,    0,     0, 0, 0]
X17_Z15_NEG: list = [-17,    0,      -15,    0, 0, 0]
X17_Z15_POS: list = [ 17,    0,      -15,    0, 0, 0]


# ═══════════════════════════════════════════════════════════
# Step 헬퍼 팩토리
# ═══════════════════════════════════════════════════════════

def ml_abs(name: str, pos: list) -> Step:
    return Step(name=name, kind=StepKind.MOVE_L_ABS, pose=list(pos),
                vel=VEL_X, acc=ACC_X)

def ml_rel(name: str, pos: list) -> Step:
    return Step(name=name, kind=StepKind.MOVE_L_REL, pose=list(pos),
                vel=VEL_X, acc=ACC_X)

def mj_abs(name: str, joints: list) -> Step:
    """관절 절대 이동 — JOINT_HOME 등에 사용."""
    return Step(name=name, kind=StepKind.MOVE_J_ABS, pose=list(joints),
                vel=VEL_J, acc=ACC_J)

def mj_rel(name: str, joints: list) -> Step:
    """관절 상대 이동 — J6 회전 등에 사용."""
    return Step(name=name, kind=StepKind.MOVE_J_REL, pose=list(joints),
                vel=VEL_J, acc=ACC_J)

def grip(name: str, pulse: int) -> Step:
    return Step(name=name, kind=StepKind.GRIP, pulse=pulse)

def wait(name: str, sec: float) -> Step:
    return Step(name=name, kind=StepKind.WAIT, sec=sec)

# ── 자주 쓰는 Step 람다 ────────────────────────────────────
GRIP_OPEN  = lambda: grip('gripper_open', PULSE_OPEN)
GRIP_CUBE  = lambda: grip('gripper_cube', PULSE_CUBE)

# ZIG_HOME: movel 절대값으로 지그 중심(INITIAL_STATE)으로 이동
ZIG_HOME   = lambda: ml_abs('zig_home', INITIAL_STATE)

# JOINT_HOME: movej 절대값으로 안전 자세([0,0,90,0,90,0])로 이동
# → execute_cb에서 전체 시퀀스 앞뒤에 자동으로 추가됨
JOINT_HOME = lambda: mj_abs('joint_home', JOINT_HOME_POS)

Z_DOWN     = lambda: ml_rel('z_down_40',  Z_40_DOWN)
Z_UP       = lambda: ml_rel('z_up_40',    Z_40_UP)
J6_CW      = lambda n='': mj_rel(f'j6_cw90{n}',  [0, 0, 0, 0, 0,  90])
J6_CCW     = lambda n='': mj_rel(f'j6_ccw90{n}', [0, 0, 0, 0, 0, -90])


# ═══════════════════════════════════════════════════════════
# 19개 토큰 시퀀스 (ZIG_HOME 사용, JOINT_HOME은 server_node에서 추가)
# ═══════════════════════════════════════════════════════════

def _u_seq() -> list:
    return [
        GRIP_OPEN(), ZIG_HOME(), Z_DOWN(),
        GRIP_CUBE(),
        J6_CW('_rot'),
        GRIP_OPEN(),
        Z_UP(), ZIG_HOME(),
    ]

def _u_prime_seq() -> list:
    return [
        GRIP_OPEN(), ZIG_HOME(), Z_DOWN(),
        GRIP_CUBE(),
        J6_CCW('_rot'),
        GRIP_OPEN(),
        Z_UP(), ZIG_HOME(),
    ]

def _u2_seq() -> list:
    return [
        GRIP_OPEN(), ZIG_HOME(), Z_DOWN(),
        GRIP_CUBE(),
        J6_CW('_1'), J6_CW('_2'),
        GRIP_OPEN(),
        Z_UP(),
        J6_CCW('_ret1'), J6_CCW('_ret2'),
        ZIG_HOME(),
    ]

def _d_seq() -> list:
    return [
        GRIP_OPEN(), ZIG_HOME(),
        ml_abs('ready_d_low', READY_D_LOW),
        GRIP_CUBE(),
        ml_abs('ready_d_up',  READY_D_UP),
        J6_CW('_rot'),
        Z_UP(),
        J6_CCW('_ret'),
        Z_DOWN(),
        GRIP_OPEN(), ZIG_HOME(),
    ]

def _d_prime_seq() -> list:
    return [
        GRIP_OPEN(), ZIG_HOME(),
        ml_abs('ready_d_low', READY_D_LOW),
        GRIP_CUBE(),
        ml_abs('ready_d_up',  READY_D_UP),
        J6_CCW('_rot'),
        Z_UP(),
        J6_CW('_ret'),
        Z_DOWN(),
        GRIP_OPEN(), ZIG_HOME(),
    ]

def _d2_seq() -> list:
    return [
        GRIP_OPEN(), ZIG_HOME(),
        ml_abs('ready_d_low', READY_D_LOW),
        GRIP_CUBE(),
        ml_abs('ready_d_up',  READY_D_UP),
        J6_CW('_1'), J6_CW('_2'),
        Z_UP(),
        J6_CCW('_ret1'), J6_CCW('_ret2'),
        Z_DOWN(),
        GRIP_OPEN(), ZIG_HOME(),
    ]

def _r_seq() -> list:
    return [
        GRIP_OPEN(), ZIG_HOME(), Z_DOWN(),
        GRIP_CUBE(),
        ZIG_HOME(),
        ml_abs('r_drop', R_DROP_POS),
        GRIP_OPEN(),
        ZIG_HOME(), Z_DOWN(),
        GRIP_CUBE(),
        J6_CW('_rot'),
        GRIP_OPEN(),
        ZIG_HOME(), Z_DOWN(),
        GRIP_CUBE(),
        ZIG_HOME(),
        ml_abs('l_drop', L_DROP_POS),
        GRIP_OPEN(), ZIG_HOME(),
    ]

def _r_prime_seq() -> list:
    return [
        GRIP_OPEN(), ZIG_HOME(), Z_DOWN(),
        GRIP_CUBE(),
        ZIG_HOME(),
        ml_abs('r_drop', R_DROP_POS),
        GRIP_OPEN(),
        ZIG_HOME(), Z_DOWN(),
        GRIP_CUBE(),
        J6_CCW('_rot'),
        GRIP_OPEN(),
        ZIG_HOME(), Z_DOWN(),
        GRIP_CUBE(),
        ZIG_HOME(),
        ml_abs('l_drop', L_DROP_POS),
        GRIP_OPEN(), ZIG_HOME(),
    ]

def _r2_seq() -> list:
    return [
        GRIP_OPEN(), ZIG_HOME(), Z_DOWN(),
        GRIP_CUBE(),
        ZIG_HOME(),
        ml_abs('r_drop', R_DROP_POS),
        GRIP_OPEN(),
        ZIG_HOME(), Z_DOWN(),
        GRIP_CUBE(),
        J6_CW('_1'), J6_CW('_2'),
        GRIP_OPEN(),
        Z_UP(),
        J6_CCW('_ret1'), J6_CCW('_ret2'),
        Z_DOWN(),
        GRIP_CUBE(),
        ZIG_HOME(),
        ml_abs('l_drop', L_DROP_POS),
        GRIP_OPEN(), ZIG_HOME(),
    ]

def _l_seq() -> list:
    return [
        GRIP_OPEN(), ZIG_HOME(), Z_DOWN(),
        GRIP_CUBE(),
        Z_UP(),
        ml_abs('l_drop', L_DROP_POS),
        GRIP_OPEN(),
        ZIG_HOME(), Z_DOWN(),
        GRIP_CUBE(),
        J6_CW('_rot'),
        GRIP_OPEN(),
        Z_UP(), ZIG_HOME(), Z_DOWN(),
        GRIP_CUBE(),
        ZIG_HOME(),
        ml_abs('r_drop', R_DROP_POS),
        GRIP_OPEN(), ZIG_HOME(),
    ]

def _l_prime_seq() -> list:
    return [
        GRIP_OPEN(), ZIG_HOME(), Z_DOWN(),
        GRIP_CUBE(),
        ZIG_HOME(),
        ml_abs('l_drop', L_DROP_POS),
        GRIP_OPEN(),
        ZIG_HOME(), Z_DOWN(),
        GRIP_CUBE(),
        J6_CCW('_rot'),
        GRIP_OPEN(),
        ZIG_HOME(), Z_DOWN(),
        GRIP_CUBE(),
        ZIG_HOME(),
        ml_abs('r_drop', R_DROP_POS),
        GRIP_OPEN(), ZIG_HOME(),
    ]

def _l2_seq() -> list:
    return [
        GRIP_OPEN(), ZIG_HOME(), Z_DOWN(),
        GRIP_CUBE(),
        ZIG_HOME(),
        ml_abs('l_drop', L_DROP_POS),
        GRIP_OPEN(),
        ZIG_HOME(), Z_DOWN(),
        GRIP_CUBE(),
        J6_CW('_1'), J6_CW('_2'),
        GRIP_OPEN(),
        Z_UP(),
        J6_CCW('_ret1'), J6_CCW('_ret2'),
        Z_DOWN(),
        GRIP_CUBE(),
        ZIG_HOME(),
        ml_abs('r_drop', R_DROP_POS),
        GRIP_OPEN(), ZIG_HOME(),
    ]

def _f_seq() -> list:
    return [
        GRIP_OPEN(), ZIG_HOME(),
        J6_CW('_pre'), Z_DOWN(),
        GRIP_CUBE(),
        Z_UP(), ZIG_HOME(),
        ml_abs('r_drop', R_DROP_POS),
        GRIP_OPEN(),
        ZIG_HOME(), Z_DOWN(),
        GRIP_CUBE(),
        J6_CW('_rot'),
        GRIP_OPEN(),
        ZIG_HOME(),
        ml_abs('r_drop', R_DROP_POS),
        GRIP_CUBE(),
        Z_UP(), ZIG_HOME(),
        J6_CW('_place'),
        ml_rel('x17_z15', X17_Z15_NEG),
        ml_rel('z10_down', Z_10_DOWN),
        GRIP_OPEN(), ZIG_HOME(),
    ]

def _f_prime_seq() -> list:
    return [
        GRIP_OPEN(), ZIG_HOME(),
        J6_CW('_pre'), Z_DOWN(),
        GRIP_CUBE(),
        Z_UP(), ZIG_HOME(),
        ml_abs('r_drop', R_DROP_POS),
        GRIP_OPEN(),
        ZIG_HOME(), Z_DOWN(),
        GRIP_CUBE(),
        J6_CCW('_rot'),
        GRIP_OPEN(),
        Z_UP(), ZIG_HOME(),
        ml_abs('r_drop', R_DROP_POS),
        GRIP_CUBE(),
        Z_UP(), ZIG_HOME(),
        J6_CW('_place'),
        ml_rel('x17_z15', X17_Z15_NEG),
        ml_rel('z10_down', Z_10_DOWN),
        GRIP_OPEN(), ZIG_HOME(),
    ]

def _f2_seq() -> list:
    return [
        GRIP_OPEN(), ZIG_HOME(),
        J6_CW('_pre'), Z_DOWN(),
        GRIP_CUBE(),
        Z_UP(), ZIG_HOME(),
        ml_abs('r_drop', R_DROP_POS),
        GRIP_OPEN(),
        ZIG_HOME(), Z_DOWN(),
        GRIP_CUBE(),
        J6_CW('_1'), J6_CW('_2'),
        GRIP_OPEN(),
        Z_UP(),
        J6_CCW('_ret1'), J6_CCW('_ret2'),
        ZIG_HOME(),
        ml_abs('r_drop', R_DROP_POS),
        GRIP_CUBE(),
        Z_UP(), ZIG_HOME(),
        J6_CW('_place'),
        ml_rel('x17_z15', X17_Z15_NEG),
        ml_rel('z10_down', Z_10_DOWN),
        GRIP_OPEN(), ZIG_HOME(),
    ]

def _b_seq() -> list:
    return [
        GRIP_OPEN(), ZIG_HOME(),
        J6_CW('_pre'), Z_DOWN(),
        GRIP_CUBE(),
        Z_UP(), ZIG_HOME(),
        ml_abs('l_drop', L_DROP_POS),
        GRIP_OPEN(),
        ZIG_HOME(), Z_DOWN(),
        GRIP_CUBE(),
        J6_CW('_rot'),
        GRIP_OPEN(),
        Z_UP(), ZIG_HOME(),
        ml_abs('l_drop', L_DROP_POS),
        GRIP_CUBE(),
        Z_UP(), ZIG_HOME(),
        J6_CW('_place'),
        ml_rel('x17_z15', X17_Z15_POS),
        ml_rel('z10_down', Z_10_DOWN),
        GRIP_OPEN(), ZIG_HOME(),
    ]

def _b_prime_seq() -> list:
    return [
        GRIP_OPEN(), ZIG_HOME(),
        J6_CW('_pre'), Z_DOWN(),
        GRIP_CUBE(),
        Z_UP(), ZIG_HOME(),
        ml_abs('l_drop', L_DROP_POS),
        GRIP_OPEN(),
        ZIG_HOME(), Z_DOWN(),
        GRIP_CUBE(),
        J6_CCW('_rot'),
        GRIP_OPEN(),
        Z_UP(), ZIG_HOME(),
        ml_abs('l_drop', L_DROP_POS),
        GRIP_CUBE(),
        Z_UP(), ZIG_HOME(),
        J6_CW('_place'),
        ml_rel('x17_z15', X17_Z15_POS),
        ml_rel('z10_down', Z_10_DOWN),
        GRIP_OPEN(), ZIG_HOME(),
    ]

def _b2_seq() -> list:
    return [
        GRIP_OPEN(), ZIG_HOME(),
        J6_CW('_pre'), Z_DOWN(),
        GRIP_CUBE(),
        Z_UP(), ZIG_HOME(),
        ml_abs('l_drop', L_DROP_POS),
        GRIP_OPEN(),
        ZIG_HOME(), Z_DOWN(),
        GRIP_CUBE(),
        J6_CW('_1'), J6_CW('_2'),
        GRIP_OPEN(),
        Z_UP(),
        J6_CCW('_ret1'), J6_CCW('_ret2'),
        ml_abs('l_drop', L_DROP_POS),
        GRIP_CUBE(),
        Z_UP(), ZIG_HOME(),
        J6_CW('_place'),
        ml_rel('x17_z15', X17_Z15_POS),
        ml_rel('z10_down', Z_10_DOWN),
        GRIP_OPEN(), ZIG_HOME(),
    ]

def _perception_seq() -> list:
    return [
        GRIP_OPEN(), ZIG_HOME(),
        ml_rel('z30_down', Z_30_DOWN),
        GRIP_CUBE(),
        ZIG_HOME(),
        ml_rel('y293_neg', Y_293_NEG),
        ml_rel('z57_down', Z_57_DOWN),
        J6_CW('_1'), J6_CW('_2'), J6_CW('_3'),
        J6_CCW('_1'), J6_CCW('_2'), J6_CCW('_3'),
        ml_rel('z57_up', Z_57_UP),
        ZIG_HOME(),
        ml_abs('cam_r_drop', CAM_R_DROP),
        GRIP_OPEN(), ZIG_HOME(),
        ml_rel('z30_down', Z_30_DOWN),
        GRIP_CUBE(),
        ZIG_HOME(),
        ml_rel('y293_neg', Y_293_NEG),
        ml_rel('z57_down', Z_57_DOWN),
        J6_CW('_4'), J6_CW('_5'),
        J6_CCW('_4'), J6_CCW('_5'),
        ml_rel('z57_up', Z_57_UP),
        ZIG_HOME(),
        ml_abs('cam_l_drop', CAM_L_DROP),
        GRIP_OPEN(), ZIG_HOME(),
    ]


# ═══════════════════════════════════════════════════════════
# 토큰 → 시퀀스 맵 (공개 API)
# ═══════════════════════════════════════════════════════════

_TOKEN_MAP = {
    'U'  : _u_seq,
    "U'" : _u_prime_seq,
    'U2' : _u2_seq,
    'D'  : _d_seq,
    "D'" : _d_prime_seq,
    'D2' : _d2_seq,
    'R'  : _r_seq,
    "R'" : _r_prime_seq,
    'R2' : _r2_seq,
    'L'  : _l_seq,
    "L'" : _l_prime_seq,
    'L2' : _l2_seq,
    'F'  : _f_seq,
    "F'" : _f_prime_seq,
    'F2' : _f2_seq,
    'B'  : _b_seq,
    "B'" : _b_prime_seq,
    'B2' : _b2_seq,
    'P'  : _perception_seq,
}

VALID_TOKENS: list = list(_TOKEN_MAP.keys())


def token_sequence(token: str) -> list:
    """토큰 문자열 → Step 리스트 반환."""
    fn = _TOKEN_MAP.get(token)
    if fn is None:
        raise ValueError(
            f'알 수 없는 토큰: {token!r}. 유효 토큰: {VALID_TOKENS}'
        )
    return fn()
