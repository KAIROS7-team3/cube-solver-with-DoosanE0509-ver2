"""
motion_library.py
─────────────────
명세서(cube_solver_node_spec) 3.2 기준 구현.
ROS2 의존성 없음 — 단독 테스트 가능.

변경사항 (DRL 스크립트 통합):
  - CUBE_SCAN_B / CUBE_SCAN_D 상수 추가 (DRL Cube_recognition_process 좌표)
  - pickup_from_jig_seq() 추가 — cube_perception 없는 환경용 임시 픽업
  - pickup_step_seq() 4단계 분리 — 명령-동작-완료 패턴 지원
  - place_step_seq() 2단계 분리 — 명령-동작-완료 패턴 지원
  - rotate_face_seq() 면별 단일 동작 — 명령-동작-완료 패턴 지원
"""

from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional


# ═══════════════════════════════════════════════════════════
# StepKind 정의 (명세서 3.2)
# ═══════════════════════════════════════════════════════════

class StepKind(Enum):
    MOVE_L_ABS = auto()
    MOVE_L_REL = auto()
    MOVE_J_ABS = auto()
    MOVE_J_REL = auto()
    GRIP       = auto()
    WAIT       = auto()


# ═══════════════════════════════════════════════════════════
# Step 데이터클래스 (명세서 3.2)
# ═══════════════════════════════════════════════════════════

@dataclass
class Step:
    kind:  StepKind
    pose:  Optional[list]  = None
    vel:   Optional[float] = None
    acc:   Optional[float] = None
    pulse: Optional[int]   = None
    sec:   Optional[float] = None


# ═══════════════════════════════════════════════════════════
# 속도 / 가속도 기본값
# ═══════════════════════════════════════════════════════════

VEL_X: float = 20.0
VEL_R: float = 20.0
ACC_X: float = 15.0
ACC_R: float = 15.0
VEL_J: float = 20.0
ACC_J: float = 15.0

# DRL 원본 속도 (Cube_recognition_process.drl 기준)
VEL_X_DRL: float = 250.0
VEL_R_DRL: float = 76.5
ACC_X_DRL: float = 1000.0
ACC_R_DRL: float = 306.0
VEL_J_DRL: float = 60.0
ACC_J_DRL: float = 100.0

# ═══════════════════════════════════════════════════════════
# TCP 설정값
# ═══════════════════════════════════════════════════════════

TCP_NAME: str  = 'GripperDA_v1'
TCP_POS:  list = [0.0, 0.0, 160.0, 0.0, 0.0, 0.0]

# ═══════════════════════════════════════════════════════════
# PULSE 상수 (명세서 1.3)
# ═══════════════════════════════════════════════════════════

PULSE_OPEN:   int = 200
PULSE_CUBE:   int = 420
PULSE_ROTATE: int = 420
PULSE_REPOSE: int = 420

# ═══════════════════════════════════════════════════════════
# 웨이포인트 상수
# ═══════════════════════════════════════════════════════════

# ZIG 위치 (INITIAL_STATE) — 큐브 안치 기준점
INITIAL_STATE: list = [373.030,   0.000,  74.660, 22.85, -180.0,  22.85]

# 워크스테이션 큐브 파지 Z 고정값 (tw 파일 실측: 큐브인식모션테스트.tw)
# 워크스테이션 바닥이 평평하므로 XY 위치와 무관하게 항상 동일한 Z에서 파지
WORKSTATION_GRIP_Z: float = 13.71   # mm

# 솔빙 동작용 드롭 위치
L_DROP_POS:    list = [373.030,   0.000,  48.530, 90.0,    90.0,  90.0 ]
R_DROP_POS:    list = [373.030,   0.000,  48.530, 90.0,   -90.0,  90.0 ]
READY_D_LOW:   list = [373.010,   0.000,  20.000, 22.85, -180.0,  22.85]
READY_D_UP:    list = [373.020,   0.000,  27.730,  6.68, -180.0,   6.68]

# 카메라 스캔 위치 (DRL Cube_recognition_process.drl 좌표)
CUBE_SCAN_B:   list = [669.3, -47.19, 505.71, 176.07,  -84.66,  179.98]  # B/R/F/L면 기준
CUBE_SCAN_D:   list = [669.3, -47.19, 505.71, 160.24,  -11.66, -161.00]  # D면 전용

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

def ml_abs(pos: list, vel: float = None, acc: float = None) -> Step:
    return Step(kind=StepKind.MOVE_L_ABS, pose=list(pos),
                vel=vel or VEL_X, acc=acc or ACC_X)

def ml_rel(pos: list, vel: float = None, acc: float = None) -> Step:
    return Step(kind=StepKind.MOVE_L_REL, pose=list(pos),
                vel=vel or VEL_X, acc=acc or ACC_X)

def mj_abs(joints: list, vel: float = None, acc: float = None) -> Step:
    return Step(kind=StepKind.MOVE_J_ABS, pose=list(joints),
                vel=vel or VEL_J, acc=acc or ACC_J)

def mj_rel(joints: list, vel: float = None, acc: float = None) -> Step:
    return Step(kind=StepKind.MOVE_J_REL, pose=list(joints),
                vel=vel or VEL_J, acc=acc or ACC_J)

def grip(pulse: int) -> Step:
    return Step(kind=StepKind.GRIP, pulse=pulse)

def wait_step(sec: float) -> Step:
    return Step(kind=StepKind.WAIT, sec=sec)

# 람다 헬퍼
GRIP_OPEN  = lambda: grip(PULSE_OPEN)
GRIP_CUBE  = lambda: grip(PULSE_CUBE)
ZIG_HOME   = lambda: ml_abs(INITIAL_STATE)
JOINT_HOME = lambda: mj_abs(JOINT_HOME_POS)
Z_DOWN     = lambda: ml_rel(Z_40_DOWN)
Z_UP       = lambda: ml_rel(Z_40_UP)
J6_CW      = lambda: mj_rel([0, 0, 0, 0, 0,  90])
J6_CCW     = lambda: mj_rel([0, 0, 0, 0, 0, -90])


# ═══════════════════════════════════════════════════════════
# PickupCube 4단계 시퀀스
# (명세서 3.1 _pickup_cb + DRL Cube_recognition_process 동작 1~6)
# 오케스트레이터가 단계별로 호출 — 명령→동작→완료 패턴
# ═══════════════════════════════════════════════════════════

def pickup_step_seq(step: str,
                    cube_x: float = None,
                    cube_y: float = None) -> list:
    """
    PickupCube 단계별 시퀀스.
    step: "RELEASE_HOME" | "DESCEND" | "GRIP" | "LIFT"

    cube_x, cube_y: cube_perception이 전달한 XY 좌표 (mm 단위)
      - None → ZIG Home 기준 임시 픽업 (테스트용)
      - 값 있음 → 워크스테이션 랜덤 위치 픽업

    DRL 동작 1~6 분배:
      RELEASE_HOME → DRL 동작 1(gripper_release) + 동작 2(movej Home)
      DESCEND      →
        [ZIG 모드]  DRL 동작 3(movel ZIG) + 동작 4(movel Z-30mm)
        [랜덤 모드] cube_pose XY + INITIAL_STATE Z로 이동
                   → WORKSTATION_GRIP_Z(13.71mm)까지 수직 하강
      GRIP         → DRL 동작 5(gripper_grap)
      LIFT         → DRL 동작 6(movel Z+40mm)
    """
    if step == 'RELEASE_HOME':
        # DRL 동작 1~2: 그리퍼 개방 + Home 자세
        return [
            GRIP_OPEN(),
            JOINT_HOME(),
        ]
    elif step == 'DESCEND':
        if cube_x is not None and cube_y is not None:
            # 워크스테이션 랜덤 위치 픽업
            # Step 1: XY=랜덤, Z=INITIAL_STATE 높이(74.660mm)로 수평 이동
            # Step 2: Z를 WORKSTATION_GRIP_Z(13.71mm)까지 수직 하강
            #         하강량 = -(74.660 - 13.71) = -60.95mm
            approach_z  = INITIAL_STATE[2]                      # 74.660mm
            descend_rel = -(approach_z - WORKSTATION_GRIP_Z)    # -60.95mm
            orient      = INITIAL_STATE[3:]                     # rx,ry,rz 유지
            return [
                ml_abs([cube_x, cube_y, approach_z] + orient),
                ml_rel([0, 0, descend_rel, 0, 0, 0]),
            ]
        else:
            # ZIG Home 기준 임시 픽업 (cube_perception 연동 전 테스트용)
            return [
                ZIG_HOME(),
                ml_rel(Z_30_DOWN),
            ]
    elif step == 'GRIP':
        # DRL 동작 5: 큐브 파지
        return [
            GRIP_CUBE(),
        ]
    elif step == 'LIFT':
        # DRL 동작 6: Z +40mm 상승
        return [
            ml_rel(Z_40_UP),
        ]
    else:
        raise ValueError(f'알 수 없는 PickupCube step: {step!r}')


# ═══════════════════════════════════════════════════════════
# RotateCubeForFace 면별 시퀀스
# (명세서 3.1 _rotate_face_cb + DRL Cube_recognition_process 동작 7~13)
# 오케스트레이터가 면별로 호출 — 명령→동작→완료 패턴
# ═══════════════════════════════════════════════════════════

def rotate_face_seq(next_face: str) -> list:
    """
    RotateCubeForFace 면별 시퀀스.
    next_face: "B" | "R" | "F" | "L" | "D" | "B_AGAIN"

    DRL 동작 7~13 분배:
      B       → DRL 동작 7:  movel Cube_scan_B (카메라 앞 초기 이동)
      R       → DRL 동작 8:  movej J6 -90° (R면 스캔)
      F       → DRL 동작 9:  movej J6 -90° (F면 스캔)
      L       → DRL 동작 10: movej J6 -90° (L면 스캔)
      D       → DRL 동작 11~12: movej J6 +270° 복귀 + movel Cube_scan_D (D면 스캔)
      B_AGAIN → DRL 동작 13: movel Cube_scan_B 재이동 (B면 재확인)
    """
    if next_face == 'B':
        # DRL 동작 7: 카메라 앞 Cube_scan_B 위치로 이동
        return [
            ml_abs(CUBE_SCAN_B, vel=VEL_X_DRL, acc=ACC_X_DRL),
        ]
    elif next_face == 'R':
        # DRL 동작 8: J6 -90° (R면)
        return [
            mj_rel([0, 0, 0, 0, 0, -90], vel=VEL_J_DRL, acc=ACC_J_DRL),
        ]
    elif next_face == 'F':
        # DRL 동작 9: J6 -90° (F면)
        return [
            mj_rel([0, 0, 0, 0, 0, -90], vel=VEL_J_DRL, acc=ACC_J_DRL),
        ]
    elif next_face == 'L':
        # DRL 동작 10: J6 -90° (L면)
        return [
            mj_rel([0, 0, 0, 0, 0, -90], vel=VEL_J_DRL, acc=ACC_J_DRL),
        ]
    elif next_face == 'D':
        # DRL 동작 11~12: J6 +270° 복귀 + Cube_scan_D 이동 (D면)
        return [
            mj_rel([0, 0, 0, 0, 0, 270], vel=VEL_J_DRL, acc=ACC_J_DRL),
            ml_abs(CUBE_SCAN_D, vel=VEL_X_DRL, acc=ACC_X_DRL),
        ]
    elif next_face == 'B_AGAIN':
        # DRL 동작 13: Cube_scan_B 재이동 (B면 재확인)
        return [
            ml_abs(CUBE_SCAN_B, vel=VEL_X_DRL, acc=ACC_X_DRL),
        ]
    else:
        raise ValueError(f'알 수 없는 face: {next_face!r}')


# ═══════════════════════════════════════════════════════════
# PlaceOnJig 2단계 시퀀스
# (명세서 3.1 _place_cb + DRL Cube_recognition_process 동작 14~16)
# 오케스트레이터가 단계별로 호출 — 명령→동작→완료 패턴
# ═══════════════════════════════════════════════════════════

def place_step_seq(step: str) -> list:
    """
    PlaceOnJig 단계별 시퀀스.
    step: "APPROACH" | "RELEASE"

    DRL 동작 14~16 분배:
      APPROACH → DRL 동작 14~15: movel ZIG + movel Z-30mm
      RELEASE  → DRL 동작 16: gripper_release
    """
    if step == 'APPROACH':
        # DRL 동작 14~15: ZIG 위치 복귀 + Z -30mm 하강
        return [
            ml_abs(INITIAL_STATE, vel=VEL_X_DRL, acc=ACC_X_DRL),
            ml_rel(Z_30_DOWN,     vel=VEL_X_DRL, acc=ACC_X_DRL),
        ]
    elif step == 'RELEASE':
        # DRL 동작 16: 그리퍼 개방 (큐브 내려놓기)
        return [
            GRIP_OPEN(),
        ]
    else:
        raise ValueError(f'알 수 없는 PlaceOnJig step: {step!r}')


# ═══════════════════════════════════════════════════════════
# 임시 픽업 통합 시퀀스 (cube_perception 없는 환경용)
# ZIG 위치에서 픽업 전 과정을 한번에 실행
# ※ cube_perception 연동 완료 후 pickup_step_seq()로 교체 예정
# ═══════════════════════════════════════════════════════════

def pickup_from_jig_seq() -> list:
    """
    ZIG에서 임시 픽업 전체 시퀀스.
    pickup_step_seq 4단계를 순서대로 합친 편의 함수.
    """
    return (
        pickup_step_seq('RELEASE_HOME') +
        pickup_step_seq('DESCEND') +
        pickup_step_seq('GRIP') +
        pickup_step_seq('LIFT')
    )


# ═══════════════════════════════════════════════════════════
# 19개 솔빙 토큰 시퀀스 (명세서 3.2)
# ═══════════════════════════════════════════════════════════

def _u_seq() -> list:
    return [
        GRIP_OPEN(), ZIG_HOME(), ml_rel(Z_30_DOWN),
        GRIP_CUBE(), J6_CW(),
        GRIP_OPEN(), ml_rel(Z_40_UP), ZIG_HOME(),
    ]

def _u_prime_seq() -> list:
    return [
        GRIP_OPEN(), ZIG_HOME(), ml_rel(Z_30_DOWN),
        GRIP_CUBE(), J6_CCW(),
        GRIP_OPEN(), ml_rel(Z_40_UP), ZIG_HOME(),
    ]

def _u2_seq() -> list:
    return [
        GRIP_OPEN(), ZIG_HOME(), ml_rel(Z_30_DOWN),
        GRIP_CUBE(), J6_CW(), J6_CW(),
        GRIP_OPEN(), ml_rel(Z_40_UP),
        J6_CCW(), J6_CCW(), ZIG_HOME(),
    ]

def _d_seq() -> list:
    return [
        GRIP_OPEN(), ZIG_HOME(),
        ml_abs(READY_D_LOW), GRIP_CUBE(), ml_abs(READY_D_UP),
        J6_CW(), ml_rel(Z_40_UP), J6_CCW(), ml_rel(Z_40_DOWN),
        GRIP_OPEN(), ZIG_HOME(),
    ]

def _d_prime_seq() -> list:
    return [
        GRIP_OPEN(), ZIG_HOME(),
        ml_abs(READY_D_LOW), GRIP_CUBE(), ml_abs(READY_D_UP),
        J6_CCW(), ml_rel(Z_40_UP), J6_CW(), ml_rel(Z_40_DOWN),
        GRIP_OPEN(), ZIG_HOME(),
    ]

def _d2_seq() -> list:
    return [
        GRIP_OPEN(), ZIG_HOME(),
        ml_abs(READY_D_LOW), GRIP_CUBE(), ml_abs(READY_D_UP),
        J6_CW(), J6_CW(), ml_rel(Z_40_UP),
        J6_CCW(), J6_CCW(), ml_rel(Z_40_DOWN),
        GRIP_OPEN(), ZIG_HOME(),
    ]

def _r_seq() -> list:
    return [
        GRIP_OPEN(), ZIG_HOME(), ml_rel(Z_40_DOWN),
        GRIP_CUBE(), ZIG_HOME(), ml_abs(R_DROP_POS),
        GRIP_OPEN(), ZIG_HOME(), ml_rel(Z_40_DOWN),
        GRIP_CUBE(), J6_CW(),
        GRIP_OPEN(), ZIG_HOME(), ml_rel(Z_40_DOWN),
        GRIP_CUBE(), ZIG_HOME(), ml_abs(L_DROP_POS),
        GRIP_OPEN(), ZIG_HOME(),
    ]

def _r_prime_seq() -> list:
    return [
        GRIP_OPEN(), ZIG_HOME(), ml_rel(Z_40_DOWN),
        GRIP_CUBE(), ZIG_HOME(), ml_abs(R_DROP_POS),
        GRIP_OPEN(), ZIG_HOME(), ml_rel(Z_40_DOWN),
        GRIP_CUBE(), J6_CCW(),
        GRIP_OPEN(), ZIG_HOME(), ml_rel(Z_40_DOWN),
        GRIP_CUBE(), ZIG_HOME(), ml_abs(L_DROP_POS),
        GRIP_OPEN(), ZIG_HOME(),
    ]

def _r2_seq() -> list:
    return [
        GRIP_OPEN(), ZIG_HOME(), ml_rel(Z_40_DOWN),
        GRIP_CUBE(), ZIG_HOME(), ml_abs(R_DROP_POS),
        GRIP_OPEN(), ZIG_HOME(), ml_rel(Z_40_DOWN),
        GRIP_CUBE(), J6_CW(), J6_CW(),
        GRIP_OPEN(), ml_rel(Z_40_UP),
        J6_CCW(), J6_CCW(), ml_rel(Z_40_DOWN),
        GRIP_CUBE(), ZIG_HOME(), ml_abs(L_DROP_POS),
        GRIP_OPEN(), ZIG_HOME(),
    ]

def _l_seq() -> list:
    return [
        GRIP_OPEN(), ZIG_HOME(), ml_rel(Z_40_DOWN),
        GRIP_CUBE(), ml_rel(Z_40_UP), ml_abs(L_DROP_POS),
        GRIP_OPEN(), ZIG_HOME(), ml_rel(Z_40_DOWN),
        GRIP_CUBE(), J6_CW(),
        GRIP_OPEN(), ml_rel(Z_40_UP), ZIG_HOME(), ml_rel(Z_40_DOWN),
        GRIP_CUBE(), ZIG_HOME(), ml_abs(R_DROP_POS),
        GRIP_OPEN(), ZIG_HOME(),
    ]

def _l_prime_seq() -> list:
    return [
        GRIP_OPEN(), ZIG_HOME(), ml_rel(Z_40_DOWN),
        GRIP_CUBE(), ZIG_HOME(), ml_abs(L_DROP_POS),
        GRIP_OPEN(), ZIG_HOME(), ml_rel(Z_40_DOWN),
        GRIP_CUBE(), J6_CCW(),
        GRIP_OPEN(), ZIG_HOME(), ml_rel(Z_40_DOWN),
        GRIP_CUBE(), ZIG_HOME(), ml_abs(R_DROP_POS),
        GRIP_OPEN(), ZIG_HOME(),
    ]

def _l2_seq() -> list:
    return [
        GRIP_OPEN(), ZIG_HOME(), ml_rel(Z_40_DOWN),
        GRIP_CUBE(), ZIG_HOME(), ml_abs(L_DROP_POS),
        GRIP_OPEN(), ZIG_HOME(), ml_rel(Z_40_DOWN),
        GRIP_CUBE(), J6_CW(), J6_CW(),
        GRIP_OPEN(), ml_rel(Z_40_UP),
        J6_CCW(), J6_CCW(), ml_rel(Z_40_DOWN),
        GRIP_CUBE(), ZIG_HOME(), ml_abs(R_DROP_POS),
        GRIP_OPEN(), ZIG_HOME(),
    ]

def _f_seq() -> list:
    return [
        GRIP_OPEN(), ZIG_HOME(), J6_CW(), ml_rel(Z_40_DOWN),
        GRIP_CUBE(), ml_rel(Z_40_UP), ZIG_HOME(), ml_abs(R_DROP_POS),
        GRIP_OPEN(), ZIG_HOME(), ml_rel(Z_40_DOWN),
        GRIP_CUBE(), J6_CW(),
        GRIP_OPEN(), ZIG_HOME(), ml_abs(R_DROP_POS),
        GRIP_CUBE(), ml_rel(Z_40_UP), ZIG_HOME(), J6_CW(),
        ml_rel(X17_Z15_NEG), ml_rel(Z_10_DOWN),
        GRIP_OPEN(), ZIG_HOME(),
    ]

def _f_prime_seq() -> list:
    return [
        GRIP_OPEN(), ZIG_HOME(), J6_CW(), ml_rel(Z_40_DOWN),
        GRIP_CUBE(), ml_rel(Z_40_UP), ZIG_HOME(), ml_abs(R_DROP_POS),
        GRIP_OPEN(), ZIG_HOME(), ml_rel(Z_40_DOWN),
        GRIP_CUBE(), J6_CCW(),
        GRIP_OPEN(), ml_rel(Z_40_UP), ZIG_HOME(), ml_abs(R_DROP_POS),
        GRIP_CUBE(), ml_rel(Z_40_UP), ZIG_HOME(), J6_CW(),
        ml_rel(X17_Z15_NEG), ml_rel(Z_10_DOWN),
        GRIP_OPEN(), ZIG_HOME(),
    ]

def _f2_seq() -> list:
    return [
        GRIP_OPEN(), ZIG_HOME(), J6_CW(), ml_rel(Z_40_DOWN),
        GRIP_CUBE(), ml_rel(Z_40_UP), ZIG_HOME(), ml_abs(R_DROP_POS),
        GRIP_OPEN(), ZIG_HOME(), ml_rel(Z_40_DOWN),
        GRIP_CUBE(), J6_CW(), J6_CW(),
        GRIP_OPEN(), ml_rel(Z_40_UP), J6_CCW(), J6_CCW(), ZIG_HOME(),
        ml_abs(R_DROP_POS), GRIP_CUBE(), ml_rel(Z_40_UP), ZIG_HOME(), J6_CW(),
        ml_rel(X17_Z15_NEG), ml_rel(Z_10_DOWN),
        GRIP_OPEN(), ZIG_HOME(),
    ]

def _b_seq() -> list:
    return [
        GRIP_OPEN(), ZIG_HOME(), J6_CW(), ml_rel(Z_40_DOWN),
        GRIP_CUBE(), ml_rel(Z_40_UP), ZIG_HOME(), ml_abs(L_DROP_POS),
        GRIP_OPEN(), ZIG_HOME(), ml_rel(Z_40_DOWN),
        GRIP_CUBE(), J6_CW(),
        GRIP_OPEN(), ml_rel(Z_40_UP), ZIG_HOME(), ml_abs(L_DROP_POS),
        GRIP_CUBE(), ml_rel(Z_40_UP), ZIG_HOME(), J6_CW(),
        ml_rel(X17_Z15_POS), ml_rel(Z_10_DOWN),
        GRIP_OPEN(), ZIG_HOME(),
    ]

def _b_prime_seq() -> list:
    return [
        GRIP_OPEN(), ZIG_HOME(), J6_CW(), ml_rel(Z_40_DOWN),
        GRIP_CUBE(), ml_rel(Z_40_UP), ZIG_HOME(), ml_abs(L_DROP_POS),
        GRIP_OPEN(), ZIG_HOME(), ml_rel(Z_40_DOWN),
        GRIP_CUBE(), J6_CCW(),
        GRIP_OPEN(), ml_rel(Z_40_UP), ZIG_HOME(), ml_abs(L_DROP_POS),
        GRIP_CUBE(), ml_rel(Z_40_UP), ZIG_HOME(), J6_CW(),
        ml_rel(X17_Z15_POS), ml_rel(Z_10_DOWN),
        GRIP_OPEN(), ZIG_HOME(),
    ]

def _b2_seq() -> list:
    return [
        GRIP_OPEN(), ZIG_HOME(), J6_CW(), ml_rel(Z_40_DOWN),
        GRIP_CUBE(), ml_rel(Z_40_UP), ZIG_HOME(), ml_abs(L_DROP_POS),
        GRIP_OPEN(), ZIG_HOME(), ml_rel(Z_40_DOWN),
        GRIP_CUBE(), J6_CW(), J6_CW(),
        GRIP_OPEN(), ml_rel(Z_40_UP), J6_CCW(), J6_CCW(),
        ml_abs(L_DROP_POS), GRIP_CUBE(), ml_rel(Z_40_UP), ZIG_HOME(), J6_CW(),
        ml_rel(X17_Z15_POS), ml_rel(Z_10_DOWN),
        GRIP_OPEN(), ZIG_HOME(),
    ]

def _perception_seq() -> list:
    return (
        pickup_from_jig_seq() +
        [ml_abs(CUBE_SCAN_B, vel=VEL_X_DRL, acc=ACC_X_DRL)] +
        rotate_face_seq('R') + rotate_face_seq('F') + rotate_face_seq('L') +
        rotate_face_seq('D') + rotate_face_seq('B_AGAIN') +
        place_step_seq('APPROACH') + place_step_seq('RELEASE')
    )


# ═══════════════════════════════════════════════════════════
# 토큰 → 시퀀스 맵 (공개 API)
# ═══════════════════════════════════════════════════════════

_TOKEN_MAP = {
    'U'  : _u_seq,  "U'": _u_prime_seq, 'U2': _u2_seq,
    'D'  : _d_seq,  "D'": _d_prime_seq, 'D2': _d2_seq,
    'R'  : _r_seq,  "R'": _r_prime_seq, 'R2': _r2_seq,
    'L'  : _l_seq,  "L'": _l_prime_seq, 'L2': _l2_seq,
    'F'  : _f_seq,  "F'": _f_prime_seq, 'F2': _f2_seq,
    'B'  : _b_seq,  "B'": _b_prime_seq, 'B2': _b2_seq,
    'P'  : _perception_seq,
}

VALID_TOKENS: list = list(_TOKEN_MAP.keys())

SCAN_FACES: list = ['B', 'R', 'F', 'L', 'D', 'B_AGAIN']
PICKUP_STEPS: list = ['RELEASE_HOME', 'DESCEND', 'GRIP', 'LIFT']
PLACE_STEPS: list = ['APPROACH', 'RELEASE']


def token_sequence(token: str) -> list:
    """토큰 문자열 → Step 리스트 반환 (명세서 3.2)."""
    fn = _TOKEN_MAP.get(token)
    if fn is None:
        raise ValueError(f'알 수 없는 토큰: {token!r}')
    return fn()


def face_transition_sequence(from_face: str, to_face: str) -> list:
    """면 전환 룩업테이블 (Phase 3 구현 예정)."""
    return []
