# Cube Solver with Doosan E0509

> Doosan E0509 + RH-P12-RN(A) + RealSense D455 기반 루빅스 큐브 자동 솔버 (ROS2 Humble)

작업공간 임의 위치에 놓인 큐브를 카메라로 감지 → 픽업 → 6면 색상 인식 → kociemba 솔빙 → 모션 실행까지 end-to-end로 수행하는 ROS2 시스템입니다. 단일 패키지에 얽혀있던 기존 구조(`cube-solver-with-DoosanE0509-ver1`)를 5개 패키지 + 1개 통합 인터페이스 패키지로 재구성하는 중입니다.

> ⚠️ **현재 상태**: 설계 단계. 구현은 진행 중이며 일부 노드만 동작합니다.

---

## 저장소 받기

```bash
git clone <이 저장소 URL> cube_solver_ver2_ws
cd cube_solver_ver2_ws
```

Doosan 드라이버는 별도 클론 (`humble` 브랜치):

```bash
git clone -b humble https://github.com/doosan-robotics/doosan-robot2.git src/doosan-robot2
```

## 하드웨어

| 구성 요소 | 모델 | 비고 |
|---|---|---|
| 로봇 암 | Doosan Robotics **E0509** | 6-DOF, payload 5kg, reach 900mm |
| 그리퍼 | ROBOTIS **RH-P12-RN(A)** | Modbus RTU, stroke 0~109mm |
| 카메라 | Intel RealSense **D455** | RGB + Depth |

## 개발 환경

- Ubuntu 22.04
- ROS2 Humble
- MoveIt2
- Python 3.10
- 의존 패키지: `doosan-robot2`, `realsense-ros`, `kociemba`

---

## 시스템 구조

5개 기능 패키지 + 1개 인터페이스 패키지 + 1개 외부 드라이버로 구성됩니다.

```
┌─────────────────────────────────────────────────────────┐
│                   cube_orchestrator                      │
│              FSM + kociemba + 토큰 디스패치              │
└──────┬───────────────┬──────────────────────┬───────────┘
       │ srv           │ action               │ topic
       ▼               ▼                      ▼
┌──────────────┐ ┌──────────────────┐ ┌──────────────────┐
│cube_perception│ │cube_robot_action │ │/orchestrator/state│
│              │ │                  │ └──────────────────┘
│ ・vla_detect │ │ ・ExecuteToken   │
│ ・color_ext  │ │ ・PickupCube     │
└──────┬───────┘ │ ・PlaceOnJig     │
       │ RealSense│ ・GoHome        │
       │         │ ・RotateForFace  │
       │         └────┬──────────┬──┘
       │              │ action   │ action
       │              ▼          ▼
       │      ┌──────────────┐ ┌──────────────────┐
       │      │ doosan-robot2│ │rh_p12_rna_       │
       │      │ (재사용)     │ │  controller      │
       │      │ Movej/Movel  │ │ GripperCommand   │
       │      └──────────────┘ └──────────────────┘
       │
       ▼
   D455 카메라
```

### 패키지 구성

| 패키지 | 책임 | 핵심 인터페이스 |
|---|---|---|
| `cube_perception` | 카메라 I/O, 큐브 3D pose 감지, 면 색상 추출 | srv: `DetectCubePose`, `ExtractFace` |
| `cube_robot_action` | 팔+그리퍼 합성 동작 액션 서버 (단일 노드, 다중 액션) | action: `ExecuteSolveToken`, `PickupCube`, `PlaceOnJig`, `GoHome`, `RotateCubeForFace` |
| `cube_orchestrator` | FSM + kociemba + 토큰 디스패치 | srv: `StartRun` / topic: `/orchestrator/state` |
| `rh_p12_rna_controller` | Modbus 기반 그리퍼 액션 서버 (stroke polling feedback) | action: `GripperCommand` |
| `cube_interfaces` | 위 4개 패키지의 srv / action 정의 통합 관리 | (정의만 보유) |
| `doosan-robot2` | 로봇 암 드라이버 (수정 없이 재사용) | action: `MovejH2r`, `MovelH2r` |

---

## 워크스페이스 디렉토리 구조

```
ros2_ws/src/
├── cube_interfaces/                    # 통합 인터페이스 패키지
│   ├── package.xml
│   ├── CMakeLists.txt
│   ├── cube_perception/srv/
│   │   ├── DetectCubePose.srv
│   │   └── ExtractFace.srv
│   ├── cube_robot_action/action/
│   │   ├── ExecuteSolveToken.action
│   │   ├── PickupCube.action
│   │   ├── PlaceOnJig.action
│   │   ├── GoHome.action
│   │   └── RotateCubeForFace.action
│   ├── cube_orchestrator/srv/
│   │   └── StartRun.srv
│   └── rh_p12_rna_controller/action/
│       └── GripperCommand.action
│
├── cube_perception/
│   └── cube_perception/
│       ├── vla_detection_node.py
│       ├── color_extraction_node.py
│       └── detection/
│           ├── base.py                 # CubeDetector(ABC)
│           ├── opencv_depth_backend.py
│           └── gemini_vla_backend.py
│
├── cube_robot_action/
│   └── cube_robot_action/
│       ├── robot_action_server_node.py
│       ├── motion_library.py           # 12 토큰 Step 시퀀스
│       ├── arm_client.py               # MovejH2r/MovelH2r 래퍼
│       └── gripper_client.py           # GripperCommand 래퍼
│
├── cube_orchestrator/
│   └── cube_orchestrator/
│       └── master_orchestrator_node.py # FSM + kociemba
│
├── rh_p12_rna_controller/
│   └── rh_p12_rna_controller/
│       └── gripper_node.py             # Modbus FC03/FC06
│
└── doosan-robot2/                       # 외부 드라이버 (수정 없음)
```

---

## 인터페이스 (Interface Contracts)

모든 srv / action 정의는 **`cube_interfaces`** 패키지에 통합되어 있습니다.

### Services

| 서비스 | 패키지 | Request → Response |
|---|---|---|
| `DetectCubePose` | cube_perception | `string hint` → `PoseStamped pose, float32 confidence, bool success, string message` |
| `ExtractFace` | cube_perception | `string face` → `string colors_9, bool success, string message` |
| `StartRun` | cube_orchestrator | `bool skip_pickup` → `bool success, string message` |

### Actions

| 액션 | 패키지 | Goal / Result / Feedback |
|---|---|---|
| `ExecuteSolveToken` | cube_robot_action | Goal: `string token` / Result: `bool success, string message` / Feedback: `string stage, float32 progress` |
| `PickupCube` | cube_robot_action | Goal: `PoseStamped cube_pose` / Result: `bool success, string message` / Feedback: `string stage` |
| `PlaceOnJig` | cube_robot_action | Goal: 없음 / Result: `bool success, string message` / Feedback: `string stage` |
| `GoHome` | cube_robot_action | Goal: 없음 / Result: `bool success, string message` |
| `RotateCubeForFace` | cube_robot_action | Goal: `string next_face` / Result: `bool success` / Feedback: `string stage` |
| `GripperCommand` | rh_p12_rna_controller | Goal: `int32 target_pulse, float32 timeout_sec` / Result: `bool success, int32 final_stroke, string message` / Feedback: `int32 current_stroke, int32 target_pulse, float32 elapsed` |

### Topics

| 토픽 | 타입 | 발행 주체 | 용도 |
|---|---|---|---|
| `/orchestrator/state` | std_msgs/String | cube_orchestrator | FSM 상태 문자열 |
| `/cube/state_raw` | std_msgs/String | color_extraction_node | 6면 54자 큐브 상태 (디버그) |
| `~/debug/image` | sensor_msgs/Image | vla_detection_node | 어노테이션된 RGB (디버그) |

---

## FSM 상태 흐름

`cube_orchestrator`의 FSM은 다음 순서로 동작합니다.

```
IDLE
 │ (StartRun 수신)
 ▼
DETECT_CUBE ──→ PICKUP ──→ PERCEIVE_FACE(U/R/F/D/L/B 6회 루프)
                                │
                                ▼
                            SOLVE (kociemba)
                                │
                                ▼
                          PLACE_ON_JIG
                                │
                                ▼
                       EXECUTE_TOKEN_i (loop)
                                │
                                ▼
                            GO_HOME ──→ DONE ──→ IDLE
                                
   ※ 임의 액션 실패 시: FAULT (cancel + INITIAL_STATE 복귀) → IDLE
```

`skip_pickup=true`로 호출하면 `IDLE → PERCEIVE_FACE(U)`로 직행하여 기존(지그에 큐브 미리 놓은) 플로우만 동작합니다.

---

## 모션 전략

기존 DRL chunk 재생 방식을 폐기하고 Python 모션 라이브러리로 전환했습니다.

- **`motion_library.py`** — 12개 솔빙 토큰(U/U'/U2, R/..., F/..., L/..., B/..., D/...) 별 `Step` 시퀀스 정의. ROS2 의존성 없는 순수 Python 라이브러리.
- **`Step` 타입** — `MOVE_L_ABS` | `MOVE_L_REL` | `MOVE_J_REL` | `GRIP` | `WAIT` 5종.
- **웨이포인트 상수** — `INITIAL_STATE [400, 0, 250, 0, 0, 0]`, `Z_40MM_DOWN/UP`, `L_DROP_POS`, `R_DROP_POS` 등을 기존 `cube_master_node.py`에서 그대로 포팅.
- **실행** — `ExecuteSolveToken` 액션 서버가 시퀀스를 순회하며 각 Step을 `ArmClient` / `GripperClient`로 await 직렬 호출. fault 시 sub-goal cancel + INITIAL_STATE 복귀.

---

## Perception 백엔드 추상화

`cube_perception/detection/`에 추상 베이스 클래스를 두고 백엔드를 교체 가능하게 구성합니다.

```python
class CubeDetector(ABC):
    @abstractmethod
    def detect(
        self,
        rgb:   np.ndarray,   # H x W x 3, uint8
        depth: np.ndarray,   # H x W, float32 (meters)
        K:     np.ndarray,   # 3x3 intrinsic matrix
    ) -> tuple[PoseStamped, float]:   # (pose, confidence)
        ...
```

런치 파라미터 `detector_backend`로 `"opencv"` 또는 `"gemini"` 중 선택합니다. 백엔드별 파라미터는 `~detector.<name>.*` 네임스페이스를 사용합니다.

| 백엔드 | 클래스 | 특징 |
|---|---|---|
| `opencv` | `OpenCVDepthDetector` | 로컬 CV, 저지연 |
| `gemini` | `GeminiVLADetector` | VLA 기반, 1~3초/호출 (DETECT_CUBE 1회만 호출) |

---

## 그리퍼 제어 (RH-P12-RN(A))

Modbus RTU 액션 서버로 통합되어 있습니다.

- **Goal** — 목표 펄스(0~500) + timeout
- **Feedback** — FC03 레지스터 10Hz 폴링으로 현재 stroke 발행
- **Cancel** — `target = current` 설정으로 즉시 정지
- **수렴 판정** — `|current_stroke - target_pulse| ≤ 10`

PULSE 상수는 모든 패키지가 공유 YAML로 참조합니다.

```yaml
PULSE_OPEN:    200   # 그리퍼 개방
PULSE_CUBE:    420   # 큐브 파지
PULSE_ROTATE:  420   # 큐브 회전 중 유지
PULSE_REPOSE:  420   # 재파지
```

기존 Trigger 서비스 4종(`/gripper/release` 등)은 폐기되었습니다.

---

## 네임스페이스 규칙

| 패키지 | 네임스페이스 | 예시 |
|---|---|---|
| cube_perception | `/cube/` | `/cube/state_raw` |
| cube_orchestrator | `/orchestrator/` | `/orchestrator/state`, `/orchestrator/start_run` |
| 서비스 / 액션 이름 | `<패키지명>/<snake_case>` | `cube_perception/extract_face` |

---

## 빌드 & 실행

> ⚠️ 구현 진행 중. 현재 빌드 가능한 패키지는 `rh_p12_rna_controller` 뿐입니다.

```bash
# 0. ROS2 Humble 등록
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# 1. 인터페이스 패키지를 먼저 빌드
cd ~/ros2_ws
colcon build --packages-select cube_interfaces
source install/setup.bash

# 2. 나머지 패키지 빌드 (구현 후)
colcon build --packages-select \
    cube_perception cube_robot_action cube_orchestrator rh_p12_rna_controller
source install/setup.bash

# 3. 전체 스택 기동 (구현 완료 후)
ros2 launch cube_orchestrator full_stack.launch.py

# 4. 솔빙 트리거
ros2 service call /orchestrator/start_run cube_interfaces/srv/StartRun "{skip_pickup: false}"
```

---

## 참고 문서

- 상세 노드 명세서: `docs/cube_solver_node_spec.docx`

---

## License

Apache-2.0
