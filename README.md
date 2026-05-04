# Cube Solver with Doosan E0509

> Doosan E0509 + RH-P12-RN(A) + RealSense D455 기반 루빅스 큐브 자동 솔버 (ROS2 Humble)

작업공간 임의 위치에 놓인 큐브를 카메라로 감지 → 픽업 → 6면 색상 인식 → kociemba 솔빙 → 모션 실행까지 end-to-end로 수행하는 ROS2 시스템입니다. 기존 단일 패키지 구조(`cube-solver-with-DoosanE0509-ver1`)를 6개 패키지로 재구성했습니다.

> ℹ️ **현재 상태**: 핵심 패키지 구현 완료. 실기 통합 검증 진행 전 단계.

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
- Python 3.10
- 의존 패키지: `doosan-robot2`, `realsense-ros`, `kociemba` (`pip install kociemba`)

---

## 시스템 구조

6개 기능 패키지 + 1개 인터페이스 패키지 + 1개 외부 드라이버로 구성됩니다.

```
┌──────────────────────────────────────────────────────────────┐
│                       cube_webui                             │
│          FastAPI + WebSocket + MJPEG (포트 8080)             │
└──────────────────────────┬───────────────────────────────────┘
                           │ srv (StartScan / StartSolve / StartRun)
                           ▼
┌─────────────────────────────────────────────────────────────┐
│                   cube_orchestrator                          │
│     FSM (scan 분기 / solve 분기) + kociemba + 토큰 디스패치  │
└──────┬──────────────────┬──────────────────────┬────────────┘
       │ srv              │ action               │ topic
       ▼                  ▼                      ▼
┌──────────────┐  ┌──────────────────┐  ┌──────────────────┐
│cube_perception│  │cube_robot_action │  │/orchestrator/    │
│              │  │                  │  │  state           │
│ ・vla_detect │  │ ・ExecuteToken   │  │/orchestrator/    │
│ ・color_ext  │  │ ・PickupCube     │  │  solution        │
└──────┬───────┘  │ ・PlaceOnJig     │  └──────────────────┘
       │ RealSense│ ・GoHome         │
       │          │ ・RotateForFace  │
       │          └────┬─────────┬───┘
       │               │ srv     │ action
       │               ▼         ▼
       │      ┌──────────────┐ ┌──────────────────────────┐
       │      │ doosan-robot2│ │rh_p12_rna_controller     │
       │      │ (재사용)     │ │ + rh_p12_rna_controller_ │
       │      │ Movej/Movel  │ │   interfaces             │
       │      └──────────────┘ │ action: SafeGrasp        │
       │                       └──────────────────────────┘
       ▼
   D455 카메라
```

### 패키지 구성

| 패키지 | 책임 | 핵심 인터페이스 | 상태 |
|---|---|---|---|
| `cube_perception` | 카메라 I/O, 큐브 3D pose 감지, 면 색상 추출 | srv: `DetectCubePose`, `ExtractFace`, `GetCubeState` | ✅ 구현 완료 |
| `cube_robot_action` | 팔+그리퍼 합성 동작 액션 서버 (단일 노드, 다중 액션) | action: `ExecuteSolveToken`, `PickupCube`, `PlaceOnJig`, `GoHome`, `RotateCubeForFace` | ✅ 구현 완료 |
| `cube_orchestrator` | FSM + kociemba + 토큰 디스패치 | srv: `StartRun`, `StartScan`, `StartSolve` | ✅ 구현 완료 |
| `rh_p12_rna_controller` | Modbus 기반 그리퍼 액션 서버 | action: `SafeGrasp` | ✅ 구현 완료 |
| `rh_p12_rna_controller_interfaces` | 그리퍼 인터페이스 정의 | action: `SafeGrasp`, `GripperCommand` | ✅ |
| `cube_webui` | FastAPI 기반 웹 UI (카메라 스트림, scan/solve 버튼) | HTTP/WebSocket/MJPEG (포트 8080) | ✅ 구현 완료 |
| `doosan-robot2` | 로봇 암 드라이버 (수정 없이 재사용) | srv: `MoveLine`, `MoveJoint` | 외부 패키지 |

---

## 워크스페이스 디렉토리 구조

```
src/
├── cube_orchestrator/
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── cube_orchestrator/
│   │   └── master_orchestrator_node.py   # FSM + kociemba
│   ├── srv/
│   │   ├── StartRun.srv
│   │   ├── StartScan.srv
│   │   └── StartSolve.srv
│   ├── launch/
│   │   ├── full_stack.launch.py
│   │   └── orchestrator.launch.py
│   └── scripts/
│       └── master_orchestrator_node
│
├── cube_perception/
│   └── cube_perception/
│       ├── vla_detection_node.py
│       ├── color_extraction_node.py
│       └── detection/
│           ├── base.py                   # CubeDetector(ABC)
│           ├── opencv_depth_backend.py
│           └── gemini_vla_backend.py
│
├── cube_robot_action/
│   ├── cube_robot_action/
│   │   ├── robot_action_server_node.py
│   │   └── motion_library.py             # 18 토큰 Step 시퀀스
│   └── action/
│       ├── ExecuteSolveToken.action
│       ├── PickupCube.action
│       ├── PlaceOnJig.action
│       ├── GoHome.action
│       └── RotateCubeForFace.action
│
├── cube_webui/
│   ├── cube_webui/
│   │   ├── server.py                     # FastAPI + rclpy
│   │   └── static/
│   │       ├── index.html
│   │       └── app.js
│   └── launch/
│       └── webui.launch.py
│
├── rh_p12_rna_controller/
│   └── rh_p12_rna_controller/
│       └── gripper_node.py               # Modbus FC03/FC06
│
├── rh_p12_rna_controller_interfaces/
│   └── action/
│       ├── SafeGrasp.action
│       └── GripperCommand.action
│
└── doosan-robot2/                        # 외부 드라이버 (수정 없음)
```

---

## 인터페이스 (Interface Contracts)

### Services

| 서비스 | 패키지 | Request → Response |
|---|---|---|
| `DetectCubePose` | cube_perception | `string hint` → `PoseStamped pose, float32 confidence, bool success, string message` |
| `ExtractFace` | cube_perception | `string face` → `string colors_9, bool success, string message` |
| `GetCubeState` | cube_perception | (없음) → `string state_54, bool success, string message` |
| `StartRun` | cube_orchestrator | (없음) → `bool success, string message` |
| `StartScan` | cube_orchestrator | (없음) → `bool success, string message, string state_54, string solution` |
| `StartSolve` | cube_orchestrator | `string solution` → `bool success, string message` |

### Actions

| 액션 | 패키지 | Goal / Result / Feedback |
|---|---|---|
| `ExecuteSolveToken` | cube_robot_action | Goal: `string token` / Result: `bool success, string message` / Feedback: `string stage, float32 progress` |
| `PickupCube` | cube_robot_action | Goal: `string step, PoseStamped cube_pose` / Result: `bool success, string message` / Feedback: `string stage` |
| `PlaceOnJig` | cube_robot_action | Goal: `string step` / Result: `bool success, string message` / Feedback: `string stage` |
| `GoHome` | cube_robot_action | Goal: (없음) / Result: `bool success, string message` |
| `RotateCubeForFace` | cube_robot_action | Goal: `string next_face` / Result: `bool success` / Feedback: `string stage` |
| `SafeGrasp` | rh_p12_rna_controller_interfaces | Goal: `int32 target_position, int32 goal_current, int32 current_threshold, float32 timeout_sec` / Result: `bool success, bool grasped, int32 final_position, int32 final_current, string message` |

### Topics

| 토픽 | 타입 | 발행 주체 | 용도 |
|---|---|---|---|
| `/orchestrator/state` | std_msgs/String | cube_orchestrator | FSM 상태 문자열 |
| `/orchestrator/solution` | std_msgs/String | cube_orchestrator | 솔빙 결과 토큰 문자열 |
| `/cube/state_raw` | std_msgs/String | color_extraction_node | 6면 54자 큐브 상태 (디버그) |
| `~/debug/image` | sensor_msgs/Image | vla_detection_node | 어노테이션된 RGB (디버그) |

---

## FSM 상태 흐름

`cube_orchestrator`는 **scan**과 **solve** 두 분기로 나뉩니다. `StartRun`은 두 분기를 순차적으로 실행합니다.

### Scan 분기 (`StartScan`)

```
IDLE
 │ (StartScan 수신)
 ▼
DETECT_CUBE → PICKUP(×4 step) → ROTATE_FOR_FACE(B/R/F/L/D/B_AGAIN)
                                      │ 각 면 후 ExtractFace
                                      ▼
                                  SOLVE (kociemba, GetCubeState 1회)
                                      │
                                      ▼
                                   DONE → 결과 반환 (state_54 + solution)
```

### Solve 분기 (`StartSolve`)

```
IDLE
 │ (StartSolve 수신, solution 문자열 또는 캐시 사용)
 ▼
PLACE_ON_JIG(×2 step) → EXECUTE_TOKEN_i(loop) → GO_HOME → DONE
```

```
※ 임의 단계 실패 시: FAULT (INITIAL_STATE 복귀) → IDLE
```

---

## 모션 전략

- **`motion_library.py`** — 18개 솔빙 토큰(U/U'/U2, R/..., F/..., L/..., B/..., D/...) + `'P'`(정지) 별 `Step` 시퀀스 정의. ROS2 의존성 없는 순수 Python 라이브러리.
- **`Step` 타입** — `MOVE_L_ABS` | `MOVE_L_REL` | `MOVE_J_REL` | `GRIP` | `WAIT` 5종.
- **웨이포인트 상수** — `INITIAL_STATE`, `JOINT_HOME_POS`, `L_DROP_POS`, `R_DROP_POS`, `CUBE_SCAN_B/D` 등을 기존 DRL에서 포팅.
- **실행** — `_run_sequence()`가 각 Step을 `await`로 직렬 호출. GRIP step은 `/gripper/safe_grasp` (`SafeGrasp` 액션)을 실제 호출.
- **그리퍼 동기화** — `await send_goal_async` + `await get_result_async` 두 단계로 goal 수락 확인 → 결과 수신까지 동기 대기. OPEN: position-only 판정 (`current_threshold=9999`), CLOSE: 전류 기반 grasped 판정 (`current_threshold=300`).

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

현재 구현은 Gemini(2D 검출) + OpenCV(깊이 unprojection)를 합성하여 단일 pose를 생성합니다. 색상 분류는 Gemini에 위임합니다.

---

## Web UI

`cube_webui` 패키지는 FastAPI + uvicorn 기반 브라우저 인터페이스를 제공합니다.

| 엔드포인트 | 설명 |
|---|---|
| `GET /` | 메인 UI |
| `WS /ws/events` | FSM 상태 실시간 push (JSON) |
| `GET /stream/camera.mjpg` | MJPEG 카메라 스트림 |
| `POST /api/start_scan` | 큐브 감지 + 색상 인식 + 솔빙 (블로킹, 결과 반환) |
| `POST /api/start_solve` | 큐브 풀기 실행 (solution 문자열 또는 캐시 사용) |
| `POST /api/start_run` | scan + solve 연속 실행 |
| `POST /api/cancel` | 진행 중 작업 취소 |

Start Scan 완료 후 solution이 자동으로 폼에 채워지며, Start Solve 버튼으로 바로 실행할 수 있습니다.

---

## 그리퍼 제어 (RH-P12-RN(A))

Modbus RTU 기반 액션 서버로 통합되어 있습니다.

- **SafeGrasp** — 목표 위치(pulse) + 목표 전류 + 전류 임계값 + timeout
- **Feedback** — FC03 레지스터 10Hz 폴링으로 현재 위치/전류 발행
- **grasped 판정** — 전류가 `current_threshold`를 초과하면 물체 접촉으로 판정

PULSE 상수:

```python
PULSE_OPEN   = 200   # 그리퍼 개방
PULSE_CUBE   = 420   # 큐브 파지
PULSE_ROTATE = 420   # 큐브 회전 중 유지
PULSE_REPOSE = 420   # 재파지
```

---

## 빌드 & 실행

```bash
# 0. ROS2 Humble 소스
source /opt/ros/humble/setup.bash

# 1. pip 의존성
pip install kociemba "uvicorn[standard]"

# 2. 빌드 (인터페이스 패키지 우선)
colcon build --packages-select rh_p12_rna_controller_interfaces
colcon build --packages-select \
    cube_perception cube_robot_action cube_orchestrator \
    rh_p12_rna_controller cube_webui
source install/setup.bash

# 3. 전체 스택 기동
ros2 launch cube_orchestrator full_stack.launch.py

# 4a. Scan (감지 + 색상 인식 + 솔빙)
ros2 service call /orchestrator/start_scan cube_orchestrator/srv/StartScan

# 4b. Solve (큐브 풀기, 캐시 또는 직접 solution 입력)
ros2 service call /orchestrator/start_solve cube_orchestrator/srv/StartSolve "{solution: 'U R F2 ...'}"

# 4c. Run (scan + solve 연속)
ros2 service call /orchestrator/start_run cube_orchestrator/srv/StartRun

# 5. Web UI
ros2 launch cube_webui webui.launch.py   # http://localhost:8080
```

---

## 알려진 제한사항 및 TODO

| 항목 | 내용 |
|---|---|
| 그리퍼 전류 임계값 | GRIP_THRESHOLD_CLOSE=300mA, GRIP_GOAL_CURRENT=500mA — 실기 튜닝 필요 |
| `face_transition_sequence()` | 미구현 (빈 리스트 반환) — 면 전환 룩업테이블 채우기 필요 |
| `time.sleep` in async | `_movel`/`_movej` 코루틴 내 `time.sleep(0.3)` → `asyncio.sleep`으로 교체 권장 |
| `_call_action` timeout | `wait_for_server`·`send_goal`·`get_result` 세 단계가 동일 `timeout_sec` 사용 → 최악 3× 대기. 단계별 timeout 분리 권장 (`wait_for_server=5s`, `send_goal=5s`, `get_result=timeout_sec`) |
| `result.message` 미설정 | `_rotate_face_cb` 실패 경로에서 `result.message` 미설정 → 로그에 빈 문자열 출력 |

---

## 참고 문서

- 상세 노드 명세서: `docs/cube_solver_node_spec.docx`
- 모션/분배 설명서: `docs/cube_robot_action_분배_설명서.docx`

---

## License

Apache-2.0
