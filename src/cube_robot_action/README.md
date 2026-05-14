# cube_robot_action

Doosan E0509 + RH-P12-RN(A) 조합으로 큐브 솔버 시나리오의 **모션 합성 액션 서버**를
한 노드(`robot_action_server_node`)에 모아 제공한다. ROS2 의존성이 없는 순수 Python
모션 라이브러리(`motion_library.py`)와, 그 라이브러리를 액션으로 노출하는 노드 두 파일이 핵심.

## 액션 5종 (모두 `cube_interfaces.action.*`)

> 액션 이름은 단독 실행/통합 런치 모두 동일하게 `/robot/...` — `robot_action_server_node`가 직접 등록하며 remap이 없다.

| 액션 이름 | Goal | 비고 |
|---|---|---|
| `/robot/execute_solve_token` | `string token` | 18개 솔빙 토큰(U, U', U2, R, …) + `'P'`(정지). 각 토큰 시퀀스는 자체적으로 `ZIG_HOME`에서 시작/종료하므로 토큰마다 별도 `JOINT_HOME` 경유 없음(orchestrator가 솔빙 시작 직전에 1회만 `GoHome` 호출). |
| `/robot/pickup_cube` | `string step, PoseStamped cube_pose` | step ∈ `RELEASE_HOME` / `DESCEND` / `GRIP` / `LIFT` |
| `/robot/place_on_jig` | `string step` | step ∈ `APPROACH` / `RELEASE` |
| `/robot/go_home` | (없음) | `JOINT_HOME` 1 step 복귀 (이전 `[JOINT_HOME, ZIG_HOME, JOINT_HOME]` 3 step에서 중복 제거됨) |
| `/robot/rotate_cube_for_face` | `string next_face` | next_face ∈ `B` / `R` / `F` / `L` / `D` / `B_AGAIN` |

> `B_AGAIN`은 자동 시퀀스에서는 호출되지 않는다(orchestrator의 `ROTATE_SEQUENCE`에서 제거됨).
> 색 분류 의심 시 webui/CLI에서 단독으로 호출해 `B`면을 재캡처할 때만 사용하는 수동 복구 경로다.

## 단일 노드 + 다중 액션 구조

- `MultiThreadedExecutor` + `ReentrantCallbackGroup`로 한 노드가 5개 액션 콜백을 동시 처리.
- `_run_sequence(steps)`가 각 `Step`을 `await`로 직렬 실행 — 한 step 완료 전에 다음 step 시작 X.
- 그리퍼 step은 `cube_interfaces.action.SafeGrasp`를 통한 `/gripper/safe_grasp` 액션 호출.
  - `await send_goal_async` + `await get_result_async` 두 단계 동기 대기.
  - OPEN: `current_threshold=9999` (전류 비활성, position-only 판정).
  - CLOSE: `current_threshold=300` (전류 임계 초과 시 `grasped=True`).

## Step 타입 (motion_library.py)

| Kind | 설명 |
|---|---|
| `MOVE_L_ABS` | `dsr_msgs2/MoveLine` 절대 좌표 (sync_type=0 동기) |
| `MOVE_L_REL` | `MoveLine` 상대 (TCP 기준 변위) |
| `MOVE_J_ABS` | `dsr_msgs2/MoveJoint` 절대 (조인트 각도 6개) — 특이점 회피용 중간 자세 등 |
| `MOVE_J_REL` | `MoveJoint` 상대 |
| `GRIP` | `/gripper/safe_grasp` 액션 호출 (`pulse` 200/420 매핑) |
| `WAIT` | 정착 대기 (`asyncio.sleep` 기반 권장) |

## 속도/가속 상수

`motion_library.py` 상단에 단위 주석과 함께 정의되어 있다.

```
VEL_X : 직선 이동 속도 (movel translation)        — mm/s
VEL_R : 직선 이동 시 회전 속도 (movel orientation) — deg/s
ACC_X : 직선 이동 가속도                           — mm/s²
ACC_R : 직선 이동 시 회전 가속도                   — deg/s²
VEL_J : 관절 회전 속도 (movej)                     — deg/s
ACC_J : 관절 회전 가속도 (movej)                   — deg/s²
# 참고 — DRL 원본 속도값: VEL_X=250.0 VEL_R=76.5 ACC_X=1000.0 ACC_R=306.0 VEL_J=60.0 ACC_J=100.0
```

DRL 원본값은 주석으로만 보존되며, 모든 모션 시퀀스는 위 기본값을 사용한다(실기 안전 마진 적용).

## TCP offset 등록

노드 기동 시 `_init_tcp_sync`가 다음 두 서비스를 호출해 RH-P12-RN(A) 그리퍼의 TCP를 등록·선택한다.

- `dsr_msgs2/srv/ConfigCreateTcp` (name + pos)
- `dsr_msgs2/srv/SetCurrentTcp` (name)

> **주의**: 이전에는 `drl_start("set_tcp(...)")`로 했으나, 이러면 그리퍼 노드가 띄워둔 long-running
> DRL TCP 서버 슬롯(`robot_system=0`)을 덮어써 그리퍼 readback이 죽었다. 전용 서비스 호출로
> 바꿔서 DRL 슬롯과 충돌하지 않는다.

## 외부 의존

- `dsr_msgs2` — Doosan 드라이버 서비스 (`MoveLine`, `MoveJoint`, `MoveStop`, `ConfigCreateTcp`, `SetCurrentTcp`)
- `cube_interfaces` — 5개 액션 + `SafeGrasp` 액션
- `rh_p12_rna_controller`(`gripper_service_node`) — `/gripper/safe_grasp` 제공자

## 단독 실행 (디버깅)

```bash
# 사전: 다른 터미널에서 dsr_bringup2 / gripper_service_node 기동
ros2 run cube_robot_action robot_action_server_node \
  --ros-args -p robot_ns:=dsr01

# 액션 호출 예 (단독/통합 동일)
ros2 action send_goal /robot/go_home cube_interfaces/action/GoHome "{}"
```

자세한 호출 예는 [`docs/TEST_COMMANDS.md`](../../docs/TEST_COMMANDS.md) §3-2 참고.

## 알려진 제한사항 및 TODO

- **큐브 지그(ZIG) 위치 고정 좌표 하드코딩** — `motion_library.py` 상단의 웨이포인트 상수들이 base frame 절대 좌표로 박혀 있다. 지그/카메라/워크스테이션 위치를 물리적으로 옮기면 코드 수정 + 재빌드가 필요하다. 향후 yaml 파라미터로 분리 권장.
  - `INITIAL_STATE = [373.030, 0.000, 74.660, 22.85, -180.0, 22.85]` — 지그 기준 자세 (ZIG_HOME).
  - `L_DROP_POS`, `R_DROP_POS` — 솔빙 토큰 실행 시 좌/우 드롭 위치.
  - `READY_D_LOW`, `READY_D_UP`, `CUBE_SCAN_B`, `CUBE_SCAN_D` — 면 캡처용 자세.
  - `WORKSTATION_GRIP_Z = 13.71` — 워크스테이션 픽업 시 그리퍼 Z 높이.
- `face_transition_sequence()` 빈 구현 — 면 전환 룩업테이블 채우기 필요.
- 코루틴 안의 `time.sleep(0.3)` → `asyncio.sleep`으로 교체 권장.
- `_call_action`의 `wait_for_server` / `send_goal` / `get_result` 단계별 timeout 분리 필요.
- `_rotate_face_cb` 실패 경로에서 `result.message` 미설정.
