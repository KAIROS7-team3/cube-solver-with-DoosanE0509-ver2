# 테스트 실행 명령어 모음

> 환경 가정: 로봇 IP `110.120.1.40` (임시), 워크스페이스 루트 `~/cube-solver-with-DoosanE0509-ver2`, ROS 2 Humble.
> 모든 명령은 별도 표기가 없으면 워크스페이스 루트에서 실행하세요. (`load_dotenv()`가 cwd 기준으로 `.env`를 읽습니다.)

---

## 0. 1회성 사전 준비

### 0-1. `.env` 작성 (Gemini API 키)

워크스페이스 루트의 `.env` 파일에 실제 키를 채워넣습니다.

```bash
cd ~/cube-solver-with-DoosanE0509-ver2
cat > .env <<'EOF'
GEMINI_API_KEY=여기에_실제_키
GEMINI_MODEL=gemini-robotics-er-1.6-preview
EOF
```

키 발급: https://aistudio.google.com/apikey

### 0-2. 빌드

```bash
cd ~/cube-solver-with-DoosanE0509-ver2
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

빌드 충돌 시 (심볼릭 링크 에러 등):

```bash
rm -rf build/<문제패키지> install/<문제패키지>
colcon build --symlink-install --packages-select <문제패키지>
```

### 0-3. 매 새 터미널마다

```bash
cd ~/cube-solver-with-DoosanE0509-ver2
source /opt/ros/humble/setup.bash
source install/setup.bash
```

---

## 1. 통합 실행 (full stack)

### 1-1. 모든 노드 한 번에

```bash
cd ~/cube-solver-with-DoosanE0509-ver2
ros2 launch cube_orchestrator full_stack.launch.py \
  dsr_host:=110.120.1.40 \
  dsr_port:=12345 \
  use_dsr:=true \
  use_gripper:=true \
  use_realsense:=true
```

이 launch가 띄우는 노드:

- `dsr_bringup2/dsr_bringup2.launch.py` (Doosan 드라이버)
- `realsense2_camera/rs_launch.py` (D435 색/정렬-깊이)
- `cube_perception/perception_stack.launch.py` (vla_detection + color_extraction)
- `rh_p12_rna_controller/gripper_node` + `gripper_service_node` (DRL 임베드 그리퍼)
- `cube_robot_action/robot_action_server_node`
- `cube_orchestrator/master_orchestrator_node`

### 1-2. 시나리오 트리거 (별도 터미널)

```bash
# 픽업 + 6면 스캔 + 풀이 + 토큰 시퀀스 + 홈 — 전체 파이프라인
ros2 service call /cube_orchestrator/start_run \
  cube_interfaces/srv/StartRun "{}"

# 6면 스캔만 (큐브를 사람이 그리퍼에 쥐어준 상태에서 사용)
ros2 service call /cube_orchestrator/start_scan \
  cube_interfaces/srv/StartScan "{}"

# 풀이만 (마지막 스캔 결과 재사용)
ros2 service call /cube_orchestrator/start_solve \
  cube_interfaces/srv/StartSolve "{}"
```

### 1-3. 상태 모니터링

```bash
ros2 topic echo /orchestrator/state
ros2 topic echo /orchestrator/fault
```

---

## 2. WebUI

```bash
cd ~/cube-solver-with-DoosanE0509-ver2
ros2 run cube_webui webui_server
# 브라우저: http://localhost:8080
```

`full_stack.launch.py`가 떠 있는 상태에서 WebUI 버튼이 동작합니다.

---

## 3. 단독 노드 테스트

### 3-1. 그리퍼 (Doosan 없이는 의미 제한적)

```bash
# 노드 단독 (DRL 모드)
ros2 run rh_p12_rna_controller gripper_node \
  --ros-args -p robot_ip:=110.120.1.40 -p tcp_port:=12345 \
            -p cmd_transport:=drl

# 별도 터미널: 서비스 호출
ros2 service call /gripper/set_position cube_interfaces/srv/SetPosition \
  "{position: 200, current: 500, timeout_sec: 8.0}"   # OPEN
ros2 service call /gripper/set_position cube_interfaces/srv/SetPosition \
  "{position: 420, current: 500, timeout_sec: 8.0}"   # CLOSE

# SafeGrasp 액션 (큐브 닿으면 grasped=true)
ros2 action send_goal /gripper/safe_grasp \
  cube_interfaces/action/SafeGrasp \
  "{target_position: 420, goal_current: 500, current_threshold: 300, timeout_sec: 8.0}"
```

> `timeout_sec`는 **그리퍼 동작 한계 시간**입니다. 너무 짧으면 readback 폴링 윗바운드가 줄어 success가 안 떨어질 수 있으니 8s 이상 권장.

### 3-2. 로봇 액션 (Doosan 드라이버 필요)

먼저 다른 터미널에서 드라이버 띄우기:

```bash
ros2 launch dsr_bringup2 dsr_bringup2.launch.py \
  host:=110.120.1.40 port:=12345 mode:=real model:=e0509
```

그 다음:

```bash
ros2 run cube_robot_action robot_action_server_node
```

별도 터미널에서 호출 (액션 이름은 단독/통합 모두 `/robot/...` — `robot_action_server_node`가 직접 등록하며 remap이 없다):

```bash
# 홈 자세
ros2 action send_goal /robot/go_home \
  cube_interfaces/action/GoHome "{}"

# 면 회전 자세 (B / R / F / L / D / B_AGAIN)
ros2 action send_goal /robot/rotate_cube_for_face \
  cube_interfaces/action/RotateCubeForFace "{next_face: 'B'}"

# 픽업 step별 (RELEASE_HOME → DESCEND → GRIP → LIFT 순서)
ros2 action send_goal /robot/pickup_cube \
  cube_interfaces/action/PickupCube "{step: 'RELEASE_HOME'}"
ros2 action send_goal /robot/pickup_cube \
  cube_interfaces/action/PickupCube "{step: 'DESCEND'}"
ros2 action send_goal /robot/pickup_cube \
  cube_interfaces/action/PickupCube "{step: 'GRIP'}"
ros2 action send_goal /robot/pickup_cube \
  cube_interfaces/action/PickupCube "{step: 'LIFT'}"

# 지그 안착 (APPROACH → RELEASE)
ros2 action send_goal /robot/place_on_jig \
  cube_interfaces/action/PlaceOnJig "{step: 'APPROACH'}"
ros2 action send_goal /robot/place_on_jig \
  cube_interfaces/action/PlaceOnJig "{step: 'RELEASE'}"

# 풀이 토큰 1개
ros2 action send_goal /robot/execute_solve_token \
  cube_interfaces/action/ExecuteSolveToken "{token: 'F'}"
```

### 3-3. perception 단독

```bash
ros2 launch cube_perception perception_stack.launch.py use_realsense:=true
```

호출:

```bash
# 큐브 위치 + U면 캐싱
ros2 service call /cube_perception/detect_cube_pose \
  cube_interfaces/srv/DetectCubePose "{hint: ''}"

# 면 캡처 (B/R/F/L/D 순서로 5번)
ros2 service call /cube_perception/extract_face \
  cube_interfaces/srv/ExtractFace "{face_label: 'B'}"

# 6면 합성 → kociemba 입력 문자열
ros2 service call /cube_perception/get_cube_state \
  cube_interfaces/srv/GetCubeState "{}"
```

### 3-4. 카메라만 확인

```bash
ros2 launch realsense2_camera rs_launch.py \
  align_depth.enable:=true depth_module.profile:=640x480x30
ros2 topic hz /camera/camera/color/image_raw
ros2 run rqt_image_view rqt_image_view
```

---

## 4. 디버깅용 토픽/서비스 목록

```bash
# 액션/서비스 목록
ros2 action list | grep -E "robot|gripper|cube"
ros2 service list | grep -E "cube|gripper|orchestrator"
ros2 topic list | grep -E "cube|gripper|orchestrator"

# perception 디버그
ros2 topic echo /cube/state_raw            # 6면 합성 직전 raw 색상
ros2 run rqt_image_view rqt_image_view     # /cube_perception/debug/image
```

---

## 5. 트러블슈팅 빠른 참조

| 증상 | 원인/조치 |
|---|---|
| `GEMINI_API_KEY is not set` | `.env`가 cwd 기준에 없음 → 워크스페이스 루트에서 실행하거나 `export GEMINI_API_KEY=...` |
| `DRL 서비스 연결 실패` | dsr_bringup2가 늦게 뜸. 그리퍼 노드는 30s 대기 후 포기 — full_stack launch 먼저 안정화 후 그리퍼 단독 실행 |
| 그리퍼 액션 success=false인데 물리 동작 OK | DRL 폴링 readback이 늦거나 실패. 이미 `_call_drl` 60s + readback fail-break + max_loops 동적화 적용됨 — `timeout_sec`을 8s 이상으로 |
| `/robot/...` 액션 wait 무한대기 | `robot_action_server_node`가 안 떠 있거나 액션 이름 오타. 옛 `cube_robot_action/...` 경로는 더 이상 존재하지 않음 — `ros2 action list \| grep robot`로 실제 이름 확인 |
| `failed to create symbolic link ... Is a directory` | `rm -rf build/<pkg> install/<pkg>` 후 재빌드 |
| RotateCubeForFace 시퀀스 멈춤 | 대상 토큰의 직전 step이 안 끝났는지 확인 — orchestrator FSM 로그 보기 |
| 특이점/충돌 비상정지 후 모션 서비스가 거부됨 | 로봇이 SAFE_STOP / SAFE_OFF 상태. 아래 §5-1 복귀 절차 참고 |

### 5-1. 특이점·SAFE_STOP 복귀 절차

`MOVE_L` 도중 손목/엘보 특이점, 충돌 토크 임계 초과 등으로 로봇이 SAFE_STOP 상태에 들어가면 `dsr_msgs2` 모션 서비스가 거부된다. 코드 변경 없이 ROS2 서비스 한두 줄로 복귀 가능 (네임스페이스는 통합런치 기본인 `dsr01` 기준).

#### (1) 현재 상태 확인

```bash
ros2 service call /dsr01/system/get_robot_state \
  dsr_msgs2/srv/GetRobotState "{}"
```

응답의 `robot_state` 가 `STATE_SAFE_STOP` 인지, 더 깊은 `STATE_SAFE_STOP2` / `STATE_SAFE_OFF` 인지 확인.

#### (2) 가벼운 SAFE_STOP — 한 줄 복귀

특이점/소프트리밋으로 인한 일반적인 SAFE_STOP 은 `RESET_SAFET_STOP` 한 번이면 STANDBY 로 복귀.

```bash
ros2 service call /dsr01/system/set_robot_control \
  dsr_msgs2/srv/SetRobotControl "{robot_control: 2}"   # CONTROL_RESET_SAFET_STOP
```

복귀 후엔 `MoveJoint` 등으로 안전 자세까지 직접 이동 (`/dsr01/motion/move_joint` 또는 `cube_robot_action`의 `/robot/go_home`).

#### (3) SAFE_STOP2 / SAFE_OFF — Recovery 모드 (= "안전모드") 3단계

충돌·SAFE_OFF 까지 빠진 경우 한 번에 안 풀린다. 저속 단일관절 이동만 허용되는 Recovery 모드로 진입해 특이점 밖으로 살짝 뺀 뒤 STANDBY 로 복귀.

```bash
# 1) Recovery 진입
ros2 service call /dsr01/system/set_robot_control \
  dsr_msgs2/srv/SetRobotControl "{robot_control: 4}"   # CONTROL_RECOVERY_SAFE_STOP
#   - SAFE_OFF 였으면 5: CONTROL_RECOVERY_SAFE_OFF

# (또는) 명시적 모드 전환
ros2 service call /dsr01/system/set_safety_mode \
  dsr_msgs2/srv/SetSafetyMode "{safety_mode: 2, safety_event: 0}"   # RECOVERY + ENTER

# 2) RViz/TP/movej 로 관절을 특이점 밖으로 이동
#    (Recovery 모드에서는 저속·단일관절 이동만 허용됨)

# 3) Recovery 종료 → STANDBY 복귀
ros2 service call /dsr01/system/set_robot_control \
  dsr_msgs2/srv/SetRobotControl "{robot_control: 7}"   # CONTROL_RESET_RECOVERY
```

#### (4) 보조 — 모드 토글

```bash
# AUTONOMOUS / MANUAL 토글 (자동운전 ↔ 수동)
ros2 service call /dsr01/system/set_robot_mode \
  dsr_msgs2/srv/SetRobotMode "{robot_mode: 1}"   # 0=MANUAL, 1=AUTONOMOUS, 2=MEASURE
```

#### `robot_control` 값 매핑 (참고)

| 값 | 상수 | 전환 |
|---|---|---|
| 0 | CONTROL_INIT_CONFIG | NOT_READY → INITIALIZING (TP 전용) |
| 1 | CONTROL_ENABLE_OPERATION | INITIALIZING → STANDBY (TP 전용) |
| **2** | **CONTROL_RESET_SAFET_STOP** | **SAFE_STOP → STANDBY** ← 가장 흔함 |
| 3 | CONTROL_RESET_SAFET_OFF | SAFE_OFF → STANDBY |
| 4 | CONTROL_RECOVERY_SAFE_STOP | SAFE_STOP2 → RECOVERY |
| 5 | CONTROL_RECOVERY_SAFE_OFF | SAFE_OFF2 → RECOVERY |
| 6 | CONTROL_RECOVERY_BACKDRIVE | H/W backdrive (전원 재부팅 필요) |
| 7 | CONTROL_RESET_RECOVERY | RECOVERY → STANDBY |

#### `safety_mode` 값 (참고)

`0=MANUAL, 1=AUTONOMOUS, 2=RECOVERY, 3=BACKDRIVE, 4=MEASURE, 5=INITIALIZE`

> **권장 흐름**: (1) `get_robot_state` → (2) SAFE_STOP 이면 `SetRobotControl 2` → 안 풀리면 (3) `SetRobotControl 4` → movej로 특이점 회피 → `SetRobotControl 7`. 이래도 안 되면 TP 에서 직접(전원 재부팅·backdrive까지).
>
> 해당 시퀀스의 자동화는 별건 — 현 시점에서는 코드 변경 없이 운영자가 위 명령을 직접 호출하는 방식만 지원한다.

---

## 6. 종료

```bash
# 각 터미널에서 Ctrl+C
# 잔여 노드 정리
pkill -f ros2; pkill -f master_orchestrator
```
