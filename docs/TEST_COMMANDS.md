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
ros2 run cube_webui server
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

별도 터미널에서 호출 (단독 실행 시 액션 이름은 **네이티브** `cube_robot_action/...`):

```bash
# 홈 자세
ros2 action send_goal /cube_robot_action/go_home \
  cube_interfaces/action/GoHome "{}"

# 면 회전 자세 (B / R / F / L / D / B_AGAIN)
ros2 action send_goal /cube_robot_action/rotate_cube_for_face \
  cube_interfaces/action/RotateCubeForFace "{next_face: 'B'}"

# 픽업 step별 (RELEASE_HOME → DESCEND → GRIP → LIFT 순서)
ros2 action send_goal /cube_robot_action/pickup_cube \
  cube_interfaces/action/PickupCube "{step: 'RELEASE_HOME'}"
ros2 action send_goal /cube_robot_action/pickup_cube \
  cube_interfaces/action/PickupCube "{step: 'DESCEND'}"
ros2 action send_goal /cube_robot_action/pickup_cube \
  cube_interfaces/action/PickupCube "{step: 'GRIP'}"
ros2 action send_goal /cube_robot_action/pickup_cube \
  cube_interfaces/action/PickupCube "{step: 'LIFT'}"

# 지그 안착 (APPROACH → RELEASE)
ros2 action send_goal /cube_robot_action/place_on_jig \
  cube_interfaces/action/PlaceOnJig "{step: 'APPROACH'}"
ros2 action send_goal /cube_robot_action/place_on_jig \
  cube_interfaces/action/PlaceOnJig "{step: 'RELEASE'}"

# 풀이 토큰 1개
ros2 action send_goal /cube_robot_action/execute_solve_token \
  cube_interfaces/action/ExecuteSolveToken "{token: 'F'}"
```

> **주의**: full_stack 런치를 함께 띄우면 위 액션들은 `/robot/...`로 remap 되어 있습니다.
> 단독 실행 = 네이티브 이름, 통합 실행 = `/robot/...` — 둘은 호환되지 않습니다.

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
| `/robot/...` 액션 wait 무한대기 | 단독 실행 중인데 remap 이름으로 호출. 네이티브 이름 `cube_robot_action/...` 사용하거나 full_stack launch 사용 |
| `failed to create symbolic link ... Is a directory` | `rm -rf build/<pkg> install/<pkg>` 후 재빌드 |
| RotateCubeForFace 시퀀스 멈춤 | 대상 토큰의 직전 step이 안 끝났는지 확인 — orchestrator FSM 로그 보기 |

---

## 6. 종료

```bash
# 각 터미널에서 Ctrl+C
# 잔여 노드 정리
pkill -f ros2; pkill -f master_orchestrator
```
