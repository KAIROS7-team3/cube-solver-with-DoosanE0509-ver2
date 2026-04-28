# cube_perception 테스트 가이드

다른 팀원이 이 노트북에서 `cube_perception` 패키지만 단독 테스트할 때 쓰는 최소 절차입니다.

## 0) 사전 조건

- ROS2 Humble 설치
- 카메라 토픽 수신 가능
  - `camera/color/image_raw`
  - `camera/depth/image_raw`
  - `camera/color/camera_info`
- Gemini API 키 설정 (`.env`)

예시 `.env`:

```bash
GEMINI_API_KEY=YOUR_API_KEY
GEMINI_MODEL=gemini-robotics-er-1.6-preview
```

## 1) 빌드

```bash
cd ~/cube_solver_ver2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select cube_perception
source install/setup.bash
```

## 2) 노드 실행 (터미널 역할 포함)

### 터미널 A: 1차 감지 노드 (`vla_detection_node`)

- 역할: `DetectCubePose` 서비스 서버
- 입력: color/depth/camera_info 토픽
- 출력: 큐브 3D pose + confidence (`/detect_cube_pose`)
- 참고: 내부적으로 상단면 색(`top_color`)은 캐시만 하고 서비스 응답에는 포함하지 않음

```bash
cd ~/cube_solver_ver2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run cube_perception vla_detection_node --ros-args -p detector_backend:=gemini
```

### 터미널 B: 2차 색 추출 노드 (`color_extraction_node`)

- 역할: `ExtractFace` 서비스 서버
- 입력: color 토픽 + face 요청(U/D/R/L/F/B)
- 출력: 요청 면의 `colors_9` (`/color_extraction_node/extract_face`)
- 참고: B/R/F/L/D 프레임 누적 후 Gemini state 추정을 수행

```bash
cd ~/cube_solver_ver2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run cube_perception color_extraction_node
```

### 터미널 C: 서비스 호출/검증용

- 역할: 테스트 클라이언트 전용
- 용도: 서비스 호출, 토픽/서비스 상태 점검

### 터미널 D: 상태 점검용 (선택)

- 역할: 노드/서비스/토픽 활성화 확인
- 용도: 문제 발생 시 원인 분리 (노드 미기동 vs 카메라 미기동)

## 3) 실행 확인 (터미널 D에서 실행)

```bash
ros2 node list | grep -E "vla_detection_node|color_extraction_node"
ros2 service list | grep -E "detect_cube_pose|extract_face"
ros2 topic list | grep -E "camera/color/image_raw|camera/depth/image_raw|camera/color/camera_info"
```

`ros2 run ...` 실행 후 프롬프트가 멈춘 것처럼 보이는 것은 정상입니다.  
노드가 foreground에서 요청 대기 중인 상태입니다.

## 4) 서비스 호출 테스트 (터미널 C에서 실행)

### 4-1. 큐브 좌표 요청 (1차)

```bash
ros2 service call /detect_cube_pose cube_perception/srv/DetectCubePose "{hint: ''}"
```

정상 시 `success: true`, `pose`, `confidence`가 반환됩니다.

### 4-2. 면 색 요청 (2차)

아래처럼 B/R/F/L/D를 먼저 한 번씩 호출한 뒤 원하는 면을 조회하는 것을 권장합니다.

```bash
ros2 service call /color_extraction_node/extract_face cube_perception/srv/ExtractFace "{face: 'B'}"
ros2 service call /color_extraction_node/extract_face cube_perception/srv/ExtractFace "{face: 'R'}"
ros2 service call /color_extraction_node/extract_face cube_perception/srv/ExtractFace "{face: 'F'}"
ros2 service call /color_extraction_node/extract_face cube_perception/srv/ExtractFace "{face: 'L'}"
ros2 service call /color_extraction_node/extract_face cube_perception/srv/ExtractFace "{face: 'D'}"
```

- 요청 가능한 face: `U D R L F B`
- 현재 구현은 `B/R/F/L/D` 이미지가 누적되어야 상태 추정이 성공합니다.
- 누적 전에는 `success: false`와 함께 `missing` 안내 메시지가 나올 수 있습니다.

## 5) 빠른 점검 명령

```bash
ros2 service list | grep -E "detect_cube_pose|extract_face"
ros2 topic list | grep -E "camera/color/image_raw|camera/depth/image_raw|camera/color/camera_info"
```

## 6) 자주 발생하는 문제

- `GEMINI_API_KEY is not set`
  - `.env` 확인, 노드 재실행
- `Camera frames or camera_info not ready`
  - 카메라 드라이버/토픽 확인
- `depth/color 크기 불일치`
  - aligned depth 토픽 사용 또는 토픽 remap 필요
- `extract failed: state estimation failed ...`
  - B/R/F/L/D 호출 누락 여부 확인 후 다시 호출

## 7) 완전 처음부터 실행 (초간단)

### 터미널 A: RealSense
```bash
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
```

### 터미널 B: perception 스택(노드 2개 동시 실행)
```bash
cd ~/cube_solver_ver2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select cube_perception
source install/setup.bash
ros2 launch cube_perception perception_stack.launch.py \
  use_realsense:=false \
  color_image_topic:=/camera/camera/color/image_raw \
  depth_image_topic:=/camera/camera/aligned_depth_to_color/image_raw \
  camera_info_topic:=/camera/camera/color/camera_info
```

### 터미널 C: 서비스 테스트
```bash
source /opt/ros/humble/setup.bash
source ~/cube_solver_ver2_ws/install/setup.bash

# 1차 API
ros2 service call /detect_cube_pose cube_perception/srv/DetectCubePose "{hint: ''}"

# 2차 API (B/R/F/L/D 순으로 1번씩)
ros2 service call /color_extraction_node/extract_face cube_perception/srv/ExtractFace "{face: 'B'}"
ros2 service call /color_extraction_node/extract_face cube_perception/srv/ExtractFace "{face: 'R'}"
ros2 service call /color_extraction_node/extract_face cube_perception/srv/ExtractFace "{face: 'F'}"
ros2 service call /color_extraction_node/extract_face cube_perception/srv/ExtractFace "{face: 'L'}"
ros2 service call /color_extraction_node/extract_face cube_perception/srv/ExtractFace "{face: 'D'}"
```

선택(디버그 GUI):
```bash
ros2 run cube_perception debug_viewer_node --ros-args -p color_image_topic:=/camera/camera/color/image_raw
```

선택(PyQt 클릭형 서비스 테스트 GUI):
```bash
ros2 run cube_perception service_tester_gui_node --color-image-topic /camera/camera/color/image_raw
```

