# cube_perception

`cube_perception`은 `cube_orchestrator`가 호출하는 서비스 패키지입니다.
이번 리팩터링 기준으로 책임이 아래처럼 분리됩니다.

## 구성 요소 역할

- `vla_detection_node.py`
  - `DetectCubePose` 서비스 서버
  - RGB/Depth/K를 받아 1차 Gemini 호출 + depth backend 계산 + 최종 `PoseStamped(base_link)` 조립
  - 첫 호출에서 얻은 `top_face_9`를 `/cube_perception/u_face_cache`로 publish
- `gemini_vla_backend.py`
  - 2D 감지 전용
  - 출력: `pixel_uv`, `top_color`, `top_face_9`
- `opencv_depth_backend.py`
  - depth/3D 계산 전용
  - 입력: `pixel_uv`, `depth`, `K`, `T_base_cam`
  - 출력: `depth_m`, `camera_xyz`, `base_xyz`
- `color_extraction_node.py`
  - `ExtractFace`(캡처 ACK) + `GetCubeState`(최종 상태 반환) 서비스 서버
  - `ExtractFace`: `B/R/F/L/D` 캡처/저장만 담당
  - `GetCubeState`: U 캐시 + 5면 추정 결과를 합쳐 `state_54` 반환
- `debug_viewer_node.py`
  - 라이브 카메라/디버그 썸네일/`U` 캐시 텍스트 모니터링
- `service_tester_gui_node.py`
  - PyQt 기반 서비스 테스트 GUI
  - `DetectCubePose`, `ExtractFace`, `GetCubeState` 호출 + 디버그 이미지 + `U` 캐시 표시

## 사전 조건

- ROS 2 Humble
- 카메라 토픽
  - `/camera/camera/color/image_raw`
  - `/camera/camera/aligned_depth_to_color/image_raw`
  - `/camera/camera/color/camera_info`
- `.env` 설정

```bash
GEMINI_API_KEY=YOUR_API_KEY
GEMINI_MODEL=gemini-robotics-er-1.6-preview
```

## 빌드

```bash
cd ~/cube_solver_ver2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select cube_perception
source install/setup.bash
```

## 실행 과정

### 1) perception 스택 실행

```bash
ros2 launch cube_perception perception_stack.launch.py \
  use_realsense:=true \
  color_image_topic:=/camera/camera/color/image_raw \
  depth_image_topic:=/camera/camera/aligned_depth_to_color/image_raw \
  camera_info_topic:=/camera/camera/color/camera_info \
  depth_unit_scale:=0.001 \
  depth_sample_radius_px:=2
```

`use_realsense:=true`로 주면 RealSense 런치를 함께 실행합니다.

### 2) 서비스 호출 순서

1. 1차 호출 (`DetectCubePose`)
   - 내부에서 `pixel_uv + top_face_9` 확보
   - 동시에 `opencv_depth_backend`로 3D/base 계산
2. 캡처 호출 (`ExtractFace`) 5회
   - `B/R/F/L/D` 순서로 캡처 ACK만 수행 (`success=true`, `colors_9` 비움)
3. 상태 조회 (`GetCubeState`) 1회
   - 여기서 Gemini 2차 추정 실행 후 U 캐시와 결합한 `state_54` 수신

CLI 예시:

```bash
# 1차 API
ros2 service call /detect_cube_pose cube_perception/srv/DetectCubePose "{hint: ''}"

# 2차 API (면 캡처 ACK: B/R/F/L/D)
ros2 service call /color_extraction_node/extract_face cube_perception/srv/ExtractFace "{face: 'B'}"
ros2 service call /color_extraction_node/extract_face cube_perception/srv/ExtractFace "{face: 'R'}"
ros2 service call /color_extraction_node/extract_face cube_perception/srv/ExtractFace "{face: 'F'}"
ros2 service call /color_extraction_node/extract_face cube_perception/srv/ExtractFace "{face: 'L'}"
ros2 service call /color_extraction_node/extract_face cube_perception/srv/ExtractFace "{face: 'D'}"

# 3차 API (최종 상태 조회)
ros2 service call /color_extraction_node/get_cube_state cube_perception/srv/GetCubeState "{}"
```

## 디버그/테스트 도구

- OpenCV viewer
```bash
ros2 run cube_perception debug_viewer_node --ros-args -p color_image_topic:=/camera/camera/color/image_raw
```

- PyQt 서비스 테스트 GUI
```bash
ros2 run cube_perception service_tester_gui_node --color-image-topic /camera/camera/color/image_raw
```

## 점검 명령

```bash
ros2 node list | rg "vla_detection_node|color_extraction_node|cube_perception_debug_viewer"
ros2 service list | rg "detect_cube_pose|extract_face|get_cube_state"
ros2 topic echo /cube_perception/u_face_cache --once
```

## 트러블슈팅

- `GEMINI_API_KEY is not set`
  - `.env` 확인 후 노드 재실행
- `Camera frames or camera_info not ready`
  - 카메라 드라이버/토픽 확인
- `depth/color 크기 불일치`
  - aligned depth 토픽 사용 확인
- `GetCubeState`가 U 캐시 없음으로 실패
  - 먼저 `/detect_cube_pose`를 1회 호출했는지 확인

