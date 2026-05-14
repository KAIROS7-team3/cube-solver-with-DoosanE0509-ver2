# cube_webui

`cube_orchestrator`를 브라우저에서 조작하기 위한 FastAPI + WebSocket + MJPEG 기반 웹 UI.
ROS2 노드(`cube_webui` = `webui_server`)가 단일 프로세스 안에서 FastAPI 서버 + rclpy
스핀 + uvicorn ASGI를 동시에 돌린다.

## 엔드포인트

| 경로 | 메서드 | 설명 |
|---|---|---|
| `/` | GET | 메인 UI (정적 `index.html` + `app.js`) |
| `/ws/events` | WS | FSM 상태/토큰/face_colors 실시간 push (JSON) |
| `/stream/camera.mjpg` | GET | RealSense 컬러 스트림 MJPEG |
| `/api/start_scan` | POST | `cube_interfaces/StartScan` 호출 — 결과 반환까지 블로킹 (최대 600s) |
| `/api/start_solve` | POST | `cube_interfaces/StartSolve` 호출 (body로 solution 전달 또는 캐시 사용). **무제한 대기** — 토큰 수·속도에 따른 가짜 timeout을 피하기 위함. 멈춘 경우 Cancel 버튼 사용 |
| `/api/start_run` | POST | `cube_interfaces/StartRun` 호출 (scan + solve 연속, fire-and-forget) |
| `/api/cancel` | POST | `std_srvs/Trigger` `/orchestrator/cancel` 호출 |

Start Scan 완료 후 응답의 `solution` 문자열이 자동으로 폼에 채워진다.

## 화면 구성

- **카메라 패널** — `/stream/camera.mjpg` 임베드.
- **상태 패널** — `/ws/events` 구독으로 받은 FSM 상태/토큰을 실시간 표시.
- **큐브 색 셀** — `/orchestrator/face_colors`(`face:9chars`) burst를 받아 6면을 색 셀로 렌더.
  GetCubeState 직후 1번 발행되며, 9자 색 문자열을 W/Y/R/O/B/G로 매핑.
- **버튼** — Start Scan / Start Solve / Start Run / Cancel.

## 파라미터

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `host` | `0.0.0.0` | uvicorn bind host |
| `port` | `8080` | HTTP 포트 |
| `camera_topic` | `/camera/camera/color/image_raw` | MJPEG 인코딩 대상 RGB 토픽 |

## 실행

`full_stack.launch.py`가 떠 있는 상태에서 별도 터미널:

```bash
cd ~/cube-solver-with-DoosanE0509-ver2
source install/setup.bash
ros2 launch cube_webui webui.launch.py
# 브라우저: http://localhost:8080
```

`ros2 run cube_webui webui_server`로 단독 실행도 가능. orchestrator/perception/robot_action이
다 떠 있어야 버튼이 정상 동작한다.

## 의존 / 설치

- pip: `fastapi`, `uvicorn[standard]`, `opencv-python` — rosdep으로 자동 설치되지 않으므로 수동 설치 필요.
  (워크스페이스 루트 README의 "pip 전용 의존" 표 참고.)
- ROS2: `cube_interfaces` (서비스/액션 타입).

## 디렉토리

```
cube_webui/
├── cube_webui/
│   ├── server.py               # FastAPI + rclpy 통합
│   └── static/
│       ├── index.html
│       └── app.js
├── launch/
│   └── webui.launch.py
└── package.xml
```
