# rh_p12_rna_controller

ROS 2 Action 기반으로 **RH-P12-RN(A) 그리퍼**를 제어하는 패키지입니다.

- **Action**: `rh_p12_rna_controller/action/GripperCommand.action`
- **Node**: `rh_p12_rna_controller/gripper_node.py` (실행 파일 이름: `gripper_node`)
- **테스트용 TCP 서버(로봇 없이)**: `rh_p12_rna_controller/fake_drl_tcp_server.py`

## 핵심 아이디어

`gripper_node`는 Modbus RTU 프레임을 만들어 로봇 컨트롤러 쪽에 전달하고, 현재 전류/위치를 받아 **완료(success) 및 파지(gripped)** 여부를 판단합니다.

제어 전송 방식은 두 가지가 있습니다.

- **`command_transport=drl`**: `dsr_msgs2/srv/DrlStart`를 통해 DRL 코드를 “짧게” 실행 (로봇 컨트롤러 내부에서 serial write/read + polling)
- **`command_transport=tcp`**: DRL에 **Pure-Python TCP 서버**를 “주입”하고, ROS 노드가 TCP로 framed-JSON 프로토콜로 명령 프레임을 전송/ACK 수신 + state 수신

## 도식화 (전체 흐름)

```mermaid
flowchart LR
  A[ROS 2 Client<br/>ros2 action send_goal] -->|GripperCommand| B[gripper_node<br/>ActionServer]
  B -->|Modbus RTU frames| C{command_transport}

  C -->|drl| D[dsr_msgs2/DrlStart<br/>drl_start]
  D --> E[DRL code executes<br/>serial write/read + polling]
  E --> F[Flange Serial<br/>(RS-485)]
  F --> G[RH-P12-RN(A) Gripper]

  C -->|tcp| H[DRL injected TCP server<br/>Pure python socket]
  B <-->|2-byte len + JSON<br/>cmd/ack/state| H
  H --> F
```

## Action 인터페이스 요약 (`GripperCommand`)

- **Goal**
  - `action`: `"open" | "grab_cube" | "grab_rotate" | "grab_repose" | "custom"`
  - `pulse`: `custom`일 때 목표 pulse (0~1000). 0이면 프리셋 사용
  - `current`: `custom`일 때 목표 current (0~1000). 0이면 프리셋 사용
- **Result**
  - `success`: 동작 성공 여부(전송/완료 판정 포함)
  - `gripped`: 전류/위치 기반 “파지 감지” 결과
  - `final_position`, `final_current`, `message`
- **Feedback**
  - `phase`, `current_position`, `target_position`, `current_load`, `progress`

## 실행 전 준비

이 패키지는 워크스페이스의 `src/` 아래에 있으므로 일반적인 ROS 2 colcon 빌드 흐름을 따릅니다.

```bash
# (워크스페이스 루트에서)
colcon build --symlink-install
source install/setup.bash
```

## 실행 방법

### A) 로봇 없이 로컬에서 TCP 프로토콜 테스트 (추천: 빠른 개발/디버깅)

`fake_drl_tcp_server.py`는 DRL TCP 서버와 동일한 framed-JSON 프로토콜을 흉내내며,
`gripper_node`의 TCP 송수신/ACK 대기/상태 파싱 로직을 검증할 수 있습니다.

1) Fake 서버 실행

```bash
python3 src/rh_p12_rna_controller/rh_p12_rna_controller/fake_drl_tcp_server.py
```

2) `gripper_node`를 TCP 외부 서버 모드로 실행

```bash
ros2 run rh_p12_rna_controller gripper_node --ros-args \
  -p command_transport:=tcp \
  -p tcp_external_server:=true \
  -p robot_ip:=127.0.0.1 \
  -p robot_port:=9000
```

3) Action 전송 예시

> 기본 action name은 `gripper_node` 파라미터 `action_name`의 기본값을 따릅니다.
> (기본: `/rh_p12_rna_controller/gripper_command`)

```bash
ros2 action send_goal /rh_p12_rna_controller/gripper_command rh_p12_rna_controller/action/GripperCommand \
  "{action: grab_cube, pulse: 0, current: 0}"
```

### B) 실제 로봇에서 TCP 모드로 실행 (DRL TCP 서버 주입 + 실시간 state)

이 모드는 `gripper_node`가 `drl_start`로 **무한 루프 DRL TCP 서버 코드를 주입**한 뒤,
해당 서버에 소켓으로 접속해 `cmd/ack/state`를 주고받습니다.

```bash
ros2 run rh_p12_rna_controller gripper_node --ros-args \
  -p command_transport:=tcp \
  -p tcp_external_server:=false \
  -p robot_ns:=dsr01 \
  -p robot_ip:=110.120.1.40 \
  -p robot_port:=9000
```

### C) 실제 로봇에서 DRL 단발 실행 모드로 실행 (`command_transport=drl`)

이 모드는 TCP 서버를 쓰지 않고, 매 동작 시 `drl_start`로 **move+poll DRL 코드**를 실행합니다.

```bash
ros2 run rh_p12_rna_controller gripper_node --ros-args \
  -p command_transport:=drl \
  -p robot_ns:=dsr01 \
  -p robot_ip:=110.120.1.40
```

## 자주 쓰는 파라미터

`gripper_node.py`에서 선언한 주요 파라미터 중, 동작에 영향이 큰 것들입니다.

- **연결/모드**
  - `command_transport`: `drl` 또는 `tcp`
  - `tcp_external_server`: `true`면 DRL 주입 없이 외부 TCP 서버로 직접 접속 (로봇 없이 테스트용)
  - `robot_ip`, `robot_port`
  - `robot_ns`: `/<ns>/drl/drl_start` 경로 prefix를 만들 때 사용 (예: `dsr01`)
- **레지스터 맵 (펌웨어/설정에 따라 튜닝)**
  - `present_current_reg` (기본 287)
  - `present_position_reg` (기본 284), `present_position_regs` (1 또는 2)
  - `goal_current_reg` (기본 276)
  - `goal_position_reg` (기본 282), `goal_position_regs` (1 또는 2)
  - `goal_position_write_mode`: `auto|fc06|fc16`
- **상태 스케일/파지 판정**
  - `position_scale` (기본 64.0): raw position → pulse 변환 스케일
  - `goal_position_scale` (기본 1.0): pulse → goal register로 쓸 값 스케일
  - `grip_current_threshold` (기본 50)
  - `done_pos_tolerance` (기본 20)
  - `action_max_wait_sec` (기본 20.0)
- **프리셋**
  - `pulse_open`, `pulse_cube`, `pulse_rotate`, `pulse_repose`
  - `init_current`, `cube_current`

## TCP framed-JSON 프로토콜 (요약)

DRL 서버(또는 fake 서버)와 ROS 노드 사이 통신은 아래 프레이밍을 사용합니다.

- **Frame**: `2-byte big-endian length` + `JSON utf-8`
- **Client → Server**
  - `{"type":"cmd","id":<int>,"frames":[<hexstr>, ...]}`
  - `{"type":"ping"}`
- **Server → Client**
  - `{"type":"hello", ...}`
  - `{"type":"ack","id":<int>,"ok":<bool>,"err":<str>}`
  - `{"type":"state","cur":<int>,"pos":<int>, ...}`

