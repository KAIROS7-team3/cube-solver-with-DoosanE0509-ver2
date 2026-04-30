# rh_p12_rna_controller (README2)

`rh_p12_rna_controller`는 **ROS 2 Action**으로 RH-P12-RN(A) 그리퍼를 제어하는 패키지입니다.

- **Action**: `rh_p12_rna_controller/action/GripperCommand.action`
- **Node 실행 파일**: `ros2 run rh_p12_rna_controller gripper_node`
- **로봇 없이 테스트용 TCP 서버**: `fake_drl_tcp_server.py`

---

## 구성 요소

- **PC (ROS 2)**
  - Action Client: `ros2 action send_goal ...`
  - Action Server(Node): `gripper_node` (`GripperCommand` 수신/실행)
- **Doosan Controller (DRL)**
  - `drl_start`로 DRL 코드 실행/주입
  - (TCP 모드) DRL 내부에서 **Pure Python socket**으로 TCP 서버 구동
- **Hardware**
  - Flange Serial(RS-485)로 Modbus RTU 프레임 송수신
  - RH-P12-RN(A) gripper

---

## 통신/제어 구조 (도식화)

### 1) 전체 아키텍처 (모드 선택 포함)

> Mermaid 오류 방지를 위해 라벨에서 HTML(`<br/>`)을 쓰지 않고 `\n` 줄바꿈을 사용합니다.

```mermaid
flowchart LR
  subgraph PC["PC (ROS 2)"]
    CL["Action Client\n(ros2 action send_goal)"]
    ND["gripper_node\n(ActionServer)"]
    CL -->|"GripperCommand"| ND
  end

  ND -->|"Modbus RTU frames (generated)"| MODE{command_transport}

  subgraph CTRL["Doosan Controller (DRL)"]
    DRL["/drl/drl_start\n(DrlStart service)"]
    SRV["Injected TCP server\n(Python socket in DRL)"]
    DRL --> SRV
  end

  MODE -->|"drl"| DRL
  MODE -->|"tcp"| SRV

  ND <-->|"TCP framed JSON\n(cmd / ack / state)"| SRV

  subgraph HW["Hardware"]
    SER["Flange Serial (RS-485)"]
    GRP["RH-P12-RN(A) Gripper"]
    SER --> GRP
  end

  DRL -->|"serial write/read"| SER
  SRV -->|"serial write/read"| SER
```

### 2) TCP 모드 시퀀스 (PC ↔ TCP ↔ DRL ↔ Gripper)

```mermaid
sequenceDiagram
  participant C as Action Client (PC)
  participant N as gripper_node (PC)
  participant D as DRL TCP Server (Controller)
  participant S as Flange Serial (RS-485)
  participant G as Gripper

  C->>N: GripperCommand Goal
  Note over N: Modbus RTU frames 생성
  N->>D: framed JSON cmd {frames:[...]}
  D->>S: serial write (Modbus RTU)
  S->>G: command
  D-->>N: framed JSON ack {ok:true/false}
  loop state stream
    D-->>N: framed JSON state {cur,pos,...}
  end
  Note over N: 전류/위치 기반으로 success, gripped 판정
  N-->>C: Result + Feedback
```

### 3) DRL 단발 실행 모드 시퀀스 (drl_start로 move+poll)

```mermaid
sequenceDiagram
  participant C as Action Client (PC)
  participant N as gripper_node (PC)
  participant R as /drl/drl_start (Controller)
  participant S as Flange Serial (RS-485)
  participant G as Gripper

  C->>N: GripperCommand Goal
  Note over N: DRL 코드(이동+폴링) 생성
  N->>R: DrlStart(code=...)
  R->>S: serial write/read + polling
  S->>G: command
  R-->>N: service response (success)
  N-->>C: Result + Feedback
```

### 4) TCP 통신 데이터 흐름 (ROS 노드/DRL/TCP/Gripper)

```mermaid
flowchart LR
  subgraph ROS["ROS 2 노드: gripper_node"]
    AS["ActionServer(GripperCommand)"]
    ST["Publisher /gripper/state (JointState, state_hz)"]
    RX["TCP recv thread -> last state cache"]
  end

  subgraph CTRL["Doosan Controller (DRL)"]
    TS["Pure python TCP server in DRL\n(JSON framed state/ack)"]
    MB["flange_serial_* + Modbus"]
  end

  G["RH-P12-RN (Modbus)"]

  U["User / higher-level node"] -->|"action goal"| AS
  ROS -->|"DrlStart inject"| TS
  ROS <-->|"TCP client"| TS
  TS <-->|"Modbus"| G
  RX --> ST
```

---

## 실행 명령어 정리

### 빌드

```bash
colcon build --symlink-install
source install/setup.bash
```

### 로봇 없이 TCP 테스트 (fake 서버)

터미널 1:

```bash
python3 src/rh_p12_rna_controller/rh_p12_rna_controller/fake_drl_tcp_server.py
```

터미널 2:

```bash
ros2 run rh_p12_rna_controller gripper_node --ros-args \
  -p command_transport:=tcp \
  -p tcp_external_server:=true \
  -p robot_ip:=127.0.0.1 \
  -p robot_port:=9000
```

Action 전송:

```bash
ros2 action send_goal /rh_p12_rna_controller/gripper_command rh_p12_rna_controller/action/GripperCommand \
  "{action: grab_cube, pulse: 0, current: 0}"
```

### 실제 로봇 (TCP 모드: DRL TCP 서버 주입)

```bash
ros2 run rh_p12_rna_controller gripper_node --ros-args \
  -p command_transport:=tcp \
  -p tcp_external_server:=false \
  -p robot_ns:=dsr01 \
  -p robot_ip:=110.120.1.40 \
  -p robot_port:=9000
```

### 실제 로봇 (DRL 단발 실행 모드: command_transport=drl)

```bash
ros2 run rh_p12_rna_controller gripper_node --ros-args \
  -p command_transport:=drl \
  -p robot_ns:=dsr01 \
  -p robot_ip:=110.120.1.40
```

