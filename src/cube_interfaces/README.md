# cube_interfaces

큐브 솔버 시스템의 **모든 ROS2 인터페이스(action / srv / msg) 정의를 담는 단일 통합 패키지**.
`ament_cmake` + `rosidl_generate_interfaces` 기반이며, 다른 노드 패키지(모두 `ament_python`)는
이 패키지 하나만 의존(`<exec_depend>cube_interfaces</exec_depend>`)하면 된다.

> 통합 전에는 `cube_orchestrator/srv/`, `cube_perception/srv/`, `cube_robot_action/action/`,
> `rh_p12_rna_controller/action/`, 그리고 별도 `rh_p12_rna_controller_interfaces` 패키지가
> 따로 있었지만 모두 이 패키지 하나로 일원화했다. **import는 일관되게 `from cube_interfaces.{action,srv,msg} import …` 형태.**

## 디렉토리 구조

원본 패키지별로 폴더를 나눠 두어 검색·관리가 쉽도록 정리되어 있다 (실제 ROS2 타입 이름은 폴더와 무관).

```
cube_interfaces/
├── CMakeLists.txt          # rosidl_generate_interfaces로 모든 정의를 한 번에 생성
├── package.xml
├── cube_orchestrator/srv/
│   ├── StartRun.srv
│   ├── StartScan.srv
│   └── StartSolve.srv
├── cube_perception/srv/
│   ├── DetectCubePose.srv
│   ├── ExtractFace.srv
│   └── GetCubeState.srv
├── cube_robot_action/action/
│   ├── ExecuteSolveToken.action
│   ├── GoHome.action
│   ├── PickupCube.action
│   ├── PlaceOnJig.action
│   └── RotateCubeForFace.action
└── rh_p12_rna_controller/
    ├── action/{GripperCommand,SafeGrasp}.action
    ├── msg/GripperState.msg
    └── srv/{GetState,SetPosition}.srv
```

## 인터페이스 목록

| 종류 | 이름 | 용도 |
|---|---|---|
| srv | `StartRun` | scan + solve 연속 실행 |
| srv | `StartScan` | 큐브 감지 + 6면 색상 인식 + kociemba 솔빙 |
| srv | `StartSolve` | 캐시(또는 입력) solution을 모션으로 실행 |
| srv | `DetectCubePose` | 큐브 1차 감지 + U면 캐싱 |
| srv | `ExtractFace` | 단일 면 캡처 ACK (colors_9는 빈 문자열) |
| srv | `GetCubeState` | 5면 분류 + U 캐시 결합 → `state_54` + `faces_json` 반환 |
| srv | `GetState` | 그리퍼 상태 1회 조회 |
| srv | `SetPosition` | 그리퍼 목표 위치 직접 명령 |
| action | `ExecuteSolveToken` | 18 솔빙 토큰 + `'P'` 단일 실행 |
| action | `PickupCube` | step 분할 픽업 (RELEASE_HOME / DESCEND / GRIP / LIFT) |
| action | `PlaceOnJig` | step 분할 안착 (APPROACH / RELEASE) |
| action | `GoHome` | INITIAL_STATE 복귀 |
| action | `RotateCubeForFace` | 카메라 앞 면 노출 자세 (B/R/F/L/D + B_AGAIN 수동) |
| action | `SafeGrasp` | 그리퍼 액션 (전류 임계 기반 grasped 판정) |
| action | `GripperCommand` | (legacy) 호환 목적 |
| msg | `GripperState` | 그리퍼 상태 메시지 (header + position + current + status_text) |

## 빌드

다른 패키지보다 먼저 빌드해야 한다(타입 생성물에 의존).

```bash
colcon build --symlink-install --packages-select cube_interfaces
source install/setup.bash
```

이후 다른 노드 패키지를 빌드:

```bash
colcon build --symlink-install --packages-select \
    cube_perception cube_robot_action cube_orchestrator \
    rh_p12_rna_controller cube_webui
```

## CLI에서 호출 시 타입 이름

```
cube_interfaces/srv/StartRun
cube_interfaces/srv/StartScan
cube_interfaces/srv/StartSolve
cube_interfaces/srv/DetectCubePose
cube_interfaces/srv/ExtractFace
cube_interfaces/srv/GetCubeState
cube_interfaces/srv/GetState
cube_interfaces/srv/SetPosition
cube_interfaces/action/ExecuteSolveToken
cube_interfaces/action/PickupCube
cube_interfaces/action/PlaceOnJig
cube_interfaces/action/GoHome
cube_interfaces/action/RotateCubeForFace
cube_interfaces/action/SafeGrasp
cube_interfaces/action/GripperCommand
cube_interfaces/msg/GripperState
```

테스트 명령어 모음은 [`docs/TEST_COMMANDS.md`](../../docs/TEST_COMMANDS.md) 참고.
