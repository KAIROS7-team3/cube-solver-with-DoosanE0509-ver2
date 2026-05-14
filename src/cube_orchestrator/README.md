# cube_orchestrator

Top-level FSM 노드. 인식(`cube_perception`) + 모션(`cube_robot_action`) + 그리퍼
(`rh_p12_rna_controller`) 패키지를 묶어 큐브 풀이 전체 시나리오를 구동한다.

## 분기

- `start_scan` : DETECT_CUBE → PICKUP×4 → ROTATE×5 + ExtractFace×5 → SOLVE
  - 결과(`state_54`, `solution`)를 응답으로 반환 + 노드 내부 캐시
  - 콜백 응답 timeout = `SCAN_CALLBACK_TIMEOUT_SEC` (600s)
- `start_solve` : PLACE×2 → **GO_HOME(1회)** → EXECUTE_TOKEN×N → GO_HOME
  - 캐시된 solution(또는 사용자가 넘긴 solution) 토큰을 순차 실행
  - 콜백 응답 timeout = **무제한** (`SOLVE_CALLBACK_TIMEOUT_SEC = None`) — 토큰 수·속도에 따른 시간 예측이 어려워 잘못된 timeout 실패를 피하기 위함. 멈춘 경우 `/orchestrator/cancel` 로 중단.
  - 토큰 루프 시작 전 GoHome 1회는 시작 자세(JOINT_HOME) normalize 용도. robot_action 서버의 토큰별 wrap 이 제거된 것과 짝.
- `start_run`   : `start_scan` → `start_solve` 일괄 (fire-and-forget — 호출 즉시 "started" 응답)

## webui 면 색 표시 흐름 (face_colors)

```
DetectCubePose  →  vla_detection_node가 U면 9자를 /cube_perception/u_face_cache에 publish
                   color_extraction_node가 이를 캐시
ExtractFace×5   →  B/R/F/L/D 각 면을 캡처(이미지만 저장). colors_9 응답은 빈 문자열.
GetCubeState    →  Gemini 2차 추론으로 5면 색 분류 + U 캐시 결합 → 54자 + faces_json 반환
                   ↓
                   orchestrator가 faces_json을 파싱해 6면 모두 /orchestrator/face_colors로
                   `face:9chars` 포맷 burst publish → webui가 WYROBG 색 셀로 렌더
                   ↓
kociemba.solve  →  solution 토큰 publish → 이후 PLACE/EXECUTE/HOME
```

**중요**: ExtractFace는 단순 캡처 ACK라 `colors_9`가 항상 빈 문자열. 색은
GetCubeState 응답에서만 만들어진다. 따라서 face_colors 발행 시점은 무조건
`_run_solve`의 GetCubeState 직후 + kociemba 호출 직전 한 군데뿐.

## 외부 인터페이스

- 서비스
  - `/orchestrator/start_scan`   — `cube_interfaces/StartScan`
  - `/orchestrator/start_solve`  — `cube_interfaces/StartSolve`
  - `/orchestrator/start_run`    — `cube_interfaces/StartRun`
  - `/orchestrator/cancel`       — `std_srvs/Trigger`
  - `/orchestrator/emergency_stop` — `std_srvs/Trigger` (즉시 정지)
- 토픽 (publish)
  - `/orchestrator/state`         — String (FSM 상태)
  - `/orchestrator/solution`      — String (kociemba 결과 토큰 시퀀스)
  - `/orchestrator/current_token` — String (실행 중 토큰)
  - `/orchestrator/face_colors`   — String `face:9chars` (위 흐름 참고)
  - `/orchestrator/last_detection`— PoseStamped (DetectCubePose 결과)
  - `/orchestrator/fault`         — String (실패 사유)

---

# 향후 옵션 (현재 미구현, 별건 작업)

## 1. 캡처 진행 인디케이터

현재 ExtractFace ACK 후 webui엔 아무 신호도 안 보낸다(가짜 색을 흘리던 옛
구현 제거함). 5면 캡처 진행을 사용자에게 보이려면 별도 채널 권장:

- 새 토픽 `/orchestrator/capture_progress`(String) 추가
- `_run_extract`에서 성공 시 `f"{face}:captured ({k}/5)"` 발행
- webui가 별도 이벤트 타입(`capture_progress`)으로 핸들 → 진행률/체크 마크 표시

face_colors 토픽에 status를 섞지 말 것 — 포맷이 같아 보여도 의미가 다르다.
기존 `face:colors_9`는 9자 색 문자열 전제이므로 status를 섞으면 webui
렌더가 다시 깨진다.

## 2. 색 확인 후 사용자 GO 게이팅

지금은 GetCubeState → face_colors publish → kociemba.solve → SOLVE 단계가
**연속 동기 흐름**이라 색이 잠깐 보였다가 바로 토큰 실행에 들어간다.
"색 확인 후 사용자가 GO 누르면 그때 solve" 식 게이팅이 필요하면:

- orchestrator FSM에 신규 상태 `WAIT_USER_CONFIRM` 추가
- 신규 서비스 `/orchestrator/confirm_colors`(`std_srvs/Trigger`) 추가 — 호출되면
  대기를 해제
- `_run_solve`에서 GetCubeState 성공 + face_colors publish 후, confirm 신호가
  올 때까지 `_call_service`-style 폴링으로 블로킹 대기
- 타임아웃 시 fault 또는 자동 진행(파라미터로 정책 결정)
- webui에 "확인" 버튼 + `/api/confirm_colors` POST 핸들러 추가

이 옵션은 사용자 인터랙션을 FSM에 끼워넣는 변경이라 cancel/emergency_stop과의
상호작용도 같이 정리해야 한다(예: confirm 대기 중 cancel 들어오면 어떻게
처리할지). 별도 작업으로 분리 권장.

## 3. 색 분류 신뢰도 표시 / 재캡처 트리거

GetCubeState 응답에 confidence 필드가 없어서 webui에서 "이 면 색이 의심스러운데
재캡처할까?" 같은 판단을 못 한다. 필요하다면:

- `cube_interfaces/srv/GetCubeState`에 `float32 confidence` 또는
  `float32[6] confidence_per_face` 추가
- `color_extraction_node._run_state_estimation`에서 Gemini 응답의 confidence를
  보존
- B_AGAIN 액션과 ExtractFace 재호출 경로는 이미 있음 — 사용자가 webui에서
  "B 다시" 버튼 누르면 `RotateCubeForFace(B_AGAIN)` + `ExtractFace(B)` 발사
  후 다시 GetCubeState 호출
