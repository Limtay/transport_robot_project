# ECU_V3 CHECKER/RECOVERY Fail-safe 종합 분석 (2026-07-17)

분석 대상: `RD_TASK_SYSTEM` 의 CHECKER→RECOVERY→FSM 체인 전체
(`rd_system.c` / `rd_uart.c` / `rd_can_motor.c` + `can_ak.c` / `rd_i2c_encoder.c` + `i2c_as5600.c` / `rd_adc.c`).
각 채널에 대해 ①선 뽑힘(완전 단절) ②간헐 접촉 불량 ③지속 노이즈/보드레이트 불일치
④부분 연결(CAN 1/4) 시나리오를 코드 경로 그대로 추적해 fail-safe 성립 여부를 판정.

> 판정 기준: **fail-safe = "모터에 위험한 명령이 나가지 않는 상태로 수렴하는가"**.
> 가용성(availability, 복구 가능 여부)은 별도 축으로 표기.

---

## 1. 방어 계층 구조 (요약)

| 계층 | 소유 | 메커니즘 | 반응 시간 |
|---|---|---|---|
| L0 프로토콜 | 드라이버/HW | CAN CRC·ACK / Dynamixel CRC / RC checksum / I2C NACK | 프레임 단위 |
| L1 진단 | CHECKER (systemTask 100Hz, i2c1Task 100Hz) | HAL 에러 분류 → health, degraded_cnt → lifecycle | 10ms tick |
| L2 복구 | systemTask (RECOVERY 호출), i2c 는 자가 복구 | 채널 재초기화 / 버스클리어 / DMA 재무장 | OFFLINE 감지 즉시 |
| L3 정책 FSM | robot_state + ACTION_STATE_* | rc_ok 산출 → motor_on / ESTOP_override / FAULT | 10ms tick |
| L4 게이트 | 비구동 훅 + PERIPHERAL_WRITE | CMD_CLEAR (reg=0) / TX skip / BRAKE 덮어쓰기 | 10ms(훅) / 5ms(TX) |
| L5 최후 | IWDG | controlTask 하트비트 정지 → MCU 리셋 | IWDG 주기 |
| L6 외부 | Orin | soft_estop / addr5 리셋 요청 / 50ms 명령 가드 | Orin 정책 |

**시간 상수 (실코드 기준):**
- 무수신 타임아웃: UART 100ms (`UART_RX_TIMEOUT_MS`) / CAN 모터별 100ms / AUTO 명령 100ms (`AUTO_TIMEOUT`)
- degraded: 지속 에러 시 **100ms 후 DEGRADED**(cnt≥200, +20/10ms), **~500ms 후 OFFLINE**(cnt 포화 1000)
- 연속 HAL 에러 즉시 OFFLINE: UART 11틱 ≈ **110ms** (`UART_FATAL_CNT_TH=10`), CAN/I2C 6틱 ≈ **60ms** (`HAL_FATAL_CNT_TH=5`)
- FAULT escalation: OFFLINE 이벤트당 `fatal_cnt +20`, 비-NOK 틱당 −1, 임계 200

---

## 2. 채널별 감시 요약

| 채널 | 감지 경로 | 복구 주체 | 최종 escalation | 주행 연동 |
|---|---|---|---|---|
| CAN (모터×4) | HAL 에러 IT + 모터별 RX 100ms + error_code/temp | systemTask RECOVERY | fatal→ **FAULT** (Orin reboot 대기) | MANUAL/AUTO motor_on, motor fault→ESTOP_SW |
| UART1 (RC) | HAL 에러 + RX 100ms + checksum | systemTask RECOVERY | fatal→ **RECOVERING 동결** + addr54 리셋 요청 (FAULT 없음) | MANUAL rc_ok |
| UART2 (RS485) | HAL 에러 + RX 100ms + Dynamixel CRC | systemTask RECOVERY | fatal→ **FAULT + 소프트 리붓** | AUTO rc_ok + AUTO_TIMEOUT |
| UART6 (IMU) | HAL 에러 + RX 100ms + 프레임 checksum | systemTask RECOVERY | fatal→ RECOVERING 동결 + 리셋 요청 | 없음 (텔레메트리) |
| I2C (엔코더×5) | 블로킹 read 실패 (NACK, timeout 2ms) | **i2c1Task 자가 복구** (버스클리어 9클럭) | 없음 (무한 자가 재시도) | 없음 (텔레메트리) |
| ADC (로드셀×2) | staleness 20ms + 레일값 | systemTask RECOVERY (Stop→Start) | 없음 | 없음 (현재 미사용) |

---

## 3. 시나리오별 상세 분석

### 3.1 CAN — AK 모터

**(A) 부분 연결: 4개 중 1개만 연결** — 사용자 관찰 "통신 잘 됨"의 실제 내부 동작

- 연결된 모터: RX 정상 → `any_running=1` → RUNNING 승격. TX 프레임의 ACK 는 버스 상의
  **아무 노드나** 해주므로 (CAN ACK 는 수신자 지정이 아님) 미연결 모터용 TX 도 에러 없이 성공.
- 미연결 모터 3개: `last_rx_tick` 미갱신 → 모터별 100ms 타임아웃 → `any_comm_err=1` →
  **HC_TIMEOUT 지속** → degraded_cnt +20/틱 → 100ms 후 DEGRADED → **~510ms 후 OFFLINE**
  → RET_NOK → systemTask 가 RECOVERY (TX 큐 리셋 + 메일박스 abort + 핸들 전체 재INIT + CAN 재기동)
  → READY → 연결 모터 RX 로 즉시 RUNNING → 다시 타임아웃 누적 → **~510ms 주기로 무한 반복**.
- fatal_can1_cnt 는 OFFLINE 틱에만 +20, 사이 ~50틱 동안 −1씩 감쇠 → **FAULT 로 escalation 되지 않음** (순환 고착).
- motor_on: 루프 순서가 CHECKER(→RECOVERY)가 ACTION 보다 먼저라, OFFLINE 은 같은 틱 안에서
  READY 로 복구된 뒤 ACTION 이 평가 → `RD_CAN_LINK_DOWN()==0` → **motor_on 유지**.

**판정: fail-operational ○ / 준안정 △** — 겉으로는 "통신 잘 되는" 게 맞지만, 실제로는
~0.5초마다 TX 큐 리셋·핸들 재초기화가 반복되는 준안정 순환. 큐 리셋 순간 프레임 수 개
유실(200Hz 기준 ~10ms 공백) + delta_tick/에러 카운터 전체 리셋으로 진단 정보도 주기적으로 지워짐.
테스트 리그(모터 1개)에선 무해하나, **실주행 중 모터 1개 커넥터 탈락 시에도 같은 순환**이 돌며
나머지 3개 모터의 TX 가 0.5초마다 순단됨 — 의도된 설계인지 결정 필요 (→ §5-H1).

**(B) 선 뽑힘 (CAN H/L 전체 단절, 주행 중)**

- TX ACK 실패 → HAL EWG/EPV 캡처 → HC_BUS_WARNING/PASSIVE (11~12) + `rx_error_cnt` 연속 증가
  → 6틱(~60ms) 내 즉시 OFFLINE → RECOVERY → 재기동 성공(페리페럴은 정상) → TX 재개 → 에러 재발.
- 순환마다 fatal_can1_cnt 순증(+20/이벤트, 사이 감쇠 −6) → **~1초 내 FATAL_MAX 도달 →
  `hw.reset.bit.can=1` + robot_state = FAULT** → motor_on=0 영구, Orin REBOOT 대기.
- OFFLINE 을 본 틱의 ACTION 에서 `RD_CAN_LINK_DOWN()` → motor_on=0 + 비구동 훅 CMD_CLEAR.

**판정: fail-safe ✓** — 수 초 내 FAULT 수렴, 잔류 명령 청소됨. (모터 전원이 살아 있어도
버스가 없어 명령 도달 불가 — 모터 측 자체 타임아웃은 AK 드라이버 펌웨어 소관.)

**(C) 간헐 접촉 불량 (커넥터 흔들림)**

- 산발 CRC/form 에러 → HC_CRC/FRAMING (경고) → degraded_cnt 누적/감쇠 히스테리시스.
  5% 미만 손실은 중립, 10%+ 손실 ~2초 → DEGRADED (주행은 유지 — MANUAL/AUTO rc_ok 는 DEGRADED 허용).
- 연속 HAL 에러 6틱은 순단성 노이즈로는 잘 안 쌓임 (clean 틱에 `rx_error_cnt=0` 리셋) → 설계 의도대로
  "빈도는 degraded, 연속은 fatal" 분리 동작.
- 접촉이 100ms 이상 끊기는 순간이 오면 (B) 경로로 진입.

**판정: fail-safe ✓ (의도된 열화 운전)** — DEGRADED 상태가 reg 로 발행되어 Orin 이 인지 가능.

**(D) 버스 지속 오염 (쇼트/도미넌트 고착 등)**

- Bus-off 발생 시 HC_BUS_OFF(13) ≥ FATAL → **즉시 OFFLINE**(단발) → RECOVERY → 원인 지속 시
  (B) 와 동일한 순환 → FAULT. **판정: fail-safe ✓**

**(E) 모터 자체 fault (과열/과전류/락업)**

- `error_code != 0` 또는 temp ≥ 75°C → `RD_MOTOR_FAULT_ACTIVE` → **ESTOP_SW** (3A 제동)
  → 해소 + CAN health 정상 시 자동 복귀. **판정: fail-safe ✓**

### 3.2 UART1 — RC 수신기 (MANUAL 안전선)

**(A) 선 뽑힘**

- IDLE 인터럽트 정지 → `last_rx_tick` stale → **100ms 후 HC_TIMEOUT** → MANUAL rc_ok 의
  `health != HC_TIMEOUT` 조건 위반 → **motor_on=0 + CMD_CLEAR (≈100~110ms)**.
- 지속 → ~500ms 후 OFFLINE → RECOVERY 1회 → 무신호 라인은 재초기화 후 에러도 수신도 없음 →
  **READY 에서 조용히 대기** (추가 escalation 없음 — silent line 에 대한 올바른 수렴).
- 재연결 → DMA 가 이미 재무장돼 있어 RX 재개 → READY→RUNNING → motor_on 자동 복귀.

**판정: fail-safe ✓ + 자동 복귀 ✓** — 무신호 단선의 모범 경로.

**(B) 보드레이트 불일치 / 지속 노이즈**

- 수신 바이트마다 FE/NE → checker 가 틱마다 소프트 재무장(AbortReceive→Receive_DMA) + `rx_error_cnt`
  연속 증가 → 11틱(~110ms) 후 OFFLINE → RECOVERY → 오염 지속 → 반복 → fatal_uart1_cnt 순증
  (+20/이벤트 − 사이 감쇠) → **수 초 내 FATAL_MAX** → `hw.reset.bit.uart1=1` +
  **lifecycle = LS_RECOVERING 동결** (RC 는 FAULT escalation 없음 — 설계 의도).
- 동결 동안 rc_ok=0 (RUNNING/DEGRADED 아님) → motor_on=0 유지. **fail-safe ✓**
- 단, 동결 해제는 **Orin 이 addr5 에 리셋 비트를 write 해야만** 수행됨 (`RD_SYSTEM_HW_RESET_HANDLE`).
  **Orin 없이 수동 운용 중이면 원인 제거(보드레이트 수정/노이즈 해소) 후에도 자동 복구가 없다**
  → ECU 전원 재시작 필요 (→ §5-H3).
- 노이즈가 checksum 을 우연히 통과할 위험: RC 프레임 유효성은 16bit checksum 1개 의존 —
  확률 낮으나 0 아님. 통과 시 스틱값 1프레임 오염 (다음 정상 프레임이 덮음, LPF 가 완충) (→ §5-H5).

**(C) 간헐 접촉 불량**

- 끊김 구간(>100ms): HC_TIMEOUT → motor_on=0 순단 → 재개 시 motor_on 0→1 상승엣지가
  LPF 리셋 → 0 에서 재가속. 짧은 끊김(<100ms): 스틱값이 마지막 프레임으로 유지된 채 주행
  (사람이 루프 안에 있는 MANUAL 특성상 수용). 산발 FE → degraded 히스테리시스 (DEGRADED 주행 허용).

**판정: fail-safe ✓** — 최악 노출은 "마지막 정상 스틱값으로 100ms" — RC 관례상 수용 범위.

### 3.3 UART2 — RS485/Orin (AUTO 안전선)

**(A) 선 뽑힘 (AUTO 주행 중)**

- 이중 감지: ① `cmd_write_tick` 100ms 워치독 (AUTO_TIMEOUT) ② uart2 HC_TIMEOUT (100ms) —
  어느 쪽이든 rc_ok=0 → **motor_on=0 + CMD_CLEAR ≈100~110ms** (Orin 측 50ms 가드가 1차).
- 지속 → OFFLINE(≈600ms) → RECOVERY 1회 → silent line → READY 대기 (RC 와 동일한 올바른 수렴).
- 재연결 → Orin 마스터 쿼리 재개 → RUNNING 복귀 → cmd 갱신 시 motor_on 재개.

**판정: fail-safe ✓ + 자동 복귀 ✓**

**(B) 보드레이트 불일치 / 지속 노이즈 — ⚠ 최대 리스크 경로**

- (RC 와 동일하게) 연속 FE → ~110ms 주기 OFFLINE→RECOVERY 순환 → fatal_rs485_cnt 순증 →
  **수 초 내 FATAL_MAX → robot_state = FAULT + `hw.reset.bit.uart2=1`** →
  `ACTION_STATE_FAULT` 가 **`RD_REBOOT_HANDLE()` = NVIC_SystemReset 즉시 호출** →
  재부팅 → 오염 지속 → 부팅 후 다시 수 초 내 FAULT → **무한 리붓 루프**.
- 모터 관점: 리붓 직전 CAN abort + 큐 리셋 수행, 리붓 중 TX 없음 → **모터는 안전 ✓**.
- 그러나 리붓 루프 동안 **MANUAL(RC) 주행도 불가능** — RS485 물리계층 장애 하나가
  수동 운용까지 죽이는 단일 장애점 (→ §5-H2). RC 채널(동결+리셋 요청)과 대칭이 깨져 있음.

**판정: fail-safe ✓ / 가용성 ✗** — 모터는 멈추지만 전 시스템이 리붓 루프에 갇힘.

**(C) 간헐 접촉 불량**

- 순단(>100ms): 워치독+TIMEOUT 이 잡고, 복귀 시 Orin 이 cmd 재게시 → 재개. LPF 리셋 3신호
  (motor_on 상승) 로 글리치 흡수. 산발 CRC: Dynamixel CRC 가 오염 패킷 폐기 → degraded 로만 집계
  → DEGRADED 주행 유지 (정책은 Orin soft_estop 위임 — 2026-07-17 결정). **판정: fail-safe ✓**

**(D) rs485Task 자체 사망 (소프트웨어)**

- IDLE ISR 는 계속 돌아 `last_rx_tick` 이 갱신됨 → uart2 TIMEOUT 미발동. 그러나 PACKET_HANDLE
  이 없어 `cmd_write_tick` 정체 → **AUTO_TIMEOUT 100ms 가 motor_on=0** ✓. MANUAL 영향 없음 ✓.

### 3.4 UART6 — IMU (텔레메트리)

- 선 뽑힘/노이즈/보드레이트: RC 채널과 동일 경로 (동결 + addr54 리셋 요청, FAULT 없음).
  주행 게이트와 무관, `delta_tick=0xFF` 로 stale 이 Orin 에 표시됨.
- **판정: fail-safe ✓ (주행 영향 0)** — 단 fatal 동결 후 재연결 자동 복구는 역시 Orin addr5 필요.

### 3.5 I2C — AS5600 엔코더 ×5 (텔레메트리, ESTOP 비연동)

**(A) 선 뽑힘 / MUX 무응답**

- 블로킹 read 라 HAL 에러콜백 없음 — 채널별 2ms timeout 실패가 감지 경로.
  전 채널 실패 → HC_HW_FAULT → degraded 포화(~500ms) → OFFLINE → **i2c1Task 가 매 10ms 틱
  자가 RECOVERY** (DeInit → SCL 9클럭 버스클리어 → 재INIT). 라인 미복구 시 **10ms 마다 무한 반복**.
- 부하: worst 5ch×(MUX 2ms+read 2ms) ≈ 루프 지연 + 버스클리어 소요가 매 틱 발생. i2c1Task 가
  Low priority 라 다른 태스크는 격리되지만 회수 제한/백오프가 없음 (→ §5-H6).

**(B) SDA 스터크 (슬레이브 hang)** — 버스클리어 9클럭 + STOP 이 정확히 이 케이스용 → 복구 ✓.

**(C) 간헐 NACK** — 일부 채널 실패 = HC_TIMEOUT → degraded 히스테리시스, 채널별 delta_tick
stale 표시. **판정: 전 시나리오 fail-safe ✓ (주행 영향 0)** — 가용성 이슈만 존재.

### 3.6 ADC — 로드셀 (현재 제어 미사용)

- DMA/변환 정지: staleness 20ms → systemTask 가 Stop→Start 재기동 ✓.
- 단선/포화(레일 고착): HC_DATA_RANGE 보고만 — **값은 계속 발행됨**. 현재는 텔레메트리라 무해하나,
  향후 로드셀이 제어 피드백에 들어가면 레일 상태에서의 사용 차단 정책이 필요 (→ §5-H8).

### 3.7 시스템 공통

| 시나리오 | 방어 | 판정 |
|---|---|---|
| controlTask hang | hb_control 정체 → systemTask 가 IWDG refresh 중단 → MCU 리셋 | ✓ |
| systemTask hang | refresh 주체 사망 → IWDG 리셋 | ✓ |
| rcTask 만 선별적 hang | 감시 없음 — ISR 가 last_rx_tick 갱신해 TIMEOUT 미발동, stale 스틱 주행 지속 | △ (§5-H4) |
| ESTOP_HW 스위치 | GPIO 폴링 → ESTOP_HW 상태 + BRAKE | ✓ |
| AUTO↔MANUAL 전환 | ESTOP 경유 + LPF 전이 리셋 | ✓ |
| 부팅 윈도우 (INIT 전) | CHECKER 가 huart NULL → RET_WAIT (escalation 금지) | ✓ |
| soft_estop 중 Orin write 지속 | TX 는 override 제동으로 대체, 훅이 매 틱 재청소 | ✓ |

---

## 4. Fail-safe 성립 종합표

| 채널 | 선 뽑힘 | 간헐 불량 | 노이즈/보드레이트 | 부분 연결 | 자동 복구(재연결) |
|---|---|---|---|---|---|
| CAN 모터 | ✓ (FAULT ~1s) | ✓ (DEGRADED 운전) | ✓ (FAULT) | **△ 순환 고착** | FAULT 후 Orin REBOOT 필요 |
| UART1 RC | ✓ (100ms 정지) | ✓ | ✓ (동결) | — | 단선 ✓ / fatal 동결은 **✗ (Orin 필요)** |
| UART2 RS485 | ✓ (100ms 정지) | ✓ | ✓ 모터안전 / **✗ 리붓 루프** | — | 단선 ✓ / fatal 은 리붓 반복 |
| UART6 IMU | ✓ (무관) | ✓ | ✓ | — | 단선 ✓ / fatal 동결은 Orin 필요 |
| I2C 엔코더 | ✓ (무관) | ✓ | ✓ | ✓ (채널별 stale) | ✓ (자가, 단 무한 재시도) |
| ADC 로드셀 | ✓ (보고만) | ✓ | ✓ | — | ✓ (Stop→Start) |

**모터 안전(협의의 fail-safe)은 전 시나리오에서 성립한다.** 위험 명령이 버스로 나가는 구멍은
발견되지 않았음 — 모든 경로가 {motor_on=0 + CMD_CLEAR} / {ESTOP 제동} / {FAULT} / {IWDG 리셋}
중 하나로 수렴. 구멍은 전부 **가용성/거동 품질** 축에 있다 (아래).

---

## 5. 취약점 목록 (우선순위순)

- **H1. CAN 부분 연결 시 ~0.5s 주기 무한 RECOVERY 순환** — 모터 1개 탈락만으로 나머지 3개의
  TX 가 0.5초마다 순단(큐 리셋)되고 진단 카운터가 주기적으로 지워짐. FAULT 로도 안 가고 정상으로도
  안 돌아오는 준안정. *개선안: 모터별 타임아웃(comm_err)은 채널 degraded 에서 분리해 per-motor
  상태로만 발행하고, 채널 OFFLINE 판정은 HAL/버스 에러 기반으로 한정. 또는 "결선 모터 수" 설정.*
- **H2. UART2 지속 오염 → FAULT → 즉시 SystemReset → 무한 리붓 루프** — RS485 물리 장애가
  MANUAL 주행까지 봉쇄하는 단일 장애점. *개선안: `hw.reset.bit.uart2` 도 RC 처럼 "동결+리셋 요청"
  으로 바꾸거나, 리붓 전 재시도 횟수 제한/백오프 + MANUAL 가용 상태면 리붓 유예.*
- **H3. UART1/6 fatal 동결 해제가 Orin addr5 의존** — Orin 없는 수동 운용에서 원인 제거 후에도
  전원 재시작 전까지 RC 복구 불가. *개선안: RECOVERING 동결 중 저빈도(예: 5s) 자가 재시도 1회씩,
  또는 GPIO 모드 스위치 long-hold 를 로컬 리셋 트리거로 활용.*
- **H4. rcTask/개별 태스크 선별 hang 은 무감시** — ISR 가 살아 있으면 TIMEOUT 이 안 떠 stale
  스틱으로 주행 지속 가능. 실발생 확률은 낮음(개별 hang 은 대부분 전역 fault→IWDG). *개선안:
  rcTask 도 hb 카운터를 두고 systemTask 가 "RX 이벤트 있는데 hb 정체" 를 교차 검사.*
- **H5. RC 프레임 유효성이 16bit checksum 단일 의존** — 강노이즈에서 오검출 통과 확률 잔존.
  LPF + 다음 프레임 덮어쓰기가 완충이라 실위험 낮음. *개선안(선택): 스틱값 범위/변화율 sanity 클램프.*
- **H6. I2C 전 채널 실패 시 10ms 마다 버스클리어 무한 반복** — 격리는 되나 백오프 없음.
  *개선안: 연속 실패 N회 후 재시도 주기를 100ms→1s 로 지수 백오프.*
- **H7. 문서/주석 불일치 (혼동 유발)** — CLAUDE.md 표 `UART_RX_TIMEOUT_MS 500ms`(실제 100ms),
  `rd_common.h` decay 주석 "20ms 마다 −2"(실제 10ms 틱 −1), CLAUDE.md `AK_TX/RX_TIMEOUT_ERR`
  상수는 현재 코드에서 미사용 경로. *→ 문서 정정 필요.*
- **H8. ADC 레일 고착은 보고만** — 현재는 정상 정책. 로드셀이 제어 피드백으로 승격되는 시점에
  "레일 시 해당 채널 사용 금지 + 제어 축 안전값" 정책을 함께 설계할 것.

## 6. 잘 설계된 부분 (유지할 것)

- **정지의 데이터화**: 모든 비정상이 {rc_ok=0 → motor_on=0 → 훅 CMD_CLEAR} 단일 경로로 수렴 —
  분기 누락으로 인한 구멍이 구조적으로 안 생김.
- **silent-line 수렴**: 단선은 RECOVERY 1회 후 READY 대기 (무한 재시도 없음), 재연결 시 자동 복귀.
- **연속(HAL) vs 빈도(degraded) 이원화** + 4× 히스테리시스 — 간헐 불량에서 flapping 없이 열화 운전.
- **에러 IT 재무장 스로틀** (CAN, 틱당 IRQ ≤1) — 노이즈 폭주 시 IRQ 스톰 방지.
- **부팅 윈도우 보호** (huart NULL → WAIT), **RECOVERING 상태 보호** (checker skip) — race 없음.
- **워치독 이중화** (Orin 50ms 가드 + STM 100ms AUTO_TIMEOUT), IWDG 하트비트 조건부 refresh.

## 7. User Comments

- **H1. CAN 부분 연결 시 ~0.5s 주기 무한 RECOVERY 순환**
  - 사용방식
    - 현재 제어보드 사용할 때, 테스트 베드에서 단일 트랙 사용 시 모터 한개만 연결.
    - 전체 테스트 할 때는 모터 4개 연결 
  - 수정 방안
    - 전체 테스트에서는 하나의 모터가 무응답이라도 전체가 멈춰야하는 시스템이 맞음 (soft estop?)
    - 하지만 이 방식에는 단일 트랙(모터) 사용 시 테스트를 진행하지 못하는 문제가 있어서 이 부분에 대해서 선택적으로 할 수 있는 기능이 필요
    - defaut 상태에서는 4개의 모터만 작동하며, 임의로 orin에서 상위 제어 명령으로 1개로 변경 가능하게?? 그런식으로 해야될 것 같음.
- **H2. UART2 지속 오염 → FAULT → 즉시 SystemReset → 무한 리붓 루프** — RS485 물리 장애가
  MANUAL 주행까지 봉쇄하는 단일 장애점. *개선안: `hw.reset.bit.uart2` 도 RC 처럼 "동결+리셋 요청"
  으로 바꾸거나, 리붓 전 재시도 횟수 제한/백오프 + MANUAL 가용 상태면 리붓 유예.*
  - 수정 방안: 만약 모든 시스템이 정상이고 UART2만 문제일 경우, AUTO mode 인 경우에는 3초 후 리셋, Manual mode인 경우는 리붓 유예, 리셋 요청 
- **H3. UART1/6 fatal 동결 해제가 Orin addr5 의존** — Orin 없는 수동 운용에서 원인 제거 후에도
  전원 재시작 전까지 RC 복구 불가. *개선안: RECOVERING 동결 중 저빈도(예: 5s) 자가 재시도 1회씩,
  또는 GPIO 모드 스위치 long-hold 를 로컬 리셋 트리거로 활용.*
  - 그런 경우에는 사용자가 직접 GPIO 모드 스위치를 길게 눌러서 전체 system reboot을 하면되니까 괜찮아. 현행 유지.
- **H4. rcTask/개별 태스크 선별 hang 은 무감시** 
  - 이게 무슨 문제인지 모르겠는데 자세히 설명해줘, 현재 SYSTEM checker loop에서 이미 timeout 감시를 하고 있는데 뭐가 문제지?? 
- **H5. RC 프레임 유효성이 16bit checksum 단일 의존** — 강노이즈에서 오검출 통과 확률 잔존.
  LPF + 다음 프레임 덮어쓰기가 완충이라 실위험 낮음. *개선안(선택): 스틱값 범위/변화율 sanity 클램프.*
  - 굳이 이런거 까지 할 필요 없을 거 같아 현행 유지
- **H6. I2C 전 채널 실패 시 10ms 마다 버스클리어 무한 반복** — 격리는 되나 백오프 없음.
  *개선안: 연속 실패 N회 후 재시도 주기를 100ms→1s 로 지수 백오프.*
  좋아 그런 식으로 수정하자
- **H7. 문서/주석 불일치 (혼동 유발)** — CLAUDE.md 표 `UART_RX_TIMEOUT_MS 500ms`(실제 100ms),
  `rd_common.h` decay 주석 "20ms 마다 −2"(실제 10ms 틱 −1), CLAUDE.md `AK_TX/RX_TIMEOUT_ERR`
  상수는 현재 코드에서 미사용 경로. *→ 문서 정정 필요.*
  문서 정정 진행해줘
- **H8. ADC 레일 고착은 보고만** — 현재는 정상 정책. 로드셀이 제어 피드백으로 승격되는 시점에
  "레일 시 해당 채널 사용 금지 + 제어 축 안전값" 정책을 함께 설계할 것.
  - 이부분은 아직 테스트 단계니까 현행 유지

추가 점검사항, 예를 들어 전원 on 상태가 안맞았는데 데이터가 안들어온다고 fatal로 넘어가면 안되니까 내가 이미 고려하고 만든 거 같은데 한번더 점검해야하는 부분이 있어. UART 상태에서 첫데이터가 들어오기 전까지는 READY 상태로 무한대기로 있는게 맞지??

> **답변 (검증 완료)**: 맞음 — UART checker 의 timeout 판정은 `lifecycle >= LS_RUNNING` 게이트,
> READY→RUNNING 승격은 실수신 이력(`last_rx_tick != 0`) 필요 → **조용한 라인은 READY 무한대기** ✓.
> 예외 2가지: ① 라인에 노이즈가 들어오면 READY 라도 RUNNING 승격 (freeze 방지 설계 — 의도됨).
> ② **CAN 은 TX 주도라 예외** — RC enable 스위치가 켜진 채 부팅하고 모터 전원이 늦으면
> TX ACK 에러 → ~1s 내 FAULT 진입 가능. enable off 부팅이면 TX 없어 안전 대기.
> → 전원 인가 순서 "모터 전원 → ECU(또는 enable off 부팅)" 운용 수칙 필요.

---

## 8. 수정 계획 (2026-07-17 토론 확정) — ✅ P1~P4 구현 완료 (Code_modify.md 참조, CubeIDE 빌드/실기 검증 대기)

### 확정 결정

| 항목 | 결정 |
|---|---|
| H1 설정 방식 | `motor_mask` 비트필드 — **CMD_SYSTEM 확장, RSVD1 에서 1바이트 (addr 192)**, default 0x0F |
| H1 무응답 반응 | **ESTOP_SW 자동복귀형** (과열 fault 와 동일 경로, 지속 무응답 ~500ms 판정) |
| H2 AUTO | FAULT 진입 + **3초 후 SystemReset** (즉시 리붓 → 지연 리붓) |
| H2 MANUAL | **FAULT 미진입** — uart2 RECOVERING 동결 + addr54 리셋 요청만, RC 주행 유지 |
| H3 / H5 / H8 | 현행 유지 (GPIO long-hold 리붓으로 커버 / 불필요 / 테스트 단계) |
| H4 | 현행 수용 — 잔존 리스크로 문서 기록만 (개별 태스크 hang 은 IWDG 범위 밖, 실확률 낮음) |
| H6 | I2C 자가 복구 지수 백오프 (100→200→400→800→1000ms cap, 성공 시 리셋) |
| H7 | CLAUDE.md / rd_common.h 문서·주석 정정 |

> **P1 개정 (2026-07-17 후속 토론)**: AK 모터가 명령 무관 **100Hz 상시 피드백**을 송신하도록
> 설정돼 있음이 확인되어 (구 분석의 "피드백=명령 응답" 전제 정정) 무응답 판정을 재설계:
> - `RD_CAN_MOTOR_COMM_LOST`(기동 유예 on_since + last==0 분기) → **`RD_CAN_MOTOR_ALL_READY`**
>   단일 프리미티브로 교체 — "mask 전 모터 피드백이 200ms 이내" 신선도 하나로 판정
>   (tick==0 은 자연히 not-ready 에 포함, 유예 불필요)
> - **존재 게이트**: ALL_READY 를 MANUAL/AUTO motor_on 전제조건에 추가 — 모터 전원 전엔
>   TX 미개시 → 빈 버스 ACK 폭주→FAULT (§3.7 전원 순서 문제) **코드로 원천 해소**, 운용 수칙 대체.
>   늦게 켜진 모터는 피드백 즉시 자동 합류.
> - 구동 중 !ALL_READY → motor_fault → ESTOP_SW (자동복귀형, 기존 결정 유지)
> - mask 는 **의도 선언** 유지 (Orin/RS485 만 write, ECU 자동 갱신 없음 — 단일 소유 원칙.
>   "tick==0 자동 감지로 mask 대체" 안은 부팅 시 커넥터 빠진 모터를 조용히 제외한 채
>   3륜 주행하는 구멍이라 기각). 단일 트랙 테스트는 mask=0x01 write 필수 (default 0x0F 는 게이트가 막음).

### P1 — motor_mask + 무응답 ESTOP_SW + 채널 escalation 분리 (H1, 최대 작업)

1. `rd_register_ecu.h`: CMD_SYSTEM_t 에 addr 192 `motor_mask` 추가 (bit0~3 = M1~4, default 0x0F)
   → REG_CMD_SYSTEM_SIZE 12→13, RSVD1 193~223 (31B). DIAG(224~) 불변. Orin hpp 미러 + static_assert 동기화.
   mtr_lock 보호범위(132~187) 밖이라 모드 무관 write 가능 (테스트베드에서 Orin/RS485 로 0x01 설정).
2. `RD_MAP_INIT`: `motor_mask = 0x0F`.
3. `rd_can_motor.c`: TRANSMIT — mask 제외 모터 TX skip. CHECKER — mask 제외 모터를
   comm_err/worst 집계에서 제외 + **per-motor RX 타임아웃을 채널 degraded 누적에서 분리**
   (채널 DEGRADED/OFFLINE 은 HAL/버스 에러 전용 → §3.1(A) 의 0.5s RECOVERY 순환 원인 자체 소멸).
4. `rd_system.c`: 활성(mask) 모터의 지속 무응답(500ms, 첫 구동 개시 이후부터 판정 — 부팅
   전원순서 오탐 방지) → `RD_MOTOR_FAULT_ACTIVE` 계열로 **ESTOP_SW** 진입, 통신 복구 시 자동 복귀.

### P2 — uart2 fatal 모드별 처리 (H2)

1. `RD_SYSTEM_CHECKER` uart2 분기: FATAL_MAX 도달 시 —
   **AUTO**: 기존대로 FAULT 진입. **MANUAL**: FAULT 미진입, uart2 lifecycle=LS_RECOVERING 동결
   + `hw.reset.bit.uart2=1` 발행 + fatal 카운터 정지. (동결 중 AUTO 전환 시 rc_ok=0 이라 안전,
   그 상태 지속이면 FAULT 경로 합류.)
2. `ACTION_STATE_FAULT`: `hw.reset.bit.uart2` 즉시 `RD_REBOOT_HANDLE()` → **3초 경과 후 리붓**으로 변경.

### P3 — I2C 백오프 (H6)

- `RD_TASK_I2C1`: RECOVERY 연속 재시도 간격 100→200→400→800→1000ms(cap), UPDATE 성공 시 리셋.

### P4 — 문서 정정 (H7)

- CLAUDE.md: `UART_RX_TIMEOUT_MS` 500→100ms, 미사용 `AK_TX/RX_TIMEOUT_ERR` 표 정리.
- `rd_common.h`: DEGRADED_TICK_DECAY 주석 "20ms 마다 −2" → "10ms 틱마다 −1".
- 운용 수칙: 전원 인가 순서 (모터 전원 → ECU, 또는 RC enable off 부팅) 명시.

### 진행 순서

P4 (문서, 위험 0) → P3 (격리된 소규모) → P2 (FSM 분기) → P1 (레지스터 레이아웃 + Orin 동기화, 최대)
— 각 단계 후 CubeIDE 빌드 확인, P1 은 Orin colcon 빌드 + 오프라인 layout 테스트 병행. 