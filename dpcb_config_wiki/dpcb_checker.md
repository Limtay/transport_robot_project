# DPC_B Checker / Failsafe 조건 및 디버깅 참조

> 최종 갱신: 2026-07-03  
> 상위: [dpcb_overview.md](dpcb_overview.md)  
> Checker 구현 완료 (Step 6). 아래 시나리오 = 실제 동작 기준.

---

## 1. Checker 공통 상수

```
UART_RX_TIMEOUT_MS      =  100 ms    마지막 수신 후 이 시간 초과 → HC_TIMEOUT
UART_FATAL_CNT_TH       =   10 회    연속 HAL 에러 누적 → 즉시 LS_OFFLINE
TX_TIMEOUT              =   10 ms    RS485 DIR 핀 TX 상태 최대 유지 시간
DEGRADED_K_100HZ        =   20       에러 1 tick당 degraded_cnt 증가 (10ms 주기)
DEGRADED_TICK_DECAY     =    1       정상 1 tick당 degraded_cnt 감소
DEGRADED_THRESHOLD_HIGH =  200       이 값 이상 → LS_DEGRADED
DEGRADED_THRESHOLD_LOW  =   50       이 값 이하 → LS_RUNNING 복귀
DEGRADED_CNT_MAX        = 1000       LS_DEGRADED 지속 중 이 값 도달 → LS_OFFLINE
FATAL_K                 =   20       RD_RS485_CHECKER RET_NOK 1회당 fatal_cnt 증가
FATAL_MAX               =  200       fatal_cnt 도달 → SYS_STATE_FAULT
```

---

## 2. lifecycle 전이 흐름

```
LS_INIT(0)
  │  RD_UART_INIT 성공
  ▼
LS_READY(1) ──── 첫 수신 OR 첫 HAL 에러 ───────────────────────────────
  ▼
LS_RUNNING(2) ◄──── degraded_cnt ≤ 50 (자동 복구)
  │ degraded_cnt ≥ 200
  ▼
LS_DEGRADED(3) ◄─── degraded_cnt ≤ 50 (자동 복구)
  │ degraded_cnt = 1000  OR  HAL 에러 10회 연속  OR  DMA 에러
  ▼
LS_OFFLINE(15) → Checker RET_NOK → systemTask: RECOVERY 호출
  │
  ▼
LS_RECOVERING(4) → 성공: LS_READY 리셋  /  실패: LS_OFFLINE + HC_FATAL
```

**HC_* (health code) 발생 조건:**

| 조건 | health | degraded_cnt | rx_error_cnt |
|------|--------|--------------|--------------|
| HAL DMA 에러 | HC_HW_FAULT | +K | = FATAL_TH+1 (즉시 OFFLINE) |
| HAL ORE/PE/FE/NE | HC_FRAMING_ERR 계열 | +K | ++ |
| 패킷 CRC 불일치 | HC_CRC_ERR | +K | 변동 없음 |
| 수신 없음 100ms 초과 | HC_TIMEOUT | +K | 변동 없음 |
| 정상 수신 | HC_OK | -1 (decay) | 0 리셋 |

---

## 3. 채널별 FAULT 정책

| 채널 | FAULT 진입 | 재부팅 조건 | 비고 |
|------|-----------|------------|------|
| DPCB_rs485 (USART2) | `fatal_rs485_cnt ≥ 200` → `payload_state = SYS_STATE_FAULT` | `RS485_TEST_ON` **해제 시** `NVIC_SystemReset()` | ECU_V3 동일 정책 |
| DPCA_uart4 (UART4) | **없음** | **없음** — `hw.reset.bit.uart4=1` + LS_RECOVERING → Orin 주도 복구 | |
| DPCB_dyn (USART6) | **없음** | **없음** — 채널 레벨 hw.error/hw.fatal 집계만. 개별 모터는 `dyn_ctrl.error.comm_flag`로 사용자 처리 | 1개라도 응답 시 채널 유지 |

> **현재 `RS485_TEST_ON`이 `rd_system.h:29`에 정의됨** → FAULT 진입해도 재부팅 차단 상태  
> 실제 운용 시 이 define 제거하면 USART2 FAULT 시 자동 재부팅 활성화

**채널별 복구 흐름:**

```
[DPCB_rs485] — ECU_V3 동일
  Checker RET_NOK → fatal_rs485_cnt += 20 → RECOVERY 시도
  RECOVERY 실패 시 fatal_rs485_cnt += 20 추가 (tick당 최대 +40)
  Checker RET_OK  → fatal_rs485_cnt -= 1 (decay)
  fatal_rs485_cnt ≥ 200 → SYS_STATE_FAULT → ACTION_STATE_FAULT (토크 OFF → 재부팅)

[DPCA_uart4] — Orin 주도 복구
  Checker RET_NOK → fatal_uart4_cnt += 20 → RECOVERY 시도
  fatal_uart4_cnt ≥ 200 → hw.reset.bit.uart4 = 1 + LS_RECOVERING 표시
  FAULT 전이 없음. Orin이 reg.sys.hw_fatal 확인 → addr5 hw_reset 비트 WRITE
  → RD_SYSTEM_HW_RESET_HANDLE → RD_UART_RECOVERY + fatal_uart4_cnt = 0

[DPCB_dyn] — 채널 레벨 감시만
  Checker RET_NOK → fatal_rs485ex_cnt += 20 → RECOVERY 시도
  FAULT / hw.reset 없음. EVALUATE_STATE가 uart6 상태 → hw.error/hw.fatal 집계
  개별 모터 탈락: dyn_ctrl.error.comm_flag = 1 (Checker 미감지, 사용자 처리)
```

---

## 4. 케이블 제거만으로는 FAULT 미진입 (공통)

- RECOVERY = `HAL_UART_Abort → DeInit → Init → DMA 재시작`
- 케이블 유무 무관하게 HAL 재초기화 자체는 성공 → fatal_cnt 첫 OFFLINE 시 +20 후 decay
- **FAULT 도달 실제 원인:**
  - HAL_UART_Init 연속 실패 = DMA 컨트롤러 / 레지스터 수준 HW 결함
  - 또는 **보드레이트 다른 장치 연결** → 연속 FE/NE 에러 → 10회 OFFLINE 사이클 → fatal_cnt 200 → FAULT (약 2초)

---

## 5. 시나리오별 에러 진행

### A. Orin RS485 케이블 처음부터 미연결

```
→ LS_READY 고착 (last_rx_tick=0, 에러 없음)
  이유: LS_READY→RUNNING 승격 조건 = 첫 수신 OR 첫 HAL 에러 — 케이블 없으면 둘 다 없음
  hw.error.uart2 = 0, reg.uart2.state = LS_READY
```

> 디버깅 주의: 에러 없는 게 정상 아님. 케이블 없이 부팅하면 조용히 대기

### B. Orin RS485 연결 후 케이블 뽑힘

```
t=0ms    마지막 패킷 수신
t=100ms  HC_TIMEOUT → degraded_cnt +20/tick
t=200ms  degraded_cnt ≥ 200 → LS_DEGRADED → hw.error.bit.uart2 = 1
t=600ms  degraded_cnt = 1000 → LS_OFFLINE → hw.fatal.bit.uart2 = 1
         Checker RET_NOK → RECOVERY → LS_READY / fatal_rs485_cnt += 20
t=600ms~ RECOVERY 성공 → LS_READY 고착 (케이블 없으면 last_rx_tick=0)
         Checker RET_OK → fatal_rs485_cnt -1 (decay) → 재차 OFFLINE 사이클 없음

FAULT 미진입. hw.reset.bit.uart2 = 1 표시 후 대기
```

### C. DPCA 케이블 처음부터 미연결

```
→ LS_READY 고착 (시나리오 A와 동일 메커니즘)
dpcaTask: TX DMA 성공 (케이블 무관), RX 없음 → last_rx_tick = 0 고착
hw.error.uart4 = 0, hw.fatal.uart4 = 0
```

### D. DPCA 연결 후 케이블 뽑힘

```
t=100ms  HC_TIMEOUT → degraded_cnt 상승
t=200ms  LS_DEGRADED → hw.error.uart4 = 1
t=600ms  LS_OFFLINE → hw.fatal.uart4 = 1 → RECOVERY → LS_READY
FAULT 없음
```

### E. Dynamixel — 1개 연결, 나머지 2개 미연결

```
연결된 1개 응답 → IDLE ISR → last_rx_tick 주기 갱신 → HC_TIMEOUT 없음
DPCB_dyn 채널: LS_RUNNING 유지 → hw.error.uart6 = 0

미연결 2개:
  RD_DYN_LOOP: TX → 2ms 대기 → rx_new=0 → RET_NOK
  MOT[i].dyn_ctrl.error.comm_flag = 1  ← 관찰 지점
  MOT[i].dyn_ctrl.is_running = 0

→ UART Checker 레벨에서 개별 모터 탈락 미감지. Step 7 MARSHAL_PUBLISH 구현 후 reg.motor_data로 발행 가능
```

### F. Dynamixel — 연결된 1개 케이블 뽑힘 (동작 확인 완료 ✓)

```
t=0ms    마지막 Dynamixel 응답
t=100ms  HC_TIMEOUT → degraded_cnt +20/tick
t=200ms  LS_DEGRADED → hw.error.bit.uart6 = 1  (reg.sys.hw_error = 0x04)
t=600ms  LS_OFFLINE  → hw.fatal.bit.uart6 = 1
         Checker RET_NOK → RECOVERY → LS_READY
         hw.error.raw = 0x00 (LS_READY = HC_OK)

케이블 재삽입 → Dynamixel 응답 → LS_RUNNING 승격 → 정상 동작 재개

FAULT 없음. hw.reset 없음 (dyn 채널 정책)
```

> **확인된 동작 (2026-06-25)**: 케이블 뽑기 → `hw.error.raw = 4` → 0 (LS_READY) → 재삽입 → Dynamixel 재탐지 정상 확인

### G. HAL 하드웨어 에러 연속 (ORE / FE / NE)

```
HAL_UART_ErrorCallback → isr_err_code 저장
Checker: rx_error_cnt++ → 재무장(DMA AbortReceive → Re-DMA) 시도
10회 연속 재무장 실패 → rx_error_cnt > 10 → 즉시 LS_OFFLINE + HC_FATAL
                       → hw.fatal.uart2/4/6 해당 비트 set

DPCB_rs485(USART2) 해당 시:
fatal_rs485_cnt += 20 → 10회 OFFLINE 사이클 ≈ 2초 → fatal_cnt 200 → SYS_STATE_FAULT
RS485_TEST_ON 해제 상태이면 → NVIC_SystemReset() (재부팅)

재부팅 발생했다면: HAL 연속 에러 + USART2 채널이 원인.
보드레이트 다른 장치를 USART2에 연결하면 이 경로로 재부팅
```

---

## 6. 디버깅 관찰 포인트

| 변수 | 정상값 | 이상 징후 |
|------|--------|----------|
| `DPCB_uart2.error.state` | 0x02 (LS_RUNNING, HC_OK) | 0x30 이상 = DEGRADED |
| `DPCA_uart4.error.state` | 0x02 (연결 시) / 0x01 (미연결) | 0x30 이상 = 경보 |
| `DPCB_uart6.error.state` | 0x02 (Dyn 1개 이상 응답 시) | 0x30 이상 = 경보 |
| `MOT[i].dyn_ctrl.error.comm_flag` | 0 | 1 = 해당 모터 무응답 |
| `fatal_rs485_cnt` | 0 | 증가 중 = OFFLINE 반복 |
| `hw.error.raw` | 0 | bit0=UART2, bit1=UART4, bit2=UART6 |
| `hw.fatal.raw` | 0 | 비트 set = 해당 채널 LS_OFFLINE |
| `reg.sys.hw_error` | 0 | hw.error.raw 그대로 (MARSHAL_PUBLISH 반영) |
| `payload_state` | 0 (INIT) | 5 (FAULT) = RS485_TEST_ON 해제 시 재부팅 임박 |

**STATE_t 값 해석 (uint8_t = health[7:4] | lifecycle[3:0]):**

```
0x01 = LS_READY   + HC_OK   (연결 없음 / 초기화 직후)
0x02 = LS_RUNNING + HC_OK   (정상)
0x32 = LS_RUNNING + HC_TIMEOUT
0x03 = LS_DEGRADED + HC_OK  (오염 누적 중)
0xF2 = LS_RUNNING + HC_FATAL (즉시 OFFLINE 직전)
```
