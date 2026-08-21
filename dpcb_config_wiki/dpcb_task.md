# DPC_B FreeRTOS 태스크 구조

> 최종 갱신: 2026-08-04 (실기 구동 검증 — LED/패널 FSM 확인, 잔여 기능 2건)  
> 상위: [dpcb_overview.md](dpcb_overview.md)

---

## 1. 태스크 일람

| 태스크 | 진입점 (main.c) | 구현 (rd_system.c) | 주기 / 트리거 | 우선순위 |
|--------|----------------|-------------------|--------------|---------|
| defaultTask | StartDefaultTask | `RD_TASK_DEFAULT` | 1000ms (LED 상태) | Normal |
| systemTask | StartSystem | `RD_TASK_SYSTEM` | **10ms** | Normal |
| controlTask | StartControl | `RD_TASK_CONTROL` | 10ms | Normal |
| rs485Task | StartRS485 | `RD_TASK_RS485` | 이벤트 (USART2 IDLE ISR) + 10ms 폴링 fallback | Normal |
| i2cTask | Start_i2c | `RD_TASK_I2C` | 10ms | Low |
| dpcaTask | StartDPCA | `RD_TASK_DPCA` | ~10ms | Normal |
| periTask | StartPeri | `RD_TASK_PERI` | Dynamixel 루프 + 1ms | Normal |

---

## 2. UART ↔ 태스크 깨우기 매핑

```
USART2 IDLE ISR → DPCB_uart2.wake_task = rs485TaskHandle  → rs485Task
USART6 IDLE ISR → DPCB_uart6.wake_task = periTaskHandle   → periTask
UART4          → wake_task 없음 (dpcaTask 폴링)
```

- IDLE ISR가 `wake_task` 플래그 set → 해당 태스크 기상 (이벤트 구동)
- UART4는 wake_task 미주입 → dpcaTask가 `osDelay(1)` 폴링으로 처리
- **rs485Task는 순수 이벤트 아님(2026-08-03)**: `osThreadFlagsWait` timeout **10ms** — IDLE 이벤트 없어도 주기 기상해 reg 발행 유지, 기상 경로 한 번 끊겨도 폴링으로 자기치유. `rx_new` 없으면 `RD_ORIN_READ` 즉시 `RET_WAIT` 라 폴링 비용 무시 수준

---

## 3. systemTask 루프 구성 (RD_TASK_SYSTEM, 10ms)

```
osDelayUntil(10ms)
  → tim_cnt 갱신 (TIM5)
  → RD_SYSTEM_HW_RESET_HANDLE()   (Orin 요청 hw_reset 처리)
  → RD_SYSTEM_CHECKER()           (3채널 Checker + Recovery)
  → RD_SYSTEM_EVALUATE_STATE()    (채널 상태 → hw.error/hw.fatal 집계)
  → payload_state == FAULT 시 ACTION_STATE_FAULT()
  → RD_MAP_MARSHAL_PUBLISH()      (상태 → reg R/O 영역 발행)
```

- Checker 세부: [dpcb_checker.md](dpcb_checker.md) 참조
- **[개정 목표]** FAULT 시 `DPC_CTL.STATE=ERROR(10)` 강제 전이 추가, PUBLISH 의 `sys_state`(57) 발행 소스를 `payload_state`→`DPC_CTL.STATE`(`DPCB_STATE_e`) 로 교체 예정. `payload_state` 는 내부 FAULT/health 처리 전용 유지. 상세: [dpcb_opmode.md](dpcb_opmode.md) §7

---

## 3-1. rs485Task 루프 구성 (RD_TASK_RS485, 이벤트+10ms) — 2026-08-03

```
osThreadFlagsWait(0x0001, 10ms)   ← USART2 IDLE ISR set 또는 10ms timeout
  → RD_MAP_MARSHAL_PUBLISH()       ← 요청 직전 재발행 (request-synchronous snapshot)
  → RD_ORIN_READ → RD_ORIN_HANDLE → RD_ORIN_WRITE
  → reboot_pending 시 TX 완료 대기 후 NVIC_SystemReset()
```

- **request-synchronous 발행**: 요청 처리 직전 PUBLISH 재호출 → 응답이 항상 요청 시점 스냅샷. 발행 주기 vs 요청 주기 비트(beat)로 생기던 중복/스테일 샘플 제거. systemTask 의 주기 발행은 유지(패널·DPC-A 등 내부 소비자 존재). 매핑 세부: [dpcb_register.md](dpcb_register.md) §3-1
- **INIT 실패 처리**: `Error_Handler`(=`__disable_irq`+`while(1)`, 보드 전체 동결) 대신 재시도 카운터 `rs485_init_fail_cnt`(0=정상, Live Watch 진단) 누적 → 10회 실패 시 `NVIC_SystemReset`. 펌웨어 결함과 배선/트랜시버 문제를 관측 가능하게 구분
- **REBOOT**: 응답 DMA TX 실제 완료(`gState==HAL_UART_STATE_READY`)+2ms 대기 후 리셋(`wr==RET_OK` 조건부) — 응답 유실 방지. [dpcb_register.md](dpcb_register.md) §4

---

## 4. rd_system.c 전역 객체 일람

```c
// UART 링버퍼
UART_Ring_t   DPCB_uart2;    // Orin RS485 백킹 UART
UART_Ring_t   DPCB_uart6;    // Dynamixel RS485 백킹 UART
UART_Ring_t   DPCA_uart4;    // DPC-A 백킹 UART

// RS485 핸들
RS485_t       DPCB_rs485;    // Orin  (.uart_obj=&DPCB_uart2, DIR=PC3)
RS485_t       DPCB_dyn;      // Dyn   (.uart_obj=&DPCB_uart6, DIR=PB15)

// 패킷 / 페리페럴 / 제어
PACKET_comm_t DPCA_PACKET;
PERIPHERAL_t  DPCB_PERIPHERAL;
CONTROL_DPC_t DPC_CTL;                   // DPC_CTL.STATE = 운용 상태 (DPCB_STATE_e 로 통일 예정, addr 57 발행원)

// 시스템 상태
volatile SYSTEM_STATE_e payload_state;  // extern — 내부 전용(FAULT/health), addr 57 발행 중단 예정
HW_ERROR_FLAG_t         hw;             // extern in rd_system.h
uint32_t                tim_cnt;        // extern in rd_system.h — TIM5 카운터 [ms]
volatile uint8_t        rs485_init_fail_cnt;  // extern — USART2 INIT 재시도 횟수(0=정상), 부팅 진단 (2026-08-03)
```

---

## 5. controlTask — `rd_control.c` 제어 루프 구현 현황 (2026-07-21 코드 기준)

`RD_CONTROL_LOOP(CTL, GPIO)` 가 10ms 주기로 `DPC_CTL`(=`CONTROL_DPC_t`, `.MODE`/`.STATE`) 기반 구동. 상태 정의·전이·버튼은 [dpcb_opmode.md](dpcb_opmode.md) §3~5.

### 5-1. 구현 완료
| 영역 | 내용 |
|------|------|
| **mode 라우팅** | `mode=0`→`CASE_IDLE`, `mode=1`→`STATE` switch (`if/else if`) |
| **MANUAL (`CASE_IDLE`)** | 패널 직접 제어 — SW2=A·B locker EN(`TIMEOUT_SOL`), SW6=서보 lock/unlock, SW3/4/5=윈치 A/B/C 승강(`RD_MOT_FORCE_DRIVE`) |
| **FSM 본체** | `CASE_INIT` / `DESCEND_1·2` / `WAIT` / `ASCEND_1·2` / `FINISH` — 각 액션 함수 + 센서·타임아웃 전이 |
| **ERROR (`CASE_ERROR`)** | torque off(lockA/B off), 자동복귀 제거 → latch. ⚠ **현재 default(=ERROR) 의 `CASE_ERROR` 호출 주석처리**(`rd_control.c:294`, 브링업 중 latch 억제) → **재활성 TODO**(plan §3-5) |
| **상태값** | `DPCB_STATE_e`(CTRL0/HOLD1/INIT2~FINISH8/RSVD9/ERROR10) enum 치환, 구 `ERROR_STATE` 제거 |
| **센서 마스크** | `rd_control.h` 상수 (2026-08-04 실기 정정) — PROX **접촉=0(active-low)**(`A_PROX_ALL_ON=0x00`/`ALL_OFF=0x07`), CON 잠금=1(`CONT_ALL_LOCK=0xF0`/`ALL_UNLOCK=0x00`) |
| **인디케이팅 LED** | LED1=MODE(MANUAL blink/AUTO solid), LED2=컨텍스트(MANUAL=locker EN / AUTO=STATE). **LED2 단일소유 확정**(CASE_IDLE 의 LED2 write 주석 제거, defaultTask 단독). LED flush=i2cTask `RD_EXIO_UPDATE`(10ms) |
| **상태 발행** | `DPC_CTL.STATE` → addr57 `sys_state` 단방향 (PUBLISH, §3-1 개정목표) |

### 5-2. 의도적 미구현 (설계 확정 — 현 버전 정상, 상세 [plan.md](plan.md) §3-1·§3-4)
| 항목 | 사유 |
|------|------|
| `RD_CONTROL_CASE_CTRL` (mode1·STATE0) | 원격제어 미포함 버전 → **빈 함수**. Orin failsafe 진입점(차기) |
| `RD_CONTROL_CASE_HOLD` (STATE1) | 장력 pos 기반 위치홀드(`CASE_INIT` 유사) 예정, TODO. 진입 판정 `CON_DATA==0xF0` |
| `RD_MAP_MARSHAL_CONSUME` | Orin 무관 정상동작 1차 검증 후 방향 결정 |
| Orin `sys_state_target`→STATE 경로 | CONSUME 미구현이라 미연결 |

> 잔여 미해결 TODO 는 [plan.md](plan.md) §3-4 참조.

### 5-3. 제어 가능 범위 현황 (2026-08-04 실기 구동 검증 반영)

| 경로 | 상태 | 비고 |
|------|------|------|
| **Orin RO 읽기** (PUBLISH) | ✅ 전량 가능 | 모터/센서/패널 상태 발행 |
| **Orin 제어 (쓰기)** | ❌ 전부 불가 | `RD_MAP_MARSHAL_CONSUME` 빈 스텁(`rd_map_dpcb.c:261`) — mode/servo/locker/boot/light/motor 미연결. **잔여 기능 ②** |
| **패널 제어 — locker(A·B), 서보, 윈치×3, mode/FSM** | ✅ **실기 확인 완료(2026-08-04)** | `CASE_IDLE`(SW2/SW6/SW5·4·3) + SW1/SW2 전이. 패널 FSM 구동 물리 검증 |
| **인디케이팅 LED (LED1/LED2)** | ✅ **실기 확인 완료** | EXIO IODIR 초기화 순서 버그 수정 후 출력 정상 (history §Session 22) |
| **`LIGHT_EN` (WAIT 상태 작업등)** | 🟡 **코드 구현·실기 미검증** | `CASE_WAIT`→`LIGHT_EN=1`(`rd_control.c:506`)→출력단(`:212`)→`LIGHT_IO` MCU 핀. **LIGHT 결선 미완으로 미확인**(opmode §3) |
| **패널 제어 — `BOOT_EN` 계열**(`EN_BOOT`/`A_EN_BOOT`/`B_EN_BOOT`) | ❌ 경로 없음 | CONSUME 전용. memo상 boot 는 부가기능 |

- **요약**: 패널 기반 FSM·수동 페리페럴·인디케이팅 LED **실기 구동 검증 완료**. WAIT 작업등(`LIGHT_EN`)은 **코드 구현 완료·결선 후 검증 대기**. **잔여 주요 미구현 = RS485 CONSUME 1건**(Orin 제어 + BOOT 부가기능 동반 개방).
