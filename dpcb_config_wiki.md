# DPC_B 시스템 구성 위키

> 작성: Claude (Sonnet 4.6) | 최종 갱신: 2026-07-03  
> 목적: DPC_B 펌웨어 신규 세션 참조 및 디버깅 가이드.

---

## 1. 시스템 개요

DPC_B는 **3축 전동 윈치 크레인**이다. Orin(마스터)의 RS485 명령을 받아 Dynamixel 윈치 3개를 구동하고, 집게 장치(DPC_A)와 연동해 화물을 자동 전개한다.

```
[Jetson Orin AGX]
       │  RS485 / Dynamixel 2.0-like 패킷 (USART2, 921600 bps)
       ▼
   [DPC_B : STM32F446RET6]
       │                    │                    │
  UART4 (4-byte pkt)   USART6 (RS485)        I2C1
       │                    │                    │
   [DPC_A]          [Dynamixel × 3]          [MCP23017]
   집게/그리퍼         윈치 모터 3개           패널 스위치
```

### 통신 채널 일람

| 채널 | 속도 | 대상 | 방향 제어 | 전담 태스크 |
|------|------|------|----------|------------|
| USART2 | 921600 | Orin RS485 | DIR=PC3, TC→RX 자동 복귀 | `rs485Task` |
| UART4 | - | DPC-A 4-byte 패킷 | 단방향 TX+RX | `dpcaTask` |
| USART6 | 57600 | Dynamixel × 3 | DIR=PB15, 동일 패턴 | `periTask` |
| I2C1 | 400 kHz | MCP23017 패널 | - | `i2cTask` |

---

## 2. 페리페럴 구조

### 2-1. Dynamixel XM430-W350 (윈치 × 3)

- ID: MOT[0]=2, MOT[1]=3, MOT[2]=4
- 제어 모드: `DYN_MODE_CUR_POSITION` (전류 기반 위치 제어)
- 초기 전류 제한: 750 (×2.69 mA ≈ 2.0 A)

**구조체 계층:**

```
PERIPHERAL_t  DPCB_PERIPHERAL
  └─ PERIPHERAL_MOT_t  MOT[3]
       ├─ int32_t   CTL_SPEED / INIT_POS / TARGET_POS
       ├─ int16_t   LPF_CURRENT
       ├─ uint8_t   DYN_IDS  (2 / 3 / 4)
       └─ DYN_Ctrl_t  dyn_ctrl
            ├─ ram.cmd.goal_position / goal_current / goal_velocity
            └─ ram.state.present_position / velocity / current / temperature / hardware_error_status
```

**periTask 루프 패턴 (모터 1개당 5회 RD_DYN_LOOP):**

```c
RD_DYN_UPDATE_STATE(dyn)    → RD_DYN_LOOP(...)   // present 상태 READ
RD_DYN_UPDATE_HWERROR(dyn)  → RD_DYN_LOOP(...)   // HW 에러 READ
RD_DYN_UPDATE_CMD(dyn, mode)→ RD_DYN_LOOP(...)   // goal 명령 WRITE
RD_DYN_OPERATE_ON(dyn, mode)→ RD_DYN_LOOP(...)   // 모드 설정
RD_DYN_TORQUE_ON(dyn, 1)    → RD_DYN_LOOP(...)   // 토크 ON
```

`RD_DYN_LOOP` 내부: `osThreadFlagsWait(0x0001, ..., 2ms)` — USART6 IDLE ISR → periTask 깨우기.

---

### 2-2. DPC-A (집게/그리퍼)

- UART4, 4-byte simple packet (`PACKET_s_t`)
- 송신: `lock_en` (집게 잠금/해제), `boot_en` (부가 기능 — 주목적과 무관, 현재 크게 고려하지 않음)
- 수신: `prox_contact` (지면 근접 3bit), `lock_contact` (DPC-A 측 집게 접점 4bit)

> **lock_contact 구분 주의**: DPC-A의 `lock_contact`(수신)은 집게 자체의 접점 상태이고,  
> DPC-B의 `lock_contact`(GPIO, 섹션 2-4)는 집게가 크레인(DPC-B) 끝단에 도달해 **고정됐는지** 확인하는 센서다.  
> auto FSM의 상승 완료 판정에는 DPC-B 측 `lock_contact`가 사용된다.

```
PACKET_comm_t  DPCA_PACKET  →  tx(Header + Data[2] + Checksum) / rx

PERIPHERAL_t  DPCB_PERIPHERAL
  ├─ A_CON_DATA   (DPC-A 수신: 집게 잠금 접점 4bit)
  ├─ A_PROX_DATA  (DPC-A 수신: 근접센서 3bit)
  ├─ A_EN_ALL / A_EN_BOOT  (DPC-A 송신)
```

**dpcaTask:** `RD_PACKET_WRITE → 폴링 → RD_PACKET_READ → RD_DPCA_UPDATE`

---

### 2-3. MCP23017 패널 (I2C)

| 포트 | 항목 |
|------|------|
| GPIOA | LED1(GPA0), LED2(GPA1), SW3B~SW6B(GPA2~5) |
| GPIOB | SW6A~SW1(GPB2~7) |
| SPST 스위치 (SW1, SW2) | 0=OFF, 1=ON |
| SPDT 스위치 (SW3~SW6) | 0=UP, 1=IDLE, 2=DOWN |

**i2cTask:** `RD_EXIO_UPDATE(&DPCB_PERIPHERAL.PANEL)` 10ms 주기.

---

### 2-4. GPIO / 솔레노이드

```
PERIPHERAL_t  DPCB_PERIPHERAL
  ├─ EN_ALL / EN_BOOT / CON_DATA (4bit) / SERVO_EN / LIGHT_EN
  └─ PERIPHERAL_IO_ALL_t  IO  (BOOT_IO, EN_IO, CON_A/B/C/D_IO, SERVO_IO, LIGHT_IO)
```

- **DPC-B `lock_contact` (CON_DATA)**: 집게(DPC-A)가 크레인 끝단에 올라와 **고정됐는지** 확인하는 접점 센서 (4bit). auto FSM descend_1 → descend_2, ascend_2 → finish 전이 판정에 사용.
- **lock_en (EN_ALL)**: 위 고정을 해제하는 솔레노이드. descend_1 진입 시 동작.

periTask 루프 끝에서 `RD_PERIPHERAL_READ` + `RD_PERIPHERAL_WRITE` 호출.

---

## 3. FreeRTOS 태스크 구조

| 태스크 | 진입점 (main.c) | 구현 (rd_system.c) | 주기 / 트리거 | 우선순위 |
|--------|----------------|-------------------|--------------|---------|
| defaultTask | StartDefaultTask | `RD_TASK_DEFAULT` | 1000ms (LED 상태) | Normal |
| systemTask | StartSystem | `RD_TASK_SYSTEM` | **10ms** | Normal |
| controlTask | StartControl | `RD_TASK_CONTROL` | 10ms | Normal |
| rs485Task | StartRS485 | `RD_TASK_RS485` | 이벤트 (USART2 IDLE ISR) | Normal |
| i2cTask | Start_i2c | `RD_TASK_I2C` | 10ms | Low |
| dpcaTask | StartDPCA | `RD_TASK_DPCA` | ~10ms | Normal |
| periTask | StartPeri | `RD_TASK_PERI` | Dynamixel 루프 + 1ms | Normal |

**UART ↔ 태스크 깨우기 매핑:**

```
USART2 IDLE ISR → DPCB_uart2.wake_task = rs485TaskHandle  → rs485Task
USART6 IDLE ISR → DPCB_uart6.wake_task = periTaskHandle   → periTask
UART4          → wake_task 없음 (dpcaTask 폴링)
```

---

## 4. 전역 객체 일람 (rd_system.c)

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
CONTROL_DPC_t DPC_CTL;

// 시스템 상태
volatile SYSTEM_STATE_e payload_state;  // extern in rd_system.h
HW_ERROR_FLAG_t         hw;             // extern in rd_system.h
uint32_t                tim_cnt;        // extern in rd_system.h — TIM5 카운터 [ms]
```

---

## 5. 운용 모드와 deploy_fsm

### mode (CMD_DPCB.mode, addr 126)

| 값 | 이름 | 상세 동작 |
|----|------|----------|
| 0 | IDLE | 패널 스위치로 윈치 3개 상승/하강을 수동 제어. DPC-A / DPC-B의 lock_en도 패널로 직접 제어. |
| 1 | HOLD | 윈치 3개를 현재 위치 기준 **포지션 모드**로 고정. DPC-B locker의 보조 역할 — 화물 하중이나 진동 시 윈치가 밀리지 않도록 잡아줌. |
| 2 | AUTO | Orin 명령 또는 패널 스위치로 진입. deploy_fsm 에 따라 하강→물건 내려놓기→상승→고정 시퀀스를 자동 실행. |

> mode 0 / 1 전환은 즉시 적용. mode 2(AUTO) 진입은 deploy_fsm이 INIT(0)부터 시작되며, FINISH 후 자동으로 mode 0 또는 1로 복귀한다.

---

### deploy_fsm (CMD_DPCB.deploy_fsm, addr 127) — AUTO 모드 전용

```
[INIT(0)] → [DESCEND_1(1)] → [DESCEND_2(2)] → [WAIT(3)]
                                                    │ Orin 기입 (4)
                                                    ▼
                              [FINISH(6)] ← [ASCEND_2(5)] ← [ASCEND_1(4)]

오류 발생 시 → [ERROR(99)] → mode 0 or 1 복귀
```

| 값 | 이름 | 동작 | 전이 조건 |
|----|------|------|----------|
| 0 | INIT | 윈치 위치 초기화 + 저전류 모드로 줄 당김 | 시간 경과 후 모터 속도 합이 충분히 작아질 것 |
| 1 | DESCEND_1 | DPC-B lock 해제(lock_en) + 저속 하강 | DPC-B `lock_contact` 전체 해제 확인 |
| 2 | DESCEND_2 | lock 동작 종료 후 고속 하강 | `prox_contact`로 지면 접촉 확인 |
| 3 | WAIT | 지면 접촉 확인 후 DPC-A lock 해제 + 대기 | **전이 조건 없음** — Orin이 deploy_fsm에 `4`(ASCEND_1)를 직접 기입해야 전이. *(타임아웃 수십 초 추가 예정)* |
| 4 | ASCEND_1 | 상승 시작 | `prox_contact` 전체 해제 + 줄 장력 확인 |
| 5 | ASCEND_2 | 고속 상승 | DPC-B `lock_contact` 전체 확인 (집게 재고정) |
| 6 | FINISH | 페리페럴 정리 후 mode → 0 or 1 복귀 | 자동 전환 |
| 99 | ERROR | 전류 공급 중인 lock 전체 중지 + 모터 position 현위치 고정 + `sys_state`에 에러 전달 + mode 0 or 1 안전 복귀 | auto FSM 내 실패 조건 발생 시 |

---

## 6. 레지스터 맵 (REGISTER_t, 256 bytes)

파일: `DPC_B/Core/Inc/rd_register_dpcb.h`

| 주소 | 크기 | 섹션 | typedef | R/W |
|------|------|------|---------|-----|
| 0~15 | 16 | DEFINE | `DEFINE_t` | R/W (sys_write_mode 잠금키 필요) |
| 16~45 | 30 | RSVD0 | - | R/O |
| 46~61 | 16 | SYSTEM | `DATA_SYSTEM_t` | R/O |
| 62~64 | 3 | SENSOR/DPCA | `DATA_SENSOR_DPCA_t` | R/O |
| 65 | 1 | UART2/data | `DATA_UART2_t` | R/O |
| 66~73 | 8 | GPIO/data | `DATA_SENSOR_DPCB_t` | R/O |
| 74~101 | 28 | MOTOR/data | `DATA_MOTOR_DYN_t` | R/O |
| 102~119 | 18 | RSVD1 | - | R/O |
| 120~121 | 2 | CMD/DPCA | `CMD_DPCA_t` | R/W |
| 122~127 | 6 | CMD/DPCB | `CMD_DPCB_t` | R/W |
| 128~142 | 15 | CMD/MOT | `CMD_MOT_t` | R/W (AUTO 모드 시에만 허용) |
| 143~174 | 32 | RSVD2 | - | R/O |
| 175~206 | 32 | DIAG | `DIAG_t` | R/O |
| 207~255 | 49 | RSVD3 | - | R/O |

### 주요 필드

**SYSTEM (addr 46~61):**

| 필드 | 타입 | 내용 |
|------|------|------|
| `degraded_cnt[8]` | uint8_t[] | 통신 오염도 % — [0]=uart2, [1]=uart4, [2]=uart6, [3]=i2c(미사용), [4~7]=RSVD |
| `hw_reset` | uint8_t | bitfield: bit0=UART2, bit1=UART4, bit2=UART6, bit3=I2C |
| `hw_fatal` | uint8_t | 동일 비트 배치 — 채널 LS_OFFLINE 시 set |
| `hw_error` | uint8_t | 동일 비트 배치 — 채널 HC_WARN 이상 시 set |
| `sys_state` | uint8_t | `SYSTEM_STATE_e` (0=INIT~5=FAULT) |
| `realtime_tick` | uint32_t | TIM5 카운터 — DPC_B 가동 시간 [ms] |

**CMD/MOT (addr 128~142):**  
`torque_en[3]`, `goal_position[3]` (int16_t), `goal_current[3]` (int16_t, default=750)

---

## 7. MARSHAL 매핑 (rd_map_dpcb.c)

### MARSHAL_PUBLISH: PERIPHERAL → reg R/O 영역 (systemTask 10ms 주기)

**SYSTEM 영역 — 구현 완료:**
```
payload_state              → reg.sys.sys_state
tim_cnt                    → reg.sys.realtime_tick
hw.reset/fatal/error.raw   → reg.sys.hw_reset / hw_fatal / hw_error
deg_pct(uart2/4/6)         → reg.sys.degraded_cnt[0/1/2]
```

**나머지 영역 — Step 6 예정:**
```
PERIPHERAL.MOT[i].dyn_ctrl.ram.state.*  → reg.motor_data.*  (position/velocity/current/temp/hw_error)
DPCB_uart2/4/6.error.state              → reg.uart2.state / sensor_dpca.uart4_state / motor_data.uart6_state
PERIPHERAL.A_PROX_DATA / A_CON_DATA     → reg.sensor_dpca.prox_contact / lock_contact
PERIPHERAL.CON_DATA                     → reg.sensor_dpcb.lock_contact
PERIPHERAL.PANEL.SW{1~6}_state          → reg.sensor_dpcb.ex_sw[0~5]
```

### MARSHAL_CONSUME: reg cmd 영역 → PERIPHERAL (Step 6 예정)

```
reg.cmd_dpca.*       → DPCA_PACKET.tx.Data  (dpcaTask가 전송)
reg.cmd_dpcb.*       → PERIPHERAL.EN_*/SERVO_EN/LIGHT_EN + DPC_CTL (모드·FSM 전환)
reg.cmd_mot.*        → PERIPHERAL.MOT[i].dyn_ctrl.ram.cmd.*
```

---

## 8. 파일 구조

```
stm_ws/DPC_B/Core/
  Inc/
    rd_system.h          전역 객체 extern, 타입 정의, 태스크 프로토타입
    rd_register_dpcb.h   레지스터 맵 구조체
    rd_uart.h            UART_Ring_t (wake_task 포함), RS485_t
    rd_peripheral_dpcb.h PERIPHERAL_t, PERIPHERAL_MOT_t
    rd_map_dyn.h         DYN_Ctrl_t, RD_DYN_LOOP
    rd_map_dpcb.h        REGISTER_t reg, DISPATCH/MARSHAL 프로토타입
    rd_comm_orin.h       ORIN_COMM_t, RD_ORIN_* (Orin RS485 패킷 레이어)
    rd_comm_dpcb.h       PACKET_s_t, PACKET_comm_t (DPC-A 4-byte)
    rd_control.h         CONTROL_DPC_t, RD_CONTROL_LOOP
    DYN_xm430_w350.h     Dynamixel 컨트롤 테이블 주소/크기 상수

  Src/
    rd_system.c          전역 객체, RD_SYSTEM_INIT, RD_TASK_* 구현
    rd_map_dpcb.c        DISPATCH_WRITE/READ, MARSHAL_PUBLISH/CONSUME
    rd_uart.c            UART 드라이버 (CHECKER / RECOVERY 포함)
    rd_peripheral_dpcb.c RD_PERIPHERAL_INIT/READ/WRITE, RD_DPCA_UPDATE
    rd_map_dyn.c         Dynamixel 루프/패킷 레이어
    rd_control.c         RD_CONTROL_LOOP, deploy FSM
    stm32f4xx_it.c       ISR (USART2/4/6 TC+IDLE 핸들러)
    main.c               CubeMX 생성, 태스크 생성
```

---

## 9. 구현 상태 (2026-06-25 기준)

| 항목 | 상태 |
|------|------|
| Step 1: 엑셀 레지스터 맵 | 완료 |
| Step 2: rd_register_dpcb.h | 완료 |
| Step 3: 엑셀 교차 검증 | 사용자 검증 예정 |
| Step 4: 통신부 리팩터링 (wake_task, USART2 ISR) | 완료 |
| Step 5: Orin RS485 modbus 통합 (rd_comm_orin, rd_map_dpcb, DISPATCH) | 완료 |
| **Step 7: RD_TASK_SYSTEM checker + MARSHAL SYSTEM 영역** | **완료 (2026-06-25, 동작 확인)** |
| Step 6: MARSHAL_PUBLISH/CONSUME 나머지 영역 | 예정 (사용자 작성) |
| Dynamixel 통신 | 동작 확인 완료 |
| DPC-A 통신 | 동작 확인 완료 |
| MCP23017 패널 | 동작 확인 완료 |
| Orin RS485 (USART2) | 동작 확인 완료 |

### 세션 히스토리

**[Session 4]** 통신부 리팩터링 (Step 4)
- `UART_Ring_t.wake_task` 추가, USART2 ISR 활성화
- rs485Task / periTask 독립 이벤트 라우팅 확인

**[Session 9]** `rd_register_dpcb.h` 작성 (Step 2)
- 256-byte `REGISTER_t` packed struct, 8종 sub-struct typedef

**[Session 10]** Orin RS485 modbus 통합 (Step 5)
- `rd_comm_orin.h/.c` 신규 생성 (`ORIN_*` 접두사, CRC-16/IBM)
- `rd_map_dpcb.h/.c` 신규 생성 (Region LUT 15개, DISPATCH_WRITE/READ)
- `RD_TASK_RS485` 완성: READ → mtr_lock 판단 → HANDLE → WRITE → REBOOT

**[Session 11]** `RD_TASK_SYSTEM` + MARSHAL SYSTEM 영역 (Step 7) — **2026-06-25**
- `rd_system.h`: `tim_cnt`, `hw` extern 선언 추가
- `rd_system.c`:
  - `RD_SYSTEM_CHECKER()` (static): 3채널 Checker+Recovery 디스패치
  - `ACTION_STATE_FAULT()` (static): 모터 토크 OFF → `#ifndef RS485_TEST_ON` 가드 재부팅
  - `RD_TASK_SYSTEM()`: 10ms `osDelayUntil` — HW_RESET_HANDLE → CHECKER → EVALUATE_STATE → FAULT 디스패치 → MARSHAL_PUBLISH
- `rd_map_dpcb.c`: `RD_MAP_MARSHAL_PUBLISH` SYSTEM 영역 구현 (sys_state, realtime_tick, hw.*, degraded_cnt)
- **동작 확인**: Dynamixel RS485 케이블 뽑기 → `hw.error.raw = 0x04` (bit2=uart6) → LS_OFFLINE → RECOVERY → `hw.error.raw = 0x00` / LS_READY → 케이블 재삽입 → Dynamixel 재탐지 확인

---

## 10. Checker 에러 감지 구조 및 디버깅 참조

> **현재 Checker 구현 완료** (Step 7). 아래 시나리오는 실제 동작 기준.

---

### 10-1. Checker 공통 상수

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

### 10-2. lifecycle 전이 흐름

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

### 10-3. 채널별 FAULT 정책

| 채널 | FAULT 진입 | 재부팅 조건 | 비고 |
|------|-----------|------------|------|
| DPCB_rs485 (USART2) | `fatal_rs485_cnt ≥ 200` → `payload_state = SYS_STATE_FAULT` | `RS485_TEST_ON` **해제 시** `NVIC_SystemReset()` | ECU_V3 동일 정책 |
| DPCA_uart4 (UART4) | **없음** | **없음** — `hw.reset.bit.uart4=1` + LS_RECOVERING → Orin 주도 복구 | |
| DPCB_dyn (USART6) | **없음** | **없음** — 채널 레벨 hw.error/hw.fatal 집계만. 개별 모터는 `dyn_ctrl.error.comm_flag`로 사용자 처리 | 1개라도 응답 시 채널 유지 |

> **현재 `RS485_TEST_ON`이 `rd_system.h:29` 에 정의되어 있음** → FAULT 진입해도 재부팅은 차단된 상태.  
> 실제 운용 시 이 define을 제거하면 USART2 FAULT 시 자동 재부팅 활성화.

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

### 10-4. 케이블 제거만으로는 FAULT 미진입 (공통)

RECOVERY = `HAL_UART_Abort → DeInit → Init → DMA 재시작`.  
케이블 유무와 무관하게 HAL 재초기화 자체는 성공 → fatal_cnt 첫 OFFLINE 시 +20 후 decay.

**FAULT까지 가는 실제 원인**: HAL_UART_Init 연속 실패 = DMA 컨트롤러 / 레지스터 수준 HW 결함.  
또는 **보드레이트가 다른 장치 연결** → 연속 FE/NE 에러 → 10회 OFFLINE 사이클 → fatal_cnt 200 → FAULT (약 2초).

---

### 10-5. 시나리오별 에러 진행

#### A. Orin RS485 케이블 처음부터 미연결

```
→ LS_READY 고착 (last_rx_tick=0, 에러 없음)
  이유: LS_READY→RUNNING 승격 조건 = 첫 수신 OR 첫 HAL 에러 — 케이블 없으면 둘 다 없음
  hw.error.uart2 = 0, reg.uart2.state = LS_READY

디버깅 주의: 에러 없는 게 정상이 아님. 케이블 없이 부팅하면 조용히 대기.
```

#### B. Orin RS485 연결 후 케이블 뽑힘

```
t=0ms    마지막 패킷 수신
t=100ms  HC_TIMEOUT → degraded_cnt +20/tick
t=200ms  degraded_cnt ≥ 200 → LS_DEGRADED → hw.error.bit.uart2 = 1
t=600ms  degraded_cnt = 1000 → LS_OFFLINE → hw.fatal.bit.uart2 = 1
         Checker RET_NOK → RECOVERY → LS_READY / fatal_rs485_cnt += 20
t=600ms~ RECOVERY 성공 → LS_READY 고착 (케이블 없으면 last_rx_tick=0)
         Checker RET_OK → fatal_rs485_cnt -1 (decay) → 재차 OFFLINE 사이클 없음

FAULT 미진입. hw.reset.bit.uart2 = 1 표시 후 대기.
```

#### C. DPCA 케이블 처음부터 미연결

```
→ LS_READY 고착 (시나리오 A와 동일 메커니즘)
dpcaTask: TX DMA 성공 (케이블 무관), RX 없음 → last_rx_tick = 0 고착
hw.error.uart4 = 0, hw.fatal.uart4 = 0
```

#### D. DPCA 연결 후 케이블 뽑힘

```
t=100ms  HC_TIMEOUT → degraded_cnt 상승
t=200ms  LS_DEGRADED → hw.error.uart4 = 1
t=600ms  LS_OFFLINE → hw.fatal.uart4 = 1 → RECOVERY → LS_READY
FAULT 없음
```

#### E. Dynamixel — 1개 연결, 나머지 2개 미연결

```
연결된 1개 응답 → IDLE ISR → last_rx_tick 주기 갱신 → HC_TIMEOUT 없음
DPCB_dyn 채널: LS_RUNNING 유지 → hw.error.uart6 = 0

미연결 2개:
  RD_DYN_LOOP: TX → 2ms 대기 → rx_new=0 → RET_NOK
  MOT[i].dyn_ctrl.error.comm_flag = 1  ← 관찰 지점
  MOT[i].dyn_ctrl.is_running = 0

→ UART Checker 레벨에서 개별 모터 탈락 미감지. Step 6 MARSHAL 구현 후 reg.motor_data로 발행 가능.
```

#### F. Dynamixel — 연결된 1개 케이블 뽑힘 (동작 확인 완료 ✓)

```
t=0ms    마지막 Dynamixel 응답
t=100ms  HC_TIMEOUT → degraded_cnt +20/tick
t=200ms  LS_DEGRADED → hw.error.bit.uart6 = 1  (reg.sys.hw_error = 0x04)
t=600ms  LS_OFFLINE  → hw.fatal.bit.uart6 = 1
         Checker RET_NOK → RECOVERY → LS_READY
         hw.error.raw = 0x00 (LS_READY = HC_OK)

케이블 재삽입 → Dynamixel 응답 → LS_RUNNING 승격 → 정상 동작 재개

FAULT 없음. hw.reset 없음 (dyn 채널 정책).
```

> **확인된 동작 (2026-06-25)**: 케이블 뽑기 → `hw.error.raw = 4` → 0 (LS_READY) → 재삽입 → Dynamixel 재탐지 정상 확인.

#### G. HAL 하드웨어 에러 연속 (ORE / FE / NE)

```
HAL_UART_ErrorCallback → isr_err_code 저장
Checker: rx_error_cnt++ → 재무장(DMA AbortReceive → Re-DMA) 시도
10회 연속 재무장 실패 → rx_error_cnt > 10 → 즉시 LS_OFFLINE + HC_FATAL
                       → hw.fatal.uart2/4/6 해당 비트 set

DPCB_rs485(USART2) 해당 시:
fatal_rs485_cnt += 20 → 10회 OFFLINE 사이클 ≈ 2초 → fatal_cnt 200 → SYS_STATE_FAULT
RS485_TEST_ON 해제 상태이면 → NVIC_SystemReset() (재부팅)

재부팅 발생했다면: HAL 연속 에러 + USART2 채널이 원인.
보드레이트가 다른 장치를 USART2에 연결하면 이 경로로 재부팅.
```

---

### 10-6. 디버깅 관찰 포인트

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

**STATE_t 값 해석 (uint8_t = health[7:4] \| lifecycle[3:0]):**

```
0x01 = LS_READY   + HC_OK   (연결 없음 / 초기화 직후)
0x02 = LS_RUNNING + HC_OK   (정상)
0x32 = LS_RUNNING + HC_TIMEOUT
0x03 = LS_DEGRADED + HC_OK  (오염 누적 중)
0xF2 = LS_RUNNING + HC_FATAL (즉시 OFFLINE 직전)
```
