# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

STM32F446RET6 기반 모바일 로봇용 ECU 펌웨어. Cortex-M4 @ 84 MHz, FreeRTOS(CMSIS-RTOS v2), 4개의 CubeMars AK 모터(CAN)와 5개의 AS5600 인코더(I2C)를 제어하며 Orin AGX(상위 레벨)와 RS485/Dynamixel 2.0 프로토콜로 통신한다.

상위 git 저장소는 `/home/limtay/tp_ws` (transport_robot_project). 이 펌웨어는 그 안의 `stm_ws/ECU_V3/` 서브트리이며, 같은 워크스페이스에 ROS2 패키지(`orin_ws`)와 구버전 펌웨어(`ECU_V2`)가 공존한다.

## Build

IDE: **STM32CubeIDE** (Eclipse managed builder). 독립 Makefile 없음.

- 빌드: STM32CubeIDE에서 Project → Build (`Ctrl+B`). 빌드 산출물은 `Debug/`.
- 플래싱/디버그: Run → Debug Configuration (ST-Link 사용)
- 하드웨어/핀맵 변경: `ECU_V3.ioc` 를 STM32CubeMX로 열어 재생성 (HAL init 코드 자동 갱신)
- 링커 스크립트: `STM32F446RETX_FLASH.ld` (일반), `STM32F446RETX_RAM.ld` (RAM 실행)

빌드 시스템이 STM32CubeIDE에 종속되어 있어 CLI 빌드/테스트 하네스가 없다. 코드 변경 후 검증은 IDE 빌드로 확인한다.

## 통신 라인 구성

| 채널 | 하드웨어 | 속도 | 용도 |
|------|---------|------|------|
| UART1 (USART1) | DMA2_Stream2 RX | 115200 bps | RC 수신기 |
| UART2 (USART2) | DMA1_Stream5 RX / Stream6 TX | 921600 bps | RS485 ↔ Orin AGX |
| CAN1 | canTxQueue (FreeRTOS Queue) | 1 Mbps | AK 모터 × 4 (ID=1..4) |
| I2C1 | Polling | 400 kHz | AS5600 인코더 × 5 (TCA9548A MUX) |

RS485 방향 제어: `RS485_DIR_Pin (PC3)` — TX 시 SET, RX 시 RESET. TC 인터럽트로 자동 RX 전환, `TX_TIMEOUT=10ms` 강제 복귀.

## 아키텍처

### 태스크 구조 (`Core/Src/main.c` → `Core/Src/rd_system.c`)

`main.c` 의 `StartXxx` 진입점은 모두 `rd_system.c` 의 `RD_TASK_*()` 무한 루프로 위임한다.
**`rd_system.c` 가 태스크 오케스트레이션 허브** — 루프 주기·FSM·Checker 호출이 모두 여기 모인다.

| 태스크 | 구동 | 주기/트리거 | 진입 함수 (`rd_system.c`) |
|--------|------|-------------|---------------------------|
| controlTask | Normal | **200 Hz** (`tick += RD_TASK_CONTROL_200Hz`=5ms) | `RD_TASK_CONTROL` |
| systemTask | Normal | **100 Hz** (`tick += 10`) | `RD_TASK_SYSTEM` |
| i2c1Task | **Low** | **100 Hz** (`tick += 10`) | `RD_TASK_I2C1` |
| rs485Task | Normal | **이벤트 구동** (`osThreadFlagsWait`, DMA IDLE ISR가 기상) | `RD_TASK_RS485` |
| rcTask | Normal | **이벤트 구동** (`osThreadFlagsWait`) | `RD_TASK_RC` |
| can1Task | Normal | 큐 블로킹 (`CAN_AK_TX_TASK_HANDLER`) | `RD_TASK_CAN1` |
| defaultTask | Normal | housekeeping | `StartDefaultTask` (main.c) |

> 주기 상수는 `Core/Inc/rd_system.h` (`RD_TASK_CONTROL_200Hz` 등). 이벤트 구동 태스크는 `RTOS_IS_AVAILABLE`(`rd_uart.h`) 정의 시 플래그 대기, 미정의 시 `osDelay(1)` 폴링으로 폴백한다.

### 핵심 데이터 흐름

```
RS485 RX ─┐
RC    RX ─┤→ REGISTER_t / ECU_PERIPHERAL  ← 단일 진실 원천
          │
controlTask(200Hz): RD_CONTROL_UPDATE → RD_PERIPHERAL_WRITE → CAN AK ×4
systemTask(100Hz) : CHECKER → EVALUATE_STATE → FSM(ACTION_STATE_*) → MAP_MARSHAL_PUBLISH → IWDG
i2c1Task(100Hz)   : RD_PERIPHERAL_I2C → (NOK 시) RD_I2C_ENCODER_RECOVERY
can1Task          : canTxQueue 소비 → CAN HW 송신
```

`controlTask` 가 `RD_PERIPHERAL_WRITE` 실패 시 `robot_state = SYS_STATE_FAULT` 로 직접 전이한다.

### 시스템 FSM (`rd_system.h` `SYSTEM_STATE_e`)

`SYS_STATE_INIT / MANUAL / AUTO / ESTOP_HW / ESTOP_SW / FAULT`. `systemTask` 가 매 tick `robot_state` 에 따라 `ACTION_STATE_*()` 디스패치.
- **모드 스위칭 안전장치**: AUTO↔MANUAL 전환 시 순간 튀는 값에 의한 폭주를 막기 위해 ESTOP을 경유하는 중간 단계를 둔다.
- controlTask는 `robot_state` 변화 감지 시 `RD_CONTROL_RESET_FILTER()` 로 LPF를 리셋해 모드 전환 글리치를 흡수한다.
- `FAULT` 는 이미 망가진 상태로 간주 → 신규 명령 수신/적용 안 함.

### 모터 명령 LOCK (Orin write 폭주 방지)

`RD_TASK_RS485` 에서 패킷 수신 시 `robot_state` 를 `taskENTER_CRITICAL()` 로 보호해 읽고, **AUTO 일 때만 `mtr_lock=0`** (모터 쓰기 허용), 그 외에는 `mtr_lock=1` 로 `RD_PACKET_HANDLE(&pkt, mtr_lock)` 에 전달한다. Orin에서 WRITE가 끊겨 이전 값으로 폭주하는 것을 막기 위한 장치 (`rd_system.c:393-397`).

### 레지스터 맵 (`Core/Inc/rd_register_ecu.h`)

256바이트 메모리 매핑 구조체 `REGISTER_t`. 외부에서 RS485 READ/WRITE로 접근:

| 주소 | 구조체 | R/W | 용도 |
|------|--------|-----|------|
| 0–15 | `DEFINE_t` | R/W | 임계값 설정, `hw_reset` 트리거 |
| 46–61 | `DATA_SYSTEM_t` | R/O | 상태, degraded_cnt, realtime_tick |
| 83–93 | `DATA_ENCODER_t` | R/O | AS5600 각도 × 5 |
| 94 | `DATA_UART2_t` | R/O | RS485 상태 |
| 95 | `DATA_RC_t` | R/O | RC 상태 |
| 96–127 | `DATA_MOTOR_t` | R/O | 모터 피드백 |
| 128–179 | `CMD_MOTOR_t` | R/W | 모터 명령 (mode/pos/vel/cur) |
| 180–191 | `CMD_SYSTEM_t` | R/W | 선속도/각속도, 시스템 명령 |
| 224–255 | `DIAG_t` | R/O | 디버그 카운터 |

### 에러 상태 머신 (`Core/Inc/rd_common.h`)

**`STATE_t` (8비트 packed):** `CH_PACK(lc,hc)` = `health<<4 | lifecycle`.
- `lifecycle` (4비트): `LS_INIT(0)` → `LS_READY(1)` → `LS_RUNNING(2)` ↔ `LS_DEGRADED(3)` → `LS_RECOVERING(4)` → `LS_OFFLINE(15)`. `memset 0 == LS_INIT` 으로 자연 초기화 (UNINIT 폐기).
- `health` (4비트): `HC_OK(0)` ~ `HC_FATAL(15)`. 임계값 `HC_THRESHOLD_*` (INFO 1 / WARN 2 / ERROR 7 / CRITICAL 11 / FATAL 13).

**Degraded Counter (채널 주파수 무관 동일 시간 진입 설계):**
- 에러 시: `degraded_cnt += DEGRADED_K_<rate>` (100Hz→20, 200Hz→10, 250Hz→8 = `2000/F`)
- Checker tick 시: `degraded_cnt -= DEGRADED_TICK_DECAY(1)`
- `> DEGRADED_THRESHOLD_HIGH(200)` → `LS_DEGRADED`, `< DEGRADED_THRESHOLD_LOW(50)` → `LS_RUNNING` (4× 히스테리시스), 포화 `DEGRADED_CNT_MAX(1000)`
- 의도: 5% 손실=중립(자연 감쇠), 10% 손실 → ~2초 후 `LS_DEGRADED`

**Checker/Recovery 분리 원칙 (핵심 설계):**
- `RD_*_CHECKER()`: 상태 진단만, 직접 복구 없음. `RET_OK/RET_WAIT/RET_NOK` 반환.
- Recovery: 상위 `systemTask` 에서 호출. 드라이버가 직접 복구 시작하지 않음.
- 예외: i2c 인코더는 ESTOP 비연동(텔레메트리용)이라 `i2c1Task` 가 직접 auto-recovery 수행.

### IWDG 워치독 + 하트비트

`systemTask` 가 IWDG를 refresh하되, **controlTask 하트비트(`hb_control`)가 직전 tick 이후 증가했을 때만** refresh한다. controlTask가 hang → `hb_control` 정체 → refresh 중단 → IWDG 리셋. systemTask 자신의 liveness는 루프가 도는 것 자체로 보장. (`rd_system.c:333-340`)

### ISR → 태스크 동기화 패턴

- ISR은 `volatile` 필드만 직접 기록 (`last_rx_tick`, `is_running`, `isr_err_code`) + 태스크 플래그 set.
- 태스크가 에러 카운터/degraded_cnt 업데이트.
- 뮤텍스 없음: 단일 소유자 패턴 (ISR 기록, 태스크 읽기). 공유 상태 읽기는 필요한 곳만 `taskENTER_CRITICAL`.
- CAN TX: `canTxQueue` 로 비동기 처리 (controlTask 블로킹 방지).

## 주요 파일 위치

```
Core/Inc/
  rd_common.h           # STATE_t, LS_*/HC_*, Degraded Counter 전체 정의
  rd_register_ecu.h     # REGISTER_t, DEFINE_t, DATA_SYSTEM_t 등 모든 데이터 구조체
  rd_system.h           # SYSTEM_STATE_e, RD_TASK_* 선언, 태스크 주기 매크로
  rd_uart.h             # UART_Ring_t, RS485_t, RTOS_IS_AVAILABLE, UART API
  rd_comm_ecu.h         # Dynamixel 2.0-like 패킷 프로토콜
  rd_control.h          # 기구학 상수 (ROBOT_WHEEL_RADIUS_M=0.10, ROBOT_TRACK_WIDTH_M=1.10)
  can_ak.h              # CAN_Ak_Handle_t, AK 프로토콜, USE_RTOS_CAN_QUEUE
  i2c_as5600.h          # AS5600_Handle_t, TCA9548A MUX

Core/Src/
  rd_system.c           # ★ 태스크 루프 전체 + FSM + Checker/Recovery 디스패치 + IWDG
  main.c                # CubeMX 생성 init, 태스크 attr/생성, StartXxx → RD_TASK_*
  stm32f4xx_it.c        # ISR (USART1/2, CAN1, DMA, TIM5/6)
  stm32f4xx_hal_msp.c   # HAL 콜백 (ErrorCallback 포함)
  rd_map_ecu.c          # RS485 READ/WRITE 레지스터 디스패치, MARSHAL_PUBLISH
  rd_control.c          # LPF, RC→레지스터 변환, 기구학, RD_CONTROL_UPDATE
  rd_peripheral_ecu.c   # 페리페럴 집계 read/write (CAN/I2C)
```

## 주요 상수 및 임계값

| 상수 | 값 | 의미 |
|------|-----|------|
| `RD_TASK_CONTROL_200Hz` | 5 (ms) | controlTask 주기 |
| `CAN_RX_TIMEOUT_MS` | 100 ms | 모터별 RX 타임아웃 |
| `UART_RX_TIMEOUT_MS` | 500 ms | RS485 RX 타임아웃 |
| `TX_TIMEOUT` | 10 ms | RS485 강제 RX 전환 |
| `AK_TEMP_WARN` | 75 °C | 과열 ESTOP 트리거 |
| `AK_TX_TIMEOUT_ERR` | 5 | 연속 TX 실패 → 에러 |
| `AK_RX_TIMEOUT_ERR` | 10 | 연속 RX 타임아웃 → 에러 |
| `UART_FATAL_CNT_TH` | 20 | HAL 에러 → LS_OFFLINE |
| `BREAK_CURRENT_SW` | 3 A | SW ESTOP 제동 전류 |

## 설계 원칙

- **SOLID + Checker/Recovery 분리**: 드라이버는 진단만, 복구·상태전이 로직은 상위 `systemTask` 에 집중.
- **Safe fault**: auto-recovery는 항시 동작 가능해야 함. 단, RC처럼 의도적으로 끄는 채널은 연속 초기화 실패 시 "꺼진 상태"로 판단하고 초기 준비 상태로 복귀해, 재기동 시 정상 동작하도록 무한 초기화 루프를 피한다.
- 새 코드는 주변 코드의 주석 밀도·네이밍(`RD_` 접두사, 한국어 주석)·ISR/태스크 소유권 규칙을 따른다.
