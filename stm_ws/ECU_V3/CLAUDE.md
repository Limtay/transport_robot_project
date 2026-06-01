# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

STM32F446RET6 기반 모바일 로봇용 ECU 펌웨어. Cortex-M4 @ 84 MHz, FreeRTOS v10.3.1, 4개의 CubeMars AK 모터와 5개의 AS5600 인코더를 제어하며 Orin AGX(상위 레벨)와 RS485/Dynamixel 2.0 프로토콜로 통신한다.

## Build

IDE: **STM32CubeIDE** (Eclipse managed builder). 독립 Makefile 없음.

- 빌드: STM32CubeIDE에서 Project → Build (`Ctrl+B`)
- 플래싱: Run → Debug Configuration (ST-Link 사용)
- 하드웨어 설정 변경: `ECU_V3.ioc` → STM32CubeMX로 열기
- 링커 스크립트: `STM32F446RETX_FLASH.ld` (일반), `STM32F446RETX_RAM.ld` (RAM 실행)

## 통신 라인 구성

| 채널 | 하드웨어 | 속도 | 용도 |
|------|---------|------|------|
| UART1 (USART1) | DMA2_Stream2 RX | 115200 bps | RC 수신기 |
| UART2 (USART2) | DMA1_Stream5 RX / Stream6 TX | 921600 bps | RS485 ↔ Orin AGX |
| CAN1 | canTxQueueHandle (FreeRTOS Queue) | 1 Mbps | AK 모터 × 4 (ID=1..4) |
| I2C1 | Polling (50 Hz, 낮은 우선순위) | 400 kHz | AS5600 인코더 × 5 (TCA9548A MUX) |

RS485 방향 제어: `RS485_DIR_Pin (PC3)` — TX 시 SET, RX 시 RESET. TC 인터럽트로 자동 RX 전환, `TX_TIMEOUT=10ms` 강제 복귀.

## 아키텍처

### 핵심 데이터 흐름

```
RS485 RX → REGISTER_t (256 bytes) ← 모든 태스크의 단일 진실 원천
                   │
    controlTask (100Hz) → CAN AK 모터 × 4 + RS485 응답 TX
    rs485Task           → RD_PACKET_READ/HANDLE/WRITE
    can1Task            → CAN_AK_TX_TASK_HANDLER (Queue 소비)
    i2c1Task (50Hz)     → AS5600 인코더 Update
    rcTask (100Hz)      → RC 채널 파싱
    systemTask (20Hz)   → FSM + ESTOP + Checker 호출
    defaultTask         → 가벼운 housekeeping
```

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

**`STATE_t` (8비트 packed):**
- `bits.lifecycle` (4비트): `LS_INIT(0)` → `LS_READY(1)` → `LS_RUNNING(2)` ↔ `LS_DEGRADED(3)` → `LS_RECOVERING(4)` → `LS_OFFLINE(15)`
- `bits.health` (4비트): `HC_OK(0)` ~ `HC_FATAL(15)` (timeout, CRC, framing, overrun, HW fault, CAN bus-off 등)

**Degraded Counter 메커니즘:**
- 에러 발생 시: `degraded_cnt += K` (K = 2000 / F_comm; 100Hz→20, 200Hz→10, 250Hz→8)
- 매 Checker 호출 시: `degraded_cnt -= DEGRADED_TICK_DECAY(1)` (20ms 주기)
- `>200` → `LS_DEGRADED`, `<50` → `LS_RUNNING`, 포화 `=1000`
- 설계 의도: 5% 손실 = 중립, 10% 손실 → 약 2초 후 `LS_DEGRADED`

**Checker/Recovery 분리 원칙:**
- `RD_*_CHECKER()`: 상태 진단만, 직접 복구 없음. `RET_OK/RET_WAIT/RET_NOK` 반환
- Recovery: 상위 `systemTask`에서 호출. 드라이버가 직접 복구 시작하지 않음

### ISR → 태스크 동기화 패턴

- ISR은 `volatile` 필드만 직접 기록 (`last_rx_tick`, `is_running`, `isr_err_code`)
- 태스크는 에러 카운터/degraded_cnt 업데이트
- 뮤텍스 없음: 단일 소유자 패턴 (ISR 기록, 태스크 읽기)
- CAN TX: `canTxQueueHandle`로 비동기 처리 (controlTask 블로킹 방지)

## 주요 파일 위치

```
Core/Inc/
  rd_common.h           # STATE_t, ERROR_STATUS_t, LS_*/HC_* 전체 정의
  rd_register_ecu.h     # REGISTER_t, DEFINE_t, DATA_SYSTEM_t 등 모든 데이터 구조체
  rd_uart.h             # UART_Ring_t, RS485_t 구조체, UART API
  rd_comm_ecu.h         # Dynamixel 2.0-like 패킷 프로토콜
  rd_system.h           # SYSTEM_STATE_e, RS485_TEST_ON 매크로
  can_ak.h              # CAN_Ak_Handle_t, AK 모터 프로토콜 상수
  i2c_as5600.h          # AS5600_Handle_t, TCA9548A MUX

Core/Src/
  stm32f4xx_it.c        # ISR 구현 (USART1/2, CAN1, DMA, TIM5/6)
  stm32f4xx_hal_msp.c   # HAL 콜백 (ErrorCallback 포함)
  rd_system.c           # 시스템 FSM
  rd_map_ecu.c          # RS485 READ/WRITE 레지스터 디스패치
  rd_control.c          # LPF, 기구학 (ROBOT_WHEEL_RADIUS_M=0.10, ROBOT_TRACK_WIDTH_M=1.10)
```

## 주요 상수 및 임계값

| 상수 | 값 | 의미 |
|------|-----|------|
| `CAN_RX_TIMEOUT_MS` | 100 ms | 모터별 RX 타임아웃 |
| `UART_RX_TIMEOUT_MS` | 500 ms | RS485 RX 타임아웃 |
| `TX_TIMEOUT` | 10 ms | RS485 강제 RX 전환 |
| `AK_TEMP_WARN` | 75 °C | 과열 ESTOP 트리거 |
| `AK_TX_TIMEOUT_ERR` | 5 | 연속 TX 실패 → 에러 |
| `AK_RX_TIMEOUT_ERR` | 10 | 연속 RX 타임아웃 → 에러 |
| `UART_FATAL_CNT_TH` | 20 | HAL 에러 → LS_OFFLINE |
| `BREAK_CURRENT_SW` | 3 A | SW ESTOP 제동 전류 |

## 검토 세션 컨텍스트

현재 세션 목표: 통신 라인별 개별 검토 후 종합 검토.
- UART1 (RC), UART2 (RS485), CAN1 (AK 모터), I2C1 (AS5600 인코더) 순서로 진행
- 확인 사항: 문법/문맥 오류, ISR 에러 핸들 오류, Safe fault (Auto Recovery 항시 작동 여부)
- 추가 구현 예정: `DEFINE_t.hw_reset` 수신 시 해당 채널 소프트 리셋 → `DATA_SYSTEM_t`와 state 업데이트
- 설계 원칙: SOLID, Checker/Recovery 분리, 상위단(systemTask) 로직 집중
