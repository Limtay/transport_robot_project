# PRA FreeRTOS 태스크 구조

> 최종 갱신: 2026-09-17 (Step4·5 — rs485Task 실동작, 그 외 골격)
> 상위: [pra_overview.md](pra_overview.md)

> CMSIS-RTOS v2 (Dynamic allocation). 태스크는 `.ioc` 정의·`main.c` 생성. **main.c 태스크 본체는 ECU 구조대로 `RD_TASK_xxx()` 호출만** 두고 로직은 `rd_system.c` 에 둔다 (현재 `RD_TASK_RS485` 만 구현).

---

## 1. 태스크 일람

| 태스크 | 진입점 (main.c) | 우선순위 | 스택 | 주기/트리거 | 현재 본체 |
|--------|----------------|----------|------|------------|-----------|
| defaultTask | `StartDefaultTask` | 24 | 512 | 100ms | `LED_G` 토글 (alive 블링크) |
| rs485Task | `Start_rs485` | 8 | 512 | 이벤트(USART6 IDLE)+10ms 폴링 | **`RD_TASK_RS485()`** — Orin modbus (Step4·5) |
| controlTask | `StartControl` | 8 | 1024 | (예정) 10ms | 빈 루프 `osDelay(1)` |
| systemTask | `StartSystem` | 8 | 512 | 10ms | **`RD_TASK_SYSTEM()`** — Checker/EVALUATE/PUBLISH (Step6) |
| can1Task | `Start_can1` | 8 | 512 | (예정) 이벤트/폴링 | 빈 루프 `osDelay(1)` |

> ⚠ 우선순위 숫자는 `.ioc` Tasks01 정의값. 실제 우선순위 정책은 ECU 이식 시 정리.
> **main.c 태스크 규약 (사용자 확정)**: `main.c` Start_xxx 본체에는 `RD_TASK_xxx()` 형태만 둔다(MX 재생성 시 USER CODE 보존). 현재 `Start_rs485`=`RD_TASK_RS485();`, `StartSystem`=`RD_TASK_SYSTEM();`. control·can1 은 빈 루프 유지.

---

## 2. 태스크별 역할

| 태스크 | 역할 | 상태 | 참조 |
|--------|------|------|------|
| **rs485Task** | USART6 Orin modbus: 요청직전 `MARSHAL_PUBLISH` → `RD_ORIN_READ→HANDLE→WRITE` → REBOOT 처리. 이벤트(IDLE ISR)+10ms 폴링 | **구현 (Step4·5, `RD_TASK_RS485`)** | `rd_system.c` |
| **systemTask** | tick 갱신 → HW_RESET_HANDLE → **Checker(USART6)** → EVALUATE_STATE(hw.error/fatal) → FAULT 처리 → `MARSHAL_PUBLISH`(SYSTEM) | **구현 (Step6, `RD_TASK_SYSTEM`)** — RS485 채널만, CAN1/I2C1 훅 주석 | [pra_checker.md](pra_checker.md) |
| **controlTask** | **`RD_TASK_CONTROL`**: `RD_PERIPHERAL_INIT` + `for(;;){READ(ADC); EXIO_UPDATE; WRITE(GPIO); RD_CONTROL_LOOP(&PRA_CONTROL,&PRA_PERIPHERAL); osDelay(10)}`. **전원 FSM 백본(`RD_CONTROL_LOOP`) 구현 중** — SYS_STATE 골격(INIT/RUN/FAULT), RUN 내 BOOT_FSM SEQ·SW 입력 TODO | **Step9 진행 중** | [pra_opmode.md](pra_opmode.md) §4-7 |
| **can1Task** | BMS CAN RX 파싱 → `SENSOR/BMS` 발행 | 미구현 (Step8) | ECU CAN 처리 |
| **(미생성) Start_i2c** | MCP23017 EXIO `RD_EXIO_UPDATE`(SW read + LED/BUZ write) 호출 — **MX 태스크 생성 필요** (`rd_peripheral_pra.c` TODO) | 미생성 (Step7) | [pra_peri.md](pra_peri.md) §5·§6 |
| (미배정) | **USART1(EX)은 미사용** → 태스크 불필요 | - | [plan.md](plan.md) |

---

## 3. UART ↔ 태스크 깨우기

```
USART6 IDLE ISR → RD_UART_IDLEHandler(&PRA_uart6) → osThreadFlagsSet(rs485TaskHandle) → rs485Task   [구현]
USART6 TC   ISR → RD_RS485_IRQ_HANDLER(&PRA_rs485)  (DIR→RX + RE/IDLE 재활성)                        [구현]
USART1          → 미사용 (EX RS485, 확장용)
CAN1 RX ISR     → can1Task (Step8)
```

- PRA `rd_uart` 는 `wake_task` 필드가 없는 단순형 → IDLE ISR 이 `Instance==USART6` 판정 후 `rs485TaskHandle` 직접 기상 (`rd_uart.c`, Step4 에서 USART1→USART6 정정)
- RS485 DIR(PC8): TX 시 SET(`RD_RS485_Transmit`) → TC 인터럽트(`RD_RS485_IRQ_HANDLER`)로 RX 복귀 + **RE/IDLE 재활성**(미수행 시 첫 TX 후 RX 정지)
- ISR 배선은 `stm32f4xx_it.c` `USART6_IRQHandler` USER CODE 영역

---

## 4. 전역 객체 (`rd_system.c`)

```c
// UART 링버퍼 + RS485 핸들 + 패킷 채널  [구현 — Step4]
UART_Ring_t   PRA_uart6;    // Orin RS485 백킹 (USART6). advanced: .error(STATE_t/degraded/lifecycle) + .wake_task
RS485_t       PRA_rs485;    // Orin (.uart_obj=&PRA_uart6, .DIR=PC8 주입)
ORIN_COMM_t   ORIN_PACKET;  // Orin RS485 패킷 채널 (tx/rx/reboot_pending)
// USART1(EX RS485) 미사용 → 객체 미생성

REGISTER_t              reg;            // 256B single source-of-truth  [rd_map_pra.c]
volatile uint8_t        rs485_init_fail_cnt;  // USART6 INIT 재시도 (부팅 진단)
volatile uint8_t        fatal_rs485_cnt;      // USART6 Checker fatal 누적 (FATAL_MAX→FAULT)  [Step6]
volatile SYSTEM_STATE_e payload_state;  // 내부 시스템 상태 (sys_state 발행원)
HW_ERROR_FLAG_t         hw;             // hw.error/fatal/reset 집계 (bit0=can1/bit1=uart2/bit2=i2c)  [Step6]
uint32_t                tim_cnt;        // alive tick [ms] — TIM5 부재, HAL_GetTick  [Step6]

PERIPHERAL_t  PRA_PERIPHERAL;  // 전원GPIO+AUX+ADC+EXIO [Step7, rd_peripheral_pra.h]
CONTROL_PRA_t PRA_CONTROL;     // 전원 제어 FSM: .STATE(INIT/RUN_OFF/TRN/ON/FAULT) .BOOT_FSM .LOOP_CNT .AUX_SW_LAST .MAIN_SW_LAST [Step9 진행]
```

- **HAL_UART_ErrorCallback**(rd_system.c): USART6 → `PRA_uart6.error.isr_err_code` 캡처 (Checker 소비)
- ADC 스냅샷(`main.c` `adc_raw_buf`/`my_adc_register`)은 페리페럴 READ(Step7)로 `PERIPHERAL_t` 갱신 → `sensor_adc` 발행은 MARSHAL_PUBLISH SENSOR 영역(**Step10**)에서 연결.
