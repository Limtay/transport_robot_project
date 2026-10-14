# PRA Checker / Failsafe 정책

> 최종 갱신: 2026-09-17 (Step6 — USART6(Orin) 채널 Checker 구현)
> 상위: [pra_overview.md](pra_overview.md)

> **상태**: RS485(USART6/Orin) 채널 Checker **구현 완료** (ECU/DPC_B advanced rd_uart + rd_system.c). **CAN1(BMS)·I2C1(패널)은 해당 페리페럴 추가 후 확장** (현재 훅만 주석). 값은 rd_common.h/DEFINE 상수 기준(실기 조정 대상).

---

## 1. 설계 원칙 (ECU/DPC_B 공통 계승)

- **Checker/Recovery 분리**: 드라이버는 진단(`RET_OK/WAIT/NOK`)만, 복구·상태전이는 상위(systemTask)에서. Checker 는 절대 INIT/RECOVERY 내부 호출 안 함
- **채널 lifecycle**(`STATE_t`): `LS_INIT→READY→RUNNING→DEGRADED→OFFLINE→RECOVERING` + health(HC_OK/TIMEOUT/CRC/FRAMING/…/FATAL)
- **degraded_cnt 누적/decay**: 에러 tick +K(`DEGRADED_K_100HZ=20`), 정상 tick −1(`DEGRADED_TICK_DECAY`). `HIGH=200`→DEGRADED / `LOW=50`→RUNNING(4× 히스테리시스), `MAX=1000`→OFFLINE
- **hw.error/hw.fatal/hw.reset 집계** → `MARSHAL_PUBLISH` 가 reg.sys(addr 20/21/22) 발행 ([pra_register.md](pra_register.md) §2-2)

---

## 2. 구현 구조 (rd_system.c, 10ms systemTask)

```
RD_TASK_SYSTEM (10ms osDelayUntil)
  → tim_cnt = HAL_GetTick()          // alive tick (TIM5 부재, ms — register ×0.1ms 스케일 후속)
  → RD_SYSTEM_HW_RESET_HANDLE()      // Orin 요청(addr5 hw_reset) 채널 리셋
  → RD_SYSTEM_CHECKER()              // 채널 Checker + Recovery (전원 FSM 미연계, #5)
  → RD_SYSTEM_EVALUATE_STATE()       // STATE_t → hw.error/hw.fatal 집계
  → RD_MAP_MARSHAL_PUBLISH()         // SYSTEM 영역(hw.*/degraded/sys_state=PRA_CONTROL.STATE/tick) 발행
```

- **rd_uart advanced 계층**: `RD_RS485_CHECKER(obj, DEGRADED_K_100HZ)` → 내부 `RD_UART_CHECKER`(HAL 에러/comm_err_flag/timeout → health, degraded, lifecycle). `RET_NOK`(OFFLINE) 시 상위가 `RD_RS485_RECOVERY`(Abort→DeInit→Init→DMA 재시작) 호출
- **HAL 에러 캡처**: `HAL_UART_ErrorCallback`(rd_system.c) → `PRA_uart6.error.isr_err_code`, Checker 가 `isr_err_take` 로 소비
- **comm_err_flag**: `rd_comm_orin` 파서가 framing/CRC 오류 OR set → Checker 가 HC_FRAMING_ERR/HC_CRC_ERR 매핑

---

## 3. 채널별 정책

| 채널 | 대상 | 상태 | FAULT 정책 |
|------|------|------|-----------|
| **USART6** (uart2 슬롯) | Orin RS485 | **구현** | `RD_RS485_CHECKER` RET_NOK → `fatal_rs485_cnt += FATAL_K(20)`, RECOVERY. `≥ FATAL_MAX(200)` → `hw.reset.uart2=1` telltale(addr22). **⚠ 전원 미연계(#5)**: self-reboot·전원차단 없음 (ACTION_STATE_FAULT·payload_state 폐기 2026-10-04) |
| CAN1 | BMS | **미구현** | 페리페럴 추가 후 (EVALUATE/CHECKER 훅 주석). degraded_cnt[0]/hw.can1 슬롯 예약 |
| I2C1 | MCP23017 | **미구현** | 페리페럴 추가 후. hw.i2c 슬롯 예약 |
| ~~USART1~~ | ~~EX RS485~~ | **미사용** | 감시 대상 아님 (확장용) |

- **채널↔슬롯**: `degraded_cnt[1]`/`hw.*.bit.uart2`/`hw_reset bit1` = USART6(Orin). `[0]`/`can1`/`bit0`=CAN1, `i2c`/`bit2`=I2C1 (register HW_BIT 순서와 일치)
- **`RS485_TEST_ON`** (`rd_system.h`): ACTION_STATE_FAULT 제거(#5)로 **현재 미사용·inert**. (통신 fatal 자동 재부팅 폐기)
- **Orin 수동 리셋**: Orin 이 `reg.reg_df.hw_reset`(addr5) 비트 write → `RD_SYSTEM_HW_RESET_HANDLE` 가 해당 채널 RECOVERY 후 플래그 클리어

---

## 4. 전원 세이프티 연동 (PRA 고유 — TODO Step9)

일반 통신 Checker 외 PRA 는 **전원단 세이프티** 추가 필요 (controlTask/전원 FSM 연동):

- 프리차징 실패(전압 미도달·타임아웃) → FAULT / 릴레이 차단
- 릴레이 접점 융착 감지(`current_input`/`voltage_output` 교차) → FAULT
- 과전류(`current_input`) / 과온(NTC) → 보호 차단
- BMS(CAN1) FAULT/통신두절 시 전원 정책 (단, Orin RS485 통신두절은 #5로 전원 미연계)
- 전원 FAULT 안전정리(릴레이 OFF)는 `RD_CONTROL_LOOP` FAULT 분기가 담당 (ACTION_STATE_FAULT 폐기)

> 세부 임계·전이: [pra_opmode.md](pra_opmode.md) §5. 실기 확정 대상.

---

## 5. 참조

- ECU_V3: `stm_ws/ECU_V3/CLAUDE.md` (Checker/Degraded Counter/IWDG 상세)
- DPC_B: [dpcb_checker.md](../dpcb_config_wiki/dpcb_checker.md) (채널별 FAULT 정책·시나리오별 에러 진행)
