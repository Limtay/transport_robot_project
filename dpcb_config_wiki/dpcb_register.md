# DPC_B 레지스터 맵 구조 및 MARSHAL 매핑

> 최종 갱신: 2026-08-03  
> 상위: [dpcb_overview.md](dpcb_overview.md)  
> 파일: `DPC_B/Core/Inc/rd_register_dpcb.h`

---

## 1. 레지스터 맵 개요 (REGISTER_t, 256 bytes)

| 주소 | 크기 | 섹션 | typedef | R/W |
|------|------|------|---------|-----|
| 0~15 | 16 | DEFINE | `DEFINE_t` | R/W (sys_write_mode 잠금키 필요) |
| 16~45 | 30 | RSVD0 | - | R/O |
| 46~61 | 16 | SYSTEM | `DATA_SYSTEM_t` | R/O |
| 62~64 | 3 | SENSOR/DPCA | `DATA_SENSOR_DPCA_t` | R/O |
| 65 | 1 | UART2/data | `DATA_UART2_t` | R/O |
| 66~73 | 8 | GPIO/data | `DATA_SENSOR_DPCB_t` | R/O |
| 74~110 | 37 | MOTOR/data | `DATA_MOTOR_DYN_t` | R/O |
| 111~119 | 9 | RSVD1 | - | R/O |
| 120~121 | 2 | CMD/DPCA | `CMD_DPCA_t` | R/W |
| 122~127 | 6 | CMD/DPCB | `CMD_DPCB_t` | R/W |
| 128~142 | 15 | CMD/MOT | `CMD_MOT_t` | R/W (AUTO 모드 시에만 허용) |
| 143~174 | 32 | RSVD2 | - | R/O |
| 175~178 | 4 | DIAG (진단 카운터) | `DIAG_t.cmd_write_tick` | R/O |
| 179~206 | 28 | DIAG_RSVD | `DIAG_t.diag_rsvd` | R/O |
| 207~255 | 49 | RSVD3 | - | R/O |

> **2026-07-03 개정**: MOTOR/data 를 데이터시트에 맞춰 정정 — `present_position`·`present_velocity` `int32_t[3]` 확장(+12 byte), `present_temperature` `uint8_t[3]` 정정(−3 byte). 순증 +9 byte(28→37 byte)를 RSVD1(18→9 byte)에서 흡수하여 addr 120 이후 CMD 블록 위치를 그대로 유지함.

---

## 2. 주요 필드

### 2-1. SYSTEM (addr 46~61)

| 필드                | 타입        | 내용                                                                                                                        |
| ----------------- | --------- | ------------------------------------------------------------------------------------------------------------------------- |
| `degraded_cnt[8]` | uint8_t[] | 통신 오염도 % — [0]=uart2, [1]=uart4, [2]=uart6, [3]=i2c(미사용), [4~7]=RSVD                                                      |
| `hw_reset`        | uint8_t   | bitfield: bit0=UART2, bit1=UART4, bit2=UART6, bit3=I2C                                                                    |
| `hw_fatal`        | uint8_t   | 동일 비트 배치 — 채널 LS_OFFLINE 시 set                                                                                            |
| `hw_error`        | uint8_t   | 동일 비트 배치 — 채널 HC_WARN 이상 시 set                                                                                            |
| `sys_state`       | uint8_t   | **[개정 목표]** 운용 상태 `DPCB_STATE_e` (0=CTRL / 1=HOLD / 2~8=FSM / 9=RSVD / 10=ERROR). ~~구: `SYSTEM_STATE_e`(0=INIT~5=FAULT)~~ |
| `realtime_tick`   | uint32_t  | TIM5 카운터 — DPC_B 가동 시간 [ms]                                                                                               |

> **2026-07-03 상태 구조 개정 (코드 미반영, 개정 대기)**: addr 57 `sys_state` 를 estop/fault용 `SYSTEM_STATE_e` → 운용 `DPCB_STATE_e` 로 repurpose. 기존 `SYSTEM_STATE_e`(payload_state)는 내부 전용 유지(발행 중단), 하드웨어 FAULT 는 `hw_fatal`(55) 로 노출. addr 127 `deploy_fsm` → `sys_state_target`(`DPCB_STATE_e` 목표값) 리네이밍 동반. 상세: [dpcb_opmode.md](dpcb_opmode.md) §1·§7.

### 2-2. MOTOR/data (addr 74~110, 37 byte)

| 필드 | 타입 | addr | 스케일 | 비고 |
|------|------|------|--------|------|
| `present_position[3]` | int32_t | 74~85 | ×1 [pulse] | 멀티턴(CUR_POSITION) 대응 — ID=2/3/4 순서 |
| `present_velocity[3]` | int32_t | 86~97 | ×0.229 [rev/min] | 데이터시트 4 byte |
| `present_current[3]` | int16_t | 98~103 | ×2.69 [mA] | |
| `present_temperature[3]` | uint8_t | 104~106 | ×1 [°C] | 소스 u8 그대로 |
| `hardware_error[3]` | uint8_t | 107~109 | raw | 소스 = `dyn_ctrl.hardware_error`(addr70) |
| `uart6_state` | STATE_t | 110 | - | USART6(Dynamixel) 채널 종합 상태 |

### 2-3. DIAG (addr 175~206, 32 byte)

- `cmd_write_tick`(uint32_t, addr 175~178): 진단 카운터 — Orin 이 CMD/MOT 에 마지막으로 쓴 시각
- `diag_rsvd[28]`(addr 179~206): **DIAG_RSVD** — 진단 확장 예약

### 2-4. CMD/MOT (addr 128~142)

- `torque_en[3]`, `goal_position[3]`(int16_t), `goal_current[3]`(int16_t, default=750)
- AUTO 모드일 때만 쓰기 허용 (`mtr_lock` 판단)

---

## 3. MARSHAL 매핑 (rd_map_dpcb.c)

### 3-1. MARSHAL_PUBLISH: PERIPHERAL → reg R/O 영역 (systemTask 10ms 주기 + rs485Task 요청 직전)

> **2026-08-03 호출처 추가**: systemTask 주기 발행 외에 **rs485Task 가 Orin 요청 처리 직전에도 PUBLISH 를 호출**(request-synchronous snapshot). 응답이 항상 요청 시점 스냅샷이 되어 발행 주기 vs 요청 주기 비트로 생기던 중복/스테일 샘플이 제거됨. systemTask 주기 발행은 유지(패널·DPC-A 등 내부 소비자용). 세부: [dpcb_task.md](dpcb_task.md) §3-1.


**SYSTEM 영역 — 구현 완료 (현행 코드 기준):**
```
payload_state              → reg.sys.sys_state
tim_cnt                    → reg.sys.realtime_tick
hw.reset/fatal/error.raw   → reg.sys.hw_reset / hw_fatal / hw_error
deg_pct(uart2/4/6)         → reg.sys.degraded_cnt[0/1/2]
```
> **[개정 목표]** `sys_state` 발행 소스를 `payload_state`(SYSTEM_STATE_e) → **`DPC_CTL.STATE`(DPCB_STATE_e)** 로 교체 예정. `payload_state` 는 내부 전용화(발행 중단). 상세: [dpcb_opmode.md](dpcb_opmode.md) §7-1.

**MOTOR / 채널 / 센서 / 패널 영역 — 구현 완료:**
```
MOT[i].dyn_ctrl.ram.state.present_position   → reg.motor_data.present_position[i]    (int32 직접 복사)
MOT[i].dyn_ctrl.ram.state.present_velocity   → reg.motor_data.present_velocity[i]    (int32 직접 복사)
MOT[i].dyn_ctrl.ram.state.present_current    → reg.motor_data.present_current[i]     (int16)
MOT[i].dyn_ctrl.ram.state.present_temperature→ reg.motor_data.present_temperature[i] (u8 직접 복사)
MOT[i].dyn_ctrl.hardware_error               → reg.motor_data.hardware_error[i]      (state 아닌 ctrl 필드)
DPCB_uart6.error.state → reg.motor_data.uart6_state
DPCB_uart2.error.state → reg.uart2.state
DPCA_uart4.error.state → reg.sensor_dpca.uart4_state
A_PROX_DATA / A_CON_DATA → reg.sensor_dpca.prox_contact / lock_contact
CON_DATA                 → reg.sensor_dpcb.lock_contact
PANEL.SW{1~6}_state      → reg.sensor_dpcb.ex_sw[0~5]
```

> **인코딩 주의(코드 기준 통일 완료)**: SPDT(SW3~6) = `0=IDLE(mid) / 1=UP / 2=DOWN` (`peripheral_exio_t` 주석 기준). 레지스터 헤더 주석을 이 정의로 수정함. SPST(SW1/SW2) = `0=OFF / 1=ON`.
> **인덱스↔ID**: `MOT[0]=ID2 / MOT[1]=ID3 / MOT[2]=ID4` (레지스터 "2/3/4 순서"와 일치).
> **스케일**: velocity/current/temperature/position 모두 소스·타깃 LSB 동일 → 산술 변환 없이 직접 복사.

**구현 방식**: SYSTEM 영역과 동일하게 `taskENTER_CRITICAL()` 진입 전 전체 스냅샷 → 단일 CRITICAL 블록 일괄 기입. `p == NULL` 시 MOTOR/센서/패널은 스킵(0 유지), SYSTEM·채널 상태는 정상 발행.

**미구현 (TODO):**
```
reg.sys.degraded_cnt[3]     → i2c(MCP23017) 채널 오염도 — Checker 미구현, 0 유지
reg.sensor_dpcb.panel_state → i2c(MCP23017) 채널 종합 상태(addr 73) — 미기입(0 유지)
```
> 두 항목 모두 i2c 채널 Checker/state 소스 미구현이 원인. i2c Checker 구현 시 연동 예정 (`rd_map_dpcb.c` 내 TODO 주석 표기).

### 3-2. MARSHAL_CONSUME: reg cmd 영역 → PERIPHERAL (Step 7 예정)

```
reg.cmd_dpca.*            → DPCA_PACKET.tx.Data  (dpcaTask가 전송)
reg.cmd_dpcb.sys_state_target → DPC_CTL.STATE  (1회성 소비, mask 폐기 — 아래)
reg.cmd_dpcb.mode         → DPC_CTL.MODE   (1회성 target, 패널 토글+Orin 공유)
reg.cmd_dpcb.*(EN/servo/light) → PERIPHERAL.EN_*/SERVO_EN/LIGHT_EN
reg.cmd_mot.*             → PERIPHERAL.MOT[i].dyn_ctrl.ram.cmd.*
```

> **`sys_state_target`·`mode` 1회성 소비 (2026-08-03 확정)**: `!=0xFF` 일 때만 `DPC_CTL.STATE`/`.MODE` 반영 후 0xFF 클리어 → 추가 소비 차단(FSM 자기전이 미간섭). **입력 mask 폐기 = Orin 자유 접근**(estop형 강제 0/10, 중도실패 재시작). 코드/근거: [dpcb_opmode.md](dpcb_opmode.md) §4, 배선 계획 [plan.md](plan.md) §3-1.

---

## 4. 접근 제어 (DISPATCH)

- Region LUT 15개 영역, 각 영역 `REG_ACC_R / W / RW` + `needs_unlock` 플래그 보유
- DEFINE(1~15): `sys_write_mode == UNLOCK` 시에만 쓰기 허용
- CMD_MOT(128~142): AUTO 모드 아닐 때 `mtr_lock=1` → `ORIN_ERR_ACCESS` 반환
- CMD_DPCA(120~121) / CMD_DPCB(122~127): 모드 무관 항상 쓰기 허용
- REBOOT: `reboot_pending=1` → rs485Task 에서 **응답 DMA TX 실제 완료(`gState==HAL_UART_STATE_READY`)+2ms 대기 후** `NVIC_SystemReset()` (`wr==RET_OK` 조건부, 2026-08-03 응답 유실 방지 강화 — 구: WRITE 호출 직후 즉시 리셋으로 응답 유실 가능)

---

## 5. Orin 명령(Instruction) 와이어 포맷 (rd_comm_orin, 2026-08-03)

노드 ID: `ORIN_MY_ID = 0xD1`(구 0xE2) / `ORIN_MASTER_ID = 0x01`. Orin 브리지 `PacketID::DPC_B`(`rd_comm.hpp`) 와 **반드시 동일값** — 한쪽만 바꾸면 ID 필터에서 조용히 폐기되어 에러 카운터도 안 오르는 "무응답" 증상.
버퍼: `ORIN_DATA_BUF_SIZE = 256`(구 90, ECU_V3 정렬) — 요청 가능 최대 `rlen = 256`.

| inst | 코드 | 파라미터 | 응답 Data | 비고 |
|------|------|----------|-----------|------|
| PING | 0x01 | - | - | |
| READ | 0x02 | `[AddrLo,Hi, LenLo,Hi] × n` (4×n B) | `Err(1) + seg0│seg1│…` 요청순 연접 | **멀티세그먼트**. `n=1` 은 기존 단일 구간과 완전 동일(하위호환) |
| WRITE | 0x03 | `[AddrLo,Hi, Data…]` (최소 3B) | `Err(1)` | `data_len<3` 조기이탈에도 tx 채움(직전 응답 잔류 방지) |
| REBOOT | 0x08 | - | `Err(1)` | 응답 TX 완료 후 리셋(위 §4) |

- **RW(0x04) 의도적 미지원**(2026-08-03 결정): ECU_V3 의 RW 는 200Hz 제어 경로 전용, DPC/PCU 는 20Hz READ + 이벤트성 WRITE 로 충분. 미지원 inst 는 `RD_ORIN_HANDLE` default 가 `ORIN_ERR_INST` 로 정상 거절.
- **멀티세그 READ 원자성 주의**: 세그먼트마다 `DISPATCH_READ` CRITICAL 을 개별 진입 → 세그먼트 간 동일시점 스냅샷은 **미보장**. 누적 길이가 응답 버퍼 초과 시 `ORIN_ERR_DATA_LEN` 거절.
