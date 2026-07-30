# DPC_B 레지스터 맵 구조 및 MARSHAL 매핑

> 최종 갱신: 2026-07-04  
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

### 3-1. MARSHAL_PUBLISH: PERIPHERAL → reg R/O 영역 (systemTask 10ms 주기)

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
reg.cmd_dpca.*       → DPCA_PACKET.tx.Data  (dpcaTask가 전송)
reg.cmd_dpcb.*       → PERIPHERAL.EN_*/SERVO_EN/LIGHT_EN + DPC_CTL (모드·FSM 전환)
reg.cmd_mot.*        → PERIPHERAL.MOT[i].dyn_ctrl.ram.cmd.*
```

---

## 4. 접근 제어 (DISPATCH)

- Region LUT 15개 영역, 각 영역 `REG_ACC_R / W / RW` + `needs_unlock` 플래그 보유
- DEFINE(1~15): `sys_write_mode == UNLOCK` 시에만 쓰기 허용
- CMD_MOT(128~142): AUTO 모드 아닐 때 `mtr_lock=1` → `ORIN_ERR_ACCESS` 반환
- CMD_DPCA(120~121) / CMD_DPCB(122~127): 모드 무관 항상 쓰기 허용
- REBOOT: 응답 먼저 송신 후 `reboot_pending=1` → rs485Task에서 `NVIC_SystemReset()`
