# PRA 레지스터 맵 구조

> 최종 갱신: 2026-09-17 (Step 2 — `rd_register_pra.h` 코드화 완료, 사용자 검증 대기)
> 상위: [pra_overview.md](pra_overview.md)
> 원본: `pra_config_wiki/raw/register_map_pra.CSV` (CP949 인코딩), `raw/Modbus Packet.xlsx`
> 코드: **`stm_ws/PRA/Core/Inc/rd_register_pra.h`** (256B packed struct, DPC_B `rd_register_dpcb.h` 컨벤션)

> **상태 = 코드화 완료(Step 2), 사용자 교차검증 대기(Step 3)**. 256 byte / 1 addr = 1 byte / Little-Endian / 실제값 = raw × Scale. 검증 체크리스트: [plan.md](plan.md) §3-1.

---

## 1. 레지스터 맵 개요 (256 byte)

| 주소      | 크기  | 섹션             | 내용                                                  | R/W |
| ------- | --- | -------------- | --------------------------------------------------- | --- |
| 0~15    | 16  | DEFINE         | 시스템 설정 파라미터 (sys_write_mode 잠금키)                    | R/W |
| 16~28   | 13  | SYSTEM         | degraded_cnt, hw_error/fatal/reset, sys_state, tick | R/O |
| 29~37   | 9   | RESERVED       | 미래 확장                                               | R/O |
| 38~47   | 10  | SENSOR / STATE | NTC×3(int8), 전류/입출력 전압(**int16 ×0.1**), state — **PRA 자체 ADC** (2026-10-13 +3B) | R/O |
| 48~64   | 17  | SENSOR / BMS   | BMS 전압/전류/SOC/최고온도/고장상태(0x98 8B) (CAN1) | R/O |
| 65~67   | 3   | SYSTEM / cmd   | ctr_mode[3] — 메인파워/AUX1/AUX2 스위치                 | R/W |
| 68~79   | 12  | RESERVED       | 미래 확장 (2026-10-13: 15→12, SENSOR/ADC +3 흡수)       | R/O |
| 80~99   | 20  | DIAG           | cmd_write_tick(80~83) + RESERVED_DIAG(84~99)        | R/O |
| 100~255 | 156 | RESERVED       | 미래 확장 (읽으면 0x00 / 쓰기 무시)                            | R/O |

> CSV `[DIAG]` 섹션 헤더는 `[80:255] (176 byte)` — DIAG(80~99) + 후행 RESERVED(100~255)를 한 헤더로 묶음. 논리적으로는 위 표처럼 DIAG(80~99, 20 byte) / RESERVED(100~255, 156 byte) 로 분리 취급.
> **2026-09-17 정리 완료**: DIAG 헤더 주소(→80 시작), degraded_cnt Group(→SYSTEM), addr 100~255 RESERVED(size 156), NTC=int8_t, SENSOR/STATE=자체 ADC — 초기 지적사항 전부 CSV 반영됨.

---

## 2. 섹션별 필드

### 2-1. DEFINE (addr 0~15) — R/W (sys_write_mode 잠금)

| addr | 필드 | 타입 | 기본값 | 범위 | 비고 |
|------|------|------|--------|------|------|
| 0 | `SYS_WRITE_MODE` | uint8_t | 0 | 0~1 | DEFINE 영역 쓰기 허용 키 |
| 1 | `ERR_TIMEOUT` | uint8_t | 15 | 10~500 ms | HW timeout |
| 2 | `FATAL_TIMEOUT` | uint8_t | 10 | 0~100 (×100ms) | 0 = FATAL 감지 X |
| 3 | `ERR_CNT` | uint8_t | 5 | 0~10 | |
| 4 | `FATAL_CNT` | uint8_t | 10 | 0~100 | 0 = FATAL 감지 X. Checker 임계 카운트 |
| 5 | `hw_reset` | bitfield | 0 | - | `CAN1｜UART2｜I2C1｜RSVD×5` — 1 세트 시 해당 HW 소프트 리셋 트리거 |
| 6~15 | `RESERVED` | - | - | - | 미래 확장(10 byte) |

> DEFINE 기본값 체계는 ECU/DPC_B 와 동일(ERR_TIMEOUT=15, FATAL_TIMEOUT=10, ERR_CNT=5, FATAL_CNT=10).
> **`hw_reset`(addr5, R/W) = Orin 이 PRA 특정 채널을 수동 리셋하는 기능** (2026-09-17 확정, 유지). 비트: **CAN1(bit0)｜UART2(bit1, Orin 채널)｜I2C1(bit2)｜RSVD×5** — degraded_cnt 인덱스 순서와 통일(Step3). 세트 시 해당 HW 소프트 리셋 트리거.

### 2-2. SYSTEM (addr 16~28) — R/O

| addr | 필드 | 타입 | 비고 |
|------|------|------|------|
| 16 | `degraded_cnt[0]` | uint8_t | CAN1 오염도 %. `(uint8_t)((input×26)>>8)` (0~1000 → 0~100%) |
| 17 | `degraded_cnt[1]` | uint8_t | uart2(=Orin 채널, ECU 레거시 명칭) |
| 18~19 | `degraded_cnt[2-3]` | uint8_t | RSVD |
| 20 | `hw_error` | bitfield | `CAN1｜uart2` (bit0=CAN1/bit1=uart2) — 세트 시 해당 HW 소프트 리셋 트리거 (CSV 기준) |
| 21 | `hw_fatal` | bitfield | 동일 비트 — 재초기화 필요 치명 에러 |
| 22 | `hw_reset` | bitfield | 동일 비트 — 현재 활성 에러 상태 플래그 (`Hardware_Error_FLAG_t` 매핑) |
| 23 | `sys_state` | uint8_t | **`SYSTEM_STATE_e`** = `PRA_CONTROL.STATE` 미러 (0=INIT/1=RUN_OFF/2=RUN_TRN/3=RUN_ON/255=FAULT) |
| 24 | `rs485_proc_delta` | uint8_t | request→response 소요 (×0.1ms) |
| 25~28 | `realtime_tick` | uint32_t | TIM5 10kHz 카운터, ×0.1ms — PRA alive time |

> `degraded_cnt`(16~19) Group = SYSTEM (2026-09-17 정정). **인덱스·HW 비트 순서 통일(Step3)**: `degraded_cnt[0]/hw_*bit0=CAN1`, `[1]/bit1=uart2`, `bit2=I2C1`. hw_error/fatal/reset 미러 3종(20~22)은 DEFINE `hw_reset`(addr5) 수동 리셋 기능과 연동 — 상세 의미는 `rd_system.h` `Hardware_Error_FLAG_t` 이식 시 확정.
> **sys_state = 운용 전원 FSM(`PRA_CONTROL.STATE`) 미러** (2026-10-04 재정의, INIT/RUN_OFF/RUN_TRN/RUN_ON/FAULT). 구 ECU 레거시(MANUAL/AUTO/ESTOP) 폐기. 통신/채널 health 는 hw_error/hw_fatal/degraded_cnt 로 별도 노출(전원과 독립). 상세: [pra_opmode.md](pra_opmode.md) §1·§4-7.
> `realtime_tick` 단위 ×0.1ms(ECU 정렬). Orin `RdClockSync` 시계동기 대상.

### 2-3. SENSOR (addr 38~60) — R/O

**SENSOR / STATE (38~47, 10 byte)** — PRA **자체 ADC** (ADC1 CH2~CH7). 2026-10-13: current/voltage int8→**int16 ×0.1**(정밀도), float→int16(×10) 변환은 `RD_MAP_MARSHAL_PUBLISH`:

| addr | 필드 | 타입 | 스케일 | ADC 채널 | 비고 |
|------|------|------|--------|---------|------|
| 38 | `NTC[0]` | int8_t | ×1 [°C] | CH2 (PA2) | 인덱스 순서 그대로(Orin 은 최대값만 사용) |
| 39 | `NTC[1]` | int8_t | ×1 [°C] | CH3 (PA3) | |
| 40 | `NTC[2]` | int8_t | ×1 [°C] | CH4 (PA4) | |
| 41~42 | `current_input` | **int16_t** | **×0.1 [A]** | CH5 (PA5) | IOUT |
| 43~44 | `voltage_input` | **int16_t** | **×0.1 [V]** | — | 의미 기준 매핑(`SENSOR_Voltage_IN`) |
| 45~46 | `voltage_output` | **int16_t** | **×0.1 [V]** | — | 의미 기준 매핑(`SENSOR_Voltage_OUT`), 프리차징 판정 |
| 47 | `state` | bitfield | - | - | 센서(ADC) 채널 상태 |

> **자체 ADC 확정 (2026-09-17)**: SENSOR/STATE 는 EX RS485 외부 센서가 아닌 **PRA 자체 ADC1 스캔 측정값**. `main.c` ConvCplt 스냅샷(`my_adc_register`) → 변환(NTC Steinhart-Hart / 전압·전류) → 레지스터 발행. **레지스터 index 순서 = ADC rank 순서(CH2→CH7)**. ⚠ NTC 보드 핀 라벨(NTC3/2/1 = CH2/3/4)과 레지스터 index(NTC[0/1/2]) 가 **역순 대응** — `NTC[0]`=CH2=보드 NTC3. 코드화 시 매핑 주의.

**SENSOR / BMS (48~64, 17 byte)** — 필수 3종(0x90/0x92/0x98) 전용 (2026-10-13: SENSOR/ADC +3 로 45→48 밀림). 상세·원천 ID·비트정의 = [BMS_protocol_can.md](BMS_protocol_can.md):

| addr | 필드 | 타입 | 스케일 | 원천 ID | 비고 |
|------|------|------|--------|---------|------|
| 48~49 | `voltage` | uint16_t | ×0.1 [V] | 0x90 B0~1 | 팩 총전압 |
| 50~51 | `current` | **int16_t** | ×0.1 [A] | 0x90 B4~5 | **30000 offset**, signed((raw−30000)) |
| 52~53 | `soc` | uint16_t | ×0.1 [%] | 0x90 B6~7 | State of Charge |
| 54 | `max_cell_temp` | uint8_t | ×1 [°C] | 0x92 B0 | 40 offset |
| 55~58 | `error_lsb` | uint32_t | - | 0x98 B0~3 | 고장 비트 [0:3] (전압/온도/전류·SOC/편차) |
| 59~62 | `error_hsb` | uint32_t | - | 0x98 B4~7 | 고장 비트 [4:7] (MOS/내부모듈/검출모듈/고장코드) |
| 63 | `delta_tick` | uint8_t | **×0.1 [s]** (BMS 예외) | (PRA 내부) | 0xFF: 25.5s 이상 지연=오염. ⚠ 타 센서 ×0.1ms 와 다름(1Hz BMS 전용) |
| 64 | `state` | STATE_t | - | (PRA 내부) | **CAN1 포트 채널 종합 상태** (STATE_t lifecycle+health) |

> BMS(48~64)는 SENSOR/ADC 와 달리 **CAN1 수집값**([pra_peri.md](pra_peri.md) §3). `delta_tick`(63) 로 신선도 감시.
> **재정의 이력**: (2026-10-10) 0x93 유래 필드 폐기·0x98 전체 8바이트를 `error_lsb`+`error_hsb` 수용·16B→17B. (2026-10-13) SENSOR/ADC int16 확장(+3B)으로 영역 45→48 밀림. ✅ `#define`/레이아웃/`REGISTER_t`/static_assert(총256) 동기화 완료.
> `error_*`(55~62) 는 BMS 가 CAN 으로 보고한 고장 비트, `state`(64)=**CAN1 통신 포트** 채널 상태(STATE_t) — 서로 다른 의미.

### 2-4. SYSTEM / cmd (addr 65~67) — R/W  *(2026-10-13: SENSOR/ADC +3 로 62→65 밀림)*

| addr | 필드 | 타입 | 범위 | 비고 |
|------|------|------|------|------|
| 65 | `ctr_mode[0]` | uint8_t | 0~1 | **main power**(high-current bus, BOOT_FSM) on/off |
| 66 | `ctr_mode[1]` | uint8_t | 0~1 | **AUX1** (MCU 계 = **내부 임베디드** 부팅용) |
| 67 | `ctr_mode[2]` | uint8_t | 0~1 | **AUX2** (ORIN 계 = **Orin 전원**) |

> Orin/로컬 스위치가 기입하는 전원 온오프 명령. AUX 역할 확정(2026-10-04): AUX1=내부 임베디드/AUX2=Orin. 스위치 정책·시퀀스는 [pra_opmode.md](pra_opmode.md) §2·§4.
> ✅ 레지스터 헤더 `REG_CMD_POWER_OFFSET`(65)·쓰기감시 `REG_CMD_POWER_S/E_OFFSET`(65/67) 동기화 완료(2026-10-13).

### 2-5. DIAG (addr 80~99, 20 byte) — R/O

CSV `[DIAG]` 섹션 헤더 `[80:255] (176 byte)` = 아래 DIAG(80~99) + 후행 RESERVED(100~255).

| addr | 필드 | 타입 | 비고 |
|------|------|------|------|
| 80~83 | `cmd_write_tick` | uint32_t | 상위 레벨이 cmd 에 마지막 write 한 시각 [tick] |
| 84~99 | `RESERVED_DIAG` | - | 향후 센서 진단 확장(16 byte) |
| 100~255 | `RESERVED` | - | 미래 확장(156 byte, 읽으면 0x00 / 쓰기 무시) |

---

## 3. Orin 정합

- **PRA = Orin PCU 노드 확정** (PacketID `0xA1`). STM 개발문서는 `PRA`, Orin/외부 관점은 `PCU` 로 지칭. [pra_overview.md](pra_overview.md) §6.
- Orin 브리지 `rd_register_pcu.hpp` 는 `DEFINE(0:16)/SYS(46:16)/POWER(96:8)/CMD(128:4)/DIAG(224:32)` 레이아웃 — **현재 PRA 맵과 주소 구획이 다름** (Orin 측 placeholder, "레지스터 미확정").
- **정합 방향 확정 (2026-09-17)**: **본 PRA 레지스터맵을 기준**으로 Orin `rd_register_pcu.hpp` 를 수정 (Orin→PRA 가 아닌 PRA→Orin 방향). **PRA 개발 완료 이후** 착수. [plan.md](plan.md) §4-1.
