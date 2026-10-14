# PRA 페리페럴 구조 및 통신 채널

> 최종 갱신: 2026-09-21 (Step7 진행 — 페리페럴 계층 §6 코드 검토·잔여작업)
> 상위: [pra_overview.md](pra_overview.md)

> 본 문서의 핀맵/채널은 **`PRA.ioc` + CubeMX 생성 `main.c` 에서 확인된 확정 값**. 구조체/드라이버 계층은 대부분 **미생성(계획)** 이며 ECU/DPC_B 이식 예정.

---

## 1. 통신 채널 일람

| 채널 | 속도 | 대상 | 방향 제어 | 전담 태스크 | 상태 |
|------|------|------|----------|------------|------|
| USART6 | 921600 8N1 | **Orin RS485 modbus** | DIR=PC8, TC→RX 자동 복귀 | `rs485Task` (`RD_TASK_RS485`) | **구현 (Step4·5)** |
| USART1 | async | **EX RS485 — 미사용** (미래 확장용) | DIR=PA8 | 없음 | MX on / 기능 미사용 |
| CAN1 | 250 kbps | **BMS** | - | `can1Task` | 골격 (Step8) |
| I2C1 | Standard (100 kHz) | **MCP23017** 패널 (SW+LED) | RST=PC13 | (미배정) | 골격 |

- RS485 방향 제어 **구현 완료**: TX 시 DIR SET(`RD_RS485_Transmit`, RE/IDLE off) → TC 인터럽트(`RD_RS485_IRQ_HANDLER`)로 DIR RX 복귀 + **RE/IDLE 재활성**. ISR 배선은 `stm32f4xx_it.c` USART6_IRQHandler. DIR=PC8
- USART6 = Orin(마스터 0x01, 노드 `0xA1`=PCU) 레지스터맵 modbus. `rd_comm_orin`(PING/READ 멀티세그/WRITE/REBOOT) + `rd_map_pra`(DISPATCH LUT) 이식 완료. 상세: [pra_task.md](pra_task.md) §2·§3, [plan.md](plan.md) §3-2
- **USART1(EX RS485) = 미사용 (확정, 2026-09-17)**: 통신형 BMS 인터페이스가 RS485/CAN 2종으로 흔해 **미래 확장 대비**로 핀·페리페럴만 열어둔 것(`.ioc` 상 on). **현 시스템은 CAN1(BMS) 만 사용** → USART1 은 기능적으로 미사용. 태스크 미배정.

---

## 2. ADC1 — 배터리/온도 감시 (6채널, DMA)

CubeMX ADC1 스캔 변환 6채널, DMA + `HAL_ADC_ConvCpltCallback` 스냅샷(`main.c:599`).

| Rank | 채널 | 핀 | 라벨 | 용도 | 레지스터 | 샘플링 |
|------|------|----|------|------|---------|--------|
| 1 | ADC_CHANNEL_2 | PA2 | `NTC3_ADC2` | 온도 | `NTC[0]`(38) | 480 cycles |
| 2 | ADC_CHANNEL_3 | PA3 | `NTC2_ADC3` | 온도 | `NTC[1]`(39) | 480 cycles |
| 3 | ADC_CHANNEL_4 | PA4 | `NTC1_ADC4` | 온도 | `NTC[2]`(40) | 480 cycles |
| 4 | ADC_CHANNEL_5 | PA5 | `IOUT_ADC5` | 전류 ×1 | `current_input`(41) | 15 cycles |
| 5 | ADC_CHANNEL_6 | PA6 | `VOUT_ADC6` | 전압 (입력측) | `voltage_input`(42) | 15 cycles |
| 6 | ADC_CHANNEL_7 | PA7 | `VOUT_ADC7` | 전압 (출력측) | `voltage_output`(43) | 15 cycles |

- 페리페럴 개요 대응: **NTC ×3 + 전압 ×2 + 전류 ×1** (총 6ch), 레지스터 `SENSOR/STATE`(38~43) 로 발행 — **자체 ADC 확정** ([pra_register.md](pra_register.md) §2-3)
- NTC 는 480 cycles(느림·안정), 전류/전압은 15 cycles(빠름) — 신호 특성별 샘플링 분리
- ⚠ **NTC index 역순 주의**: 레지스터 `NTC[0/1/2]` = ADC CH2/3/4 = 보드 핀 라벨 **NTC3/2/1** (index 와 핀 번호 역순)
- **변환 헬퍼 (`main.c`, 구현 완료)**:
  - `Convert_NTC_To_Celsius()` — Steinhart-Hart Beta 공식(`NTC_BETA`/`NTC_R25`, R_ref=10k). ADC≥4095 → 오픈/쇼트 예외 `-999.0f`
  - `Convert_ADC_To_Force()` — `(adc - offset) × sensitivity` (범용 변환 헬퍼)
- **프리차저 순차 제어의 핵심 입력** = `voltage_output`(CH7) 전압 측정 (프리차징 완료 판정), `current_input`(CH5) 인러시 감시 (opmode §2 참조)
- ⚠ **Vout 계측 제약 (2026-10-10, HW 리비전까지)**: 프리차저 body diode 역류 결함의 임시조치로 **PC 출력↔P릴레이 출력단 사이에 단방향 다이오드** 설치 → `SENSOR_Voltage_OUT`(Vout)은 **프리차저 ON 상태에서만 유효 계측**(PC OFF 시 다이오드 장벽으로 Vout≈0 오독). 이로 인해 BOOT_FSM `5_ON` Vin↔Vout 비교는 SW적으로 임시 중지(opmode §4-7 ⚠ 참조)

---

## 3. CAN1 — BMS 통신

- 250 kbps (Prescaler 10, BS1/BS2 `.ioc` 정의), **CAN 2.0B 29-bit 확장 ID**
- 대상: BMS — 요청→응답 폴링. **필수 수집 3종 = 0x90(전압/전류/SOC)·0x92(최고 셀온도)·0x98(에러상태)**. 호스트=0x40/BMS=0x01 고정, ID=`0x18[데이터ID]4001`(응답)
- **프로토콜 상세·PCAN 실측 디코드·레지스터 매핑 = [BMS_protocol_can.md](BMS_protocol_can.md)** (2026-10-10 신설, Step8 착수 문서). 원본: [raw/BMS protocol.md](raw/BMS%20protocol.md)
- 수집 데이터 → 레지스터맵 `SENSOR/BMS`(addr 45~60) 발행 예정. 필드 상세: [pra_register.md](pra_register.md) §2-3. ⚠ 전류 offset 30000·0x98 에러 저장소 미결은 BMS_protocol_can.md §6 참조
- **CAN RX 파서 미구현** — `can1Task` 빈 루프. ECU 의 CAN 처리 패턴 참조 예정

---

## 4. GPIO 파워단 / 인디케이터

`.ioc` GPIO 라벨 기준.

| 핀 | 라벨 | 방향 | 용도 |
|----|------|------|------|
| PB10 | `RP_EN` | OUT | 메인파워 **P 릴레이**(+) |
| PB12 | `RN_EN` | OUT | 메인파워 **N 릴레이**(−) |
| PB13 | `PC_EN` | OUT | **솔리드스테이트 프리차저** enable |
| PB14 | `AUX1_EN` | OUT | 서브파워 **AUX1 SSR** = **내부 임베디드**(MCU 계) 부팅용 — 부팅 자동 ON (opmode §4) |
| PB15 | `AUX2_EN` | OUT | 서브파워 **AUX2 SSR** = **Orin 전원**(ORIN 계) — SW2 ≥3s 토글 (opmode §4) |
| PC11 | `LED_G` | OUT | 녹색 상태 LED (alive 블링크, defaultTask) |
| PC12 | `LED_R` | OUT | 적색 상태 LED (FAULT 표시 예정) |
| PC13 | `exio_RESET` | OUT | MCP23017 리셋 |

- **메인파워 시퀀스**: `PC_EN`(프리차저) → 전압 상승 확인 → `RP_EN`/`RN_EN`(릴레이) 순차 투입 → `PC_EN` 해제. 전압 측정 기반. 상세: [pra_opmode.md](pra_opmode.md) §2
- **레지스터 cmd 대응**: `ctr_mode[0]`(addr61)=메인파워 / `ctr_mode[1]`(62)=AUX1 / `ctr_mode[2]`(63)=AUX2 ([pra_register.md](pra_register.md) §2-4)

---

## 5. I2C1 — MCP23017 패널 (SW + LED/BUZ)

- Standard mode (100 kHz), RST=PC13(`exio_RESET`). 드라이버 `MCP23017.{h,c}` 존재, EXIO 스켈레톤 구현(`rd_peripheral_pra.c`)
- **EXIO 핀 매핑 (`rd_peripheral_pra.h`, GPA 포트)**:

| MCP 핀 | 매크로 | 방향 | 필드 | 용도 |
|--------|--------|------|------|------|
| GPA0 | `EXIO_SWA1` | IN | `SW1_state` | **SW1 = 메인버스 스위치**(BOOT_FSM 방향천이 ≥3s), opmode §4-2 |
| GPA1 | `EXIO_LED1` | OUT | `LED1_state` | 상태 LED 1 |
| GPA2 | `EXIO_SWA2` | IN | `SW2_state` | **SW2 = Orin(AUX2) 전원 토글** (≥3s), opmode §4-3. AUX1 강제복구(≥10s)는 보류 |
| GPA3 | `EXIO_LED2` | OUT | `LED2_state` | 상태 LED 2 |
| GPA4 | `EXIO_SWA3` | — | — | **DNP**(미실장) |
| GPA5 | `EXIO_LED3` | OUT | `BUZ_state` | **액티브 부저**(LED 대신) |

- GPB 포트 스위치(SW1~SW6A)는 **주석 처리**(미사용) — DPC_B 대비 패널 스위치 대폭 축소(SPST 2개 + LED 2 + BUZ)
- ✅ **EXIO 초기화 순서 준수**: `RD_EXIO_INIT` 가 RST 토글(RESET→100ms→SET)을 방향설정(`EXIO_Set_OUTPUT/INPUT`) **앞에서** 수행 — DPC_B history §22 교훈 반영됨
- ⚠ **미배선**: EXIO 갱신(`RD_EXIO_UPDATE`) 호출 태스크 없음 — 현 5태스크에 i2c 없음. `rd_peripheral_pra.c`에 "MX에서 Start_i2c 태스크 생성" TODO. (신규 태스크 or 기존 태스크 폴딩 결정 필요)

---

## 6. 페리페럴 계층 (`rd_peripheral_pra.{h,c}`) — **Step7 진행 중** (2026-09-21 통합)

DPC_B `rd_peripheral_dpcb` 기반 PRA 페리페럴 추상화 계층. **전원 GPIO(+AUX) + 자체 ADC + EXIO(MCP23017)** 를 `PERIPHERAL_t` 하나로 통합. **controlTask(`RD_TASK_CONTROL`) 에 배선 완료** — INIT 후 `for(;;){READ; WRITE;}`.

```
PERIPHERAL_t
  ├ ADC data          SENSOR_Voltage_IN/_OUT/Current_BUS (float V/A) / NTC[3] (float °C)
  │                   + adc_raw_buf[6] / my_adc_register[6] (uint32, struct 내)
  ├ CONTROL (uint8)   EN_SSP / EN_RelayP / EN_RelayN / EN_AUX1 / EN_AUX2
  ├ IO  PERIPHERAL_IO_ALL_t  GPIO_SSP(PC_EN)/RelayP(RP_EN)/RelayN(RN_EN)/AUX1(PB14)/AUX2(PB15)
  ├ EXIO peripheral_exio_t   hi2c / SW1·SW2_state(IN) / LED1·LED2·BUZ_state(OUT)
  └ oldTick / deltaTick
```

**함수 현황**:
| 함수 | 상태 |
|------|------|
| `RD_PERIPHERAL_INIT` | ✅ IO 핀 주입(Relay/SSP/AUX) + 상태 0 + `RD_EXIO_INIT` + `RD_ADC_INIT` |
| `RD_PERIPHERAL_READ` | ✅ `RD_UPDATE_ADC` 호출 |
| `RD_PERIPHERAL_WRITE` | ✅ Relay P/N·SSP·**AUX1/AUX2** write |
| `RD_ADC_INIT` / `RD_UPDATE_ADC` | ✅ 호출됨, **버퍼폭 uint16 수정 완료** (NTC[2]·Current 정정) |
| `RD_EXIO_INIT` / `RD_EXIO_UPDATE` | ✅ INIT + **루프 주기 UPDATE 배선됨** |
| `Convert_NTC/_To_Voltage(×0.0245)/_To_Current((raw−2048)×0.1465)` | ✅ (인자 uint16, 헤더 미선언) |
| `RD_TASK_CONTROL` (controlTask) | ✅ INIT + `for(;;){READ; EXIO_UPDATE; WRITE; osDelay(10);}` |

> ✅ **완료(유저)**: AUX 필드·핀주입·**WRITE 반영**, EXIO LED/BUZ 버그·**루프 UPDATE 배선**, ADC 변환·NTC[2]·Current 정정, 구조체 SENSOR_*/NTC **float 전환**, `PRA_PERIPHERAL` 정의 + controlTask 배선 + INIT 에 ADC/EXIO_INIT, **ADC 버퍼 일원화**(전역 `adc_raw_buf`→ConvCplt→struct `adc_value[]`), **osDelay(10ms) 추가**, **ADC 버퍼폭 `uint16[6]`+`(uint32*)` 캐스트(수정안 A)**.

> ✅ **실보드 수동 검증 (2026-10-04)**: **ADC 정상** — NTC×3·Vin·Vout·전류(Current_BUS) 값 정상 확인. **MCP23017 EXIO I2C 통신 정상** — SW1/SW2 가 `1` 로 읽힘 = 회로설계상 **10k 풀업**(유저)의 비눌림 상태 → read/통신 경로 정상.
>
> ✅ **Step9 전원제어 실보드 검증 (2026-10-10)**: **버튼 기반 부팅 시퀀스(BOOT_FSM)·AUX1/AUX2 포트 구동 정상.** 단 프리차저 body diode 역류 HW 결함 → 임시 다이오드 조치 → Vout 이 PC ON 시에만 유효 → **BOOT_FSM `5_ON` Vin↔Vout 비교 SW 임시 중지**(§4 ADC 주석·opmode §4-7). HW 리비전 후 원복 예정.

### 6-1. Step7 잔여 작업 (2026-09-21 재검토)

**A. ✅ ADC 버퍼 폭 — 해소 완료 (수정안 A)**
- `adc_raw_buf`/`adc_value` = `uint16_t[6]`, `HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_raw_buf, 6)`, `Convert_*` 인자 uint16 → `.ioc` DMA HALFWORD(MSIZE 16bit) 와 정합. 6채널이 슬롯당 1개씩 정상 적재. (HAL `uint32_t* pData` 는 API 타입일 뿐 전송폭 미결정)

**B. 매핑 정합 (Step10 정합 시 확인)**
- **NTC index↔버퍼 순서**: `NTC[0]=buf[2](CH4)` … 레지스터 규약([pra_register.md](pra_register.md) §2-3)은 `NTC[0]=CH2=buf[0]` → 역순. MARSHAL(Step10) 정합 시 통일
- **Vin/Vout in·out**: `Voltage_IN=buf[5](CH7)`/`Voltage_OUT=buf[4](CH6)` vs 레지스터 `voltage_input=CH6`/`output=CH7` → 반대. 확인·통일

**C. 스케일 (Step10 연계)**
- 저장은 **float**(V/A/°C) 로 정리됨. 레지스터는 int8 ×0.1V / int8 ×1°C → MARSHAL(Step10) 에서 float→int8 스케일 변환 매핑
- ⚠ (경미) `Convert_ADC_To_Voltage/Current` 의 `if(adc>=4095) return -999` 는 NTC 오픈/쇼트 검출용 관용구를 그대로 복사한 것 — **전압/전류는 만스케일 근처가 정상값**일 수 있어 오검출 소지. 전압/전류용 예외 처리는 별도 정책 권장

**D. ✅ 정리 완료 (2026-09-21)**
- 헤더 선언 추가: `RD_ADC_INIT`/`RD_UPDATE_ADC`/`Convert_NTC/Voltage/Current`
- copy 흔적 제거: `.h`/`.c` 헤더 주석 `rd_peripheral_dpcb`→`pra`, MCP23017 `//test`, `RD_TASK_CONTROL`/`READ`/ConvCplt stale 주석. (`DYN_NUM_MOTORS`/`PERIPHERAL_MOT_t` 는 유저가 기제거)
- (미결) `RD_TASK_CONTROL`(controlTask) = 페리페럴 I/O 루프. **Step9 전원 FSM** 통합/별도 배치는 추후 결정

> LED_R(PC12)·LED_G(PC11) MCU 직결 — LED_G=defaultTask 토글, LED_R(FAULT) 편입 여부 미정.
