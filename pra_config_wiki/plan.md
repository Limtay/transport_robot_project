# PRA 구현 플랜 및 진행 상태

> 최종 갱신: 2026-09-17 (프로젝트 착수 — 초기 세팅 문서화)
> 상위: [pra_overview.md](pra_overview.md)
> 이력 상세: [history.md](history.md)

> DPC_B 의 Step 체계를 계승. PRA 는 **ECU/DPC_B 검증 자산 이식** 이 기본 전략 — 신규 작성보다 참조·수정.

---

## 1. 구현 단계(Step) 정의

| Step | 내용                                                         | 담당  | 상태                            |
| ---- | ---------------------------------------------------------- | --- | ----------------------------- |
| 0    | CubeMX HW 세팅 (ADC/CAN/USART×2/I2C/GPIO/DMA)                | 사용자 | **완료**                        |
| 1    | 레지스터 맵 초안 작성 (CSV/xlsx)                                    | 사용자 | **완료 (초안)**                   |
| 2    | `rd_register_pra.h` 작성 (CSV → 256B packed struct, ECU 참조)  | 클로드 | **완료 (2026-09-17)**           |
| 3    | 레지스터 맵 ↔ CSV 교차 검증 (사용자 검증) → 이후 4·5 동시 진행                 | 사용자 | **완료 (2026-09-17)**           |
| 4    | 통신부 이식 (`rd_comm_orin` Orin RS485 + `rd_common.h` STATE_t) | 클로드 | **완료 (2026-09-17)**           |
| 5    | modbus 통합 (`rd_map_pra` DISPATCH + MARSHAL 스텁)             | 클로드 | **완료 (2026-09-17)**           |
| 6    | `rd_system.c` Checker/reset (ECU 참조)                       | 클로드 | **완료 (2026-09-17, USART6만)** — CAN1/I2C1 은 페리페럴 후 |
| 7    | **페리페럴 구현** (`rd_peripheral_pra` 본체: 전원GPIO+AUX+ADC + **MCP23017 EXIO** + controlTask 연동) | 협업  | **완료 + 실보드 ADC/EXIO 검증 (2026-10-04)** |
| 8    | **BMS CAN 파서** (CAN1 요청→응답 폴링 → SENSOR/BMS + Checker)      | 협업 | **8-1~8-6 구현 완료·실기 검증**([pra_bms_can.md](pra_bms_can.md)) — 폴링·자기복구·디코드→reg·채널 state(addr61)·delta_tick. hw_reset 미구현(자동복구). PHY: HSI 허용오차 → SJW=4/SP75% 임시해결, HSE 크리스털은 리비전. **코드 대부분 close**. 남음=8-7 Orin 연동 / 8-8 리비전 실 BMS·과열(프로젝트 종료) |
| 9    | **전원 제어 로직** (스위치 기반 on/off + BOOT_FSM 순차) — controlTask `RD_CONTROL_LOOP` | 협업  | **✅ 실보드 검증 통과 (2026-10-10)** — 버튼 부팅 시퀀스·AUX 구동 정상. ⚠ 프리차저 body diode HW 결함 → 5_ON Vin↔Vout 비교 임시 중지(HW 리비전 후 원복). opmode §4-7 |
| 10   | **MARSHAL PUBLISH/CONSUME 본체** (SENSOR_ADC/BMS 발행 + cmd_power consume) — **프로젝트 완료** | 협업  | 예정                            |

> **Step 순서 재구성 (2026-09-21, 사용자 지시)**: 구 `7 MARSHAL / 8 전원 / 9 CAN / 10 EX·패널` → 신 `7 페리페럴(EXIO 포함) / 8 BMS CAN / 9 전원제어 / 10 MARSHAL(완료)`. 페리페럴(EXIO)을 먼저 세우고 → 센서(CAN) → 제어 → 마지막에 레지스터 발행/소비(MARSHAL)를 붙여 마무리. 구 Step10 "EX RS485"는 미사용 확정으로 폐기, "MCP23017 패널"은 Step7 페리페럴에 흡수.

---

## 2. 진행 상태 (2026-09-17)

| 항목                                                                  | 상태                                                                                                                                                        |
| ------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------- |
| CubeMX 페리페럴 초기화                                                     | 완료 (`.ioc` + `MX_*_Init`)                                                                                                                                 |
| ADC DMA + ConvCplt 스냅샷                                              | 완료 (`main.c`)                                                                                                                                             |
| NTC(Steinhart-Hart)/로드셀 변환 헬퍼                                       | 완료 (`main.c`)                                                                                                                                             |
| `rd_uart` 드라이버 이식                                                   | 완료                                                                                                                                                        |
| `rd_peripheral.h` / `rd_define.h`                                   | 골격 (GPIO_IO_t 만 / 빈 파일)                                                                                                                                   |
| `rd_peripheral_pra.{h,c}` (PERIPHERAL_t: 전원GPIO+AUX+ADC+EXIO)       | **스켈레톤 (2026-09-21, 유저)** — AUX1/2 추가 완료, 미연동·잔여 TODO([pra_peri.md](pra_peri.md) §6) = **Step7 본체**                                                       |
| MCP23017 드라이버 (`MCP23017.{h,c}`)                                    | 존재 (EXIO_* 함수)                                                                                                                                            |
| 레지스터 맵 코드화 (`rd_register_pra.h`)                                    | **완료 (Step2·3 검증)**                                                                                                                                       |
| RS485 modbus (`rd_comm_orin`/`rd_map_pra`)                          | **완료 (2026-09-17, Step4·5)** — Orin READ/WRITE/PING/REBOOT + DISPATCH LUT                                                                                 |
| `rd_common.h` STATE_t 이식                                            | **완료** (DPC_B 상위호환본)                                                                                                                                      |
| `rd_system.c` (RS485 태스크 + 전역 객체 + INIT)                            | **완료**                                                                                                                                                    |
| `rd_uart` advanced 교체 (STATE_t/degraded/lifecycle/Checker/Recovery) | **완료 (Step6)**                                                                                                                                            |
| Checker / systemTask (`RD_TASK_SYSTEM`)                             | **완료 (Step6, USART6 채널만)** — CAN1/I2C1 훅 주석                                                                                                               |
| MARSHAL PUBLISH — SYSTEM 영역(hw/degraded/sys_state/tick)             | **완료 (Step6)** / SENSOR/BMS=rd_can_bms 발행(Step8-4) / **SENSOR/ADC 미연결(🔴 유저 TODO: `DATA_SENSOR_ADC_t` int8→float 변경 + PRA_PERIPHERAL→reg.sensor_adc 하네스, pra_bms_can.md §Step8-7)** / CONSUME 은 **Step10** |
| 페리페럴 본체 (rd_peripheral_pra 연동 + EXIO)                               | **완료 (Step7)** — controlTask 배선·INIT·READ(ADC)·WRITE(AUX)·EXIO UPDATE·osDelay·버퍼 일원화·**ADC 버퍼폭 uint16 수정** 완료. 잔여=매핑·스케일(Step10 정합)·헤더/copy 정리(낮음) (§4-4) |
| BMS CAN 파서                                                          | 미생성 (**Step8**) — 프로토콜([BMS_protocol_can.md](BMS_protocol_can.md)) + 작업계획([pra_bms_can.md](pra_bms_can.md)) 수립 완료, 구현 대기                                       |
| 전원 제어 FSM                                                           | **✅ 실보드 검증 통과 (Step9, 2026-10-10)** — `RD_CONTROL_LOOP`: STATE FSM + BOOT_FSM 0~9 + SW 뗄때델타 + FAULT 정책. 버튼 부팅·AUX 정상. ⚠ 5_ON Vin↔Vout 비교 임시중지(프리차저 HW 결함) (opmode §4-7) |
| FreeRTOS 태스크 본체                                                     | **rs485Task·systemTask·controlTask(RD_TASK_CONTROL) 실동작** / default(LED) / can1 빈 루프                                                                   |

---

## 3. 우선 작업 경로 (권장)

1. ~~**Step 2 — 레지스터맵 코드화**~~ **[완료]** → `rd_register_pra.h`.
2. ~~**Step 3 — 사용자 교차 검증**~~ **[완료]** → §3-1.
3. ~~**Step 4·5 — Orin 통신 + modbus**~~ **[완료 2026-09-17]** → §3-2.
4. ~~**Step 6 — Checker/systemTask (USART6)**~~ **[완료 2026-09-17]** → §3-3.
5. ~~**Step 7 페리페럴 구현**~~ **[완료 2026-09-21]** → controlTask(`RD_TASK_CONTROL`)에 전원GPIO+AUX·ADC·MCP23017 EXIO 통합(별도 Start_i2c 불필요, EXIO 를 페리페럴 루프에 폴딩). §3-4.
6. ~~**실보드 수동 IO 검증**~~ **[완료 2026-10-04]** → ADC(NTC/Vin/Vout/전류) 정상, MCP23017 EXIO I2C 통신 정상(SW 10k 풀업 비눌림=1). 출력단(릴레이/AUX/LED/BUZ) 구동은 Step9 전 확인 권장. §3-4.
7. ~~**Step 9 스위치 기반 전원 제어**~~ **[로직 완결 2026-10-04]** → `RD_CONTROL_LOOP`(**SW1→BOOT_FSM 메인버스 방향천이, SW2→AUX2(Orin) 토글**, AUX1 내부임베디드 부팅 자동, UVLO/OVLO 기반). B~F 수정 + A(STATE↔BOOT_FSM 경합, BOOT_FSM 미러 구조로 전환) + 발진(SW1 stale-delta) + FAULT 교차전파(온도·전압=전 시스템 / boot=버스만) 전부 해소. opmode §4-7.
8. ~~**Step 9 실보드 검증**~~ **[✅ 통과 2026-10-10]** → 하드웨어 결선(버튼·릴레이·Vin/Vout/GND·외장 NTC + AUX 가상부하=벅컨버터) 후 전원제어 실동작 검증. 버튼 부팅 시퀀스·AUX 포트 정상. **⚠ 프리차저 body diode 역류 HW 결함** 발견 → 임시 단방향 다이오드 조치 → Vout 이 PC ON 시에만 유효 → **BOOT_FSM 5_ON Vin↔Vout 비교 SW 임시 중지**(HW 리비전 후 원복). opmode §4-7.
9. **▶ 다음**: **HW 리비전**(프리차저 역류 차단) 후 5_ON·OFF 전압비교 원복 → **Step 8 BMS CAN**(프로토콜 입력 필요) → **Step 10 MARSHAL 본체**(프로젝트 완료).

### 3-1. Step 3 검증 결과 (2026-09-17 사용자 확정)

- ✅ **STATE_t 의존성**: rd_common.h 이식 필요 확정 → Step4 에서 반영 완료. (FSM 은 유저가 추후 수정)
- ✅ **degraded_cnt ↔ HW 비트 순서 통일**: `bit0=CAN1 / bit1=uart2 / bit2=I2C1` 로 변경 완료.
- ✅ **addr 53/60 명명 확정**: `bms_state`(53)=BMS 보고 상태, `state`(60)=CAN1 포트 채널(STATE_t). 상수 `BMS_STATE_*`.
- ℹ **CSV 동기화 불필요**: `register_map_pra.CSV` 는 사용자가 스프레드시트에서 export 한 작업용 사본 → 원본(스프레드시트)에서 유지, 헤더/wiki 와 별도 동기화 안 함.

### 3-2. Step 4·5 결과 (2026-09-17)

Orin RS485 modbus 계층 이식 완료. **DPC_B `rd_comm_orin`/`rd_map_dpcb` 기준**, PRA 단순형 `rd_uart` API 에 맞춰 적응.

- **생성**: `rd_comm_orin.{h,c}`(노드 `0xA1`=PCU, PING/READ(멀티세그)/WRITE/REBOOT, RW 미지원), `rd_map_pra.{h,c}`(영역 LUT 10개 + DISPATCH R/W + `RD_MAP_INIT` + MARSHAL void 스텁), `rd_system.{h,c}`(전역 객체 `PRA_uart6`/`PRA_rs485`/`ORIN_PACKET`/`reg` + `RD_SYSTEM_INIT` + `RD_TASK_RS485`)
- **교체**: `rd_common.h` → DPC_B 상위호환본(STATE_t/LS_*/HC_*/degraded, RD_RET 유지)
- **rd_uart 수정 (통신 이식 필수분)**: ① RX/TX 버퍼 128→**256/272**(공유 버스 전범위 READ 응답), ② IDLE wake `USART1`→**`USART6`**(PRA Orin 채널 정정), ③ `RD_RS485_IRQ_HANDLER` 신설(TC 시 DIR→RX + **RE/IDLE 재활성** + gState 복원 — 미수행 시 첫 TX 후 RX 영구정지, ECU 이식)
- **main.c (USER CODE 영역만)**: Includes 에 `rd_system.h`, BEGIN 2 에 `RD_SYSTEM_INIT()`, `Start_rs485` 본체 = `RD_TASK_RS485()` (ECU 구조)
- **stm32f4xx_it.c (USER CODE 영역만)**: USART6 IRQ 에 TC→`RD_RS485_IRQ_HANDLER`, IDLE→`RD_UART_IDLE_HANDLER` 배선
- **write 권한 lock**: 현재 `RD_ORIN_HANDLE(...,0)` 자유 접근 — 모드 기반 권한(opmode §4/§6) 은 TODO
- **컴파일 검증**: STM32CubeIDE 에서 사용자 수행 (이 환경 ARM 툴체인 없음)

> ※ Step4 는 당시 PRA **단순형 rd_uart** 위에서 구성. Step6 에서 rd_uart 를 advanced 로 교체하며 위 rd_uart 수정(①~③)은 advanced 판에 흡수됨(버퍼 256/272 동일, wake_task 기반 IDLE, RD_RS485_IRQ_HANDLER 포함). rd_comm_orin 도 comm_err_flag/`RD_RS485_TRANSMIT` 로 복원.

### 3-3. Step 6 결과 (2026-09-17) — USART6 Checker

**사용자 결정: ECU식 고급 rd_uart 이식** 후 rd_system.c Checker(USART6/Orin 채널만).

- **rd_uart advanced 교체**: `rd_uart.{h,c}` 를 ECU/DPC_B advanced 판으로 전면 교체 — `ERROR_STATUS_t error`(STATE_t lifecycle+health / degraded_cnt / rx_error_cnt) / `RD_UART_INIT(obj,huart)` 2-arg / `RD_UART_CHECKER(obj,k)` / `RD_UART_RECOVERY` / `RD_RS485_INIT/RECOVERY/TRANSMIT/CHECKER/IRQ_HANDLER` / `wake_task` / CR1 RMW(`uart_crit`) / comm_err_flag. 단순형(`is_fatal`/self re-init) 폐기
- **rd_system.c Checker 추가**: `RD_TASK_SYSTEM`(10ms) → `RD_SYSTEM_HW_RESET_HANDLE`(addr5 Orin 요청 채널 리셋) → `RD_SYSTEM_CHECKER`(USART6: `RD_RS485_CHECKER` RET_NOK→`fatal_rs485_cnt`+RECOVERY, `≥FATAL_MAX`→`hw.reset.uart2`+`FAULT`) → `RD_SYSTEM_EVALUATE_STATE`(STATE_t→hw.error/fatal) → `ACTION_STATE_FAULT`(RS485_TEST_ON 해제 시 reset) → `RD_MAP_MARSHAL_PUBLISH`
- **globals 추가**: `hw`(HW_ERROR_FLAG_t, bit0=can1/bit1=uart2/bit2=i2c) / `fatal_rs485_cnt` / `tim_cnt` / `HAL_UART_ErrorCallback`(USART6 isr_err_code 캡처)
- **MARSHAL_PUBLISH SYSTEM 영역 구현**: `reg.sys` hw_error/fatal/reset ← hw.raw, sys_state ← payload_state, degraded_cnt[1] ← deg_pct(uart2), realtime_tick ← tim_cnt. SENSOR_ADC/BMS 는 **Step10**
- **main.c(USER CODE)**: `StartSystem` = `RD_TASK_SYSTEM()`. it.c IDLE 핸들러명 `RD_UART_IDLE_HANDLER` 로 갱신
- **RS485_TEST_ON** 정의(FAULT 자동 재부팅 차단, 브링업) — 실운용 시 제거
- **⚠ 제약 준수**: CAN1(BMS)·I2C1(패널) Checker/EVALUATE 훅은 **주석 처리** — 해당 페리페럴 추가 후 활성
- **미결**: `realtime_tick` 스케일(ms vs register ×0.1ms, TIM5 부재) / `ACTION_STATE_FAULT` 는 #5 결정으로 **제거됨**(통신 fatal↔전원 미연계, 전원 FAULT 정리는 `RD_CONTROL_LOOP` FAULT 분기 단독)
- **컴파일 검증**: 사용자(STM32CubeIDE)

### 3-4. Step 7 결과 (2026-09-21) — 페리페럴 계층

`rd_peripheral_pra` 본체 완성 + `PRA_PERIPHERAL` 전역 + **controlTask(`RD_TASK_CONTROL`) 배선**.
- **구성**: `RD_PERIPHERAL_INIT`(IO 핀 주입 Relay/SSP/AUX + `RD_EXIO_INIT` + `RD_ADC_INIT`) → `for(;;){ READ(RD_UPDATE_ADC) ; RD_EXIO_UPDATE ; WRITE(Relay/SSP/AUX) ; osDelay(10) }`
- **ADC**: 전역 `adc_raw_buf`(uint16[6]) DMA(HALFWORD, `(uint32*)` 캐스트) → ConvCplt → struct `adc_value[]` → 변환(NTC/전압/전류 float)
- **EXIO**: MCP23017 SW1/SW2 read + LED1/LED2/BUZ write, RST-먼저 INIT. **별도 Start_i2c 태스크 불필요**(페리페럴 루프에 폴딩)
- **정리(2026-09-21)**: 헤더 선언(RD_ADC_INIT/RD_UPDATE_ADC/Convert_*) 추가, copy 흔적(주석 `rd_peripheral_dpcb`/`//test`/stale 주석) 제거. `DYN_NUM_MOTORS`/`PERIPHERAL_MOT_t` 는 유저가 기제거
- **잔여**: NTC index/Vin·Vout 매핑·float→int8 스케일(**Step10 MARSHAL**), `Convert_V/I` >=4095 오검출(경미)
- **✅ 실보드 검증(2026-10-04)**: ADC(NTC×3/Vin/Vout/전류) 정상, MCP23017 EXIO I2C 통신 정상(SW1/2=1, 10k 풀업 비눌림). 출력단(릴레이/AUX/LED/BUZ) 구동은 Step9 전 확인 권장 → **게이트 통과, Step8 진행 가능**

---

## 4. 핵심 미확정 / TODO

### 4-1. Orin 정합
- **PRA = Orin PCU(`0xA1`) 확정** — STM 문서=PRA / Orin·외부=PCU ([pra_overview.md](pra_overview.md) §6). `ORIN_MY_ID`=`0xA1`
- **정합 방향 확정 (2026-09-17)**: **PRA 레지스터맵을 기준**으로 Orin `rd_register_pcu.hpp` 를 수정. **PRA 개발 완료 이후 진행** (현 Orin PCU 맵은 placeholder 유지)
- Orin `enable_pcu_read` 활성화 시점 (PCU 맵 정합 후)

### 4-2. 레지스터맵
- **[완료 2026-09-17]** CSV 정합성 정리: DIAG 헤더 `[80:255]` 정정, degraded_cnt Group=SYSTEM, addr 100~255 RESERVED(156), NTC=int8_t, SENSOR/STATE=자체 ADC(CH2~CH7). 초기 지적사항 전부 반영됨
- `rd_register_pra.h` 코드화 (Step 2)

### 4-3. 하드웨어/역할
- **[확정 2026-09-17] USART1(EX RS485) = 미사용** — 통신형 BMS(RS485/CAN 2종) 확장 대비로 MX on, 현 시스템은 CAN1 만 사용. 태스크·객체·Checker 대상 아님
- **[확정 2026-09-21] I2C1 MCP23017 매핑**: SPST SW1(GPA0)/SW2(GPA2) + LED1(GPA1)/LED2(GPA3)/BUZ(GPA5, 액티브 부저) + SWA3(GPA4 DNP). GPB 미사용. ([pra_peri.md](pra_peri.md) §5)
- 전원 시퀀스 임계값·타임아웃·릴레이 순서 (실기, [pra_opmode.md](pra_opmode.md) §5)

### 4-4. Step 7 페리페럴 구현 항목 (`rd_peripheral_pra`)
[pra_peri.md](pra_peri.md) §6-1 상세.

**✅ 완료(유저, 2026-09-21)**: AUX1/2 추가·**WRITE 반영**, EXIO LED/BUZ 버그·**루프 UPDATE 배선**, ADC 변환·NTC[2]·Current 정정, 구조체 SENSOR_*/NTC **float 전환**, `PRA_PERIPHERAL` 정의 + **controlTask(`RD_TASK_CONTROL`) 배선**(INIT + READ/EXIO_UPDATE/WRITE + **osDelay(10ms)**), **ADC 버퍼 일원화**(전역 `adc_raw_buf`→ConvCplt→struct `adc_value[]`).

**✅ 추가 완료(2026-09-21)**: ADC 버퍼폭 **수정안 A 적용** — `adc_raw_buf`/`adc_value` `uint16[6]` + `(uint32*)` 캐스트 + `Convert_*` 인자 uint16 → DMA HALFWORD 정합, ADC 정상 적재.

**잔여 (Step10 정합 / 낮음):**
- **매핑 확인**(Step10 정합): NTC index↔buf 역순(`NTC[0]=buf[2]`), Vin·Vout in·out 반대
- **스케일**(Step10): float(V/A/°C) → 레지스터 int8 ×0.1V·×1°C MARSHAL 변환. + `Convert_V/I` 의 `>=4095 return -999`(NTC 관용구 복사) 전압/전류 오검출 소지 (경미)
- **I2C1 Checker** 확장(편입 후, [pra_checker.md](pra_checker.md) §3)
- 정리(낮음): `RD_ADC_INIT`/`RD_UPDATE_ADC`/`Convert_*` 헤더 선언, copy 흔적(주석/`//test`/`DYN_NUM_MOTORS`/`PERIPHERAL_MOT_t`/stale 주석)

> **Step7 페리페럴 = 실질 완료** (기능 버그 0). 남은 것은 Step10 MARSHAL 정합(매핑·스케일)과 저순위 정리뿐.

### 4-5. 이식 시 주의 (기존 보드 교훈)
- **EXIO 초기화 순서**: MCP23017 RST 토글을 IODIR 설정보다 먼저 (DPC_B history §22)
- **버퍼 크기**: 공유 RS485 버스 → ECU 정렬(RX/TX 256/272), 초과 시 FRAMING 폭주 (DPC_B history §20)
- **노드 ID 정합**: Orin `PacketID` 와 반드시 일치 (불일치 시 조용한 무응답)
- **✅ MX 재생성 안전성 감사 (2026-09-21)**: MX 생성 파일(main.c/stm32f4xx_it.c/freertos.c/hal_msp.c)의 손수 코드가 전부 `USER CODE BEGIN/END` 내부에 있고 태그 균형 정상 → **값 변경 재생성 시 유실 없음**. 단 ①MX 에서 **태스크/페리페럴 이름 변경·삭제 금지**(USER CODE 블록명이 바뀌면 내용 고아화), ②신규 태스크 본체 호출은 해당 `USER CODE BEGIN Start_xxx` 안에, ③HAL 콜백(`HAL_ADC_ConvCpltCallback`/`HAL_UART_ErrorCallback`)은 손수 파일(rd_*.c)에 유지
