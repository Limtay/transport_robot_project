# PRA 운용 상태(sys_state) 및 전원 제어 시퀀스

> 최종 갱신: 2026-10-04 (스위치 버튼 정책 §4 확정 — 코드 미착수)
> 상위: [pra_overview.md](pra_overview.md)

> **상태 = 설계 개념 + 버튼 정책 확정, 코드 미착수**. 전원 제어 FSM 미구현(`controlTask`=`RD_TASK_CONTROL` 은 현재 페리페럴 I/O 루프). 본 문서의 시퀀스·임계값은 목표 동작이며 실기 확정 대상(TODO). **§4 스위치 버튼 정책은 2026-10-04 사용자 확정(Step9 구현 대상).**

---

## 1. sys_state (addr 23) — SYSTEM_STATE_e (2026-10-04 재정의)

전원 제어 FSM 의 운용 상태. 구 ECU 레거시(MANUAL/AUTO/ESTOP) **폐기** → 전원 운용 상태로 재정의(`rd_system.h`).

| 값 | 이름 | 내용 |
|----|------|------|
| 0 | INIT | 부팅·초기화 (UVLO/OVLO 안정화 대기, 모든 출력 off) |
| 1 | RUN_OFF | 정상 구동, 대전류 버스 **OFF** (AUX1 내부 임베디드 ON) |
| 2 | RUN_TRN | 정상 구동, 대전류 버스 **천이중** (BOOT_FSM 진행) |
| 3 | RUN_ON | 정상 구동, 대전류 버스 **ON** |
| 255 | FAULT | 전이 실패/온도·전압 락아웃/통신 치명 등 — 전 출력 강제 off |

> **MANUAL/AUTO 는 상태가 아닌 입력원**(로컬 SW §4 vs Orin `ctr_mode`)으로 재해석 — 둘 다 동일 FSM(`PRA_CONTROL.STATE`)을 구동. 상세 백본: §4-7.
> ✅ **addr23 발행원 = `PRA_CONTROL.STATE` 미러 (2026-10-04 확정·구현)**: `MARSHAL_PUBLISH`(`rd_map_pra.c:173`)가 운용 전원 FSM 을 addr23 에 발행. `payload_state` **폐기**. 통신/채널 health 는 `hw_error`/`hw_fatal`/`degraded_cnt` 로 별도 노출(전원과 독립, §4-7 #4/#5).

---

## 2. 메인파워 순차 투입 시퀀스 (전압 측정 기반) — 목표 설계

100A급 메인파워는 릴레이 직접 투입 시 인러시(inrush) 로 접점·소자 손상 → **프리차저로 출력 커패시터를 선충전한 뒤 릴레이 투입**하는 순차 제어.

**투입(ON) 순서 (목표)**:

```
1. 명령 수신 (SW2 ≥2s BOOT_FSM 방향천이 §4-3  또는 ctr_mode[0]=1)
2. PC_EN = 1        프리차저 ON → 출력 커패시터 선충전
3. 프리차징 완료 판정   LOOP_CNT ≥ 임계  AND  |SENSOR_Voltage_IN − SENSOR_Voltage_OUT| ≤ 0.1V (전위차 수렴)
   └ 타임아웃(LOOP_CNT 초과에도 미수렴) → FAULT (프리차징 실패)
4. RN_EN / RP_EN = 1  N·P 릴레이 순차 투입 (본선 연결)
5. PC_EN = 0        프리차저 OFF (역할 종료)
```

**차단(OFF) 순서 (목표)**:

```
1. 명령 수신 (ctr_mode[0]=0  또는 SW/ESTOP)
2. RP_EN / RN_EN = 0  릴레이 개방
3. PC_EN = 0          (안전상 확실히 OFF)
```

- **핵심 입력** = `PERIPHERAL_t.SENSOR_Voltage_IN`/`SENSOR_Voltage_OUT`(float V) + `SENSOR_Current_BUS`(float A) ([pra_peri.md](pra_peri.md) §6). 프리차징 완료(전위차 ≤0.1V) 판정이 시퀀스 진행의 게이트
- **`BOOT_FSM` = DPC_B 식 switch-case 상태머신 (유저 작성)**. 각 전이는 `LOOP_CNT`(루프 경과) + 전압/전류 조건 조합 (§4-6)
- **TODO (실기 확정)**: 프리차징 전위차 임계(예시 0.1V)·LOOP_CNT 타임아웃, 릴레이 P/N 투입 순서·간격, 인러시 전류(`SENSOR_Current_BUS`) 리밋, 릴레이 접점 융착 감지
- **P/N 릴레이 역할**: `RP_EN`=양극(+) 릴레이 / `RN_EN`=음극(−) 릴레이. 투입 순서는 회로 설계에 따름(확정 필요)

---

## 3. 서브파워 (AUX) — 2채널 (2026-10-04 역할 확정)

| 채널 | 핀 | cmd | 대상 | 구동 |
|------|----|----|------|------|
| AUX1 | PB14 `AUX1_EN` | `ctr_mode[1]`(addr62) | **내부 임베디드 부팅용** (MCU 계) | 부팅 시 UVLO/OVLO(`SENSOR_Voltage_IN`) OK → **자동 ON** / SW2 ≥10s 강제복구(§4) |
| AUX2 | PB15 `AUX2_EN` | `ctr_mode[2]`(addr63) | **Orin 전원** (ORIN 계) | SW1 ≥2s 토글(§4) / Orin cmd |

- 솔리드스테이트 릴레이 직접 on/off (순차 제어 불필요). 페리페럴 반영 완료(`EN_AUX1/2` + `GPIO_AUX1/2`, `RD_PERIPHERAL_WRITE`)
- ✅ **역할 확정 (2026-10-04)**: HW 동일 규격이라 유저 정의 우선 → **AUX1=내부 임베디드, AUX2=Orin**. 레지스터 §2-4(`AUX1=MCU 계`/`AUX2=ORIN 계`)와 **정합**(MCU 계=내부 임베디드). 이전 상충 해소
- Orin 전원(AUX2) 차단 시 **통신 두절 순서 주의** (안전 정책 TODO)

---

## 4. 스위치 버튼 정책 (SW1/SW2 + BOOT_FSM) — 2026-10-04 (코드 기준, F 확정)

입력원: MCP23017 EXIO — **SW1=`SWA1`(GPA0), SW2=`SWA2`(GPA2)**, 10k 풀업(비눌림=1 / 눌림=0, 실보드 검증 [pra_peri.md](pra_peri.md) §6). `controlTask`(`RD_TASK_CONTROL`)가 `RD_EXIO_UPDATE` 로 SW 상태를 읽어 처리. **누름시간 판정 = 뗄 때 델타**(SW 해제(state=1) 시점 `HAL_GetTick() − *_SW_LAST`, DPC_B 패턴). 임계 **단일 `SW_HIGH=3000`(3s, DPC_B 동일)**.

> **⚠ 역할 (2026-10-04 F 확정 — 코드 기준)**: 구 문서의 SW1/SW2 가 **반대였음**. 실배선/코드 기준으로 **SW1=메인버스, SW2=Orin** 으로 확정.

### 4-1. 부팅 시 (자동)
- PRA 전원 인가 → **UVLO/OVLO 체크 정상이면 `AUX1`(내부 임베디드 부팅용) 자동 ON**. SW 무관.
- **전압 체크 기준 = `PERIPHERAL_t.SENSOR_Voltage_IN`** (입력 버스 전압). UVLO/OVLO 임계는 실기 확정.

### 4-2. SW1 — 메인버스 (high-current bus `BOOT_FSM`)
- **≥3초(SW_HIGH) 길게 누름 → `BOOT_FSM`(프리차저+P/N 릴레이 순차, §2) 방향 천이**(off↔on). `MAIN_SW_LAST` 사용. 코드상 RUN_OFF/RUN_ON 에서 길게→ `STATE=RUN_TRN` → BOOT_FSM 킥.

### 4-3. SW2 — AUX2(Orin 전원) 토글
- **≥3초 길게 누름 → `AUX2`(Orin) on↔off 토글**. `AUX_SW_LAST` 사용. RUN 상태에서만. (짧은 누름 무시)

### 4-4. 누름시간 판정 (뗄 때 델타, 단일 임계)
| 구분 | 임계 | 동작 |
|------|------|------|
| 짧게 | `SW_LOW`(100ms) ~ `SW_HIGH`(3000ms) | 무시 |
| 길게 | ≥ `SW_HIGH`(3000ms) | SW1→메인버스 BOOT_FSM 방향천이 / SW2→AUX2(Orin) 토글 |

> DPC_B 와 동일하게 3s 단일 임계. (구 LONG 2s/VERY_LONG 10s 폐기)

### 4-5. 역할 매핑 요약
| 전원/대상 | 핀 | 역할 | 트리거 |
|------|----|-----------|--------|
| Main bus | RP/RN/PC (`BOOT_FSM`) | 100A 고전류 버스 순차(§2) | **SW1** ≥3s 방향 천이 |
| AUX2 | PB15 `AUX2_EN` | Orin 전원 | **SW2** ≥3s 토글 |
| AUX1 | PB14 `AUX1_EN` | 내부 임베디드 부팅용 | 부팅 UVLO/OVLO(`SENSOR_Voltage_IN`) OK→자동 ON |

> **AUX1 강제복구(2s off·재부팅)는 현재 미구현·보류** — reboot 블록 주석(STATE 별도값 필요). 추후 결정.

### 4-6. 구현 방식 (Step9 — 유저 구현)
- **`BOOT_FSM` = DPC_B 식 switch-case 상태머신** (구현됨, §4-7). 전이: `LOOP_CNT ≥ RELAY_DELAY_TICK` + 프리차징 `|Vin−Vout| < PC_VOLTAGE_DIFF(0.2V)`, 각 state `BOOT_SEQ_TIMEOUT` 초과 시 FAULT
- UVLO/OVLO 부팅 체크(`SENSOR_Voltage_IN`), SW 뗄때델타, AUX2 토글, FAULT 정책(온도·전압=전 시스템 / boot=버스만) — **구현 완결**(실보드 검증 대기). AUX1 강제복구 — 보류
- Orin(AUTO) 경로(`ctr_mode`)와 로컬 SW 경로의 우선순위/충돌 정책 — TODO (§5)

### 4-7. RD_CONTROL_LOOP 구현 현황 (2026-10-10, ✅ 로직 완결 + 실보드 1차 검증 통과)

`rd_system.c` 에 `CONTROL_PRA_t PRA_CONTROL` + `RD_CONTROL_INIT` + `RD_CONTROL_LOOP(CTL, PERI)`. `RD_TASK_CONTROL`(controlTask)에서 PERI READ/EXIO_UPDATE/WRITE 후 호출. **STATE FSM + BOOT_FSM 순차 + SW 뗄때델타 + FAULT 정책 구현 완료.** ✅ **실보드 검증(2026-10-10)**: 버튼 기반 부팅 시퀀스·AUX 포트 구동 정상. 단 프리차저 HW 결함으로 5_ON 전압비교 임시 중지(아래 ⚠).

**⚠ 하드웨어 제약 — 프리차저 body diode 역류 (2026-10-10 발견 / 2026-10-13 ideal diode 임시조치)**
> **갱신(2026-10-13)**: 임시 **ideal diode** 연결로 **역류 문제 해소**(과열 해결). 단 **PC OFF 시 Vout 계측 불가**는 동일 → 5_ON Vin↔Vout 비교 주석처리 유지, 실보드 테스트 완료. HW 리비전 시 원복.

- **결함**: 프리차저 OFF + P릴레이 ON 시, 프리차저 body diode 를 통해 역류 발생 → 칩 과열
- **임시 조치(HW)**: PC 출력과 P릴레이 출력단 연결 사이에 **단방향 다이오드** 설치. 이로 인해 **`SENSOR_Voltage_OUT`(Vout) 계측은 프리차저 ON 상태에서만 유효**(PC OFF 시 다이오드가 전압 장벽을 막아 Vout≈0 오독)
- **코드 임시 중지(SW)**: `case BOOT_FSM_5_ON` 의 Vin↔Vout 전압비교(릴레이 정상 폐로 확인) **DNP** — `rd_system.c:347~348` `delta_voltage` 계산 주석처리 + `delta_voltage = 0` 강제 → line 349 FAULT 조건 무효화(미중지 시 Vout≈0 ⇒ delta 과대 ⇒ 상시 FAULT). `EN_SSP=0`(line 340)도 동반 조정
- **부수 영향**: `case BOOT_FSM_OFF` 의 방전 확인(`Vout > RUN_VOLTAGE_TH → FAULT`, line 272)도 동일 이유로 **상시 inert**(PC OFF라 Vout≈0, 오검출은 없으나 "버스 미방전" 미탐지). 프리차지 구간(2_PC_ON/6_PC_ON, PC ON)의 Vout 비교는 정상
- **복구 예정**: HW 리비전(역류 차단 설계 반영) 후 5_ON·OFF 전압비교 원복

- **STATE FSM**(= sys_state addr23 미러): `INIT`(UVLO/OVLO 정상 + `BOOT_TICK`(1s) 경과 → RUN_OFF, 전이 시 `MAIN_SW_LAST` 리셋) / `RUN_OFF·RUN_TRN·RUN_ON`(`STATE∈[1,3]`, EN_AUX1=1 + 온도·전압 락아웃) / `FAULT(255)`(전 출력 off, **리부트 전용 래치**)
- **BOOT_FSM**(대전류 버스 순차): 0 OFF → 1 LSR_ON → 2 PC_ON(프리차지 `|Vin−Vout|<0.2V` 완료) → 3 HSR_ON → 4 PC_OFF → 5 ON / 종료 역순(6 PC_ON→7 HSR_OFF→8 PC_OFF→9 LSR_OFF→OFF), 255 FAULT(default: 릴레이·SSP off). 각 state `BOOT_SEQ_TIMEOUT` 초과 → FAULT
- **스위치(뗄때델타)**: **SW1=메인버스**(≥SW_HIGH 롱 → `BOOT_FSM` 직접 킥: OFF→1 / 5_ON→6, `STATE=RUN_TRN`, `LOOP_CNT=0`), **SW2=Orin**(롱 → `EN_AUX2` 토글). 킥 가드 = `BOOT_FSM==OFF||==5_ON` (전이 중 재킥 차단)
- **임계(`rd_system.h`)**: UVLO40/OVLO60[V], OTLO80/UTLO-20[°C], BOOT_TICK 1s, RELAY_DELAY_TICK 100, PC_VOLTAGE_DIFF 0.2V, RUN_VOLTAGE_TH 1.0V, BOOT_SEQ_TIMEOUT 200, SW_LOW 100/SW_HIGH 3000

**✅ 버그 수정 이력 (2026-10-04 유저)**
- B: `case 1_LSR_ON`→`2_PC_ON` / C: `case 6_PC_ON`→`7_HSR_OFF` (복붙 전이 정정)
- D: OFF fault `Vout > RUN_VOLTAGE_TH` / 5_ON fault `delta > PC_VOLTAGE_DIFF` (반전 정정)
- E: 킥 시 `LOOP_CNT=0` 추가(정착/아크 dwell 보장) / `BOOT_FSM_e` 라벨 6~9 정합
- F: **SW1=메인버스 / SW2=Orin** 확정 (코드 유지, 문서 정정)
- (앞서) addr23←PRA_CONTROL.STATE 미러·payload_state 폐기·통신↔전원 미연계

**✅ A — 해결 (STATE↔BOOT_FSM 경합 → 미러 구조로 전환)**
- **원 증상**: 휴지 `case OFF`/`5_ON` 의 `STATE=` 강제 + SW 간접 킥(`else if STATE==RUN_TRN`) 구조 → SW가 세운 RUN_TRN 이 같은 루프에 덮여 킥 분기 도달 불가 → 버스 on/off 트리거 불가
- **채택 해법**: SW 핸들러가 **BOOT_FSM 직접 킥**(STATE-FSM 구간, switch 이전 실행). STATE 는 BOOT_FSM **미러**(OFF→RUN_OFF / 5_ON→RUN_ON / 전이중 RUN_TRN). 킥을 `STATE` 조건 대신 `BOOT_FSM==OFF||5_ON` 로 가드 → STATE 경합 제거
- **발진(파생) 해결**: SW1 핸들러를 RUN 블록(`STATE∈[1,3]`, RUN_TRN 포함) 직하로 올려 **`MAIN_SW_LAST` 를 전이 중에도 매 루프 갱신** → stale-delta 자가 재킥(버스 on↔off 발진) 제거. 이중방어 = tick 상시 갱신 + BOOT_FSM 킥 가드

**✅ FAULT 정책 확정 (2026-10-04 유저)**
- **온도/전압 폴트 = 전 시스템 중지**: 락아웃 시 `BOOT_FSM=FAULT` **및** `STATE=FAULT` 동시 세팅(switch default 가 릴레이 즉시 차단 + FAULT 분기가 AUX1/AUX2 off)
- **BOOT 폴트 = boot 만 중지**: 시퀀스 timeout 시 `BOOT_FSM=FAULT` 만, **`STATE` 미전파**(RUN_TRN 잔류 → EN_AUX1 유지·SW2 Orin 토글 유지). 역전파(BOOT→STATE) 코드 **없음**(의도)
- **`STATE==FAULT` = 리부트 전용 래치**(자동 복구 전이 없음): 자동부팅 임베디드 보호 목적. 코드적 자동 재부팅 금지, 향후 **버튼 특수로직으로 PRA 리붓** 예정

**남은 관찰(조치 불요)**: ① BOOT 폴트 가시성 — `sys_state` 가 RUN_TRN 유지되어 Orin 이 "전이중"과 "boot 실패 교착" 구분 불가(필요 시 `BOOT_FSM` 을 DIAG 노출) ② 온도/전압 폴트 시 EN_AUX1 1-루프(10ms) 지연 off(대전류 버스는 당 루프 즉시 차단, 무해) ③ **[보류]** AUX1 강제복구, Orin ctr_mode↔SW 우선순위(§5)

---

## 5. 제어 입력원 (MANUAL / AUTO) — **상태 아님, 입력원**

> sys_state(§1)가 INIT/RUN_*/FAULT 로 재정의되며 MANUAL/AUTO 는 **상태값이 아니라 동일 FSM(`PRA_CONTROL.STATE`)을 구동하는 입력 경로**로 재해석됨.

- **MANUAL (로컬)**: MCP23017 패널 SW1/SW2 (§4) 로 AUX2(Orin)/메인버스(BOOT_FSM) 제어. LED/BUZ 로 상태 인디케이팅
- **AUTO (Orin)**: Orin 이 `ctr_mode[0~2]`(addr 61~63) 기입 → controlTask 소비 → 동일 FSM 구동
- 두 입력원의 우선순위/전환·충돌 정책 = **확정 필요 (TODO)**. DPC_B 의 소유권 공유(패널+Orin) 패턴 참고

---

## 6. 미확정 / TODO

- ✅ ~~AUX1/AUX2 역할 상충~~ **확정(2026-10-04)**: AUX1=내부 임베디드, AUX2=Orin (레지스터 §2-4 와 정합)
- **UVLO/OVLO 임계값** 확정 (소스=`SENSOR_Voltage_IN` 확정, 임계 전압은 실기) — AUX1 자동 부팅 게이트
- `BOOT_FSM` 상태 정의·전이표 (유저 switch-case 작성): 프리차징 완료 전위차 임계(예시 0.1V)·LOOP_CNT 타임아웃·인러시 리밋·P/N 투입 순서
- SW 누름시간 임계(LONG 2s/VERY_LONG 10s) 실기 조정 (판정=뗄 때 델타 확정)
- MANUAL(로컬 SW)↔AUTO(Orin `ctr_mode`) 전환 조건·우선순위·충돌
- ESTOP_SW / ESTOP_HW 진입 트리거·복귀 정책 (Orin 명령 vs 로컬)
- FAULT 진입 조건(통신/프리차징 실패/릴레이 융착) 및 재부팅 정책 (ECU `RS485_TEST_ON` 패턴)
- 릴레이 접점 융착/이상 감지 (`SENSOR_Current_BUS`/`SENSOR_Voltage_OUT` 교차 검증)
