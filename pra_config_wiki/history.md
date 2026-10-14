# PRA 작업 히스토리

> 최종 갱신: 2026-10-13 (Session 34 — BMS delta_tick ×0.1s + SENSOR/ADC int16(×0.1) 레이아웃 변경·MARSHAL 하네스 완성)
> 상위: [pra_overview.md](pra_overview.md)
> 진행 상태 요약: [plan.md](plan.md)

---

## [Session 34] BMS delta_tick ×0.1s + sensor_adc 하네스 확인 (2026-10-13)

- ✅ **BMS delta_tick ×0.1s 스케일 변경(유저 지시, CSV 변경 반영)**: `RD_CAN_BMS_CHECKER` delta_tick = `(now−last_rx_tick)[ms]/100` = ×0.1s(0xFF=25.5s↑). **BMS 예외 스케일**(타 센서 ×0.1ms 와 다름). 1Hz 폴링서 정상치 ~10(1.0s)로 신선도 유효. 반영: rd_can_bms.c, rd_register_pra.h addr60 주석, pra_bms_can.md §8-6, pra_register.md, BMS_protocol_can.md §6
- 🔍 **sensor_adc 하네스 확인(유저 "확인하고") — 선결 2건으로 코드 미작성**:
  - ① **레지스터 아직 `int8_t`**(float 미변경) — PRA_PERIPHERAL 값은 float → 지금 하네싱하면 절단(53.8V→53). 유저 float 변경 선행 필요
  - ② **NTC 인덱스 역순**: `RD_UPDATE_ADC` 에서 PERI.NTC[0]=CH4·[1]=CH3·[2]=CH2 인데 register ntc[0]=CH2·[1]=CH3·[2]=CH4 → 채널 정합하려면 **reg.ntc[i] ← PERI.NTC[2−i]**
  - ③ 🔴 **전압 IN/OUT 채널↔이름 교차**(구 "확인·통일" TODO): PERI.SENSOR_Voltage_IN=adc_value[5]=**CH7**, OUT=adc_value[4]=**CH6**. 그러나 register voltage_input=**CH6**/voltage_output=**CH7** → 이름 매핑 시 채널 반대. 전원 FSM 은 PERI.Voltage_OUT(CH6)로 프리차지 판정(실보드 통과). **물리 IN/OUT 어느 채널인지 유저 확정 필요**
  - ⇒ 하네스 작성 전 **①float 변경 + ③전압 IN/OUT 물리 매핑 확정** 선행. current(CH5)·NTC(역순) 매핑은 명확
- ✅ **SENSOR/ADC 레이아웃 변경 + 하네스 완성(유저 지시 2026-10-13)**:
  - `current_input`/`voltage_input`/`voltage_output` **int8→int16, 단위 ×1→×0.1**(A/V). NTC 는 int8 ×1 유지(인덱스 그대로 — Orin 은 최대값만 사용)
  - SENSOR/ADC 7B→**10B(38~47)**, +3B → **SENSOR/BMS 48~64, CMD/POWER 65~67 로 밀림, RSVD1 15→12(68~79)**, DIAG 80 고정·총 256 유지. `#define`/`REGISTER_t.reserved1[12]`/상단레이아웃/static_assert 동기화
  - **전압 IN/OUT = 의미 기준 매핑**(채널 순서 아님, 유저 확정 — 현 수치 정상). `RD_MAP_MARSHAL_PUBLISH` 에서 **float→int16 ×10**(0.1 단위): `voltage_input=(int16)(SENSOR_Voltage_IN*10)` 등. NTC 는 `(int8)NTC[i]`. (유저가 의미매핑 작성, 클로드가 스케일/타입 반영)
  - 반영: rd_register_pra.h(ADC struct·#define·REGISTER_t·레이아웃), rd_map_pra.c(MARSHAL 스케일·LUT·cmd 주석), pra_register.md(영역표·SENSOR/ADC·SENSOR/BMS 48~64·§2-4 65~67), BMS_protocol_can.md §6(48~64)
- 반영: 위 코드·문서, history/memory

---

## [Session 33] BMS CAN 8-4 검증 + 8-5/8-6 구현 (CAN 코드 close) (2026-10-10)

- ✅ **8-4 실기 검증 통과**: `reg.sensor_bms` = voltage 538(53.8V)/soc 680(68%)/max_cell_temp 64(−40=23°C) 정상
- ✅ **8-5 Checker**: `RD_CAN_BMS_CHECKER` — 신선도(`last_rx_tick`)+HAL에러(`BMS_isr_err_code`)→**STATE_t(lifecycle+health)**→`reg.sensor_bms.state`(addr61). health: BOF/EPV/EWG/TIMEOUT/OK, lifecycle: READY(첫응답전)/RUNNING(신선)/DEGRADED(끊김). 디버그 전역 `BMS_ch_state`. 🔒 **로깅 전용·시스템 무전파**, ECU degraded decay·OFFLINE fatal escalation 미이식(1Hz 과함). 복구는 8-3 단독
- ✅ **8-6 delta_tick**: CHECKER 내 `reg.sensor_bms.delta_tick`(addr60)=now−last_rx_tick(ms, 0xFF 포화). ⚠ ×0.1ms 필드라 1Hz BMS 는 대부분 0xFF 포화→신선도는 addr61 state.lifecycle 권장. **hw_reset(addr5) 미구현**(유저: 자동복구라 불요), 0x98↔전원 미연계
- RD_TASK_CAN 루프 = REQUEST_ALL→osDelay→UPDATE→CHECKER→recovery. **CAN 파서 코드 8-1~8-6 대부분 close**
- 🗂 **Step 재구성(유저)**: 8-7=Orin 연동 실테스트(Orin 준비 후), **8-8=HSE 리비전 보드+실 BMS CAN 송수신+과열 등 전반 하드웨어 체크 → 프로젝트 종료**
- 🔴 **기록(유저 지시, Step8-7)**: `reg.sensor_adc`(SENSOR/ADC addr38~44) **MARSHAL 미연결** + 레지스터 필드 임시 `int8_t`인데 실제 PRA_PERIPHERAL 값 전부 **float** → 유저가 ①`DATA_SENSOR_ADC_t` 타입 변경(int8→float/int16+scale) ②PRA_PERIPHERAL→reg.sensor_adc 하네스 연결 예정. Orin ADC READ 선결(BMS CAN 과 독립)
- 반영: rd_can_bms.{h,c}, pra_bms_can.md(§3 Step8-5~8-8 재작성·Step8-7 ADC TODO·§6 로그), plan Step8·MARSHAL, history/memory. **금일 작업 마침**

---

## [Session 32] BMS CAN Step 8-4 구현 — 디코드→레지스터 발행 (2026-10-10)

8-3 빌드 성공 후 8-4 착수. `rd_can_bms.{h,c}` 확장 + 프로토콜↔레지스터 정합 수정.

- **`RD_CAN_BMS_UPDATE`**: 핸들 raw(ISR 적재) snapshot(`taskENTER_CRITICAL`) → **big-endian 디코드** → `reg.sensor_bms` 발행(프로젝트 규약=`taskENTER_CRITICAL`, **ResmapMutex 아님** — PRA 는 critical section 사용). `RD_TASK_CAN` 루프서 REQUEST_ALL+osDelay 후 매 주기 호출. 인클루드 `rd_map_pra.h`(reg)+FreeRTOS.h/task.h 추가
- **변환(레지스터 저장)**: voltage=BE(B0,B1)u16 / **current=(int16)(BE(B4,B5)−30000)** 오프셋 제거(0=무부하, 유저 int16 설계) / soc=BE(B6,B7)u16 / **max_cell_temp=0x92 B0 raw(40 offset 보존, uint8)** / err_lsb=0x98 B0~3·err_hsb=B4~7(LE 패킹). valid=0 필드는 0
- 🔧 **정합 불일치 수정(유저 지시 검토)**: `max_cell_temp` 는 **uint8 → 음수 못 담음 → 오프셋 보존(raw 저장)** 이 정답. BMS_protocol_can.md §6/§7 의 "온도 raw−40"(오프셋 제거) 표기는 **오류** → "raw 보존"으로 정정. **current(int16, 오프셋 제거) vs temp(uint8, 오프셋 보존) 비대칭**은 필드 타입이 강제 — Orin rd_register_pcu.hpp 변환 시 필드별 상이 적용 주의. (register 헤더는 이미 "40 offset"으로 정확했음)
- delta_tick(60)/state(61) 는 8-5/8-6
- **▶ 검증(유저)**: Orin READ/디버거 `reg.sensor_bms` = PCAN 값 — 538(53.8V)/current 0/680(68%)/max_cell_temp 64(−40=24°C)/error 0
- 반영: rd_can_bms.{h,c}, BMS_protocol_can.md(§6·§7 온도 정정·§3 Step8-4·§6 로그), plan Step8, history/memory. **다음=Step 8-5(Checker·degraded·addr61 state) / 8-6(통합)**

---

## [Session 31] BMS CAN Step 8-3 구현 — 자기완결 복구 (2026-10-10)

SJW 워크어라운드로 통신 가능해진 뒤 8-3 착수. `rd_can_bms.{h,c}` 확장.

- **`RD_CAN_BMS_RECOVERY(hcan)`**: `HAL_CAN_Stop → DeInit(MspDeInit: NVIC/GPIO 해제) → Init(MspInit 재설정) → RD_CAN_BMS_INIT(필터/clear/Start/Notification 재무장)`. ECU_V3 `CAN_RECOVERY` 개념. 🔒 시스템/전원 무전파
- **`RD_TASK_CAN` 복구 트리거**: 루프 = `REQUEST_ALL → osDelay(1000) → 판정`. 판정 = `(HW_NowTick()−BMS.last_rx_tick) > BMS_RX_TIMEOUT_MS(1500)` ‖ `BMS_isr_err_code!=0`(EWG/EPV/BOF) → `RD_CAN_BMS_RECOVERY`. 타임아웃=유저 "1주기(1000) 무응답"+지터 마진
- **INIT `last_rx_tick = HW_NowTick()`**(구 0) → init/recovery 시점부터 타임아웃 측정(즉시 오탐 방지). 복구 성공 시 last_rx_tick·isr_err_code 리셋되어 창 재시작
- **BMS 미연결**: ACK 실패→타임아웃/EWG→복구→재시도 무한(격리, PRA 무영향). 연결 시 응답→복구 안 함
- **▶ 검증(유저)**: ①정상 BMS ~1s 폴링·복구無 ②케이블 분리 ~1.5s 후 CAN 복구 반복·전원/STATE 불변 ③재연결 자동 재개
- 반영: rd_can_bms.{h,c}, pra_bms_can.md(Step8-3 ✅·§6), plan Step8, history/memory. **다음=Step 8-4(디코드 big-endian→reg SENSOR/BMS, ResmapMutex)**

---

## [Session 30] BMS CAN PHY 원인 확정 — HSI 오실레이터 허용오차 (2026-10-10)

Session 29 의 "트랜시버 TX 사망" 결론이 PCAN 에러로그로 반전·해결.

- 🔁 **반전**: PCAN 에러로그(raw/test.txt) — 에러가 **PRA 1초 폴링 TX 와 동기 버스트**(버스트당 3요청)·PCAN 에러카운터 0x85 포화 → **PRA 송신이 버스 거쳐 PCAN 도달하되 비트레벨 깨짐** = 트랜시버 TX 정상(칩 교체 불요), 원인=타이밍/클럭
- 🔑 **클럭**: 설정상 250k 정확(SYSCLK 160M **HSI**÷8×160÷2→APB1 40M, 10×16tq, SP87.5%). 그러나 CAN 이 **내부 HSI(RC ±1%)** 구동 — 이 config 허용오차 예산 ~0.31%(SJW=1) 초과. 선례: ECU_V3(HSI)도 AK 정상/ODrive EOF→크리스털로 해결(=[[stm-can-needs-crystal]])
- ✅ **원인 확정·임시해결**: `.ioc` **BS1=11/BS2=4/SJW=4, SP 75%**(250k 유지)로 PRA, PCAN SJW=4 → **EOF 無·통신 정상**. 예산 0.31%→~0.98%(HSI ±1% 커버) = **오실레이터 허용오차 확정**
- **결정(유저)**: SJW 워크어라운드로 **일단 진행**, 근본해결 **HSE 크리스털은 보드 리비전**. 나머지 펌웨어 먼저 작성
- ⚠ **실 BMS 캐비엇**: SJW↑는 자기 수신 마진만↑. PCAN 은 SJW=4 맞춰 양방향 OK 였으나 실 BMS SJW 는 벤더 고정(미지) → 실 BMS 연결 시 재확인, 크리스털이 확실
- **펌웨어/트랜시버/MCU 전부 정상** — 8-1·8-2 코드 변경 불요. **▶ 다음 = 나머지 펌웨어(Step 8-3 복구 / 8-4 디코드·reg 등)**
- 반영: pra_bms_can.md(§5-2 확정·§6 로그), plan Step8, history, memory([[stm-can-needs-crystal]] 신설·[[pra-project-context]])

---

## [Session 29] BMS CAN 벤치 PHY 디버깅 — 트랜시버 TX 고장 결론 (2026-10-10)

Step 8-2 실기 검증 중 송수신 불안정. 증상 누적 → **PRA CAN 트랜시버 TX(dominant 구동) 경로 HW 고장**으로 수렴. 펌웨어 무관 확정.

- **증상**: ① 8-1 1프레임 수신 OK + Form error ② 8-2 송신 시 즉시 에러 ③ `REQUEST_ALL` 주석(펌웨어 송신 0회)·종단 60Ω 정상인데도 **PCAN bus-off** ④ ESR `0xFFF80057`=REC255/TEC248 포화·EWGF·EPVF·BOFF0(bus-heavy)·**LEC=5 Bit Dominant Error**
- **진단 논리**: PCAN bus-off = PCAN TX ACK 미수신 = PRA 가 ACK(dominant) 버스에 못 실음. LEC=5(dominant 송신→recessive 모니터) 일치. 펌웨어 송신 0회 → ACK/에러플래그는 HW 자동 → **트랜시버 TX 경로 HW 문제**. 종단 정상 배제. MCU 측 정상(250k@87.5% 계산확인, Mode NORMAL, RX 동작)
- **결론**: 수신 OK / dominant 송신 불가. 후보: 트랜시버 **모드핀(STB/S/EN) Standby·Silent**(1순위)/VCC/PA12→TXD 배선
- **미실시 결정 테스트**: `CAN_MODE_SILENT_LOOPBACK`(트랜시버 우회) 깨끗→HW 확정, PA12 스코프
- **영향**: CAN 격리(로깅 전용)→PRA 본기능·전원·리붓 무영향. 트랜시버 수정 병렬 가능. **펌웨어 8-1/8-2 변경 불요**
- 반영: pra_bms_can.md(§5-1 PHY 디버깅·§6 로그), history/memory. **▶ 트랜시버 HW 복구 후 TX 검증 재개 → Step 8-3~**

---

## [Session 28] BMS CAN Step 8-2 구현 — 핸들·요청·RX apply (2026-10-10)

8-1 빌드 성공 확인 후 8-2 착수. `rd_can_bms.{h,c}` 확장.

- **`BMS_Handle_t`(전역 `BMS`)**: `msg_vic`(0x90)/`msg_temp`(0x92)/`msg_fault`(0x98) 각 `volatile data[8]`+`valid`, + `volatile last_rx_tick`(ISR latch). INIT 에서 clear
- **`CAN_BMS_REQUEST_ALL(hcan)`**: 3요청(`0x18[90/92/98]0140`, DLC8, 8×0x00) **메일박스 직접 적재**(`HAL_CAN_AddTxMessage`×3). 전부 OK/일부 WAIT/전무 NOK. AutoRetransmission DISABLE → 무ACK 시 자동 해제(부재 BMS starvation 없음)
- **`CAN_BMS_RX_POP`**(FIFO0→frame: ext_id/data_id/dlc/data) + **`CAN_BMS_RX_APPLY`**(data_id∈{90,92,98} 매칭 캐시 적재+valid+last_rx_tick latch, 비대상 무시)
- **RX 콜백 pop→apply 전환**(디버그 `BMS_last_rx` 유지), **`RD_TASK_CAN` ~1s 주기 `REQUEST_ALL`** 추가(`BMS_POLL_PERIOD_MS`=1000). 복구 트리거는 8-3 TODO
- **▶ 검증(유저)**: 실 BMS ~1s 요청→`BMS.msg_*.valid=1`+raw=PCAN값 일치/`last_rx_tick` 갱신. 또는 PCAN 더미응답으로 apply 확인
- 반영: rd_can_bms.{h,c}, pra_bms_can.md(Step8-2 ✅·로그), history/memory. **다음=검증후 Step 8-3**(폴링 복구 트리거) 또는 8-4(디코드·reg)

---

## [Session 27] BMS CAN Step 8-1 구현 — HW 브링업 (2026-10-10)

`rd_can_bms.{h,c}` 신설(단일 파일). CAN1 수신 경로까지 자급 구성.

- **필터**: `0x18xx4001` mask(bank0, 32-bit, FIFO0). ID(29b) `0x18004001`/mask(29b) `0x1F00FFFF`(바이트관점 0xFF00FFFF 와 유효ID서 동일, `<<3` 오버플로 회피로 0x1F 사용), 레지스터 `id<<3|IDE`. 응답 3종만 통과
- **자급식 IRQ(격리·단일파일·MX안전)**: `CAN1_RX0_IRQHandler`/`CAN1_SCE_IRQHandler` 를 rd_can_bms.c 가 직접 정의(startup weak override) + `RD_CAN_BMS_INIT` 가 NVIC enable. PRA `.ioc` 는 CAN1 인터럽트 OFF·it.c 에 CAN 핸들러 없음 확인함 → **.ioc 인터럽트 켜면 중복 심볼 링크에러(유저 주의)**
- **RX 콜백**: FIFO drain + `BMS_rx_count++` + `BMS_last_rx`(ext_id/data_id/dlc/data[8]) 디버그 캡처. **Error 콜백**: `BMS_isr_err_code|=`+`BMS_err_count++`+에러 notification off(재무장은 8-5)
- `HW_NowTick`=HAL_GetTick(ms), `RD_TASK_CAN`(INIT 실패 시 CAN만 재시도→idle, 요청폴링 8-3). **main `Start_can1` 에 `RD_TASK_CAN();` 는 유저 수동 추가**
- **▶ 검증(유저)**: .ioc CAN1 IRQ OFF 확인→빌드→하네스 추가→**PCAN 더미패킷(18904001/18924001/18984001) 송신→`BMS_rx_count`/`BMS_last_rx` 확인**
- 반영: rd_can_bms.{h,c} 신설, pra_bms_can.md(Step8-1 ✅·로그), history/memory

### [Session 27 추가] Step 8-1 실기 검증 통과 + Form error 조치 (2026-10-10)

- ✅ **검증**: `18904001`(DLC8) 송신 → `BMS_rx_count=1`, ExtId·data 전부 정상 수신 = **수신 경로 통과**
- ⚠ `BMS_err_count=1`/`isr_err_code=0x10`(`HAL_CAN_ERROR_FOR`, Form error) 1건 = **LEC 단발 트랜션트**(버스 idle→active, 수신 데이터 정상이라 타이밍·버스 정상). err_count 1 고정 = Error 콜백 notification-off 폭주차단 정상 동작
- **조치**: `CAN_IT_LAST_ERROR_CODE` 구독 제거(Activate/Deactivate 양쪽) — form/stuff/CRC 단발은 복구 판정에 불요(타임아웃+EWG/EPV/BOF). 재빌드 시 err_count 0 예상
- **▶ 다음 = Step 8-2**(핸들 `BMS_Handle_t`·요청 빌드 `CAN_BMS_REQUEST_ALL`·RX pop/apply)

---

## [Session 26] BMS CAN 설계 확정 (§4 결정 + 격리 원칙) (2026-10-10)

[pra_bms_can.md](pra_bms_can.md) §4 결정 5건 유저 확정 + 최우선 격리 제약 명문화.

- 🔒 **격리 원칙(최우선)**: BMS CAN 파서는 PRA 동작·전원·리붓에 **절대 무관여**. 수신→reg 갱신만. BMS 미연결/외부서플라이 구동 미응답이 PRA 에 영향 0. 통신단절(타임아웃/HW에러)→**CAN 서브시스템만** 재초기화 재시도(ECU CAN_RECOVERY 개념). ECU 의 OFFLINE→fatal→`SYS_STATE_FAULT` escalation 사다리 **미이식**
- ✅ **1. TX = 메일박스 직접 송신**: bxCAN TX 메일박스 3개 = 요청 3개 동시 적재, 소실 없음. 큐/stale 제거
- ✅ **2. 폴링 = 3요청 동시적재(간격무시) ~1s 주기(저율 로깅)**. 응답 타임아웃 = `delta_tick>1000ms`(1주기 무응답) → CAN recovery
- ✅ **3. 단일 파일 `rd_can_bms`**: 내부에 CAN ISR(HAL weak 콜백 override) + `RD_TASK_CAN()`. main `Start_can1` 하네스는 **유저 수동 추가**. rd_system/systemTask 무수정(Checker/Recovery 도 RD_TASK_CAN 자체 호출)
- ✅ **4. 0x98 순수 로깅**: 전원 FSM 연계 금지(opmode §4-7 FAULT 와 완전 분리)
- ✅ **5. RX 필터 `0x18xx4001` mask**(ID `0x18004001`/mask `0xFF00FFFF`, FIFO0)
- 반영: pra_bms_can.md(§0 격리 callout·ECU차이표·§1·§2 단일파일·§3 Step 재작성·§4 확정·§5 리스크·로그), history/memory
- **▶ 구현 요청 대기** (Step 8-1부터)

---

## [Session 25] BMS CAN 구현 계획 수립 (2026-10-10)

ECU_V3 CAN 구조(can_ak/rd_can_motor/rd_system/it) 조사 → PRA(요청→응답 폴링)용 작업 계획 문서 신설. 프로토콜(BMS_protocol_can.md)과 역할 분리: 신 문서는 계획·Step·로그 전용.

- ✅ **신설 [pra_bms_can.md](pra_bms_can.md)**: §0 전제/현상태·ECU 차이표, §1 ECU 이식/변경, §2 파일 구성, §3 **Step 8-1~8-7**, §4 결정사항, §5 리스크, §6 로그
- **ECU 이식 자산**: 4함수 I/F(INIT/UPDATE/TRANSMIT/CHECKER)+RECOVERY, Checker/Recovery 분리, STATE_t+degraded_cnt, RX pop/apply split, ErrorCallback+isr_err_take+ActivateNotification 재무장, CAN_Init 필터/Start, HW_NowTick 포트
- **BMS 특화 변경**: unsolicited(100Hz)→**요청 폴링 엔진**, TX 큐 제거(저율 3종 직접 송신), 확장ID 필터 `0x18xx4001`, per-motor mask→per-데이터ID, ALL_READY 게이트→응답 신선도 보고
- **PRA 현상태 확인**: hcan1 250k(BS1 13/BS2 2, AutoRetransmission DISABLE), can1Task 빈 루프, 페리페럴에 CAN 핸들/에러구조 미존재, HW_NowTick 미정의
- **Step 8-1~8-7**: HW브링업(필터/Start/시계) → 저수준 프레임(요청빌드/RX pop·apply) → 폴링엔진(RD_TASK_CAN1) → 디코드·reg매핑(UPDATE/MARSHAL) → Checker/Recovery(systemTask) → 시스템통합(hw_reset/0x98 전원연계) → 실보드검증
- **결정 대기(§4)**: TX 큐 여부·폴링 타이밍·파일 계층(단일 vs 2계층)·0x98↔전원 FSM 연계·필터 범위
- 반영: pra_bms_can.md 신설, plan Step8·§2, pra_overview 인덱스, history/memory
- **▶ 유저 작업 지시 대기** (Step 단위 착수 예정)

---

## [Session 24] BMS CAN 프로토콜 정리 + BMS 레지스터 영역 재정의 (2026-10-10)

유저가 PCAN 실기 통신으로 BMS 프로토콜 검증, raw 문서(0x98 전체 8바이트 포함)·레지스터맵 재설계. 문서화 담당.

- ✅ **신설 [BMS_protocol_can.md](BMS_protocol_can.md)**: CAN 2.0B 250k·29-bit 확장 ID(`0x18[데이터ID][dst][src]`, host0x40/BMS0x01)·필수 3종(0x90/0x92/0x98)·전체 ID 카탈로그·PCAN 실측 디코드·레지스터 매핑·구현 메모
- ✅ **PCAN 실측 디코드 확정**: 0x90=`02 1A 00 00 75 30 02 A8`→53.8V(14S팩)/0.0A/68.0%, 0x92=`40 01...`→24°C, 0x98=all0→정상. **바이트오더 big-endian, 전류 offset 30000** 확정
- ✅ **0x98 전체 8바이트 정리**(유저 raw 갱신): B0 전압/B1 온도/B2 전류·SOC/B3 편차/B4 MOS/B5 내부모듈/B6 검출모듈/B7 고장코드 — 전부 유의미
- ✅ **레지스터 `DATA_SENSOR_BMS_t` 재정의 반영**(유저 코드): 필수 3종 전용. `voltage`(u16,45)·`current`(**int16 signed**,47)·`soc`(u16,49)·`max_cell_temp`(u8,51)·`error_lsb`(u32,52=0x98 B0~3)·`error_hsb`(u32,56=0x98 B4~7)·`delta_tick`(60)·`state`(61). **0x93 유래 필드(bms_state/life/soh)·min_cell_temp 폐기**. 영역 16B→**17B(45~61)**, **CMD/POWER 62~64 로 +1 밀림**
- ✅ **헤더 `#define`/레이아웃 동기화 + static_assert 활성화 완료**(2026-10-10, 유저 지시로 클로드 수정): `REG_SENSOR_BMS_SIZE 17`·`REG_CMD_POWER_OFFSET 62`·`REG_CMD_POWER_S/E 62/64`·`REG_RSVD1 65/size15`, 상단 레이아웃 주석, **`REGISTER_t.reserved1[16]→[15]`**(총 256 유지 핵심), static_assert 7개 주석해제(C11 필요). `rd_map_pra.c` LUT/cmd판정은 심볼참조라 자동반영+주석 동기 정정
- 반영: BMS_protocol_can.md(§3-3 0x98 8B·§4·§5·§6·§7), pra_register.md(§영역표·SENSOR/BMS·§2-4 CMD 62~64), pra_peri.md §3, pra_overview 인덱스, plan Step8, rd_register_pra.h(#define/REGISTER_t/assert), rd_map_pra.c(주석), history/memory
- **▶ 다음 = Step8 CAN 파서 구현**(유저). 잔여 코드 정리(무관): addr23 인라인 주석 구값, `BMS_STATE_*` orphan 매크로

---

## [Session 23] Step9 전원제어 실보드 검증 통과 + 프리차저 HW 결함 대응 (2026-10-10)

하드웨어 결선(버튼·릴레이·Vin/Vout/GND·외장 NTC + AUX 가상부하=벅컨버터) 완료 후 실보드 검증.

- ✅ **검증 통과**: 버튼 기반 부팅 시퀀스(BOOT_FSM on/off 천이)·AUX1/AUX2 포트 구동 정상 확인
- ⚠ **프리차저 HW 설계 결함 발견**: 프리차저 OFF + P릴레이 ON 시 **프리차저 body diode 역류 → 칩 과열**
  - **임시 HW 조치**: PC 출력 ↔ P릴레이 출력단 연결 사이 **단방향 다이오드** 설치
  - **부작용**: `SENSOR_Voltage_OUT`(Vout) 이 **프리차저 ON 상태에서만 유효 계측**(PC OFF 시 다이오드 장벽 → Vout≈0 오독)
  - **SW 임시 대응**: `case BOOT_FSM_5_ON` 의 Vin↔Vout 전압비교(릴레이 정상 폐로 확인) **DNP** — `rd_system.c:347~348` `delta_voltage` 계산 주석처리 + `= 0` 강제 → FAULT 조건 무효화(미중지 시 Vout≈0 ⇒ delta 과대 ⇒ 상시 FAULT). `EN_SSP=0`(line 340) 동반
  - **부수**: `case OFF` 방전 확인(line 272)도 동일 이유로 inert(오검출 없음, 미탐지만). 프리차지 구간(2/6_PC_ON, PC ON) Vout 비교는 정상
  - **복구 예정**: HW 리비전(역류 차단 설계) 후 5_ON·OFF 전압비교 원복
- **▶ 다음**: HW 리비전 → 전압비교 원복 → Step 8 BMS CAN → Step 10 MARSHAL
- 반영: wiki(opmode §4-7 ⚠ 하드웨어 제약, peri §4·§6-1 검증노트, plan Step9·§3), history/memory. 코드=유저 작성·검증

---

## [Session 22] Step9 전원제어 로직 완결 — A 해결·발진 제거·FAULT 정책 (2026-10-04)

Session 21 의 A/발진/FAULT 교차전파 결함을 유저 반복 수정으로 전부 해소. Step9 로직 완결, 실보드 검증 단계로 이행.

- ✅ **A 해결 (STATE↔BOOT_FSM 경합 → 미러 구조)**: SW 핸들러가 `STATE=RUN_TRN` 간접 킥 대신 **BOOT_FSM 직접 킥**(STATE-FSM 구간, switch 이전). 킥 가드 = `BOOT_FSM==OFF||5_ON`(STATE 조건 무시). `STATE` 는 BOOT_FSM **미러**(OFF→RUN_OFF/5_ON→RUN_ON/전이중 RUN_TRN). 휴지 `case OFF`/`5_ON` 의 STATE= 는 미러 세팅으로만 기능 → 경합 소멸
- ✅ **발진(SW1 stale-delta) 제거**: 기존 `if(STATE==RUN_OFF||RUN_ON)` 중첩 게이트가 RUN_TRN 에서 `MAIN_SW_LAST` 갱신을 멈춰, 전이(~4s) 완료 직후 delta>SW_HIGH 로 자가 재킥 → 버스 on↔off 발진. SW1 핸들러를 RUN 블록(`STATE∈[1,3]`, RUN_TRN 포함) 직하로 올려 **`MAIN_SW_LAST` 전이 중에도 매 루프 갱신**. 이중방어(tick 상시 갱신 + BOOT_FSM 킥 가드). 킥에 `LOOP_CNT=0` 추가(정착/아크 dwell 보장)
- ✅ **FAULT 정책 확정(유저)**:
  - **온도/전압 폴트 = 전 시스템 중지**: 락아웃 시 `BOOT_FSM=FAULT` **및** `STATE=FAULT` 동시(switch default 릴레이 즉시 차단 + FAULT 분기 AUX1/AUX2 off). switch 가 FAULT 출력을 되살리던 결함 해소
  - **BOOT 폴트 = boot 만 중지**: 시퀀스 timeout 시 `BOOT_FSM=FAULT` 만, `STATE` 미전파(RUN_TRN 잔류 → EN_AUX1·SW2 Orin 유지). 역전파 코드 **없음**(의도)
  - **`STATE==FAULT` = 리부트 전용 래치**: 자동 복구 전이 없음(자동부팅 임베디드 보호). 향후 버튼 특수로직으로 PRA 리붓 예정
- ✅ **경미 수정**: 중복 `MAIN_SW_LAST` 제거, INIT→RUN_OFF 전이에 `MAIN_SW_LAST` 리셋(파워업 오킥 차단)
- **남은 관찰(조치 불요)**: BOOT 폴트 가시성(sys_state RUN_TRN 유지), 온도/전압 폴트 EN_AUX1 1-루프 지연(무해)
- **▶ 다음 = 실보드 검증**: 유저가 하드웨어 결선(버튼·릴레이·Vin/Vout/GND·외장 NTC + AUX 가상부하=벅컨버터) 완료 후 전원제어 실동작 검증 → 결과 공유 예정. BMS CAN(Step8)·MARSHAL(Step10)은 이후
- 반영: wiki(opmode §4·§4-7 재작성, plan §2·§3·Step9, history) + memory. **코드는 유저 작성·검증(CubeIDE)**

---

## [Session 21] RD_CONTROL_LOOP B~F 검증 + A 토론 + SW 역할 확정 (2026-10-04)

Session 18 검토 버그에 대한 유저 수정(B~F) 검증 + 구조버그 A 토론 + 결정 F 문서화.

- ✅ **B~F 수정 검증(유저 코드, 확인완료)**:
  - B: `case BOOT_FSM_1_LSR_ON` → `BOOT_FSM_2_PC_ON` 전이 정정 (rd_system.c:272)
  - C: `case BOOT_FSM_6_PC_ON` → `BOOT_FSM_7_HSR_OFF` 전이 정정 (:350)
  - D: fault 조건 반전 정정 — OFF `SENSOR_Voltage_OUT > RUN_VOLTAGE_TH`(:259), 5_ON `delta_voltage > PC_VOLTAGE_DIFF`(:335)
  - E: BOOT_FSM 킥 시 `MAIN_SW_LAST`/`LOOP_CNT` 리셋 추가 (:193·197), `BOOT_FSM_e` 라벨 6~9 정합(종료 역순 이름)
  - F: **SW1 = 메인버스(BOOT_FSM 방향천이), SW2 = Orin(AUX2) 토글** 확정 — 코드 유지, 문서 정정
- 🔴 **A 토론 (STATE↔BOOT_FSM 경합, 미해결)**: `case OFF`/`5_ON` 이 매 루프 `STATE=RUN_OFF/RUN_ON` 강제 → SW가 세운 `RUN_TRN` 이 같은 루프에 덮여 다음 루프 킥 분기(`else if STATE==RUN_TRN`) 도달 불가 → 버스 on/off 트리거 불가(구조적). **권고 Option 2** = SW 핸들러가 BOOT_FSM 직접 킥(OFF→1/5_ON→6, LOOP_CNT=0)·STATE는 BOOT_FSM 파생값(단일 진실원). **대안 1b** = round-trip 유지·resting 케이스 STATE= 삭제·완료 엣지에서만 세팅. 공백: BOOT_FSM FAULT→STATE 미전파. **코드 미수정(유저 Option 선택 대기)**
- **코드 실매핑 확인**: SW1_state→MAIN_SW path→BOOT_FSM(:180~197), SW2_state→AUX_SW path→EN_AUX2 토글(:417~425)
- 반영: `pra_opmode.md`(§4·§4-7 재작성), `pra_peri.md`(§4 PB15·§5 GPA0/GPA2 SW 역할 정정), `plan.md`(Step9·§3 경로), history/memory. **코드 미수정**(A 결정 대기)

---

## [Session 20] sys_state 발행원 단일화 구현 (2026-10-04)

#4 권고안 유저 승인 → 코드 반영.

- **addr23 ← `PRA_CONTROL.STATE` 미러**: `rd_map_pra.c:173` `reg.sys.sys_state=(uint8_t)PRA_CONTROL.STATE`
- **`payload_state` 폐기**: 전역 정의·extern·사용처 전부 제거 (rd_system.c/.h, rd_map_pra.c)
- **`ACTION_STATE_FAULT` 제거(#5)**: 통신 fatal self-reboot(`NVIC_SystemReset`)·전원차단 안 함. RD_TASK_SYSTEM 의 호출도 제거. 전원 FAULT 안전정리는 `RD_CONTROL_LOOP` FAULT 분기 단독. checker 는 fatal 시 `hw.reset.uart2` telltale(addr22)만
- **extern 추가**: `rd_system.h` 에 `extern CONTROL_PRA_t PRA_CONTROL;`(+`PRA_PERIPHERAL`)
- **주석/레지스터 갱신**: `rd_register_pra.h` addr23 enum → INIT/RUN_OFF/RUN_TRN/RUN_ON/FAULT(255), 발행원=PRA_CONTROL.STATE
- **`RS485_TEST_ON` define**: 이제 미사용·inert (reboot 경로 제거됨)
- **검증**: payload_state/ACTION_STATE_FAULT 실코드 참조 0건(주석만) — 컴파일 안전. **빌드 검증은 사용자**
- 반영: 코드(rd_system.c/.h, rd_map_pra.c, rd_register_pra.h) + wiki(opmode §1·§4-7 #4/#5, register §2-2, checker §2·§3), history/memory

---

---

## [Session 19] 백본 주의 처리 + sys_state 발행원 토론 (2026-10-04)

Session 18 주의 9건에 대한 유저 처리 + #4 토론.

- ✅ **버그 수정(유저)**: #1 RUN 전압폴트 `STATE=FAULT` 정정, #2 FAULT 릴레이 `EN_RelayN` 정정
- ✅ **#3 정리(유저)**: `CONTROL_PRA_t.STATE`/`BOOT_FSM` 은 enum 정의 준수, 혼동 주석 삭제
- ✅ **#5 결정(유저)**: **통신 FAULT ↔ 전원 FSM 의도적 미연계**. Orin 통신 단절(프로그램 종료 등)로 메인 전원 안 내림. checker 는 UART RECOVERY 만, 전원차단·self-reboot 안 함 → `ACTION_STATE_FAULT` self-reboot 제거 권고
- 🗣 **#4 토론 (sys_state 발행원 단일화)**: 조사 결과 `SYSTEM_STATE_e` 를 `payload_state`(통신 checker, addr23 발행, INIT/FAULT만)와 `PRA_CONTROL.STATE`(운용 FSM, 미발행)가 공유 — 의미 혼선. **통신 health 는 이미 `hw_error`/`hw_fatal`/`degraded_cnt` 로 독립 발행** → payload_state 의 sys_state 발행은 중복. **권고: `sys_state`(addr23) ← `PRA_CONTROL.STATE` 미러링, payload_state 폐기(또는 내부전용)**. DPC_B(DPC_CTL.STATE→sys_state) 패턴. 필요변경: rd_system.h `extern PRA_CONTROL` + rd_map_pra.c:173 미러 + register addr23 주석 enum 갱신 + Orin PCU 정합(PRA 개발 후). **유저 확정 대기**
- 반영: `pra_opmode.md`(§4-7 주의 1~8 상태갱신·§5 입력원 재해석), history/memory. **코드 미수정**(토론 대기)

---

---

## [Session 18] Step9 RD_CONTROL_LOOP 백본 검토 (2026-10-04)

유저가 `rd_system.c` 에 전원 제어 FSM 백본 구현 중(AUX1 강제복구 제외, SYS_STATE 골격). 검토 + 주의사항 정리(코드 미수정).

- **구조**: `CONTROL_PRA_t PRA_CONTROL` + `RD_CONTROL_INIT` + `RD_CONTROL_LOOP(CTL,PERI)`, `RD_TASK_CONTROL` 루프에서 호출. `SYSTEM_STATE_e` **재정의**(INIT/RUN_OFF/RUN_TRN/RUN_ON/FAULT255, 구 MANUAL/AUTO/ESTOP 폐기). `BOOT_FSM_e`(0~9 대전류 버스 순차 + 255). 임계 UVLO40/OVLO60[V]·OTLO80/UTLO-20[°C]·BOOT_TICK 1s
- **STATE 골격**: INIT(안정화·출력off, SSP먼저→릴레이 지연off) → BOOT_TICK 경과+전압정상 → RUN_OFF. RUN(1~3) EN_AUX1=1 + 온도/전압 락아웃. FAULT 전출력off
- **⚠ 주의(코드 수정 권장, opmode §4-7 상세)**: ①[버그] RUN 전압폴트가 STATE 대신 `LOOP_CNT=255`(`:191`) → FAULT 미진입 ②[버그] FAULT 릴레이off `EN_RelayP` 2회·`EN_RelayN` 누락(`:212~213`) ③강제복구 reboot `STATE==2` ↔ `RUN_TRN(2)` 충돌 ④addr23=`payload_state` 발행(운용 FSM `PRA_CONTROL.STATE` 미발행) ⑤통신FAULT↔전원FSM 미연계(릴레이 차단 안됨) ⑥`BOOT_FSM_e` 라벨 숫자≠값(`_6_` 중복) ⑦LOOP_CNT 상태간 공유 리셋 일관성 ⑧주석 RD_PROTECT_CHECKER OVLO 오타
- **진행예정**: RUN 내 BOOT_FSM SEQ, SW 뗄때델타 입력, AUX2(Orin) 토글, AUX1 강제복구
- 반영: `pra_opmode.md`(§1 sys_state 재정의·§4-7 백본+주의), `pra_task.md`(controlTask·globals), `pra_register.md` 연계, plan §1·§9, history/memory

---

---

## [Session 17] Step9 전원제어 설계 결정 (2026-10-04)

Session 16 정책에 대한 4개 결정 확정 → wiki 반영(코드 미착수).

- **① AUX 역할 정정**: **AUX1=내부 임베디드(MCU 계), AUX2=Orin** (HW 동일규격, 유저 정의 우선). → Session 16 의 반대 매핑 수정. **레지스터 §2-4(AUX1=MCU/AUX2=ORIN)와 정합** → 이전 "상충" 해소. 기능 재매핑: **SW1 ≥2s → AUX2(Orin) 토글** / **AUX1(내부 임베디드) 부팅 자동 ON + SW2 ≥10s 강제복구(2s off)**
- **② 전압 체크 기준 = `PERIPHERAL_t.SENSOR_Voltage_IN`** (입력 버스) — UVLO/OVLO 부팅 게이트
- **③ `BOOT_FSM` = DPC_B 식 switch-case (유저 작성)**. 전이 예시: `2→3` = `LOOP_CNT ≥ 임계` AND `|SENSOR_Voltage_IN − SENSOR_Voltage_OUT| ≤ 0.1V`(프리차징 완료). §2 순차 연계
- **④ SW 누름시간 판정 = 뗄 때 델타** (릴리즈 시점 tick 델타, DPC_B 패턴). 1회 판정이라 LONG/VERY_LONG 중복 발동 자연 해소
- **반영**: `pra_opmode.md`(§2 프리차징 판정·§3 AUX 역할·§4 버튼정책 재매핑·§6 TODO), `pra_peri.md`(§4 AUX·§5 SW), `pra_register.md`(§2-4 ctr_mode 역할), plan/task 참조, history/memory
- **코드 미착수** — `RD_TASK_CONTROL` 에 유저가 switch-case 로 구현 예정

---

---

## [Session 16] 스위치 기반 전원 on/off 정책 확정 (2026-10-04)

유저가 SW 버튼 정책 확정 → wiki 반영(코드 미착수). **BMS CAN(Step8)은 추후로 연기, 스위치 전원제어(Step9) 선행.**

- **정책 (opmode §4 신설)**:
  - **SW1(SWA1/GPA0)**: AUX1 전용 — **≥2초 길게 누름 → on/off 토글**. 단순 Orin 전원 온오프
  - **AUX2(내부 임베디드 부팅용)**: 부팅 시 UVLO/OVLO 체크 정상이면 **자동 ON**(SW 무관)
  - **SW2(SWA2/GPA2) = 메인 스위치**: **≥2초 → `BOOT_FSM`(high-current bus 프리차저+P/N 릴레이 순차) 방향 천이**; **≥10초 → BOOT_FSM 유지 + AUX2 2초 off·복구(강제복구/내부 임베디드 재부팅)**
  - 누름시간 임계(안): LONG ≥2000ms / VERY_LONG ≥10000ms
- **⚠ AUX1/AUX2 역할 라벨 상충 발견**: 레지스터 §2-4·peri §4 는 `AUX1=MCU계`/`AUX2=ORIN계` 인데 **정책은 `AUX1=Orin`/`AUX2=내부임베디드(MCU계)` 로 반대** → 실배선 확정·정정 필요(opmode §3·§6, peri §4 플래그)
- **태스크 명칭 변경 확인**: `RD_TASK_PERIPHERAL`→**`RD_TASK_CONTROL`**(rd_system.c/.h/main.c 일관, 코드 정상). 전원 FSM 은 이 controlTask 루프에 통합 예정
- **반영**: `pra_opmode.md`(§3 AUX 역할·§4 버튼정책 신설·§5 입력원 재번호·§6 TODO), `pra_peri.md`(§4 AUX 플래그·§5 SW 기능·RD_TASK_CONTROL 정합), `plan.md`(§1 Step8 추후/Step9 선행·§3), `pra_task.md`(controlTask), history/memory
- **코드 미착수**(유저 지시) — 정책만 문서화

---

---

## [Session 15] Step7 실보드 수동 검증 통과 (2026-10-04)

- **✅ ADC 정상**: NTC×3·Vin·Vout·전류(Current_BUS) 값 실보드 정상 확인 (uint16 버퍼폭 수정 효과 검증됨)
- **✅ MCP23017 EXIO I2C 통신 정상**: SW1/SW2 가 `1` 로 읽힘 — 유저 회로설계상 **10k 풀업**의 비눌림 상태(정상). read/통신 경로 동작 확인
- ※ 출력단(릴레이 RP/RN·프리차저 PC·AUX1/2·LED/BUZ) 수동 구동은 본 보고에 미명시 → Step9 전원제어 착수 전 확인 권장
- **게이트 통과 → Step8(BMS CAN) 진행 가능**. 착수 시 BMS CAN 프로토콜(메시지 ID·바이트 레이아웃·스케일·주기) 입력 필요
- 반영: `pra_peri.md`(§6 검증), `plan.md`(§1 Step7·§3·§3-4), `pra_overview.md`, history/memory

---

---

## [Session 14] Step7 정리 + 완료 표기 (2026-09-21)

유저 지시로 Step7 저순위 정리 + plan §1 완료 표기. 이후 실보드 수동 IO 검증 예정.

- **코드 정리(클로드)**:
  - `rd_peripheral_pra.h`: 헤더 주석 `rd_peripheral_dpcb.h`→`pra`, MCP23017 include `//test` 제거, **ADC/Convert 프로토타입 추가**(`RD_ADC_INIT`/`RD_UPDATE_ADC`/`Convert_NTC/Voltage/Current`)
  - `rd_peripheral_pra.c`: 헤더 주석 정정, `RD_PERIPHERAL_READ` stale 주석(Start_i2c/TODO ADC) 정리, ConvCplt 죽은 `//memcpy` 주석 제거
  - `rd_system.c` `RD_TASK_PERIPHERAL`: stale 주석(`//RD_ADC_INIT`, `//TODO EXIO setup`) 제거 + 동작 주석화
  - (`DYN_NUM_MOTORS`/`PERIPHERAL_MOT_t` 는 유저가 기제거 확인)
- **plan §1: Step 7 = 완료(2026-09-21)** 표기. EXIO 는 별도 Start_i2c 없이 controlTask(`RD_TASK_PERIPHERAL`) 루프에 폴딩 확정
- **▶ 다음 게이트: 실보드 수동 IO 검증** — 릴레이(RP/RN)·프리차저(PC)·AUX1/2·LED/BUZ·SW·ADC(V/I/NTC) 수동 동작 확인 후 Step8(BMS CAN) 진행
- **잔여(검증 무관)**: NTC index/Vin·Vout 매핑·float→int8 스케일 = Step10 MARSHAL 정합, `Convert_V/I` >=4095 오검출(경미)
- 반영: `pra_peri.md`(§6-1 D 완료), `plan.md`(§1 Step7 완료·§3-4 신설), `pra_overview.md`, history/memory

---

---

## [Session 13] ADC 버퍼폭 수정안 A 적용 — 최종 검토 (2026-09-21)

유저가 수정안 A 적용(+ADC 사용 인자 uint16 변경). 최종 검토 = **정상**.

- ✅ `adc_raw_buf`(전역)/`adc_value`(struct) = `uint16_t[6]`, `HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_raw_buf, 6)`, `Convert_NTC/Voltage/Current` 인자 uint16, ConvCplt 동일폭 복사 → `.ioc` DMA HALFWORD 와 정합, 6채널 슬롯당 1개 정상 적재. uint32 잔존 0
- **Step7 페리페럴 = 실질 완료 (기능 버그 0)**. 잔여 = Step10 MARSHAL 정합(NTC index/Vin·Vout in·out 매핑, float→int8 스케일) + 저순위(헤더 선언·copy 흔적·`Convert_V/I` 의 >=4095 관용구 전압/전류 오검출 소지)
- 반영: `pra_peri.md`(§6 함수현황·§6-1 A 완료), `plan.md`(§2·§4-4), history/memory

---

> ⚠ **Step 번호 재구성 (2026-09-21, Session 9)**: Session 6~8 의 이전 step 번호는 아래 매핑으로 갱신됨. 과거 로그 본문의 "(Step7/8/9)" 표기는 당시 번호 기준(아카이브).
> | 구 | 신 | 내용 |
> |---|---|---|
> | 7 MARSHAL | **10** | MARSHAL PUBLISH/CONSUME 본체(완료) |
> | 8 전원 FSM | **9** | 전원 제어 로직 |
> | 9 BMS CAN | **8** | BMS CAN 파서 |
> | (신설) | **7** | 페리페럴 구현(rd_peripheral_pra + MCP23017 EXIO) |
> | 10 EX·패널 | 폐기/흡수 | EX RS485 미사용, MCP23017 은 Step7 흡수 |

---

## [Session 12] Step7 페리페럴 2차 검토 — ADC 버퍼폭 버그 확정 (2026-09-21)

유저가 Session 11 잔여 5건 수정 후 재검토 요청(특히 항목2 ADC 자료형).

- **✅ 정상 확인**: ①ADC 버퍼 일원화(전역 `adc_raw_buf` 유지, ConvCplt 가 struct `adc_value[]` 로 캡처, RD_UPDATE_ADC 도 `adc_value` 사용) ③`RD_TASK_PERIPHERAL` `osDelay(10)` 추가 ④루프에 `RD_EXIO_UPDATE` 추가 ⑤`RD_PERIPHERAL_WRITE` 에 AUX1/AUX2 write 추가
- **★ 항목2 = 실제 버그 확정**: `adc_raw_buf`/`adc_value` 가 `uint32[6]` 인데 `.ioc` DMA `MemDataAlignment=HALFWORD`(MSIZE 16bit). **HAL `HAL_ADC_Start_DMA(…, uint32_t* pData, …)` 의 `uint32_t*` 는 API 고정 포인터 타입일 뿐 전송폭 미결정** — 실제 폭은 MemDataAlignment 가 결정. → DMA 가 6×16bit 를 2byte MINC 로 써서 uint32[6] 의 [0]=(CH3<<16)|CH2 … [2]=(CH7<<16)|CH6, [3~5]=0 → **2채널 패킹, ADC 전 채널 오동작**
  - **수정 A(권장)**: 버퍼 `uint16[6]` + `(uint32*)` 캐스트(.ioc 무변경, 12bit ADC 표준 관용구) / **B**: .ioc Mem=Word 재생성
- **잔여(나머지)**: 매핑 확인(NTC index↔buf 역순, Vin/Vout in·out) + 스케일 float→register int8 → **Step10 MARSHAL 정합**. 헤더 선언·copy 흔적·stale 주석(낮음)
- 반영: `pra_peri.md`(§6 함수현황·§6-1 A 재작성=버퍼폭), `plan.md`(§2·§4-4), history/memory

---

## [Session 11] Step7 페리페럴 통합 검토 (2026-09-21)

유저가 페리페럴을 rd_system 에 통합. 검토 후 잔여 갱신.

- **완료 확인(유저)**: `PRA_PERIPHERAL` 전역 정의(rd_system.c), rd_system.h 에 rd_peripheral_pra.h include, **`RD_TASK_PERIPHERAL`** 신설(INIT + `for(;;){READ;WRITE;}`) → **main.c StartControl(controlTask) 에 배선**. `RD_PERIPHERAL_INIT` 에 `RD_ADC_INIT`+`RD_EXIO_INIT` 호출, `RD_PERIPHERAL_READ` 에 `RD_UPDATE_ADC` 호출. 구조체 SENSOR_*/NTC **float 전환**(타입 절단 해소), NTC[2]·Current 변환 정정. ConvCplt 를 struct 멤버 기반으로 변경
- **잔여 이슈 (통합 후 재검토, peri §6-1)**:
  - ★ **ADC DMA 타겟 버그**: `RD_ADC_INIT` 는 전역 `adc_raw_buf` 로 DMA, ConvCplt 는 `PRA_PERIPHERAL.adc_raw_buf`(struct) 복사 → struct 미충전 → **ADC 값 항상 0**. DMA 타겟 struct 통일 필요
  - ★ **버퍼 uint32 vs DMA HALFWORD** → uint16
  - ★ **`RD_TASK_PERIPHERAL` for(;;) osDelay 없음** → busy-spin
  - **EXIO 주기 갱신 미배선**(RD_EXIO_UPDATE 루프 미호출), **RD_PERIPHERAL_WRITE AUX 미반영**
  - 매핑 확인(NTC 순서·Vin/Vout in·out), 스케일(float→register int8, Step10), 헤더 선언·copy 흔적
- **controlTask 성격**: 현재 페리페럴 I/O 루프. Step9 전원 FSM 을 여기 통합/배치할지 결정 필요
- 반영: `pra_peri.md`(§6 갱신), `plan.md`(§2·§4-4), `pra_task.md`(controlTask), history/memory

---

## [Session 10] Step7 페리페럴 코드 검토 — 잔여작업 정리 (2026-09-21)

유저가 ADC read 함수·구조체 구현 + `RD_EXIO_UPDATE` 버그 수정 후 코드 검토 요청. 현행 반영 + 잔여 리스트업.

- **완료 확인(유저)**: AUX1/AUX2(PB14/15) 필드·핀주입(INIT), `RD_EXIO_UPDATE` LED2/BUZ 정정, `PERIPHERAL_t` ADC 필드(SENSOR_Voltage_IN/OUT/Current_BUS/NTC[3] + adc_raw_buf/my_adc_register), 변환함수(`Convert_ADC_To_Voltage` ×0.0245 / `_To_Current` (raw−2048)×0.1465 / NTC Steinhart-Hart), `RD_ADC_INIT`/`RD_UPDATE_ADC` 골격, `PRA_PERIPHERAL` 명칭 정정. main.c ADC 콜백/버퍼 이동(중복 없음 확인)
- **검토 발견 — Step7 잔여**(peri §6-1 상세):
  - **ADC 버퍼 uint32→uint16**: DMA=HALFWORD 인데 전역 버퍼 uint32 → 폭 불일치 버그
  - **버퍼 이원화**: ConvCplt/RD_ADC_INIT=file-scope 전역, RD_UPDATE_ADC=struct 멤버 → struct 멤버 미충전(항상 0). 통일 필요
  - **RD_UPDATE_ADC 매핑 버그**: NTC[2] 누락(NTC[1] 이중대입), NTC index↔buf 역순(레지스터 규약 정합), `SENSOR_Current_BUS` 가 `Convert_ADC_To_Voltage` 오호출, Vin/Vout in·out 반대
  - **타입/스케일**: uint16 필드에 float 변환값 대입(절단·음수·-999) → 저장표현·레지스터 스케일 확정(Step10 연계)
  - **헤더/배선**: RD_ADC_INIT/UPDATE/Convert_* 헤더 미선언, INIT→ADC_INIT 미호출, READ→UPDATE_ADC 미호출, WRITE에 AUX1/2 미반영
  - **통합**: `PRA_PERIPHERAL` 정의 없음(extern만), RD_SYSTEM_INIT 미연동, Start_i2c 태스크 미생성
  - copy 흔적: 헤더 주석/MCP23017 `//test`/`DYN_NUM_MOTORS`/`PERIPHERAL_MOT_t`
- 반영: `pra_peri.md`(§6 전면 갱신), `plan.md`(§4-4), history/memory

---

## [Session 9] Step 순서 재구성 + AUX 반영 (2026-09-21)

유저 지시로 잔여 Step 순서 재배치. **페리페럴(EXIO 포함)을 먼저 세우고 → 센서(CAN) → 제어 → 마지막에 MARSHAL 로 마무리**. AUX1/AUX2 는 유저가 코드 수정 완료.

- **신 Step 순서**: 7 페리페럴 구현(rd_peripheral_pra 본체 + MCP23017 EXIO + Start_i2c + RD_SYSTEM_INIT 연동) → 8 BMS CAN 파서(+CAN1 Checker) → 9 전원 제어 로직(controlTask) → **10 MARSHAL PUBLISH/CONSUME 본체(프로젝트 완료)**
- 구 Step10 "EX RS485/MCP23017 패널": EX RS485 미사용 폐기, MCP23017 패널은 Step7 페리페럴에 흡수
- **AUX1/AUX2 추가 완료**(유저): `PERIPHERAL_t` IO/EN/INIT/WRITE 에 PB14/PB15 반영 → peri §6-1 AUX 항목 완료 처리
- **부수**: main.c `#include "rd_common.h"W` 오타(Session 8 발견분) 제거 확인
- 반영: `plan.md`(§1 표 재번호 + 재구성 note·§2·§3·§4-4/4-5), `pra_overview.md`·`pra_peri.md`·`pra_checker.md`·`pra_task.md` step 참조 정합
- **다음 착수**: Step 7 페리페럴 본체 (ADC read + EXIO 버그수정 + PRA_PERIPHERAL 정의·연동 + Start_i2c)

---

## [Session 8] rd_peripheral_pra 스켈레톤 검토·문서화 (2026-09-21)

유저가 `rd_peripheral_pra.{h,c}` 생성(DPC_B `rd_peripheral_dpcb` 기반). 코드 읽고 구조·추가/TODO 를 wiki 반영. **연동·본체는 Step7/8 예정, 현재 미연동 스켈레톤.**

- **구조**: `PERIPHERAL_t` = ADC data(Voltage_IN/OUT/Current_BUS/NTC[3] raw uint16) + CONTROL(EN_SSP/EN_RelayP/EN_RelayN) + IO(GPIO_SSP=PC_EN/RelayP=RP_EN/RelayN=RN_EN) + EXIO(`peripheral_exio_t`: SW1·SW2 in / LED1·LED2·BUZ out) + tick. 함수: `RD_PERIPHERAL_INIT/WRITE/READ`, `RD_EXIO_INIT/UPDATE`
- **MCP23017**: 드라이버 `MCP23017.{h,c}` 존재. EXIO 매핑 GPA0=SW1/GPA1=LED1/GPA2=SW2/GPA3=LED2/GPA4=SWA3(DNP)/GPA5=BUZ(액티브 부저). GPB 스위치 주석(미사용). ✅ `RD_EXIO_INIT` RST-먼저 순서 준수(DPC_B §22 교훈 반영)
- **발견 TODO/이슈** (peri §6-1):
  - **AUX1/AUX2(PB14/15) 누락** — IO/EN/INIT/WRITE 에 서브파워 2채널 없음 (ctr_mode[1]/[2] 구동 불가)
  - **ADC read 미구현** — `RD_PERIPHERAL_READ` TODO (my_adc_register→Vin/Vout/Ibus/NTC)
  - **`RD_EXIO_UPDATE` 버그** — LED2/BUZ write 가 `EXIO_LED1`(복붙) → LED2/LED3 로 수정 필요
  - **인스턴스/연동** — `PRA_PERIPHERAL` 미정의(`extern DPCB_PERIPHERAL` = 명칭 오류), RD_SYSTEM_INIT/MARSHAL 미연동
  - **Start_i2c 태스크 미생성** — EXIO_UPDATE 호출 주체 없음(MX 생성 필요)
  - copy 흔적: 헤더 주석 `rd_peripheral_dpcb.h` / `DYN_NUM_MOTORS 3`(PRA 모터 없음) / 주석 `PERIPHERAL_MOT_t`
- **부수 수정**: `main.c` USER CODE Includes 의 `#include "rd_common.h"W` — 끝에 오타 `W`(build-breaking) 발견 → 제거
- 반영: `pra_peri.md`(§5 EXIO 매핑·§6 페리페럴 계층 신설), `pra_opmode.md`(§3 AUX 누락), `plan.md`(§2·§4-3·§4-4), `pra_task.md`(§2 Start_i2c·§4 globals)

---

## [Session 7] Step 6 — rd_uart advanced 교체 + USART6 Checker/systemTask (2026-09-17)

**사용자 결정(AskUserQuestion): ECU식 고급 rd_uart 이식** (단순형 유지+경량 checker 대안 기각). RS485(USART6/Orin) 채널만 Checker, CAN1·I2C1 은 페리페럴 추가 후.

- **rd_uart 전면 교체** (`rd_uart.{h,c}` → ECU/DPC_B advanced): `ERROR_STATUS_t error`(STATE_t lifecycle+health/degraded_cnt/rx_error_cnt), `RD_UART_INIT(obj,huart)` 2-arg, `RD_UART_CHECKER(obj,k)`/`RD_UART_RECOVERY`, `RD_RS485_INIT/RECOVERY/TRANSMIT/CHECKER/IRQ_HANDLER`, `wake_task`(IDLE ISR 범용 기상), CR1 RMW 원자화(`uart_crit`), `comm_err_flag`. 단순형(`is_fatal`/self re-init, USART6 하드코드 wake) 폐기 → Step4 의 rd_uart 임시수정(①버퍼②wake③IRQ_HANDLER)은 advanced 판에 흡수
- **rd_system.c Checker 추가**: `RD_TASK_SYSTEM`(10ms osDelayUntil) → `RD_SYSTEM_HW_RESET_HANDLE`(reg addr5 Orin 요청 → 채널 RECOVERY + 플래그 클리어) → `RD_SYSTEM_CHECKER`(USART6: `RD_RS485_CHECKER(&PRA_rs485,DEGRADED_K_100HZ)` RET_NOK→`fatal_rs485_cnt+=FATAL_K`+RECOVERY, `≥FATAL_MAX(200)`→`hw.reset.uart2`+`payload_state=FAULT`) → `RD_SYSTEM_EVALUATE_STATE`(STATE_t→hw.error[health≥WARN]/hw.fatal[OFFLINE]) → FAULT 시 `ACTION_STATE_FAULT`(`RS485_TEST_ON` 해제 빌드에서 `NVIC_SystemReset`) → `RD_MAP_MARSHAL_PUBLISH`
- **globals/콜백**: `hw`(HW_ERROR_FLAG_t: bit0=can1/bit1=uart2/bit2=i2c, register HW_BIT 순서 일치), `fatal_rs485_cnt`, `tim_cnt`(=HAL_GetTick, TIM5 부재), `HAL_UART_ErrorCallback`(USART6→isr_err_code)
- **MARSHAL_PUBLISH SYSTEM 영역 구현**(rd_map_pra.c, +include rd_system.h): reg.sys hw_error/fatal/reset←hw.raw, sys_state←payload_state, degraded_cnt[1]←deg_pct(uart2), realtime_tick←tim_cnt. SENSOR_ADC/BMS·CONSUME 은 Step7
- **rd_comm_orin.c 복원**: advanced rd_uart 이므로 framing/CRC → `comm_err_flag |= COMM_ERR_*` OR set(Checker HC_ 매핑), 송신 `RD_RS485_TRANSMIT`(대문자)
- **main.c/it.c(USER CODE만)**: `StartSystem`=`RD_TASK_SYSTEM()`, it.c IDLE 핸들러명 `RD_UART_IDLE_HANDLER` 로 갱신, TC→`RD_RS485_IRQ_HANDLER` 유지
- **제약 준수**: CAN1/I2C1 Checker·EVALUATE·HW_RESET 훅 **주석 처리**(페리페럴 추가 후 활성). USART1(EX) 미사용
- **미결**: `realtime_tick` 스케일(ms vs register ×0.1ms) / `ACTION_STATE_FAULT` 전원 액추에이터 정리(Step8) / `RS485_TEST_ON` 제거 시점(실운용)
- **잔재 스캔**: 구 단순 API(`RD_UART_IDLEHandler`/`RD_RS485_Checker`/`is_fatal` 등) 0건 확인. **컴파일 검증은 사용자(CubeIDE)**
- 반영: `pra_checker.md`(전면 개정), task/overview/plan/history

---

## [Session 6] Step 4·5 — Orin RS485 통신 + modbus 이식 (2026-09-17)

DPC_B `rd_comm_orin`/`rd_map_dpcb` 를 PRA 단순형 `rd_uart` API 에 맞춰 이식. Orin(PCU 노드 `0xA1`) READ/WRITE/PING/REBOOT 서비스 완성.

**신규 생성:**
- `rd_comm_orin.{h,c}` — Orin RS485 패킷(0xAA55 헤더, CRC-16/IBM). `ORIN_MY_ID=0xA1`(=PacketID::PCU), 버퍼 256, 멀티세그 READ, RW(0x04) 미지원. **PRA rd_uart 는 comm_err_flag 없음** → framing/CRC/ID 불일치는 카운터 미변경·RET_WAIT (DPC_B 대비 진단 단순화). 송신은 `RD_RS485_Transmit`(PRA 명칭)
- `rd_map_pra.{h,c}` — `REGISTER_t reg` 정의 + 영역 LUT 10개(0/DEFINE1~15/SYSTEM/RSVD0/SENSOR_ADC/SENSOR_BMS/CMD_POWER/RSVD1/DIAG/RSVD2) + DISPATCH R/W(경계·R/O·UNLOCK 검증, CMD_POWER write 시 cmd_write_tick 갱신) + `RD_MAP_INIT`(DEFINE 기본값) + MARSHAL **void 스텁**(PERIPHERAL_t 미정의라 인자 없음)
- `rd_system.{h,c}` — 전역 객체 `PRA_uart6`/`PRA_rs485`/`ORIN_PACKET` + `SYSTEM_STATE_e`/`payload_state`/`rs485_init_fail_cnt` + `RD_SYSTEM_INIT`(huart6 바인딩·MAP/ORIN INIT) + `RD_TASK_RS485`(init 재시도 + 이벤트 for(;;): flagsWait→PUBLISH→READ→HANDLE(lock=0)→WRITE→REBOOT)

**교체:**
- `rd_common.h` → DPC_B 상위호환본(STATE_t/LS_*/HC_*/degraded 상수 추가, RD_RET/RET_* 유지) — `rd_register_pra.h` 의 `state`(STATE_t) 필드 컴파일 가능해짐

**rd_uart 수정 (통신 이식 필수분, "완료" 드라이버 손봄 — 사용자 인지 필요):**
- ① `RX/TX_BUFFER_SIZE` 128→**256/272** — 256B 전범위 READ 응답(≈264B) 및 공유 버스 타 노드 프레임 수용 (구 128 시 오버플로우→FRAMING 폭주, DPC_B history §20 동종)
- ② IDLE wake 대상 `USART1`→**`USART6`** — PRA Orin 채널 정정 (USART1=EX 미사용)
- ③ **`RD_RS485_IRQ_HANDLER` 신설**(ECU 이식) — TC 시 DIR→RX + **RE/IDLE 재활성** + gState 복원. `RD_RS485_Transmit` 이 TX 진입 시 RE/IDLE 를 끄는데 재활성 함수가 없어 **첫 TX 후 RX 영구정지**하던 갭 해소

**main.c / stm32f4xx_it.c (USER CODE 영역만 편집 — MX 재생성 보존):**
- main.c: Includes+`rd_system.h`, BEGIN 2+`RD_SYSTEM_INIT()`, `Start_rs485` 본체=`RD_TASK_RS485()` (ECU 구조: 태스크는 RD_TASK_xxx 호출만)
- it.c: Includes+`rd_system.h`, USART6_IRQ USER CODE 0=TC→`RD_RS485_IRQ_HANDLER`, USER CODE 1=IDLE→`RD_UART_IDLEHandler`

**미결/후속:**
- write 권한 lock=0(자유) — 모드 기반 권한(opmode §4/§6) TODO
- `realtime_tick`: PRA 에 TIM5 없음 → 발행원 후속 결정(Step6/7)
- MARSHAL 본체(Step7), Checker(Step6), 전원 FSM(Step8), BMS CAN(Step9)
- **컴파일 검증은 사용자(STM32CubeIDE)** — 이 환경 ARM 툴체인 없음

---

## [Session 5] Step 3 사용자 검증 반영 (2026-09-17)

사용자 교차검증 3건 확정 → `rd_register_pra.h`·wiki 반영. (CSV 는 파일 잠김으로 보류)

- **① rd_common.h 이식 확정**: `state` 필드 STATE_t 위해 rd_common.h 이식 필요 확정 → **Step4 통신 이식과 함께** 반영. **FSM 은 유저가 추후 수정** 예정
- **② HW 비트 순서 통일**: `degraded_cnt` 순서(CAN1, uart2)에 맞춰 HW 비트를 `bit0=CAN1 / bit1=uart2 / bit2=I2C1` 로 변경 (구: bit0=UART2). 헤더 `HW_BIT_CAN1/UART2/I2C1` + hw_reset addr5 주석. Step2 때 제기한 순서 불일치 해소
- **③ addr 53/60 명명 확정**: `charge_state`(53) → **`bms_state`**(BMS 가 CAN 으로 보고한 배터리 동작 상태). `state`(60) = **CAN1 포트 채널 상태**(STATE_t). 상수 `BMS_CHG_*` → `BMS_STATE_*` 개명. 두 필드는 서로 다른 의미(BMS 보고값 vs 통신 포트 상태)
- **반영 파일**: `rd_register_pra.h`(HW_BIT 순서, bms_state, BMS_STATE_*, addr60 주석), register §2-1·§2-2·§2-3
- **⏳ CSV 보류**: `register_map_pra.CSV` 편집기 열림으로 write 잠김(Device or resource busy) → hw_reset 비트순서(`CAN1 | UART2 | I2C1 | RSVD *5`)·addr53 명(`State`→`BMS_state`) 2건 미반영. **파일 닫은 뒤 재시도 필요**
- **다음**: Step 3 완료 → Step 4·5(통신 이식 + modbus 통합) 동시 착수 대기

---

## [Session 4] Step 2 — `rd_register_pra.h` 코드화 (2026-09-17)

확정 CSV(`register_map_pra.CSV`) → 256B packed `REGISTER_t`. DPC_B `rd_register_dpcb.h` 컨벤션(영역 offset/size #define, packed sub-struct, 주석 addr 표기, static_assert 주석) 그대로 이식.

- **파일 생성**: `stm_ws/PRA/Core/Inc/rd_register_pra.h`
- **영역 (9개)**: DEFINE(0~15) / SYSTEM(16~28) / RSVD0(29~37) / SENSOR_ADC(38~44) / SENSOR_BMS(45~60) / CMD_POWER(61~63) / RSVD1(64~79) / DIAG(80~99) / RSVD2(100~255). 합 256 byte 검산 완료
- **sub-struct**: `DEFINE_t` / `DATA_SYSTEM_t` / `DATA_SENSOR_ADC_t` / `DATA_SENSOR_BMS_t` / `CMD_POWER_t` / `DIAG_t` + `reserved0/1/2`
- **타입 반영**: NTC/전류/전압 = int8_t ×1, BMS voltage/current/soc = uint16_t, realtime_tick/soh/cmd_write_tick = uint32_t. `state`(44/60) = STATE_t
- **#define/상수**: 영역 offset·size, HW_BIT_UART2/CAN1/I2C1(`hw_reset` addr5 순서 기준), DEF_ERR/FATAL 기본값, CTR_MODE_OFF/ON, BMS_CHG_*, sys_state 값(SYSTEM_STATE_e 주석)
- **enum 미정의**: `SYSTEM_STATE_e` 는 rd_system.h(ECU 이식) 소유 → 값만 주석. 필드 uint8_t 유지
- **의존성 명기**: `state` 필드가 STATE_t 사용 → 현 최소본 rd_common.h 에 STATE_t 없음. Step4/6 통신·Checker 이식 시 DPC_B/ECU rd_common.h 함께 이식 필요 (헤더 상단 주석)
- **Step3 검증 이슈 표기**: degraded_cnt 인덱스(CAN1,uart2) vs HW 비트(UART2,CAN1) 순서 불일치 = CSV 그대로 반영 후 사용자 확인 요청. 상세 체크리스트: [plan.md](plan.md) §3-1
- **다음**: Step 3 사용자 교차검증 → 통과 후 Step 4·5(통신 이식 + modbus 통합) 동시 진행

---

## [Session 3] Orin 정합 방향 + USART1 미사용 확정 (2026-09-17)

- **Orin 정합 방향 확정**: **PRA 레지스터맵을 기준**으로 Orin `rd_register_pcu.hpp` 를 수정(PRA→Orin 방향). **PRA 개발 완료 이후** 착수 (현 Orin PCU 맵 placeholder 유지). 반영: register §3, plan §4-1
- **USART1(EX RS485) = 미사용 확정**: 통신형 BMS 인터페이스가 RS485/CAN 2종으로 흔해 **미래 확장 대비**로 MX 에서 핀·페리페럴만 열어둔 것. **현 시스템은 CAN1(BMS) 만 사용** → USART1 기능 미사용. 태스크·전역객체·Checker 감시 대상에서 제외. 반영: peri §1, overview §2, task §2·§3·§4, checker §2, plan §4-3

---

## [Session 2] 사용자 피드백 반영 + CSV 갱신 검토 (2026-09-17)

Session 1 문서에 대한 사용자 피드백 4건 반영 + 갱신된 `register_map_pra.CSV` 검토.

- **① PRA = PCU 확정**: Orin PCU 노드(PacketID `0xA1`) = PRA. **명칭 규약** — STM 개발문서(`pra_config_wiki`/`stm_ws/PRA`)는 `PRA`, Orin·외부 관점은 `PCU`. `ORIN_MY_ID=0xA1`. 반영: overview §1·§6, register §3, plan §4-1
- **② DIAG 헤더 불일치 설명 추가**: CSV `[DIAG]` 헤더 `[84:99]` vs 실제 `cmd_write_tick`=80~83 → 실제 DIAG=**80~99(20 byte)**. 앞 RESERVED 헤더 `[64:79]` 라 addr 80~83 이 어느 헤더에도 없는 공백. 코드화 시 `[80:99]` 정정 권장. register §2-5 상세 설명
- **② NTC 타입 int8_t 확정**: CSV 갱신(int16→int8_t) 반영. register §2-3
- **② addr 100~255 = RESERVED 확정**: register §1 표·노트
- **③ SENSOR = 자체 ADC 확정**: SENSOR/STATE(38~43) = PRA 자체 ADC1 CH2~CH7. 레지스터↔채널 매핑표 작성(NTC[0/1/2]=CH2/3/4=보드 NTC3/2/1 **역순**, current_input=CH5, voltage_input=CH6, voltage_output=CH7). EX RS485(USART1) 로드셀 추정은 근거 소멸 → 역할 미정. 반영: register §2-3, peri §2·§1
- **CSV 검토 피드백 (사용자 전달)**:
  - 반영 양호: int8_t NTC, ADC sub-group, current_input/voltage_input/voltage_output(×1 스케일), SENSOR/STATE 에서 rs485_proc_delta 제거(자체 ADC 라 불필요)
  - 잔여 경미 이슈 (a)~(e) 제기 → **사용자 CSV 반영 완료**:
    - (a) DIAG 헤더 주소 → `[80:255] (176 byte)` 로 정정 (DIAG 80~99 + 후행 RESERVED 100~255 통합 헤더)
    - (b) addr 100 RESERVED size → 156 (100~255)
    - (c) `state`(44) Sub → `ADC` 로 통일
    - (d) degraded_cnt(16~19) Group → `SYSTEM` 일괄 정정
    - (e) hw_error/hw_reset "소프트 리셋 트리거" 문구 = **의도된 기능(유지)** — Orin 이 PRA 특정 채널(포트)을 수동 리셋하는 용도. `hw_reset`(addr5, R/W) 비트도 `UART2｜CAN1｜I2C1｜RSVD×5` 로 갱신
  - 반영: register §1·§2-1·§2-2·§2-3·§2-5

---

## [Session 1] pra_config_wiki 착수 — 초기 세팅/구성 문서화 (2026-09-17)

신규 PRA 프로젝트 착수. 기존 코드는 미수정, 워크스페이스 구조·`pra_config_wiki.md`·`.ioc`·`main.c`·레지스터 CSV 를 읽고 dpcb_config_wiki 구조를 참고해 문서 세팅.

- **문서 생성 (8종)**: overview / peri / register / opmode / task / checker / plan / history — dpcb_config_wiki 구조 미러
- **확정 사항 정리 (`.ioc` + `main.c`)**:
  - MCU STM32F446RET6, FreeRTOS CMSIS v2
  - ADC1 6채널 DMA: NTC1/2/3(PA4/PA3/PA2, 480cyc) + IOUT(PA5) + VOUT×2(PA6/PA7, 15cyc). ConvCplt 스냅샷 + NTC(Steinhart-Hart Beta)/로드셀 변환 헬퍼 구현
  - USART6 921600 = Orin RS485(DIR PC8) / USART1 = EX RS485(DIR PA8, 외부센서 추정) / CAN1 250kbps = BMS / I2C1 = MCP23017(RST PC13)
  - GPIO 파워단: RP_EN(PB10)/RN_EN(PB12) 메인 릴레이, PC_EN(PB13) 프리차저, AUX1_EN(PB14)/AUX2_EN(PB15) 보조전원, LED_G(PC11)/LED_R(PC12)
  - 태스크: defaultTask(LED 블링크만 구현)/rs485Task/controlTask/systemTask/can1Task — 나머지 빈 루프
  - `rd_uart` 이식 완료, `rd_peripheral`(GPIO_IO_t만)/`rd_define`(빈) 골격
- **레지스터맵**: CSV 초안(CP949) 정리 → 256B 개요표 + 섹션별 필드(DEFINE/SYSTEM/SENSOR(STATE·BMS)/cmd/DIAG). sys_state = ECU `SYSTEM_STATE_e`(INIT~FAULT) 사용
- **핵심 발견 — Orin PCU 연관**: Orin 브리지에 `PCU` 노드(PacketID `0xA1`, 레지스터 미확정, `enable_pcu_read` off) 예약됨. 기능 일치 → **PRA = PCU 강하게 추정**. 단 양측 레지스터맵 주소 구획 불일치 → 정합 TODO
- **CSV 정합성 이슈 식별**: DIAG 헤더 주소(84:99) vs `cmd_write_tick`(80) 불일치, NTC 타입(int16)/크기(1) 불일치, addr 100~255 미정의 → 코드화 시 정리 대상
- **미착수 (계획)**: 레지스터맵 코드화, RS485 modbus 계층, Checker/rd_system, 전원제어 FSM, BMS CAN 파서 — ECU/DPC_B 이식 전략
- 반영: `pra_config_wiki/` 전 문서

> 다음 단계 후보: Step 2(레지스터맵 코드화) 또는 Orin PCU 정합 선결. [plan.md](plan.md) §3.
