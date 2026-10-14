# PRA BMS CAN 구현 계획 / 작업 로그

> **이 문서 = 작업 계획·Step·진행 로그 전용.** 프로토콜 규격·바이트 정의·스케일·PCAN 실측은 중복 금지 → [BMS_protocol_can.md](BMS_protocol_can.md) 단독 소유.
> 참조 구현: ECU_V3 CAN 계층 — `stm_ws/ECU_V3/Core/{Inc,Src}/can_ak.{h,c}`(저수준), `rd_can_motor.{h,c}`(래퍼), `rd_system.c`(태스크/Checker 디스패치), `stm32f4xx_it.c`(RX ISR). ECU 설계원칙은 `stm_ws/ECU_V3/CLAUDE.md`.
> 상위: [pra_overview.md](pra_overview.md) · [plan.md](plan.md) Step 8

---

## 0. 전제 / 현재 상태 (2026-10-10)

- **채널**: BMS = **CAN1**(`hcan1`). MX 현재값: Prescaler 10 / BS1 13TQ / BS2 2TQ / SJW 1 → **250 kbps**, Mode NORMAL, **AutoRetransmission DISABLE**, ReceiveFifoLocked DISABLE.
- **태스크**: `can1Task`(`Start_can1`) **빈 루프** 존재. 전역 `hcan1` 존재.
- **미존재**: CAN 핸들 래퍼·에러 구조(`BMS_COMM_t` 등) 페리페럴에 없음, RX 콜백·필터·Start 미구현, `HW_NowTick` 포트 미정의.
- **레지스터**: SENSOR/BMS(**48~64**, 17B) + `#define`/`static_assert` 동기화 완료(2026-10-13: SENSOR/ADC int16 확장으로 45→48 밀림). 수집 대상·필드·스케일 = [BMS_protocol_can.md](BMS_protocol_can.md) §6.
- **구동 방식**: **요청→응답 폴링**(마스터 PRA 가 요청 프레임 송신 후 응답 수신). 필수 3종 = 0x90/0x92/0x98. **저율(~1s 주기) 단순 로깅용**.

### 🔒 격리 원칙 (최우선 제약, 2026-10-10 유저 확정)

> **BMS CAN 파서는 PRA 의 동작·전원 제어·시스템 리붓에 절대 관여하지 않는다.** 오직 CAN 수신값을 레지스터맵(SENSOR/BMS)에 갱신하는 것만 수행.
- BMS 미연결 / 외부 서플라이 구동 등 **미응답 상황이 PRA 에 어떤 문제도 일으키면 안 됨** (요청 ACK 실패·타임아웃이 FAULT·리붓으로 번지지 않음)
- 통신 단절(응답 타임아웃 / CAN HW 에러) 시 **CAN 서브시스템만 재초기화 후 재시도** (ECU `CAN_RECOVERY` 와 동일 개념). 전원 FSM·`PRA_CONTROL.STATE`·`NVIC_SystemReset` 등과 **무연계**
- 0x98 고장상태 포함 모든 BMS 값 = **순수 로깅**. 내부 시스템 판정에 사용 금지
- ⇒ ECU 의 "OFFLINE → fatal 카운터 → `SYS_STATE_FAULT` escalation" 사다리는 **BMS CAN 에 이식하지 않음**. CAN 은 자기 완결 루프(수신→reg 갱신 / 끊기면 자기 복구)로만 동작

### ⚠ ECU 와의 핵심 차이 (이식 시 반드시 반영)

| 항목     | ECU_V3 (AK 모터)                      | PRA (BMS)                                          |
| ------ | ----------------------------------- | -------------------------------------------------- |
| 수신 모델  | **Unsolicited** 상시 피드백(명령 무관 100Hz) | **Poll**: 요청 프레임 송신 후 응답 대기                        |
| TX 성격  | 제어 명령(200Hz, 큐 비동기)                 | 요청 트리거(저율 3종 **동시 송신**, ~1s 주기)                    |
| TX 경로  | `canTxQueue` + TX task              | **메일박스 직접 송신**(3 mailbox = 3 요청, 큐 불요)             |
| ID     | 확장 29-bit, 하위 8bit=모터ID             | 확장 29-bit `0x18[데이터ID][dst][src]`, 응답 `0x18xx4001` |
| 노드 수   | 4 (per-motor mask)                  | 1 (per-데이터ID 폴링)                                   |
| 존재 게이트 | `ALL_READY`(피드백 신선도 → motor_on)     | **없음** — 응답 신선도(`delta_tick`)는 로깅 + CAN 자기복구 트리거만, **전원 무연계(격리)** |
| 복구 escalation | OFFLINE → fatal → `SYS_STATE_FAULT` | **없음** — CAN 자기 재초기화 루프만, 시스템 전파 금지 |

---

## 1. ECU 참조 — 가져올 것 / 바꿀 것

**그대로 이식(설계 자산)**
- **4함수 인터페이스 + RECOVERY**: `*_INIT / *_UPDATE / *_TRANSMIT / *_CHECKER` + `*_RECOVERY` (rd_can_motor 패턴)
- **Checker/Recovery 분리**: 드라이버는 진단만(`RET_OK/WAIT/NOK`), 복구·전이는 `systemTask`
- **STATE_t + Degraded Counter**: `err.can.state`(lifecycle+health), `degraded_cnt` 증감/히스테리시스, `HAL_FATAL_CNT_TH` 즉시 OFFLINE
- **ISR↔task 분리**: RX 콜백은 `volatile` 필드만 단일 store + ts latch, 카운터/degraded 는 task
- **HAL 에러 처리**: `HAL_CAN_ErrorCallback` 에서 `isr_err_code` 캡처(폭주 차단 위해 notification off) → CHECKER 가 `isr_err_take()` read-clear + 매 tick `HAL_CAN_ActivateNotification` 재무장
- **RX split**: `*_RX_POP`(FIFO→프레임) → `*_RX_APPLY`(ID 매칭 시 핸들 갱신)
- **CAN_Init/RECOVERY**: 필터+인터럽트+Start / Stop→DeInit→Init 재초기화
- **HW_NowTick 포트**: delta_tick 체계용 free-run tick 주입

**변경(BMS 특화, §4 확정 반영)**
- **Unsolicited → Poll 엔진**: `RD_TASK_CAN()` 가 **3 요청을 메일박스 3개에 동시 적재**(간격 무시) → ~1s 주기 반복. 저율 로깅
- **TX 큐 제거**: 메일박스 직접 송신(`HAL_CAN_AddTxMessage` ×3). bxCAN TX 메일박스 3개 = 요청 3개 → 소실 없음. `canTxQueue`·stale 로직 전부 불요
- **확장 ID 필터**: `0x18xx4001` **mask**(응답만 통과). 요청 ID(…0140)는 자기 송신이라 수신 불필요
- **per-motor mask → per-데이터ID**: RX 콜백이 ExtId[23:16](데이터ID)로 분기해 핸들 ID별 raw 캐시에 적재
- **ALL_READY / escalation 제거**: 격리 원칙(§0) — 응답 신선도(`delta_tick`)는 **① 로깅 ② CAN 자기복구 트리거**로만. 전원/시스템 무연계
- **복구 트리거 단순화**: `delta_tick > 1000ms`(1 주기 무응답) **또는** CAN HAL 에러 → `CAN_RECOVERY`(CAN만 재초기화). ECU 의 degraded 히스테리시스·OFFLINE fatal 사다리 **미이식**

---

## 2. 파일 구성 (확정 — 단일 파일)

**`rd_can_bms` 단일 파일**에 전부 집약(§4-3). 파서가 자기 완결적이라 외부 결합 최소화 = 격리 원칙에 부합.

| 구분 | 파일 | 포함 내용 |
|------|------|-----------|
| **신규(단일)** | `Core/Src,Inc/rd_can_bms.{c,h}` | ① 핸들/에러 구조 `BMS_Handle_t`·`BMS_COMM_t` ② `CAN_Init`(필터/Start) ③ 요청 빌드·메일박스 송신 ④ **HAL CAN 콜백**(`HAL_CAN_RxFifo0MsgPendingCallback`·`HAL_CAN_ErrorCallback` — weak override) ⑤ RX pop/apply ⑥ 디코드→reg 매핑 ⑦ Checker/Recovery ⑧ **`RD_TASK_CAN()`** 폴링 엔진 ⑨ `HW_NowTick` |
| 참조 | `rd_register_pra.h` | `DATA_SENSOR_BMS_t`(48~64) — 발행 대상 |
| 참조 | `rd_map_pra.*` / `ResmapMutex` | reg SENSOR/BMS 영역 갱신 경로 (기존 규약 준수) |
| **수동(유저)** | `main.c`(USER CODE) | `Start_can1` 본체에 **`RD_TASK_CAN();` 하네스 추가는 유저가 코드 확인 후 수동 반영**. 필터/Start 는 `RD_TASK_CAN` 진입부 또는 `MX_CAN1_Init` USER CODE |

> **인터럽트 소유(2026-10-10 변경, ECU_V3 정합)**: CAN1 RX0/SCE 를 **.ioc(CubeMX)에서 활성** → `CAN1_RX0_IRQHandler`/`CAN1_SCE_IRQHandler`(→`HAL_CAN_IRQHandler(&hcan1)`)는 `stm32f4xx_it.c`, NVIC enable(prio 5)은 `HAL_CAN_MspInit`(stm32f4xx_hal_msp.c)이 생성·소유. `rd_can_bms.c` 는 **HAL weak 콜백(RxFifo0MsgPending/Error)만** override. (초기 8-1 의 "자급식 IRQ + .ioc OFF" 방식에서 전환 — 변수 배제·표준 흐름)
> `rd_system.c`/systemTask 는 **건드리지 않음**(격리) — Checker/Recovery 도 `RD_TASK_CAN` 자신이 호출.

---

## 3. 작업 Step (Step 8 세분)

### Step 8-1 — CAN1 HW 브링업 (필터·인터럽트·Start·시계 포트) — ✅ 구현(검증 대기)
**파일**: `Core/{Inc,Src}/rd_can_bms.{h,c}` 신설.
- **확장 ID 필터** `0x18xx4001` **mask**(FIFO0, bank0 32-bit): ID(29b) `0x18004001`, mask(29b) `0x1F00FFFF`(상위 `0x18`·호스트 `40`·BMS `01` 매치, 데이터ID 바이트[23:16] don't-care). 레지스터는 `id<<3|IDE` 로 변환(CAN_RIxR 배치). 응답 3종만 통과, 요청/표준프레임 차단
- **인터럽트(2026-10-10 변경)**: CAN1 RX0/SCE 를 **.ioc 에서 활성** → 핸들러=`stm32f4xx_it.c`, NVIC=`HAL_CAN_MspInit` (CubeMX 생성·소유). `rd_can_bms.c` 는 HAL weak 콜백만. (초기 자급식 IRQ+.ioc OFF 에서 전환) *— `RD_CAN_BMS_INIT` 의 NVIC enable·자작 핸들러 제거됨*
- `HAL_CAN_Start` + `HAL_CAN_ActivateNotification`(`RX_FIFO0_MSG_PENDING` + ERROR 계열)
- `HW_NowTick()` = `HAL_GetTick()`(ms, TIM5 부재 — ×0.1ms 스케일 불일치 기존 이슈 계승)
- **RX 콜백**(`HAL_CAN_RxFifo0MsgPendingCallback`): FIFO drain + `BMS_rx_count++` + 최근 프레임 디버그 캡처(`BMS_last_rx`: ext_id/data_id/dlc/data)
- **Error 콜백**: `BMS_isr_err_code|=`, `BMS_err_count++`, 에러 notification off(폭주 차단, 재무장은 8-5)
- `RD_TASK_CAN()`: INIT(실패 시 CAN 만 재시도) 후 idle(요청 폴링은 8-3). **main `Start_can1` 에 `RD_TASK_CAN();` 는 유저 수동 추가**
- **검증(유저)**: `.ioc` CAN1 **RX0/SCE 인터럽트 활성**(재생성) → 빌드 → `Start_can1`에 `RD_TASK_CAN();` 추가 → **PCAN 으로 이전 BMS 응답 더미패킷(`18904001`/`18924001`/`18984001`) 송신** → 디버거로 `BMS_rx_count` 증가 + `BMS_last_rx` 내용 확인

**✅ 실기 검증 결과 (2026-10-10)**: `18904001`(DLC8) 송신 → `BMS_rx_count=1`, ExtId·data 전부 정상 수신 = **수신 경로 통과**. 단 `BMS_err_count=1`/`BMS_isr_err_code=0x10`(`HAL_CAN_ERROR_FOR`, Form error) 1건 발생 → **LEC(Last Error Code) 단발 트랜션트**(버스 idle→active 전환 시 흔함, 수신 데이터 정상이므로 비트타이밍·버스 정상). err_count 가 1 에 멈춘 건 Error 콜백의 notification-off 폭주차단이 정상 동작한 것.
> **조치**: `CAN_IT_LAST_ERROR_CODE` 구독 제거(ActivateNotification/Deactivate 양쪽) — form/stuff/CRC 단발 프레임 에러는 복구 판정에 불필요(타임아웃 + EWG/EPV/BOF 로 판정). 재빌드 시 `BMS_err_count` 0 유지 예상.
> **참고**: 현 비트타이밍(presc 10 / BS1 13 / BS2 2 = 16tq)에서 250k 는 CAN클럭(APB1) 40MHz 전제 — 수신 정상이므로 타이밍 적정. form error 가 매 프레임 재현되면 APB1 클럭/샘플포인트 재확인 권장(현재는 1회성).

### Step 8-2 — 핸들 + 요청 빌드 + RX pop·apply — ✅ 구현(검증 대기)
- 핸들 `BMS_Handle_t`(전역 `BMS`): ID별 raw 캐시 `msg_vic`/`msg_temp`/`msg_fault`(각 `volatile data[8]`+`valid`) + `last_rx_tick`(`volatile`, ISR latch). 에러 집계 `BMS_COMM_t` 는 8-5
- `CAN_BMS_REQUEST_ALL(hcan)`: 요청 3종(`0x18[90/92/98]0140`, 8×`0x00`)을 **TX 메일박스 3개에 직접 적재**(`HAL_CAN_AddTxMessage` ×3). 전부 성공 `RET_OK`/일부 `RET_WAIT`/전무 `RET_NOK`. AutoRetransmission DISABLE → 무ACK 시 1회 시도 후 메일박스 자동 해제(부재 BMS 에도 starvation 없음)
- `CAN_BMS_RX_POP(hcan,&frame)`: FIFO0→`BMS_RxFrame_t`(ext_id/data_id=ExtId[23:16]/dlc/data)
- `CAN_BMS_RX_APPLY(&BMS,&frame)`: data_id ∈ {0x90,0x92,0x98} 매칭 시 해당 캐시에 raw 저장 + `valid=1` + `last_rx_tick=HW_NowTick()`. 비대상(필터 통과하나 미요청) 무시
- RX 콜백을 pop→apply 구조로 전환(디버그 `BMS_last_rx` 유지), `RD_TASK_CAN` 에 **~1s 주기 `REQUEST_ALL`** 추가(복구 트리거는 8-3)
- **검증(유저)**: 실 BMS 연결 시 ~1s 주기 요청 → `BMS.msg_*.valid=1` + raw 가 PCAN 값과 일치 / `last_rx_tick` 갱신. (또는 PCAN 더미 응답 수동 송신으로 apply 적재 확인)

### Step 8-3 — 폴링 엔진 `RD_TASK_CAN()` + 자기완결 복구 — ✅ 구현(검증 대기)
- `RD_TASK_CAN()`: `INIT(실패 시 CAN 재시도) → for(;;){ REQUEST_ALL → osDelay(1s) → 복구 판정 }`. 3 요청 동시 적재
- 수신은 ISR(콜백)이 비동기 적재 → 루프는 요청 송신 + 복구 판정만
- **복구 판정(자기완결)**: `(HW_NowTick() - BMS.last_rx_tick) > BMS_RX_TIMEOUT_MS(1500)` **또는** `BMS_isr_err_code != 0`(HAL EWG/EPV/BOF) → `RD_CAN_BMS_RECOVERY(&hcan1)`. **systemTask·전원 FSM 무관(격리)**
  - 타임아웃 임계 = **1500ms**(유저 "1주기(1000) 무응답" + 지터 마진, 응답 BMS 오탐 방지 — 튜닝 가능)
- **`RD_CAN_BMS_RECOVERY`**: `HAL_CAN_Stop → DeInit(MspDeInit: NVIC/GPIO 해제) → Init(MspInit 재설정) → RD_CAN_BMS_INIT(필터/clear/Start/Notification 재무장)`. INIT 이 `last_rx_tick=now`·`BMS_isr_err_code=0` 리셋 → 복구 후 타임아웃 창 재시작. ECU_V3 `CAN_RECOVERY` 개념
- `RD_TASK_CAN()` 하네스(main `Start_can1`)는 **유저가 코드 확인 후 수동 추가**
- **BMS 미연결 거동**: 요청 ACK 실패 → 타임아웃(또는 EWG) → 복구 → 재시도 무한 반복(격리, PRA 무영향). BMS 연결 시 응답 → last_rx_tick 갱신 → 복구 안 함
- **검증(유저)**: ① 정상 BMS: ~1s 폴링 지속, 복구 안 일어남 ② 케이블 분리: ~1.5s 후 CAN 복구 반복, **전원/STATE 불변** ③ 재연결 시 자동 재개

### Step 8-4 — 디코드/스케일 + 레지스터 갱신 — ✅ 구현(검증 대기)
- `RD_CAN_BMS_UPDATE`: 핸들 raw(ISR 적재) **snapshot(taskENTER_CRITICAL)** → **big-endian 디코드** → **reg.sensor_bms 발행**(프로젝트 규약 = `taskENTER_CRITICAL`, ResmapMutex 아님). `RD_TASK_CAN` 루프서 REQUEST_ALL+osDelay 후 매 주기 호출
- 변환(레지스터 저장): voltage=BE(B0,B1) u16 / **current=(int16)(BE(B4,B5)−30000)** 오프셋 제거 / soc=BE(B6,B7) u16 / **max_cell_temp=0x92 B0 raw(40 offset 보존, uint8)** / err_lsb=0x98 B0~3·err_hsb=B4~7(LE 패킹). 미수신(valid=0) 필드는 0
- ⚠ **정합 수정**: `max_cell_temp` 는 **오프셋 보존(raw, uint8)** 이 정답(uint8 이 음수 못 담음) — BMS_protocol_can.md §6/§7 의 "raw−40" 표기는 오류였어 정정함. current(int16, 오프셋 제거)와 **비대칭**(필드 타입 강제)
- delta_tick(60)/state(61) 는 8-5/8-6
- **검증(유저)**: Orin READ(또는 디버거 `reg.sensor_bms`) = PCAN 값 일치 — 53.8V(538)/0A(current=0)/68%(680)/24°C(max_cell_temp=64, −40 후 24)/error=0

### Step 8-5 — Checker (채널 STATE_t 발행, 로깅 전용) — ✅ 구현(검증 대기)
- `RD_CAN_BMS_CHECKER`: 응답 신선도(`last_rx_tick`) + HAL 에러(`BMS_isr_err_code`) → **STATE_t(lifecycle+health)** → `reg.sensor_bms.state`(addr61) 발행. 디버그 전역 `BMS_ch_state`
  - health: `BOF`→HC_BUS_OFF / `EPV`→HC_BUS_PASSIVE / `EWG`→HC_BUS_WARNING / 응답끊김→HC_TIMEOUT / else HC_OK
  - lifecycle: 첫 응답 전=LS_READY / 신선=LS_RUNNING / 끊김=LS_DEGRADED
- 🔒 **로깅 전용** — addr61 발행만, **어떤 시스템 카운터/FSM 에도 가산 금지**. ECU degraded decay·OFFLINE→fatal→SYS_STATE_FAULT escalation **미이식**(1Hz 채널엔 과함). 복구는 8-3(RD_TASK_CAN 자기완결)이 단독 — CHECKER 는 분류/발행만
- **검증(유저)**: 정상 addr61=`RUNNING|OK`, 케이블 분리 시 `DEGRADED|TIMEOUT`(또는 BUS_*), 전원/STATE 불변

### Step 8-6 — delta_tick 발행 (최소 통합) — ✅ 구현(검증 대기)
- `RD_CAN_BMS_CHECKER` 내 `delta_tick`(addr60) = `(now − last_rx_tick)[ms] / 100` = **×0.1s 스케일**(0xFF 포화=25.5s↑). **BMS 예외 스케일**(타 센서 ×0.1ms 와 다름, register CSV 변경 반영) — 1Hz 폴링서 정상치 ~10(1.0s)로 신선도 판정 유효
- ❌ **`hw_reset`(addr5 CAN1 bit) 미구현**(유저 확정): CAN 자기복구(8-3)로 Orin 수동 리셋 불요
- ❌ **0x98 ↔ 전원 FSM 연계 안 함**(§4-4): opmode §4-7 FAULT 와 완전 분리, 순수 로깅
- ⇒ **CAN 파서 코드 대부분 close** (8-1~8-6). 남은 건 Orin 연동(8-7)·리비전 실 BMS(8-8)

### Step 8-7 — Orin 연동 실테스트 (**Orin 준비 후**)
- Orin READ 로 SENSOR/BMS(48~64) 정합 / `rd_register_pcu.hpp` 매핑(current int16·max_cell_temp 40offset §6) 확인 / 케이블 분리·재연결 Orin 관점 거동
- 🔴 **선결 TODO — SENSOR/ADC(addr 38~44) reg 발행 미연결 (유저 작업)**: 현재 `reg.sensor_adc` 가 `PRA_PERIPHERAL` 자체 ADC 값(NTC[3]/Vin/Vout/Current)과 **MARSHAL 미연결**(`RD_MAP_MARSHAL_PUBLISH` 에 `sensor_adc` TODO 스텁). 게다가 레지스터 필드는 **임시 `int8_t`**(×1)인데 실제 값은 **전부 float** → 정밀/범위 부족. **유저가 ① 레지스터맵 `DATA_SENSOR_ADC_t` 타입 변경(int8→float 또는 int16+scale) ② MARSHAL 하네스(PRA_PERIPHERAL→reg.sensor_adc) 연결** 예정. Orin 이 ADC 값을 읽으려면 이 선결 필요(BMS CAN 과 독립). *cf. 메인 plan Step10 MARSHAL*

### Step 8-8 — 리비전 보드 최종 검증 (**프로젝트 종료**)
- **HSE 크리스털** 리비전 보드(§5-2)에서 **실 BMS CAN 송수신** 정상(SJW 워크어라운드 없이) / 3값 디코드 / **과열 등 전반 하드웨어 체크** → 프로젝트 마무리

---

## 4. 확정 사항 (2026-10-10 유저)

1. ✅ **TX 경로 = 메일박스 직접 송신**. bxCAN TX 메일박스 3개 = 요청 3개 → 동시 적재해도 소실 없음. `canTxQueue`·stale 로직 전부 제거
2. ✅ **폴링 타이밍 = 3 요청 동시 적재(간격 무시), 전체 ~1s 주기**(저율 로깅). **응답 타임아웃 = `delta_tick > 1000ms`(1 주기 무응답) → CAN recovery**
3. ✅ **파일 = `rd_can_bms` 단일**. 내부에 **CAN ISR(HAL 콜백) + `RD_TASK_CAN()`** 포함. main `Start_can1` 하네스는 **유저 수동 추가**
4. ✅ **0x98 = 순수 로깅, 전원 FSM 연계 금지**. 외부 서플라이 구동 등 미응답 상황이 PRA 에 영향 0 (격리 §0)
5. ✅ **RX 필터 = `0x18xx4001` mask**(ID `0x18004001` / mask `0xFF00FFFF`)

---

## 5. 리스크 / 메모

### 5-2. 🔁 반전 (2026-10-10, PCAN 에러로그 `raw/test.txt`) — **TX는 버스 도달, 비트 레벨 깨짐 (타이밍/클럭)**

§5-1의 "트랜시버 TX 사망" 결론을 **PCAN 에러로그가 뒤집음**. **칩 교체 보류.**

> ### ✅ 원인 확정 + 임시 해결 (2026-10-10)
> **SJW=4 양방향(PCAN 4 / STM 4) + SP 75% 로 변경하니 EOF 조차 안 나고 통신 정상.** → **오실레이터 허용오차(HSI)가 근본 원인 확정.** 재동기 헤드룸이 HSI ±1% 를 덮음.
> - **현재 STM 타이밍**(`MX_CAN1_Init`): Prescaler 10 / **BS1 11 / BS2 4 / SJW 4** → 250k 유지, **SP 75%**. 예산 0.31%→**~0.98%**(≈±1%, HSI 커버)
> - **결정(유저)**: 현재 SJW 워크어라운드로 **일단 진행**, **근본해결(HSE 크리스털)은 보드 리비전 시** 반영. 펌웨어 나머지 먼저 작성
> - ⚠ **실 BMS 캐비엇**: SJW↑는 **자기 수신** 마진만 늘림. PCAN 은 SJW=4 로 맞춰줘서 양방향 OK 였으나, **실 BMS 의 SJW 는 벤더 고정(미지)** → PRA 요청(HSI 오차)을 BMS 가 못 받으면 응답 없음. 실 BMS 연결 시 재확인 필요, **크리스털이 확실한 해결**
> - 펌웨어·트랜시버·MCU 전부 정상 — HW 클럭 소스만 추후 교체

- **관측**: PCAN 에러가 **~985ms 주기 버스트**(= PRA 1초 폴링 TX와 동기), 버스트당 2~3개(= 3요청 각각). PCAN 에러카운터(데이터 Byte2)가 **0x85(133, error-passive)까지 포화**
- **해석**: PRA 송신이 **버스를 거쳐 PCAN까지 실제 도달**("뭔가 들어옴") → **트랜시버 TX 정상**. 다만 **비트 레벨에서 깨져** PCAN 디코드 실패 = **타이밍/신호 품질** 문제(TX 단선·칩사망 아님). 앞선 "미도달/bus-off"는 PCAN 이 bus-off 로 로깅 멈춘 상태였던 것
- **클럭**: 설정상 250k 정확(SYSCLK 160M÷→APB1 40M, 10×16tq=250k, SP 87.5%). 단 **`OscillatorType=HSI`/`PLLSource=HSI`** — CAN 이 **내부 RC(±1%, 최대 ±2%)** 기반. 이 config 의 오실레이터 허용오차 예산 ≈ **0.31%**(SJW=1) → HSI 초과 가능. 수신은 재동기 흡수로 간헐 성공과 정합. ⚠ **단 ECU_V3 도 HSI 사용(CAN 동작)** → HSI 단정 불가, 아래로 가림
- **결정 측정**: 스코프로 CANH/CANL **1비트 폭 = 4.0µs 확인**(벗어나면 비트레이트 오차 확정) + dominant 레벨(CANH~3.5/CANL~1.5)·엣지 품질
- **빠른 진단**: `.ioc` CAN 타이밍 **BS1=11/BS2=4/SJW=4**(16tq=250k 유지, SP 75%)로 변경 → 허용오차 예산 0.31%→~1%. 에러 감소 시 **오실레이터 허용오차 원인 확정 → HSE(크리스털) 전환**이 근본해결(보드 HSE 유무 확인). 무변화 시 PCAN 설정(정확 250k·SP)·신호품질
- **다음**: ① 비트폭 스코프 ② SJW=4/SP75% 재테스트(**PRA+PCAN 양방향**) ③ PCAN 타이밍 확인 ④ 효과 시 HSE 전환
- **칩 교체·펌웨어 변경 불요** (펌웨어·트랜시버 모두 기능 정상, 문제는 비트타이밍 마진/클럭 소스)

#### 🔑 선례 corroboration (ECU HSI → 크리스털, 유저 2026-10-10)
- ECU_V3(HSI)는 **AK 모터**와 정상(AK가 EOF 에러에도 통신 유지·내부 수신 마진 큼 → HSI 오차 흡수) → "ECU 도 HSI 인데 CAN 됨" 역설 해소
- 같은 ECU가 **ODrive**(SJW 작음·마진 빡빡)에서 **EOF 발생** → HSI 오차 노출 → **크리스털 장착 신규버전으로 전량 해결**
- ⇒ **HSI(±1%)는 CAN 마진 부족**이 ECU 에서 이미 실증. 상대 노드/타이밍 마진에 따라 "무증상(AK)↔EOF(ODrive)↔전면 불통(PRA 현재)"로 발현. **PRA 도 HSE 크리스털이 확정적 해결** (SJW=4 에서 뭐라도 잡히면 원인 확정)
- **PRA 가 전면 불통인 이유**: SP 87.5%→Phase_Seg2=2tq(재동기 여유 작음, SJW≤2 제한) + PRA HSI + PCAN 오차 합산 → ODrive 때보다 예산 더 초과. SJW=4/SP75% 는 Phase_Seg2=4tq 확보로 헤드룸 최대화

### 5-1. 🔴 벤치 PHY 디버깅 (2026-10-10, Step 8-2 검증 중) — ~~트랜시버 TX 경로 HW 고장 결론~~ **(§5-2 에서 반전됨)**

증상 누적 → 단일 원인(PRA가 버스를 **dominant 로 구동 못 함** = CAN 트랜시버 TX 경로 고장)으로 수렴.

- **관측**: ① 8-1 1프레임 수신 OK + Form error 1 ② 8-2 송신 추가 시 즉시 에러 ③ `REQUEST_ALL` 주석(펌웨어 송신 0회)·종단 정상(60Ω)인데도 **PCAN 측 bus-off** ④ PRA ESR `0xFFF80057` = REC 255/TEC 248 포화, EWGF·EPVF, BOFF=0(bus-heavy), **LEC=5 Bit Dominant Error**
- **진단**: PCAN bus-off = PCAN TX 가 **ACK 를 못 받음** = 수신자 PRA 가 ACK(dominant)를 버스에 못 실음. LEC=5(dominant 송신→recessive 모니터)와 정확히 일치. 펌웨어는 송신 호출 0회였으므로 **ACK/에러플래그는 컨트롤러·트랜시버 자동 동작 → HW 문제**. 종단은 정상이라 배제
- **결론(갱신 2026-10-10)**: PRA CAN **수신 OK / dominant 송신 불가**. 트랜시버 = **ISO1042BDWVR**(절연형, **모드핀 없음** → 스탠바이 가설 폐기). ISO1042 TXD 내부 풀업 → **TXD 플로팅/미연결 시 항상 recessive = dominant 못 냄**, 증상과 일치. **RX 동작 = VCC1·VCC2·RXD·PA11 정상** → 고장은 **TX 반쪽(PA12→TXD)에 국한**. MCU 측 정상(250k@87.5%, Mode NORMAL)
  - ⇒ **1순위: PA12(CAN1_TX) ↔ ISO1042 1번핀 TXD 연결 단선/냉납/직렬저항 오픈** (이 보드 TX측 조립)
- ✅ **루프백 테스트로 MCU 측 확정(2026-10-10)**: `CAN_MODE_LOOPBACK` + `REQUEST_ALL` 활성 → **TSR=`0x1C030303`**(TXOK0/1/2=1 전부 송신성공, RQCP0/1/2=1, TME0/1/2=1 메일박스 비움) + **ESR=0**. ⇒ **MCU CAN 송신엔진·비트타이밍·`REQUEST_ALL` 코드 100% 정상.** 펌웨어 의심 종결
  - 루프백은 Tx핀도 실제 구동 → PRA 3프레임 송신+Tx핀 구동했으나 **PCAN 미수신** ⇒ 신호가 **버스까지 도달 못 함** = **PA12→ISO1042 TXD→CANH/CANL 경로 고장** 양방향 확정
  - (루프백 RX 미적재 "비어 보임"은 정상: 자기 요청 ID `0x18xx0140`가 필터 `0x18xx4001`에 걸러짐)
- **HW 체크 경과(2026-10-10)**: ✅ PA11/PA12 ↔ 트랜시버 RXD/TXD 도통 정상(멀티미터) ✅ VCC2=4.998V ✅ 솔더 정상(핀 상단 프로빙). → 쉬운 후보 전부 제거
- **남은 2개 후보**: ① **절연 버스측 접지 GND2(8번핀) ↔ CAN 커넥터 GND ↔ PCAN GND** 기준 — ISO1042 절연형이라 VCC2 생존(4.998V)만으론 버스측 공통모드 기준을 보장 못 함. GND2 미연결 시 **송·수신 모두 마진 불량**(= "RX 1회성"과 정합). **멀티미터로 GND2↔PCAN GND 도통 확인** ② 위 정상 시 **ISO1042 칩 손상(TX 드라이버)** → 칩 교체/여분 보드 교차
- ⚠ **DTO 주의**: ISO1042 TXD Dominant Timeout(~1.7–3.8ms) — PA12 정적 LOW 고정 테스트는 ~2ms 후 드라이버 자동해제로 recessive 오판. 정적 드라이브 테스트 금지, GND2 체크→칩 교체 순
- **영향 범위**: CAN 파서는 격리(로깅 전용)라 **PRA 본 기능·전원·리붓 무영향**. HW 수정 병렬 진행 가능. BMS 로깅은 TX 경로 복구 후 동작
- **펌웨어 상태**: Step 8-1/8-2 코드 = **TX 엔진까지 루프백 검증 완료, 변경 불요**. 실버스 TX 검증만 HW 복구 후 재개


- **격리 불변식(최우선)**: 어떤 CAN 경로도 `PRA_CONTROL.STATE`·전원 출력·`NVIC_SystemReset` 에 쓰지 않음. 리뷰 시 BMS CAN → 시스템 방향 의존 0 을 확인
- **AutoRetransmission DISABLE**: 요청/응답 1회성 — ~1s 폴링 라운드가 자연 재시도. `delta_tick>1000ms` 타임아웃 → CAN 재초기화가 신뢰성 핵심
- **BMS 미연결 시 요청 ACK 실패** → `ErrorCallback` 에러. `ActivateNotification` tick당 ≤1 재무장으로 IRQ 폭주 억제, recovery 루프가 흡수. **시스템 영향 없음**(escalation 미이식)
- **bxCAN 필터 뱅크**: 32-bit mask 모드(확장 ID). MX 기본 필터 없음 → `rd_can_bms`/USER CODE 구성 필수
- **reg 경합**: `RD_CAN_BMS_UPDATE` 의 SENSOR/BMS 쓰기는 `ResmapMutex` 등 기존 발행 규약 준수(rs485 READ 와 경합 보호)
- **시계 스케일**: `HW_NowTick`=ms(TIM5 부재). `realtime_tick`/`delta_tick` ×0.1ms 스케일 불일치는 프로젝트 공통 미결 계승
- **MX 재생성 안전**: 필터/Start/콜백/`RD_TASK_CAN` 호출은 전부 `rd_can_bms.c` 또는 USER CODE 구간에 두어 보존

---

## 6. 진행 로그

| 날짜 | Step | 내용 |
|------|------|------|
| 2026-10-10 | — | 계획 문서 신설. ECU CAN 구조 조사 기반 Step 8-1~8-7 수립 |
| 2026-10-10 | — | §4 결정 5건 확정 반영 + **격리 원칙**(CAN↔시스템 무연계) 명문화. 메일박스 직접송신·~1s 폴링·`delta_tick>1000ms`→CAN recovery·단일파일(ISR+`RD_TASK_CAN`)·0x98 순수로깅·`0x18xx4001` mask |
| 2026-10-10 | **8-1** | ✅ **구현**: `rd_can_bms.{h,c}` 신설 — 필터(0x18xx4001 mask, bank0)·자급식 IRQ(CAN1_RX0/SCE + NVIC enable, .ioc OFF 유지)·Start·Notification·RX 콜백(`BMS_rx_count`/`BMS_last_rx`)·Error 콜백·`HW_NowTick`·`RD_TASK_CAN`(INIT+idle). main 하네스 유저 수동 |
| 2026-10-10 | **8-1** | ✅ **실기 검증 통과**: `18904001` 송신 → rx_count=1, ExtId·data 정상. Form error(0x10) 1건은 LEC 단발 트랜션트(수신 정상) → **`CAN_IT_LAST_ERROR_CODE` 구독 제거**로 오탐 억제. 재빌드 성공 확인 |
| 2026-10-10 | **8-2** | ✅ **구현**: `BMS_Handle_t`(전역 `BMS`, msg_vic/temp/fault raw+valid+last_rx_tick) · `CAN_BMS_REQUEST_ALL`(메일박스 3 직송) · `CAN_BMS_RX_POP`/`RX_APPLY`(데이터ID 분기 적재+ts latch) · RX 콜백 pop→apply 전환 · `RD_TASK_CAN` ~1s 폴링 |
| 2026-10-10 | **8-2** | 🔴 **PHY 이슈(펌웨어 무관)**: 벤치 테스트서 송수신 불안정. 아래 §5-1 참조 — **PRA CAN 트랜시버 TX(dominant 구동) 경로 HW 고장**으로 결론. 펌웨어는 수신 범위 내 정상 |
| 2026-10-10 | **8-2** | ✅ **루프백으로 MCU 확정**: TSR=0x1C030303(TXOK×3)+ESR=0 → MCU 송신엔진·코드 정상. 고장 = **PA12→ISO1042 TXD→버스 경로**(§5-1). 펌웨어 TX 엔진까지 검증 완료. HW 복구 후 실버스 재검증 |
| 2026-10-10 | **8-2** | 🔧 **인터럽트 소유 전환(ECU_V3 정합·변수 배제)**: 유저가 .ioc 에서 CAN1 RX0/SCE 활성+재생성 → it.c 핸들러·MspInit NVIC(prio5) 생성됨. `rd_can_bms.c` 의 **자작 핸들러 2개 + INIT NVIC enable + BMS_CAN_IRQ_PRIO 제거**(중복정의 방지), HAL weak 콜백만 유지. 절연 버스접지 정상 확인 → 유일 가능성=ISO1042 칩 손상 |
| 2026-10-10 | **8-2** | 🔬 **PHY 최종 분기 — 스코프 예정**: PA12(CAN1_TX) vs CANH/CANL 관측 (이후 PCAN 에러로그로 재해석됨 ↓) |
| 2026-10-10 | **8-2** | 🔁 **반전(PCAN 에러로그 raw/test.txt)**: 에러가 PRA 1초 TX 와 동기 버스트 → **TX 버스 도달·비트레벨 깨짐**(트랜시버 정상, 칩 교체 보류). 원인=타이밍/클럭(HSI ±1% vs 예산 0.31%). §5-2 |
| 2026-10-10 | **8-2** | ✅ **원인 확정·임시해결**: SJW=4 양방향(STM BS1 11/BS2 4/SJW 4, SP 75%, 250k 유지)→EOF 無·통신 정상 = **HSI 오실레이터 허용오차 확정**. 결정=SJW 워크어라운드로 진행, HSE 크리스털은 보드 리비전. ⚠실 BMS SJW(벤더고정)는 재확인 필요. 펌웨어/트랜시버/MCU 정상 |
| 2026-10-10 | **8-3** | ✅ **구현**: `RD_CAN_BMS_RECOVERY`(Stop→DeInit→Init→INIT, CAN 서브시스템만) + `RD_TASK_CAN` 자기완결 복구 트리거((now−last_rx_tick)>1500ms ‖ HAL 에러→RECOVERY, 격리). INIT last_rx_tick=now 초기화 |
| 2026-10-10 | **8-4** | ✅ **구현**: `RD_CAN_BMS_UPDATE`(raw snapshot→big-endian 디코드→reg.sensor_bms, taskENTER_CRITICAL). current=(int16)(raw−30000) 오프셋 제거, max_cell_temp=raw(40 offset 보존, uint8). **정합 수정**: §6/§7 "온도 raw−40"→"raw 보존"(uint8 음수 불가) |
| 2026-10-10 | **8-4** | ✅ **실기 검증 통과**: reg.sensor_bms = voltage 538(53.8V)/soc 680(68%)/max_cell_temp 64(−40=23°C) 정상 |
| 2026-10-10 | **8-5·8-6** | ✅ **구현**: `RD_CAN_BMS_CHECKER`(신선도+HAL에러→STATE_t→addr61 state 발행, `BMS_ch_state`) + delta_tick(addr60, ms 0xFF 포화) 발행. 로깅 전용·시스템 무전파. **hw_reset 미구현**(자동복구라 불요), 0x98↔전원 미연계. **CAN 코드 대부분 close** |
| 2026-10-10 | **Step 재구성** | 8-7=Orin 연동 실테스트(Orin 준비 후), **8-8=HSE 리비전 보드 + 실 BMS 송수신 + 과열 등 전반 체크 → 프로젝트 종료**. 금일 작업 마침 |
