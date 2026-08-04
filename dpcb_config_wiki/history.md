# DPC_B 작업 히스토리

> 최종 갱신: 2026-08-03 (Session 21 — CONSUME 1회성 소비+mask 폐기 확정)  
> 상위: [dpcb_overview.md](dpcb_overview.md)  
> 진행 상태 요약: [plan.md](plan.md)

---

## [Session 4] Step 4 — 통신부 리팩터링

- `rd_uart.h` `UART_Ring_t`에 `wake_task (osThreadId_t)` 필드 추가 (`#ifdef RTOS_IS_AVAILABLE`)
- `rd_uart.c` IDLE ISR: `USART6` 하드코딩 → `wake_task != NULL` 분기 범용화
- `stm32f4xx_it.c` USART2_IRQHandler 활성화 (TC→`RD_RS485_IRQ_HANDLER`, IDLE→`RD_UART_IDLE_HANDLER`)
- `rd_system.h` extern 정비: `DPCB_rs485`, `huart2`, `periTaskHandle` 추가
- `rd_system.c` `RD_TASK_PERI`/`RD_TASK_RS485`: wake_task 주입
- 결과: USART6 IDLE → periTask, USART2 IDLE → rs485Task 독립 라우팅 확인

---

## [Session 9] Step 2 — rd_register_dpcb.h 작성

- 파일 생성: `stm_ws/DPC_B/Core/Inc/rd_register_dpcb.h`
- `Modbus_MAP_DPCB_v2.csv` 기반 256-byte `REGISTER_t` packed struct 정의
- 포함 요소:
  - 영역 offset/size `#define`
  - `SYS_WRITE_LOCK/UNLOCK`, `HW_BIT_UART2/4/6/I2C1`
  - `DEF_*` 기본값 (ERR_TIMEOUT=15, FATAL_TIMEOUT=10, ERR_CNT=5, FATAL_CNT=10, GOAL_CURRENT=750)
  - 모드/서보/deploy_fsm 상수
  - sub-struct typedef 8종 + `REGISTER_t` master struct
- 의존성: `rd_common.h`(`STATE_t`)만 include

---

## [Session 10] Step 5 — Orin RS485 modbus 통합

**신규 생성:**
- `rd_comm_orin.h/.c` — Orin RS485 패킷 레이어
  - `ORIN_*` 접두사 (DPCA 4-byte 패킷과 타입명 충돌 방지)
  - `ORIN_MY_ID = 0xE2`, `ORIN_MASTER_ID = 0x01` *(→ 2026-08-03 `0xD1` 로 변경, Session 20 참조)*
  - CRC-16/IBM, Header(0xAA/0x55)+Length+Instruction+CRC
  - `RD_ORIN_READ` / `RD_ORIN_HANDLE` / `RD_ORIN_WRITE`
- `rd_map_dpcb.h/.c` — 레지스터 맵 디스패치 + 마샬 레이어
  - `REGISTER_t reg` 전역 인스턴스 (256B 단일 source-of-truth)
  - Region LUT 15개 영역 (`REG_ACC_R/W/RW` + `needs_unlock`)
  - `RD_MAP_DISPATCH_WRITE/READ`
  - `RD_MAP_MARSHAL_PUBLISH/CONSUME`: **stub (Step 6 예정)**

**기존 수정:**
- `rd_system.h`: include + `ORIN_PACKET`, `payload_state` extern 추가
- `rd_system.c`: `ORIN_PACKET` 정의, `RD_SYSTEM_INIT` MAP/ORIN INIT 추가, `RD_TASK_RS485` 완성

---

## [Session 11] Step 7 — RD_TASK_SYSTEM + MARSHAL SYSTEM 영역 (2026-06-25)

- `rd_system.h`: `tim_cnt`, `hw` extern 선언 추가
- `rd_system.c`:
  - `RD_SYSTEM_CHECKER()` (static): 3채널 Checker+Recovery 디스패치
  - `ACTION_STATE_FAULT()` (static): 모터 토크 OFF → `#ifndef RS485_TEST_ON` 가드 재부팅
  - `RD_TASK_SYSTEM()`: 10ms `osDelayUntil` — HW_RESET_HANDLE → CHECKER → EVALUATE_STATE → FAULT 디스패치 → MARSHAL_PUBLISH
- `rd_map_dpcb.c`: `RD_MAP_MARSHAL_PUBLISH` SYSTEM 영역 구현 (sys_state, realtime_tick, hw.*, degraded_cnt)
- **빌드 이슈**: `rd_map_dpcb.c`에서 `tim_cnt`/`hw` 미정의 컴파일 에러 → `rd_system.h`에 extern 추가로 해결
- **동작 확인**: Dynamixel RS485 케이블 뽑기 → `hw.error.raw = 0x04`(bit2=uart6) → LS_OFFLINE → RECOVERY → `0x00`/LS_READY → 재삽입 → Dynamixel 재탐지 확인

---

## [Session 12] 문서 재정리 (2026-07-03)

- 단일 `dpcb_config_wiki.md` → `dpcb_config_wiki/` 디렉토리 8개 문서로 분할
  - overview / opmode / peri / task / register / checker / plan / history
- 기존 `dpcb_config_wiki.md` 원본 유지
- `memo_26024.md` 기반 mode/deploy_fsm 용도 정합성 반영
- 문서 서술 규칙: 명사형 종결·개조식 적용

---

## [Session 13] 레지스터맵 개정 + Step 재정렬 + PUBLISH 사전검토 (2026-07-03)

**opmode 갱신 (`memo_26024.md` 57줄 이후 최신 반영):**
- mode 3종→4종: `0 MANUAL`(구 IDLE) / `1 HOLD` / `2 AUTO` / **`3 CTRL`**(Orin 원격 제어)
- mode별 레지스터 Write 권한 표 신설, mode=2 시 Orin↔STM deploy_fsm 핸드셰이크 구조 추가

**Step 재정렬 (`plan.md`):**
- Step 3(엑셀 교차검증) → 사용자 검토 완료
- Step 6 ↔ 7 교체: Step 6 = Checker/reset, Step 7 = MARSHAL 본체
- Step 8 신설: mode에 따른 기존 시스템 변경 및 통합

**레지스터맵 개정 (`rd_register_dpcb.h`):**
- MOTOR/data 데이터시트 정정: `present_position`·`present_velocity` `int16_t[3]`→`int32_t[3]`(+12 byte, 멀티턴 오버플로우 해소), `present_temperature` `int16_t[3]`→`uint8_t[3]`(−3 byte). 순증 +9 byte(28→37 byte), RSVD1(18→9 byte)에서 흡수, addr 120 이후 불변
- DIAG 영역 구조 명확화: `cmd_write_tick`(4B 카운터) + `diag_rsvd[28]`(DIAG_RSVD). 예약필드 `reserved`→`diag_rsvd` 개명
- SPDT 스위치 주석 통일(코드 기준): `ex_sw[2~5]` = `0=IDLE(mid) / 1=UP / 2=DOWN`

**MARSHAL_PUBLISH 사전검토:**
- Read 단방향이라 Step 8 선행 가능 확인. 매핑 전 항목 확정(스케일 직접 복사, MOT[0/1/2]=ID2/3/4)
- 미확정 2건 해소: ① position int16 오버플로우 → int32 확장, ② SPDT 인코딩 불일치 → 코드 기준 통일

---

## [Session 14] MARSHAL_PUBLISH 구현 (2026-07-03)

- `rd_map_dpcb.c` `RD_MAP_MARSHAL_PUBLISH` 본체 작성 — SYSTEM 영역 외 나머지 발행 완료
  - MOTOR 피드백(position/velocity/current/temperature/hardware_error + uart6_state), 채널 상태(uart2/uart4), 센서(prox/lock), 패널(ex_sw[0~5])
  - 방식: CRITICAL 진입 전 전체 스냅샷 → 단일 `taskENTER_CRITICAL()` 일괄 기입. `p == NULL` 가드(모터/센서/패널 스킵, SYSTEM·채널은 발행)
  - 전부 직접 복사(스케일 변환 없음), `hardware_error`는 `ram.state` 아닌 `dyn_ctrl` 필드
- **미구현(TODO 주석 표기)**: `degraded_cnt[3]`(i2c 오염도), `sensor_dpcb.panel_state`(i2c 채널 상태, addr 73) — i2c Checker 미구현이 원인, 0 유지
- 파일 상단/구획/CONSUME 주석의 Step 표기 현행화(PUBLISH 완료, CONSUME=Step 7 잔여)
- 동작 확인: 레지스터맵에 모터 위치/속도, 패널 스위치 등 정상 반영 확인

---

## [Session 15] mode + sys_state 통합 구조 확정 (2026-07-04, 문서만 / 코드 보류)

> Orin 통합 과정의 시스템 구성 변경. Session 13 의 4모드(MANUAL/HOLD/AUTO/CTRL)+deploy_fsm 구조를 **2모드 + sys_state** 로 대체(supersede).

- **구조**: `mode(126)` 2종(0=MANUAL 패널 / 1=AUTO Orin관제) + `sys_state(57, R/O)` 운용상태 + `sys_state_target(127, R/W)` 목표(구 deploy_fsm 자리)
- **`DPCB_STATE_e` 신규**: CTRL=0 / HOLD=1 / FSM 2~8(구 deploy_fsm INIT~FINISH) / RSVD 9 / ERROR 10. 필드는 uint8_t 유지, enum 은 값 정의용
- **sys_state_target 입력 mask** `{0,1,2,6}`: CTRL/HOLD/FSM개시(INIT)/상승개시(ASCEND_1)만 Orin 기입 허용 (중간단계는 STM 내부 전이)
- **핸드셰이크**: target=2→STM 2~5 자동→WAIT(5) 대기→target=6→6~8→FINISH(8) 후 CTRL(0) 자동 복귀
- **상태 2축 분리 (B 토론 결정)**:
  - addr 57 을 `SYSTEM_STATE_e`→`DPCB_STATE_e` repurpose. 기존 `SYSTEM_STATE_e`(payload_state)는 내부 전용 유지, SYSTEM 영역 발행 중단
  - FAULT 노출 = `hw_fatal(55)` 단독 / 통신 FAULT 시 `DPCB_STATE`→ERROR(10) 강제 전이 (FAULT→MCU 재부팅으로 초기값 복귀하므로 큰 문제 없음)
  - 운용 실제값 `DPC_CTL.STATE`→`DPCB_STATE_e` 통일, 부팅 기본 CTRL(0)
- **교차검증**: addr 127 `deploy_fsm`(uint8_t)→`sys_state_target` 순수 리네이밍 확인(참조처 주석뿐). `SYSTEM_STATE_e` 사용처 5곳 확인(실제 set 은 INIT/FAULT 뿐, `mtr_lock`(`rd_system.c:329`) 상시 잠금 상태)
- **반영 문서**: opmode(전면 개정), register(§2-1 sys_state·§3-1), plan(Step 8 TODO 6항목), task/overview(개정 목표 주석)
- **코드 미착수**: 사용자 wiki 확인 후 개정 착수 예정

---

## [Session 16] 코드 개정 착수 (1) — addr 127 리네이밍 + enum 신설 (2026-07-07)

> Step 8 코드 TODO 중 저영향 2건 반영 (요약 확인 후 진행). 나머지(발행 소스/FAULT 연동/Write 권한/제어 통합)는 잔여.

- `rd_register_dpcb.h`:
  - **enum 신설**: `DPCB_MODE_e`(MANUAL=0/AUTO=1), `DPCB_STATE_e`(CTRL=0/HOLD=1/INIT2·DESCEND_1 3·DESCEND_2 4·WAIT5·ASCEND_1 6·ASCEND_2 7·FINISH8/RSVD9/ERROR10) — 구 `MODE_IDLE/HOLD/AUTO`·`DEPLOY_*`(99=ERROR) #define 제거
  - **필드는 uint8_t 유지** (레지스터 packing 보존, enum 은 값 정의용)
  - `CMD_DPCB_t` addr 127 `deploy_fsm` → **`sys_state_target`** 리네이밍, addr 126 `mode` 주석 → DPCB_MODE_e
  - addr 57 `sys_state` 주석 → `DPCB_STATE_e` (발행 소스 교체는 잔여 TODO 로 명시)
- `rd_map_dpcb.c:275` / `rd_map_dpcb.h:85`: CONSUME 스텁 주석의 `deploy_fsm` → `sys_state_target`
- **로직 영향 없음**: 구 상수·필드가 실제 로직 미사용이었음(정의·주석뿐). 값 변경 주의: 구 `DEPLOY_ERROR=99` → `DPCB_STATE_ERROR=10`
- 컴파일 검증은 STM32CubeIDE 에서 사용자 수행 예정(이 환경 ARM 툴체인 없음)

---

## [Session 17] 코드 개정 (2) — addr 57 발행 소스 교체 (2026-07-07)

- `rd_map_dpcb.c` `RD_MAP_MARSHAL_PUBLISH`: `reg.sys.sys_state` 발행 소스를 `payload_state`(SYSTEM_STATE_e) → **`DPC_CTL.STATE`(DPCB_STATE_e)** 단방향 복사로 교체
- 파일 상단 include 주석 `payload_state`→`DPC_CTL` 로 갱신 (rd_map_dpcb.c 에서 payload_state 미참조화)
- `DPC_CTL.STATE`: `volatile uint8_t`(rd_control.h:52), `rd_system.h:84` extern → 접근 OK
- **주의(과도기)**: 현재 `DPC_CTL.STATE` 값 체계(실제 .c: 0=manual/1=INIT/2=DESCEND_1/3=DESCEND_2/4=WAIT/5=ASCEND_1/6=ASCEND_2/7=FINISH/10=ERROR)는 `DPCB_STATE_e` 번호와 아직 불일치 → 값 정합은 `RD_CONTROL_LOOP` 통합 TODO(plan §3-3)에서 완성. 이번 작업은 발행 소스(변수) 교체에 한정. (구 보고의 압축형 0~4 는 `rd_control.h:52` 헤더 주석 오류 기반이었음 — 정정)
- `payload_state`(SYSTEM_STATE_e)는 내부 FAULT 처리 전용으로 유지(발행 중단 완료)

---

## [Session 18] §3-3 미결 질문 Q1~Q7 close (2026-07-21, `memo_26024.md`)

- `memo_26024.md` A1~A7 을 plan §3-3(c) Q1~Q7 에 매핑·확정 (문서만, 코드 미착수)
- 결정 요지: 핵심 case 함수는 **스텁만 생성·본체 미구현** (사용자 추후 작성)
  - Q1/A1 `RD_CONTROL_CASE_HOLD` 스텁 (CON_DATA==0x0F → present_pos 1회 저장 → pos 고정)
  - Q2/A2 `RD_CONTROL_CASE_CTRL` 스텁, 패널제어=mode0, CASE_IDLE→mode0 이관은 추후(신규 TODO)
  - Q3/A3 CONSUME(`sys_state_target`→STATE) 본체 미구현 유지
  - Q4/Q6·A4/A6 진입/WAIT 트리거 = 로컬 SW1 0.5초↑ → target=2 / target=6 (Orin 직접 경로는 TODO)
  - Q5/A5 ERROR 자동복귀 제거·함수 공란 (Orin target=0 강제)
  - Q7/A7 WAIT 타임아웃 구현 제외 → 토론 이관
- 반영 파일: `plan.md`(§3-3(c), §3-4 신설), `dpcb_opmode.md`(§2 라우팅, §3-1 WAIT, §5 트리거, §7-2 ERROR/타임아웃)
- **추가 검토 3건 확정 (2026-07-21)**:
  - 명칭: addr 127 = **`sys_state_target` 유지 확정** (memo `sys_state_fsm` 제안 폐기, 이미 리네이밍 완료된 코드 회귀 방지)
  - 트리거 경로: 로컬 SW1 → **`DPC_CTL.STATE` 직접 전이**(레지스터 우회). Orin 은 `sys_state_target`→CONSUME. CONSUME 구현 시 mask 충돌은 사용자 추후 해결(TODO)
  - STATE=0 라우팅: `mode=0`→`CASE_IDLE`(패널), `mode=1`→`CASE_CTRL`(미구현 스텁)

---

## [Session 19] `rd_control.c` mode+STATE 통합 구현 + 코드 검토 (2026-07-21)

사용자가 CONSUME 제외하고 `rd_control.c/.h` 를 mode+STATE 로 구현 → 클로드 검토(코드 미변경) → 사용자 수정 반복.

- **구현 확정 (Session 18 의 일부 잠정안을 실코드 기준으로 정정)**:
  - **패널 버튼**: **SW1 = mode 토글**, **SW2 = FSM 진입/전이**(mode=1). CTRL/HOLD 에서 SW2 길게(>SW_HIGH)→INIT(2), WAIT 에서 SW2 짧게→ASCEND_1(6)·길게→CTRL(0). *(Session 18 의 "SW1 0.5초" 는 SW2 로 정정)*
  - **트리거**: SW2 → `DPC_CTL.STATE` 직접 전이(레지스터 우회). Orin `sys_state_target`→CONSUME 경로는 미구현
  - **mode 라우팅**: `rd_control.c:95/111` `if/else if` — mode0→`CASE_IDLE`, mode1→STATE switch
  - **enum**: 전이/ERROR 모두 `DPCB_STATE_*` 로 치환, `ERROR_STATE` #define 제거. (case 라벨 리터럴·일부 `=0` 잔존은 경미)
  - **센서 마스크 상수화** (`rd_control.h`): PROX **접촉=1** → DESCEND_2=`A_PROX_ALL_ON`(0x07)/ASCEND_1=`A_PROX_ALL_OFF`(0x00), CON **잠금=1** → DESCEND_1=`CONT_ALL_UNLOCK`(0x00)/ASCEND_2=`CONT_ALL_LOCK`(0xF0). 상세 [dpcb_opmode.md](dpcb_opmode.md) §3-1
- **검토로 발견·수정된 결함**:
  - ERROR 자동복귀(`CTL->STATE=0`) → 주석처리 (A5)
  - DESCEND_2 `avr_pos` 누산 오류 + 미초기화(UB) → `=0` 초기화·`+=` 수정
  - INIT `sum` int16 오버플로 → `int32_t`
  - **PROX 마스크 리팩터 회귀**(DESCEND/ASCEND 가 같은 상수로 충돌) → 폴라리티 확정(접촉=1) 후 ON/OFF 분리
  - case 라벨 직하 선언 → 각 case `{ }` 블록화
  - CON_DATA 전체잠금 = **0xF0**(bit7~4), memo 의 0x0F 오기 정정
- **의도적 미구현 (설계 확정)**:
  - `CASE_CTRL`(mode1 CTRL): 원격제어 미포함 버전이라 **빈 함수가 정상**. Orin failsafe 진입점(차기)
  - `CASE_HOLD`: 장력 pos 기반 위치홀드(CASE_INIT 유사), TODO
  - **CONSUME 보류 사유**: Orin 무관하게 레지스터맵 베이스+신 mode/FSM 정상동작 1차 검증 후 방향 결정
- 반영 파일: `dpcb_opmode.md`(§2 CTRL 의도, §3-1 센서 폴라리티, §5 SW2), `plan.md`(§2 Step8, §3-1 CONSUME 사유, §3-3/§3-4)

---

## [Session 20] 공동작업자(limtay) RS485/Orin 통신 push 반영 (2026-08-03, commit `65e9d41`)

협업자 펌웨어 커밋 `FIX: DPC-B - Multi-seg + buffer 256/272` 수신 → 펌웨어↔wiki 정합 점검 → 불일치/신규 5건 wiki 반영. 대상 파일: `rd_comm_orin.*`, `rd_uart.*`, `rd_system.*`, `rd_map_dpcb.c`.

- **노드 ID `0xE2`→`0xD1`** (`ORIN_MY_ID`): Orin 브리지 `PacketID::DPC_B`(`rd_comm.hpp`) 와 짝맞춤 필수 — 불일치 시 ID 필터에서 조용히 폐기(에러 카운터 미상승 "무응답"). 반영: `register.md` §5, 본 로그 Session 10 inline 노트
- **버퍼 확장(ECU_V3 정렬)**: `ORIN_DATA_BUF_SIZE` 90→256 / `RX_BUFFER_SIZE` 128→256 / `TX_BUFFER_SIZE` 128→272. 공유 버스라 DMA 링에 ECU 앞 132B 응답까지 유입 → 구 128B 초과 시 `rx_length=132%128=4` 쓰레기 길이 → FRAMING 100Hz 누적 → degraded 포화 → LS_OFFLINE→RECOVERY 근본원인. 반영: `register.md` §5
- **멀티세그먼트 READ**(`RD_ORIN_HANDLE`): `[Addr,Len]×n` (4×n B), n=1 하위호환, 응답 `Err(1)+seg 연접`. `RW(0x04)` 의도적 미지원. 세그먼트 간 원자성 미보장. 반영: `register.md` §5
- **request-synchronous 발행**: rs485Task 가 요청 직전 `RD_MAP_MARSHAL_PUBLISH` 추가 호출 → 발행/요청 주기 비트로 인한 중복·스테일 샘플 제거. `osThreadFlagsWait` 무한→10ms timeout(폴링 자기치유). 반영: `register.md` §3-1, `task.md` §1 표·§2·§3-1
- **견고화(구현 상세)**: WRITE 조기이탈 tx 채움 버그 수정 / UART CR1 RMW 원자화(`uart_crit_*`, RE 비트 lost-update 로 인한 영구 수신정지 방지) / `RD_UART_INIT` memset 후 `wake_task` 보존(RECOVERY 재호출 시 채널 사망 방지) / INIT 실패 `Error_Handler`→재시도+10회 시 리셋(`rs485_init_fail_cnt`) / REBOOT TX 완료 대기 후 리셋. 반영: `task.md` §3-1·§4, `register.md` §4
- **레지스터 맵**: `rd_map_dpcb.c` 주석 MOTOR/data 74~101→74~110·RSVD1→111~119 = **wiki(2026-07-03 개정)에 이미 반영된 값** → 펌웨어가 따라옴, 정합. 변경 불필요
- **미검증 크로스레포 의존**: ①ID 0xD1 ②버퍼 256 ③멀티세그 포맷 3건 모두 Orin 브리지(`rd_comm.hpp`/`RdMap::EncodeNode`) 대응 필요 — 이 커밋엔 `orin_ws` 변경 없음. **Orin 측 반영 여부 확인 잔여(TODO)**

---

## [Session 21] CONSUME → ctl.state 방향성 확정 — 1회성 소비 + mask 폐기 (2026-08-03, 토론)

§3-3 잔여 TODO 였던 CONSUME 의 `sys_state_target`→`DPC_CTL.STATE` 반영 방향성 확정. **코드는 사용자 작성 예정(미착수), 본 세션은 설계 결정 wiki 반영만.**

- **핵심 결정 = 1회성(one-shot) 소비**: `if (sys_state_target != 0xFF) { STATE = target; target = 0xFF; }`. 매 consume 마다 target 을 대입하면 FSM 자기전이를 계속 되돌리는 문제 → 1회 반영 후 0xFF sentinel 로 추가 소비 차단해 해소. single-producer(rs485Task)/single-consumer(controlTask), STATE write 는 controlTask 단독 유지(race 없음)
- **입력 mask 폐기 (자유 접근)**: 구 설계의 {0,1,2,6} 제한 제거 → Orin 이 모든 `DPCB_STATE_e` 자유 기입. 사유: ①1회성이 "실제>목표" 복사 문제를 mask 없이 해소 ②estop형 강제 정지(target=0/10) ③중도실패 시 특정 FSM 스텝 재주입. **시퀀스 합법성 책임 STM→Orin 이전**
  - ⚠ 비용: STM 이 전이 합법성 미보장 → 모든 전이쌍 물리 안전(각 case 진입부 이전상태 모터 정리)이 전제. CTRL/HOLD 스텁 구현 시 필수
- **mode 소유권 = 패널+Orin 공유**: `reg.cmd_dpcb.mode` 도 동일 1회성 target. Orin 절대값 / 패널 SW1 토글(`!MODE`) 기입. 통신·버튼 동시 랜덤성 감수
- **atomicity 불필요**: read-clear 태스크 간 비원자지만 STATE 미러(addr57) 폐루프로 Orin 재전송 자기치유
- **부팅 기본값 TODO**: `reg` zero-init → `sys_state_target`/`mode` = 0x00 유령소비(현재 무해) → 추후 0xFF 초기화
- **배선 TODO**: `RD_MAP_MARSHAL_CONSUME` 현재 호출부 없음 → controlTask·`RD_CONTROL_LOOP` 직전 배선 예정(타 영역 정상동작 검증 후)
- 반영 파일: `dpcb_opmode.md`(§4 전면 개정·§5-1), `plan.md`(§3-1·§3-2·§3-3 Q3/Q4·§3-4), `dpcb_register.md`(§3-2)
