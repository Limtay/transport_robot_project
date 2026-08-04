# DPC_B 구현 플랜 및 진행 상태

> 최종 갱신: 2026-08-03 (CONSUME 1회성 소비 + 입력 mask 폐기 확정 — §3-1, opmode §4)  
> 상위: [dpcb_overview.md](dpcb_overview.md)  
> 완료 이력 상세: [history.md](history.md)

---

## 1. 구현 단계(Step) 정의

| Step | 내용 | 담당 |
|------|------|------|
| 1 | 엑셀 레지스터 맵 작성 | 사용자 |
| 2 | 엑셀 기반 `rd_register_dpcb.h` 작성 (ECU_V3 참고) | 클로드 |
| 3 | 레지스터 맵 ↔ 엑셀 교차 검증 | 사용자 |
| 4 | 통신부 리팩터링 (DPCA/Orin RS485 명칭 분리, wake_task 구조) | 클로드 |
| 5 | modbus 시스템 통합 (DISPATCH + MARSHAL 스텁) | 클로드 |
| 6 | rd_system.c Checker / reset 구현 (ECU_V3 참고) | 클로드 |
| 7 | MARSHAL_PUBLISH/CONSUME 본체 작성 | 사용자 작성, 클로드 검수 |
| 8 | mode에 따른 기존 시스템 변경 및 통합 | 클로드 |

---

## 2. 진행 상태 (2026-07-03 기준)

| 항목                                                 | 상태                                                                                                   |
| -------------------------------------------------- | ---------------------------------------------------------------------------------------------------- |
| Step 1: 엑셀 레지스터 맵                                  | 완료                                                                                                   |
| Step 2: rd_register_dpcb.h                         | 완료                                                                                                   |
| Step 3: 엑셀 교차 검증                                   | 사용자 검토 완료                                                                                            |
| Step 4: 통신부 리팩터링                                   | 완료                                                                                                   |
| Step 5: Orin RS485 modbus 통합                       | 완료                                                                                                   |
| Step 6: RD_TASK_SYSTEM checker + MARSHAL SYSTEM 영역 | 완료 (동작 확인)                                                                                           |
| Step 7: MARSHAL 나머지 영역                             | PUBLISH 구현 완료 / CONSUME **예정 — 사용자 작성**                                                              |
| Step 8: mode에 따른 기존 시스템 변경 및 통합                    | **진행 중** — `rd_control.c` mode+STATE 통합 구현·검토 완료(2026-07-21). CTRL/HOLD/CONSUME 는 의도적 미구현(§3-1·§3-4) |
| Dynamixel 통신                                       | 동작 확인 완료                                                                                             |
| DPC-A 통신                                           | 동작 확인 완료                                                                                             |
| MCP23017 패널                                        | 동작 확인 완료                                                                                             |
| Orin RS485 (USART2)                                | 동작 확인 완료                                                                                             |

---

## 2-1. 완성까지 남은 경로 (2026-07-21 확정)

**현 상태**: Orin RO 읽기(PUBLISH) 전량 가능 + 패널 기반 FSM·수동 페리페럴 제어 가능. Orin 제어(CONSUME)는 미구현 → `LIGHT_EN`·`BOOT_EN` 2종만 패널 제어 불가. 제어 가능 범위 상세: [dpcb_task.md](dpcb_task.md) §5-3.

**완성 시나리오 (2단계)**:
1. **실기 검증** — 현 빌드를 실제 시스템에 물려 **패널 기반 FSM 구동 + 수동 페리페럴 제어가 정상 동작**함을 확인. (Orin 무관 독립 동작 1차 검증 — CONSUME 보류 사유, §3-1)
2. **CONSUME + 부가기능 매칭** — 위 검증 통과 후, `RD_MAP_MARSHAL_CONSUME` 본체 작성 + `LIGHT_EN`/`BOOT_EN` 등 부가기능 레지스터 연결만 진행하면 **완성**. (모든 Orin 제어의 단일 미싱링크 = CONSUME)

> 즉 남은 관문은 **①실기 정상동작 확인 → ②CONSUME/부가기능 매칭** 둘 뿐.

---

## 3. 잔여 작업

### 3-1. Step 7 — MARSHAL 본체

- `RD_MAP_MARSHAL_PUBLISH`: **구현 완료** — 모터 피드백 / 채널 상태 / 센서·패널 발행 (스냅샷 → 단일 CRITICAL 일괄 기입)
  - 미구현(TODO): `degraded_cnt[3]`(i2c 오염도), `sensor_dpcb.panel_state`(i2c 채널 상태) — i2c Checker 미구현이 원인, 0 유지
- `RD_MAP_MARSHAL_CONSUME`: **예정(사용자 작성, 클로드 검수)** — cmd_dpca / cmd_dpcb / cmd_mot → PERIPHERAL 적용
  - **CONSUME 보류 사유 (2026-07-21 확정)**: 1차 목표는 **Orin 관여 없이도** ①레지스터맵 베이스 구성 + ②개정된 mode/FSM 시스템이 기존처럼 정상 동작하는지 검증. 이 검증 후에 CONSUME 방향성(또는 lock 기반 제어)을 결정. 따라서 현 펌웨어 버전은 CONSUME 미구현이 정상.
  - **CONSUME 방향성 확정 (2026-08-03 토론)**: sys_state 방향 이슈 해결 = **1회성 소비 + 입력 mask 폐기**. `sys_state_target != 0xFF` 일 때만 `DPC_CTL.STATE` 반영 후 0xFF 클리어 → FSM 자기전이 미간섭. mask 제거로 Orin 자유 접근(estop형 강제 0/10, 중도실패 시 특정 스텝 재주입). mode 도 동일 1회성 target(패널 토글+Orin 공유). atomicity 는 STATE 미러 폐루프로 불필요. 부팅 기본값 0xFF 는 추후. 상세: [dpcb_opmode.md](dpcb_opmode.md) §4. **코드는 사용자 작성, 배선은 타 영역 검증 후.**
- 매핑 상세: [dpcb_register.md](dpcb_register.md) 섹션 3

### 3-2. Step 8 — mode + sys_state 통합 개정 (클로드)

**2026-07-03 구조 변경**: 기존 4모드(MANUAL/HOLD/AUTO/CTRL) + deploy_fsm → **2모드(mode) + sys_state** 통합. 상세: [dpcb_opmode.md](dpcb_opmode.md). **문서(.md) 개정 완료 / 코드 미착수.**

**상태 구조 결정 (2026-07-03 확정 — B 토론 결과):**
- **`DPCB_STATE_e` 신규 도입** (CTRL=0/HOLD=1/FSM 2~8/RSVD 9/ERROR 10), 레지스터 필드는 uint8_t 유지·enum 은 값 정의용
- addr 57 `sys_state` repurpose (`SYSTEM_STATE_e`→`DPCB_STATE_e`), 기존 `SYSTEM_STATE_e`(payload_state)는 **내부 전용 유지**·SYSTEM 영역 발행 중단
- **FAULT 노출 = `hw_fatal`(55) 단독** (P2)
- **통신 FAULT 시 `DPCB_STATE`→ERROR(10) 강제 전이** (P3, 안전)
- 운용 실제값 `DPC_CTL.STATE`→`DPCB_STATE_e` 통일 (P1), 부팅 기본 CTRL(0) (P5)

코드 개정 TODO:
- **[완료 2026-07-07] addr 127 리네이밍**: `CMD_DPCB_t.deploy_fsm` → `sys_state_target`(uint8_t). 참조 주석 갱신(`rd_map_dpcb.c:275`, `rd_map_dpcb.h:85`)
- **[완료 2026-07-07] enum/#define 정리**: `DPCB_MODE_e`(MANUAL/AUTO), `DPCB_STATE_e`(CTRL/HOLD/FSM 2~8/RSVD 9/ERROR 10) 신설, 구 `MODE_IDLE/HOLD/AUTO`·`DEPLOY_*` 제거. 필드는 uint8_t 유지
- **[완료 2026-07-07] addr 57**: 주석→`DPCB_STATE_e` 갱신 + PUBLISH(`rd_map_dpcb.c`) 발행 소스 `payload_state`→**`DPC_CTL.STATE`** 단방향 복사로 교체. *단, DPC_CTL.STATE 값 체계(현 0=manual/1=INIT/2=DESCEND_1/3=DESCEND_2/4=WAIT/5=ASCEND_1/6=ASCEND_2/7=FINISH/10=ERROR)의 DPCB_STATE_e 정합은 아래 제어 통합 TODO(§3-3)에서 완성*
- **[TODO] FAULT→ERROR 연동**: `ACTION_STATE_FAULT`(또는 RD_TASK_SYSTEM FAULT 분기)에서 `DPC_CTL.STATE=DPCB_STATE_ERROR` 세팅 추가
- **[TODO] Write 권한**: `mtr_lock`(`rd_system.c:329`, 현재 상시 잠금) 폐기 → mode+`DPCB_STATE` 기반(opmode §6)으로 `RD_MAP_DISPATCH_WRITE`/rs485Task 재작성
- **[TODO] 제어 통합**: `rd_control.c` `RD_CONTROL_LOOP` 를 mode(입력원)+`DPCB_STATE_e` 기반으로 통합. `DPC_CTL.STATE` 값 체계를 `DPCB_STATE_e` 로 재매핑(FSM 1~7 → 2~8 shift, HOLD=1 신설), 리터럴→enum 상수 치환, mode 분기·sys_state_target **1회성 소비(mask 폐기, 2026-08-03)**·ERROR 유지 반영. **상세 계획·미결 질문: §3-3**

### 3-3. sys_state 정합성 재검토 — RD_CONTROL_LOOP 통합 계획

> `DPC_CTL.STATE`(현) → `DPCB_STATE_e`(신) 재매핑 계획. **코드 변경 전 아래 미결 질문 전부 close 필요.**

#### (a) 값 매핑 — 현 DPC_CTL.STATE(rd_control.c 실제) → 신 DPCB_STATE_e

| 현 STATE          | 현 case 함수           | → 신 값   | 신 이름      |
| ---------------- | ------------------- | ------- | --------- |
| 0                | CASE_IDLE(패널)       | 0 (동일)  | CTRL      |
| —                | (신설)                | 1       | HOLD      |
| 1                | CASE_INIT           | 2       | INIT      |
| 2                | CASE_DESCEND_1      | 3       | DESCEND_1 |
| 3                | CASE_DESCEND_2      | 4       | DESCEND_2 |
| 4                | CASE_WAIT           | 5       | WAIT      |
| 5                | CASE_ASCEND_1       | 6       | ASCEND_1  |
| 6                | CASE_ASCEND_2       | 7       | ASCEND_2  |
| 7                | CASE_FINISH         | 8       | FINISH    |
| 10 (ERROR_STATE) | CASE_ERROR(default) | 10 (동일) | ERROR     |

→ **FSM 1~7 은 +1 shift, 0·10 은 불변, HOLD(1) 신설.** (참고: `rd_control.h:52` 헤더 주석은 실제 .c 와 불일치 → 수정 필요)

#### (b) 코드 변경 항목 (rd_control.c)

1. `case N:` 라벨 및 `CTL->STATE = N` 리터럴 재매핑(1~7→2~8), **`DPCB_STATE_*` enum 상수로 치환**
2. 프로토타입/주석(:34~42) case 번호 갱신, `RD_CONTROL_INIT` 초기값 0=CTRL
3. **HOLD(1) case 신설** — 동작 정의 필요(Q1)
4. **mode(126) 분기 추가** — mode=0(MANUAL)→패널 입력, mode=1(AUTO)→Orin 입력 + sys_state 구동(Q2)
5. **sys_state_target(127) 1회성 소비** — `!=0xFF` 시 target→DPC_CTL.STATE 반영 후 0xFF 클리어(CONSUME/Step7 연동). **입력 mask 폐기(2026-08-03, opmode §4)** — Orin 자유 접근(Q3)
6. **진입 트리거 교체** — 현 `SW1==1 → STATE=INIT`(:96), `SW1==1 → ASCEND`(:173) 를 Orin target 기반으로(Q4)
7. **ERROR 유지** — 현 CASE_ERROR 의 `CTL->STATE=0` 자동 복귀(:472) 제거 → Orin target=0 기입 시 복귀(opmode §7-2)(Q5)
8. FINISH → CTRL(0) 복귀(:224 현 `=0` 유지, opmode 확정)

#### (c) 미결 질문 — CLOSE (2026-07-21, `memo_26024.md` A1~A7 확정)

> Q1~Q7 전부 close. 결정 요지: **핵심 case 함수는 스텁(껍데기)만 생성·본체 미구현**, 실제 로직은 사용자가 추후 작성. 관련 미구현 항목은 §3-4 로 이관.

- **Q1 (HOLD 동작) ← A1**: `RD_CONTROL_CASE_HOLD` **함수 스텁만 생성·미구현 유지**. (참고 요약: ① `GPIO->CON_DATA==0xF0` 확인 [실제 비트구성: bit7~4 = CON_A~D, 전체 잠금=0xF0. memo 의 0b00001111 은 오기] → ② INIT 유사 방식으로 윈치 장력 걸린 시점의 `present_pos` **1회 저장** → ③ 저장 pos 를 모터 위치 고정값으로 송신. 본체는 사용자 작성)
- **Q2 (CTRL 입력 경로) ← A2/A3**: `RD_CONTROL_CASE_CTRL` **함수 스텁만 생성·미구현 유지**. `DPC_CTL.STATE=0` 에서 **mode 로 라우팅** — `mode=0`→`CASE_IDLE`(패널, 현행 유지), `mode=1`→`CASE_CTRL`(Orin, 미구현). `RD_CONTROL_LOOP` 에 STATE=0 mode 분기 추가(Step8 코드 작업).
- **Q3 (target 소비 주체) ← A3**: `sys_state_target`→`STATE` 적용은 **CONSUME 에서 처리**. **방향성 확정(2026-08-03)**: 1회성 소비(`!=0xFF` 시 반영 후 0xFF 클리어) + mask 폐기(자유 접근). 코드는 사용자 작성·배선은 타 영역 검증 후. 상세: [dpcb_opmode.md](dpcb_opmode.md) §4, §3-1.
- **Q4 (진입 트리거) ← A2/A4 (2026-07-21 구현 확정)**: **SW1=mode 토글, SW2=FSM 진입**(mode=1). SW2 → `DPC_CTL.STATE` 직접 전이. CTRL/HOLD 에서 SW2 길게(>SW_HIGH)→INIT(2). *(구 문서의 SW1 0.5초 기술은 SW2 로 정정)* *(2026-08-03: mask 폐기로 충돌 해소 — SW2 직접 전이·Orin CONSUME 둘 다 controlTask 내 STATE write, 마지막 기입 반영. opmode §5-1)*
- **Q5 (ERROR 복귀) ← A5 (구현 완료)**: **자동 복귀 제거**. CASE_ERROR 의 `CTL->STATE=0` **주석처리 완료**(`rd_control.c:536`). ORIN 이 target=0 강제 시에만 복귀.
- **Q6 (WAIT 핸드셰이크) ← A4/A6**: WAIT(5)→ASCEND 트리거는 **로컬 SW2 짧게 → `DPC_CTL.STATE=6` 직접 전이**(SW2 길게는 CTRL 복귀). Orin 의 `sys_state_target` 경로(→CONSUME)는 **미구현 TODO**(§3-4).
- **Q7 (타임아웃) ← A7**: WAIT(5) 타임아웃 **이번 구현 대상 아님**. TODO/구현예정에서 **제거**하고 **토론 이관**(시나리오상 아직 불필요).

- 참고 문서: [dpcb_opmode.md](dpcb_opmode.md) §3~5, 근거: `memo_26024.md`

### 3-4. 미구현 확정 항목 (스텁만 생성 / 본체 사용자 작성)

> §3-3 Q close 결과. 클로드는 **함수 껍데기·연동 지점만 생성**하고 본체는 비워둠. 사용자가 추후 작성·업데이트.

| 항목 | 처리 | 담당 |
|------|------|------|
| mode 라우팅 (`mode0`→`CASE_IDLE` / `mode1`→ STATE switch) | **구현됨**(`rd_control.c:95/111` else if). CTRL/HOLD case 는 전이만·본체 미구현 | 사용자 |
| `RD_CONTROL_CASE_HOLD` | 빈 스텁 생성 완료(`rd_control.c:306`), 본체 미구현 (요약: CON_DATA==**0xF0** → present_pos 1회 저장 → pos 고정 송신) | 사용자 |
| `RD_CONTROL_CASE_CTRL` | 빈 스텁 생성 완료(`rd_control.c:300`), 본체 미구현 (Orin 관제 입력, mode=1 & STATE=0) | 사용자 |
| CONSUME (`sys_state_target`→STATE) | **1회성 소비·mask 폐기 확정(2026-08-03, opmode §4)**. 본체 미구현 | 사용자 (Step7) |
| 패널 SW2 FSM 진입 트리거 (SW2 → `DPC_CTL.STATE` **직접** 전이: =2 / =6) | 구현됨(전이부) / CTRL·HOLD 본체 미구현 | 사용자 |
| SW2 직접 전이 ↔ Orin target→CONSUME 충돌 | **해소(mask 폐기)** — 둘 다 controlTask STATE write, 마지막 기입 반영 | — |
| **자유 접근 비용 = 전이쌍 액추에이터 안전** (각 case 진입부 이전상태 모터 정리) | mask 폐기로 STM 이 시퀀스 합법성 미보장 → estop·재시작 안전은 case 진입부 책임. **CTRL/HOLD 스텁 구현 시 필수** | 사용자 |
| `sys_state_target`·`mode` 부팅 기본값 0xFF 초기화 | 미구현 TODO (현재 zero-init 유령소비 무해, opmode §4-3) | 사용자 |
| mode target화 (패널 토글+Orin 공유, 1회성) | 미구현 TODO (opmode §5-1) | 사용자 |
| Orin 직접 `sys_state_target` 변경 경로 | 미구현 TODO | 추후 |
| CASE_ERROR 자동복귀 제거 | **완료** (`rd_control.c:536` 주석처리) | 클로드 |
| WAIT(5) 타임아웃 | 구현 제외 → 토론 이관 | 보류 |

### 3-5. `rd_control` 코드 잔여 TODO (2026-07-21 검토 기준)

> Session 19 `rd_control.c` mode+FSM 구현/검토 후 남은 구현 항목. 상세 현황: [dpcb_task.md](dpcb_task.md) §5.

| 항목 | 현 상태 | 비고 |
|------|---------|------|
| 모터 통신오류 → ERROR 전역감시 | `rd_control.c:86~92` **주석** | 테스트 환경 모터 3개 미연결 → 추후 활성 |
| 통신 FAULT → ERROR 강제전이 | 미구현 | `rd_system` 연동 필요 (opmode §7-1 P3, task §3-1 개정목표) |
| ASCEND 초과상승 위치제한 | `rd_control.c:264~268` 주석 | 상승 position 리밋 미구현 |
| ASCEND 과전류 감지 | 주석(`max_current` 계산만) | `> ASC_CURR_LIM` 에러 조건 미연결 |
| Write 권한(`mtr_lock`) | 구 `payload_state==AUTO` 로직 잔존 (상시 잠금) | mode+`DPCB_STATE` 기반 재작성 필요 (opmode §6, §4 하단) |
| 잔여 리터럴 `STATE=0` | `:223`(WAIT롱), `:280`(FINISH), `:105`(mode전환), `RD_CONTROL_INIT` | `DPCB_STATE_CTRL`/`DPCB_MODE_MANUAL` 치환 권장 (경미) |
| DESCEND_2 하강거리 제한(`avr_pos<MAX_POS`) | 구현됨(실질 타임아웃 우선 발동) | 동작 여지 낮음, 유지 |

---

## 4. 미확정 정책 / TODO

- `RS485_TEST_ON` 제거 시점 (재부팅 활성화 전환) — 실운용 진입 시 결정
- Checker 에러 → recovery/estop 전이 세부 조건 — 사용자 추후 업데이트
- i2c(패널) 채널 Checker 미구현 → PUBLISH 에서 `degraded_cnt[3]`·`sensor_dpcb.panel_state`(addr 73) 0 고정. i2c Checker 구현 시 연동 예정
- **`SYSTEM_STATE_e` 개정 예정** (ECU 유래, addr 57 repurpose 대비) — 현재 사용처 교차검증 결과:
  - 정의: `rd_system.h:45~51`(enum), `:93`(extern payload_state)
  - 인스턴스: `rd_system.c:26` `payload_state = SYS_STATE_INIT`
  - 동작 사용: `rd_system.c:138`(→SYS_STATE_FAULT 설정), `:296`(FAULT 체크→ACTION_STATE_FAULT), `:329`(`mtr_lock = (payload_state==SYS_STATE_AUTO)?0:1`), `rd_map_dpcb.c:182`(PUBLISH→reg.sys.sys_state addr57)
  - 주석: `rd_system.c:121/177`, `rd_register_dpcb.h:159`, `rd_common.h:27`
  - **참고**: payload_state 는 실제로 INIT/FAULT 만 set 됨. MANUAL/AUTO/ESTOP_SW/ESTOP_HW 는 정의만 존재(set 없음)하며, `:329` 의 `==SYS_STATE_AUTO` 비교는 현재 항상 거짓 → `mtr_lock` 상시 1. 개정 시 이 로직 재설계 필요
