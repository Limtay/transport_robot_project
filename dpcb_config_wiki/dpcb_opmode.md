# DPC_B 운용 모드 및 sys_state (통합 구조)

> 최종 갱신: 2026-08-03 (CONSUME 1회성 소비 + 입력 mask 폐기 확정 — §4)
> 상위: [dpcb_overview.md](dpcb_overview.md)

> **2026-07-03 구조 개정**: 기존 **4모드(MANUAL/HOLD/AUTO/CTRL) + deploy_fsm** 을 **2모드(mode) + sys_state** 로 통합함. 풍부한 상태머신을 `mode`에서 `sys_state`로 이관하고, `deploy_fsm`(addr 127)을 `sys_state_target`으로 대체함. **본 문서는 개정된 목표 구조 기준이며, 코드(`rd_control.c` deploy_fsm 상수 등)는 아직 미반영(전면 개정 예정).**

---

## 1. 통합 구조 개요

| 레지스터 | addr | R/W | 역할 |
|----------|------|-----|------|
| `mode` | 126 | R/W | 최상위 운용 모드 (Manual / AUTO) |
| `sys_state` | 57 | **R/O** | 시스템의 **실제** 상태 (STM 이 갱신) |
| `sys_state_target` | 127 | **R/W** | Orin 이 요청하는 **목표** 상태 (구 `deploy_fsm` 자리) |

- `mode = 1(AUTO)` 일 때 실제 동작은 **`sys_state` 가 결정**함.
- Orin 은 `sys_state_target` 에 목표를 write → STM 이 전이 후 실제값을 `sys_state` 에 반영하는 구조.
- → 기존 레지스터 **이름 변경 필요** (addr 127 `deploy_fsm` → `sys_state_target`).

**상태 enum / 소유 (2026-07-03 확정)**
- `sys_state`(57) / `sys_state_target`(127) 값은 신규 **`DPCB_STATE_e`** (CTRL=0 / HOLD=1 / FSM 2~8 / RSVD 9 / ERROR 10) 를 사용. 레지스터 필드는 `uint8_t` 유지, `DPCB_STATE_e` 는 **값(상수) 정의용**.
- 운용 상태 **실제값은 `rd_control.c` 의 `DPC_CTL.STATE` 가 보유** → `DPCB_STATE_e` 로 통일. PUBLISH 가 이를 addr 57 로 발행.
- 기존 **`SYSTEM_STATE_e`**(`payload_state`, ECU 레거시)는 **내부 전용으로 유지** — SYSTEM 레지스터 영역(addr 57)에는 더 이상 발행 안 함. 하드웨어 FAULT 노출은 **`hw_fatal`(addr 55) 로 일원화**.

---

## 2. mode (addr 126) — 2종

| 값 | 이름 | 상세 동작 |
|----|------|----------|
| 0 | MANUAL | **(default)** 페리페럴의 패널 기반 제어. 패널 스위치로 윈치 3개 상승/하강, lock_en 직접 제어 |
| 1 | AUTO | Orin 이 RS485 로 관제. **세부 동작은 `sys_state` 에 따름** |

- **MANUAL(0) ↔ CTRL(sys_state 0) 관계 (2026-07-21 확정, A3)**: `DPC_CTL.STATE=0` 에서 **mode 로 처리 함수 분기** —
  - `mode=0(MANUAL)` → **`CASE_IDLE`**(패널 스위치 입력, 현행 로직 유지)
  - `mode=1(AUTO)` → **`CASE_CTRL`**(Orin 원격 제어 입력) — **현재 펌웨어 버전에선 의도적으로 빈 함수**
  - 즉 두 로직은 공유가 아니라 **mode 로 분리**. `RD_CONTROL_LOOP` 가 `STATE==0` 일 때 mode 로 라우팅.
  - **설계 의도 (2026-07-21 확정)**: `CASE_CTRL` 은 Orin 이 mode=1·CTRL 로 **원격 failsafe/수동 제어**하기 위한 자리. **현 버전은 원격제어 미포함** → 빈 함수가 정상. ERROR 진입 시 Orin 이 mode=1 CTRL 로 복구하는 시나리오의 진입점(차기 버전).
- **sys_state 유효 범위**: `mode=1(AUTO)` 에서만 운용적 의미를 가짐. `mode=0(MANUAL)` 중에는 `sys_state` 가 **CTRL(0) 에 유휴 파킹**됨. 부팅 기본값도 CTRL(0) (mode 0 부팅 → mode 1 진입 시 sys_state 는 이미 0).

---

## 3. sys_state (addr 57, R/O) — 실제 상태

| 값   | 이름    | 내용                                               |
| --- | ----- | ------------------------------------------------ |
| 0   | CTRL  | Orin 이 직접 관제하는 모드 (구 CTRL)                       |
| 1   | HOLD  | 수송을 위한 고정 모드 — 현재 위치 포지션 고정 (구 HOLD)             |
| 2~8 | FSM   | 자동 전개 세부 단계 (구 deploy_fsm INIT~FINISH 7단계, §3-1) |
| 9   | RSVD  | 예약 — 향후 확장 고려                                    |
| 10  | ERROR | ① 자동 전개(FSM) 실패(기존 deploy_fsm 진입 로직 차용) **또는** ② 통신 FAULT(`payload_state=FAULT`, uart2 fatal) 시 STM 이 강제 전이 |

### 3-1. FSM 세부 단계 (sys_state 2~8) — 구 deploy_fsm 7단계 대응

| sys_state | 구 deploy_fsm | 이름        | 동작                                         | 전이 조건                                                                                                        |
| --------- | ------------ | --------- | ------------------------------------------ | ------------------------------------------------------------------------------------------------------------ |
| 2         | 0            | INIT      | 각종 시스템 상태 정상 확인 + 윈치 위치 초기화 + 저전류 모드로 줄 당김 | 시간 경과 후 모터 속도 합이 충분히 작아질 것                                                                                   |
| 3         | 1            | DESCEND_1 | DPC-B lock 해제(lock_en) + 저속 하강             | DPC-B `lock_contact` 전체 해제 확인                                                                                |
| 4         | 2            | DESCEND_2 | lock 동작 종료 후 고속 하강                         | `prox_contact` 로 지면 접촉 확인                                                                                    |
| 5         | 3            | WAIT      | 지면 접촉 확인 후 DPC-A lock 해제 + 대기              | **STM 자동 전이 없음** — `DPC_CTL.STATE=6`(ASCEND_1) 진입 시 전이 (로컬 SW2 짧게 또는 Orin target). *(타임아웃은 §7-2 토론 이관 — 미구현)* |
| 6         | 4            | ASCEND_1  | 상승 시작                                      | `prox_contact` 전체 해제 + 줄 장력 확인                                                                               |
| 7         | 5            | ASCEND_2  | 고속 상승                                      | DPC-B `lock_contact` 전체 확인 (집게 재고정)                                                                          |
| 8         | 6            | FINISH    | 페리페럴 정리 후 **CTRL(0) 자동 복귀**                | 자동 전환                                                                                                        |

- 자동 진행 구간: INIT(2)→DESCEND_1(3)→DESCEND_2(4)→**WAIT(5)** 는 STM 이 조건 충족 시 자동 전이.
- WAIT(5) 이후: Orin 개입 필요 (§5 핸드셰이크).
- ASCEND_1(6)→ASCEND_2(7)→FINISH(8) 는 다시 자동 전이.

**센서 폴라리티 / 전이 마스크 (2026-07-21 확정, `rd_control.h` 상수)**
- **PROX (지면 접촉, `A_PROX_DATA` 하위 3bit)**: **접촉 시 비트=1**.
  - `A_PROX_ALL_ON = 0x07` (전체 접촉) — **DESCEND_2 전이**(줄이 모두 지면 도달)
  - `A_PROX_ALL_OFF = 0x00` (전체 이탈) — **ASCEND_1 전이**(모두 지면에서 떨어짐)
  - ※ 두 조건은 정반대이므로 반드시 다른 상수 사용 (DESCEND=ON / ASCEND=OFF)
- **CON (DPC-B locker, `CON_DATA` bit7~4 = CON_A~D)**: **잠금 시 비트=1**.
  - `CONT_ALL_UNLOCK = 0x00` (전체 해제) — **DESCEND_1 전이**
  - `CONT_ALL_LOCK = 0xF0` (전체 잠금) — **ASCEND_2 전이** (집게 재고정 확인). HOLD 진입 판정도 0xF0

---

## 4. sys_state_target (addr 127, R/W) — 1회성(one-shot) CONSUME

Orin 이 목표 상태를 기입하는 레지스터. **2026-08-03 확정: 매 주기 반영이 아닌 "수신 시 1회성" 소비 구조, 입력 mask 는 폐기(자유 접근).**

### 4-1. 1회성 소비 메커니즘
- CONSUME 로직: `if (sys_state_target != 0xFF) { DPC_CTL.STATE = sys_state_target; sys_state_target = 0xFF; }`
- `0xFF` = "명령 없음" sentinel (`DPCB_STATE_e` 유효범위 0~10 밖). 1회 소비 후 target 을 0xFF 로 되돌려 **추가 소비 차단** → FSM 자기전이를 계속 되돌리던 문제 소실.
- addr127 write 가 이벤트성(수신 시 1회)이라 target 도 이벤트성 → single-producer(rs485Task 기입)/single-consumer(controlTask 소비) handoff. **`DPC_CTL.STATE` write 주체는 controlTask 단독** 유지 → FSM 자기전이와 race 없음.

### 4-2. 입력 mask 폐기 (자유 접근) — 2026-08-03
- **구 설계**: target 을 {CTRL0, HOLD1, INIT2, ASCEND_1 6} 4개로 제한(mask), 중간스텝/ERROR 는 STM 내부전용 (아래 구 표).
- **폐기 사유**:
  - 1회성 구조가 "target 을 실제 상태로 그대로 복사 불가(FSM 진행 중 실제>목표)" 문제를 **mask 없이 이미 해소**(1회 반영 후 0xFF 라 FSM 자기진행 미간섭).
  - 자유 접근이 운용상 필요 — **estop형 강제 정지**(Orin target=0 CTRL / target=10 ERROR 강제 이탈), **중도실패 재시작**(실패 지점의 특정 FSM 스텝 3/4/5/7/8 부터 재주입).
- → **모든 `DPCB_STATE_e` 값 자유 기입 가능.** 시퀀스 합법성 책임을 STM→Orin 으로 이전.
- ⚠ **비용(구현 시 필수)**: STM 이 전이 합법성을 안 지키므로 **모든 전이쌍이 물리적으로 안전**해야 함 = 각 case 진입부가 이전 상태 액추에이터 정리(모터 stop/재지령)를 스스로 보장해야 estop·재시작이 실제 안전. **특히 CTRL/HOLD 스텁 구현 시 반드시 포함.**
- source-state 가드(예: target=6은 WAIT 에서만)도 **의도적 배제** — 자유 접근 유지.

> **구 설계(폐기) — 입력 mask 표** (traceability):
> | 허용 target | 의미 |
> |---|---|
> | 0 CTRL / 1 HOLD / 2 INIT / 6 ASCEND_1 | 진입점·휴지만 허용, 나머지 무시 |

### 4-3. 부팅 기본값 (TODO)
- `reg` zero-init → `sys_state_target = 0x00`(=CTRL) → **부팅 후 최초 1회 유령 소비**(STATE=0 주입 후 0xFF 클리어). STATE 초기값도 0이라 현재 무해하나 "명령없음=0xFF" 불변식이 부팅 시 깨짐.
- 조치(추후): `RD_SYSTEM_INIT`/MAP init 에서 `sys_state_target = 0xFF` 명시. **mode target 도 동일 처리 필요**(§5-1).

### 4-4. atomicity — 불필요 (mirror 폐루프)
- rs485Task write vs controlTask read-clear 는 태스크 간 비원자 → 최악 시 신규 target 1건 드롭(1바이트 정렬이라 손상 없음). **STATE 미러(addr57 `sys_state`)를 Orin 이 보고 재전송 → 자기치유**하므로 별도 CRITICAL 불필요.

---

## 5. mode = 1(AUTO) 자동 전개 핸드셰이크

### 5-1. 패널 버튼 전이 방식 (2026-07-21 구현 확정, `memo_26024.md` A2/A4)

**누름 시간 판정 방식**: 스위치를 **뗄 때**(`SW*_state == 0`) `HAL_GetTick() - *_SW_LAST` 로 눌린 시간을 계산. 임계값 `SW_LOW=100`(≈0.1s) / `SW_HIGH=5000`. 눌린 동안 `*_SW_LAST` 는 갱신되지 않아 뗄 때 델타 = 누름 길이.

**SW1 = mode 토글 전용** (양 모드 공통, `MODE_SW_LAST` 사용)
| 조건 | 동작 |
|------|------|
| 짧게(LOW~HIGH) @ mode0 | `mode=1`(AUTO) 진입 + **STATE=CTRL(0) 리셋** |
| 짧게(LOW~HIGH) @ mode1 | `mode=0`(MANUAL) 복귀 (STATE 미리셋 — 재진입 시 위에서 0 리셋) |
| 길게(>SW_HIGH) | 리부팅 예약 — **미구현** |

> **mode 소유권 = 패널 + Orin 공유 (2026-08-03 확정)**: `reg.cmd_dpcb.mode` 도 `sys_state_target` 과 **동일한 1회성 target**(0xFF sentinel)으로 사용 예정. Orin 은 절대값(0/1) 기입, 패널 SW1 은 토글이라 `!(현재 MODE)` 를 계산해 target 에 기입(Orin 은 미러 참조). 부팅 기본값 0xFF 필요(§4-3). 통신·버튼 동시발생 시 랜덤 반영은 감수. 현재 SW1 은 `DPC_CTL.MODE` 직접 기입 — target 경유 통일은 코드 개정 시(사용자).

**SW2 = 컨텍스트 의존** (`FSM_SW_LAST` 사용)
- `mode=0(MANUAL)`: 누르는 동안(`SW2_state==1`) A·B locker EN — 모멘터리 (`CASE_IDLE`, `TIMEOUT_SOL` 제한)
- `mode=1(AUTO)`: STATE 별 전이 (뗄 때 판정, `DPC_CTL.STATE` **직접** 전이 — 레지스터 우회)

| 현 STATE | SW2 짧게(LOW~HIGH) | SW2 길게(>SW_HIGH) |
|----------|--------------------|--------------------|
| CTRL(0) | → HOLD(1) | → INIT(2) FSM 개시 |
| HOLD(1) | → CTRL(0) | → INIT(2) FSM 개시 |
| WAIT(5) | → ASCEND_1(6) | → CTRL(0) 복귀 |
| INIT·DESCEND·ASCEND·FINISH | — (센서/타임아웃 자동전이) | — |

- **2차 — Orin 직접 기입**: 동일 전이를 Orin 이 `sys_state_target`(127) write → CONSUME(1회성, §4)이 `DPC_CTL.STATE` 로 반영. **배선은 타 영역 검증 후**(plan §3-1).
- **경로 관계 (mask 폐기로 충돌 해소, 2026-08-03)**: 로컬 SW2 는 `DPC_CTL.STATE` 직접 전이, Orin 은 `sys_state_target`→CONSUME. 둘 다 controlTask 내 STATE write 라 race 없음. mask 가 없으므로 값 충돌도 없음 — **마지막 기입이 반영**(SW2·Orin 동시 시 랜덤성은 감수, 아래 mode 정책과 동일).

### 5-2. 핸드셰이크 흐름
1. `mode = 1(AUTO)` 진입 후, `DPC_CTL.STATE = 2`(INIT) 진입 (SW2 길게 / Orin target=2 → CONSUME)
2. STM 이 에러 체크 후 INIT(2)→DESCEND_1(3)→DESCEND_2(4)→**WAIT(5)** 까지 자동 진행
3. Orin 이 주기적으로 `sys_state`(57) 를 Read 하여 `5`(WAIT) 도달 확인
4. 자기 처리(물체 하강/lock 처리 등) 완료 후 `DPC_CTL.STATE = 6`(ASCEND_1) 진입 (SW2 짧게 / Orin target=6 → CONSUME)
5. STM 이 ASCEND_1(6)→ASCEND_2(7)→FINISH(8) 진행
6. FINISH(8) 완료 후 **CTRL(0) 로 자동 복귀**

> WAIT(5) 에서는 STM 자체 전이 조건이 없음 — `DPC_CTL.STATE=6` 전이(SW2 짧게 또는 Orin target=6)가 있어야만 상승 단계로 전이.

---

## 6. mode 별 레지스터 Write 권한

Read 는 모든 상황에서 전체 허용. Write 범위만 제한함.

- **mode = 0 (MANUAL)**: `mode`(126) 만 write 가능. (default)
- **mode = 1 (AUTO)**: `sys_state` 값에 따라 차등:

| sys_state | 이름 | Write 허용 범위 |
|-----------|------|----------------|
| 0 | CTRL | **모든 RW 레지스터** |
| 1 | HOLD | `mode`(126), `sys_state_target`(127) |
| 2~8 | FSM | `mode`(126), `sys_state_target`(127) |
| 10 | ERROR | `mode`(126), `sys_state_target`(127) |

---

## 7. 미확정 / TODO

### 7-1. 상태 구조 결정 (2026-07-03 확정 — 코드 미반영, 개정 대기)

- **addr 57 repurpose**: `sys_state`(57) 를 `DPCB_STATE_e`(운용) 로 전환. 기존 `SYSTEM_STATE_e`(payload_state)는 내부 전용 유지, SYSTEM 영역 발행 중단.
- **FAULT 노출**: `hw_fatal`(addr 55) 단독. addr 57 은 FAULT 를 직접 표시하지 않음.
- **FAULT ↔ 운용상태 연동**: 통신 FAULT(`payload_state=FAULT`) 발생 시 STM 이 `DPCB_STATE` 를 **ERROR(10) 로 강제 전이** → addr 57 이 거짓 상태 방송 방지 (특히 `RS485_TEST_ON` 리부팅 차단 빌드).
- **실제값 소유**: `DPC_CTL.STATE` → `DPCB_STATE_e` 통일, 부팅 기본 CTRL(0).

### 7-2. 잔여 미확정

- **Error(10) 복귀 (2026-07-21 확정, A5)**: **자동 복귀 완전 제거** — CASE_ERROR 의 자동복귀 로직(`CTL->STATE=0`) 삭제, **자동복귀 함수는 공란 유지**. Orin 이 `sys_state_target=0`(CTRL) 을 강제 기입해야만 복귀. *단, 통신 FAULT 유래 ERROR 는 통신 복구 전까지 Orin 기입 불가 — 통신 복구 후 복귀 또는 리부팅(실운용 빌드)에 의존.*
- **Error 진입 조건**: FSM 실패는 기존 코드 로직 차용, 통신 FAULT 는 §7-1 강제 전이. (그 외 신규 조건 미정)
- **WAIT(5) 타임아웃 — 토론 이관 (2026-07-21, A7)**: 시나리오상 아직 불필요 → **구현 대상에서 제거**, 향후 토론 결정. 현재는 `sys_state_target=6` 기입에만 의존.
- sys_state 9 (RSVD) 확장 용도 미정
- `mtr_lock`/Write 권한: 기존 `payload_state==AUTO` 판정(`rd_system.c:329`, 현재 상시 잠금) 폐기 → mode+`DPCB_STATE` 기반(§6)으로 재작성 필요
- **코드 전면 개정 예정**: 구 `mode` #define(IDLE/HOLD/AUTO) 및 `deploy_fsm` 상수(`rd_register_dpcb.h`) → 2모드 + sys_state 체계로 재작성, `rd_control.c` `RD_CONTROL_LOOP` 통합. 레지스터 addr 127 리네이밍(`deploy_fsm`→`sys_state_target`) 포함. (본 .md 개정 후 착수)
