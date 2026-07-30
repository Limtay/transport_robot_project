# 01 — 운용 모드 정의 · 통합 FSM · 슬롯 프레임 · shadow 권위 모델

> 선행: [00_overview.md](00_overview.md). 결정: 2026-07-26 사용자 확정 (1차 Q1~Q8, 2차 R1~R6).
> 이 문서가 정하는 것: "모드" 이름 규칙, 각 모드가 결정하는 항목, 통합 FSM, 슬롯 프레임,
> 명령 삽입 규칙, **shadow 권위 모델**, 개명표, 레지스터·버퍼 확장.
> 이 문서가 정하지 않는 것: 파일 배치(02), 메시지 필드(03), 슬롯 구현 상세(04).

---

## 1. 확정 결정

### 1차 라운드 (Q1~Q8)

| # | 결정 | 값 |
|---|------|-----|
| Q1 | 브리지 모드 표현 | **단일 enum** `bridge_mode: project \| control \| manual`. `traction_test_mode` 삭제 |
| Q2 | 런타임 모드 전환 | **기동 시 고정**. 변경 = 브리지 재시작 |
| Q3 | 슬롯 수 | **10칸** — ECU RW×5 + DPC×1 + PCU×1 + Command×3 |
| Q4 | CLI 통합 | `command_cli` **삭제**. raw `reg` 서브커맨드 없음 — 명령은 상위에서 의미 단위로 조립 |
| Q5 | 메시지 패키지 | `mgs01_base_msgs`=`JeonGae` 만 / `mgs_tp_msgs`=나머지 전부 |
| Q6 | `testbed` 이름 | **`control` 로 개명** (§8) |
| Q7 | 웹 착수 | 계약(03) 먼저, 구현은 리팩터링 후 |
| Q8 | STM 레지스터 재배치 | 채택 — SYS 영역 16→**17B**, `rs485_proc_delta` → **addr 32** (§9) |

### 2차 라운드 (R1~R6)

| # | 결정 | 근거 |
|---|------|------|
| R1 | **`developer_mode` 파라미터 삭제 → `manual` 모드에 흡수** | 그 권한이 필요한 작업이 전부 manual 성격. 축을 하나 줄인다 (§4.1) |
| R2 | **`manual` 재정의**: write 전면 거부 → **"자동 설정이 전무한 고급 수동 모드"** | 백지에서 사용자가 직접 쌓아 올리는 콘솔 (§4.1) |
| R3 | **기동 조합 거부 규칙 6개 → 2개** | write 범위가 `auto_mode` 에서 파생되면 겹침이 **구조적으로 불가능** (§4.3) |
| R4 | **읽기 프리셋에 read-back 세그 넣지 않는다** (`{128,4}`, `{164,16}` 제외) | 5ms 예산이 버퍼보다 상위 제약. read-back은 **shadow 권위 모델**로 대체 (§7) |
| R5 | **STM 버퍼 90 → 256** (+`RX_BUFFER_SIZE` 64→256, `TX_BUFFER_SIZE` 128→272) | 이월 버그 **P8 의 근본 수정** + INIT 전 범위 스캔 1 트랜잭션화 (§9.2) |
| R6 | **정기 READ = IDLE 전용.** RUNNING 중에는 프리셋 고정, 1회성 READ만 허용 | 센서 5ms 민감도 + **200Hz 피드백 스트림 균일성**(R5 원칙) (§6.3) |

---

## 2. "모드" 이름 규칙 — 4중 충돌 최종 해소

00 §2.1의 4개 축에 겹치지 않는 최종 이름을 부여한다. 코드·문서·로그·CLI는 반드시 이 이름을 쓴다.

| 축 | 최종 이름 | 사는 곳 | 값 |
|---|-----------|---------|-----|
| Orin 브리지 프레임 | **`bridge_mode`** | 기동 파라미터 (enum) | `project` / `control` / `manual` |
| ECU 조종권 | **`ecu.mode`** | ECU addr **190** | `MANUAL(0)` / `AUTO(1)` |
| ECU 명령 해석 경로 | **`ecu.auto_mode`** | ECU addr **188** | §3 표 |
| 브리지 write 상태기계 | **`ControlState`** | 브리지 내부 FSM | `INIT`/`IDLE`/`RUNNING`/`LOCKED` |

**금지 표현**: 맨몸의 "모드", `control_mode`(bool), `traction_test_mode`, `TestbedState`, `developer_mode`.

```
bridge_mode          ── 무엇을 통신하나 (슬롯 테이블·읽기 프리셋·토픽 세트)
   └─▶ ecu.mode      ── 조종권. AUTO 여야 ECU가 Orin 명령을 받는다
          └─▶ ecu.auto_mode ── 무엇을 쓰나. **write 범위를 파생**시킨다
                     └─▶ ControlState ── 무엇을 넣나 (값의 출처)
```

---

## 3. ecu.auto_mode ↔ write 범위

**원칙: bridge 가 쓰는 영역 = ECU 가 건드리지 않는 영역.**

| `ecu.auto_mode` | 값 | ECU 동작 | bridge write 범위 |
|---|---|---|---|
| `NONE` | — (§3.1) | — | **∅** |
| `KINEMATIC` | 0 | `ctr_mode`를 VELOCITY로 강제 + 자체 kinematics | **180:8** (`cmd_lin_vel`, `cmd_ang_vel`) |
| `CURRENT` | 1 | `ctr_mode`를 CURRENT로 강제 (자가치유) | **164:16** (`cmd_current`) |
| `VELOCITY` | **4 (신규)** | `ctr_mode`를 VELOCITY로 강제 | **148:16** (`cmd_velocity`) |
| `POSITION` | **5 (신규)** | `ctr_mode`를 POSITION으로 강제 | **132:16** (`cmd_position`) |
| `DIRECT` | 2 | 무가공 통과 (미개입) | **128:52** (`CMD_MOTOR_t` 전체) |
| `CONTROL` | 3 | 미구현 (motor off) | **금지** |

### 3.1 `NONE` 의 정체 — ECU 값이 아니다

"None: Read 동작만 수행"은 **브리지 쪽 write 범위 = ∅** 이며, ECU addr 188에 넣는 값이 아니다.
`ecu.mode=AUTO` 에서 write를 멈추면 `reg.diag.cmd_write_tick` 미갱신으로 ECU `AUTO_TIMEOUT`(100ms)이
`motor_on=0` 을 내린다 → 모터 정지.

**이것을 거부 사유로 삼지 않는다 (R3)**: 모터를 정지 유지한 채 200Hz로 센서만 읽는 것은 유효한
의도된 동작이다 (200Hz 센서 로깅). 기동 시 INFO 로그만 남긴다:
`"auto_mode=none — write 범위 ∅, 모터는 AUTO_TIMEOUT(100ms) 후 정지 유지"`

### 3.2 신규 값 번호 부여 — append only

`VELOCITY=4`, `POSITION=5` 로 **뒤에 붙인다**. 0~3의 의미를 절대 재배치하지 않는다 — 플래시된
ECU와 Orin 상수가 어긋나면 "전류 명령이 위치 명령으로 해석되는" 사고가 된다.

> ⚠ **STM 작업 (CubeIDE 빌드)**: `ACTION_STATE_AUTO` 의 `ctr_mode` 자가치유 분기에 VELOCITY/POSITION
> 케이스 추가. 없으면 ECU가 `DATA_RANGE` 로 거부한다 — 거부가 정상 동작이므로 순서 의존성은 안전.

### 3.3 write 범위 선택기 — 전 모드 공통 메커니즘

`auto_mode` 별 `TaskConfig_t` **6개를 생성자에서 미리 만들고** atomic 셀렉터로 고른다.
런타임 구조체 변경 금지 (200Hz 스레드와 레이스).

셀렉터 입력 = **shadow 의 `auto_mode` 값** (§7 권위 모델에 따라 신뢰 가능한 값).

안전장치:
- **3 tick 연속 동일 값**일 때만 전환 (단발 오염으로 범위가 튀는 것 방지)
- 전환 시 INFO 로그 (`auto_mode 1→2, write 164:16 → 128:52`)
- **범위 진입 전 shadow 소독은 전 전환에 적용된다 (2026-07-27 일반화)**. 새 write 범위는 그동안
  **어느 읽기 프리셋에도 안 들어 있던 자리**일 수 있고(예: `cmd_position` 132:16), 그러면 shadow 에
  INIT 전범위 스캔(§6.4 ②) 시점의 낡은 값이 남아 있다. 소독 없이 범위를 옮기면 **그 낡은 값이
  첫 tick 에 그대로 ECU 로 나간다.**

  | 전환 | 소독값 |
  |---|---|
  | `→CURRENT` / `→VELOCITY` / `→KINEMATIC` | `0` |
  | **`→POSITION`** | **`fb_position`** — §6.1.2 IDLE 재시드가 이 역할을 겸한다 |
  | `→DIRECT` (확장) | `ctr_mode=CURRENT` / `cmd_current=0` / `cmd_velocity=0` / `cmd_position=fb_position` |

  순서: ① 소독 → ② `auto_mode` write 성공 확인 → ③ 범위 전환
- `DIRECT→축소` 는 ① `auto_mode` write 성공 → ② 범위 축소 순서
- 전환은 **`IDLE` 에서만** 일어나므로(§6.3 C 게이트), 소독이 실험 데이터를 오염시키지 않는다

---

## 4. bridge_mode 정의

| 결정 항목 | `project` | `control` | `manual` |
|---|---|---|---|
| **목적** | 실차 운용 (주행 + 전개) | 200Hz 결정론 제어 (실험·MPC) | **고급 수동 — 모든 설정을 직접** |
| **tick** | 5ms (200Hz) | 5ms (200Hz) | 5ms (200Hz) |
| **프레임** | 10 tick = 50ms (20Hz) | 매 tick ECU | 10 tick = 50ms |
| **ECU INST** | `RW` (odd tick, 100Hz) | `RW` (매 tick, 200Hz) | `READ` (정기) / 슬롯 명령 |
| **읽기 프리셋** | `project` | `control` / `control_test` | 자유 (슬롯 교체) |
| **기본 `auto_mode`** | `KINEMATIC` | `CURRENT` | `NONE` |
| **INIT 전 범위 스캔** | ○ | ○ | ○ |
| **INIT write 검증** | mask → auto_mode → mode | mask → auto_mode → mode | **없음** |
| **`ecu.mode`** | AUTO 로 전환 | AUTO 로 전환 | **건드리지 않음** |
| **정기 write** | `cmd_vel` (RUNNING) | write_source (RUNNING) | **없음** — 슬롯 명령으로만 |
| **IDLE write 정책** | **중단** → `motor_on=0` (§6.1.1) | **0A 유지** (§6.1.1) | 해당 없음 |
| **DPC/PCU 슬롯** | ○ (각 20Hz) | ✗ | ○ |
| **Command 슬롯** | 3칸 (각 20Hz) | 0칸 (§6.3) | **10칸 전부 자유** |
| **`/jeongae` 시퀀스** | ○ | ✗ | ✗ |
| **프로파일 action** | ✗ | ○ | ✗ |
| **200Hz 피드백 토픽** | ✗ | ○ | ✗ |
| **raw WRITE / `SET_ORIGIN`** | ✗ | ✗ | **○** |
| **cmd_vel guard** | ○ | 무관 | 무관 |

**`traction` 은 모드가 아니다**: `control` + `auto_mode=CURRENT` + `control_test` 프리셋 + 전류
프로파일로 완전히 표현된다.

### 4.1 `manual` = 고급 수동 모드 (R1·R2)

**정의**: 브리지가 아무것도 자동으로 설정하지 않는 백지 상태. 사용자가 슬롯 명령으로 하나씩 쌓는다.

```
기동 → 전 범위 READ 스캔 (진실 원천 확보) → IDLE, read-only
   ↓ 사용자가 직접
   슬롯에 motor_mask WRITE  → 슬롯에 auto_mode WRITE → 슬롯에 mode=AUTO WRITE
   → forever 슬롯으로 cmd_current WRITE (20Hz = 50ms < AUTO_TIMEOUT 100ms → 모터 유지)
```

`developer_mode` 를 흡수한다 — 아래 권한이 manual 에서만 열린다:

| 권한 | 왜 manual 인가 |
|---|---|
| raw 레지스터 WRITE (주소 직접 지정) | 디버깅 작업 |
| `SET_ORIGIN` (AK mode 5, 엔코더 원점) | 셋업 작업 |
| `mode`/`auto_mode` 수동 write | 백지 모드의 본질 |
| 수동 read-back 세그 지정 | R4 — 프리셋에서 뺀 read-back을 필요할 때만 |

> **`DIRECT` 에서 모터별 `ctr_mode` 혼재는 게이트에서 제외했다**: `DIRECT` 자체가 이미 "브리지가
> `ctr_mode` 를 소유한다"는 선언이므로 그 안에서 추가 권한을 요구하는 것은 과잉이다.

**유지되는 안전장치**: `safe_stop` 게이트(§6.2), `LOCKED` 래치, `active_motors` 불변식.
manual 은 "안전장치가 없는 모드"가 아니라 "자동 설정이 없는 모드"다.

기동 로그에 WARN 을 남기고 `result.json` 의 `node_params` 에도 기록한다.

### 4.2 기동 파라미터 (최종)

| 파라미터 | 타입 | 기본 | 비고 |
|---|---|---|---|
| `bridge_mode` | string enum | `project` | `project`/`control`/`manual` |
| `auto_mode` | string enum | 모드별 (§4) | `none`/`kinematic`/`current`/`velocity`/`position`/`direct` |
| `read_preset` | string enum | 모드별 | `project`/`control`/`control_test`/`diag` (§5.3) |
| `active_motors` | int[] | `[1,2,3,4]` | **실연결 모터와 정확히 일치 필수** |
| `cmd_current_max` | double | 30.0 | [A] 전역 클램프 |
| `cmd_vel_guard_enable` | bool | true | project 전용 — false 면 `RUNNING→IDLE` 자동 전이 없음 |
| `cmd_vel_topic_timeout` / `cmd_vel_zero_timeout` | double | 0.1 / 3.0 | project 전용 — **`RUNNING→IDLE` 전이 임계** (§6.1.1) |
| `imu_frame_id` | string | `imu_link` | — |

**삭제**: `control_mode`, `traction_test_mode`, `developer_mode`.

**단어형 문자열 채택 이유**: 로그·CLI에서 `auto_mode=1` 보다 `auto_mode=current` 가 사고를 줄인다.
내부에서 §3 표의 정수로 즉시 변환한다.

### 4.3 기동 조합 검증 — 거부는 2개뿐 (R3)

원래 6개 규칙을 두었으나, 검증해 보니 **write 범위가 `auto_mode` 에서 파생되면 브리지와 ECU가
같은 자리를 쓰는 상황이 구조적으로 발생하지 않는다**:

| `auto_mode` | ECU 가 소유 | bridge write | 겹침 |
|---|---|---|---|
| `KINEMATIC` | `ctr_mode` 128:4 | 180:8 | 없음 |
| `CURRENT` | `ctr_mode` 128:4 | 164:16 | 없음 |
| `VELOCITY` | `ctr_mode` 128:4 | 148:16 | 없음 |
| `POSITION` | `ctr_mode` 128:4 | 132:16 | 없음 |
| `DIRECT` | (없음) | 128:52 | 없음 |
| `NONE` | (없음) | ∅ | 없음 |

> 과거 `control + KINEMATIC` 을 금지한 근거("ECU가 `ctr_mode` 를 덮어써 경쟁")는 **브리지가
> KINEMATIC 인데도 `cmd_current` 를 쓰던 시절의 버그 흔적**이다. 범위 파생 원칙이 그 원인을 제거했다.

**최종 거부 규칙**:

| 조합 | 판정 |
|---|---|
| `auto_mode=control(3)` | **거부** — ECU 미구현 (기능 부재) |
| `active_motors` 빈 배열 / 5 이상 / 중복 / 범위 밖 | **거부** — 형식 오류 |
| 그 외 전부 | **허용** (필요 시 INFO/WARN 로그) |

거부 = 노드 즉시 종료 (exit≠0). **USB 대기 전에 판정**한다 — 오타 때문에 "Waiting for USB..." 로
매달리는 상황을 만들지 않는다.

허용하되 로그를 남기는 조합:

| 조합 | 로그 |
|---|---|
| `control` + `none` | INFO — "write ∅, 200Hz 센서 로깅 전용" |
| `control` + `kinematic` | INFO — "200Hz cmd_lin/ang_vel 경로" |
| `manual` + `auto_mode≠none` | INFO — "manual 은 정기 write 없음. 슬롯 명령으로 사용" |
| `bridge_mode=manual` | **WARN** — "고급 수동 모드: 자동 설정 없음" |

---

## 5. 슬롯 프레임 · 읽기 프리셋

### 5.1 `project` / `manual` — 10칸 프레임

프레임 = **10 tick × 5ms = 50ms (20Hz)**

| tick (frame 내) | 슬롯 | 대상 | 유효 주기 |
|---|---|---|---|
| 1, 3, 5, 7, 9 (odd) | **ECU ×5** | ECU | **100Hz** |
| 0 | DPC | DPC | 20Hz |
| 2 | PCU | PCU | 20Hz |
| 4 | CMD0 | 사용자 | 20Hz |
| 6 | CMD1 | 사용자 | 20Hz |
| 8 | CMD2 | 사용자 | 20Hz |

- ECU 100Hz → IMU 100Hz 발행 요구 충족. `AUTO_TIMEOUT`(100ms) 대비 10배 여유
- DPC/PCU 레지스터 미확정 → `enable_dpc`/`enable_pcu` 기본 OFF. OFF면 그 tick은 CMD로 양보
- `manual` 은 10칸 전부 사용자 교체 가능 (ECU 정기 READ 슬롯도 사용자가 비울 수 있다)

### 5.2 `control` — 매 tick ECU RW (200Hz)

전용 슬롯이 없다. 사용자 명령은 §6.3 규칙으로 처리한다.

### 5.3 읽기 프리셋 — 코드가 아니라 **데이터** (R4)

| 프리셋 | 세그 | 응답 payload | 용도 |
|---|---|---|---|
| `project` | `{16,17} {48,22} {86,42}` | 81+1 = **82B** | sys 전체 + IMU + UART2/RC + 모터 (**엔코더 제외**) |
| `control_test` | `{16,17} {42,6} {88,40}` | 63+1 = **64B** | sys 전체 + **로드셀** + 모터 (견인 실험) |
| `control` | `{16,17} {47,81}` | 98+1 = **99B** | sys 전체 + 로드셀 + IMU + **엔코더** + UART2/RC + 모터 |
| `diag` | `{0,16} {16,17} {224,32}` | 65+1 = **66B** | DEFINE + SYS + DIAG (진단·INIT) |

> **엔코더(70~85)는 `control` 전용이다 (2026-07-26 확정)**: linkage encoder 는 링크가 몸체에 대해
> 기울어진 정도이며, **IMU·GPS 와 함께 로봇 global position 추정(MPC)의 입력**이다. 그 제어기는
> `control` 모드에서 돌므로 `project` 프리셋은 엔코더를 읽지 않는다 (`{48,22}`+`{86,42}` 로 70~85 건너뜀).
> → `project` 모드에는 링크 각도 발행 토픽이 없다 (03 §5.2).

**read-back 세그(`{128,4}` ctr_mode, `{164,16}` cmd_current)를 넣지 않는다.** 진실 원천은
INIT 전 범위 스캔 + write ACK 로 확보한다 (§7). 디버깅 시에는 `manual` 에서 수동 세그로 읽는다.

### 5.4 와이어 예산 검산

921600bps → 1바이트 = **10.85µs**. 와이어 = payload + 8 (Header2+ID1+Len2+Inst1+CRC2).

| 조합 | 요청 payload | 응답 payload | 와이어 합 | 시간 |
|---|---|---|---|---|
| `project` RW (write 180:8) | 1+12+2+8 = 23 | 82 | 121B | **1.31ms** (100Hz → 10ms 예산) |
| `control_test` RW (CURRENT) | 1+12+2+16 = 31 | 54 | 101B | **1.10ms** |
| `control` RW (CURRENT) | 1+8+2+16 = 27 | 88 | 131B | **1.42ms** |
| `control` RW (DIRECT) | 1+8+2+52 = 63 | 88 | 167B | **1.81ms** |
| `diag` READ | 12 | 66 | 94B | **1.02ms** |
| INIT 전 범위 READ `{0,256}` | 4 | 257 | 277B | **3.01ms** (200Hz 아님) |

5ms tick 예산 대비 최악 1.81ms — STM proc(~0.3ms) + USB latency(~1ms) 더해도 **3.1ms** 로 여유가 있다.

> **참고 — 넣지 않기로 한 union 프리셋**: `{16,17}+{42,86}+{128,52}` = 156B 응답 → DIRECT 요청과
> 합쳐 2.81ms 와이어 → 총 4.1ms/5ms. 가능하지만 여유 0.9ms. **버퍼는 확장하되 상시 쓰지 않는다**
> — 프로토콜 상한(§9.2)과 200Hz 예산은 별개 문제다.

### 5.5 슬롯 테이블 wire 예산 검증 (신규)

프리셋·write 범위 조합이 tick 예산을 넘지 않는지 **슬롯 테이블 생성 시점에 검증**한다:

```
(요청_바이트 + 응답_바이트) × 10.85µs  ≤  0.5 × tick_period
```

초과 시 기동 거부 + 어떤 조합이 얼마나 넘었는지 메시지 출력. 프로토콜이 아니라 **스케줄러가
상한을 지키는 구조** — R1(모드는 데이터)에 부합하고, 버퍼 확장 후에도 200Hz 안전이 보장된다.

### 5.6 `control` 프리셋이 P6(진단 stale)을 해소한다

현행은 `{27,5}` 로 `sys_state` 만 봐서 `hw_error`·`hw_fatal`·`hw_reset` 이 거짓 정상이었다.
새 `{26,7}` 은 `hw_reset(26)` + `sys_state(27)` + `realtime_tick(28~31)` + `rs485_proc_delta(32)` 를
연속 구간으로 덮는다. `hw_error(24)`·`hw_fatal(25)`·`degraded_cnt(16~23)` 까지 필요하면
**2026-07-27 확정 — 모든 프리셋이 `{16,17}`(SYS 전체) 을 쓴다.** `{26,7}` 은 `degraded_cnt[8]`·
`hw_error`·`hw_fatal` 을 빠뜨리는데, 그 10B 를 더 읽어도 `control` 최악(DIRECT) 이 1.93ms 로
예산 2.5ms 안이다 (04 §2.5). 대신 얻는 것: ① `NodeStatus` 매핑이 보드·모드 분기 없이 성립
② 03 §3.3 의 "`{16,17}` 전체를 읽은 트랜잭션에서만 발행" 규칙이 모든 프리셋에서 만족
③ 통신 오염도(`degraded_cnt`)를 실험 중에도 200Hz 로 기록할 수 있다.

**이것이 Q8 레지스터 재배치의 진짜 이유다**: `rs485_proc_delta` 를 DIAG(228)에서 SYS 끝(32)으로
옮기면 세그가 하나 줄고 진단 필드가 sys 와 연속이 된다.

---

## 6. 통합 FSM 과 명령 삽입 규칙

### 6.1 통합 FSM

> ## 정정 (2026-07-30) — 아래 원안은 두 곳이 낡았다
>
> **① `STREAM` 은 별도 상태로 살아 있다.** 이 절은 *"`STREAM` 을 별도 상태로 두지 않는다"*
> 로 시작하지만 **§6.1.3 이 나중 결정이고 코드가 그쪽을 따른다** — `ControlState::STREAM`
> 은 arm 게이트를 가진 5번째 상태이고 `ControlFeedback.control_state` 도 `3=STREAM` 이다.
> (같은 정정이 §8.1 에도 있다.)
>
> **② LOCKED 진입은 3경로가 아니라 1경로다.** 아래 화살표에 *"스테일 / write 연속 거부 /
> FAULT"* 셋이 적혀 있으나 코드에는 하나뿐이다:
>
> | 원안의 경로 | 실제 |
> |---|---|
> | 스테일 | → **IDLE** (+ arm off). §6.1.3 이 뒤집었다 — "슬라이더에서 손을 뗐다" 는 정상 조작이라 REARM 을 요구할 이유가 없다 |
> | write 연속 거부 | ✅ **이것뿐이다** — 50 tick(0.25초) 연속 거부 |
> | FAULT | **미구현.** `sys_state` 는 보고만 하고 LOCKED 를 만들지 않는다 |
>
> ⚠ 다만 **모터 fault 는 결과적으로 LOCKED 를 만든다** — ECU 가 fault 로 AUTO 를 벗어나면
> (`ESTOP_SW`) 모터 명령을 전부 거부하고, 그 거부가 50 tick 쌓여 위 1경로로 들어간다.
> 즉 "FAULT → LOCKED" 는 **설계된 경로가 아니라 증상을 통한 우회로**다. 그래서 LOCKED
> 사유 문자열에는 거부 사실만 적히고 진짜 원인(모터 fault)은 안 적힌다.
> → 결정 필요: FAULT 를 1급 경로로 구현할 것인가, 아니면 사유에 `ecu_sys_state` 를 실을 것인가.

상태 5개 + **`write_source` 필드**로 분리해 조합 폭발을 막는다.

```
        ┌──────┐  전 범위 READ 스캔 + (모드별) write init 검증 성공
        │ INIT │ ──────────────────────────────────┐
        └──┬───┘  실패 → 노드 종료(exit≠0)          │
           │                                       ▼
           │                                  ┌────────┐ ◀── 스테일 (+ arm off)
           │        goal 수락 / cmd_vel fresh  │  IDLE  │ ──┐ arm on + 스트림 fresh
           │        ┌────────────────────────▶ └────┬───┘   │      ┌────────┐
           │        │                              ▲│       └─────▶│ STREAM │
        ┌──▼────────┴─┐  완료·취소·guard 만료       ││ REARM        └────┬───┘
        │   RUNNING   │ ───────────────────────────┘│ (명시 호출)       │
        └──────┬──────┘                             │                  │
               │ RW write 연속 거부 50 tick          │                  │
               ▼                                    │                  │
        ┌──────────┐                                │                  │
        │  LOCKED  │ ───────────────────────────────┴──────────────────┘
        └──────────┘        (STREAM 에서도 같은 조건으로 들어온다)
```

| 상태 | write 값 | 의미 |
|---|---|---|
| `INIT` | (루프 시작 전) | 전 범위 READ 스캔 → shadow 동기화 → (모드별) write init |
| `IDLE` | **모드별** — §6.1.1 | 기본 상태 |
| `RUNNING` | `write_source` 가 정하는 값 | 프로파일 재생 |
| `STREAM` | 스트림 스냅샷 (§6.1.3) | 실시간 조작. **arm 게이트 필요**, 스테일이면 IDLE + arm off |
| `LOCKED` | **안전값 래치** | 원인 확인 + 명시적 `REARM` 없이는 못 나온다 |

**`write_source`** (RUNNING 중에만 유효):

| 값 | bridge_mode | 입력 | 스테일 정책 |
|---|---|---|---|
| `CMD_VEL` | project | `/carrier_cmd_vel` | 100ms 미수신 / 3s 0 수렴 → IDLE |
| `PROFILE` | control | `run_profile` action (사전 샘플링 배열) | tick 소진 → IDLE |
| `STREAM` | control | `/carrier/control/cmd_motor` (웹 슬라이더 + MPC) | 스테일 > timeout → **IDLE** (§6.1.3) |

`SETPOINT` 는 B5 로 `STREAM` 에 흡수되었다 (03 §6.2) — 웹 슬라이더와 MPC 는 브리지 입장에서 같은
스트림이고, HANDOFF 의 "단순 바 형태로 제어" 요구사항은 그 발신자 중 하나다.
**RUNNING 진입은 배타적** — 한 번에 하나의 `write_source` 만, 다른 소스 요청은 reject.

#### 6.1.3 `STREAM` 진입은 **명시적 arm** 을 요구한다 (신규, 2026-07-27)

`PROFILE` 은 goal 수락이, `CMD_VEL` 은 상위 노드 존재가 곧 의사표시다. 그런데 `STREAM` 은
초안대로면 **메시지가 도착하는 것만으로 RUNNING 이 된다.** 그러면:

- 노드를 띄운 직후 남아 있던 발행자(웹 탭, 죽다 만 MPC)가 **조작자 모르게 모터를 움직인다**
- `auto_mode` 를 바꾸는 등 설정 작업 중에도 스트림이 들어오면 그대로 나간다

→ **`STREAM` 은 `arm` 이 켜져 있을 때만 `RUNNING` 으로 간다.** 웹의 **run / stop 버튼**이 이것이다.

| | `arm=off` (stop) | `arm=on` (run) |
|---|---|---|
| `CmdMotor` 수신 | **무시** (구독은 하되 소비 안 함) | `write_source=STREAM` 으로 RUNNING |
| write 값 | §6.1.2 안전값 (POSITION 이면 `fb_position` 재시드) | 스트림 값 |
| 스테일 timeout | 해당 없음 | → `IDLE` (arm 은 **off 로 내린다**) |

- 진입점: `ControlConfig` op 7 `SET_STREAM_ARM` (03 §6.4). `IDLE` 에서만 on 가능
- **스테일 시 `LOCKED` 가 아니라 `IDLE`** 인 이유: §6.1.2 재시드가 IDLE 을 안전하게 만들었으므로,
  "슬라이더에서 손을 뗐다" 는 정상 조작에 `REARM` 을 요구할 이유가 없다. 초안이 `LOCKED` 였던 것은
  `SETPOINT`(timeout→IDLE) 를 `STREAM`(timeout→LOCKED) 에 흡수하면서 IDLE 쪽 정책이 사라진
  결과였고, 03 §6.2 는 같은 상황을 "안전 복귀" 라고 써서 **두 문서가 어긋나 있었다** — 여기서 정리한다
- `LOCKED` 는 여전히 유효하다. `write` 연속 거부·`FAULT` 등 **비정상** 경로가 그쪽이다.
  스테일은 정상 경로다

##### 운용 흐름 — 웹이 해야 할 일은 이것뿐이다 (2026-07-27 확정)

```
① 현재 자세 확인      ControlFeedback.fb_position 실시간 표시
② (선택) 원점 잡기     SET_ORIGIN 펄스 — run 누르기 **전에만** (§6.3 C-1)
③ 시작 위치 맞추기     슬라이더를 원하는 값에 놓는다 (아직 나가지 않는다)
④ run                arm=on → 슬라이더 값이 write_source=STREAM 으로 나가기 시작
⑤ 조작                슬라이더를 움직이면 모터가 실시간으로 따라온다
⑥ stop               arm=off → IDLE. 안전값(fb_position 재시드) 유지, 모터 정지
```

**진입 시 변위를 브리지가 검사하지 않는다 (기각 결정)**: `run` 순간 슬라이더가 현재 자세에서
멀면 모터가 그만큼 움직이는데, 이것을 `|슬라이더 − fb_position| > 허용치` 로 **거부하자는 안은
채택하지 않는다** (2026-07-27). ① 이 실시간으로 보이고 ③④ 가 분리되어 있으므로 조작자가 이미
판단할 수 있고, 허용치의 적정값을 정할 근거도 없다. **조작자 책임으로 둔다.**

> **웹의 범위는 여기까지다.** ①~⑥ 이상의 자동화·검증·보정을 웹에 넣지 않는다 (사용자 확정).
> 실험의 정밀한 파형은 프로파일(`RunProfile`)의 몫이고, 웹은 **눈으로 보고 손으로 미는 도구**다.
> 05 §4.1 의 "웹 편집기는 세그먼트 열을 내보낸다" 도 같은 선에 있다 — 웹은 만들고 보내는 데까지,
> 재생과 검증은 브리지가 한다.

`manual` 은 정기 write 가 없으므로 항상 `IDLE`(또는 `LOCKED`)에 머문다. 모터 명령은 슬롯으로 낸다.

### 6.1.1 `IDLE` 의 write 정책은 모드별로 다르다

`IDLE` 은 "안전값을 쓰는 상태"가 아니라 **"명령 소스가 없는 상태"** 다. 그 상태에서 write 를
계속 낼지 끊을지는 모드의 운용 성격이 정한다.

| `bridge_mode` | `IDLE` 에서 | ECU 결과 | 근거 |
|---|---|---|---|
| `control` | **안전값 write 유지** (§6.1.2) | `motor_on` 유지 (여자 유지) | 실험 중 언제든 즉시 명령이 나가야 한다. 여자를 껐다 켜면 첫 명령에 응답 지연·과도가 생겨 실험 데이터가 오염된다 |
| `project` | **write 중단** | `AUTO_TIMEOUT`(100ms) → `motor_on=0` | 주행 대기 시간이 길다. idle 전류를 끊어 발열·전력을 줄인다 |
| `manual` | 해당 없음 (정기 write 자체가 없음) | 슬롯 명령이 끊기면 동일하게 `motor_on=0` | — |

**`LOCKED` 는 두 모드 모두 write 중단**한다 — 래치의 목적이 "확실한 정지" 이므로 ECU 워치독까지
동원하는 것이 맞다.

##### 정지 워치독 4층 (2026-07-27 보강 — AK 층이 문서에 없었다)

"write 를 멈추면 모터가 선다" 는 문장 뒤에는 **독립된 타임아웃 4개**가 있다. 각각 다른 곳이
죽어도 아래 층이 받는다.

| 층 | 감시 대상 | 임계 | 발동 시 | 근거 |
|---|---|---|---|---|
| 1 | 브리지 FSM | 즉시 | `IDLE`/`LOCKED` 진입 → 안전값 또는 write 중단 | §6.1.1 |
| 2 | Orin 측 명령 가드 | 50ms | shadow 를 안전값으로 | `rd_system.h:32` 주석 |
| 3 | ECU `AUTO_TIMEOUT` | **100ms** | `cmd_write_tick` 미갱신 → `motor_on=0` | `rd_system.h:33` |
| 4 | **AK 모터 내부** | **200ms** | **CAN 명령 미수신 → 모터 자체 정지** | **사용자 설정값 (2026-07-27)** |

- **4층이 최후 방어선이다.** 브리지가 죽든 Orin 이 꺼지든 ECU 가 멈추든, CAN 프레임이 200ms 끊기면
  모터가 스스로 선다. 이 값은 **사용자가 AK 모터에 직접 설정한 것**이라 코드·문서 어디에도 없었다
- 임계가 `50 < 100 < 200` 으로 **단조 증가**하는 것이 중요하다. 상위 층이 먼저 반응하고 하위 층은
  상위가 죽었을 때만 뜬다. 순서가 뒤집히면 정상 동작 중에 하위 층이 오발동한다
- `control` 의 `IDLE` 이 write 를 유지하는 것(여자 유지)은 **3·4층을 의도적으로 안 건드리는 것**이다.
  안전값이 계속 나가므로 CAN 도 계속 흐르고, 그래서 `run` 을 누른 순간 응답 지연이 없다 (§6.1.2)
- 반대로 `project` 의 `IDLE`·양 모드의 `LOCKED` 는 write 를 끊어 **3층을 일부러 발동시킨다**

#### 6.1.2 `control` IDLE 의 "안전값" 은 `auto_mode` 가 정한다 (신규, 2026-07-27)

초안은 이것을 **"0A write 유지"** 라고 단위째 박아 두었다. `auto_mode` 에 VELOCITY/POSITION 이
추가되면(§3.2) 그 문장이 **위험해진다** — write 범위가 `cmd_position` 인데 0 을 쓰는 것은
**"원점으로 가라"는 명령**이다. 트랙이 200도 돌아간 상태에서 스틱을 놓으면 원점까지 되돌아간다.

| `auto_mode` | write 범위 | IDLE·LOCKED 안전값 | 뜻 |
|---|---|---|---|
| `CURRENT` | `164:16` | `0` | 토크 없음 |
| `VELOCITY` | `148:16` | `0` | 정지 |
| **`POSITION`** | `132:16` | **`fb_position` 재시드** | **현재 자세 유지** — 0 이 아니다 |
| `KINEMATIC` | `180:8` | `0` | 정지 (`project` 전용) |
| `DIRECT` | `128:52` | 모터별 `ctr_mode` 를 따름 | 위 규칙을 모터별로 적용 |

**재시드 규칙 (bumpless transfer)**:

```
IDLE 에 있는 동안 매 tick:  setpoint ← fb_position   (POSITION 계열만)
RUNNING 진입 순간:          그 setpoint 가 첫 명령이 된다 → 변위 0 → 튀지 않는다
```

- **매 tick 재시드**이지 latch 가 아니다. IDLE 중 외력으로 트랙이 밀려도 명령이 따라오므로
  `run` 을 누르는 순간의 변위가 항상 0 이다
- 측정 노이즈가 명령에 실려 드리프트하는 것 아니냐 — **IDLE 에서는 문제가 안 된다.** 명령이 실측을
  따라갈 뿐 실측이 명령을 따라가지 않으므로 폐루프가 닫히지 않는다. `RUNNING` 으로 넘어가는 순간
  재시드가 멈추고 조작자 값이 소스가 된다
- 이 규칙이 **§3.3 의 "범위 진입 전 shadow 소독" 을 겸한다.** `auto_mode` 를 POSITION 으로 바꾸면
  `cmd_position`(132:16)이 그때까지 **어느 프리셋에도 안 읽히던 자리**라 shadow 에 INIT 스캔 시점의
  낡은 값이 남아 있는데, 전환 직후 IDLE 재시드가 그것을 실측값으로 덮는다
- **재시드 대상은 브리지가 wire 로 내보내는 값이지 웹 슬라이더가 아니다.** 슬라이더는 조작자 소유이며
  `IDLE` 중에도 조작자가 원하는 위치에 놓아 둘 수 있다 (§6.1.3 운용 흐름). 브리지가 슬라이더를
  현재 자세로 끌고 가면 **조작자가 시작 위치를 미리 맞춰 둘 수 없다**
- 따라서 `run` 순간의 변위는 **조작자가 슬라이더를 어디에 두었는가**로 정해진다. 현재 자세 근처에
  두면 0 이고, 멀리 두면 그만큼 움직인다 — **그것이 조작자의 의도다** (§6.1.3)

#### `cmd_vel` 스테일 임계값의 정체

`project` 의 `RUNNING → IDLE` 전이 임계값이 바로 기존 두 파라미터다:

| 파라미터 | 기본 | 조건 | 막는 위험 |
|---|---|---|---|
| `cmd_vel_topic_timeout` | 0.1s | `/carrier_cmd_vel`·`/jeongae` 모두 미수신 지속 | **상위 노드 사망** — write 중단이 곧 ECU 에 보내는 정지 신호 |
| `cmd_vel_zero_timeout` | 3.0s | `cmd_vel` 이 0 에 수렴한 채 지속 (`|v| < 1e-3`) | **대기 중 모터 여자** — 발열·전력 |

현행 구현은 이것을 FSM 전이가 아니라 "50Hz WRITE 슬롯 skip" 이라는 애드혹 분기로 처리한다
(`rd_schedule.cpp:370`, `rd_bridge.cpp:267`). 재설계에서는 **`RUNNING → IDLE` 전이 + `IDLE` write
정책**으로 표현되어 동작은 같고 표현만 일반화된다.

> **함께 정리될 중복**: 현행에는 `GetRosInputs()` 의 **0.5s 워치독**(shadow 를 0 으로 덮어씀,
> `rd_bridge.cpp:296`)이 별도로 있다. 0.1s guard 가 먼저 걸리므로 이것은 `cmd_vel_guard_enable=false`
> 일 때만 의미가 있는 백업 경로다. FSM 통합 후에는 **`IDLE` 진입이 값을 0 으로 만드는 것을 겸하므로
> 0.5s 워치독은 제거**한다 (경로 이중화 해소).

### 6.2 `safe_stop` — "안전 정지 상태" 의 정의

```
safe_stop :=
      ControlState ∈ {IDLE, LOCKED}                       // 명령 소스가 안전값
  AND ∀ m ∈ active_motors : |fb_velocity[m]| < 5 RPM       // 물리적으로 멈춰 있다
  AND ∀ m ∈ active_motors : |fb_current[m]|  < 1.0 A       // 토크가 걸려 있지 않다
  AND rw_err == 0                                          // 직전 트랜잭션 정상
```

- `IDLE` 이어도 관성으로 아직 돌 수 있으므로 **속도·전류 실측 조건이 필수**다. 상태만 보고 판정하면
  회전 중에 `motor_mask` 를 바꾸는 사고가 난다.
- `LOCKED` 포함 이유: 안전값 래치 상태이므로 진단·복구 WRITE 가 가능해야 한다.
- 판정에 쓰는 값은 **직전 RW 트랜잭션의 read 스냅샷 하나뿐** — 여러 tick 을 섞지 않는다.

#### ⚠ `safe_stop` 을 요구하지 않는 것 — 정지 명령

`safe_stop` 을 모든 WRITE 에 걸면 **회전 중에 0을 쓰거나 ESTOP 을 거는 것도 차단된다**. 분리한다:

| 분류 | 대상 | 게이트 |
|---|---|---|
| **설정 변경 WRITE** | `motor_mask`(192) / `mode`(190) / `auto_mode`(188) / `ctr_mode` / `SET_ORIGIN` / raw / `REBOOT` | **`safe_stop` 필요** |
| **감속 방향 WRITE** | `soft_estop`(189)=0 / 명령값을 0으로 / 슬롯 RESET / action cancel | **무조건 허용** |

`safe_stop` 위반 시 응답에 **위반한 조건을 명시**한다 (`"거부: M2 fb_velocity=42.1 RPM (한계 5.0)"`).
원인 불명 실패를 만들지 않는다.

### 6.3 명령 삽입 규칙

세 종류를 완전히 다르게 취급한다.

#### (A) 정기 READ → **IDLE 전용** (R6)

RUNNING 중 프리셋 교체를 금지하는 이유 2가지:
1. 센서 데이터는 5ms 단위 변화에 민감하다 — 정기적 세그 교체는 시계열에 주기적 결손을 만든다
2. **200Hz 피드백 스트림이 불균일해진다** — 그 tick 의 메시지 필드 구성이 달라져 분석 코드가
   결손 필드를 다뤄야 한다. "1 트랜잭션 = 1 메시지" 원칙(00 §5.1 R5) 위반이며, 이쪽이 더 강한 근거다

→ **RUNNING 중 읽기 프리셋은 고정. 예외 없음.** `control` 프리셋의 `{16,17}` 이 이미 최소 진단
(sys_state·hw_reset·tick·proc_delta)을 상시 확보하므로 추가 정기 READ 가 필요하지 않다.

`IDLE` 에서는 `diag` 프리셋으로 교체하거나 슬롯 READ 를 자유롭게 쓴다.

#### (B) 임의 1회성 READ

| 방식 | 명령 손실 | 피드백 손실 | RUNNING |
|---|---|---|---|
| write 유지 + read 세그 교체 | **0** | 1 샘플 (필드 불일치) | ○ |
| 순수 READ 패킷으로 1 tick 대체 | 1 tick (5ms) | 1 샘플 | **✗ 거부** |

- 앞쪽은 write 가 끊기지 않으므로 실험 오염이 "피드백 1샘플 결손" 뿐이다. 발생 횟수를 action result 의
  `irregular_tick_cnt` 로 내보내고 `result.json` 에 기록해 분석이 감안할 수 있게 한다
- 요청 세그가 커서 예산(§5.5)을 넘으면 순수 READ 로 강등 → RUNNING 중에는 거부
- 동시 1건. in-flight 중 새 요청 즉시 거부 (큐잉 없음)
- `safe_stop` 불필요 — READ 는 모터에 영향이 없다

#### (C) WRITE

| 대상 | 방법 | tick 소모 |
|---|---|---|
| **in-span 지속** (현재 write 범위 안, 값을 유지) | shadow 수정 → 다음 RW tick 자연 반영 | **0** |
| **in-span 1회성 (펄스)** — §6.3.1 | shadow 에 **1 tick 만** 넣고 다음 tick 원복 | **0** |
| **out-of-span** (`192`/`190`/`188`/DEFINE 등) | RW 1 tick 을 일반 WRITE 패킷으로 대체 | **1** (§7.3) |
| `REBOOT` | 전용 패킷 + 3초 blackout | 1 |

#### (C-1) `in-span 1회성 WRITE` — 레벨 레지스터에 실린 엣지 명령 (신규, 2026-07-27)

`SET_ORIGIN`(AK mode 5) 이 이 분류를 만들게 한 사례다. **동작은 1회성인데 그것을 담는 그릇
(`ctr_mode`, addr 128)은 매 tick 전송되는 레벨 레지스터**라, 기존 두 분류 어디에도 안 들어간다:

- `auto_mode=DIRECT` 여야 `ctr_mode` 가 브리지 소유다 (§3) → 그러면 `128` 은 **in-span**
- in-span "지속" 으로 처리하면 shadow 의 5 가 **매 tick 나간다**

> **위험도 — 미확인 (실기 확인 필요)**: `SET_ORIGIN`(AK mode 5)이 매 tick 반복 수신될 때 AK 모터가
> 어떻게 반응하는지는 **확인된 바 없다.** 원점이 매번 다시 잡히면 트랙이 도는 중에 `cmd_position`
> 이 "직전 위치 기준 상대각" 이 되어 **위치 제어가 적분기처럼 굴러버린다.**
>
> 확인 방법이 있다: `fb_position`(addr 88)이 control 프리셋 안이므로, DIRECT + `ctr_mode=5` 를
> 몇 tick 유지하면서 `fb_position` 이 계속 0 으로 리셋되는지 보면 된다. **확인 전까지는 반복 전송을
> 하지 않는다** — 어느 쪽이든 1회성 장치가 필요하다는 결론은 같으므로, 이 확인은 06 검증 항목이지
> 설계 선행조건이 아니다.

**펄스 규약**:

1. 요청 수락 시 `safe_stop` 검사 (설정 변경 WRITE 분류 — §6.2)
2. shadow 의 해당 바이트에 값을 넣고 **`pulse_restore` 를 예약**한다 (원복값 = 직전 값)
3. **다음 tick 진입 시 무조건 원복.** `write_err` 여부와 무관 — 실패해도 두 번 보내지 않는다
   (엣지 명령을 재시도하면 의도가 두 번 실행된다)
4. 결과 확인은 **다음 트랜잭션의 read 로** 한다. `SET_ORIGIN` 이면 `fb_position`(addr 88, control
   프리셋 `{47,81}` 안)이 0 근처인지 본다
5. 동시 1건. in-flight 중 새 펄스 요청은 즉시 거부

**`SET_ORIGIN` 은 `manual` 전용이 아니다 (제약 해제)**: §4.1 과 03 §6.3 이 "`manual` 전용 1회성
셋업" 으로 적었으나, `control` 의 `IDLE` 에서도 필요하다 — 위치 실험의 기준점을 잡는 동작이기 때문이다.
`control` 은 커맨드 슬롯이 0개라(04 §2.4.1) `CommandSet` 경로가 없으므로, **이 펄스 경로가
`control` 에서 `SET_ORIGIN` 을 내는 유일한 수단**이다.

**자동화하지 않는다.** `auto_mode=POSITION` 진입 시 원점을 자동으로 잡는 안은 **기각**한다
(2026-07-27): 실험마다 기준이 달라져 기록 간 비교가 깨지고, 조작자가 의도하지 않은 순간에
원점이 바뀐다. 원점은 **사람이 웹에서 실시간 pose 를 보고 명시적으로 누르는** 동작이다.

게이트는 §6.2 분류를 따른다. `RUNNING` 중 설정 변경 WRITE 는 "먼저 abort 하세요" 안내와 함께 거부.

#### (D) 슬롯 만석 (`project`/`manual`)

기존 우선순위 규칙 유지: **빈 슬롯 중 최상단 배치 → 모두 차 있으면 최하위 우선순위 슬롯을
일시정지하고 자리 차용 → 처리 후 원래 슬롯 복귀.**

#### 요약 결정표

| 명령 | 슬롯 소비 | tick 대체 | `safe_stop` | RUNNING 중 |
|---|---|---|---|---|
| 정기 READ (A) — 같은 보드의 다른 구간 | ✗ (**프리셋 교체**) | ✗ | ✗ | **✗** |
| 정기 READ (A) — 다른 보드 (DPC/PCU) | ○ (IDLE) | ✗ | ✗ | **✗** |
| 1회성 READ (B) — 세그 교체 | ✗ | ✗ | ✗ | ○ (카운트) |
| 1회성 READ (B) — 예산 초과 | ✗ | ○ | ✗ | ✗ |
| in-span WRITE (C) | ✗ | ✗ | 분류별 | 감속만 |
| out-of-span WRITE / REBOOT (C) | ✗ | ○ | ○ | ✗ |

> **정정 (2026-07-27)**: 이전 표는 "슬롯 소비" 와 "tick 대체" 를 뒤섞어, out-of-span WRITE 가
> 슬롯을 쓰는 것처럼 적혀 있었다. 그러면 커맨드 슬롯이 0개인 `control` 이 자기 `ControlConfig`
> 서비스(`SET_ACTIVE_MOTORS`·`SET_AUTO_MODE`·`REBOOT`)를 실행할 수 없다는 모순이 된다.
> **둘은 다른 메커니즘이다** — 04 §2.4.1:
>
> - **슬롯** = *반복·지속* 명령의 거치대. 프레임에 물리적 칸이 있어야 매 주기 돌아온다
> - **tick 대체(OOS)** = *단발* 명령이 다음 tick 하나를 빌리는 것. 슬롯 조회 **이전**에
>   실행되므로(04 §7 ①) 프레임에 칸이 없어도 동작한다
>
> 그래서 슬롯을 소비하는 것은 **다른 보드를 정기적으로 읽을 때뿐**이고, 같은 보드의 다른 구간을
> 정기적으로 읽는 것은 **읽기 프리셋 교체**(04 §2.4.2, `control_cli config preset`, IDLE 전용)로
> 슬롯 없이 해결한다.

### 6.4 INIT 플로우

```
① RS485 개통 대기
② 전 범위 READ 스캔 — {0,256} 1 트랜잭션 (버퍼 256 확장 후, §9.2)
     목적: shadow == ECU 실값 보장 = **진실 원천 확보** (§7)
③ bridge_mode == manual 이면 여기서 IDLE 진입, 종료
④ write init — 순서 필수
     motor_mask(192) → auto_mode(188) → mode(190)=AUTO
   각 단계 WRITE 후 write_err 확인, 0.2s 간격 10회 재시도, 전부 실패 → 노드 종료(exit≠0)
⑤ 2차 소형 스캔 — {16,17} + {188,5}
     AUTO 진입 후 ECU 자가치유가 반영된 ctr_mode·sys_state 확정 (§7.2)
⑥ ControlState: INIT → IDLE. action/service 서버 오픈
```

- **②가 신규**다. 현재는 스캔 없이 write init 을 한다 → shadow 의 기본값이 ECU 실값과 다른 상태에서
  `WRITE_REG`(섀도 영역 전송)를 쓰면 의도치 않은 값이 나간다. 스캔이 이 구멍을 막는다
- **④의 순서가 안전의 핵심**: `auto_mode` 를 AUTO 진입 **전에** 확정해야 KINEMATIC 이 활성인 순간
  (`ctr_mode` 덮어쓰기)을 한 tick 도 거치지 않는다
- **⑤가 신규**다 (§7.2 참조). ②만으로는 AUTO 진입 후 ECU 가 자가치유한 `ctr_mode` 를 모른다
- `active_motors` 는 **실제로 전원·CAN 이 살아 있는 모터와 정확히 일치**해야 한다. 불일치 시 AUTO
  진입의 ESTOP 경유 프레임이 부재 모터로 CAN 송신되어 ACK 실패 → CAN fatal → FAULT(sticky)

---

## 7. shadow 권위 모델 (신규 — R4의 기반)

읽기 프리셋에서 read-back 세그를 뺄 수 있는 근거. **"상시 read-back" 을 "초기 스캔 + write ACK"
로 대체한다.**

### 7.1 규칙

| # | 규칙 |
|---|------|
| S1 | **INIT 전 범위 스캔**으로 shadow = ECU 실값 을 확보한다 (진실 원천의 출발점) |
| S2 | **write 범위 밖 shadow 는 절대 쓰지 않는다.** 그 자리에는 read 로 들어온 ECU 값만 존재한다 |
| S3 | write 직전 대상 바이트의 이전 값을 백업한다 (설정 레지스터만) |
| S4 | 응답 `write_err == NONE` → **커밋**. shadow 값이 곧 ECU 값이다 |
| S5 | 응답 `write_err != NONE` → **shadow 를 백업값으로 롤백** + 실패 사유 로그/응답 |
| S6 | read 세그로 들어온 값은 무조건 shadow 에 반영한다 (ECU 가 진실 원천인 영역) |

### 7.2 왜 read-back 이 불필요한가 — 용도별 검증

read-back 세그의 원래 용도는 2개였다. 둘 다 대체된다.

**용도 ⓐ 프로파일 수락 가드** ("활성 모터 전부 `ctr_mode==CURRENT` 인가")

판정 기준을 `ctr_mode` 에서 **`auto_mode` 로 한 단계 올린다**:

```
auto_mode == CURRENT  → 통과. ECU 가 ctr_mode 를 CURRENT 로 100Hz 자가치유하므로 보장된다
auto_mode == DIRECT   → shadow 의 ctr_mode 확인. bridge 소유 + write ACK 로 검증된 값이다
그 외                  → 거부 ("전류 프로파일은 auto_mode=current 또는 direct 에서만")
```

`auto_mode` 는 INIT ④에서 write ACK 로 확정되고, 이후 out-of-span write ACK 로만 바뀐다 → 신뢰 가능.

**용도 ⓑ `SET_CTR_MODE` 검증**

`SET_CTR_MODE` 는 `auto_mode=DIRECT` 에서만 유효하고(CURRENT 에서는 즉시 거부), DIRECT 에서
`ctr_mode` 는 in-span 이다 → 브리지가 쓰고 `write_err` 로 확인한다. read-back 불필요.

**남는 구멍 1개와 그 처리**

`auto_mode=CURRENT` 에서 ECU 가 `ctr_mode` 를 자가치유하는 순간을 브리지가 모른다. INIT ②의 스캔은
AUTO 진입 **전**이므로 `ctr_mode` 가 기본값(VELOCITY)으로 읽힌다.

→ **INIT ⑤ 2차 소형 스캔**(`{16,17}` + `{188,5}`)으로 AUTO 진입 후 값을 한 번 확정한다.
이후 `auto_mode` 가 바뀌지 않는 한 ECU 는 그 값을 유지하므로 상시 확인이 불필요하다.

> **testbed_spec §2.6-0 과의 관계**: 당시 하네스가 발견한 실패("ECU 는 VELOCITY 인데 가드가
> CURRENT 로 보고 goal 통과")의 원인은 **브리지가 write 범위 밖 shadow 를 낙관적으로 덮어쓴 것**이었다.
> 그때의 처방은 "read 세그로 매 tick 확인"이었으나, 근본 수정은 **S2(범위 밖 shadow 를 쓰지 않는다)**
> 다. S2 를 지키면 read-back 은 중복이다.

### 7.3 out-of-span WRITE 가 2 tick → 1 tick 으로 줄어든다

기존: WRITE tick + READ 검증 tick = 최대 2 tick.
신규: `write_err == NONE` 을 신뢰 → **1 tick**. 실패 시 shadow 롤백 + 즉시 실패 응답.

**예외 — `mode`(190)**: GPIO 스위치와 Orin 이 공동 소유하는 유일한 레지스터다. write 성공 후에도
사람이 스위치를 돌리면 바뀐다. 별도 read-back 대신 **`sys_state`(27)로 상시 교차 확인**한다
(`sys_state` 1=MANUAL / 2=AUTO). 모든 프리셋이 `sys_state` 를 포함하므로 추가 비용 0.

### 7.4 스트리밍 명령에는 롤백이 무의미하다

`cmd_current` 처럼 매 tick 갱신되는 값은 tick N 이 거부되어도 tick N+1 이 새 값을 쓴다.
롤백 대상이 아니다.

| 분류 | `write_err != NONE` 처리 |
|---|---|
| 스트리밍 명령 (`cmd_current` 등) | 롤백 없음. **연속 거부 streak 카운트만** → 50 tick(0.25s) 초과 시 `LOCKED` |
| 설정 레지스터 (단발) | shadow 롤백 + 실패 응답 + 사유 명시 |

---

## 8. 개명표 (Q6: testbed → control)

### 8.1 코드 심볼

| Before | After |
|---|---|
| `RdTestbed` | `RdControl` |
| `TestbedState` | `ControlState` |
| `rd_testbed.{hpp,cpp}` | `rd_control.{hpp,cpp}` |
| `TestbedWrite_t` | `ControlWrite_t` |
| `PublishTestbedFeedback()` | `PublishControlFeedback()` |
| `control_mode_` / `traction_test_mode_` | `bridge_mode_` (enum) |

> ⚠ **이 문단의 종전 서술은 §6.1.3 이 뒤집었다 (2026-07-29 정정).**
> 여기에는 *"`STREAM` 상태는 삭제되고 `write_source = STREAM` 으로 흡수된다"* 라고 적혀
> 있었으나, §6.1.3 이 **`STREAM` 을 arm 게이트를 가진 FSM 상태로 되살렸다.**
> 코드도 그쪽을 따른다 — `TestbedState::STREAM` 이 존재하고 `ControlFeedback.control_state`
> 는 `0=INIT 1=IDLE 2=RUNNING 3=STREAM 4=LOCKED` 다. **§6.1.3 이 정본이다.**

### 8.2 메시지 (패키지 이동 + 개명 동시)

| Before | After |
|---|---|
| `mgs01_base_msgs/msg/TestbedFeedback` | `mgs_tp_msgs/msg/ControlFeedback` |
| `mgs01_base_msgs/srv/TestbedConfig` | `mgs_tp_msgs/srv/ControlConfig` |
| `mgs01_base_msgs/action/RunProfile` | `mgs_tp_msgs/action/RunProfile` |
| `mgs01_base_msgs/msg/CommLatency` | **흡수 검토** → `ControlFeedback` 에 병합 (03에서 필드 단위 결정) |
| `mgs01_base_msgs/msg/CmdMotor` | `mgs_tp_msgs/msg/CmdMotor` |
| `mgs01_base_msgs/srv/CommandSet` | `mgs_tp_msgs/srv/CommandSet` |
| `mgs01_base_msgs/msg/JeonGae` | **유지** (프로젝트 전용) |

### 8.3 토픽·서비스·액션

| Before | After |
|---|---|
| `/carrier/testbed/feedback` | `/carrier/control/feedback` |
| `/carrier/testbed/comm_latency` | (흡수 검토 — 03) |
| `/carrier/testbed/config` | `/carrier/control/config` |
| `/carrier/testbed/run_profile` | `/carrier/control/run_profile` |

### 8.4 패키지·실행 파일

| Before | After |
|---|---|
| `testbed_cli` | `control_cli` |
| `comm_test_node` | `firmware_bridge_node` |
| `command_cli` | **삭제** (Q4) |
| `testbed_web` (계획) | `control_web` |

### 8.5 마이그레이션 비용 — 분석 파이프라인

기존 bag 은 `/carrier/testbed/feedback` + `mgs01_base_msgs/TestbedFeedback` 으로 녹화되어 있다.
`analysis/traction/` · `analysis/latency/` 디코더에 구버전 분기가 필요하다.

완화: 패키지 이동(Q5)만으로도 메시지 타입이 이미 깨진다. **개명을 같은 타이밍에 하면 파괴적
변경이 1회로 합쳐진다** — 나눠서 하면 디코더 분기가 2개 생긴다.

```
topic /carrier/testbed/feedback + mgs01_base_msgs/TestbedFeedback → legacy 경로
topic /carrier/control/feedback + mgs_tp_msgs/ControlFeedback     → 신 경로
```
`test_index.csv` 에 `schema` 열 추가로 bag 별 명시.

---

## 9. STM 작업 (⚠ 전부 CubeIDE 빌드 필요)

### 9.1 레지스터 재배치 — 주소 확정

HANDOFF 노트의 구조체 **순서는 맞지만 주석의 addr 번호에 오류**가 있다
(`rs485_proc_delta` 를 addr 29로 적었는데 `realtime_tick` 이 4바이트로 28~31을 점유한다).
Control task 읽기 세그 `26 7` 이 정확히 맞아떨어지는 배치는 다음과 같다:

```c
/* ===== [SYSTEM] addr 16~32 (17 bytes) ===== */
typedef struct __attribute__((packed)) {
    /* addr 16 */ uint8_t  degraded_cnt[8];   // 16~23
    /* addr 24 */ uint8_t  hw_error;          // 순서 변경 (구 26)
    /* addr 25 */ uint8_t  hw_fatal;
    /* addr 26 */ uint8_t  hw_reset;          // 순서 변경 (구 24)
    /* addr 27 */ uint8_t  sys_state;
    /* addr 28 */ uint32_t realtime_tick;     // 28~31
    /* addr 32 */ uint8_t  rs485_proc_delta;  // DIAG(228) 에서 이동 ← 노트는 29로 오기
} DATA_SYSTEM_t;                              // = 17 bytes
```

검산: `{26,7}` = `hw_reset`+`sys_state`+`realtime_tick(4)`+`proc_delta` = **7B** ✓ (HANDOFF 일치)
`{16,17}` = SYS 전체 = **17B** ✓ (HANDOFF 일치)

**영역 조정 — 하위 주소가 밀리지 않는다**:

| 영역 | Before | After |
|---|---|---|
| `REG_SYS` | 16, size 16 (16~31) | 16, size **17** (16~32) |
| `reserved0` | 32, size 10 (32~41) | **33**, size **9** (33~41) |
| `LOADCELL` 이하 | 42~ | **변경 없음** |
| `DIAG.rs485_proc_delta` | 228 | **삭제** (reserved 확장) |

→ `LOADCELL`(42) 부터 주소가 그대로다. 기존 bag·분석의 오프셋 가정이 깨지지 않는다.

효과: ① control 읽기 세그 감소 (`{27,5}`+`{228,1}` → `{26,7}`) ② 진단 필드가 sys 와 연속 (§5.6)
③ `hw_error`→`hw_fatal`→`hw_reset` 심각도 순 배치

> Orin `rd_register_ecu.hpp` 에 `static_assert` 가 있어 **불일치는 컴파일 타임에 잡힌다** — 안전망.

### 9.2 패킷 버퍼 확장 (R5) — **이월 버그 P8 의 근본 수정**

#### 원인 규명

`rd_uart.h:74` / `rd_uart.c:109`
```c
#define RX_BUFFER_SIZE  64          // DMA 링버퍼
rx_length = (tail - head + RX_BUFFER_SIZE) % RX_BUFFER_SIZE;
```

RW 요청의 와이어 크기 (= payload + 8):

| `auto_mode` | 요청 payload | 와이어 | RX 링버퍼 64 |
|---|---|---|---|
| `CURRENT` (write 164:16) | 1+24+2+16 = 43 | **51B** | ✓ |
| `DIRECT` (write 128:52) | 1+24+2+52 = 79 | **87B** | ✗ **오버런** |

87바이트가 64바이트 링버퍼에 들어오면 앞 23바이트가 뒤 23바이트에 덮이고, `% 64` 때문에
`rx_length` 가 **23으로 계산**된다. Length 필드는 82를 주장하는데 실제 23바이트뿐 → 파싱/CRC 실패
→ `RD_FATAL`. 이월 버그 "DIRECT 전환 크래시 (재현 2/2)" 와 정확히 일치한다.

**즉 P8 은 브리지 재시작 경합이 아니라 STM DMA 버퍼 크기 문제다.**

#### 변경할 상수 3개

| 상수 | 파일 | 현재 | 변경 | 근거 |
|---|---|---|---|---|
| `PACKET_DATA_BUF_SIZE` | `rd_comm_ecu.h:50` | 90 | **256** | Orin `MAX_DATA_LEN`=248 이미 지원. INIT 전 범위 스캔 1 트랜잭션화 |
| `RX_BUFFER_SIZE` | `rd_uart.h:74` | 64 | **256** | 최대 요청 95B 의 2.7배 — **P8 수정** |
| `TX_BUFFER_SIZE` | `rd_uart.h:75` | 128 | **272** | 256 payload + 프레임 8B + 여유 |

RAM 비용: UART 핸들당 `rx_buffer(256) + temp_buffer(256) + tx_buffer(272)` = 784B.
3채널(uart1 RC / uart2 RS485 / uart6 IMU) = **약 2.4KB** — STM32F446 128KB 대비 무의미.

#### 프로토콜 상한 ≠ 200Hz 예산

버퍼를 256으로 올려도 **상시 최대치를 쓰지 않는다.** 200Hz 안전은 §5.5 wire 예산 검증이 담당한다.
버퍼 확장의 목적은 ① P8 수정 ② INIT 스캔 단축 ③ 향후 제약 제거이며, 프리셋은 §5.3 대로 작게 유지한다.

### 9.3 `auto_mode` 확장

`ACTION_STATE_AUTO` 의 `ctr_mode` 자가치유 분기에 `VELOCITY(4)` / `POSITION(5)` 추가 (§3.2).

### 9.4 미커밋 이력

CAN SJW 1→3TQ + BS1/BS2 재배분 (2026-07-21 CAN 안정화) — ✅ `532ec69`(7/25)에 이미 커밋됨 (2026-07-29 확인). 이번
CubeIDE 작업 시 함께 커밋한다.

---

## 10. 02에서 결정할 것

| # | 질문 |
|---|------|
| A1 | `rd_schedule`(L1) → `rd_bridge`(L3) 역방향 의존을 어떤 인터페이스로 뒤집을지 (콜백 객체 / 관찰자 / 순수 데이터 반환) |
| A2 | `rd_bridge` 1162줄 분할 단위 — 후보: `rd_node`(셸) / `rd_telemetry`(발행) / `rd_control_api` / `rd_carrier_api` |
| A3 | `rd_command`(394줄)의 슬롯 관리와 jeongae 전개 시퀀스를 분리할지 |
| A4 | 스레드 소유권 최종안 — 현재 3개(스케줄 200Hz / ROS spin / 프로파일 action) 유지 여부 |
| A5 | 코어 계층을 rclcpp 없이 단위 테스트 가능하게 만들 범위 |

---

## 부록: 결정 사항 요약 카드

```
bridge_mode   : project | control | manual        (기동 고정. developer_mode 는 manual 에 흡수)
ecu.mode      : addr190  MANUAL | AUTO
ecu.auto_mode : addr188  NONE* | KINEMATIC(0) | CURRENT(1) | DIRECT(2)
                         | VELOCITY(4,신규) | POSITION(5,신규) | CONTROL(3,금지)
                         (*NONE = 브리지 쪽 write 범위 ∅, ECU 값 아님)
ControlState  : INIT → IDLE ⇄ RUNNING → LOCKED --REARM--> IDLE
write_source  : CMD_VEL | PROFILE | STREAM        (3개. SETPOINT 는 STREAM 에 흡수 — B5)
                STREAM 진입은 **명시적 arm**(웹 run/stop, op 7) 필요 — §6.1.3
                STREAM 스테일 → IDLE + arm off  (LOCKED 아님. LOCKED 는 비정상 경로 전용)
IDLE write    : control = 안전값 유지(여자 유지) / project = 중단(motor_on=0, 절전)
                LOCKED = 두 모드 모두 중단
                **안전값은 auto_mode 가 정한다** — §6.1.2
                  CURRENT 0A / VELOCITY 0RPM / POSITION = fb_position 매 tick 재시드
                  POSITION 에서 0 은 "원점으로 가라" 다. bumpless transfer 로 진입 변위 0
                project 의 RUNNING→IDLE 임계 = cmd_vel_topic_timeout(0.1s) / zero_timeout(3.0s)

명령 삽입 4종  : in-span 지속(tick 0) / **in-span 1회성=펄스(tick 0, §6.3 C-1)**
                out-of-span(tick 1 대체) / REBOOT(tick 1 + 3s blackout)
                펄스 = SET_ORIGIN 처럼 레벨 레지스터에 실린 엣지 명령. 1 tick 후 무조건 원복,
                       실패해도 재시도 없음(엣지를 두 번 실행하면 안 된다). manual 전용 제약 해제
shadow 소독    : 범위 진입 전 소독은 **전 auto_mode 전환에 적용** — §3.3
                → POSITION 은 fb_position 으로 (IDLE 재시드가 겸한다)

기동 거부      : auto_mode=control(3) / active_motors 형식 오류  — 이 2개뿐

슬롯 10칸 (project/manual, 50ms 프레임)
  odd×5 = ECU(100Hz) | DPC | PCU | CMD0 | CMD1 | CMD2 (각 20Hz)
control = 매 tick ECU RW(200Hz)

읽기 프리셋 (read-back 세그 없음)
  project      {16,17} {48,22} {86,42}   82B   1.40ms  (엔코더 제외)
  control_test {16,17} {42,6}  {88,40}   64B   1.20ms
  control      {16,17} {47,81}           99B   1.54ms (DIRECT 1.93ms)
  diag         {0,16}  {16,17} {224,32}  66B   1.23ms
  ※ 모든 프리셋이 {16,17}(SYS 전체) 포함 — NodeStatus 매핑 무분기 (04 §2.3)
  예산 검증: (요청+응답)×10.85µs ≤ 0.5×tick

shadow 권위    : INIT 전 범위 스캔 + write ACK. 범위 밖 shadow 는 쓰지 않는다
                 write_err==NONE → 커밋 / !=NONE → 설정은 롤백, 스트리밍은 streak 카운트
프로파일 가드   : auto_mode==CURRENT 통과 (ECU 보장) / DIRECT 는 shadow ctr_mode 확인

safe_stop = ControlState∈{IDLE,LOCKED} ∧ |vel|<5RPM ∧ |cur|<1A ∧ rw_err==0
  설정 변경 WRITE 만 요구. 감속 방향(0·ESTOP·cancel)은 무조건 허용

정기 READ = IDLE 전용. RUNNING 중에는 1회성 READ(세그 교체)만

STM: SYS 16~32(17B), proc_delta→addr32, LOADCELL(42) 이하 불변
     PACKET_DATA_BUF_SIZE 90→256 / RX_BUFFER 64→256 / TX_BUFFER 128→272  ← P8 수정
```
