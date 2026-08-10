# L2 — `policy/` : 제어 FSM · 커맨드 · 프로파일 · 전개 시퀀스

> **이 층이 하는 일**: "지금 무엇을 쓸 것인가", "지금 이 명령을 받아도 되는가" 를 결정한다.
> 상태를 **소유**하는 층이다. L3 는 서버 핸들만 갖고 상태를 갖지 않는다 —
> 그 규칙이 지켜지면 웹(`control_web`)이 붙을 때 같은 L2 를 그대로 재사용한다.
>
> **rclcpp 를 모른다.** 그래서 `test_control_fsm`·`test_sequence`·`test_idle_safe_value` 등이
> 노드 없이 돈다.

## 파일 목록

| 파일 | 역할 | 상태를 갖나 |
|---|---|---|
| `rd_control.{hpp,cpp}` | 제어 FSM · write 값 선택 · safe_stop · STREAM · 펄스 | ✅ |
| `rd_command.{hpp,cpp}` | 커맨드 슬롯 4개 · blackout · 우선순위 차용 | ✅ |
| `rd_command_catalog.hpp` | **의미 단위 명령 표** + 게이트 (순수 헤더·순수 함수) | ❌ |
| `rd_sequence.{hpp,cpp}` | jeongae 자동 전개 FSM + `ISlotHost` | ✅ |
| `rd_oos.{hpp,cpp}` | out-of-span 단발 WRITE 큐 | ✅ |
| `rd_profile.{hpp,cpp}` | YAML 파싱 · 검증 · 사전 샘플링 | ✅ (데이터) |
| `rd_profile_player.{hpp,cpp}` | 재생 진행 (인덱스) | ✅ (진행) |
| `rd_status.{hpp,cpp}` | GET_STATUS 정형 JSON 조립 | ❌ |

---

## 1. `RdControl` — 제어 FSM

### 1.1 핵심 불변식

> **RW 자체는 모든 상태에서 계속 돈다** (read 스트림 = 피드백 발행 유지).
> 상태가 결정하는 것은 오직 **write 소스** 뿐이다.

```
 INIT ──MarkInitDone()──▶ IDLE ──BeginProfile()──▶ RUNNING ──EndProfile()──▶ IDLE
                           │  ▲                                                ▲
             arm on + 신선  │  │ 스테일 → IDLE + arm off                        │
                           ▼  │                                                │
                        STREAM ┘                                               │
                           │                                                   │
                    Lock(reason) (어느 상태에서든)                              │
                           ▼                                                   │
                        LOCKED ──Rearm() (config REARM 전용)───────────────────┘
```

| 상태 | write 소스 | config 서비스 |
|---|---|---|
| `INIT` | (루프 시작 전) | **전부 거부** |
| `IDLE` | 안전값 (아래 §1.3) | 전부 허용 |
| `RUNNING` | 프로파일 샘플 | 거부 (실험 오염 방지 — 중단은 action cancel) |
| `STREAM` | 최신 `/carrier/control/cmd_motor` | 거부 (단 arm **off** 는 통과) |
| `LOCKED` | 안전값 (래치) | **REARM 만** |

`LOCKED` 는 "모터락" 아이디어의 구현이다 — 조용히 자동 복귀하지 않고 원인 확인 후
명시적 재무장을 요구한다. ECU 측 `mtr_lock`/`AUTO_TIMEOUT` 은 그 뒤의 최후방 방어선.

### 1.2 API

```cpp
// ── 주입 (L3 가 1회) ──
void Bind(RobotState_t* shadow, float clamp_max);   // ⚠ 빼먹으면 PrepareWrite 가 조용히 no-op
void SetLogger(ILogger*);  void SetClock(IClock*);  void SetStreamTimeout(double sec);

// ── 상태 ──
ControlState State() const;   bool IsRunning() const;
void MarkInitDone();                       // INIT → IDLE (1회성. 재진입 방지)
bool BeginProfile(uint32_t goal_id);       // IDLE 에서만 true — 동시 goal 불가가 여기서 걸린다
void EndProfile();                         // RUNNING → IDLE (완료/취소/에러 공통)
void Lock(const std::string& reason);      // 최초 사유 보존 (연쇄 래치 시 원인 유실 방지)
bool Rearm();                              // LOCKED → IDLE
std::string LockReason() const;
bool AcceptsConfig(bool is_rearm) const;

// ── write 값 ──
uint8_t AutoMode() const;  void SetAutoMode(uint8_t);
void SetCtrMode(int idx, uint8_t);  uint8_t CtrMode(int idx) const;
void SetProfileLimits(float lo, float hi);          // 재생 중 실효 클램프 (단위는 mode 가 정함)
void SetProfileSample(const float cur[4], float t, uint16_t seg);   // 재생기가 매 tick
ControlWrite_t SelectWrite();                        // 현재 상태의 write 소스 확정
void PrepareWrite();                                 // [스케줄 스레드] 섀도에 반영

// ── 안전 ──
bool SafeStop(std::string* why) const;
void NoteWriteErrStreak(uint64_t streak);            // ≥ kWriteErrLockStreak(50) → LOCKED
bool SetStreamArm(bool on, std::string* why);  bool StreamArmed() const;
void PushStreamCommand(const StreamCmd_t&);          // [L3 구독자 스레드]
bool RequestOriginPulse(std::string* why);  bool OriginPulsePending() const;
```

### 1.3 `PrepareWrite()` — **안전값은 auto_mode 가 정한다**

매 tick Encode 직전에 불려 섀도의 `cmd_motor` 영역을 채운다.

> 초안은 안전값을 "0A write 유지" 라고 **단위째 박아 뒀는데**, write 범위가
> `cmd_position` 일 때 0 을 쓰는 것은 **"원점으로 가라"** 는 명령이다.
> 2026-07-28 실기에서 실제로 그렇게 됐다 — `auto_mode=5` 로 바꾸는 순간
> 모터가 41.6도에서 0도로 슬루했다.

| auto_mode | write 범위 | 명령 중 | **안전값 (IDLE/LOCKED/INIT)** |
|---|---|---|---|
| CURRENT(1) | 164:16 | `clamp_a(clamp_prof(w))` | **0 A** — 토크 없음 |
| VELOCITY(4) | 148:16 | `clamp_prof(w)` | **0 RPM** — 정지 |
| POSITION(5) | 132:16 | `clamp_prof(w)` | **`fb_position`** — 현재 자세 유지 (매 tick 재시드) |
| DIRECT(2) | 128:52 | 모터별 `ctr_mode` 를 따름 | 모터별로 위 규칙 |
| KINEMATIC(0)/CONTROL(3) | — | (기동 검증이 막는다) | 아무것도 안 씀 |

**POSITION 의 매 tick 재시드가 latch 가 아닌 이유**: IDLE 중 외력으로 트랙이 밀려도 명령이
따라오므로 RUNNING 진입 순간의 변위가 항상 0 이다 (bumpless transfer).
IDLE 에서는 명령이 실측을 따라갈 뿐 실측이 명령을 따라가지 않아 폐루프가 닫히지 않으므로
측정 노이즈가 드리프트를 만들지 않는다.
이 규칙이 "범위 진입 전 shadow 소독" 도 겸한다.

**클램프 두 겹**

```cpp
clamp_a(v)     // ±clamp_max_ [A]. **전류에만** 적용 — RPM·deg 에 걸면 단위가 다른 값을 자른다
clamp_prof(v)  // 재생 중 [prof_lo_, prof_hi_]. 비대칭 (position 의 관절 가동범위)
```

**DIRECT 분기의 함정**

```cpp
const uint8_t mode = ctr_mode_[i];      // 모터별 ctr_mode 가 단위를 정한다
cm.ctr_mode[i] = mode;
switch (mode) {
    case CTR_MODE_CURRENT: case 2:  cur = clamp_a(clamp_prof(...)); break;
    case CTR_MODE_VELOCITY:         vel = clamp_prof(...);          break;
    case CTR_MODE_POSITION:         pos = clamp_prof(...);          break;
    default:                        break;    // ESTOP(0)/SET_ORIGIN(5) — 전 채널 0
}
```

> 구 코드는 DIRECT 에서 명령 중이면 무조건 CURRENT 로 취급해 `w.current[i]` 를
> 단위와 무관하게 `cmd_current` 에 꽂았다. position 프로파일이면 **각도가 암페어로 나갔고**,
> `clamp_a`([A])로 잘려 90도 샘플이 30A 상한까지 갔다.
> `AcceptsAutoMode` 는 바로 그 조합(DIRECT + ctr_mode=POSITION + mode:position)을
> 수락하므로 **도달 가능한 경로**였다.

### 1.4 `SelectWrite()` — 상태별 소스

```cpp
struct ControlWrite_t {
    float    current[4];        // 이름은 이력. 단위는 auto_mode 가 정한다
    uint32_t goal_id;           // 0 = 없음 — 피드백 태그
    float    profile_time;
    uint16_t segment_index;
};
```

`UpdateStreamStateLocked()` 가 매 tick 여기서 불린다 — **상태 전이는 이 지점에서만 일어난다.**

STREAM 의 값 선택은 DIRECT 만 예외다:

```cpp
if (am == AUTO_MODE_DIRECT)   // 모터별 ctr_mode 가 단위를 정한다
    w.current[i] = ctr_mode_[i]==POSITION ? stream.position[i]
                 : ctr_mode_[i]==VELOCITY ? stream.velocity[i] : stream.current[i];
else
    w.current[i] = am==AUTO_MODE_POSITION ? stream.position[i]
                 : am==AUTO_MODE_VELOCITY ? stream.velocity[i] : stream.current[i];
```

### 1.5 STREAM / arm (01 §6.1.3)

```cpp
struct StreamCmd_t {
    uint8_t ctr_mode[4];    // ⚠ 예약 필드 — 담기만 하고 아무도 읽지 않는다
    float   position[4];    // [deg]
    float   velocity[4];    // [RPM]
    float   current[4];     // [A]
    double  stamp;          // 수신 epoch [s] — **메시지 header.stamp 가 아니다**
};
```

**arm 을 요구하는 이유**: STREAM 을 "메시지가 도착하면 RUNNING" 으로 두면, 노드 기동 직후
남아 있던 발행자(웹 탭, 죽다 만 MPC)가 **조작자 모르게 모터를 움직인다.**

- `SetStreamArm(true)` 는 **IDLE 에서만** 가능. 켜는 순간 `stream_cmd_.stamp = 0` 으로
  무효화한다 — 스테일 값이 남아 있으면 arm 하자마자 옛 명령이 나간다.
- `SetStreamArm(false)` 는 **언제나 허용** — 감속 방향이다.
- 스테일(`stream_timeout_`, 기본 0.1s) 이면 IDLE 로 내리고 **arm 도 함께 off** 한다.
  조작자가 손을 뗀 것을 시스템이 기억하면 안 된다.
- LOCKED 가 아니라 IDLE 인 이유: "슬라이더에서 손을 뗐다" 는 **정상 조작**이라 REARM 을
  요구할 이유가 없다.

**스테일 판정 기준은 수신 시각이다** — 발행자 시계가 틀어져 있으면(브라우저·다른 호스트)
`header.stamp` 기반 판정은 통째로 무의미해진다. "언제 만들어졌나" 가 아니라
"우리가 언제 받았나" 가 워치독의 질문이다.

### 1.6 `SafeStop()` — 설정 변경 WRITE 의 게이트

```
① ControlState ∈ {IDLE, LOCKED}
② 직전 RW write 거부 스트릭 == 0
③ 활성 모터 전부: |fb_velocity| < 5 RPM  and  |fb_current| < 1 A
```

**상태만 보고 판정하면 회전 중에 설정을 바꾸는 사고가 난다** — IDLE 이어도 관성으로 아직
돌 수 있다. 판정에 쓰는 값은 **직전 RW 트랜잭션의 read 스냅샷 하나뿐**이다 (여러 tick 을
섞지 않는다). 위반 시 `why` 에 **위반한 조건을 명시**한다 — 원인 불명 실패를 만들지 않는다.

### 1.7 SET_ORIGIN 펄스 (01 §6.3 C-1)

**동작은 1회성인데 그것을 담는 그릇(`ctr_mode`, addr 128)은 매 tick 전송되는 레벨 레지스터**다.
"지속" 으로 처리하면 섀도의 5 가 매 tick 나가고, 원점이 매번 다시 잡히면
위치 제어가 적분기처럼 굴러버린다.

```
NONE ──RequestOriginPulse()──▶ ARMED ──PrepareWrite 1회──▶ SENT ──다음 tick──▶ NONE(원복)
```

**규약: 다음 tick 에 무조건 원복한다.** `write_err` 여부와 무관 —
엣지 명령을 재시도하면 의도가 두 번 실행된다. 결과 확인은 다음 read 의 `fb_position` 으로.

조건: `auto_mode == DIRECT` (ctr_mode 가 bridge 소유여야 펄스가 유지된다) + `SafeStop()`.
**자동화하지 않는다** — POSITION 진입 시 자동 원점은 기각됐다 (실험마다 기준이 달라져
기록 간 비교가 깨진다). 사람이 실시간 pose 를 보고 누르는 동작이다.

### ⚠ 락 순서

```
shadow_->state_mutex  →  mutex_  →  ctr_mutex_
```

`PrepareWrite` 가 이 순서로 잡는다. **반대로 잡는 경로가 생기면 즉시 데드락**이다.
`SafeStop` 이 `mutex_` 를 스코프로 먼저 놓고 `state_mutex` 를 잡는 것은 이 규약 때문 —
붙여 쓰면 안 된다.

---

## 2. `RdCommand` — 커맨드 슬롯 4개

### 2.1 구조

```cpp
constexpr int      CMD_NUM_SLOTS        = 4;      // 번호 낮을수록 우선순위 높다
constexpr uint8_t  CMD_SLOT_AUTO        = 255;    // 빈 슬롯 자동 선택
constexpr uint16_t CMD_DURATION_FOREVER = 0;
constexpr uint16_t CMD_DURATION_ONCE    = 1;
constexpr uint16_t CMD_DURATION_MAX_SEC = 100;
constexpr uint32_t kOnceMaxAttempts     = 40;     // **시간이 아니라 시도 횟수**
```

> **once 만료가 시도 횟수인 이유**: REBOOT 후 blackout 이 3초인데 종전 once timeout 은
> 벽시계 2초였다. blackout 중에는 한 번도 시도되지 않는데 시계는 흘러,
> REBOOT 직후 예약한 명령이 **시도 0회로 죽었다.**

```cpp
struct CommandRequest_t {          // CommandSet.srv 와 동일한 평면 구조
    uint8_t  slot;         // 0~3 또는 255(auto)
    uint8_t  action;       // 0=RESET / 1=SET
    uint8_t  target_id;    // TARGET::{ECU,DPC,PCU}
    uint8_t  cmd;          // cmdcat::CMD_*  ★ 의미 단위 명령
    std::vector<uint8_t> args;         // WRITE 계열: args[0] = 값
    uint16_t start_addr;   // CMD_RAW_* 전용
    uint16_t data_len;     // CMD_RAW_READ 전용
    std::vector<uint8_t> data;         // CMD_RAW_WRITE 전용
    uint16_t duration;
};
```

### 2.2 스레드별 진입점

| 스레드 | 함수 |
|---|---|
| **ROS** | `HandleRequest()` — 검증 후 슬롯에 SET/RESET |
| **스케줄** | `GetSlotTask(slot, &task)` — 발사할 태스크를 꺼낸다 (false = 이번 차례 skip) |
| **스케줄** | `ReportResult(slot, ret)` — 결과 보고 |
| **스케줄** | `IsTargetBlackedOut(target_id)` |
| **ROS(조회)** | `SlotSnapshot()`, `LastReadSnapshot()` |
| **스케줄** | `TickAutoSequence()` → `seq_.Tick()` 위임 |

### 2.3 `GetSlotTask` 의 판정 순서 (중요)

```
① !active           → false
② blackout 중       → false   (시도 카운트 안 올림)
③ once && attempts >= 40  → 포기 로그 + 슬롯 해제 → false
④ duration>=2 && 경과 초과 → 만료 해제 → false
⑤ s.attempts++      ← 여기까지 왔으면 **실제로 발사된다**
⑥ inst 별 TaskConfig_t 생성
```

`CmdInst` 4종 → 실제 트랜잭션:

| CmdInst | TaskConfig_t |
|---|---|
| `WRITE_REG`(0) | `{tid, WRITE, start_addr, data_len}` — **섀도 값**을 그대로 보낸다 |
| `WRITE_DATA`(1) | 사용자 값을 섀도에 memcpy 후 `{tid, WRITE, start_addr, size}` |
| `READ`(2) | `read_span` 있으면 멀티세그 READ, 없으면 단일 구간 |
| `REBOOT`(3) | `{tid, REBOOT, 0, 0}` — duration 은 once 로 강제 |

### 2.4 `ReportResult`

- **REBOOT 성공** → 해당 보드 **3초 blackout** + `is_connected = false` + 슬롯 해제
- **once 성공** → READ 면 hex 덤프 로그 + `last_read_` 기록 → 슬롯 해제
- **once 실패** → 재시도 (해제 안 함). 만료 판정은 `GetSlotTask` 가 한다
- **forever / N초** → 성공 시 `err_streak=0`, 실패 시 `++`, 25회마다 WARN (5Hz 기준 5초)

**`last_read_` 를 남기는 이유** (07 §2 Tab3): 레지스터 표를 그리려면 어느 바이트가 신선한지
알아야 한다. 섀도는 언제나 값을 갖고 있으므로, 이것 없이 그리면
**한 번도 읽은 적 없는 자리의 0 이 방금 읽은 값처럼 보인다.**

```cpp
struct LastRead_t {
    bool valid; uint8_t target_id;
    ReadSpanEntry spans[kMaxReadSegs]; uint8_t count;
    double age_s;              // 스냅샷 시점 기준 경과 — **값 자체는 담지 않는다**
};
```
값을 담지 않는 이유: 질의 시점의 섀도가 언제나 더 신선하다.

### 2.5 우선순위 차용 (`AcquireAutoSlotLocked`)

자동 시퀀스가 슬롯을 빌릴 때:

```
빈 칸 중 가장 위 → 있으면 그것
없으면 → 최하위(slot 3) 를 saved_slot_ 에 백업하고 비운 뒤 차용
        끝나면 ReleaseAutoSlotLocked 가 복귀시킨다 (start_time 은 재개 시점부터 재계산)
```

`is_auto` 슬롯은 사용자 SET/RESET 이 **거부**된다.

---

## 3. `rd_command_catalog.hpp` — 의미 단위 명령 표

### 3.1 왜 있는가

종전 `CommandSet.srv` 는 `start_addr`/`data_len`/`data[]` 를 그대로 받았다.
그 위에 *"raw 는 manual 전용"* 게이트만 얹혀 있었고, 그 결과
**control 에서는 레지스터를 읽을 방법이 아예 없었다.**
읽기는 아무것도 바꾸지 않으므로 막을 이유가 없었는데, 막힌 것은 "raw 주소" 라는 표현 방식
때문이었다.

→ **주소를 아는 주체를 브리지 하나로 좁힌다.** 호출자는 "무엇을"(`CMD_READ_MOTOR`) 만
말하고, 그것이 어느 구간인지는 이 표가 안다.

### 3.2 표

```cpp
enum class CmdKind : uint8_t { READ, WRITE1, REBOOT, RAW_READ, RAW_WRITE };

struct CmdDef {
    uint8_t cmd; const char* name; CmdKind kind;
    const ReadPreset* read;   // READ 계열의 구간
    uint16_t waddr;           // WRITE1 계열의 주소
    bool manual_only;         // raw 만 true
    bool needs_safe_stop;
};
```

| cmd | 이름 | kind | 구간 / 주소 | manual만 | safe_stop |
|---|---|---|---|---|---|
| 0 | `read_sys` | READ | `{16,17}` | | |
| 1 | `read_motor` | READ | `{88,40}` | | |
| 2 | `read_sensor` | READ | `{42,44}` (로드셀+IMU+엔코더 연속) | | |
| 3 | `read_diag` | READ | `{0,33}+{224,32}` | | |
| 4 | `read_all` | READ | `{0,33}+{42,151}+{224,32}` = **216B** | | |
| 13 | `set_soft_estop` | WRITE1 | 189 | | ✅ |
| 14 | `set_use_lpf` | WRITE1 | 191 | | ✅ |
| 20 | `reboot` | REBOOT | — | | ✅ |
| 30 | `raw_read` | RAW_READ | 호출자 지정 | ✅ | |
| 31 | `raw_write` | RAW_WRITE | 호출자 지정 | ✅ | ✅ |

> **`read_all` 이 예약 구간을 빼는 이유**: 256B 를 통째로 읽으면 응답이 `err(1)+256 = 257B` 로
> `MAX_DATA_LEN`(248)을 넘어 한 트랜잭션에 안 들어간다. RSVD0(33:9)·RSVD1(193:31)은
> 어차피 표시할 것이 없으므로 빼면 **216B 로 한 번에 들어간다.**

> **`CMD_SET_MOTOR_MASK`/`CMD_SET_MODE`/`CMD_SET_AUTO_MODE`/`CMD_SET_ORIGIN` 은 없다.**
> `ControlConfig` 가 같은 넷을 갖고 있고, 그쪽에는 in-span/out-of-span 처리와 IDLE 게이트,
> write 범위 전환 시 shadow 소독까지 붙어 있다. 여기 같은 이름을 또 만들면
> **auto_mode 를 바꾸는 길이 둘이 되고 게이트가 다르다** — 슬롯 경로는 소독을 하지 않으므로
> 조용히 더 약한 길이 된다. 표에 없는 것이 곧 "여기 없다" 는 뜻이 되게 두는 편이,
> 있는데 거부되는 것보다 정직하다.

### 3.3 게이트 — **순수 함수 하나**

```cpp
struct GateResult { bool ok; const char* why; };
inline GateResult Gate(uint8_t cmd, bool is_manual, bool safe_stop, uint8_t arg0);
```

rclcpp 도 FSM 도 모르므로 조합을 전부 유닛 테스트로 고정할 수 있다 (`test_b6_catalog`).
그리고 이것이 **서비스 쪽 게이트**다 — 클라이언트에만 있는 가드는 가드가 아니다.

**예외 하나**: soft ESTOP **작동**(`arg0 == 0`)은 감속 방향이라 `safe_stop` 없이도 허용한다.
멈추라는 명령을 "안 멈춰 있어서" 거부하면 게이트의 목적과 정반대가 된다.

---

## 4. `RdSequence` — jeongae 자동 전개 FSM

### 4.1 왜 `rd_command` 에서 떼어냈나

그 파일은 두 가지를 하고 있었다: ① 커맨드 슬롯 관리 ② 전개 시퀀스.
**②는 ①의 사용자이지 일부가 아니다.** 섞여 있으면 슬롯 로직을 고칠 때 시퀀스가 딸려 오고,
시퀀스를 테스트하려면 슬롯 전체를 띄워야 한다.

### 4.2 `ISlotHost` — 시퀀스가 슬롯에 요구하는 전부

```cpp
class ISlotHost {
    virtual bool PostAutoWriteTo(uint8_t target, uint16_t addr, uint8_t value) = 0;
            bool PostAutoWrite(uint16_t addr, uint8_t value);   // ECU 단축형
    virtual bool AutoCommandDone(bool* ok) const = 0;
    virtual void SetCmdVelPaused(bool) = 0;   virtual bool IsCmdVelPaused() const = 0;
    virtual bool DpcSysState(uint8_t* out) const = 0;   // ★ 신선도를 반드시 함께 준다
};
```

> `DpcSysState` 가 `bool` 을 돌려주는 이유: DPC 섀도는 0으로 초기화되는데
> `sys_state == 0` 은 `CTRL`(정상 기본값)이라, **한 번도 안 읽은 상태와 "정상 대기 중" 이
> 값으로 구분되지 않는다.** 값만 돌려주면 시퀀스가 읽지도 않은 0 을 보고
> "이미 도착했다" 고 판정한다. (같은 함정을 IMU 에서 이미 한 번 밟았다.)
>
> 신선도 기준은 `dpc.comm.is_connected` — DPC 와 성공한 트랜잭션이 한 번이라도 있었는가.
> `enable_dpc_read` 가 꺼져 있으면 false 로 남는다 (의도한 대로).

### 4.3 상태 9개

```
IDLE ─trigger(+unlock)─▶ ESTOP_SET(ECU 189=0) ─▶ DPC_STATE_CHECK
                                                       │
                            DPC_DEPLOY(target=INIT) ◀──┘
                                    │
                            DPC_WAIT_CAMERA (sys_state == WAIT(5))
                                    │
                            CAMERA_ACTION  ⚠ **카메라 미연결 — 통과만 한다**
                                    │
                            DPC_RETRACT(target=ASCEND_1)
                                    │
                            DPC_WAIT_RETRACT (FINISH(8) 또는 CTRL(0))
                                    │
                            ESTOP_RELEASE(ECU 189=1) ─▶ IDLE + lock ON
```

- `PostDpcTargetLocked` 는 `dpc::IsWritableTarget()` 로 **FSM 중간 상태 쓰기를 막는다**
  (CTRL/HOLD/INIT/ASCEND_1 만 허용).
- `WaitDpcStateLocked` 는 셋을 **같이** 본다: ① 신선도 ② `STATE_ERROR` ③ 타임아웃.
  하나라도 빠지면 매달리거나 거짓으로 통과한다.
- `DPC_WAIT_RETRACT` 가 FINISH 와 CTRL 을 **둘 다** 받는 이유: FINISH 후 DPC 가 CTRL 로
  자동 복귀하므로 폴링이 FINISH 를 스쳐 지나갈 수 있다.
- `AbortLocked` — soft ESTOP 이 걸린 상태면 해제 시도 후 종료. **`lock_` 을 켜 재트리거를 막는다.**
- 시퀀스 종료 시에도 `lock_` ON — 재전개는 `/carrier/jeongae_lock` 로 unlock 해야 한다.

### ⚠ `kWaitTicksMax = 150` 의 실제 값

헤더 주석은 "5Hz tick 기준 30초" 라고 적혀 있지만, `RdSchedule::RunLoop` 의 project/manual
분기는 `command_->TickAutoSequence()` 를 **매 tick(200Hz)** 부른다.
따라서 실제 대기 상한은 **약 0.75초**다. 시퀀스를 손볼 때 이 값부터 확인할 것.

---

## 5. `RdOos` — out-of-span 단발 WRITE

RW write 범위(128~179) **밖** 레지스터(`motor_mask` 192 / `mode` 190 / `auto_mode` 188)는
패킷에 끼워넣지 못하므로 **RW 1 tick 을 일반 WRITE/READ 패킷으로 대체**해 처리한다.
IDLE 은 write 값이 안전값이라 1~2 tick 대체가 무해하다는 것이 근거다.

```cpp
enum class OosPhase : uint8_t { NONE, WRITE, READ, DONE };
struct OosStep_t { OosPhase phase; uint16_t addr; uint8_t value; };

bool Request(uint16_t addr, uint8_t value, std::string* msg);  // [service 스레드] 완료까지 대기
bool TakeStep(OosStep_t* step);                                // [스케줄 스레드] 매 tick 첫머리
void ReportResult(bool tx_ok, uint8_t readback);               // [스케줄 스레드]

static constexpr int kVerifyTicks = 10;  kVerifyTimeoutMs = 50;
```

**제약 4개**

1. IDLE 에서만, **동시 1건** — in-flight 중 새 요청은 즉시 거부 (큐잉 없음).
   대기시켰다가 뒤늦게 반영되면 호출자가 언제 적용됐는지 알 수 없어 실험 기록이 흐려진다.
2. 대체된 tick 은 read 스냅샷이 없다 → **그 tick 의 피드백 발행 skip** (보간 금지).
   대신 `OnIrregularTick()` 으로 구멍을 센다.
3. WRITE 실패 시 재시도 없이 다음 tick 정상 RW 복귀 (재시도는 호출자 몫).
4. 검증은 WRITE tick + READ tick, **최대 2 tick**.

타임아웃이어도 `in_flight_` 를 반드시 해제한다 — 안 하면 영구 블록.

---

## 6. 프로파일 — `RdProfile` + `RdProfilePlayer`

### 6.1 왜 둘로 나눴나 — **스레드가 다르다**

| | 스레드 | 소요 |
|---|---|---|
| `RdProfile` — 파싱·검증·사전 샘플링 | action 콜백 | 수십 ms 무방 |
| `RdProfilePlayer` — 재생 진행 | 200Hz 스케줄 | 배열 인덱싱만, 수 µs |

같은 클래스에 두면 락 범위 설계가 어려워진다. **데이터(샘플 배열)와 진행(인덱스)을 분리한다.**

### 6.2 `RdProfile`

```cpp
static constexpr int    kTickHz      = 200;
static constexpr double kDt          = 1.0/200;
static constexpr double kMaxDuration = 3600.0;      // [s]
static constexpr size_t kMaxTicks    = 720000;      // 펼치기 **전에** 이 값으로 검사한다

enum class Mode : uint8_t { CURRENT = 0, VELOCITY, POSITION };

bool LoadFromYaml(const std::string& yaml, uint8_t active_mask,
                  float global_max_current, std::string* err);
bool SampleAt(size_t tick, float out[4], uint16_t* seg) const;   // 연산 없음
bool AcceptsAutoMode(uint8_t auto_mode, const uint8_t ctr[4], uint8_t mask, std::string* err) const;
```

**YAML 스키마**

```yaml
name: my_experiment
mode: current | velocity | position        # 없으면 current
seed: 42                                   # 없으면 시각 기반 (Seed() 로 반드시 회수된다)
limits:
  max_abs: 5.0                             # 대칭 [-5, 5]
  range: [-30, 90]                         # 비대칭 — position 은 **필수**
  max_current: 5.0                         # deprecated 별칭, mode: current 전용
  slew_rate: 20.0                          # [단위/s] — 위반은 성형이 아니라 **reject**
motors:
  m1:
    - {type: ramp,  duration: 2.0, from: 0, to: 3}
    - {type: hold,  duration: 1.0, value: 3}
  m2:
    - {type: sine,  duration: 5.0, amp: 2, freq: 1.0, offset: 0}
```

**세그먼트 9종**

| type | 필수 키 | 검증 |
|---|---|---|
| `hold` | `duration`, `value` | |
| `ramp` | `duration`, `from`, `to` | 마지막 tick 이 정확히 `to` (히스테리시스 실험용) |
| `step` | `duration`, `from`, `to`, `t_step` | `0 < t_step < duration` — 아니면 step 이 아니라 hold |
| `stair` | `values[]`, `step_duration` | values 1~1000, step_duration > 0 |
| `sine` | `duration`, `amp`, `freq` (+`offset`) | `0 < freq ≤ 25` (주기당 8샘플이 하한) |
| `chirp` | `duration`, `amp`, `f0`, `f1` (+`offset`) | f0/f1 0~25 Hz |
| `prbs` | `duration`, `low`, `high`, `bit_duration` | `bit_duration ≥ 5ms`, `low ≠ high` |
| `noise` | `duration`, `mean`, `std` | `std > 0`, **`mean±4σ` 가 실효 한계 안** |
| `custom` | `samples[]` (+`rate`, `interp`) | `1 ≤ rate ≤ 200`, `interp ∈ {linear, nearest}` |

**설계 규칙 4개** (여기가 이 파일의 요점이다)

1. **자동 성형 없이 거부한다.** `slew_rate` 위반은 몰래 고쳐 재생하지 않는다 —
   프로파일이 곧 실험 기록이므로 고치면 기록과 실제가 어긋난다.
2. **`noise` 는 slew 검사에서 면제**된다 (경계 tick 포함). 백색 가우시안 잡음은 정의상
   tick 간 델타가 무제한이라, 두 규칙을 함께 지키면 noise 세그먼트가 **사실상 항상 거부**된다.
   면제한 tick 수는 `SlewExemptTicks()` 로 `result.json` 에 남는다 —
   그 구간은 레이트 제한이 없었다는 사실을 분석이 알아야 한다.
3. 대신 **진폭 쪽을 `mean±4σ` 가드가 지킨다.** 클램프가 분포의 꼬리를 잘라내면
   **실제 재생된 분포가 YAML 이 선언한 정규분포와 달라진다** — 시스템 식별 입력이 선언과
   다르면 동정 결과가 틀리는데, 그 사실이 기록 어디에도 남지 않는다.
4. **200Hz 초과 `custom` 은 거부**한다. 브리지가 조용히 다운샘플하면
   "재생된 것과 기록된 것이 달라진다". 변환은 내보내는 쪽 책임.

**`custom` 의 기본 보간이 `linear` 인 이유**: `nearest` 는 rate 가 낮을수록 큰 계단
불연속을 만든다. 50Hz 로 그린 매끄러운 곡선이 200Hz 재생에서 4 tick 마다 튀는 계단이 되고,
전류 지령이면 그대로 토크 충격이다. `slew_rate` 와 함께 쓰면 그 계단이 위반으로 잡혀
**사용자가 그리지도 않은 이유로 reject** 된다. (`rate==200` 이면 두 방식 결과가 같다.)

**패딩 규칙**: 전체 길이 = 가장 긴 모터. 짧은 쪽은 0 패딩, 미지정 활성 모터는 전 구간 0
(미지정 = "끄고 싶다" 가 아니라 "0 을 유지" 라는 뜻).

**`AcceptsAutoMode` — 수락 규칙 2단**

```
mode: current  → auto_mode 가 CURRENT(1) 이거나 DIRECT(2)
mode: velocity → auto_mode 가 VELOCITY(4) 이거나 DIRECT(2)
mode: position → auto_mode 가 POSITION(5) 이거나 DIRECT(2)

DIRECT 일 때만 ctr_mode 까지 내려간다 — 그때는 ctr_mode 가 bridge 소유라 신뢰 가능하다.
```

> 구 가드는 "활성 모터 전부 `ctr_mode == CURRENT`" 를 요구했다. v1 player 가 전류 전용이던
> 시절의 규칙이며, velocity/position 프로파일을 통째로 막는다.
> **판정 기준을 ctr_mode 에서 auto_mode 로 한 단계 올린 것**이 지금의 규칙이다.
>
> ⚠ 순서 주의: **프로파일을 먼저 파싱**해야 `mode` 를 알 수 있으므로 `Load()` 가 이 검사보다
> 앞선다. 구 코드는 반대였고, 그래서 프로파일 내용과 무관한 고정 규칙일 수밖에 없었다.

### 6.3 `RdProfilePlayer`

```cpp
void SetClampMax(float a);                       // L3 가 BridgeConfig 에서 1회 주입
bool Load(const std::string& yaml, uint8_t mask, std::string* err);   // [action 스레드]
uint32_t Begin();                                // goal_id 채번 + 인덱스 초기화
void Activate();                                 // 이 시점부터 스케줄 tick 이 샘플을 소비
size_t End();                                    // 소비한 tick 수 반환
void Tick(RdControl* control);                   // [스케줄 스레드] 매 tick — 인덱싱만
void Progress(size_t* tick, bool* done) const;   // [action 스레드] 5Hz 피드백용
```

`Tick()` 은 `active_ && !done_` 일 때만 `control->SetProfileSample(...)` 하고 인덱스를 올린다.
마지막 샘플을 넘기면 `done_ = true` — 정리는 action 실행부가 한다.

---

## 7. `rd_status` — GET_STATUS 정형 JSON

### 7.1 왜 자유 함수인가

조립을 rclcpp 밖에 두면 **노드 없이 유닛 테스트할 수 있다.**
스키마는 계약이므로 값이 아니라 **키의 존재와 타입**을 테스트로 고정한다 (`test_status_json`).

> 종전 응답은 `"state=IDLE motor_mask=0x01 ctr_mode=[4,1,1,1] ..."` 라는 한국어 섞인
> 문장이었고, `control_web` 이 그걸 정규식으로 뜯어 쓰고 있었다.
> 조작 UI 를 그 위에 올릴 수 없었던 이유가 이것이다.

### 7.2 규칙 4개

1. **모든 키가 항상 존재한다.** 값이 없으면 `null` — 키 유무로 분기하게 만들지 않는다.
2. enum 은 **문자열**이다. 정수 원본이 필요하면 별도 키를 둔다 (`ecu_sys_state`).
3. `safe_stop_detail` 은 `safe_stop == false` 일 때만 문자열.
4. `stamp_valid == false` 면 `stamp_quality_ms`·`drift_ppm` 은 `null`.

### 7.3 변환 함수

```cpp
std::string StatusToJson(const StatusSnapshot_t& s);
const char* CtrModeName(uint8_t);        // estop/current/current_brake/velocity/position/set_origin
const char* AutoModeJsonName(uint8_t);   // kinematic/current/direct/control/velocity/position/none
const char* WriteSourceName(uint8_t);    // none/cmd_vel/profile/stream
const char* EcuModeName(uint8_t);        // manual/auto
const char* TargetName(uint8_t);         // ecu/dpc/pcu
std::string JsonEscape(const std::string&);
```

> 숫자는 `snprintf("%.*f")` 로 직접 찍는다 — `std::to_string(double)` 은 소수점이 로케일을
> 따라가 `,` 가 나올 수 있고, 그러면 **JSON 이 깨진다.** `NaN`/`inf` 는 `null` 로 나간다.
>
> `JsonEscape` 를 빼면 **거부 사유 하나가 응답 전체를 깨뜨린다** (한글과 따옴표가 섞여 온다).
> UTF-8 본문은 그대로 통과시키고 제어문자만 `\u` 이스케이프한다.
