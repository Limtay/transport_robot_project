# L1 — `sched/` : 200Hz 루프와 슬롯 테이블

> **이 층이 하는 일**: 5ms 마다 깨어나 RS485 트랜잭션을 **정확히 1건** 한다.
> 그 1건이 무엇인지는 **표**(`rd_slot_table.hpp`)가 정하고, "지금 해도 되는가" 는
> 루프 안의 게이트가 정한다.
>
> 이 스레드가 곧 `main()` 이다. SCHED_FIFO 80 / CPU core 11.

## 파일 목록

| 파일 | 역할 |
|---|---|
| `rd_slot_table.hpp` | **표.** 프레임·슬롯 정의 + 컴파일 타임 검산 (순수 헤더) |
| `rd_schedule.{hpp,cpp}` | 200Hz 루프 · 표 굽기 · INIT 플로우 · 트랜잭션 실행 |
| `rd_txn_queue.hpp` | 락프리 SPSC 링버퍼 (발행 분리용, 순수 헤더) |
| `rd_telemetry_sink.hpp` | `ITelemetrySink` — L1 이 정의하고 L3 가 구현 |

---

## 1. 슬롯 테이블 — 스케줄을 **데이터**로

### 1.1 왜 표인가

종전에는 `tick%2`, `odd_idx%2`, `(odd_idx-1)/2` 같은 산술로 매번 되짚었다.
스케줄을 바꾸려면 그 식을 읽어내야 했고, 주기가 몇 Hz 인지는 세어 보기 전엔 알 수 없었다.

```cpp
tick_in_frame = tick_count % frame.ticks
slot          = frame.slots[tick_in_frame]
```

### 1.2 슬롯이 답하는 것은 넷이다

**①②를 한 enum 에 섞지 않는다** (`ECU_RW`, `ECU_READ`, `DPC`… 하면 조합이 곱셈으로 불어난다).

```cpp
enum class SlotId   : uint8_t { EMPTY, ECU, DPC, PCU, COMMAND };   // ① 누구에게
enum class SlotInst : uint8_t { NONE, READ, WRITE, RW };           // ② 무엇을

enum class ReadSrc  : uint8_t { NONE, FIXED, PRESET };   // ③ 무엇을 읽는가
struct ReadSpan { ReadSrc src; const ReadPreset* fixed; ... };

enum class WriteSrc : uint8_t { NONE, FIXED, AUTO_MODE };// ④ 무엇을 쓰는가
struct WriteSpan { WriteSrc src; uint16_t addr, len; ... };

struct SlotDef { SlotId id; SlotInst inst; ReadSpan read; WriteSpan write; uint8_t cmd_index; };
```

- `ReadSrc::FIXED` — 표가 명시한 구간 묶음 (`const ReadPreset*` 를 **가리키기만** 한다)
- `ReadSrc::PRESET` — ECU 전용. **런타임에 선택된 읽기 프리셋**을 따른다
- `WriteSrc::AUTO_MODE` — ECU 전용. `auto_mode` 가 6종 중 런타임에 고른다

> ③④가 슬롯 안에 있는 것이 핵심이다. 프레임이 프리셋 하나를 공유하면
> **DPC·PCU 슬롯이 무엇을 읽는지 표현할 방법이 없다.**

### 1.3 프레임

```cpp
constexpr uint8_t kMaxFrameTicks = 40;

struct FrameDef {
    const char* name;
    uint8_t     ticks;                    // 반복 주기 [tick]. 5ms/tick
    SlotDef     slots[kMaxFrameTicks];
    uint64_t    user_slot_mask;           // bit t == 1 → tick t 를 커맨드가 훔칠 수 있다

    constexpr const SlotDef& At(uint64_t tick) const;
    constexpr bool  UserSlotAt(uint64_t tick) const;
    constexpr uint8_t CountOf(SlotId) const;              // 검산용
    constexpr uint8_t CountOf(SlotId, SlotInst) const;
    constexpr uint8_t UserSlotCount() const;
    constexpr bool    AllValid() const;
    constexpr uint16_t MaxRespPayload() const;            // 예산 검증의 최악값
};
```

> ⚠ **언제 훔쳐도 되는가는 표가 정하지 않는다.** 표는 "이번에 무엇을 할 차례인가" 만
> 답한다. RUNNING/STREAM 중 금지 같은 판단은 루프의 게이트가 한다.

### 1.4 프레임 4개

**`kProject` — 10 tick = 50ms**

```
tick  0      1        2      3        4      5        6      7        8      9
     DPC   ECU RW    PCU   ECU RW   Cmd0  ECU RW   Cmd1  ECU RW   Cmd2  ECU RW
     20Hz  100Hz     20Hz            (양보 가능: 4·6·8)
```

- ECU RW 는 `kPresetProject` 를 읽으면서 `cmd_system`(180) 8B 를 쓴다 (`cmd_lin/ang_vel`)
- `user_slot_mask = (1<<4)|(1<<6)|(1<<8)`

> **종전 40칸(200ms)에서 무엇이 달라졌나**: 센서 READ(100Hz) + cmd_vel WRITE(50Hz) +
> sys READ(10Hz) 세 갈래를 **ECU RW×5 한 갈래**로 합쳤다. ECU 트랜잭션이 200ms 당
> 32건 → 20건으로 **줄었고**, cmd_vel 은 50Hz → 100Hz 로 **올랐다.**

**`kManual` — 10 tick, 배치는 같고 ECU 가 READ**

`user_slot_mask = 0x3FF` — **10칸 전부** 조작자가 덮어쓸 수 있다.
`slots[]` 는 "아무것도 안 넣었을 때의 기본 동작" 일 뿐이다.

> RW 로 바뀐 뒤로는 **게이트로 write 만 뺄 수가 없다** (한 패킷이라 write 를 막으면 읽기까지
> 사라진다). 그래서 manual 을 별도 프레임으로 분리했다 — 종전엔 `kManual = kProject` 였다.

**`kControl` / `kControlRead` — 40 tick, 매 tick ECU RW(또는 READ)**

```cpp
constexpr uint64_t kControlUserMask = 1ULL << 39;    // tick 39 한 칸만
```

프레임을 40칸으로 편 이유는 **양보 상한을 표로 표현하기 위해서**다.
1칸 프레임에 마스크를 세우면 "매 tick 양보 가능" 이 되어 `forever` 커맨드 하나가
제어를 통째로 굶긴다. 40칸 중 1칸이면 상한이 표에서 바로 읽힌다:

> 최대 5Hz(200ms 주기) · 제어 tick 손실 2.5% · 5ms 결손
> (AK CAN timeout 200ms 대비 40배 여유)

### 1.5 `ActionFor` — 쓰기가 막혔을 때

```cpp
enum class SlotAction : uint8_t { NORMAL, READ_FALLBACK, SKIP };

constexpr SlotAction ActionFor(const SlotDef& s, bool block_write) {
    if (!block_write || !s.write.Has()) return SlotAction::NORMAL;
    if (s.inst == SlotInst::RW)         return SlotAction::READ_FALLBACK;
    return SlotAction::SKIP;
}
```

**순수 함수로 빼 둔 이유**: 실기에서 구분이 안 된다. 폴백이 걸려 READ 가 나가든,
게이트가 안 먹어 RW 가 나가든 트랜잭션 수는 똑같이 1이라 heartbeat 로는 판별할 수 없다.
그래서 단위 테스트가 닿는 곳에 둔다 (`test_slot_table`).

RW 를 통째로 버리지 않는 이유: **센서 시계열에도 구멍이 나기 때문**이다.

### 1.6 컴파일 타임 검산

표 아래에 `static_assert` 25줄이 있다. 표를 고치면 **빌드가 먼저 막는다.**

```cpp
static_assert(frames::kProject.ticks == 10, "project 프레임은 10 tick = 50ms");
static_assert(frames::kProject.CountOf(SlotId::ECU, SlotInst::RW) == 5, "ECU RW 는 100Hz");
static_assert(frames::kProject.CountOf(SlotId::ECU, SlotInst::READ) == 0,
              "project 의 ECU 읽기는 전부 RW 에 흡수됐다");
static_assert(frames::kProject.CountOf(SlotId::EMPTY) == 0, "빈 tick 이 있다");
static_assert(frames::kProject.UserSlotCount() == 3, "커맨드 용량은 3");
static_assert(frames::kControl.UserSlotCount() == 1, "control 의 양보는 프레임당 1칸");
static_assert(frames::kProject.MaxRespPayload() <= kMaxRespPayload, "응답 예산 초과");
// … manual·control 도 같은 방식
```

> **wire 예산을 사람이 더해 적지 않는다.** 04 §2.5 가 지적한 사고(43 을 손으로 옮겨 적어
> payload 와 wire 를 섞었다)가 바로 그 계산을 사람이 했기 때문에 났다.

---

## 2. `RdSchedule` — 루프

### 2.1 생성자 — 의존성 9개 + 콜백 2개 + 어댑터 3개

```cpp
RdSchedule(RdComm* comm, RdMap* map, RobotState_t* state,
           ITelemetrySink* sink, RdCommand* command,
           const BridgeConfig& cfg, RdOos* oos, RdProfilePlayer* player, RdControl* control,
           std::function<void(bool with_profiles)> on_init_done = nullptr,
           std::function<bool()> skip_cmd_write = nullptr,
           ILogger* log = nullptr, IClock* clock = nullptr, IRunGate* gate = nullptr);
```

L2 객체 3개(`oos_`·`player_`·`control_`)를 **직접** 잡는다 — 노드를 거칠 이유가 없었고,
그 경유가 곧 역방향 의존이었다.

### 2.2 생성자가 하는 일 — **표 굽기**

200Hz 스레드는 인덱싱만 해야 하므로, 기동 시 표를 `TaskConfig_t` 배열로 미리 굽는다.

```cpp
TaskConfig_t project_task_[kMaxFrameTicks];        // project 프레임 tick 별
TaskConfig_t project_task_read_[kMaxFrameTicks];   // 같은 tick 의 write 뺀 READ (Q3 폴백)
TaskConfig_t manual_task_[kMaxFrameTicks];
TaskConfig_t task_control_read_[ecu::kPresetCount];              // auto_mode:none 용
TaskConfig_t task_control_[ecu::kPresetCount][kIdxModeCount];    // (프리셋 × auto_mode)
std::atomic<uint8_t> read_preset_{0};
```

`kIdxModeCount` 는 4다: `kIdxCurrent`(164:16) / `kIdxDirect`(128:52) /
`kIdxVelocity`(148:16) / `kIdxPosition`(132:16).

즉 **`3 프리셋 × 4 모드 = 12개` 를 기동 시 전부 만들어 둔다.**
교체는 atomic 인덱스만 바꾼다 — 200Hz 루프가 읽는 데이터를 건드리지 않는다.

```cpp
const TaskConfig_t& SelectControlTask(uint8_t auto_mode) const {
    const uint8_t pi = read_preset_.load(std::memory_order_relaxed);
    switch (auto_mode) {
        case ecu::AUTO_MODE_DIRECT:   return task_control_[pi][kIdxDirect];
        case ecu::AUTO_MODE_VELOCITY: return task_control_[pi][kIdxVelocity];
        case ecu::AUTO_MODE_POSITION: return task_control_[pi][kIdxPosition];
        default:                      return task_control_[pi][kIdxCurrent];   // 안전측
    }
}
```

### 2.3 `MainLoopStart()` — 유일한 입구

```cpp
int MainLoopStart();      // 프로세스 종료 코드를 반환. 호출한 스레드에서 그대로 돈다
void Stop();
```

1. **파라미터 오류를 통신 시도 전에 판정한다** — 오타 하나 때문에 "Waiting for USB..." 만
   찍히며 매달려 있는 상황을 만들지 않는다. `bridge_mode`/`read_preset` 오타는
   **모드와 무관하게** 막는다 (기본값으로 떨어뜨리면 모터가 도는 모드인지가 갈린다).
2. `ApplyRtScheduling(pthread_self())` — SCHED_FIFO 80 + CPU core 11.
   권한 없으면 **WARN 후 일반 우선순위로 계속**한다.
3. `SupervisorLoop()` → `RunLoop()` 이 `RD_OK` 아니면 1초 뒤 재시도.
   단 `init_fatal_` 이면 **재시도하지 않는다** (설정 오류는 반복해도 같은 결과).
4. 반환: `init_fatal_ ? 1 : 0`.

### 2.4 `Initialize()` — 포트 열기

```
comm_->Stop()                     ← 복구 재진입 시 깨진 fd/카운터 강제 리셋
while (gate_->Ok()) { if (comm_->Init(&packet_obj_) == RD_OK) break; 1초 대기 }
```

### 2.5 INIT 플로우 (`InitControl`) — control 전용

**순서가 안전의 핵심이다.**

| 순서 | 레지스터 | 값 | 왜 이 순서인가 |
|---|---|---|---|
| ① | `motor_mask`(192) | `cfg_.active_motor_mask` | AUTO 전에 확정 → 의도하지 않은 트랙이 한 tick 도 안 돈다 |
| ② | `auto_mode`(188) | `cfg_.auto_mode_param` | AUTO 전에 확정 → KINEMATIC(기본값 0)이 활성인 순간을 안 거친다 |
| ③ | `mode`(190) | `MODE_AUTO`(1) | RC 없이 레지스터 write 로 AUTO 진입 |

②를 빠뜨리면 KINEMATIC 이 살아 있는 동안 ECU 가 100Hz 로 `ctr_mode` 를 VELOCITY 로
덮어써 bridge 의 CURRENT write 와 경쟁한다.

각 단계는 `WriteVerifyByte()` — **WRITE 후 같은 주소 READ 로 read-back 검증**.
`kInitMaxRetry = 10` 회 × `kInitRetryIntervalMs = 200ms`. 전부 실패하면 노드 종료(exit≠0).

```cpp
uint8_t* RegBytePtr(uint16_t addr);   // 화이트리스트: mode(190) / motor_mask(192) / auto_mode(188)
                                      // 대상이 아니면 nullptr — 조용히 엉뚱한 곳을 쓰지 않도록
```

> 매 시도마다 섀도를 기대값으로 되돌린다 — 직전 시도의 READ decode 가 섀도를 ECU 실값으로
> 덮었을 수 있어서, 빠뜨리면 2회차부터 엉뚱한 값을 쓴다.

②가 끝나면 `control_->SetAutoMode(am)` 로 write 범위 셀렉터를 확정하고,
비-DIRECT 면 `AutoModeForcedCtrMode(am)` 로 `ctr_mode` 섀도를 맞춘다
(안 하면 status 가 `auto_mode=5(POSITION)` 인데 `ctr_mode=[1,1,1,1](CURRENT)` 로 보고한다).

### 2.6 모드별 기동 분기 (`RunLoop` 앞부분)

| 조건 | 하는 일 |
|---|---|
| `IsReadOnlyControl()` | `SetAutoMode(AUTO_MODE_NONE)` + `MarkInitDone()` + `on_init_done(false)` |
| `IsControl()` | `InitControl()` 3단 검증 → 실패 시 `init_fatal_` + exit≠0. 성공 시 `on_init_done(true)` |
| `IsManual()` | 자동 설정 **전무**. `MarkInitDone()` + `on_init_done(false)` |
| `IsProject()` | 로그만. **FSM 은 INIT 에 머문다** |

> **`MarkInitDone()` 을 빼면 조작 입구가 통째로 막힌다.** FSM 이 INIT 이면
> `AcceptsConfig` 가 전부 거부한다. 견인 실험(`auto_mode:none`)에서 실제로 그랬고,
> `cli status` 가 안 되고 웹 Tab3 이 죽고 bag 전체가 `control_state=INIT` 로 기록됐다.

> `SetAutoMode(none)` 을 함께 하는 이유: 이 값이 피드백의 `auto_mode` 로 나가고
> `result.json` 에 남는다. 기본값(CURRENT)을 두면 **쓰지 않는 런이 "전류 명령을 쓴 런" 으로
> 기록된다.**

### 2.7 루프 본체 — tick 하나

```cpp
next_cycle += period;                        // period = 5ms
std::this_thread::sleep_until(next_cycle);
wake_us = now - next_cycle;                  // 순수 wake latency (스케줄 지연)
```

**활성 프레임 선택**

```cpp
const FrameDef& active_frame =
    control_mode ? (read_only_control ? frames::kControlRead : frames::kControl)
  : cfg_.AutoConfiguresEcu() ? frames::kProject
                             : frames::kManual;
```

**양보 게이트**

```cpp
const bool user_slot_open =
    active_frame.UserSlotAt(tick_count_) &&
    control_->State() != ControlState::RUNNING &&
    control_->State() != ControlState::STREAM;
```

막는 것은 **진행 중인 실험**뿐이다. INIT·IDLE·LOCKED 는 막지 않는다.
> ⚠ INIT 을 막으면 `auto_mode:none` 에서 양보가 **영영 열리지 않는다** —
> 그 경로는 `InitControl()` 을 타지 않기 때문이다.
>
> `State()` 는 양보 tick 에서만 묻는다 (단축 평가) — 200Hz RT 루프에서 매 tick FSM 락을
> 잡을 이유가 없다.

**`RunUserSlot(&ret_val)`**

```cpp
for (int i = 0; i < CMD_NUM_SLOTS; i++) {           // 슬롯 0→3 = 우선순위 높은 순
    if (!command_->GetSlotTask(i, &task)) continue;
    *ret_val = ExecuteTask(task, &tx_res);
    command_->ReportResult(i, tx_res);
    return true;
}
return false;    // 실행할 것이 없으면 이 tick 은 평소대로 제어에 쓰인다
```

> 커맨드가 없을 때의 동작은 **종전과 바이트 단위로 같다.** 훔친 tick 만 제어가 빠진다.

**모드별 트랜잭션** — [README §4](README.md#4-한-tick-의-전-과정) 의 그림 참조.
control 분기의 순서가 중요하다:

```cpp
sink_->OnWriteErrTotal(map_->RwWriteErrTotal());
player_->Tick(control_);           // 사전 샘플 1개 → FSM
control_->PrepareWrite();          // FSM 이 고른 write 값을 섀도에 반영
ExecuteTask(SelectControlTask(control_->AutoMode()), &tx_res);
if (tx_res == RD_OK) {
    sink_->OnRwErr(map_->LastRwErr());
    sink_->OnTxn(last_txn_);       // ★ 먼저 — 시계 추정기를 갱신한다
    sink_->OnFeedback();           // ★ 나중 — 그 결과 캐시를 읽는다. 뒤집으면 한 tick 늦는다
    control_->NoteWriteErrStreak(map_->RwWriteErrStreak());
}
```

**주기 판정**

```cpp
elapsed_us = now - next_cycle;                    // wake latency + 처리시간
proc_us    = elapsed_us - wake_us;                // 스파이크 출처 분리
if (time_elapsed > 5 * period)  → RD_FATAL (Supervisor 로)
else if (time_elapsed > period) → next_cycle 재설정 + exceeded_cnt_++ + sink_->OnLateTick()
```

> `OnLateTick` 이 세는 것은 **"없어진 tick" 이 아니라 "늘어난 시간"** 이다.
> 루프는 밀린 만큼 따라잡지 않고 시간축을 뒤로 미루는데, 프로파일의 시각은 여전히
> `tick * kDt`(공칭 5ms)로 계산된다. 10초 프로파일이 10.3초에 걸쳐 재생돼도 기록에는
> 2000 tick 이 t=0~10.0s 로 남는다 — 이 카운터가 그 차이를 아는 유일한 수단이다.

### 2.8 Heartbeat — 400 tick(2초)마다

```
====== [RdSchedule Heartbeat] ======
 Ticks : 12455
 Timing: avg 1832 us / max 4210 us | over-period 3/400 (0.8%)
 Spike : wake_max 210 us / proc_max 4180 us  <- 스파이크 출처
 I/O   : clear_max 30 / write_max 1650 / read_max 2400 us
 Comm  : Tx 12455 / Rx 12453 (Loss: 2)
 Nodes : ECU[ON]  DPC[OFF]  PCU[OFF]
====================================
```

평균 처리시간이 예산(`kBudgetUs = 4000`)을 넘으면 tail spike 가 아니라 **전형 케이스가
무거운 것** → WARN. 출력 후 구간 통계를 전부 리셋한다.

> RT 루프 안에서 per-tick 동기 콘솔 로깅은 그 자체로 지터를 만든다. 그래서 카운트만 하고
> 여기서 요약한다.

### 2.9 `ExecuteTask` — 트랜잭션 1건

```cpp
RD_RET ExecuteTask(const TaskConfig_t& config, RD_RET* tx_result = nullptr);
```

반환값은 **루프 제어용**(`RD_FATAL` 이면 Supervisor 로), `tx_result` 는
**"이번 트랜잭션이 성공했는가"** 다. 둘을 나눈 이유는 개별 실패가 곧 재접속은 아니기 때문.

```
Encode → Clear(flush) → t_req 기록 → Write → Read(2,2) → t_resp 기록 → Decode
                                                      ↑ 단계별 최대 소요를 stat_*_max_ 에 적재
```

`last_txn_` (`TxnTiming_t`) 을 채우고, `valid` 는 **응답 + Decode 둘 다 성공**했을 때만 true.
`kWireOverheadBytes = 8` (Header2 + ID1 + Len2 + Inst1 + CRC2).

### 2.10 `SetReadPreset` — 런타임 프리셋 교체

```cpp
bool SetReadPreset(uint8_t id, std::string* why);
uint8_t ReadPresetId() const;
```

**IDLE 판정은 호출자(config 서비스)의 몫**이고 여기서는 id 범위만 본다 —
없는 프리셋으로 바꿨다고 응답하는 일이 없어야 한다.

성공하면 `sink_->SetReadPreset(ecu::kPresets[id])` 로 텔레메트리에 알린다.
**이걸 빼면 낡은 섀도 값이 신선한 값처럼 계속 발행된다.**

> 메서드 이름이 `ReadPresetId` 인 이유: `ReadPreset` 은 타입 이름이라 멤버 함수에 같은
> 이름을 쓰면 멤버 본문 안에서 타입이 가려진다 (실제로 컴파일이 깨졌다).

---

## 3. `ITelemetrySink` — L1 이 정의하고 L3 가 구현

```cpp
class ITelemetrySink {
    virtual void OnTxn(const TxnTiming_t& txn) = 0;   // 트랜잭션 타이밍 (valid=false 여도 통계용)
    virtual void OnFeedback() = 0;                    // 이번 tick 피드백 발행
    virtual void OnRwErr(uint8_t rw_err) = 0;         // read_err | write_err<<4
    virtual void OnWriteErrTotal(uint64_t total) = 0; // 누적 — action result 구간 차분용
    virtual void OnIrregularTick() = 0;               // 정규 RW 를 못 한 tick (양보·oos)
    virtual void OnLateTick() = 0;                    // 주기 초과로 위상 리셋된 tick
    virtual void SetReadPreset(const ReadPreset* preset) = 0;
};
```

**의존성 역전**: L1 이 L3 에게 "발행해라" 를 알려야 하는데, 그 통지를 위해 L1 이 L3 헤더를
include 하면 계층이 역류한다. 인터페이스를 L1 이 소유하면
`rd_schedule.hpp` 에서 `#include "rd_node.hpp"` 를 지울 수 있다 — 그것이 이 파일의 전부다.

> **구현체의 메서드는 200Hz 스케줄 스레드에서 불린다. 블록하면 안 된다.**

---

## 4. `RdTxnQueue` — 락프리 SPSC 링버퍼

```cpp
template <typename T, size_t N = 256>   // N 은 2의 거듭제곱 (static_assert)
class RdTxnQueue {
    bool TryPush(const T& v);       // [생산자=스케줄] 가득 차면 드롭 + drop_cnt++. 절대 블록 안 함
    bool Pop(T* out);               // [소비자=발행 스레드]
    uint64_t DropCount() const;     // 누적 (리셋 없음). 0 이면 한 번도 안 막혔다
    bool Empty() const;
};
```

**왜**: DDS 발행이 블록되면 200Hz tick 이 그대로 밀린다. 큐로 끊는다.

설계 규칙:

| | |
|---|---|
| Q1 | SPSC — 생산자 1(스케줄) / 소비자 1(발행). atomic head/tail + release/acquire |
| Q2 | 생산자는 절대 대기하지 않는다. **새 샘플을 드롭**한다 (오래된 것을 버리면 시계열 순서가 꼬인다) |
| Q3 | 드롭이 있었으면 다음 성공 샘플에 누적 `drop_cnt` 를 실어 보낸다 — **분석이 구멍의 위치와 크기를 안다** |
| Q4 | 256 슬롯 ≈ 1.28초 분량. 이걸 넘겼다면 소비자가 1초 이상 멈춘 것 = 이미 비정상 |

테스트: `test/unit/test_txn_queue.cpp` (순수 헤더라 링크 불필요).
