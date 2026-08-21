# orin_firmware_bridge — 코드 구조 문서 (종합)

> **누구를 위한 문서인가**: 이 패키지의 코드를 **직접 고치려는 사람**.
> 프리셋을 바꾸거나, 모드를 하나 더 만들거나, 명령을 추가하려 할 때
> "그것을 어느 파일의 어느 함수에서 하는가" 를 찾기 위한 지도다.
>
> 운용 방법(빌드·기동·웹/CLI 조작)은 `DOC/` 가, 설계 근거·결정 이력은 `redesign/00~08` 이
> 갖는다. 이 문서는 **지금 코드가 실제로 어떻게 생겼는지**만 적는다.

## 문서 구성

| 문서 | 다루는 것 |
|---|---|
| **README.md** (여기) | 전체 지도 · 공유 변수 · 한 tick 의 전 과정 · 스레드/락 |
| [L0_core.md](L0_core.md) | `core/` — 시리얼·패킷·레지스터 섀도·프리셋 표 |
| [L1_sched.md](L1_sched.md) | `sched/` — 200Hz 루프·슬롯 테이블·트랜잭션 큐 |
| [L2_policy.md](L2_policy.md) | `policy/` — 제어 FSM·커맨드 슬롯·프로파일·전개 시퀀스 |
| [L3_ros.md](L3_ros.md) | `ros/` — 토픽·서비스·액션·발행 |
| [RECIPES.md](RECIPES.md) | **"○○ 를 바꾸려면"** — 프리셋 추가, auto_mode 추가, 명령 추가 등 |

---

## 1. 계층이란 무엇인가

디렉터리가 곧 계층이다. `include/orin_firmware_bridge/` 와 `src/` 가 같은 이름으로 나뉜다.

```
                    ┌──────────────────────────────────────────┐
   L3  ros/         │ RdNode · RdTelemetry · RdCarrierApi       │  rclcpp 를 아는 유일한 층
                    │ RdControlApi · rd_ros_adapters           │
                    └───────────┬───────────────────┬──────────┘
                                │ 조립·주입          │ ITelemetrySink 로 통지받음
                    ┌───────────▼───────────────────┴──────────┐
   L1  sched/       │ RdSchedule (200Hz 루프)                   │  ← main 스레드가 여기서 돈다
                    │ rd_slot_table (표) · RdTxnQueue           │
                    └───────────┬──────────────────────────────┘
                                │ 매 tick 정책 객체를 호출
                    ┌───────────▼──────────────────────────────┐
   L2  policy/      │ RdControl · RdCommand · RdSequence        │
                    │ RdOos · RdProfile(Player) · rd_status     │
                    └───────────┬──────────────────────────────┘
                                │
                    ┌───────────▼──────────────────────────────┐
   L0  core/        │ RdUart → RdComm → RdMap                  │
                    │ 레지스터 헤더 · 읽기 프리셋 · 시계동기     │
                    └──────────────────────────────────────────┘

   공용 (루트)       rd_config.hpp (BridgeConfig) · rd_logger.hpp · rd_clock.hpp
```

### ⚠ 번호와 의존 방향이 어긋난다

이름은 L0→L1→L2→L3 인데 **실제 include 방향은 `sched → policy` 다.**
스케줄 루프가 매 tick 정책 객체를 부르므로 부르는 쪽(sched)이 의존하는 쪽이다.

- `policy/` 는 `sched/` 를 **한 번도** include 하지 않는다.
- `sched/rd_schedule.hpp` 는 `policy/` 3개(`rd_command`·`rd_oos`·`rd_profile_player`)를 include 한다.

이 사실은 `test/unit/test_layer_boundary.cpp` 가 테스트로 고정해 두었다 (그 파일 주석이
`redesign/02` 문서의 자기모순도 함께 기록해 두었다).

### 경계가 지켜지는 방식 — 두 겹

1. **링크로 막는다** (`CMakeLists.txt` §2). 라이브러리가 둘로 갈린다.
   - `rd_core_lib` = L0+L1+L2. **`rclcpp` 를 링크하지 않는다.**
   - `rd_ros_lib` = L3. 여기만 rclcpp 를 안다.
   - `rd_bridge_lib` = 둘을 합친 INTERFACE 타깃 (기존 호출부 호환).

   하위 계층에서 `RCLCPP_INFO` 를 쓰면 **링크 에러**가 난다.

2. **테스트로 막는다** (`test_layer_boundary.cpp`). 링커가 못 잡는 둘을 본다.
   - 헤더만 include 한 위반 (`#include <rclcpp/...>` 만 하고 심볼 미사용)
   - 역방향 include (`core/` 가 `ros/` 를 include)

**로깅은 `ILogger`** (`rd_logger.hpp`) 로 한다 — `RD_INFO(log_, "이름", fmt, ...)`.
형태는 `RCLCPP_INFO` 와 같고, 구현체만 L3(`RclcppLogger`)에서 주입된다.

---

## 2. 조립 — `main.cpp` 가 전부 보여준다

`src/main.cpp` 92줄이 이 패키지의 **유일한 조립 지점**이다. 여기 순서가 곧 소유 관계다.

```cpp
rclcpp::init(argc, argv);

RdUart uart("/dev/ttyUSB0");        // L0 — 포트
RdComm comm(&uart);                 // L0 — 패킷
RobotState_t robot_state{};         //     ★ 전 계층이 공유하는 유일한 상태
robot_state.ecu.reg.cmd_system.soft_estop = ecu::SOFT_ESTOP_RELEASE;   // 섀도 기본값
RdMap  map;                         // L0 — 인코더/디코더 (상태 없음)

auto bridge_node = std::make_shared<RdNode>(&robot_state);   // L3 — 파라미터 파싱까지 끝난다
RdCommand command(&robot_state);                             // L2 — 커맨드 슬롯 4개
bridge_node->AttachCommand(&command);

static RclcppLogger  ros_logger;    // L3 어댑터 — 하위 계층에 주입
static RosClock      ros_clock;
static RclcppRunGate ros_gate;

RdSchedule scheduler(&comm, &map, &robot_state, bridge_node.get(), &command,
                     bridge_node->Config(), &bridge_node->Oos(),
                     &bridge_node->Player(), &bridge_node->Control(),
                     on_init_done, skip_cmd_write,
                     &ros_logger, &ros_clock, &ros_gate);

bridge_node->AttachReadPresetSetter(...);   // 스케줄러가 이제야 존재하므로 나중에 꽂는다
bridge_node->Start();                       // spin 스레드 + 발행 스레드 기동
return scheduler.MainLoopStart();           // ★ 이 스레드가 200Hz 실시간 루프가 된다
```

**핵심 세 가지**

- `RobotState_t` 는 `main()` 스택에 있고 **주소로만 전달된다.** 수명이 전부보다 길다.
- L2 객체 3개(`RdControl`·`RdProfilePlayer`·`RdOos`)는 **`RdNode` 가 멤버로 소유**하고,
  스케줄러는 그 주소를 직접 잡는다 (노드를 거치지 않는다).
- `RdNode` 는 자신의 L3 3개보다 **먼저 선언**된 L2 3개를 갖는다 —
  선언 순서가 곧 소멸 순서의 역순이므로 순서를 바꾸면 종료 시 죽은 객체를 가리킨다.

---

## 3. 공유 변수 — 무엇이 계층을 넘나드는가

### 3.1 `RobotState_t` — 레지스터 섀도 (가장 중요)

`core/rd_map.hpp` 정의. **STM 보드 3장의 레지스터를 Orin 메모리에 복사해 둔 것**이다.

```cpp
struct CommHealth_t { uint16_t timeout_cnt; bool is_connected; };
struct EcuState_t { CommHealth_t comm; ecu::REGISTER_t reg; };   // reg = 256B packed
struct DpcState_t { CommHealth_t comm; dpc::REGISTER_t reg; };
struct PcuState_t { CommHealth_t comm; pcu::REGISTER_t reg; };

struct RobotState_t {
    EcuState_t ecu;  DpcState_t dpc;  PcuState_t pcu;
    mutable std::mutex state_mutex;      // ★ 전 계층 공용 락
};
```

`reg` 는 `__attribute__((packed))` 구조체이며 **주소 = 바이트 오프셋**이다.
`ShadowBase(state, target_id, &total)` 가 그 평평한 바이트 배열의 시작 포인터를 준다.

| 누가 | 언제 | 무엇을 |
|---|---|---|
| `RdMap::Encode` | 매 tick (WRITE/RW) | 섀도에서 **읽어** 패킷에 싣는다 |
| `RdMap::Decode` | 매 tick (READ/RW) | 응답을 섀도에 **쓴다** (세그먼트별 scatter) |
| `RdControl::PrepareWrite` | 매 tick | `cmd_motor` 영역에 명령값을 **쓴다** |
| `RdCarrierApi::GetRosInputs` | 100Hz | `cmd_system.cmd_lin_vel/ang_vel` 를 **쓴다** |
| `RdCommand::GetSlotTask` | 슬롯 차례 | WRITE_DATA 의 사용자 값을 섀도에 **쓴다** |
| `RdTelemetry::Publish*` | 100/10/200Hz | 섀도를 통째로 스냅샷해 **읽는다** |
| `RdControlApi` (config/status) | 서비스 콜백 | 검증 read-back 을 **읽는다**, 소독 시 **쓴다** |

> **섀도의 함정**: 값은 언제나 있지만 **신선하다는 보장은 없다.**
> 지금 읽기 프리셋이 그 구간을 안 읽으면 그 자리는 낡은 값(혹은 초기화 0)이다.
> 그래서 발행 경로는 `RdTelemetry::Reads(addr, len)` 로 매번 물어보고,
> 안 읽는 자리는 **0 이 아니라 미판독 센티넬**(uint8 `0xFF` / float `NaN`)로 내보낸다.
> 자세한 것은 [L0_core.md](L0_core.md#4-읽기-프리셋) 참조.

### 3.2 `BridgeConfig` — 기동 1회 확정 후 불변 (`rd_config.hpp`)

`RdNode` 생성자가 ROS 파라미터를 파싱해 채운다. 하위 계층은 **`const&` 로 받는다**.

| 필드 | 기본 | 의미 |
|---|---|---|
| `bridge_mode` | `project` | `project` / `control` / `manual` (`BridgeMode` enum) |
| `read_preset_param` | 0(`control`) | 기동 시 프리셋 id |
| `active_motor_mask` | `0x0F` | bit0~3 = M1~M4. **⚠ 유일하게 런타임에 바뀌는 필드** |
| `auto_mode_param` | `1`(CURRENT) | INIT 이 ECU 에 쓸 목표값. `0xFF`=`AUTO_MODE_NONE` |
| `cmd_current_max` | 30.0 | [A] 전역 클램프 |
| `cmd_vel_topic_timeout` / `zero_timeout` | 0.1 / 3.0 | [s] cmd_vel 워치독 |
| `stream_timeout` | 0.1 | [s] STREAM 스테일 한계 |
| `comm_diag_enable` | false | 지연 원자료 발행 |
| `enable_dpc_read` / `enable_pcu_read` | false | 보드 미장착 시 timeout 폭주 방지 |
| `imu_frame_id` | `imu_link` | |

**편의 술어** — 호출부가 enum 비교를 흩뿌리지 않도록:
`IsControl()` `IsManual()` `IsProject()` `WritesCommands()` `IsReadOnlyControl()` `AutoConfiguresEcu()`

> `cmd_vel_guard_enable` 은 **여기 없다.** `ros2 param set` 으로 런타임 토글되므로
> "기동 후 불변" 이라는 이 구조체의 약속을 깬다 — `RdCarrierApi::guard_enable_`(atomic)이 소유한다.

### 3.3 주입되는 인터페이스 (A5 — rclcpp 고리 끊기)

| 인터페이스 | 정의 | 구현체(L3) | 폴백 |
|---|---|---|---|
| `ILogger` | `rd_logger.hpp` | `RclcppLogger` | `DefaultLogger()` (stderr) |
| `IClock` | `rd_clock.hpp` | `RosClock`(=`SystemClock`) | `DefaultClock()` |
| `IRunGate` | `rd_clock.hpp` | `RclcppRunGate`(`rclcpp::ok()`) | `AlwaysOkGate` |
| `ITelemetrySink` | `sched/rd_telemetry_sink.hpp` | `RdNode` → `RdTelemetry` 위임 | 없음 (nullptr 검사) |

**절대 nullptr 을 돌려주지 않는 폴백**이 있으므로 주입을 깜빡해도 조용히 사라지지 않는다.
`ITelemetrySink` 만 L1 이 정의하고 L3 가 구현하는 **의존성 역전** 형태다 —
이것 덕에 `rd_schedule.hpp` 가 `rd_node.hpp` 를 include 하지 않는다.

### 3.4 콜백으로 건네지는 것

| 콜백 | 방향 | 하는 일 |
|---|---|---|
| `on_init_done(bool with_profiles)` | L1 → L3 | INIT 완료. `false` 면 config 만 열고 재생 서버는 안 연다 |
| `skip_cmd_write()` | L1 → L3 | cmd_vel 토픽 신선도 판정 (`RdCarrierApi::ShouldSkipCmdWrite`) |
| `SetReadPresetSetter(fn)` | L3 → L1 | 프리셋 교체는 L1 소유인데 스케줄러가 나중에 생기므로 나중에 꽂는다 |
| `counters()` | L3(node) → L3(api) | `RunCounters_t` 스냅샷 — **한 번에 모아서** 준다 |
| `SetSlotSnapshot` / `SetLastRead` / `SetReadPresetPtr` | L3 → L3 | GET_STATUS·GET_REGISTERS 의 출처 |

### 3.5 원자적 상태 (락 없이 계층을 넘는 값)

| 변수 | 소유 | 읽는 쪽 |
|---|---|---|
| `RdSchedule::read_preset_` | L1 | `SelectControlTask`, `SetReadPreset` |
| `RdControl::auto_mode_` | L2 | `SelectControlTask`(L1), `PrepareWrite`(L2), 발행(L3) |
| `RdControl::stream_armed_` | L2 | config service (L3) |
| `RdControl::prof_lo_/prof_hi_` | L2 | `PrepareWrite` |
| `RdCommand::cmd_vel_paused_` | L2 | 스케줄 게이트 (L1) |
| `RdSequence::trigger_`, `lock_` | L2 | jeongae 토픽 콜백 (L3) |
| `RdCarrierApi::guard_enable_` | L3 | param 콜백 |
| `RdTelemetry::read_preset_` (포인터) | L3 | `Reads()` |

---

## 4. 한 tick 의 전 과정

**tick = 5ms 에 RS485 트랜잭션 정확히 1건.** 그 1건이 무엇인지는 **표**가 정한다.

```
 ┌─ RdSchedule::RunLoop 루프 1회 ────────────────────────────────────────────┐
 │                                                                          │
 │ ① sleep_until(next_cycle)          ← 5ms 주기. wake latency 계측          │
 │                                                                          │
 │ ② 활성 프레임 선택                                                        │
 │      control  → kControl / kControlRead (40칸)                           │
 │      project  → kProject (10칸)                                          │
 │      manual   → kManual  (10칸)                                          │
 │                                                                          │
 │ ③ 양보 tick 인가?  frame.UserSlotAt(tick) && FSM ∉ {RUNNING, STREAM}      │
 │      예 → RunUserSlot(): 슬롯 0→3 훑어 대기 중 명령 1건 발사              │
 │           성공하면 sink_->OnIrregularTick() 후 이 tick 끝                 │
 │                                                                          │
 │ ④ 모드별 분기                                                             │
 │   ┌ read_only_control (auto_mode:none) ─ ExecuteTask(READ) → 발행         │
 │   ├ control ─ RdOos::TakeStep() 있으면 그 tick 을 단발 WRITE/READ 로 대체  │
 │   │           없으면: player_->Tick() → control_->PrepareWrite()          │
 │   │                   → ExecuteTask(SelectControlTask(auto_mode))         │
 │   │                   → OnRwErr / OnTxn / OnFeedback / NoteWriteErrStreak │
 │   └ project·manual ─ command_->TickAutoSequence()                        │
 │                      슬롯 종류에 따라 COMMAND / ECU·DPC·PCU 트랜잭션      │
 │                      게이트: blackout, enable_dpc/pcu_read, cmd_vel guard │
 │                      ActionFor(slot, block_write) → NORMAL/READ_FALLBACK/SKIP │
 │                                                                          │
 │ ⑤ tick_count_++ / 주기 초과 판정 (초과 시 OnLateTick + 위상 리셋)          │
 │ ⑥ 400 tick 마다 Heartbeat 출력 후 구간 통계 리셋                          │
 └──────────────────────────────────────────────────────────────────────────┘
```

`ExecuteTask` 안쪽 (L1 → L0):

```
map_->Encode(config, robot_state_, &packet_obj_, &data_len)   [state_mutex]
comm_->Clear()                     ← 유일한 flush 지점. 잔여 바이트 제거
last_txn_.t_req = clock_->NowEpoch()
comm_->Write(&packet_obj_, data_len)      → RdUart::Write (턴어라운드 대기 포함)
comm_->Read(&packet_obj_, 2, 2)           → 헤더 2ms + 바디 2ms  (예산 4ms < 5ms)
last_txn_.t_resp = clock_->NowEpoch()
map_->Decode(&packet_obj_, config, robot_state_)              [state_mutex]
```

---

## 5. 스레드와 락

### 5.1 스레드 4종

| 스레드 | 만드는 곳 | 하는 일 | 우선순위 |
|---|---|---|---|
| **main** | `main.cpp` | `RdSchedule::MainLoopStart()` — 200Hz RS485 | SCHED_FIFO 80, CPU core 11 |
| **spin** | `RdNode::Start()` | `MultiThreadedExecutor` — 타이머·구독·서비스·액션 | 일반 |
| **telemetry** | `RdTelemetry::Start()` | `RdTxnQueue` 를 비워 publish | 일반 |
| **action exec** | `HandleProfileAccepted` | goal 당 1개 detach. 5Hz 피드백 | 일반 |

> 200Hz 루프는 **절대 DDS 발행으로 블록되지 않는다.** 메시지 구성까지는 스케줄 스레드가
> 하고(그래야 tick 시점의 값이 실린다), `RdTxnQueue::TryPush` 로 던진 뒤 발행 스레드가 받는다.
> 큐가 가득 차면 **새 샘플을 드롭**하고 `drop_cnt++` — 오래된 것을 버리면 시계열 순서가 꼬인다.

### 5.2 뮤텍스 목록

| 락 | 보호 대상 | 주의 |
|---|---|---|
| `RobotState_t::state_mutex` | 레지스터 섀도 3장 | 가장 넓게 쓰인다. 스냅샷만 뜨고 즉시 풀 것 |
| `RdControl::mutex_` | FSM 상태·프로파일 샘플·스트림·펄스 | |
| `RdControl::ctr_mutex_` | `ctr_mode_[4]` | |
| `RdCommand::mutex_` | 슬롯 4개·blackout·last_read | `mutable` (const 스냅샷이 잠근다) |
| `RdSequence::mutex_` | 전개 FSM 상태 | |
| `RdOos::mutex_` + `cv_` | 단발 write 단계 | service 스레드가 여기서 최대 50ms 대기 |
| `RdProfilePlayer::mutex_` | 샘플 배열·재생 인덱스 | |
| `RdCarrierApi::data_mutex_` / `RdControlApi::data_mutex_` | 입력 스냅샷·짧은 임계구역 | |
| `RdUart::port_mutex_` | 시리얼 포트 | |

### ⚠ 락 순서 규약

```
shadow_->state_mutex  →  RdControl::mutex_  →  RdControl::ctr_mutex_
```

`RdControl::PrepareWrite` 가 `state_mutex` 를 쥔 채 `mutex_` 를 잡으므로,
**반대 순서로 잡는 경로가 생기면 즉시 데드락**이다.
`SafeStop()` 이 `mutex_` 를 스코프로 먼저 놓고 `state_mutex` 를 잡는 것은 이 규약 때문이다 —
붙여 쓰면 안 된다 (`policy/rd_control.hpp` 주석 참조).

---

## 6. 모드 3개 × 구성

`bridge_mode` 는 **축 하나**다 (불리언 둘이면 "둘 다 true" 같은 표현 불가능한 조합이 생긴다).

| bridge_mode | 프레임 | ECU 슬롯 | 커맨드 경로 | INIT |
|---|---|---|---|---|
| `project` | `kProject` 10칸 | RW×5 (100Hz) | tick 4·6·8 양보 | 없음 (FSM INIT 유지) |
| `control` | `kControl` 40칸 | RW×40 (200Hz) | tick 39 양보 1칸 | motor_mask→auto_mode→mode(AUTO) 3단 검증 |
| `control` + `auto_mode:none` | `kControlRead` 40칸 | READ×40 | tick 39 | write 없음 → `MarkInitDone()` 만 |
| `manual` | `kManual` 10칸 | READ×5 | **10칸 전부** 양보 | 자동 설정 전무 → `MarkInitDone()` 만 |

**구 `traction_test_mode` 는 별도 모드가 아니다** — control 의 한 구성이다:

```bash
ros2 run orin_firmware_bridge comm_test_node --ros-args \
  -p bridge_mode:=control -p auto_mode:=none -p read_preset:=control_test
```

---

## 7. ROS 인터페이스 요약 (L3)

**노드는 하나다** (`firmware_bridge_node`). L3 를 셋으로 나눈 것은 C++ 책임 분할이지
ROS 노드 분할이 아니다 — `ros2 node list` 는 그대로다.

| 종류 | 이름 | 소유 |
|---|---|---|
| sub | `/carrier_cmd_vel` (Twist) | `RdCarrierApi` |
| sub | `/jeongae` (JeonGae) | `RdCarrierApi` |
| sub | `/carrier/control/cmd_motor` (CmdMotor) | `RdControlApi` |
| srv | `/carrier/command_set` (CommandSet) | `RdCarrierApi` |
| srv | `/carrier/jeongae_lock` (SetBool) | `RdCarrierApi` |
| srv | `/carrier/control/config` (ControlConfig) | `RdControlApi` |
| action | `/carrier/control/run_profile` (RunProfile) | `RdControlApi` |
| pub | `/carrier_battery` (UInt8), `/carrier_imu` (Imu) | `RdTelemetry` |
| pub | `/carrier/{ecu,dpc,pcu}/status` (NodeStatus) | `RdTelemetry` |
| pub | `/carrier/ecu/motor` (MotorStatus) | `RdTelemetry` |
| pub | `/carrier/control/feedback` (ControlFeedback) | `RdTelemetry` — control 전용 |
| pub | `/carrier/control/comm_diag` (CommDiag) | `RdTelemetry` — `comm_diag_enable` 시 |

---

## 8. 이 코드가 반복해서 지켜 온 규칙 3개

고칠 때 이 셋을 깨지 않도록 주의한다. 셋 다 **실기에서 사고가 나서** 생긴 규칙이다.

1. **"안 읽음" 과 "정상 0" 을 절대 섞지 않는다.**
   미판독은 uint8 `0xFF`, float `NaN`. 0 은 "고장 없음"·"정지" 라는 **유효한 관측값**이다.
   → `ReadPreset::Covers()`, `RdTelemetry::Reads()`, `RdCommand::DpcSysState()`

2. **200Hz 스레드가 읽는 데이터를 런타임에 조립하지 않는다.**
   (프리셋 × auto_mode) 조합을 **기동 시 전부 구워 두고** atomic 인덱스만 바꾼다.
   → `RdSchedule` 생성자의 표 굽기, `SelectControlTask()`

3. **사람이 숫자를 옮겨 적지 않는다.**
   응답 예산·구간 길이·프레임 주기는 전부 `constexpr` 파생 + `static_assert` 검산이다.
   (`kMaxRespPayload = MAX_DATA_LEN - 1` 처럼) 손으로 적은 `88` 하나가 몇 주를 잡아먹은 적 있다.

---

## 9. 알아 둘 어긋남 (코드 읽을 때 헷갈리는 지점)

문서·주석이 코드와 다른 곳이다. 고치기 전에 알고 있어야 한다.

| 어긋남 | 실제 |
|---|---|
| 주석의 "계층 방향 L1→L2" 화살표 | 코드는 `sched → policy`. `test_layer_boundary.cpp` 가 정본 |
| `RdSequence::Tick()` 주석 "5Hz 로 호출" | `RunLoop` 의 project/manual 분기에서 **매 tick(200Hz)** 불린다. 따라서 `kWaitTicksMax=150` 은 30초가 아니라 **약 0.75초**다 |
| `src/rd_sequence.cpp` (루트) | **빌드에 포함되지 않는 잔재.** 실물은 `src/policy/rd_sequence.cpp`. 루트 쪽은 없는 헤더 경로를 include 한다 |
| 슬롯 표의 `Cmd(i)` 인덱스 | project 의 커맨드 발사는 사실상 `RunUserSlot()` (슬롯 0→3 우선순위 훑기)이 한다. 표의 `cmd_index` 는 양보가 안 일어난 폴백 경로에서만 쓰인다 |
| `HW_BIT_*` 매크로 (`rd_common.hpp`) | 정의만 있고 **사용처가 0**. 이름(`HW_BIT_UART6`)이 실제 채널과 맞는지 테스트가 잡아 주지 못한다 |

---

## 10. 테스트 — 무엇을 고치면 무엇이 깨지는가

`colcon test --packages-select orin_firmware_bridge` 로 전부 돈다.
**노드를 안 띄우고 도는 유닛 테스트가 많다** (`rd_core_lib` 만 링크) — 그게 A5 분리의 성과다.

| 테스트 | 무엇을 고정하나 |
|---|---|
| `test_layer_boundary` | 계층 경계 (rclcpp 누출·역방향 include) |
| `test_golden_wire` | **RS485 출력 바이트열** — 프리셋/슬롯을 바꾸면 여기가 먼저 깨진다 |
| `test_slot_table` | 프레임 구성·주기·양보 마스크 |
| `test_read_preset` | `Covers()` 의미와 프리셋 불변식 |
| `test_b6_catalog` | 명령 카탈로그 게이트 조합 |
| `test_status_json` | GET_STATUS 스키마 (키 존재·타입) |
| `test_stm_mirror` | STM 헤더와 Orin 미러의 바이트 단위 일치 (`stm_ws` 있을 때만) |
| `test_no_dead_code` / `test_counter_producers` | **소스를 읽는다** — 생산자 없는 카운터·죽은 코드 |
| `test_control_fsm` / `test_sequence` / `test_txn_queue` / … | 각 FSM·큐 동작 |

컴파일 타임 검산(`static_assert`)이 1차 방어선이다 — 예산을 넘는 프리셋은 **빌드가 안 된다.**
