# RECIPES — "○○ 를 바꾸려면 어디를 고치나"

> 각 항목은 **고칠 파일 목록 → 순서 → 확인 방법** 순이다.
> 계층 개념은 [README.md](README.md), 상세는 각 계층 문서를 본다.

## 공통 원칙 3개 (모든 레시피에 적용)

1. **숫자를 손으로 옮겨 적지 않는다.** 길이·예산은 `constexpr` 로 파생시키고
   `static_assert` 로 검산한다.
2. **200Hz 스레드가 읽는 데이터를 런타임에 조립하지 않는다.** 조합을 기동 시 전부
   만들어 두고 atomic 인덱스만 바꾼다.
3. **"안 읽음" 과 "정상 0" 을 섞지 않는다.** 새 필드를 발행 경로에 추가하면
   미판독 경로(`0xFF`/`NaN`)도 같이 만든다.

## 빌드·테스트

```bash
cd ~/orin_ws
colcon build --packages-select orin_firmware_bridge && source install/setup.bash
colcon test --packages-select orin_firmware_bridge && colcon test-result --verbose
```

---

## 1. 읽기 프리셋 추가 (센서 조합 바꾸기)

**난이도: 낮음.** 표 한 곳만 고치면 나머지는 파생된다.

### 고칠 곳 — `include/orin_firmware_bridge/core/rd_read_preset.hpp` **하나**

```cpp
// ① 프리셋 정의 (namespace ecu 안)
constexpr ReadPreset kPresetMyTest = {
    "my_test",                                  // ← 파라미터·CLI 에서 쓸 이름
    {{REG_SYS_OFFSET, REG_SYS_SIZE},            // 16:17  ★ 모든 프리셋 공통 규칙
     {REG_LOADCELL_OFFSET, REG_LOADCELL_SIZE},  // 42:6
     {REG_MOTOR_DATA_OFFSET, REG_MOTOR_DATA_SIZE}},  // 88:40
    3                                            // ← count. spans 개수와 반드시 일치
};

// ② 선택 가능 목록에 추가
constexpr const ReadPreset* kPresets[] = {
    &kPresetControl, &kPresetDiag, &kPresetControlTest, &kPresetMyTest };
// kPresetCount 는 sizeof 로 자동 계산된다

// ③ 불변식 못박기 — 최소 이 둘
static_assert(kPresetMyTest.RespPayload() <= kMaxRespPayload, "my_test 응답 예산 초과");
static_assert(kPresetMyTest.Covers(REG_SYS_OFFSET, REG_SYS_SIZE),
              "my_test 가 SYS 전체를 안 읽는다 (04 §2.3 변경 1)");
```

### 자동으로 따라오는 것

| 무엇 | 어디서 |
|---|---|
| `(프리셋 × auto_mode)` 태스크 굽기 | `RdSchedule` 생성자 — `ecu::kPresetCount` 루프 |
| `auto_mode:none` READ 태스크 | 같은 루프 (`task_control_read_[]`) |
| 기동 파라미터 `-p read_preset:=my_test` | `ParseReadPreset()` 가 이름으로 찾는다 |
| `config read_preset 3` (런타임 교체) | `RdSchedule::SetReadPreset` 이 id 범위만 본다 |
| 발행 신선도 판정 | `RdTelemetry::Reads()` → `Covers()` |

### 확인

```bash
colcon test --packages-select orin_firmware_bridge --ctest-args -R test_read_preset
colcon test --packages-select orin_firmware_bridge --ctest-args -R test_golden_wire
```

### 주의 4가지

- **`count` 를 안 고치면 조용히 틀린다.** `spans[]` 에 4개를 적고 `count=3` 이면
  마지막 세그가 무시된다 — 컴파일러가 잡아 주지 않는다.
- **세그먼트는 6개까지** (`kMaxReadSegs`). 초과하면 `static_assert` 로 막을 것.
- **읽기를 빼면 조용히 망가지는 자리가 있다.**
  - `{128,4}` 를 빼면 `SET_CTR_MODE` 검증이 **무조건 통과**한다 (자기가 쓴 값을 자기가 읽는다)
  - `{164,16}` 을 빼면 `ControlFeedback.cmd` 가 낡은 값을 낸다
  - 센서 블록 **끝바이트**(47/69/85/86/87/127)를 놓치면 그 채널의 `lc`/`hs` 가 미판독이 된다
- **기존 프리셋을 수정하면 골든이 깨진다** → 아래 §9.

---

## 2. `auto_mode` 추가 (새 제어 모드)

**난이도: 높음.** 손댈 곳이 **9개 파일**이다. 하나라도 빠지면
"동작은 옳고 보고만 틀린" 상태가 되거나, 안전값이 잘못된 단위로 나간다.

> 새 모드가 정말 새 `auto_mode` 인지 먼저 확인한다. ECU 가 지원하지 않으면
> 브리지만 고쳐도 아무 일도 일어나지 않는다 — **ECU 펌웨어가 먼저다.**

### 체크리스트

| # | 파일 | 고칠 것 |
|---|---|---|
| ① | `core/rd_register_ecu.hpp` | `AUTO_MODE_XXX` 상수 |
| ② | 〃 | `AutoModeName()` — 표시 이름 |
| ③ | 〃 | `AutoModeWriteSpan()` — `"132:16"` 같은 문자열 |
| ④ | 〃 | `AutoModeForcedCtrMode()` — 비-DIRECT 면 ECU 가 강제하는 `ctr_mode` |
| ⑤ | `sched/rd_schedule.hpp` | `enum { kIdxCurrent, ..., kIdxModeCount }` 에 인덱스 추가 |
| ⑥ | `sched/rd_schedule.cpp` | 생성자의 `build(waddr, wlen)` 한 줄 + `SelectControlTask()` case |
| ⑦ | `policy/rd_control.cpp` | `PrepareWrite()` switch — **안전값**과 클램프 |
| ⑧ | 〃 | `SelectWrite()` STREAM 분기 — 어느 필드를 쓸 것인가 |
| ⑨ | `ros/rd_node.cpp` | `auto_mode` 파라미터 단어형/정수형 파싱 |
| ⑩ | `ros/rd_control_api.cpp` | `OP_SET_AUTO_MODE` 의 `am_ok` 화이트리스트 |
| ⑪ | `ros/rd_telemetry.cpp` | `OnFeedback()` 의 `m.cmd[i]` 선택 |
| ⑫ | `policy/rd_status.cpp` | `AutoModeJsonName()` |
| ⑬ | `policy/rd_profile.cpp` | (프로파일 `mode` 도 추가한다면) `AcceptsAutoMode()` 의 native 매핑 |

### 순서

```
① 상수  →  ②③④ 이름·범위·강제 ctr_mode  →  ⑤⑥ 태스크 굽기·셀렉터
        →  ⑦ **안전값** (가장 위험)  →  ⑧ 스트림
        →  ⑨⑩ 입구 (파라미터·서비스)  →  ⑪⑫ 보고
```

### ⑦ 안전값이 가장 위험하다

```cpp
case ecu::AUTO_MODE_XXX:      // write A:B
    for (int i = 0; i < 4; i++)
        cm.cmd_xxx[i] = cmd_active ? clamp_prof(w.current[i]) : <안전값>;
    break;
```

**`<안전값>` 을 0 으로 두기 전에 그 단위에서 0 이 무엇을 뜻하는지 확인한다.**
`cmd_position` 에서 0 은 **"원점으로 가라"** 는 명령이다.
2026-07-28 실기에서 `auto_mode=5` 로 바꾸는 순간 모터가 41.6도에서 0도로 슬루했다.

| 단위 | 안전값 | 근거 |
|---|---|---|
| 전류 [A] | `0` | 토크 없음 |
| 속도 [RPM] | `0` | 정지 |
| 위치 [deg] | **`fb_position` (매 tick 재시드)** | 현재 자세 유지 = bumpless transfer |

**클램프도 단위를 본다.** `clamp_a()` 는 `clamp_max_`([A])를 쓰므로 **전류에만** 건다.
RPM·deg 에 걸면 단위가 다른 값을 잘라내는 셈이다.

### ⑥ 태스크 굽기 (`rd_schedule.cpp` 생성자)

```cpp
task_control_[pi][kIdxXxx] = build(ecu::REG_CMD_XXX_OFFSET, ecu::REG_CMD_VALUE_SIZE);
```

`SelectControlTask()` 의 `default:` 는 `kIdxCurrent` 다 — **미지원 값이 와도 안전측으로
떨어진다.** 기동 검증이 이미 막지만 런타임 config service 가 바꿀 수 있으므로 그대로 둔다.

### 확인

```bash
colcon test --packages-select orin_firmware_bridge --ctest-args \
  -R "test_direct_units|test_idle_safe_value|test_control_fsm|test_status_json"
```

`test_idle_safe_value` 가 **새 모드의 안전값을 커버하도록 케이스를 추가**한다.
그 테스트가 없으면 "IDLE 인데 모터가 움직인다" 를 실기에서 발견하게 된다.

### 실기 확인

```bash
./cli.sh                     # 또는 웹
> config auto_mode <값>      # write 범위가 바뀌는지
> status                     # auto_mode / write_span / ctr_mode 셋이 **서로 맞는지**
```

> `status` 가 `auto_mode=5(POSITION)` 인데 `ctr_mode=[1,1,1,1](CURRENT)` 로 보이면
> ④ `AutoModeForcedCtrMode()` 를 빠뜨린 것이다.

---

## 3. 커맨드(슬롯 명령) 추가

**난이도: 낮음~중간.** 카탈로그 표 + srv 상수 두 곳.

### 3.1 READ 계열 추가 (구간만 다른 것)

**`policy/rd_command_catalog.hpp`**

```cpp
namespace spans {
constexpr ReadPreset kMyBlock = {"my_block", {{ecu::REG_XXX_OFFSET, ecu::REG_XXX_SIZE}}, 1};
static_assert(kMyBlock.RespPayload() <= kMaxRespPayload, "read_my_block 예산 초과");
}

constexpr uint8_t CMD_READ_MY_BLOCK = 5;      // ⚠ srv 와 같은 번호

constexpr CmdDef kCatalog[] = {
    ...
    // READ 계열 — safe_stop 불필요, 모든 모드 허용. **읽기는 아무것도 바꾸지 않는다.**
    {CMD_READ_MY_BLOCK, "read_my_block", CmdKind::READ, &spans::kMyBlock, 0, false, false},
};
```

**`../mgs_tp_msgs/srv/CommandSet.srv`**

```
uint8 CMD_READ_MY_BLOCK = 5    # 설명
```

> 번호는 **양쪽이 같아야 한다.** srv 를 고쳤으면 `mgs_tp_msgs` 부터 다시 빌드한다.

### 3.2 WRITE1 계열 추가

```cpp
{CMD_SET_XXX, "set_xxx", CmdKind::WRITE1, nullptr, ecu::REG_XXX_OFFSET, false, true},
//                                                                       ↑manual만  ↑safe_stop
```

**`needs_safe_stop = true` 를 기본으로 둔다.** 예외는 **감속 방향** 명령뿐이며,
그 판정은 값에 달렸으므로 표가 아니라 `Gate()` 안에서 한다:

```cpp
const bool decelerating = (cmd == CMD_SET_SOFT_ESTOP && arg0 == ecu::SOFT_ESTOP_ACTIVE);
```

### 3.3 새 `CmdKind` 가 필요하면

`policy/rd_command.cpp` 의 **switch 두 곳**을 같이 고친다:

- `HandleRequest()` — 인자 검증 (범위·길이·대상 보드)
- `SetSlotLocked()` — 의미 단위 이름 → 실제 트랜잭션 번역

### 확인

```bash
colcon test --packages-select orin_firmware_bridge --ctest-args -R test_b6_catalog
```

허용/거부 조합을 테스트에 **반드시 추가한다** — 게이트 조합이 순수 함수 하나에 모여 있는
값어치는 그 조합이 고정될 때만 생긴다.

### ⚠ 넣지 말아야 할 것

`ControlConfig` 에 이미 있는 것(`SET_ACTIVE_MOTORS`·`SET_MODE`·`SET_AUTO_MODE`·`SET_ORIGIN`)은
**여기 만들지 않는다.** 슬롯 경로에는 shadow 소독·IDLE 게이트·write 범위 전환이 없어
**조용히 더 약한 길**이 된다. 같은 것을 바꾸는 길이 둘이고 게이트가 다르면 그게 사고다.

---

## 4. 슬롯 표 / 프레임 바꾸기 (주기·배치 변경)

**난이도: 중간.** 표는 데이터지만 **표를 바꾸는 게 공짜는 아니다.**

### 고칠 곳 — `sched/rd_slot_table.hpp`

```cpp
constexpr FrameDef kProject = {
    "project", 10, {
        /* 0 */ DpcRd(&spans::kDpcSys10Hz),
        /* 1 */ EcuRw(&ecu::kPresetProject, ecu::REG_CMD_SYSTEM_OFFSET, 8),
        ...
    },
    kProjectUserMask
};
```

**슬롯 생성 도우미** (표를 읽을 수 있게 유지한다):

| 도우미 | 만드는 것 |
|---|---|
| `EcuRd(preset)` | ECU READ, 고정 구간 |
| `EcuWr(addr, len)` | ECU WRITE, 고정 구간 |
| `EcuRw(preset, waddr, wlen)` | ECU RW — 읽기 고정 + 쓰기 고정 (project) |
| `EcuRwPreset()` | ECU RW — 읽기 **런타임 프리셋** + 쓰기 **auto_mode 파생** (control) |
| `EcuRdPreset()` | ECU READ, 런타임 프리셋 (auto_mode:none) |
| `DpcRd(p)` / `PcuRd(p)` | 각 보드 READ |
| `Cmd(i)` | 커맨드 슬롯 (늦은 바인딩) |

### 반드시 같이 고칠 것

| 무엇 | 왜 |
|---|---|
| `user_slot_mask` | **용량은 `slots[]` 가 아니라 마스크가 정한다.** 어긋나면 "표에는 Cmd 자리인데 못 쓰는 칸" 이 생긴다 |
| 아래 `static_assert` 들 | `ticks`·`CountOf`·`UserSlotCount`·`MaxRespPayload` 전부 |
| `RdSchedule` 생성자의 `bake()` | 새 `SlotInst` 를 추가했다면 (지금은 READ/WRITE/RW 만 안다) |

### 확인

```bash
colcon test --packages-select orin_firmware_bridge --ctest-args -R "test_slot_table|test_golden_wire"
```

### 함정 3개 (Q3 재편 때 실제로 겪은 것)

1. **`SlotInst::RW` 를 굽는 경로**가 없으면 표에만 있고 동작하지 않는다.
2. **RW 는 게이트로 write 만 뺄 수 없다** — 읽기와 쓰기가 한 패킷이다.
   쓰기를 막아야 하면 `SlotAction::READ_FALLBACK` 용 태스크를 **따로 구워 둬야 한다**
   (`project_task_read_[]`). 아니면 별도 프레임으로 분리한다 (`kManual` 이 그 예).
3. **양보 마스크는 활성 프레임에서 읽는다.** `frames::kControl` 로 하드코딩하면
   project·manual 의 마스크가 무시된다.

### `kMaxFrameTicks = 40` 을 넘겨야 한다면

`user_slot_mask` 가 `uint64_t` 이므로 **64칸이 상한**이다. 그 이상은 마스크 타입부터 바꿔야 한다.

---

## 5. 프로파일 세그먼트 타입 추가

**난이도: 중간.** 한 함수 안이지만 검증 규칙을 같이 만들어야 한다.

### 고칠 곳 — `policy/rd_profile.cpp` 의 `ExpandSegment()`

```cpp
} else if (type == "my_wave") {
    double amp = 0.0, param = 0.0;
    if (!Req(seg, "amp",   &amp,   motor_no, seg_idx, "my_wave", err)) return false;
    if (!Req(seg, "param", &param, motor_no, seg_idx, "my_wave", err)) return false;

    // ★ 물리적으로 재생 불가능한 입력은 **거부한다** (조용히 근사하지 않는다)
    if (!(param > 0.0 && param <= 25.0)) { /* err 에 사유 + 해야 할 일 */ return false; }

    for (size_t i = 0; i < n; i++) {
        const double t = i * RdProfile::kDt;
        out->push_back(static_cast<float>(/* ... */));
    }
}
```

마지막 `else` 의 에러 메시지에 **새 타입 이름을 추가**한다
(`"지원: hold/ramp/stair/step/sine/chirp/prbs/noise/custom"`).

### 규칙 4개를 반드시 지킨다

1. **자동 성형 금지.** 한계를 넘으면 `err` 에 사유를 적고 `false`.
   **프로파일이 곧 실험 기록**이므로 몰래 고쳐 재생하면 기록과 실제가 어긋난다.
2. **펼치기 전에 tick 수를 검사**한다 (`kMaxTicks`). 메모리를 한 바이트도 쓰기 전에 거부.
3. **에러 메시지에 위치와 조치를 적는다** — `"m2 세그먼트[1] (type=my_wave): ..."`.
   사용자가 YAML 어디를 고칠지 알아야 한다.
4. **난수를 쓴다면** `rng`(모터별 독립 스트림, `seed + motor_no`)를 쓰고,
   `slew_rate` 면제가 필요한지 판단한다 (`was_noise` 와 같은 방식).

### 검증 규칙이 slew 와 충돌하는지 확인

`noise` 는 정의상 tick 간 델타가 무제한이라 `slew_rate` 와 함께 쓰면 **사실상 항상 거부**된다.
그래서 slew 를 면제하고 대신 **진폭 가드(`mean±4σ`)** 를 걸었다.
새 타입이 비슷한 성질이면 같은 판단이 필요하다.

### 확인

- `test/profiles/` 에 **수락 케이스 1개 + 거부 케이스 1개** YAML 을 추가한다
  (`ok_*.yaml` / `reject_*.yaml` 명명 규칙).
- `test_profile_corpus` 가 그 코퍼스를 읽어 C++/Python 판정 일치를 고정한다.

```bash
colcon test --packages-select orin_firmware_bridge --ctest-args \
  -R "test_profile_corpus|test_profile_parser|test_e3_noise_slew"
```

---

## 6. 발행 필드 추가 (새 데이터를 ROS 로 내보내기)

### 순서

1. **메시지 정의** — `../mgs_tp_msgs/msg/*.msg`. 주석에 **단위와 미판독 표현**을 적는다.
2. **`mgs_tp_msgs` 를 먼저 빌드**한다.
3. `ros/rd_telemetry.cpp` 의 해당 발행 함수에 채운다.
4. **신선도 게이트를 반드시 만든다.**

```cpp
const bool xxx_fresh = Reads(ecu::REG_XXX_OFFSET, ecu::REG_XXX_SIZE);
m.my_field = xxx_fresh ? reg.xxx.value * kScale : NaNf();      // float
m.my_code  = xxx_fresh ? reg.xxx.code          : kUnread;      // uint8 (0xFF)
```

5. **`delta_tick` 이 있는 블록이면 그것도 본다** — 채널별 `delta_tick` 이 있으면 채널별로.

```cpp
const bool ch_live = xxx_fresh && reg.xxx.delta_tick[i] != ecu::DELTA_STALE;
```

### 세 겹 판정이 필요한 이유 (실기 이력)

| 무엇만 봤을 때 | 무슨 일이 났나 |
|---|---|
| `state` 만 | IMU 가 `lc=1/hs=0`("정상")인데 데이터가 안 와서 `orientation=(0,0,0,0)` 이 발행됐다 |
| `delta_tick` 만 | 한 번도 안 읽은 블록은 `delta_tick == 0` = "아주 신선함" 으로 보인다 |
| 값의 그럴듯함 | 죽은 엔코더 채널의 12bit 잔값이 **정상 각도로 나갔다** |

→ `Reads()` **먼저**, 그다음 `delta_tick`, 그다음 `state`.

### 200Hz 경로(`OnFeedback`)에 추가한다면

- **메시지 구성은 스케줄 스레드에서 끝낸다.** 발행 시점에 섀도를 다시 읽으면
  tick 시점의 값이 아닌 것이 나간다.
- 값이 큐를 통과하므로 `TeleItem_t` 크기가 늘어난다 (256 슬롯 × sizeof).

### 새 카운터를 추가한다면

`test_counter_producers` 가 **소스를 읽어** 생산자 존재를 확인한다.
생산자 없는 카운터는 늘 0 이고, **0 은 "사고가 없었다" 와 구분되지 않는다** —
`irregular_tick_cnt` 가 실제로 그 상태로 방치돼 있었다.

---

## 7. 기동 파라미터 추가

### 순서

1. `rd_config.hpp` 의 `BridgeConfig` 에 필드 + 기본값.
2. `ros/rd_node.cpp` 생성자에서 `declare_parameter` 후 채운다.
3. 하위 계층은 `const BridgeConfig&` 로 받으므로 **추가 배선이 없다.**

### ⚠ 넣으면 안 되는 것

**런타임에 변하는 값.** `auto_mode` 현재값·FSM 상태·tick 카운터는 설정이 아니다.
런타임 토글이 필요하면 소유자 쪽 `std::atomic` 으로 두고 param 콜백이 민다
(`RdCarrierApi::guard_enable_` 이 그 예).

### 오타를 어떻게 다룰 것인가

**기동을 거부한다.** 기본값으로 떨어뜨리면 조작자가 의도하지 않은 모드로 뜨는데,
그건 **모터가 도는 모드인지 아닌지가 갈리는 문제**다.

```cpp
if (!Parse(s, &config_.field)) {
    RCLCPP_ERROR(...,"field='%s' 는 알 수 없음 — a|b|c 중 하나", s.c_str());
    config_.field_valid = false;         // → MainLoopStart 가 통신 시도 없이 exit≠0
}
```

`RdSchedule::MainLoopStart()` 의 검증 블록에 `*_valid` 를 추가한다 —
**통신 시도 전에** 판정해야 "Waiting for USB..." 만 찍히며 매달리지 않는다.

### 정수도 받고 싶으면

```cpp
rcl_interfaces::msg::ParameterDescriptor d;  d.dynamic_typing = true;
const rclcpp::ParameterValue v = declare_parameter("name", rclcpp::ParameterValue(std::string("기본")), d);
if (v.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) { ... } else { ... }
```

없으면 `-p name:=2` 를 주는 순간 타입 불일치로 **노드가 죽는다** (exit 134).

---

## 8. 레지스터 맵 변경 (STM 과 함께)

**난이도: 높음. 두 워크스페이스를 동시에 고쳐야 한다.**

### 순서

1. **STM 먼저** — `stm_ws/ECU_V3/Core/Inc/rd_register_ecu.h`.
2. **Orin 미러** — `core/rd_register_ecu.hpp`. 구조체·`REG_*_OFFSET/SIZE`·`static_assert`.
3. 오프셋이 바뀐 구간을 참조하는 곳을 전부 찾는다:

```bash
grep -rn "REG_XXX_OFFSET\|REG_XXX_SIZE" include/ src/ test/
```

4. **프리셋 세그먼트**(`rd_read_preset.hpp`)와 **카탈로그 구간**(`rd_command_catalog.hpp`)을 갱신.
5. `RdTelemetry` 의 필드 매핑 확인.

### 확인

```bash
colcon test --packages-select orin_firmware_bridge --ctest-args -R "test_stm_mirror|test_golden_wire"
```

> `test_stm_mirror` 는 **`stm_ws` 가 함께 있을 때만** 빌드된다 (Orin 배포본에는 없다).
> `rd_register_ecu.hpp` 의 `static_assert` 는 **크기만** 보므로 **필드 순서 변경을 못 잡는다** —
> 그래서 이 테스트가 따로 있다. STM 을 고쳤으면 **반드시 개발 머신에서 돌린다.**

### 하위호환 고려

`LOADCELL`(42) 이하 주소를 건드리면 **기존 bag·분석의 오프셋 가정이 깨진다.**
2026-07-27 재배치가 42 이상만 건드린 것이 그 이유다.

---

## 9. 골든 파일 갱신 (`test/golden/ecu_wire.txt`)

**골든 불일치는 예외 없이 롤백이 원칙이다.**
기대값을 고치는 것은 **"동작을 의도적으로 바꾼다" 는 별도 결정**일 때만이다.

```bash
# 1) 먼저 diff 를 눈으로 본다 — 무엇이 왜 바뀌었는지 설명할 수 있어야 한다
colcon test --packages-select orin_firmware_bridge --ctest-args -R test_golden_wire
colcon test-result --verbose

# 2) 의도한 변경이 맞을 때만
RD_GOLDEN_UPDATE=1 ./build/orin_firmware_bridge/test_golden_wire
git diff src/orin_firmware_bridge/test/golden/ecu_wire.txt      # 반드시 리뷰
```

골든 파일은 **소스 트리**에 있다 (빌드 디렉터리는 지워지므로).
`# ⚠ 이 파일을 손으로 고치지 말 것` — shadow 는 `raw[i] = (i*7+13) & 0xFF` 결정론적 패턴이다.

---

## 10. 새 보드 추가 (ECU/DPC/PCU 다음)

1. `core/rd_register_yyy.hpp` — 레지스터 맵 + `static_assert`.
2. `core/rd_comm.hpp` — `PacketID::YYY` 추가.
3. `core/rd_map.hpp`
   - `namespace TARGET` 에 상수
   - `struct YyyState_t { CommHealth_t comm; yyy::REGISTER_t reg; };`
   - `RobotState_t` 에 멤버
   - `ShadowBase()` 에 case
4. `core/rd_map.cpp` — `Encode`/`Decode` 의 `switch (target_id)` 두 곳.
   > `EncodeNode`/`DecodeNode` 는 템플릿이라 **본문은 안 고쳐도 된다** —
   > `{CommHealth_t comm; REGISTER_t reg;}` 레이아웃만 만족하면 된다.
5. `policy/rd_command.cpp` — `TargetIndex()` / `TargetName()`,
   `blackout_until_` 배열 크기(현재 3).
6. `policy/rd_status.cpp` — `TargetName()`.
7. `sched/rd_slot_table.hpp` — `SlotId::YYY`, `SlotIdName()`, `YyyRd()` 도우미, 프레임에 배치.
8. `sched/rd_schedule.cpp` — `bake()` 의 `tgt` 결정 삼항식, 게이트의 `enable_yyy_read`.
9. `ros/rd_telemetry.cpp` — `NodeStatus` 발행자 + 매핑.
10. `rd_config.hpp` — `enable_yyy_read` (**기본 off** — 보드 미장착 시 timeout 폭주 방지).

> **공용 매핑을 기대하지 말 것.** DPC 만 해도 SYS 위치(46 vs 16)·`hw_*` 순서·tick 단위가
> ECU 와 다르다. 보드별로 적는 것이 그 결정의 대가다.

---

## 11. 자주 밟는 함정

| 증상 | 원인 | 볼 곳 |
|---|---|---|
| `PrepareWrite` 가 아무것도 안 한다 | `control_.Bind()` 미호출 → `shadow_ == nullptr` 로 **조용히 no-op** | `rd_node.cpp` 생성자 |
| 발행값이 전부 0 / 계측이 죽었다 | L3 3개를 **파라미터 파싱 전에** 만들었다 | `rd_node.cpp` 생성 순서 |
| `cli status` 가 안 된다 | FSM 이 INIT — `MarkInitDone()` 미호출 → `AcceptsConfig` 전부 거부 | `rd_schedule.cpp` `RunLoop` 모드 분기 |
| `SET_CTR_MODE` 가 **항상** 성공한다 | 프리셋이 `{128,4}` 를 안 읽어 자기가 쓴 값을 자기가 읽는다 | `rd_read_preset.hpp` |
| 낡은 값이 신선하게 보인다 | `sink_->SetReadPreset()` 통지 누락 | `rd_schedule.cpp::SetReadPreset` |
| 종료 시 SIGSEGV | 발행 스레드가 `rclcpp::ok()` 를 안 본다 / L2·L3 선언 순서 | `rd_telemetry.cpp`, `rd_node.hpp` |
| 주기가 밀린다 | RT 루프 안 동기 로깅 · `tcdrain` · 무거운 프리셋 | Heartbeat 의 `wake_max` vs `proc_max` |
| 데드락 | 락 순서 위반 (`state_mutex → mutex_ → ctr_mutex_`) | `rd_control.hpp` 주석 |
| 새 카운터가 늘 0 | 생산자가 없다 (0 과 "사고 없음" 이 구분 안 됨) | `test_counter_producers` |

### 고칠 때 발견한, 아직 안 고친 것

> 이 문서를 쓰며 코드에서 확인한 불일치다. **고치기 전에 영향 범위를 확인할 것.**

| 위치 | 내용 |
|---|---|
| ~~`CommandSet.srv:49` + `cli.py:268` 의 `TARGET_DPC` 불일치~~ | **해소됨 (2026-08-03)** — DPC-B ID 를 0xD1 로 확정하면서 srv/CLI/브리지를 209(0xD1)로 통일했다 |
| `RdSequence` | `Tick()` 이 **200Hz** 로 불린다(주석은 5Hz). `kWaitTicksMax=150` 은 30초가 아니라 **약 0.75초** |
| `src/rd_sequence.cpp` (루트) | 빌드에 포함되지 않는 잔재. 실물은 `src/policy/rd_sequence.cpp` |
| `rd_common.hpp` `HW_BIT_*` | 정의만 있고 사용처 0 — 이름이 실제 채널과 맞는지 아무도 검증하지 않는다 |
