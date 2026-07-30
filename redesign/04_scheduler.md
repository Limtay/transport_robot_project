# 04 — 스케줄러 · 슬롯 테이블 · 커맨드 처리 · CLI

> 선행: [00](00_overview.md) · [01](01_modes.md) · [02](02_layers.md) · [03](03_interfaces.md).
> 이 문서가 정하는 것: 슬롯 테이블의 데이터 형식, tick 루프 의사코드, `duration`·재시도·blackout
> 처리 규칙, `GET_STATUS` JSON 스키마, `control_cli` 명령 체계, DPC/PCU 표준화 제안.
> 이 문서가 정하지 않는 것: 프로파일 YAML 스키마(05), 마이그레이션 순서(06).

---

## 1. 확정 결정 (C1~C5)

| # | 결정 |
|---|------|
| C1 | 슬롯 테이블은 **`constexpr` 데이터**. 슬롯 = **`SlotId`(누구에게) × `SlotInst`(무엇을) + `ReadSpan`/`WriteSpan`(어느 구간)**. 세그 개수·응답 크기 같은 파생값은 저장하지 않고, 조합 유효성(구조만)과 wire 예산을 슬롯별 `static_assert` 로 검증 (§2) |
| C2 | `once` 만료를 **시간 기준 → 시도 횟수 기준**으로 변경. blackout 중 skip 은 시도로 세지 않는다 (버그 수정, §3) |
| C3 | `GET_STATUS` 는 정형 JSON. AI 자동화의 파싱 계약 (§4) |
| C4 | `control_cli` 가 `command_cli` 를 흡수. 의미 단위 `cmd` + `manual` 전용 `raw` (§5) |
| C5 | DPC/PCU 는 **ECU 와 동일한 SYS 레이아웃(16~32)** 을 채택하도록 제안 → `NodeStatus` 매핑이 자동 (§6) |

---

## 2. C1 — 슬롯 테이블

### 2.1 원칙

R1: **모드는 설정(데이터)이지 코드 분기가 아니다.** 새 모드 추가 = 테이블 항목 추가이며
`rd_schedule.cpp` 는 건드리지 않는다. 현행 `RunLoop()` 의 traction/control/일반 3중 `if-else`(P5)가
여기서 사라진다.

### 2.2 자료구조 — 슬롯 테이블이란 무엇인가

#### 2.2.0 먼저: 이게 무슨 물건인가

스케줄 스레드는 **5ms 마다 한 번씩 깨어나 RS485 트랜잭션을 정확히 1건** 수행한다.
그 1회를 **tick** 이라 부른다. "이번 tick 에 무엇을 할 것인가"를 매번 `if-else` 로 판단하는 대신
**표에서 찾아본다** — 그 표가 슬롯 테이블이다.

```
tick_count:   0    1    2    3    4    5    6    7    8    9   10   11 ...
                └─────────────── 프레임 1주기 (10 tick = 50ms) ────────┘ └ 반복
tick_in_frame = tick_count % 10
슬롯 조회    = frame.slots[tick_in_frame]
```

용어 3개가 층을 이룬다:

| 용어 | 무엇인가 | 비유 |
|---|---|---|
| **tick** | 5ms 짜리 시간 칸 1개. RS485 트랜잭션 1건이 들어간다 | 악보의 8분음표 1개 |
| **슬롯(`SlotDef`)** | 그 칸에서 **누구에게 무슨 명령을 보낼지** 적어둔 것 | 그 음표에 적힌 음 |
| **프레임(`FrameDef`)** | 슬롯을 N개 늘어놓은 **반복 단위** | 한 마디 |

`kFrameProject` 의 `frame_ticks = 10` 은 "10 tick(=50ms) 마다 같은 패턴이 반복된다"는 뜻이고,
그 안에 ECU RW 가 5칸 있으므로 **ECU 는 50ms 에 5번 = 100Hz** 로 통신한다. `kFrameControl` 은
`frame_ticks = 1` 이고 그 1칸이 ECU RW 이므로 **매 tick = 200Hz** 다. Q3 의 "슬롯 10칸" 이
바로 이 표를 가리킨다.

**핵심은 `constexpr` 이라는 점이다.** 이 표는 컴파일 타임에 확정되는 상수 데이터이므로
① 런타임에 바뀌지 않아 200Hz 스레드와 레이스가 없고, ② 표 자체를 컴파일 타임에 검산할 수 있다
(§2.5 의 `static_assert`). R1 의 **"모드는 설정(데이터)이지 코드 분기가 아니다"** 는 곧
"모드를 늘리는 일 = 이 표에 항목 하나 추가, `rd_schedule.cpp` 는 손대지 않음" 이라는 뜻이다.

#### 2.2.1 슬롯 = **ID(대상) × INST(명령)**

슬롯이 답해야 하는 질문은 사실 **둘**이다 — "**누구에게**" 와 "**무엇을**".
이 둘을 한 enum 에 섞으면 (`ECU_RW`, `ECU_READ`, `DPC`, `PCU` …) 조합이 늘 때마다 값이
곱셈으로 불어나고, 규칙이 바뀔 때 enum 자체를 고쳐야 한다. **직교하는 두 축으로 분해한다.**

```cpp
// sched/rd_slot_table.hpp   (L1, rclcpp 없음)

// 누구에게 — 트랜잭션 상대
enum class SlotId : uint8_t {
    EMPTY = 0,   // 아무에게도 — tick 여유 (커맨드에 양보되는 자리)
    ECU,
    DPC,
    PCU,
    COMMAND,     // 늦은 바인딩: 대상·명령을 런타임에 커맨드 슬롯이 정한다 (§2.2.2)
};

// 무엇을 — RS485 명령 종류
enum class SlotInst : uint8_t {
    NONE = 0,    // EMPTY / COMMAND 전용 (테이블이 정하지 않음)
    READ,
    WRITE,
    RW,          // 한 트랜잭션에 write + read 동시 (01 §3)
};

// 무엇을 읽는가 — 세그먼트 목록만 적는다. 개수·응답크기는 파생 (§2.2.3)
struct ReadSpan {
    Segment_t segs[TaskConfig_t::MAX_READ_SEGS] = {};   // len == 0 인 칸은 미사용

    constexpr uint8_t SegCount() const {
        uint8_t n = 0; for (auto& s : segs) if (s.len) n++; return n;
    }
    constexpr uint16_t RespPayload() const {            // err(1) + 세그 길이 합
        uint16_t b = 1; for (auto& s : segs) b += s.len; return b;
    }
};

// 무엇을 쓰는가 — 구간을 누가 정하는지가 두 갈래다
enum class WriteSrc : uint8_t {
    NONE = 0,     // write 없음 (READ 슬롯)
    FIXED,        // 테이블이 명시한 고정 구간
    AUTO_MODE,    // ECU 전용 — auto_mode 가 6종 중 런타임에 고른다 (01 §3.3)
};
struct WriteSpan {
    WriteSrc  src = WriteSrc::NONE;
    Segment_t seg = {0, 0};                            // src == FIXED 일 때만 유효

    constexpr uint16_t MaxLen() const {                 // 예산 검증용 **최악값**
        return src == WriteSrc::AUTO_MODE ? kAutoModeMaxWrite   // 52 = DIRECT(128:52)
             : src == WriteSrc::FIXED     ? seg.len
             : 0;
    }
};

struct SlotDef {
    SlotId    id        = SlotId::EMPTY;
    SlotInst  inst      = SlotInst::NONE;
    ReadSpan  read      = {};    // inst ∈ {READ, RW}
    WriteSpan write     = {};    // inst ∈ {WRITE, RW}
    uint8_t   cmd_index = 0;     // id == COMMAND 일 때 커맨드 슬롯 번호
};
```

**읽기·쓰기 구간이 슬롯 안에 있는 것이 핵심이다.** 초안은 `ReadPreset` 을 `FrameDef` 에 하나만
두고 프레임 전체가 공유했는데, 그러면 **DPC·PCU 슬롯이 무엇을 읽는지 표현할 방법이 없다** —
§7 의사코드의 `DPC/PCU: ExecuteTask(preset)` 은 ECU 용 프리셋으로 DPC 를 읽으라는 뜻이 되어 버린다.
현행 `TaskConfig_t`(`rd_map.hpp:32~64`)가 이미 `target_id` · `inst` · write 구간(`start_addr`/
`data_len`) · read 세그(`segs[]`)를 **한 구조체에** 담고 있으며, 그 형태가 옳다.

슬롯이 자기 구간을 들고 있으면 얻는 것:

| | 프레임이 프리셋 1개 공유 (초안) | 슬롯이 구간 소유 (확정) |
|---|---|---|
| DPC/PCU 읽기 범위 | **표현 불가** | `DpcRd(kPresetDpc)` |
| 한 프레임에서 ECU 를 두 구간으로 번갈아 읽기 | 불가 | 슬롯마다 다른 `ReadSpan` |
| 표를 읽는 사람 | 슬롯만 봐서는 무엇을 읽는지 모른다 | **한 줄에 대상·명령·구간이 다 보인다** |
| 커맨드·CLI 명령 조립 | 프레임 프리셋을 따로 찾아야 한다 | 슬롯 구조 그대로 = 명령 구조 |
| wire 예산 검증 | 프리셋별 수동 나열 | **슬롯별 자동** (§2.5) |

**이 분해가 사주는 것**:

| 상황 | 통합 enum | ID × INST |
|---|---|---|
| DPC 에 WRITE 를 보내야 한다 (C5 협의 후) | `DPC_WRITE` 값 추가 + 모든 `switch` 갱신 | `{DPC, WRITE}` — **테이블만 수정** |
| 4번째 보드 추가 | `XXX_READ`/`XXX_RW` 2개 추가 | `SlotId` 에 1개 추가 |
| project 를 ECU READ 로 돌려보고 싶다 | 새 enum 필요 없음(우연히 존재) | `{ECU, READ}` |
| "WRITE 계열은 `safe_stop` 검사" 같은 **명령 성격별 규칙** | 대상별로 흩어진 값을 일일이 열거 | `inst` 하나로 판정 |

마지막 줄이 실질적으로 가장 크다. 01 §6.2 의 게이트, §7.4 의 롤백, §2.5 의 예산 검증은 전부
**"무엇을(inst)"에만 의존**하고 "누구에게(id)"와는 무관하다. 축이 갈라져 있으면 그 규칙들을
`id` 를 몰라도 쓸 수 있다.

#### 2.2.2 `COMMAND` 는 왜 `SlotId` 에 남는가

`{ECU, READ}` 처럼 `COMMAND` 도 `(id, inst)` 로 쪼갤 수 있을 것 같지만 **쪼갤 수 없다** —
그 칸의 대상과 명령은 **테이블이 모르기 때문**이다. 사용자가 `control_cli cmd read dpc all` 을
넣으면 그때 `(DPC, READ)` 가 되고, `cmd reboot ecu` 면 `(ECU, WRITE)` 가 된다.

즉 `SlotId::COMMAND` 는 대상의 이름이 아니라 **"이 칸의 `(id, inst)` 는 런타임에 결정된다"는
표식**이다. 늦은 바인딩(late binding)을 테이블에 명시하는 자리이며, 그래서 `inst = NONE` 이다.

```cpp
// tick 디스패치에서 늦은 바인딩이 풀리는 지점 (§7 ②)
case SlotId::COMMAND:
    if (!command.GetSlotTask(slot.cmd_index, &task)) continue;   // (id, inst) 가 여기서 확정
    ...
```

#### 2.2.3 프레임 — 파생 가능한 값은 저장하지 않는다

```cpp
struct FrameDef {
    uint8_t  frame_ticks;        // project/manual = 10, control = 1
    SlotDef  slots[10];
    uint16_t user_slot_mask;     // 사용자가 교체 가능한 프레임 위치 (bit i = tick i)

    // 커맨드 레지스트리 용량 = 사용자가 덮어쓸 수 있는 tick 수 (§2.4.3)
    constexpr uint8_t CmdCapacity() const { return popcount(user_slot_mask); }
};
```

`ReadSpan` 의 세그 개수·응답 크기, `FrameDef` 의 커맨드 슬롯 개수는 **전부 세그먼트 목록에서
계산되는 값**이다. 저장 필드로 두면 두 가지가 나빠진다:

1. 표를 쓰는 사람이 **바이트를 손으로 세야 한다.** `{16,17},{48,22},{86,42}` 를 적고 나서
   `82` 를 따로 계산해 적는 일은 프리셋을 추가할 때마다 반복된다
2. **어긋날 수 있다.** 세그를 하나 늘리고 `resp_payload` 갱신을 잊으면, §2.5 의 예산
   `static_assert` 는 **거짓말을 검증**한다 — 검증 장치가 통과시켜 버리는 최악의 실패다

그래서 `constexpr` 메서드로 파생시킨다. 표에 적는 것은 **세그먼트 목록 하나뿐**이다.

```cpp
constexpr ReadSpan kPresetControl = { {{16,17}, {47,81}} };   // 끝. 개수·크기는 파생.
```

#### 2.2.4 유효성 검사는 **구조**만 본다 — 정책은 넣지 않는다

컴파일 타임 검사에 무엇을 넣을지가 갈림길이다. 초안은 `DPC/PCU 는 READ 만` 처럼
**"지금 펌웨어가 지원하는 범위"** 를 넣었는데, 이것은 잘못이다:

- ECU 는 이미 WRITE 를 한다 — `motor_mask`(192)·`mode`(190)·`auto_mode`(188)·`REBOOT` 는
  전부 out-of-span WRITE 다 (01 §6.3 C). `ECU: READ|RW` 규칙은 **당장 현실과 어긋난다**
- DPC/PCU WRITE 도 곧 생긴다 (C5). 그때마다 `IsValidSlot` 을 고쳐야 하는데, 그건
  **규칙이 아니라 진행 상황**이다. 진행 상황을 `static_assert` 에 넣으면 검사기가 매번 낡는다
- 거부 규칙이 대상별로 갈라지면 조합 수만큼 늘어난다 — ID×INST 로 나눈 이유가 사라진다

**결정: `ECU`/`DPC`/`PCU` 는 `READ`/`WRITE`/`RW` 를 전부 개방한다.**
컴파일 타임 검사는 **"명령과 구간이 서로 맞는가"** 라는 구조적 정합성만 본다.
"DPC 에 WRITE 를 보내도 되는가" 는 펌웨어 지원 여부의 문제이므로 **런타임 거부**
(`enable_dpc` + 명령 수락 단계)로 처리한다 — 그쪽은 값을 바꾸면 끝이다.

```cpp
constexpr bool IsValidSlot(const SlotDef& s) {
    const bool has_r = s.read.SegCount() > 0;
    const bool has_w = s.write.src != WriteSrc::NONE;

    switch (s.id) {
    // 테이블이 명령을 정하지 않는 두 종류 — 구간도 없어야 한다
    case SlotId::EMPTY:
    case SlotId::COMMAND:
        return s.inst == SlotInst::NONE && !has_r && !has_w;

    // 보드 슬롯 — 세 명령 모두 개방. 구간이 명령과 맞는지만 본다
    case SlotId::ECU:
    case SlotId::DPC:
    case SlotId::PCU:
        switch (s.inst) {
        case SlotInst::READ:  return  has_r && !has_w;
        case SlotInst::WRITE: return !has_r &&  has_w;
        case SlotInst::RW:    return  has_r &&  has_w;
        case SlotInst::NONE:  return false;
        }
    }
    return false;
}
```

남는 제약은 **하나뿐**이고 그것도 구조적이다: `WriteSrc::AUTO_MODE` 는 ECU 전용이다
(`auto_mode` 는 ECU 레지스터 188 이고 6종 write 범위가 전부 ECU 주소다 — 01 §3.3).
DPC/PCU 도 WRITE 는 얼마든지 하되 `WriteSrc::FIXED` 로 구간을 명시하면 된다.

> **`COMMAND` 슬롯은 컴파일 타임에 검사할 수 없다** — `(id, inst)` 와 구간이 런타임에 정해지기
> 때문이다. 그래서 커맨드 슬롯의 유효성·예산 검사는 **명령 수락 시점**에 한다 (§3, §5.2 `cmd`).
> 정적 검사가 닿지 않는 유일한 자리이므로 런타임 검사가 필수다.

#### 2.2.5 슬롯 축약 생성자 — 프레임은 이름만 나열한다

`SlotDef` 를 통째로 쓰면 §2.4 가 읽기 어려워지므로, 모드가 쓰는 조합에 이름을 준다.
**읽기·쓰기 구간을 인자로 받는 것**이 핵심이다 — 슬롯을 보는 순간 무엇을 읽고 쓰는지 보인다.

```cpp
constexpr SlotDef kSlotEmpty = {};
constexpr SlotDef EcuRw(ReadSpan r) { return {SlotId::ECU, SlotInst::RW,   r, {WriteSrc::AUTO_MODE}}; }
constexpr SlotDef EcuRd(ReadSpan r) { return {SlotId::ECU, SlotInst::READ, r, {}}; }
constexpr SlotDef DpcRd(ReadSpan r) { return {SlotId::DPC, SlotInst::READ, r, {}}; }
constexpr SlotDef PcuRd(ReadSpan r) { return {SlotId::PCU, SlotInst::READ, r, {}}; }
constexpr SlotDef Cmd(uint8_t i)    { return {SlotId::COMMAND, SlotInst::NONE, {}, {}, i}; }
// 예: DPC WRITE 가 생기면 (C5) — IsValidSlot 수정 없이 그대로 쓴다
constexpr SlotDef DpcWr(Segment_t w){ return {SlotId::DPC, SlotInst::WRITE, {}, {WriteSrc::FIXED, w}}; }
```

### 2.3 읽기 프리셋 (01 §5.3 + lc/hs 반영)

세그먼트 목록만 적는다 — 개수·응답 크기는 `SegCount()`/`RespPayload()` 가 파생시킨다 (§2.2.3).

```cpp
constexpr ReadSpan kPresetProject     = { {{16,17}, {48,22}, {86,42}} };  // 엔코더(70~85) 제외
constexpr ReadSpan kPresetControlTest = { {{16,17}, {42,6},  {88,40}} };  // 로드셀 (견인 실험)
constexpr ReadSpan kPresetControl     = { {{16,17}, {47,81}}           };  // 로드셀+IMU+엔코더+모터
constexpr ReadSpan kPresetDiag        = { {{0,16},  {16,17}, {224,32}} };  // INIT·진단
constexpr ReadSpan kPresetDpc         = { /* C5 협의 후 확정 — 04 §6.3 */ };
constexpr ReadSpan kPresetPcu         = { /* C5 협의 후 확정 — 04 §6.3 */ };
```

**변경 2건 (2026-07-27 확정)**:

1. **모든 프리셋이 `{16,17}` 로 시작한다** (기존 `control` 계열은 `{26,7}`). SYS 블록 전체
   — `degraded_cnt[8]`·`hw_error`·`hw_fatal`·`hw_reset`·`sys_state`·`realtime_tick`·`proc_delta` —
   를 상시 확보한다. 이로써 `NodeStatus` 매핑(03 §5.3)이 **보드·모드 분기 없이** 성립하고,
   "`{16,17}` 전체를 읽은 트랜잭션에서만 발행" 규칙이 모든 프리셋에서 만족된다
2. `kPresetControl` 의 `{48,80}` → **`{47,81}`** (1바이트 확장). 로드셀 `STATE_t`(addr 47)를
   포함시켜 진단 인덱스 6개가 **모두** 들어온다 — 아래

**lc/hs 가 거의 공짜인 이유**: 진단 채널 6개의 `STATE_t` 는 각 센서 블록 **끝바이트**에 흩어져 있는데
(03 §3.1), `control` 프리셋의 `{48,80}`(=48~127)이 이미 5개를 덮고 있었다. 발행만 안 하고 있었다.

| idx | 채널 | `STATE_t` 주소 | `kPresetControl` |
|---|---|---|---|
| 0 | RC (uart1) | 87 | ✔ |
| 1 | RS485 (uart2) | 86 | ✔ |
| 2 | IMU (uart6) | 69 | ✔ |
| 3 | 모터 (can1) | 127 | ✔ |
| 4 | 엔코더 (i2c1) | 85 | ✔ |
| 5 | 로드셀 | **47** | `{47,81}` 로 1바이트 확장 → ✔ |

> **`kPresetControlTest` 는 idx 1·2·4 를 읽지 않는다** (IMU·엔코더·uart2 블록이 세그 밖).
> 그 자리는 **`0xFF` = "이번 프리셋이 읽지 않음"** 센티널로 발행한다 — `delta_tick` 의
> `0xFF = stale` 관례와 같은 방식이며, "값이 없다" 와 "OK(0)" 를 구분하지 못하는 사고를 막는다.

### 2.4 모드별 프레임

```cpp
// project — 10 tick = 50ms (20Hz). ECU 100Hz.
constexpr FrameDef kFrameProject = {
    /*frame_ticks*/ 10,
    /*slots*/ {
        DpcRd(kPresetDpc),        // tick 0
        EcuRw(kPresetProject),    // tick 1
        PcuRd(kPresetPcu),        // tick 2
        EcuRw(kPresetProject),    // tick 3
        Cmd(0),                   // tick 4
        EcuRw(kPresetProject),    // tick 5
        Cmd(1),                   // tick 6
        EcuRw(kPresetProject),    // tick 7
        Cmd(2),                   // tick 8
        EcuRw(kPresetProject),    // tick 9
    },
    /*user_slot_mask*/ 0b0101010000,   // tick 4,6,8 만 교체 가능
};

// control — 매 tick ECU RW (200Hz). 대체 불가 프레임 (§2.4.1)
constexpr FrameDef kFrameControl = {
    1, { EcuRw(kPresetControl) }, 0b0000000000 };

// manual — 배치는 project 와 같지만 ECU 는 READ, 10칸 전부 교체 가능
constexpr FrameDef kFrameManual = {
    10, { DpcRd(kPresetDpc),     EcuRd(kPresetProject), PcuRd(kPresetPcu),
          EcuRd(kPresetProject), Cmd(0),                EcuRd(kPresetProject),
          Cmd(1),                EcuRd(kPresetProject), Cmd(2),
          EcuRd(kPresetProject) },
    0b1111111111 };                                // 전부 교체 가능 (01 §4.1)
```

- **슬롯 한 줄에 대상·명령·구간이 다 보인다.** `DpcRd(kPresetDpc)` 는 "DPC 에게 READ 를 보내되
  DPC 프리셋 구간을 읽는다" 이고, 초안처럼 프레임 프리셋을 찾아 거슬러 올라갈 필요가 없다
- `cmd_slot_count` 는 사라졌다 — 용량은 `CmdCapacity()` = `popcount(user_slot_mask)` 다 (§2.4.3)
- `user_slot_mask` 가 01 §4.1 "manual 은 10칸 전부 자유"와 §5.1 "project 는 3칸"을 **데이터로** 표현한다
- DPC/PCU 가 비활성(`enable_dpc/pcu=false`)이면 그 tick 은 `kSlotEmpty` 로 치환되어 커맨드에 양보한다
- `manual` 이 `EcuRd(...)`(= `{ECU, READ}`)인 것이 R2 "write 가 전무한 모드"를 **테이블 한 칸으로**
  표현한 것이다. 이제 "READ 라서 write 게이트 대상이 아니다"를 `inst` 만 보고 판정할 수 있다

### 2.4.1 `user_slot_mask == 0` = 대체 불가 프레임 (확정)

`kFrameControl` 은 커맨드 슬롯이 0개다. 그런데 01 §6.3 요약 결정표는 여러 명령을
**"슬롯 소비 ○"** 라고 적어서, 글자대로 읽으면 `control` 은 `SET_ACTIVE_MOTORS`(192)·
`SET_AUTO_MODE`(188)·`REBOOT` 를 못 한다는 뜻이 된다 — 그건 `ControlConfig` 서비스가 하는 일
그 자체다 (03 §6). 모순처럼 보이지만, 원인은 **표가 서로 다른 두 메커니즘을 한 열에 뭉갠 것**이다.

| 메커니즘 | 무엇인가 | 슬롯이 필요한가 |
|---|---|---|
| **커맨드 슬롯** | *반복·지속* 명령의 거치대 (`duration: forever`/`N초`) — 프레임에 물리적 칸이 있어야 매 주기 돌아온다 | **필요** |
| **1 tick 대체 (OOS)** | *단발* 명령이 다음 tick 하나를 통째로 빌리는 것. 슬롯 조회 **이전**에 실행된다 (§7 ①) | **불필요** |

§7 의사코드의 `oos.TakeStep()` 이 슬롯 조회보다 앞에 있으므로, 프레임에 칸이 없어도 단발 명령은
동작한다. 따라서 커맨드 슬롯이 없다고 해서 명령을 못 넣는 것이 아니다.

**확정 규칙 — 프레임 하나로 표현한다:**

```
user_slot_mask == 0  ⇒  이 프레임은 **슬롯 대체 불가**.
                        명령 삽입 경로는 1 tick 대체(OOS) 하나뿐이다.
                        반복 실행이 필요하면 → 프리셋 read 영역 교체 (§2.4.2)
                                            또는 다른 슬롯 셋(모드) 으로 기동
```

이 한 줄이 `control` 을 특수 케이스로 만들지 않는다. "control 은 커맨드를 못 넣는다" 가 아니라
**"대체 불가 프레임은 1 tick 대체만 된다"** 는 일반 규칙이고, `control` 은 그 규칙을 만족하는
프레임 하나일 뿐이다. 나중에 다른 모드가 같은 성질을 원하면 마스크를 0 으로 두면 된다.

> **01 §6.3 표 정정 필요**: "슬롯 소비" 열에서 out-of-span WRITE·`REBOOT`·예산초과 READ 를
> **✗ 로 고친다** — 그것들은 tick 을 대체할 뿐 슬롯을 쓰지 않는다. 슬롯을 소비하는 것은
> **정기(반복) READ 뿐**이며, 그마저 §2.4.2 로 대체된다.

### 2.4.2 정기 센서 변경 = 프리셋 교체 (커맨드 슬롯이 아니다)

"IDLE 에서 다른 센서를 계속 보고 싶다" 는 요구를 **커맨드 슬롯으로 푸는 것은 층이 틀렸다.**
그건 새 트랜잭션을 프레임에 끼우는 일이 아니라, **기존 슬롯이 읽는 구간을 바꾸는 일**이다.

| | 커맨드 슬롯 | **프리셋 교체** |
|---|---|---|
| 하는 일 | 프레임에 **별도 트랜잭션**을 끼운다 | **같은 슬롯**의 read 구간을 바꾼다 |
| 쓰는 자리 | 다른 보드를 읽을 때 (DPC·PCU) | 같은 보드의 다른 구간을 읽을 때 |
| tick 소모 | 슬롯 1칸 | **0** |
| 피드백 | 그 tick 은 필드 구성이 다름 | 발행 메시지 구성이 통째로 바뀜 |
| `control` 에서 | 자리 없음 | **가능** |

`control` 이 IDLE 에서 진단 구간을 보고 싶으면 `kPresetControl` → `kPresetDiag` 로 갈아끼우면
되고, 이건 슬롯 없이 된다. 즉 §2.4.1 의 "대체 불가" 는 기능 결손이 아니다.

**교체 규칙**:

- 프리셋은 **미리 만들어 둔 것 중에서 고른다** — 01 §3.3 의 write 범위 셀렉터와 같은 방식
  (`constexpr` 배열 + atomic 인덱스). 200Hz 스레드가 읽는 데이터를 런타임에 조립하지 않는다
- **교체는 `IDLE` 에서만.** RUNNING 중 교체는 발행 메시지의 필드 구성을 바꿔 실험 시계열을
  중간에 갈라놓는다 — R6("정기 READ = IDLE 전용")이 지키려던 것이 바로 이것이다
- 교체 시 `static_assert` 로 이미 예산이 검증된 프리셋만 고를 수 있으므로 wire 예산은 자동으로 안전하다

**(가) 채택 — IDLE/RUNNING 프레임 분리는 하지 않는다.** 프레임을 상태별로 둘로 나누면 전환 시점·
tick 카운터 리셋·슬롯 내용 보존 규칙이 새로 필요한데, 위 두 경로(1 tick 대체 + 프리셋 교체)로
실제 요구가 이미 충족된다. 그 이상이 필요한 상황은 **`manual` 로 기동하거나 슬롯 셋을 하나 더
정의**하는 것이 맞다 — 슬롯 테이블을 데이터로 만든 것이 바로 그러라고 만든 확장점이다.

### 2.4.3 ⚠ 정정 — 커맨드 슬롯 용량은 `slots[]` 가 아니라 마스크가 정한다

01 §4 표는 커맨드 슬롯을 **`project` 3칸 / `control` 0칸 / `manual` 10칸 전부 자유** 라고 정했다.
그런데 §2.4 의 `kFrameManual` 에는 `Cmd(0)`·`Cmd(1)`·`Cmd(2)` **3개뿐**이다.
`CmdSlotCount()` 로 `slots[]` 를 세면 `manual` 이 3 이 나와 **01 과 어긋난다**.

원인은 두 개념을 하나로 본 것이다:

| | 무엇인가 | `manual` |
|---|---|---|
| `slots[i]` | 사용자가 아무것도 넣지 않았을 때 그 tick 의 **기본 동작** | `EcuRd` 7칸 + `Cmd` 3칸 |
| `user_slot_mask` bit i | 사용자가 그 tick 을 **자기 명령으로 덮어쓸 수 있다** | 10칸 전부 |

01 §5.1 의 "`manual` 은 10칸 전부 사용자 교체 가능 (ECU 정기 READ 슬롯도 사용자가 비울 수 있다)"
가 바로 두 번째 줄이다. 즉 **동시 등록 가능한 명령 수 = `popcount(user_slot_mask)`** 이고,
`slots[]` 의 `Cmd(i)` 는 "기본값이 비어 있는 전용 자리" 를 표시할 뿐이다.

```cpp
constexpr uint8_t CmdCapacity() const { return popcount(user_slot_mask); }
```

| 모드 | `user_slot_mask` | 용량 | `slots[]` 의 `Cmd` | 나머지 마스크 자리 |
|---|---|---|---|---|
| `project` | `0b0101010000` | **3** | tick 4·6·8 (전용) | — (일치) |
| `manual` | `0b1111111111` | **10** | tick 4·6·8 (전용) | tick 0~3·5·7·9 = `EcuRd`/`DpcRd`/`PcuRd` 를 **덮어쓸 수 있는 자리** |
| `control` | `0` | **0** | 없음 | 없음 (§2.4.1 대체 불가) |

디스패치는 이렇게 읽는다 — 명령이 있으면 명령이, 없으면 기본 동작이 나간다:

```
if (frame.user_slot_mask & (1u << tick_in_frame)) and command.HasSlot(tick_in_frame):
    ExecuteCommand(...)          # 사용자 명령이 이 tick 을 차지
else:
    ExecuteSlot(frame.slots[tick_in_frame])   # 기본 동작 (Cmd 자리면 EMPTY = 아무것도 안 함)
```

> **더 줄일 수 있는 여지 (미채택, 06 이후 판단)**: 이 모델에서 `SlotId::COMMAND` 는
> "기본값이 `EMPTY` 인 마스크 자리" 와 완전히 같다. `Cmd(i)` 를 `kSlotEmpty` 로 바꾸고
> `SlotId::COMMAND` 와 `cmd_index` 를 통째로 없애도 동작이 같다 (`cmd_index` = 마스크에서
> 몇 번째 set bit 인가로 파생). 열거값이 하나 줄지만, 표를 볼 때 "여기는 커맨드 자리" 라는
> 의도가 마스크로만 표현되어 덜 읽힌다 — 지금은 **명시성을 택해 `COMMAND` 를 유지**한다.

### 2.5 wire 예산을 컴파일 타임에 검증한다

02 §5.5 의 규칙을 `constexpr` 로 구현하면 **빌드가 곧 검증**이 된다. 슬롯이 자기 구간을
갖게 되었으므로(§2.2.2) 검증도 **프리셋별 수동 나열이 아니라 슬롯별 자동**이 된다.

```cpp
constexpr double kByteTimeUs  = 10.85;     // 921600bps, 10bit/byte
constexpr double kTickUs      = 5000.0;
constexpr double kBudgetRatio = 0.5;       // 와이어가 tick 의 절반을 넘지 않는다
constexpr uint16_t kFrameBytes      = 8;   // 헤더5 + Inst1 + CRC2 (rd_comm.hpp:10,11)
constexpr uint16_t kAutoModeMaxWrite = 52; // 6종 write 범위의 최대 = DIRECT(128:52)

// ⚠ payload 구성이 INST 마다 다르다 (2026-07-27 골든 바이트로 실측 확인 — §2.5.2)
constexpr uint16_t ReqBytes(const SlotDef& s) {
    const uint16_t w    = s.write.MaxLen();
    const uint16_t nseg = s.read.SegCount();
    uint16_t payload = 0;
    switch (s.inst) {
    case SlotInst::READ:  payload = 4 * nseg;                 break;  // [addr|len]×n 만
    case SlotInst::WRITE: payload = 2 + w;                    break;  // 시작주소 + 데이터
    case SlotInst::RW:    payload = 1 + 4 * nseg + 2 + w;     break;  // **세그 개수 바이트는 RW 만**
    case SlotInst::NONE:  payload = 0;                        break;
    }
    return payload + kFrameBytes;
}
constexpr uint16_t RespBytes(const SlotDef& s) { return s.read.RespPayload() + kFrameBytes; }

constexpr bool FitsBudget(const SlotDef& s) {
    return (ReqBytes(s) + RespBytes(s)) * kByteTimeUs <= kTickUs * kBudgetRatio;
}

// 프레임 하나를 통째로 — 조합 유효성(§2.2.4) + 예산을 한 번에
constexpr bool FrameOk(const FrameDef& f) {
    for (uint8_t i = 0; i < f.frame_ticks; i++) {
        const SlotDef& s = f.slots[i];
        if (!IsValidSlot(s)) return false;
        if (s.id == SlotId::EMPTY || s.id == SlotId::COMMAND) continue;  // 정적 검사 불가
        if (!FitsBudget(s)) return false;
    }
    return true;
}

static_assert(FrameOk(kFrameProject), "project 프레임 — 조합 무효 또는 wire 예산 초과");
static_assert(FrameOk(kFrameControl), "control 프레임 — 조합 무효 또는 wire 예산 초과");
static_assert(FrameOk(kFrameManual),  "manual 프레임 — 조합 무효 또는 wire 예산 초과");
```

**`MaxLen()` 이 최악값을 쓰는 이유**: `WriteSrc::AUTO_MODE` 슬롯의 실제 write 길이는 런타임의
`auto_mode` 가 정한다(∅/8/16/16/16/52 — 01 §3). 컴파일 타임에는 알 수 없으므로 **가장 큰 52
(DIRECT)** 로 검증한다. 이러면 어떤 `auto_mode` 로 기동해도 예산이 보장된다. 초안처럼
`static_assert(FitsBudget(preset, 16))` / `(preset, 52)` 를 사람이 나열하면 **조합 하나를 빠뜨려도
빌드가 통과**하는데, 이제 그럴 수 없다.

**검산** (예산 한계 = 2.5ms = **230B**):

| 슬롯 | 세그 | 요청 | 응답 | 합 | 시간 | |
|---|---|---|---|---|---|---|
| `project` READ (manual) | 3 | 20B | 90B | 110B | 1.19ms | ✓ |
| `project` RW (auto_mode 최악 52) | 3 | 75B | 90B | 165B | 1.79ms | ✓ |
| `control_test` RW (CURRENT 16) | 3 | 39B | 72B | 111B | 1.20ms | ✓ |
| `control_test` RW (최악 52) | 3 | 75B | 72B | 147B | 1.60ms | ✓ |
| `control` READ | 2 | 16B | 107B | 123B | 1.33ms | ✓ |
| `control` RW (CURRENT 16) | 2 | 35B | 107B | 142B | 1.54ms | ✓ |
| **`control` RW (최악 52 = DIRECT)** | 2 | 71B | 107B | **178B** | **1.93ms** | ✓ 여유 52B |
| `diag` READ | 3 | 20B | 74B | 94B | 1.02ms | ✓ |

### 2.5.2 골든 바이트로 검산한 결과 (2026-07-27)

06 §2.2 의 골든 기준선(`test/golden/ecu_wire.txt`)을 만들면서 **실제 `RdMap::Encode` 출력**과
위 공식을 대조했다. 두 가지가 확인되고 하나가 정정됐다.

| 케이스 | 골든 실측 payload | 공식 |
|---|---|---|
| `rw_current_164_16` (6세그) | **43B** | `1 + 4×6 + 2 + 16` = 43 ✓ |
| `rw_direct_128_52` (6세그) | **79B** | `1 + 4×6 + 2 + 52` = 79 ✓ |
| `read_traction_preset` (5세그) | **20B** | `4×5` = 20 ✓ |
| `write_motor_mask` | **3B** | `2 + 1` = 3 ✓ |

43·79 는 01 §9.2 의 P8 분석표가 적은 payload 와 **정확히 같다** — 그 분석이 실제 인코더 동작과
일치함을 처음으로 기계가 확인한 것이다.

> **정정**: 초안의 `ReqBytes` 는 세그 개수 바이트(`1`)를 **모든 INST 에** 더했다. 실제로는
> **RW 에만** 있다 (READ 는 Length 필드로 세그 수를 역산할 수 있고, WRITE 는 세그가 없다).
> 그래서 READ 행이 1B 씩 과대 계산되어 있었다. 위 표는 정정본이다.
> **결론은 바뀌지 않는다** — 과대 계산이었으므로 예산 판정은 보수적이었다.

> **구 검산표 정정 (2026-07-27)**: 이전 표의 `project` 요청 33B·`control_test` 요청 43B 는
> 잘못된 값이었다. 그 43 은 01 §9.2(P8 분석)의 **6세그먼트 프리셋 payload**(`1+4×6+2+16 = 43`)
> 인데, 3세그 프리셋의 **wire 바이트**로 옮겨 적혔다 — payload 와 wire, 6세그와 3세그가 섞였다.
> 위 표는 `ReqBytes`/`RespBytes` 공식 하나로 전부 재계산한 값이다.
> **결론은 바뀌지 않는다** (모든 조합이 예산 내, 최악 케이스도 77%).

> **STM 버퍼 확장(01 §9.2)과의 관계**: 버퍼 256 은 **상한**이고, `static_assert` 는 **200Hz 에서
> 실제로 쓸 크기**를 제한한다. 둘은 다른 층의 방어선이며 둘 다 필요하다.

### 2.5.1 정적 검사가 닿지 않는 곳 — 커맨드 슬롯

`FrameOk` 은 `COMMAND` 슬롯을 건너뛴다. 그 슬롯의 `(id, inst)` 와 구간은 사용자가 명령을
넣는 순간 정해지기 때문이다(§2.2.2). **그래서 커맨드 수락 경로에 같은 검사를 런타임으로 둔다**:

```
CommandSet 수락 시:
    slot = BuildSlotFromCommand(cmd)      // (id, inst, read, write) 확정
    if (!IsValidSlot(slot))  → reject "구간과 명령이 맞지 않음"
    if (!FitsBudget(slot))   → reject "wire 예산 초과 — 세그를 줄이거나 나눠서 요청"
```

`IsValidSlot`/`FitsBudget` 이 `constexpr` 이면서 **일반 함수로도 호출 가능**하다는 점이 여기서
값을 한다 — 컴파일 타임 검사와 런타임 검사가 **같은 코드**라 규칙이 갈라질 수 없다.

---

## 3. C2 — `duration` · 재시도 · blackout

### 3.1 현행 규칙

| 항목 | 현행 |
|---|---|
| `duration=1` (once) | `RET_OK` 까지 재시도, **2초 timeout** 후 포기 |
| `duration=0` (forever) | 슬롯 RESET 전까지 지속. 실패는 `err_streak` 로 누적, 25회마다 WARN |
| `duration=2~100` | N초 후 자동 해제 |
| REBOOT 성공 | 대상 보드 **3초 blackout** + `is_connected=false` |
| blackout 중 | `GetSlotTask` 가 `false` 반환 (그 차례 skip) |

### 3.2 ⚠ 발견한 문제 — blackout 이 `once` 를 죽인다

`rd_command.cpp:GetSlotTask` 의 순서:

```cpp
if (blackout) return false;                       // ①
auto elapsed = now() - s.start_time;
if (duration == ONCE && elapsed > 2s) { 포기; }    // ②
```

**blackout(3초) > once timeout(2초)** 이다. REBOOT 직후 같은 보드에 `once` 명령을 넣으면:
- 0~3초: ① 에서 계속 skip — **한 번도 시도되지 않는다**
- 3초 시점: blackout 해제 → ② 에서 `elapsed=3s > 2s` → **즉시 포기**

즉 **실행 기회를 한 번도 얻지 못하고 timeout 으로 죽는다.** 사용자에게는 "명령이 조용히 사라진"
것으로 보인다.

### 3.3 수정 — 만료 기준을 "경과 시간"에서 "시도 횟수"로

```
once   : kOnceMaxAttempts = 40 회 **시도** 후 포기
         ※ blackout·슬롯 미도래로 skip 된 것은 시도로 세지 않는다
forever: 변경 없음 (RESET 까지)
N초    : 변경 없음 (시간 기준이 사용자 의도 — IClock 사용)
blackout: 변경 없음 3초 (시간 기준, IClock)
```

**왜 시도 횟수인가**:
1. §3.2 버그가 정의상 사라진다 — 시도하지 않았으면 만료도 없다
2. **주기 독립**이다. 커맨드 슬롯 주기는 모드(project 20Hz / manual 20Hz)와 슬롯 점유 상황
   (auto 시퀀스 차용)에 따라 달라지는데, 시간 기준이면 재시도 횟수가 그때그때 바뀐다
3. **결정론적** — 테스트에서 40회를 정확히 재현할 수 있다 (02 §7.2 `IClock` 과 함께)

`40회` 의 근거: project 20Hz 기준 2초 = 40회로 현행 체감 시간과 같다.

### 3.4 tick 단위 처리 규칙

| 상황 | 처리 |
|---|---|
| 슬롯 차례 도래 + 활성 | `GetSlotTask` → `TaskConfig_t` 생성 → 실행 → `ReportResult` |
| blackout 중 | **skip. 시도 카운트 증가 없음.** 로그 없음(스팸 방지) |
| 슬롯 비활성 | skip |
| `once` + `RET_OK` | 즉시 슬롯 해제. READ 면 결과를 로그로 덤프 |
| `once` + 실패 | 시도 카운트++. 40회 도달 시 포기 + ERROR 로그 |
| `forever`/N초 + 실패 | `err_streak`++. 25회마다 WARN (5Hz 기준 ≈5초) |
| N초 만료 | 슬롯 해제 + INFO |
| REBOOT + `RET_OK` | 3초 blackout 설정 + `is_connected=false` + 슬롯 해제 |

### 3.5 우선순위·차용 규칙 (유지)

01 §6.3-D 그대로:
1. `slot=255`(자동) 요청 → **빈 슬롯 중 번호가 가장 작은 것**에 배치
2. 전부 차 있으면 → **번호가 가장 큰 슬롯(최하위 우선순위)을 일시정지**하고 자리 차용
3. 처리 완료 후 원래 슬롯 **복귀**

`rd_sequence`(jeongae, A3 분리) 가 이 경로의 유일한 자동 사용자다.

### 3.6 `safe_stop` 게이트를 어디서 거는가

01 §6.2 분류를 **슬롯 SET 요청 시점**에서 판정한다 (실행 시점이 아니다).

```
CommandSet 수신
  → cmd 분류 판정
      READ 계열        → 게이트 없음, 즉시 슬롯 배치
      감속 방향 WRITE   → 게이트 없음, 즉시 배치 (soft_estop=0, 명령값 0, RESET)
      설정 변경 WRITE   → safe_stop 검사
                           통과 → 배치
                           위반 → 즉시 거부 + 위반 조건 명시
```

**실행 시점이 아니라 요청 시점에 거는 이유**: 사용자가 즉시 응답을 받아야 한다. 슬롯에 넣어 두고
실행 차례에 거부하면 "왜 안 되는지"가 로그에만 남는다.

단 `forever` 설정 변경 WRITE 는 매 실행마다 재검사한다 — 배치 후 모터가 돌기 시작할 수 있으므로.

---

## 4. C3 — `GET_STATUS` JSON 스키마

AI 자동화의 파싱 계약 (testbed_spec §5.1 TODO ⓐ 해소). **텍스트 정규식 파싱을 시키지 않는다.**

```json
{
  "bridge_mode":      "control",
  "control_state":    "IDLE",
  "write_source":     "none",
  "ecu_mode":         "auto",
  "ecu_sys_state":    2,
  "auto_mode":        "current",
  "write_span":       "164:16",
  "read_preset":      "control_test",
  "active_motors":    [2, 3],
  "motor_mask":       6,
  "ctr_mode":         ["estop", "current", "current", "estop"],

  "goal_id":          0,
  "profile_time":     0.0,

  "safe_stop":        true,
  "safe_stop_detail": null,

  "stamp_valid":      true,
  "stamp_quality_ms": 0.42,
  "drift_ppm":        -19583,
  "rtt_ms":           1.7,
  "rw_err":           0,
  "drop_cnt":         0,

  "lock_reason":      null,
  "slots": [
    {"index": 0, "active": false},
    {"index": 1, "active": true, "cmd": "READ_SYS", "target": "ecu", "duration": 0, "attempts": 128}
  ]
}
```

규칙:
- **모든 키가 항상 존재한다.** 값이 없으면 `null` — 키 유무로 분기하게 만들지 않는다
- enum 은 **문자열**(사람·AI 가독). 정수 원본이 필요하면 별도 키(`ecu_sys_state`)
- `safe_stop_detail` 은 `safe_stop=false` 일 때만 문자열
  (예: `"M2 fb_velocity=42.1 RPM (한계 5.0)"`)
- `stamp_valid=false` 면 `stamp_quality_ms`·`drift_ppm` 은 `null`

---

## 5. C4 — `control_cli` 명령 체계

### 5.1 원칙

- **원샷** (REPL 아님). 한 번 실행 → 종료코드 반환이 계약
- `command_cli`(C++ REPL) 삭제, 기능 흡수 (Q4)
- **주소를 노출하지 않는다.** 의미 단위 명령만. raw 는 `manual` 전용 (B6)
- 단어형 인자 채택 (TODO ⓑ 해소). 정수 코드도 병행 허용

### 5.2 명령 목록

```bash
# ── 상태 ──────────────────────────────────────────────
control_cli status [--json]

# ── 설정 (ControlConfig, control 모드) ────────────────
control_cli config motors 2 3                # SET_ACTIVE_MOTORS
control_cli config ctr_mode 2 3 current      # SET_CTR_MODE — <모터...> <mode>
control_cli config auto_mode current|direct|velocity|position
                                             # SET_AUTO_MODE — kinematic(0)·control(3) 은 거부
                                             #   velocity(4)/position(5) 은 01 §3.2 신규
control_cli config mode auto|manual          # SET_MODE
control_cli config preset control|control_test|diag|project
                                             # SET_READ_PRESET (op 6) — **IDLE 전용** (§2.4.2)
                                             #   control 처럼 슬롯이 없는 모드에서 정기 READ 를
                                             #   바꾸는 유일한 경로
control_cli rearm                            # LOCKED -> IDLE
control_cli arm on|off                       # SET_STREAM_ARM (op 7) — 웹 run/stop 과 같은 것
                                             #   on 은 IDLE 에서만. STREAM 진입 게이트 (01 §6.1.3)
control_cli origin 2 3                       # SET_ORIGIN 펄스 (01 §6.3 C-1)
                                             #   IDLE + safe_stop + auto_mode=DIRECT 필요
                                             #   1 tick 만 나가고 원복. 결과는 fb_position 으로 확인

# ── 프로파일 (RunProfile action) ──────────────────────
control_cli run <profile.yaml> [--record] [--name NAME] [--bag-dir DIR] [--timeout SEC]
control_cli abort

# ── 커맨드 슬롯 (CommandSet, 전 모드) — command_cli 흡수분 ──
control_cli cmd read sys|motor|sensor|diag|all
                 [--target ecu|dpc|pcu] [--slot N] [--duration once|forever|N]
control_cli cmd set motor_mask 2 3
control_cli cmd set mode auto|manual
control_cli cmd set auto_mode current|direct|kinematic|velocity|position
                                             # ⚠ none 은 없다 — ECU addr 188 에 넣는 값이
                                             #   아니라 브리지 write 범위 ∅ 를 뜻하는
                                             #   기동 파라미터다 (01 §3.1)
control_cli cmd set soft_estop on|off
control_cli cmd set use_lpf on|off
control_cli cmd set origin 2 3               # manual 의 슬롯 경로 (control 은 위 `origin`)
control_cli cmd reboot ecu|dpc|pcu
control_cli cmd reset --slot N               # 슬롯 비우기
control_cli cmd list                         # 슬롯 현황

# --slot N 의 범위는 **모드 의존** = FrameDef::CmdCapacity() (§2.4.3)
#   project 3 (N=0~2) / manual 10 (N=0~9) / control 0 — control 은 cmd 계열 전부 거부
#   범위 밖이면 exit 1 + "이 모드의 슬롯은 0~<cap-1> (control 은 슬롯 없음)"

# ── raw (manual 전용) ─────────────────────────────────
control_cli raw read  ecu 88 40
control_cli raw write ecu 190 01
```

### 5.3 종료 코드 (유지)

| 코드 | 의미 |
|---|---|
| 0 | 성공 (run 완주 / 명령 수락) |
| 1 | 사용법·파일 오류 |
| 2 | 거부 — goal reject / service `ok=false` / `safe_stop` 위반 |
| 3 | 중단 — Ctrl+C / abort / 재생 중 LOCKED 전이 |
| 4 | 브리지 미실행 (5s 탐색 실패) |
| 5 | 타임아웃 (`--timeout`, 기본 = 프로파일 길이 + 30s) |

### 5.4 `run` 동작 순서 (유지 — `--record` 시)

1. `status` 확인 — IDLE 아니면 즉시 실패 (exit 2)
2. **bag 녹화 시작** → 토픽 구독 확인까지 대기(≤2s)
   — goal 보다 먼저 떠야 프로파일 첫 hold(분석 tare 창)가 온전히 잡힌다
3. YAML 읽어 goal 전송 → accept 확인
4. 5Hz 피드백으로 진행률 출력 (`--json` 시 억제)
5. result 수신 → **+1s 여유 후 bag 종료**
6. 폴더에 `profile.yaml` 사본 + `result.json` 기록

**녹화 토픽 (개명 반영)**: `/carrier/control/feedback` — 1개.
`comm_diag` 는 기본 off 이므로 녹화 대상이 아니다 (03 §4.2). `--diag` 옵션 시 2개.

### 5.5 Ctrl+C 안전 규칙 (필수 — 유지)

`run` 중 SIGINT: ① action cancel 전송 → ② result 수신 대기(≤3s) → ③ bag 정상 종료 → ④ exit 3.
**CLI 만 죽으면 브리지의 goal 은 계속 재생되어 모터에 전류가 계속 나간다** — 이 경로가 안전 구멍이다.
2회 연속 SIGINT 시 즉시 종료하되 `"goal 이 계속 돌 수 있음 — control_cli abort 실행"` 경고 출력.

---

## 6. C5 — DPC / PCU 표준화 제안  📋 **TODO — 미합의**

> **이 절은 아직 확정이 아니다 (2026-07-27).** DPC/PCU 펌웨어 담당과 협의가 선행되어야 하며,
> 그 전까지는 **제안 상태**로 둔다. 아래 §6.1~§6.3 은 협의에 들어갈 초안이다.
>
> **협의 전까지의 안전한 기본값**: `enable_dpc = enable_pcu = false`.
> 그러면 두 슬롯은 `kSlotEmpty` 로 치환되어 커맨드에 양보되므로(§2.4), 나머지 설계는
> 이 절의 결론과 무관하게 진행할 수 있다.
>
> **막혀 있는 것 2가지**:
> 1. `kPresetDpc` / `kPresetPcu` 의 실제 세그먼트 (§2.3 이 자리만 비어 있다)
> 2. `rd_sequence`(jeongae 전개, A3) 의 TODO 단계 5개 — DPC 레지스터 확정 대기
>
> 합의되면 §2.3 의 두 프리셋을 채우고 §2.5 의 `static_assert` 가 예산을 자동 검증한다.
> **거부되면** `rd_telemetry` 에 보드별 매핑 함수를 두는 대안으로 간다 (§6.2 말미).

### 6.1 현재 상태

레지스터 맵 미확정. `enable_dpc` / `enable_pcu` 기본 `false` (보드 미장착 시 timeout 폭주 방지).
`rd_register_dpc.hpp` / `rd_register_pcu.hpp` 는 골격만 있다.

### 6.2 제안 — SYS 영역 레이아웃을 ECU 와 통일

```c
/* 모든 보드 공통 — addr 16~32 (17 bytes) */
typedef struct __attribute__((packed)) {
    /* addr 16 */ uint8_t  degraded_cnt[8];
    /* addr 24 */ uint8_t  hw_error;
    /* addr 25 */ uint8_t  hw_fatal;
    /* addr 26 */ uint8_t  hw_reset;
    /* addr 27 */ uint8_t  sys_state;
    /* addr 28 */ uint32_t realtime_tick;
    /* addr 32 */ uint8_t  rs485_proc_delta;
} DATA_SYSTEM_t;
```

이렇게 하면:
- `NodeStatus`(03 §5.3) 매핑이 **보드별 분기 없이** 자동으로 된다
- 읽기 세그 `{16,17}` 이 세 보드 공통 — 슬롯 테이블이 단순해진다
- 시간 동기(B1)를 DPC/PCU 로 **확장할 수 있다** (`realtime_tick` + `proc_delta` 가 같은 자리)
- `lc[8]`/`hs[8]` 인덱스 규약(03 §3.1)은 보드마다 다른 서브시스템을 가리키지만, **인덱스 = 통신 채널**
  이라는 정의는 유지된다 (보드별 채널 매핑 표를 문서화)

> **협의 필요**: DPC/PCU 펌웨어 담당과 합의가 선행되어야 한다. 합의 전까지 `enable_*=false` 유지.
> 이 제안이 거부되면 `rd_telemetry` 에 보드별 매핑 함수를 두는 것으로 대체 가능하다 (비용은 그만큼 증가).

### 6.3 확정 후 채워야 할 것

| 보드 | 필요한 최소 데이터 |
|---|---|
| DPC | 전개 FSM state, 공벽 1/2번칸 위치, 전개판 상태, 카메라 위치 도달 플래그 |
| PCU | 배터리 SOC(→ `/carrier_battery`), SOH, 전압·전류, 온도 |

`rd_sequence`(jeongae 전개, A3)의 TODO 단계 5개가 DPC 레지스터 확정에 막혀 있다.

---

## 7. tick 루프 의사코드 (전체 통합)

02 의 계층 분리와 01 의 규칙을 하나로 합친 결과. 06 구현 시 이 순서를 따른다.

```
RdSchedule::RunLoop()                       // 스케줄 스레드, SCHED_FIFO 80, core 11
  frame = kFrameTable[config.bridge_mode]        # 모드당 프레임 1개 (§2.4.2 (가) 확정)

  loop every 5ms (sleep_until, 절대 시각 기준):
      tick_in_frame = tick_count % frame.frame_ticks
      slot          = frame.slots[tick_in_frame]

      # ── ① 1회성 READ / out-of-span WRITE 가 이번 tick 을 요구하는가 (01 §6.3) ──
      if oos.TakeStep(&step):
          if step.needs_full_tick and control.State() == RUNNING and
             control.WriteSource() == PROFILE:
              oos.Reject("RUNNING+PROFILE 중 tick 대체 불가")
          else:
              ExecuteOosStep(step)          # write 유지형이면 read 세그만 교체
              sink->OnTransaction(...)      # 필드 불일치 → drop 표시
              continue

      # ── ② 슬롯 디스패치 — 먼저 id(누구에게), 그 다음 inst(무엇을) ──
      switch slot.id:
        EMPTY:      continue
        DPC / PCU:  if !enable → continue
                    ExecuteTask(slot.id, slot.inst, slot.read, slot.write)
                    continue                    # 구간이 슬롯 안에 있다 (§2.2.2)
        COMMAND:    # 늦은 바인딩: (id, inst) 가 여기서 확정된다 (§2.2.2)
                    if !command.GetSlotTask(slot.cmd_index, &task) → continue
                    ret = ExecuteTask(task)
                    command.ReportResult(slot.cmd_index, ret)
                    continue
        ECU:
          if slot.inst == READ:
              ExecuteTask(ECU, READ, slot.read)
          else:                                 # slot.inst == RW
            # ── ③ write 소스 확정 (L2 정책) ──
            w = control.SelectWrite()     # IDLE=안전값 / RUNNING=source / LOCKED=래치
                                          # project IDLE 은 write 중단 (01 §6.1.1)
            if w.skip_write:
                ExecuteTask(ECU, READ, slot.read)       # inst 가 RW→READ 로 강등된다
            else:
                span = kWriteSpan[shadow.auto_mode]     # 6개 중 선택 (01 §3.3)
                ApplyToShadow(w, span)                  # 범위 밖 shadow 는 쓰지 않는다 (S2)
                BackupSettingBytes(span)                # 설정 레지스터만 (01 §7.4)
                ret = ExecuteTask(ECU, RW, slot.read, span)
                # slot.write.src == AUTO_MODE 이므로 span 은 런타임 셀렉터가 준다

      # ── ④ 결과 처리 ──
      if rw_err.write != NONE:
          RollbackSettingBytes()            # 설정만. 스트리밍은 streak 만 (01 §7.4)
          control.NoteWriteErrStreak(map.RwWriteErrStreak())

      # ── ⑤ 통지 (논블로킹) ──
      txn = BuildTxnResult(tick, ret, rw_err, timing, control, shadow)
      sink->OnTransaction(txn)              # 큐 push. 가득 차면 드롭 + drop_cnt++

      tick_count++
```

**순서의 근거**

| 단계 | 왜 이 자리인가 |
|---|---|
| ① 이 최우선 | tick 을 통째로 쓰는 요청이므로 다른 판단보다 먼저 결론이 나야 한다 |
| ③ 이 실행 직전 | write 소스는 가장 최신 상태를 반영해야 한다. 미리 정하면 그 사이 FSM 이 바뀔 수 있다 |
| 백업이 실행 직전 | 롤백 대상은 "이번에 보낸 값" 이다 |
| ⑤ 가 마지막, 논블로킹 | DDS 가 200Hz 루프를 막지 않는다 (02 §6.3) |

---

## 8. 05 에서 결정할 것

| # | 질문 |
|---|------|
| D1 | 프로파일 YAML 스키마 확장 — `mode:` 키(velocity/position 프로파일, `auto_mode` 확장 연동) |
| D2 | 세그먼트 9종의 파라미터 검증 규칙 정밀화 (특히 `custom` 의 rate·길이 상한) |
| D3 | 웹 그래프 드로잉 → `custom` 샘플 변환 규격 (샘플 수 상한, 보간 방식) |
| D4 | `result.json` 스키마 갱신 — `irregular_tick_cnt`·`drop_cnt`·`node_params` 필드 추가 |
| D5 | 기록 폴더 규격 유지 여부 + 개명 반영 (`/carrier/control/feedback`) |

---

## 부록: 결정 요약 카드

```
슬롯 테이블 (C1)
  슬롯 = SlotId(EMPTY|ECU|DPC|PCU|COMMAND) × SlotInst(NONE|READ|WRITE|RW)
         + ReadSpan(읽을 세그) + WriteSpan(NONE|FIXED|AUTO_MODE)   ← 구간이 슬롯 안에 있다
         COMMAND 는 늦은 바인딩 표식 — (id,inst)·구간을 런타임에 커맨드가 정한다
  파생값은 저장 안 함: SegCount() RespPayload() CmdCapacity()  ← 손으로 세지 않는다
  ECU/DPC/PCU 는 READ/WRITE/RW 전부 개방. 검사는 "명령과 구간이 맞는가" 구조만 본다
         (펌웨어 지원 여부는 런타임 거부 — 진행 상황을 static_assert 에 넣지 않는다)
  project 10tick: [DpcRd, EcuRw, PcuRd, EcuRw, Cmd0, EcuRw, Cmd1, EcuRw, Cmd2, EcuRw]
                                                                  user_mask=0b0101010000
  control  1tick: [EcuRw(kPresetControl)]                          user_mask=0 = 대체 불가
  manual  10tick: project 배치 + ECU 는 EcuRd                       user_mask=0b1111111111
  user_slot_mask == 0  ⇒ 슬롯 대체 불가, 1 tick 대체(OOS)만. 반복은 프리셋 교체로
  커맨드 용량 = popcount(user_slot_mask)  (project 3 / manual 10 / control 0)
               slots[] 의 Cmd(i) 는 "기본값이 빈 전용 자리" 표시일 뿐 — 용량이 아니다
  검증 (빌드가 곧 검증) — 슬롯별 자동:
    static_assert(FrameOk(frame))  = IsValidSlot(구조) + FitsBudget(AUTO_MODE 는 최악 52)
    COMMAND 슬롯만 정적 검사 불가 → 명령 수락 시 같은 함수로 런타임 검사

duration (C2)
  once    = 40회 **시도** 후 포기 (시간 아님)  ← blackout 이 once 를 죽이던 버그 수정
  forever = RESET 까지, err_streak 25회마다 WARN
  N초     = 시간 기준 (IClock)
  blackout= 3초, skip 은 시도로 세지 않는다
  safe_stop 게이트는 **요청 시점**에 (forever 설정 변경만 매 실행 재검사)

GET_STATUS (C3)  모든 키 항상 존재, 값 없으면 null. enum 은 문자열
control_cli (C4) status/config/rearm/run/abort + cmd(의미 단위) + raw(manual 전용)
                 exit 0~5 유지. 녹화 토픽은 /carrier/control/feedback 1개
DPC/PCU (C5)     SYS 영역 16~32 레이아웃을 ECU 와 통일 제안 → NodeStatus 매핑 자동
                 ※ 펌웨어 담당 협의 선행

tick 루프 순서   ① OOS 선점 → ② 슬롯 디스패치 → ③ write 소스 확정+백업 → ④ ACK 처리/롤백
                 → ⑤ 논블로킹 통지
```
