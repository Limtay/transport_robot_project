# 02 — Layer 구조 · 의존 방향 · 스레드 소유권

> 선행: [00_overview.md](00_overview.md) · [01_modes.md](01_modes.md).
> 결정: 2026-07-26 사용자 확정 (A1~A5).
> 이 문서가 정하는 것: 계층 정의, 파일 배치, 의존 방향과 **그것을 강제하는 수단**, 계층 간
> 데이터 계약, 스레드·락 소유권.
> 이 문서가 정하지 않는 것: 메시지 필드(03), 슬롯 구현 상세(04), 마이그레이션 순서(06).

---

## 1. 확정 결정 (A1~A5)

| # | 결정 |
|---|------|
| A1 | 역방향 의존은 **성격별 3종 처방**으로 뒤집는다 — (a)설정=주입 / (b)통지=`ITelemetrySink` / (c)(d)정책=L2 직접 참조 |
| A2 | L3 를 **4개**로 분할: `rd_node` / `rd_telemetry` / `rd_control_api` / `rd_carrier_api`. L2 에 **2개 신설**: `rd_profile_player` / `rd_oos` |
| A3 | `rd_command` 를 **슬롯 관리**와 **jeongae 시퀀스**(`rd_sequence`)로 분리 |
| A4 | 프로파일 액션 스레드 → **타이머로 흡수**. **200Hz 발행을 별도 스레드로 분리** (DDS 블로킹 차단) |
| A5 | **안 2** — 로거 + 시계 추상화. L0/L1/L2 를 rclcpp 없이 단위 테스트 가능하게 |

---

## 2. 계층 정의

| 계층 | 이름 | 책임 | rclcpp | 상태 소유 |
|---|---|---|---|---|
| **L3** | ROS 인터페이스 | 외부 계약(topic/service/action)의 입출구. 메시지 ↔ 내부 타입 변환 | **○ (유일)** | ✗ (핸들만) |
| **L2** | 정책 | "무엇을 명령할 것인가" — FSM, 프로파일 재생, 슬롯, 시퀀스 | ✗ | ○ |
| **L1** | 스케줄 | "언제 무엇을 통신할 것인가" — tick 루프, 슬롯 테이블 실행 | ✗ | tick 카운터·통계 |
| **L0** | 전송 | "어떻게 바이트를 주고받을 것인가" — 패킷·시리얼·레지스터 매핑 | ✗ | shadow(`RobotState_t`) |

**의존 방향: L3 → L2 → L1 → L0 단방향.** 예외는 `ITelemetrySink` 하나뿐이며, 이것은 **L1 이 정의하고
L3 가 구현**하므로 의존 방향을 어기지 않는다 (의존성 역전).

---

## 3. A1 — 역방향 의존을 어떻게 뒤집는가

### 3.1 현재 상태

```
rd_schedule.hpp:11   #include "rd_bridge.hpp"        ← L1 이 L3 를 include
rd_schedule.hpp:41   std::shared_ptr<RdBridge> bridge_node_;
rd_schedule.cpp      bridge_node_-> ... 22 곳
```

### 3.2 22곳의 성격별 처방

| 성격 | 호출 | 개수 | 처방 |
|---|---|---|---|
| **(a) 설정 조회** | `IsControlMode` `IsTractionTestMode` `ActiveMotorMask` `ActiveMotorsValid` `AutoModeParam` `AutoModeValid` `AutoMode` `SetAutoMode` | 10 | **생성 시 주입** — 기동 고정값(Q2)이므로 물어볼 이유가 없다 |
| **(b) 결과 통지** | `PublishCommLatency` `PublishTestbedFeedback` `NoteRwErr` `NoteWriteErrTotal` `SetHardwareStatus` | 6 | **`ITelemetrySink` 인터페이스** — L1 이 정의, L3 가 구현 |
| **(c) 정책 위임** | `TickProfile` `PrepareControlCommand` `Testbed().MarkInitDone` `Testbed().NoteWriteErrStreak` `ShouldSkipCmdWrite` | 4 | **L2 직접 참조** — `L1→L2` 는 정상 방향. `rd_bridge` 는 전달만 하고 있었다 |
| **(d) 상태기계 협조** | `TakeOutOfSpanStep` `ReportOutOfSpanResult` | 2 | **`rd_oos` 로 L2 분리** → `L1→L2` 정상 방향 |

> **핵심**: 22곳 중 ROS 노드에 진짜로 남아야 하는 건 **(b) 6곳뿐**이고, 그 6곳도 통합하면
> **콜백 2개**(`OnTransaction` / `OnHardwareStatus`)로 줄어든다. 나머지 16곳은 `rd_bridge` 가
> L2 정책을 껴안고 있어서 생긴 인위적 의존이다.

### 3.3 `ITelemetrySink` — L1 이 정의하는 유일한 상향 인터페이스

```cpp
// sched/rd_telemetry_sink.hpp  (L1, rclcpp 없음)
namespace orin_bridge {

class ITelemetrySink {
public:
    virtual ~ITelemetrySink() = default;
    // 매 tick 트랜잭션 완료 직후 (200Hz). 반드시 논블로킹이어야 한다.
    virtual void OnTransaction(const TxnResult& r) = 0;
    // 연결 상태 변화 시에만 (에지 트리거)
    virtual void OnHardwareStatus(bool ok, const char* msg) = 0;
};

} // namespace orin_bridge
```

- 스케줄러는 `ITelemetrySink*` 만 안다. `RdBridge`/`RdTelemetry` 라는 이름을 **모른다**
- **논블로킹 계약**이 핵심이다. 구현체(`rd_telemetry`)는 큐에 넣고 즉시 반환한다 (§6.3)
- 테스트에서는 벡터에 쌓는 `FakeSink` 를 주입하면 스케줄러를 단독 검증할 수 있다

---

## 4. 파일 배치 — 디렉터리가 곧 계층

의존 방향을 **주석이나 관례가 아니라 빌드 시스템으로 강제**한다.

```
orin_firmware_bridge/
├── include/orin_firmware_bridge/
│   ├── rd_config.hpp              # BridgeConfig — 전 계층 공용 (POD)
│   ├── rd_logger.hpp              # ILogger — 전 계층 공용 (A5)
│   ├── rd_clock.hpp               # IClock  — 전 계층 공용 (A5)
│   │
│   ├── core/                      # ── L0 ── rclcpp 금지
│   │   ├── rd_uart.hpp
│   │   ├── rd_comm.hpp
│   │   ├── rd_map.hpp
│   │   ├── rd_clock_sync.hpp
│   │   ├── rd_common.hpp
│   │   └── rd_register_{ecu,dpc,pcu}.hpp
│   │
│   ├── sched/                     # ── L1 ── rclcpp 금지
│   │   ├── rd_schedule.hpp
│   │   ├── rd_slot_table.hpp      # 신규: 모드별 슬롯·읽기 프리셋 (데이터)
│   │   ├── rd_telemetry_sink.hpp  # 신규: ITelemetrySink + TxnResult
│   │   └── rd_txn_queue.hpp       # 신규: 락프리 SPSC 큐
│   │
│   ├── policy/                    # ── L2 ── rclcpp 금지
│   │   ├── rd_control.hpp         # 구 rd_testbed (개명)
│   │   ├── rd_profile.hpp         # 파싱·검증·사전 샘플링 (기존)
│   │   ├── rd_profile_player.hpp  # 신규: 재생 상태 (rd_bridge 에서 분리)
│   │   ├── rd_oos.hpp             # 신규: out-of-span 큐 (rd_bridge 에서 분리)
│   │   ├── rd_command.hpp         # 축소: 슬롯 관리만
│   │   └── rd_sequence.hpp        # 신규: jeongae 전개 FSM (A3)
│   │
│   └── ros/                       # ── L3 ── rclcpp 유일 사용처
│       ├── rd_node.hpp            # 셸: 파라미터·조립·스레드
│       ├── rd_telemetry.hpp       # 발행 전담 (ITelemetrySink 구현)
│       ├── rd_control_api.hpp     # run_profile action + control/config service
│       └── rd_carrier_api.hpp     # cmd_vel/jeongae sub + command_set service
├── src/  (동일 구조)
└── test/
    ├── unit/                      # 노드 없이 도는 유닛 테스트 (신규)
    └── integration/               # 기존 픽스처 기반
```

### 4.1 CMake 로 계층을 못박는다

```cmake
# L0+L1+L2 — rclcpp 를 링크하지 않는다
add_library(rd_core_lib
  src/core/... src/sched/... src/policy/...)
target_include_directories(rd_core_lib PUBLIC include)
# ament_target_dependencies(rd_core_lib rclcpp)  ← 절대 추가하지 않는다

# L3 — rclcpp 는 여기서만
add_library(rd_ros_lib src/ros/...)
ament_target_dependencies(rd_ros_lib rclcpp rclcpp_action sensor_msgs ...)
target_link_libraries(rd_ros_lib rd_core_lib)
```

> **이것이 재설계의 실질적 안전장치다.** 누군가 `policy/` 에 `RCLCPP_INFO` 를 쓰면 **링크 에러로
> 즉시 막힌다.** 문서 규약은 잊히지만 빌드 실패는 잊히지 않는다.

CI 보조 검사 (선택):
```bash
! grep -rn "rclcpp" src/core src/sched src/policy include/orin_firmware_bridge/{core,sched,policy}
```

---

## 5. 각 단위의 역할과 데이터 계약

### 5.1 L3 — ROS 인터페이스 (4개)

| 단위 | 단일 목적 | 소유 | 의존 |
|---|---|---|---|
| `rd_node` | 파라미터 파싱·검증 → `BridgeConfig` 확정 → 객체 조립 → 스레드 기동 | `BridgeConfig`, 모든 객체의 수명 | 전부 |
| `rd_telemetry` | shadow·`TxnResult` → ROS 메시지 **변환·발행 전담** | 발행자, EMA 필터 상태, `frame_id` | `ITelemetrySink` 구현 |
| `rd_control_api` | control 계약 — `run_profile` action, `control/config` service | 서버 핸들만 (**상태 없음**) | `rd_control`, `rd_profile_player`, `rd_oos` |
| `rd_carrier_api` | project 계약 — `cmd_vel`/`jeongae` sub, `command_set` service | 구독자·서버 핸들만 (**상태 없음**) | `rd_command`, `rd_sequence`, `rd_control` |

**`*_api` 가 상태를 갖지 않는 것이 규칙이다.** 요청을 받아 L2 에 위임하고 응답만 만든다.
"얇은 계층"의 구체적 의미이며, 이것이 지켜지면 웹(`control_web`)이 붙을 때 같은 L2 를 재사용한다.

### 5.2 L2 — 정책 (6개)

| 단위 | 단일 목적 | 소유 상태 |
|---|---|---|
| `rd_control` | FSM(`INIT/IDLE/RUNNING/LOCKED`) + `write_source` + write 값 선택 | 상태, lock 사유, 현재 샘플 |
| `rd_profile` | YAML 파싱 → 검증 → 사전 샘플링 (**재생 안 함**) | 샘플 배열 |
| `rd_profile_player` | 재생 진행 — tick 인덱스, `goal_id` 채번, 진행률, 완료 판정 | 재생 상태 |
| `rd_oos` | out-of-span 요청 큐 + 결과 회신 | 요청 1건 + 결과 |
| `rd_command` | 커맨드 슬롯 N칸 관리 (SET/RESET/만료/blackout) | 슬롯 배열 |
| `rd_sequence` | jeongae 전개 시퀀스 FSM (A3 분리) | 시퀀스 상태, 차용 슬롯 |

**`rd_profile` 과 `rd_profile_player` 를 나누는 이유**: 파싱·검증은 **액션 콜백 스레드**에서
(수십 ms 걸려도 무방), 재생은 **200Hz 스케줄 스레드**에서 배열 인덱싱만 (수 µs). 두 스레드가
같은 클래스를 공유하면 락 범위 설계가 어려워진다. 데이터(샘플 배열)와 진행(인덱스)을 분리한다.

### 5.3 L1 — 스케줄 (2개)

| 단위 | 단일 목적 |
|---|---|
| `rd_schedule` | tick 루프. 슬롯 테이블을 **실행만** 한다 (해석·분기 없음) |
| `rd_slot_table` | 모드별 슬롯 배치·읽기 프리셋·write 범위를 **데이터로** 보유 + wire 예산 검증 (01 §5.5) |

`rd_slot_table` 신설이 원칙 R1("모드는 설정이지 코드 분기가 아니다")의 구현체다.
새 모드 = 테이블 항목 추가이며, `rd_schedule.cpp` 는 건드리지 않는다.

### 5.4 계층 간 데이터 계약 (4종)

```cpp
// rd_config.hpp — 기동 1회 확정, 이후 불변. 전 계층이 const 참조로 받는다
struct BridgeConfig {
    BridgeMode  bridge_mode;        // project | control | manual
    uint8_t     auto_mode;          // 01 §3
    uint8_t     active_motor_mask;
    const FrameDef* frame;          // 슬롯·읽기 구간은 프레임이 소유 (04 §2.2.2)
    float       cmd_current_max;
    double      cmd_vel_topic_timeout, cmd_vel_zero_timeout;
    bool        cmd_vel_guard_enable;
    std::string imu_frame_id, device_path;
};

// policy/rd_control.hpp — L2 → L1, 매 tick
struct ControlWrite_t {
    float    value[4];              // 단위는 auto_mode 가 정한다 (A / RPM / deg)
    uint8_t  ctr_mode[4];           // DIRECT 에서만 유효
    float    lin_vel, ang_vel;      // KINEMATIC 에서만 유효
    uint32_t goal_id;
    float    profile_time;
    uint16_t segment_index;
};

// sched/rd_telemetry_sink.hpp — L1 → L3, 매 tick (큐 경유)
struct TxnResult {
    uint64_t     tick;
    RD_RET       ret;
    uint8_t      rw_err;            // read_err | write_err<<4
    uint32_t     drop_cnt;          // 직전까지 누적 드롭 (§6.3)
    TxnTiming_t  timing;            // t_req/t_resp/wire_up/wire_down/proc_delta
    ControlState state;
    uint8_t      write_source;
    uint32_t     goal_id;
    float        profile_time;
    uint16_t     segment_index;
    uint8_t      motor_mask;
    uint8_t      reg[256];          // shadow 스냅샷 — **값 복사** (§6.3)
};

// policy/rd_command.hpp — L3 → L2, 비동기
struct SlotRequest { /* target, inst, addr, len, data, duration */ };
```

> **`TxnResult` 가 값 복사인 것이 락프리의 전제다.** 포인터를 넣으면 소비자가 꺼낼 때 shadow 는
> 이미 다음 tick 값으로 바뀌어 있다. 320B memcpy(≈50ns)는 5ms 예산에서 무시 가능하다.

> **한 트랜잭션 = 구조체 하나**: 현재는 `PublishCommLatency` + `PublishTestbedFeedback` +
> `NoteRwErr` + `NoteWriteErrTotal` 4번 호출로 흩어져 있다. 같은 트랜잭션의 결과이므로 묶는 것이
> 맞고, 01 §R5("1 트랜잭션 = 1 메시지")와도 일치한다. 03 의 `CommLatency` 흡수 논의와 직결된다.

---

## 6. 스레드 소유권 (A4)

### 6.1 Before / After

| | Before | After |
|---|---|---|
| 200Hz 스케줄 | main 스레드 (`main.cpp:120`) | **전용 스레드** (SCHED_FIFO 80, core 11) |
| ROS 콜백 | `spin_thread_` (`rd_bridge.cpp:196`) | 유지 (SCHED_OTHER) |
| 프로파일 액션 | **goal 당 detached 스레드** (`rd_bridge.cpp:1046`) | **삭제** → spin 스레드의 5Hz 타이머 |
| 200Hz 발행 | 스케줄 스레드가 직접 `publish()` | **발행 전용 스레드** (SCHED_OTHER) |
| `ThreadStart()` | 죽은 경로 (`rd_schedule.cpp:113`) | **삭제** |

### 6.2 최종 스레드 3개

| # | 스레드 | 스케줄링 | 하는 일 |
|---|---|---|---|
| 1 | **스케줄** | SCHED_FIFO 80, core 11 | tick 루프 → L2 조회 → L0 I/O → 큐 push. **ROS API 호출 금지** |
| 2 | **발행** | SCHED_OTHER | 큐 pop → `ControlFeedback` 발행 |
| 3 | **ROS spin** | SCHED_OTHER | 구독·서비스·액션 콜백 + 저주기 타이머 발행(100Hz/10Hz/5Hz) |

**스레드 1 의 불변식: rclcpp 함수를 한 줄도 호출하지 않는다.** L1 이 rclcpp 를 링크하지 않으므로
(§4.1) 이것은 **컴파일 타임에 보장**된다.

### 6.3 발행 분리 — 락프리 SPSC 큐

DDS 발행이 블록되면 200Hz tick 이 밀린다. 큐로 끊는다.

```
[스케줄 스레드]  TxnResult 작성 → queue.TryPush(r) → 즉시 반환 (블록 없음)
                          │
                    락프리 링버퍼 (SPSC, 256 슬롯 ≈ 1.28초 분량)
                          │
[발행 스레드]    queue.Pop(&r) → 메시지 변환 → publish()  (여기서 블록돼도 무방)
```

**설계 규칙**

| # | 규칙 | 근거 |
|---|---|---|
| Q1 | **SPSC** (생산자 1 = 스케줄, 소비자 1 = 발행). `std::atomic` head/tail + release/acquire | 락 불필요. mutex 도입 시 RT 스레드가 블록될 수 있다 |
| Q2 | **생산자는 절대 블록·대기하지 않는다.** 큐가 가득 차면 **새 샘플을 드롭**하고 `drop_cnt++` | RT 스레드 보호가 최우선. 오래된 것을 버리면 시계열 순서가 꼬인다 |
| Q3 | 드롭 발생 시 다음 성공 샘플의 `drop_cnt` 에 누적값을 실어 보낸다 | **분석이 구멍의 위치와 크기를 안다.** 조용한 손실을 만들지 않는다 |
| Q4 | 큐 크기 256 슬롯 (≈82KB) | 1.28초 분량. 이걸 넘기면 소비자가 1초 이상 멈춘 것 = 이미 비정상 |
| Q5 | 발행 스레드는 **CPU 코어를 고정하지 않는다** | core 11 은 RT 전용으로 비워 둔다 |

**저주기 발행은 큐를 쓰지 않는다**: `project`/`manual` 의 100Hz·10Hz 토픽은 지금처럼 spin 스레드
타이머가 shadow 를 `state_mutex` 로 읽어 발행한다. 큐는 **`control` 모드의 200Hz 피드백 전용**이다.

### 6.4 프로파일 액션 스레드 제거

```
Before: HandleProfileAccepted → std::thread{ExecuteProfile}.detach()
          그 스레드가 rclcpp::Rate(5.0) 루프를 돌며 피드백 발행·완료 감시

After:  HandleProfileAccepted → rd_profile_player 에 goal 등록 후 즉시 반환
          spin 스레드의 5Hz 타이머가 player 진행 상태를 읽어 피드백 발행·완료 처리
```

- **재생 자체는 원래도 스케줄 스레드**(`TickProfile`)가 했다. 이 스레드는 액션 프로토콜을 시중들 뿐이라
  별도 스레드일 이유가 없다
- `.detach()` 제거로 **종료 시 정리 경로가 생긴다** (현재는 `rclcpp::ok()` 에만 의존)
- goal 이 여러 개여도 동시 실행은 1개뿐이므로(01 §6.1 배타 규칙) 타이머 1개로 충분

### 6.5 락 소유권과 순서 규약

분해 후 mutex 는 각 L2 객체가 하나씩 소유한다.

| mutex | 소유자 | 경합 |
|---|---|---|
| `state_mutex` | `RobotState_t` (L0) | 스케줄 ↔ 저주기 타이머 |
| `mutex_` | `rd_control` | 스케줄 ↔ API 콜백 |
| `mutex_` | `rd_profile_player` | 스케줄 ↔ 타이머/액션 콜백 |
| `mutex_` + condvar | `rd_oos` | 스케줄 ↔ 서비스 콜백 |
| `mutex_` | `rd_command` | 스케줄 ↔ 서비스 콜백 |
| `mutex_` | `rd_sequence` | 스케줄 ↔ 구독 콜백 |
| (없음) | `rd_txn_queue` | **락프리** |

**규약 L1 — 한 번에 하나만 잡는다.**
두 mutex 를 동시에 잡아야 하는 코드는 **설계 오류로 간주**하고, 첫 락에서 값을 복사한 뒤 풀고
두 번째를 잡는 방식으로 푼다. 현행 `GetRosInputs()`(`rd_bridge.cpp:294~304`)가 이미 이 패턴이다 —
규약으로 승격한다. 락 순서 정의가 아예 불필요해지므로 데드락이 **구조적으로** 불가능하다.

**규약 L2 — 임계구역에서는 값 복사만.**
YAML 파싱·메시지 직렬화·발행 등 긴 작업은 락 밖에서 한다. 200Hz 스레드가 L2 mutex 를 잡으므로,
API 콜백이 락을 오래 쥐면 그대로 tick 지연이 된다.

**규약 L3 — RT 스레드에서 동적 할당 금지.**
`TxnResult` 는 큐에 미리 할당된 슬롯에 쓴다. `std::vector` 확장·`make_shared` 를 tick 안에서 하지 않는다.

---

## 7. A5 — rclcpp 없는 테스트

### 7.1 현재 측정치

| 파일 | rclcpp 사용 | 용도 |
|---|---|---|
| `rd_uart` `rd_comm` `rd_map` `rd_profile` `rd_clock_sync` | **0** | 이미 순수 |
| `rd_testbed` | 6 | **전부 로그** |
| `rd_command` | 21 | **전부 로그** |
| `rd_schedule` | 30 | 로그 + `bridge_node_` |

**L0/L2 는 이미 rclcpp 에서 거의 자유롭다.** 걸림돌은 로깅뿐이고, `rd_schedule` 은 A1 을 풀면
로깅만 남는다.

### 7.2 추상화 2개

```cpp
// rd_logger.hpp — 전 계층 공용
class ILogger {
public:
    virtual ~ILogger() = default;
    virtual void Log(LogLevel lv, const char* tag, const char* msg) = 0;
};
// L3 구현: RCLCPP_* 로 포워딩 / 테스트 구현: 벡터에 수집

// rd_clock.hpp — 전 계층 공용
class IClock {
public:
    virtual ~IClock() = default;
    virtual double NowSec() const = 0;      // 단조 시각 [s]
};
// L3 구현: node->now() / 테스트 구현: 수동 진행 가상 시계
```

**`IClock` 이 중요한 이유**: `cmd_vel` guard(0.1s/3.0s), 프로파일 스테일, `safe_stop`, blackout(3s),
INIT 재시도(0.2s×10) 가 전부 시간 의존이다. 지금은 **실제로 3초를 기다려야** 테스트된다.
가상 시계를 주입하면 즉시 검증되고 타이밍 흔들림이 사라진다.

### 7.3 테스트 구조

```
test/unit/          # rclcpp 없음. rd_core_lib 만 링크 — 빠르고 결정론적
  test_control_fsm.cpp        # 상태 전이 전수
  test_slot_table.cpp         # 프리셋 wire 예산 (01 §5.5)
  test_shadow_authority.cpp   # write ACK 커밋/롤백 (01 §7)
  test_txn_queue.cpp          # SPSC 경계·드롭 카운트
  test_profile_parser.cpp     # (이전)
test/integration/   # 기존 픽스처 — 노드 + 가짜 ECU
  test_config_service.cpp / test_run_profile_action.cpp / ...
```

현행 `rd_test_fixture.hpp` 는 FSM 하나를 보려고 `RdBridge` 노드 + 클라이언트 노드 2개를 띄운다.
분리 후에는 정책 검증이 유닛 테스트로 내려가고, 통합 테스트는 **계약(서비스 응답·액션 결과)** 만
확인한다. 이것이 R6("동작 불변, 구조만"을 `colcon test` 로 증명)의 실질적 수단이다.

---

## 8. 전체 조립도

```
                              main
                                │
                        ┌───────▼────────┐
                        │    rd_node     │  파라미터 → BridgeConfig → 조립 → 스레드 기동
                        └───┬────────┬───┘
             ┌──────────────┘        └──────────────┐
             ▼                                      ▼
  ┌──────────────────┐              ┌───────────────────────────────┐
  │  rd_telemetry    │              │  rd_control_api               │  L3
  │  (ITelemetrySink)│              │  rd_carrier_api               │
  └────────▲─────────┘              └───────────────┬───────────────┘
           │ 큐 pop (발행 스레드)                     │ 위임 (spin 스레드)
           │                                        ▼
   ┌───────┴────────┐              ┌──────────────────────────────────┐
   │  rd_txn_queue  │              │ rd_control · rd_profile(_player) │  L2
   │  (락프리 SPSC) │              │ rd_oos · rd_command · rd_sequence│
   └───────▲────────┘              └──────────────▲───────────────────┘
           │ push                                 │ 매 tick 조회
           │                                      │
   ┌───────┴──────────────────────────────────────┴───────┐
   │  rd_schedule  +  rd_slot_table                       │  L1  (스케줄 스레드)
   │  ITelemetrySink* 로만 위를 안다 — 구체 타입 모름       │
   └───────────────────────┬──────────────────────────────┘
                           ▼
   ┌──────────────────────────────────────────────────────┐
   │  rd_map → rd_comm → rd_uart   (+ rd_clock_sync)      │  L0
   │  shadow: RobotState_t                                │
   └──────────────────────────────────────────────────────┘

   rd_config.hpp / rd_logger.hpp / rd_clock.hpp — 전 계층 공용 (의존 없음)
```

---

## 9. 문제 진단(00 §4)과의 대응

| # | 문제 | 해소 |
|---|---|---|
| P1 | God object 1162줄 | §5.1 L3 4분할 + §5.2 정책 L2 이관 |
| P2 | 모드 4중 충돌 | 01 §2 이름 규칙 |
| P3 | CLI 이중화 | Q4 — `command_cli` 삭제 |
| P4 | 토픽 난립 47개 | 03 에서 (본 문서는 `rd_telemetry` 단일 소유만 확정) |
| P5 | 모드 3중 분기 | §5.3 `rd_slot_table` — 모드가 데이터가 된다 |
| P6 | 진단 stale | 01 §5.6 `control` 프리셋 |
| P7 | 메시지 패키지 혼재 | Q5 — 03 에서 실행 |
| P8 | DIRECT 크래시 | 01 §9.2 — STM 버퍼 확장 (원인 규명 완료) |
| — | **역방향 의존** | §3 — A1 3종 처방 + §4.1 CMake 강제 |
| — | **detached 스레드** | §6.4 |
| — | **200Hz 스레드의 DDS 블로킹** | §6.3 |
| — | **락 순서 미정의** | §6.5 규약 L1 |

---

## 10. 03 에서 결정할 것

| # | 질문 |
|---|------|
| B1 | `CommLatency` 를 `ControlFeedback` 에 **흡수**할지, 어떤 필드를 남길지 (사용자 제기) |
| B2 | 47개 토픽을 어떻게 접을지 — HANDOFF 의 `uint8 lc[8]`/`hs[8]` 배열안 구체화 |
| B3 | 모드별 발행 세트 — `project`(20Hz status / 100Hz imu / 10Hz battery) vs `control`(200Hz feedback) |
| B4 | `mgs_tp_msgs` 최종 메시지 목록과 필드 |
| B5 | 서비스 op 구성 — `ControlConfig` 에 `SETPOINT`·`SET_ORIGIN` 을 추가할지, 별도 서비스로 뺄지 |
| B6 | 커맨드 슬롯의 "의미 단위 명령"(Q4) 스펙 — raw addr 대신 무엇을 받을지 |

---

## 부록: 결정 요약 카드

```
계층      L3 ros/ (rclcpp 유일) → L2 policy/ → L1 sched/ → L0 core/
강제      CMake 2 라이브러리: rd_core_lib(rclcpp 링크 안 함) / rd_ros_lib
          → policy/ 에 RCLCPP_ 쓰면 링크 에러로 즉시 차단

L3   rd_node(셸) rd_telemetry(발행) rd_control_api rd_carrier_api   ※ api 는 무상태
L2   rd_control rd_profile rd_profile_player rd_oos rd_command rd_sequence
L1   rd_schedule + rd_slot_table(모드=데이터)
L0   rd_map rd_comm rd_uart rd_clock_sync + shadow

역방향 22곳 처방
  (a)설정 10곳 → BridgeConfig 주입
  (b)통지  6곳 → ITelemetrySink (OnTransaction / OnHardwareStatus) — L1 정의, L3 구현
  (c)정책  4곳 → L2 직접 참조 (정상 방향)
  (d)OOS   2곳 → rd_oos 로 L2 분리

스레드 3개
  1 스케줄 (SCHED_FIFO 80, core 11) — rclcpp 호출 금지 (컴파일 타임 보장)
  2 발행   (SCHED_OTHER)            — 큐 pop → publish
  3 spin   (SCHED_OTHER)            — 콜백 + 저주기 타이머 + 액션 5Hz
  삭제: 프로파일 detached 스레드, ThreadStart() 죽은 경로

큐  락프리 SPSC 256슬롯. 생산자 무블록, 가득 차면 새 샘플 드롭 + drop_cnt 누적 통지
    TxnResult 는 값 복사 (reg[256] 포함, ≈320B/50ns)

락  규약1: 한 번에 하나만 (동시 획득 = 설계 오류)
    규약2: 임계구역에서는 값 복사만
    규약3: RT 스레드에서 동적 할당 금지

테스트  ILogger + IClock 주입 → test/unit(노드 없음) / test/integration(계약만)
```
