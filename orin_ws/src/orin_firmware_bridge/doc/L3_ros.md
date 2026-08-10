# L3 — `ros/` : 토픽 · 서비스 · 액션 · 발행

> **이 층이 하는 일**: ROS 표면을 만들고, 하위 계층을 조립하고, 파라미터를 파싱해 내려보낸다.
> **rclcpp 를 아는 유일한 층**이다 (`rd_ros_lib`).
>
> **상태를 갖지 않는 것이 규칙이다.** 서버 핸들만 갖고 실제 상태는 전부 L2 가 갖는다.
> 이 규칙이 지켜지면 웹(`control_web`)이 붙을 때 같은 L2 를 그대로 재사용한다.

## 파일 목록

| 파일 | 역할 |
|---|---|
| `rd_node.{hpp,cpp}` | **조립과 수명만.** 파라미터 파싱 · spin 스레드 · `ITelemetrySink` 위임 |
| `rd_telemetry.{hpp,cpp}` | 섀도 → 메시지 **변환·발행 전담** + 시계 동기 |
| `rd_carrier_api.{hpp,cpp}` | project 계약 (cmd_vel · jeongae · command_set) |
| `rd_control_api.{hpp,cpp}` | control 계약 (config srv · run_profile action · cmd_motor sub) |
| `rd_ros_adapters.hpp` | `ILogger`/`IClock`/`IRunGate` 의 rclcpp 구현체 |

> **노드는 하나다** (`firmware_bridge_node`). L3 를 넷으로 나눈 것은 **C++ 책임 분할**이지
> ROS 노드 분할이 아니다 — 노드를 쪼개면 `ros2 node list` 와 그래프가 바뀌어
> 외부에서 관측 가능한 변화가 생긴다. 셋 다 `RdNode` 위에 엔티티를 만든다.

---

## 1. `RdNode` — 조립과 수명

### 1.1 멤버 — **선언 순서가 곧 소멸 순서의 역순**

```cpp
BridgeConfig config_;                       // 기동 1회 확정 후 불변
RobotState_t* state_;
RdCommand*    command_ = nullptr;

RdControl       control_;    // ⚠ L2 셋이 먼저 —
RdProfilePlayer player_;     //   아래 L3 셋이 이들을 생 포인터로 잡으므로
RdOos           oos_;        //   L2 가 먼저 선언돼야 L3 가 먼저 사라진다

std::unique_ptr<RdTelemetry>  telemetry_;
std::unique_ptr<RdCarrierApi> carrier_api_;
std::unique_ptr<RdControlApi> control_api_;

std::thread spin_thread_;
rclcpp::TimerBase::SharedPtr publish_timer_fast_;   // 100Hz
rclcpp::TimerBase::SharedPtr publish_timer_slow_;   //  10Hz
```

> 순서를 바꾸면 종료 시 이미 죽은 객체를 가리킨다 — **컴파일러는 알려주지 않는다.**

> ⚠ `rd_node.hpp` 에 **메시지·서비스·액션 헤더를 추가하지 않는다.** 종전 여기에
> twist·imu·set_bool·command_set 등 13개가 있었는데, 그 멤버들이 L3 로 옮겨간 뒤에도
> 헤더만 남아 "이 노드가 아직 그것들을 다룬다" 처럼 보였다.

### 1.2 생성자 — 파라미터 파싱 (13개)

| 파라미터 | 타입 | 기본 | 비고 |
|---|---|---|---|
| `bridge_mode` | string | `project` | `project\|control\|manual`. 오타면 **기동 거부** |
| `read_preset` | string | `control` | `control\|diag\|control_test`. 오타면 기동 거부 |
| `auto_mode` | **dynamic** | `current` | `none\|current\|direct\|velocity\|position`. **정수도 받는다** |
| `active_motors` | int[] | `[1,2,3,4]` | 1~4 밖이면 기동 거부. 빈 리스트도 거부 |
| `cmd_current_max` | double | 30.0 | [A] |
| `cmd_vel_guard_enable` | bool | true | **런타임 토글 가능** (`param_cb_`) |
| `cmd_vel_topic_timeout` | double | 0.1 | [s] |
| `cmd_vel_zero_timeout` | double | 3.0 | [s] |
| `stream_timeout` | double | 0.1 | [s] |
| `comm_diag_enable` | bool | false | |
| `enable_dpc_read` | bool | false | 보드 붙였으면 켠다 |
| `enable_pcu_read` | bool | false | 레지스터 미확정 |
| `imu_frame_id` | string | `imu_link` | |

**`auto_mode` 만 `dynamic_typing = true`** 인 이유: 단어형이 정본이지만 정수도 받아야 한다.
안 그러면 문자열로 선언한 파라미터에 `-p auto_mode:=2` 를 주는 순간 타입 불일치 예외로
**노드가 죽는다** (exit 134).

**`active_motors` 를 try/catch 로 감싸는 이유**: CLI 의 `-p active_motors:="[]"` 는 타입
추론이 안 돼 예외가 온다. 그냥 두면 `std::terminate` 로 죽어 원인이 안 보인다.

**금지 auto_mode 2개**: `kinematic` (ECU 가 100Hz 로 `ctr_mode` 를 VELOCITY 로 덮어써
bridge write 와 경쟁), `control` (ECU 미구현 — motor off).

### ⚠ 생성 순서 — L3 셋은 **반드시 파라미터 파싱 뒤에**

```cpp
control_.Bind(state_, cmd_current_max);      // ← 이걸 빼면 PrepareWrite 가 조용히 no-op
control_.SetStreamTimeout(stream_timeout);
player_.SetClampMax(cmd_current_max);

telemetry_   = std::make_unique<RdTelemetry>(this, state_, config_, &control_);
carrier_api_ = std::make_unique<RdCarrierApi>(this, state_, config_);
control_api_ = std::make_unique<RdControlApi>(this, state_, config_, &control_, &player_, &oos_,
                                              counters_lambda);
```

`RdTelemetry` 는 생성자에서 `cfg_.IsControl()`/`cfg_.comm_diag_enable` 을 보고
**조건부 publisher 를 만들지 말지 결정**하고, `RdCarrierApi` 는 `cmd_vel_guard_enable_default`
를 읽는다. 앞에 두면 둘 다 기본값으로 굳어 **§2.5 계측이 통째로 죽는다**
(2026-07-28 기준런에서 rtt·clock_offset 이 전부 0 으로 나와 발견했다).

### 1.3 `AttachCommand` — 나중에 꽂는 것들

`RdCommand` 는 `main.cpp` 에서 노드 뒤에 만들어지므로 나중에 꽂는다.
**한 번에 다섯 개를 같이 꽂아야** "커맨드는 붙었는데 상태에는 안 보이는" 어긋남이 안 생긴다.

```cpp
command_ = command;
carrier_api_->AttachCommand(command);        // command_set / jeongae_lock 서비스 오픈
carrier_api_->AttachControl(&control_);      // B6 게이트가 safe_stop 을 물어야 한다
control_api_->SetSlotSnapshot(...);          // GET_STATUS 의 slots[] 출처
control_api_->SetLastRead(...);              // GET_REGISTERS 의 신선도
control_api_->SetReadPresetPtr(...);         //     "
```

`AttachReadPresetSetter(fn)` 는 **`main.cpp` 에서 스케줄러 생성 후** 꽂는다 —
프리셋 교체는 L1 소유인데 스케줄러가 노드보다 나중에 생기기 때문.

### 1.4 `Start()` — spin 스레드

```cpp
telemetry_->Start();                          // 발행 스레드
spin_thread_ = std::thread([this]{
    if (!rclcpp::ok()) return;                // 기동 직후 종료 경합
    try {
        rclcpp::executors::MultiThreadedExecutor exec;
        exec.add_node(this->get_node_base_interface());
        exec.spin();
    } catch (const rclcpp::exceptions::RCLError& e) { /* 종료 경합은 정상 경로 */ }
});
```

**MultiThreaded 인 이유**: config service 가 read-back 검증으로 최대 50ms 블록되는 동안
action 피드백·취소가 멈추면 안 된다. 기본 콜백 그룹은 `MutuallyExclusive` 라 기존 콜백들끼리의
직렬성은 그대로 유지된다 — **전용 그룹을 가진 config service 만 병렬로 돈다.**

`try/catch` 가 필요한 이유: 이미 shutdown 된 컨텍스트에서 spin 이 guard condition 을 만들다
`RCLError` 로 죽으면 프로세스가 SIGABRT 로 끝난다. `ok()` 검사만으로는 창이 남는다.

### 1.5 타이머 2개

```cpp
publish_timer_fast_ = create_wall_timer(10ms,  [this]{ GetRosInputs(); PublishMotorFeedback(); });
publish_timer_slow_ = create_wall_timer(100ms, [this]{ PublishStatus(); });
```

### 1.6 `StartServers(bool with_profiles)` — 두 서버를 **따로** 연다

```cpp
if (with_profiles) control_api_->StartProfileServer();   // config 도 같이 연다
else               control_api_->StartConfigService();
```

> 종전에는 하나의 `StartProfileServer()` 가 둘을 같이 만들었고, 그것을 부르는 곳이
> "INIT 검증을 통과한 control 구성" 하나뿐이었다. 그래서 `auto_mode: none`(견인 실험)에서는
> **config 서비스까지 통째로 안 열렸다** — `cli status` 가 아예 안 되고, 웹 Tab3 이 죽고,
> 프리셋 교체도 못 했다.

| 서버 | 열리는 조건 | 왜 |
|---|---|---|
| `config` | control 이면 **항상** | 상태를 못 읽으면 조작자가 눈을 잃는다 |
| `run_profile` | **명령을 쓰는 구성에서만** | write 가 없는데 goal 을 수락하면 "재생은 됐다는데 아무것도 안 움직인다" — 거부보다 나쁘다 |

---

## 2. `RdTelemetry` — 변환·발행 전담

### 2.1 발행 계약 (03 §4 에서 전면 교체됨)

> project 47 토픽 → 6 / control 47+2 → 1(+디버그 1) / 진단 26개 → NodeStatus×3 + MotorStatus

| 토픽 | 타입 | 주기 | 조건 |
|---|---|---|---|
| `/carrier_battery` | `UInt8` | 10Hz | 항상 |
| `/carrier_imu` | `sensor_msgs/Imu` | 100Hz | 항상 |
| `/carrier/ecu/status` | `NodeStatus` | 10Hz | 항상 |
| `/carrier/dpc/status` | `NodeStatus` | 10Hz | 항상 |
| `/carrier/pcu/status` | `NodeStatus` | 10Hz | 항상 |
| `/carrier/ecu/motor` | `MotorStatus` | 10Hz | 항상 |
| `/carrier/control/feedback` | `ControlFeedback` | **200Hz** | `IsControl()` |
| `/carrier/control/comm_diag` | `CommDiag` | 5Hz (200Hz÷40) | `comm_diag_enable` |

### 2.2 미판독 표현 — 이 클래스의 절반

```cpp
static constexpr uint8_t kUnread = 0xFF;     // 진단 배열 uint8
static float NaNf();                          // float
bool Reads(uint16_t addr, uint16_t len) const;   // = read_preset_->Covers(addr, len)
```

> **"안 읽음" 과 "정상(0)" 을 절대 섞지 않는다.** 프리셋이 그 구간을 안 읽으면 그렇게 나간다.

`Reads()` 가 **nullptr 일 때 true 를 돌려주면 안 되는 이유**: 그건 "안 읽는 것까지 읽었다고
주장" 하는 쪽이라, 이 함수가 막으려던 바로 그 일을 하게 된다.
2026-07-29 실기에서 실제로 그렇게 나왔다 — 기동 직후 `hw_error` 가 미판독(255)이 아니라
**0("정상")** 으로 발행됐고, 실제 값은 16(미연결 엔코더)이었다.
그래서 기본값을 `&ecu::kPresetControl` 로 두어 **첫 tick 부터 사실과 맞춘다.**

`SetReadPreset()` 통지를 받아야 하는 이유도 같다 — 프리셋이 바뀌면
**어떤 섀도 필드는 더 이상 갱신되지 않는데**, 그걸 모르고 발행하면 낡은 값이 신선한 값처럼 나간다.

### 2.3 `PublishStatus()` (10Hz)

```cpp
{
    std::lock_guard lock(state_->state_mutex);
    ecu_reg = state_->ecu.reg;      // 256B 스냅샷만 뜨고 즉시 해제
    dpc_reg = state_->dpc.reg;      // publish()/DDS 직렬화는 lock 밖에서
    ...
}
```

**NodeStatus 의 `lc`/`hs`** — 채널별 `STATE_t` 에서 모은다. 인덱스 규약(03 §3.1):

```
ECU: 0=uart1/RC  1=uart2/RS485  2=uart6/IMU  3=can1/모터  4=i2c1/엔코더  5=adc/로드셀
DPC: 0=uart2     1=uart4        2=uart6      3=i2c1                      ← **보드마다 다르다**
```

> ⚠ **DPC 는 ECU 와 공용 매핑을 쓸 수 없다**: SYS 위치가 46:16 이고, `hw_*` 순서가 역순이며,
> tick 단위가 ms 다(ECU 는 ×0.1ms). 보드별로 적는 것이 그 결정의 대가다.
>
> DPC 의 `lc`/`hs` 는 **아직 미판독**이다 — 채널별 `STATE_t` 가 SYS(46:16) **밖**에
> 흩어져 있어(uart4=64 / uart2=65 / panel=73 / uart6=110) 지금 읽기 세그로는 안 온다.
> 세그를 넓히기 전까지 0 으로 채우지 않는다.

PCU 는 `publish_stub` 으로 `connected` 만 유효하고 나머지 전부 `kUnread`.

### 2.4 `PublishMotorFeedback()` (100Hz) — IMU

**판정 기준 3중** (순서가 중요):

```cpp
const bool imu_live = Reads(REG_IMU_OFFSET, REG_IMU_SIZE)          // ① 지금 읽고 있나
                   && (imu.delta_tick != ecu::DELTA_STALE)         // ② 취득 시각이 신선한가
                   && (imu.state.bits.lifecycle != LS_OFFLINE);    // ③ 채널이 살아 있다 하나
```

> **`Reads()` 가 맨 앞에 와야 한다.** 섀도는 0으로 초기화되므로 **한 번도 안 읽은 블록은
> `delta_tick == 0`, 즉 "지연 0ms = 아주 신선함" 으로 보인다.** `control_test` 프리셋은
> IMU 를 안 읽는데, 이 가드를 `delta_tick` 만으로 두면 초기값 0이 통과해
> **전부 0인 자세가 그대로 발행된다** (2026-07-30 실기).
>
> **`state` 만 봐서도 안 된다.** IMU 채널이 `lc=1/hs=0`("정상")을 보고하면서 데이터는
> 하나도 안 오는 상태가 관측됐다. 그래서 `orientation = (0,0,0,0)` 이 유효한 자세처럼
> 발행됐다 — 크기가 0인 사원수는 물리적으로 존재할 수 없고, 정규화하는 소비자는 0으로 나눈다.

죽었을 때는 **두 가지를 같이** 한다: 전 필드 `NaN` + `covariance[0] = -1.0`(REP-145).
REP-145 를 지키는 소비자는 covariance 로 걸러내고, 안 지키는 소비자도 NaN 을 만나
결과가 NaN 으로 번지므로 **조용히 틀리지 않는다.**

**단위 환산 상수** (발행 시점에만 쓰이므로 여기가 제자리):

```cpp
kQuatScale  = 0.0001f;                    // ×0.0001
kGyroToRads = 0.1f * 0.01745329252f;      // ×0.1 [deg/s] → [rad/s]
kAccToMs2   = 0.001f * 9.81f;             // ×0.001 [g] → [m/s²]
kEncToDeg   = 360.0f / 4096.0f;           // AS5600 12bit
```

### 2.5 `OnFeedback()` (200Hz) — `ControlFeedback`

**1 RW 트랜잭션 = 1 메시지.**

**시간축**

```cpp
if (last_clock_valid_)
    m.header.stamp = rclcpp::Time((reg.sys.realtime_tick * 1e-4 + last_clock_offset_) * 1e9);
else
    m.header.stamp = node_->now();     // fallback. stamp_valid=false 로 알린다
```

**신선도 게이트 4개**

```cpp
motor_fresh    = Reads(88, 36);
loadcell_fresh = Reads(42, 6);
imu_fresh      = Reads(48, 22);
enc_fresh      = Reads(70, 16);
```

`dt_*` 는 안 읽는 구간이면 `NaN` — 0 으로 두면 "지연 없음" 으로 오독된다.
`fb_current/velocity/position` 도 `motor_fresh` 가 아니면 `NaN` — 0 은 "정지" 라는
**유효한 관측값**이다.

**`link_angle` 은 채널별로 판정한다**

```cpp
const bool ch_live = enc_fresh && reg.encoder.delta_tick[i] != ecu::DELTA_STALE;
```

> AS5600 5채널은 I2C MUX 로 순차 취득하므로 **일부만 죽는 것이 정상적인 고장 양상**이고,
> 실제로 2026-07-29 진단에서 ch0 만 살아 있었다. 그때 raw 값은 ch1~4 도 12bit 범위 안의
> **그럴듯한 잔값**이었다 — 값의 그럴듯함으로 판정하면 죽은 채널이 정상 각도로 나간다.
> 기준은 `delta_tick` 하나뿐이다.

**`cmd` 는 auto_mode 가 정한 write 범위의 값이어야 한다**

```cpp
m.cmd[i] = (am == AUTO_MODE_POSITION) ? reg.cmd_motor.cmd_position[i]
         : (am == AUTO_MODE_VELOCITY) ? reg.cmd_motor.cmd_velocity[i]
                                      : reg.cmd_motor.cmd_current[i];
```
`cmd_current` 를 고정으로 실으면 POSITION·VELOCITY 에서 **명령이 나가고 있는데 0 으로 보인다.**

**IMU 축 순서**: 메시지는 `(x,y,z,w)`, 레지스터는 `(z,y,x,w)` — **다르다.**
여기가 유일한 변환 지점이므로 인덱스를 명시한다.

### ⚠ 메시지 구성은 스케줄 스레드에서 끝낸다

```cpp
TeleItem_t item; item.has_feedback = true; item.feedback = m;
tele_q_.TryPush(item);      // 가득 차면 드롭 + drop_cnt++ — 절대 블록하지 않는다
```

발행 시점에 섀도를 읽으면 **tick 시점의 값이 아닌 것이 나간다.** 넘기는 것은 `publish()` 뿐이다.

### 2.6 `OnTxn()` (200Hz) — 시계 추정기

**추정기는 항상 돈다.** `comm_diag` 가 꺼져 있어도 `ControlFeedback.header.stamp` 가
이 결과를 쓴다 — 발행만 가리는 것이지 계산을 가리는 것이 아니다.

```cpp
proc_prev = (reg.sys.rs485_proc_delta == DELTA_STALE) ? -1.0 : raw * 1e-4;
auto est  = clock_sync_.Update(txn, reg.sys.realtime_tick, proc_prev);
// last_clock_offset_ / last_clock_valid_ / last_rtt_ / last_quality_ / last_drift_ppm_ 갱신
```

수렴하면 **1회만** INFO 로 알린다. `CommDiag` 는 `kCommDiagDecim = 40` 으로 솎아 5Hz.

### 2.7 `TelemetryLoop()` — 발행 스레드

```cpp
while (tele_running_.load() && rclcpp::ok()) {          // ⚠ ok() 를 반드시 같이 본다
    while (tele_q_.Pop(&item)) { ...publish... }
    if (!any) std::this_thread::sleep_for(1ms);
}
```

> `rclcpp::ok()` 없이 두면 SIGTERM 으로 `shutdown()` 이 먼저 돌 때 **파괴 중인 컨텍스트에
> publish** 하게 된다 (5단계에서 실기 SIGSEGV 로 드러났다).
> 종료 시 남은 것을 비우되, 컨텍스트가 죽었으면 버린다 — 크래시보다 낫다.

---

## 3. `RdCarrierApi` — project 계약

```cpp
sub  /carrier_cmd_vel        geometry_msgs/Twist
sub  /jeongae                mgs01_base_msgs/JeonGae
srv  /carrier/command_set    mgs_tp_msgs/CommandSet
srv  /carrier/jeongae_lock   std_srvs/SetBool
```

**상태를 갖지 않는 것이 규칙**이나 예외 하나가 `inputs_`(`RosInputs_t`) 다 —
"마지막으로 언제 왔는가" 라는 **ROS 표면의 사실**이라 여기가 제자리다.

### 3.1 cmd_vel 워치독 — 두 겹

```cpp
bool ShouldSkipCmdWrite() {                        // 스케줄러가 매 tick 묻는다
    if (!guard_enable_.load()) return false;
    if (now - last_topic_time   > cmd_vel_topic_timeout) return true;   // ① 100ms 미수신
    if (now - last_nonzero_time > cmd_vel_zero_timeout)  return true;   // ② 3초 0 수렴
    return false;
}
```

`jeongae` 토픽도 `last_topic_time` 을 갱신한다 (명령 토픽에 포함).

`GetRosInputs()` (100Hz) 는 두 단계로 나눠 락을 짧게 유지한다:

```
Step 1: data_mutex_ 로 inputs_ 복사 + 0.5초 워치독 판정
Step 2: state_mutex_ 로 cmd_system.cmd_lin_vel/cmd_ang_vel 에 반영 (또는 0)
```

> ⚠ 종전 Step 2 에서 `dpc.reg.cmd.cmd_jeongae = jeongae;` 를 했다. 그 필드는 레지스터
> 미확정 시절의 **골격 스텁**이었고 실제 맵에 "jeongae" 필드는 없다 — 전개는
> `sys_state_target`(127)로 FSM 을 몰고, 잠금은 `servo_cmd`(125)로 한다.
> 그 write 는 어차피 아무 데도 가지 않았다(프레임에 DPC WRITE 슬롯이 없다).
> **`/jeongae` 토픽의 실제 경로는 `RdSequence` 다.**

### 3.2 `CallbackCommandSet` — 게이트를 여기서 건다

```cpp
if (req->action != ACTION_RESET) {
    const bool safe   = control_ ? control_->SafeStop(&why_stop) : false;
    const uint8_t a0  = req->args.empty() ? 0 : req->args[0];
    const auto g = cmdcat::Gate(req->cmd, cfg_.IsManual(), safe, a0);
    if (!g.ok) { res->accepted = false; res->message = "거부: " + g.why + ...; return; }
}
```

- **판정은 카탈로그의 순수 함수가 한다** — 여기서 조건을 다시 적으면 표와 갈라진다.
  이 층이 하는 일은 **컨텍스트를 모으는 것**뿐이다.
- **게이트를 CLI 에만 두지 않는다** — 클라이언트에만 있는 가드는 가드가 아니다.
- **RESET 은 어느 모드에서든 허용** — 잘못 넣은 명령을 빼는 수단까지 잠그면
  게이트의 목적과 정반대가 된다.
- `control_` 미주입이면 `safe = false` → WRITE 계열 전부 거부. **"모르면 막는다" 가 안전측이다.**

---

## 4. `RdControlApi` — control 계약

```cpp
sub     /carrier/control/cmd_motor    mgs_tp_msgs/CmdMotor        (항상 열려 있다)
srv     /carrier/control/config       mgs_tp_msgs/ControlConfig   (StartConfigService)
action  /carrier/control/run_profile  mgs_tp_msgs/RunProfile      (StartProfileServer)
```

> 구독은 **항상** 연다. arm=off 면 받아서 버린다 — 구독 자체를 늦게 열면 arm 을 켠 직후
> 첫 메시지를 놓쳐 STREAM 진입이 한 박자 늦는다.

`cfg_` 가 **비 const** 인 이유: `OP_SET_ACTIVE_MOTORS` 가 `active_motor_mask` 를 런타임에
바꾼다. `BridgeConfig` 의 "기동 후 불변" 약속에서 유일하게 벗어나는 필드이며,
그 사실을 **타입으로 드러내 둔다.**

### 4.1 `ControlConfig` 서비스 — op 목록

| op | 게이트 | 하는 일 |
|---|---|---|
| `OP_GET_STATUS` | **없음** (부작용 없음) | `StatusToJson(snap)` — RUNNING 중에도 물을 수 있어야 한다 |
| `OP_GET_REGISTERS` | **없음** | 섀도 hex 덤프 + **신선도 구간** + `read_age_s` |
| `OP_REARM` | LOCKED 전용 | `control_->Rearm()` |
| `OP_SET_STREAM_ARM` (value=0) | **없음** (감속 방향) | arm off — 웹의 stop 버튼 |
| `OP_SET_ACTIVE_MOTORS` | IDLE | out-of-span WRITE(192) + 검증 → `cfg_.active_motor_mask` 갱신 |
| `OP_SET_CTR_MODE` | IDLE + **DIRECT 전용** | in-span 섀도 변경 → 다음 RW 의 read 세그로 검증 |
| `OP_SET_MODE` | IDLE | out-of-span WRITE(190) |
| `OP_SET_AUTO_MODE` | IDLE | 아래 §4.2 |
| `OP_SET_READ_PRESET` | IDLE | `set_read_preset_(id, &why)` 콜백 (L1 소유) |
| `OP_SET_ORIGIN` | IDLE + DIRECT + safe_stop | `control_->RequestOriginPulse()` |
| `OP_SET_STREAM_ARM` (value≠0) | IDLE | arm on |

**게이트가 앞에 오지 않는 셋**: `GET_STATUS`·`GET_REGISTERS` 는 부작용이 없고,
`arm off` 는 **감속 방향**이다.

> STREAM 에서 config 를 통째로 막으면 **stop 이 안 눌린다** — 달리는 중에 멈추는 수단을
> 잠그는 셈이라 게이트의 목적과 정반대가 된다. (arm **on** 은 설정 변경이므로 게이트를 받는다.)

**`OP_GET_REGISTERS` 의 `fresh` 는 두 출처의 합집합이다**

```
① 200Hz 루프가 매 tick 읽는 현재 프리셋 (ECU 만)   ← preset_ptr_()
② 마지막으로 성공한 슬롯 READ                       ← last_read_()
```

여기 안 들어가는 자리는 섀도에 값이 있어도 **읽은 적이 없거나 낡은 것**이다.
콜백이 없으면 `fresh` 가 비고 UI 는 전 구간을 회색으로 그린다 —
**모르면 회색이 안전측이다** ("모르면 관대하게" 가 낡은 값을 신선하게 보이게 만든 적이 있다).

### 4.2 `DoSetAutoMode` — **순서가 안전의 핵심**

**1→2 (범위 확장, `→ DIRECT`)**

```
① shadow 소독:  ctr_mode = CURRENT, cmd_position/velocity = 0   (FSM + 섀도 양쪽)
② out-of-span WRITE(188) + read-back 검증
③ control_->SetAutoMode(mode)     ← 검증 성공 후에만 범위 확장
```

소독을 먼저 하지 않고 범위를 넓히면, 그동안 read 로 유입됐거나 과거에 남은 섀도의
`ctr_mode`/`pos`/`vel` 잔재가 **그대로 ECU 로 나간다** (128:52 를 쓰기 시작하므로).

**2→1 등 (범위 축소)**

```
① WRITE + 검증  →  ② SetAutoMode  →  ③ ctr_mode 섀도를 AutoModeForcedCtrMode(mode) 로 맞춤
```

축소는 쓰는 바이트가 줄어드는 방향이라 잔재가 나갈 창이 없다.
③은 read-back 혼선 방지 — ECU 가 강제하는 값과 섀도를 맞춰 둔다.

### 4.3 `DoInSpanCtrMode` — in-span 은 패킷 삽입이 없다

`ctr_mode`(128~131)는 RW write 범위 안이므로 **섀도만 바꾸면 다음 RW tick 이 자연 반영**한다.
검증은 같은 트랜잭션의 read 세그(`{128,4}`)로 돌아온 값을 본다 —
최대 `kVerifyTicks`(10) tick 대기.

> `auto_mode != DIRECT` 면 **거부한다.** 그때 `ctr_mode` 는 bridge write 범위(164:16) 밖이라
> wire 로 나가지도 않고 ECU 가 100Hz 로 CURRENT 를 강제한다.
> 조용히 10 tick 타임아웃으로 실패하느니 이유를 분명히 밝히고 거부한다.
>
> 이 검증이 성립하려면 **활성 프리셋이 `{128,4}` 를 읽어야 한다.**
> 안 읽으면 브리지가 쓴 값을 자기가 읽어 **검증이 무조건 통과한다** —
> `rd_read_preset.hpp` 의 `static_assert` 가 그것을 막는다.

### 4.4 `run_profile` action

**goal 수락 (`HandleProfileGoal`)** — 순서가 중요하다

```
① FSM 이 IDLE 인가                      (RUNNING 중 새 goal 거부 = 기존 실험 오염 방지)
② player_->Load(yaml, mask, &err)      ← **먼저 파싱해야 mode 를 안다**
③ player_->AcceptsAutoMode(auto_mode, ctr_mode[], mask, &why)
④ player_->EffectiveLimits(&lo, &hi) → control_->SetProfileLimits(lo, hi)
⑤ ACCEPT_AND_EXECUTE
```

실패는 전부 reject — RUNNING 에 들어간 뒤 죽는 것보다 낫다.

**실행 (`ExecuteProfile`)** — **별도 스레드에서 detach.**
여기서 블록하면 executor 가 막혀 cancel 도 못 받는다.

```
c_at_start = counters_()               ← 런 시작 스냅샷
goal_id = player_->Begin()
control_->BeginProfile(goal_id)        ← 실패하면 abort (그 사이 LOCKED 등)
player_->Activate()                    ← 이 시점부터 스케줄 tick 이 샘플을 소비

5Hz 피드백 루프: cancel? / done? / FSM 이 RUNNING 이 아니면 break
                gh->publish_feedback({t, segment_index, progress})

executed = player_->End();  control_->EndProfile()
c_end = counters_()
result 의 카운터는 **c_end - c_at_start 구간 증분**
```

> **증분으로 내는 이유**: 노드 기동 이후 누적을 그대로 실으면 두 번째 런부터 이전 런의
> 사고가 섞여 들어와 그 런을 잘못 판정하게 된다.

**`result.clock_converged` 는 시작과 끝 **둘 다** true 여야 true 다** —
어느 한쪽이라도 미수렴이면 그 런의 `header.stamp` 는 **등급이 다르다.**

`run_dir` 은 빈 문자열 — **폴더는 CLI 소유다. 브리지는 경로를 모른다.**

### 4.5 `RunCounters_t` — 한 스냅샷으로 모아 온다

```cpp
struct RunCounters_t {
    uint64_t write_err_total;  uint32_t irregular_tick_cnt, late_tick_cnt;
    uint64_t drop_cnt;         bool clock_converged;  float drift_ppm;
    float stamp_quality_ms;    double rtt_ms;  uint8_t rw_err;  const char* read_preset_name;
};
```

**한 스냅샷인 것이 중요하다** — 필드마다 따로 읽으면 tick 경계를 가로질러 서로 다른 시점의
값이 섞인다. 그리고 이것이 **L2/L3-api 가 `RdTelemetry` 를 직접 참조하지 않기 위한 경계**다.

### 4.6 `CmdMotor.ctr_mode` 는 예약 필드다

담기만 하고 아무도 읽지 않는다. 발행자가 모르고 채워 보내면 "모드를 지정했는데 안 먹는다"
가 되므로 **1회만** 경고한다 (200Hz 로 들어오므로 반복 경고는 로그를 덮는다).

> 읽게 만들지 않는 이유: 200Hz 스트림이 제어 모드를 바꾸면 **회전 중 모드 전환**이 가능해져
> `safe_stop` 게이트가 무의미해진다. 모드는 `config ctr_mode`(IDLE 전용)로 설정한다.

---

## 5. `rd_ros_adapters.hpp` — rclcpp 구현체

```cpp
class RclcppLogger : public ILogger { ... };   // RCLCPP_* 로 그대로 넘긴다 (출력 동일)
using RosClock = SystemClock;                  // steady_clock / system_clock
class RclcppRunGate : public IRunGate { bool Ok() const override { return rclcpp::ok(); } };
```

> **이 헤더를 L0/L1/L2 에서 include 하면 안 된다.** 여기가 rclcpp 를 아는 유일한 지점이고,
> `rd_core_lib`/`rd_ros_lib` 분리에서 이 파일은 후자에 남는다.
>
> `RosClock` 을 `rclcpp::Clock` 으로 바꾸지 않는 이유: `use_sim_time` 여부에 따라
> 시간동기 축이 달라진다.

인스턴스는 `main.cpp` 에서 `static` 으로 만들어 하위 계층에 주소로 주입한다 —
수명이 scheduler/bridge 보다 길다.
