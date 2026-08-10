# L0 — `core/` : 시리얼 · 패킷 · 레지스터 섀도

> **이 층이 하는 일**: 바이트를 주고받고, 그 바이트를 레지스터 섀도의 제 자리에 놓는다.
> 정책은 하나도 없다. "지금 해도 되는가" 를 묻지 않고 "시키는 대로" 한다.
>
> **rclcpp 를 모른다.** 로깅은 `ILogger`, 시각은 `IClock`.
> 그래서 `test/unit/*` 대부분이 노드 없이 돈다.

## 파일 목록

| 파일 | 역할 |
|---|---|
| `rd_common.hpp` | `RD_RET`, `STATE_t`(lifecycle/health), `LS_*`/`HC_*` 상수 |
| `rd_uart.{hpp,cpp}` | LibSerial 포트 + poll() 수신 + RS485 턴어라운드 |
| `rd_comm.{hpp,cpp}` | 패킷 프레이밍 · CRC-16 · 2-stage 수신 |
| `rd_map.{hpp,cpp}` | `TaskConfig_t` → 패킷 Data 인코딩 / 응답 → 섀도 디코딩 |
| `rd_register_ecu.hpp` | ECU 256B 레지스터 맵 (STM 헤더의 C++ 미러) |
| `rd_register_dpc.hpp` | DPC-B 256B 맵 + `DPCB_STATE_e` |
| `rd_register_pcu.hpp` | PCU 256B 맵 (레지스터 일부 미확정) |
| `rd_read_preset.hpp` | 읽기 프리셋 표 + `Covers()` |
| `rd_clock_sync.{hpp,cpp}` | ECU tick ↔ Orin epoch 매핑 추정기 |

---

## 1. `RD_RET` — 이 패키지의 반환 코드

```cpp
typedef enum { RD_OK, RD_TIMEOUT, RD_ERROR, RD_FATAL, RD_CLOSED } RD_RET;
```

**`RD_FATAL` 만 특별하다.** 스케줄 루프가 이것을 받으면 `RunLoop` 을 빠져나가
`SupervisorLoop` 이 1초 뒤 포트를 다시 연다. 나머지는 그 tick 만 실패한 것이다.

## 2. `RdUart` — 포트

```cpp
RdUart(const std::string& port_name);
RD_RET Init();                                    // 921600 8N1, VMIN=0/VTIME=0
RD_RET Stop();                                    // 닫고 Uninitialized 로
void   Flush();                                   // FlushInputBuffer
RD_RET Write(uint8_t* pBuf, size_t length);
RD_RET Read(uint8_t* pBuf, size_t length, size_t timeout_ms);
```

### 알아야 할 두 가지

**① `Write` 뒤의 턴어라운드 대기 (`rd_uart.cpp:105~115`)**

RS485 반이중이라 송신이 물리적으로 끝나기 전에 `Read` 를 시작하면 TX/RX 가 겹쳐
0x00 프레이밍 에러가 난다. 과거엔 `tcdrain` 을 썼는데 FTDI 에서 **데이터 양과 무관하게
8~13ms 블록** — 200Hz 주기 초과의 단독 원인이었다.

지금은 계산된 전송시간 + 마진만 잔다:

```cpp
static constexpr int64_t kTxUsbMarginUs = 1500;          // USB 풀스피드 프레임 + FTDI 여유
const int64_t tx_bits_us = length * 10 * 1000000 / 921600;   // 8N1 = 10bit/byte
std::this_thread::sleep_for(std::chrono::microseconds(tx_bits_us + kTxUsbMarginUs));
```

> Loss 가 보이면 `kTxUsbMarginUs` 를 1000~3000us 사이에서 튜닝한다.

**② `Read` 는 `poll()` 기반이다 (`rd_uart.cpp:141~177`)**

LibSerial 의 busy-poll(스핀 + 벽시계 비교)을 쓰면 읽는 도중 스레드가 선점당했을 때
**벽시계 기준 오탐 타임아웃**이 난다. 커널 블로킹으로 바꿔 그 손실 원인을 제거했다.

### 에러 누적 정책 (`HandleErrorState`)

```
counter < 10  → RD_TIMEOUT (조용히 무시)
counter < 30  → RD_ERROR   (경고 출력)
그 이상        → RD_FATAL   (Supervisor 가 포트 재오픈)
```

성공하면 카운터가 0 으로 리셋된다. `RdComm` 도 같은 구조를 갖는다 (5 / 20).

---

## 3. `RdComm` — 패킷

### 3.1 와이어 포맷

```
 [0xAA][0x55][ ID ][Len L][Len H][Inst][ Data ... ][CRC L][CRC H]
  └─ HEADER_PART_LEN = 5 ─┘                          └ TAIL_LEN = 2 ┘

 Length = Inst(1) + Data(N) + CRC(2) = N + 3        (Little Endian)
```

| 상수 | 값 | 의미 |
|---|---|---|
| `MAX_PACKET_LEN` | 256 | |
| `MAX_DATA_LEN` | 248 | `= 256 - 5 - 1 - 2`. **응답 예산의 병목** |
| `MIN_LENGTH_FIELD` | 3 | Data 0바이트 케이스 |
| `MY_ID` | 0x01 | ORIN — 수신 필터링용 |

### 3.2 ID 와 Inst

```cpp
enum class PacketID : uint8_t {
    ORIN = 0x01, ECU = 0xE1, DPC_B = 0xD1,   // 2026-08-03 0xE2 → 0xD1
    DPC_A = 0xD2,   // ⚠ 미확인 — 브리지는 쓰지 않는다 (DPC-B 가 UART4 로 중계)
    PCU = 0xA1,
};
enum class PacketInst : uint8_t { PING=0x01, READ=0x02, WRITE=0x03, RW=0x04, REBOOT=0x08 };
```

**`RW`(0x04)가 이 프로토콜의 핵심이다** — 한 트랜잭션에 write 적용 후 read 스냅샷.
읽기와 쓰기가 한 왕복이라 200Hz 제어가 성립한다.

### 3.3 `Read` — 2-stage 가변 길이 수신

```cpp
RD_RET Read(PACKET_comm_t* pkt, size_t header_timeout_ms, size_t body_timeout_ms);
```

1. **Stage 1** — 헤더 5B. `0xAA 0x55 0x01` 셋 다 맞아야 한다. 하나라도 어긋나면
   라인 동기화가 깨진 것 → `HandleCommError`.
2. Length sanity check (`3 ≤ len ≤ MAX_DATA_LEN+3`)
3. **Stage 2** — 바디 `length` 바이트.
4. CRC-16/IBM (`0x8005`, LSB-first 룩업 테이블) 검증 — 헤더 5B 부터 CRC 직전까지.
5. 구조체에 직접 `memcpy`. `rx.data_len = length - 3` (= Data 유효 바이트 수).

> 호출은 `RdSchedule::ExecuteTask` 에서 `comm_->Read(&packet_obj_, 2, 2)` —
> **헤더 2ms + 바디 2ms = 최악 4ms < 5ms 주기** (1ms 마진).

### 3.4 flush 는 이 클래스가 하지 않는다

**유일한 flush 지점은 `RdSchedule::ExecuteTask` 의 `comm_->Clear()`** — Write 직전 1회.
어떤 에러로 빠져나가든(sync/length/body/CRC) 잔여 바이트는 다음 사이클 시작 시 비워진다.
`RdComm::Read` 안에 flush 를 또 두면 복구 지점이 둘이 되어 어느 쪽이 치웠는지 알 수 없게 된다.

---

## 4. 읽기 프리셋 (`rd_read_preset.hpp`)

### 4.1 왜 있는가

"IDLE 에서 다른 센서를 계속 보고 싶다" 를 **커맨드 슬롯으로 푸는 것은 층이 틀렸다.**
그건 프레임에 트랜잭션을 끼우는 일이 아니라 **같은 슬롯이 읽는 구간을 바꾸는 일**이다.
슬롯을 안 쓰므로 tick 소모가 0 이고, 커맨드 슬롯이 0개인 control 에서도 가능하다.

### 4.2 타입

```cpp
struct ReadSpanEntry { uint16_t addr; uint16_t len; };

constexpr uint16_t kMaxRespPayload = MAX_DATA_LEN - 1;   // = 247. **파생시킨다**
constexpr uint8_t  kMaxReadSegs    = 6;

struct ReadPreset {
    const char*   name;
    ReadSpanEntry spans[kMaxReadSegs];
    uint8_t       count;

    constexpr uint16_t RespPayload() const;              // Σ spans[i].len
    constexpr bool     Covers(uint16_t addr, uint16_t len) const;
};
```

> `kMaxRespPayload` 를 손으로 적지 않는 이유: 종전에 `88` 이 박혀 있었는데 그것은 STM
> `PACKET_DATA_BUF_SIZE` 가 90 이던 시절 값이다. STM 이 256 으로 커졌는데 이 숫자가
> 따라오지 않아 **들어갈 수 있는 읽기 구간이 컴파일 단계에서 거부되고 있었다.**

### 4.3 `Covers()` — 이 파일의 핵심

`[addr, addr+len)` 이 **어느 한 세그먼트에 온전히 담기는가.** 두 세그먼트에 걸치면 `false` 다 —
부분적으로 읽힌 블록을 "읽었다" 고 하면 그 안의 어떤 필드가 낡았는지 알 수 없다.

이것이 있어야 텔레메트리가 매 발행마다 "이 블록을 지금 읽고 있나" 를 물을 수 있고,
안 읽는 자리를 **0 이 아니라 미판독 센티넬**로 내보낼 수 있다.

### 4.4 프리셋 4개

| id | 이름 | 세그먼트 | 응답 | 용도 |
|---|---|---|---|---|
| 0 | `control` | `{16,17} {42,86} {128,4} {164,16}` | 123B | 기본. 제어 실험 |
| 1 | `diag` | `{0,33} {128,4} {224,32}` | 69B | 진단 (모터 블록 빠짐) |
| 2 | `control_test` | `{16,17} {42,6} {88,40} {164,16}` | 79B | 견인 실험 (로드셀 중심) |
| — | `project` | `{16,17} {48,22} {86,42}` | 81B | **선택 불가.** project 프레임이 FIXED 로 소유 |

```cpp
constexpr const ReadPreset* kPresets[] = { &kPresetControl, &kPresetDiag, &kPresetControlTest };
constexpr uint8_t kPresetCount = 3;
inline bool ParseReadPreset(const std::string& s, uint8_t* out);   // 이름 → id
```

> `kPresetProject` 를 `kPresets[]` 에 넣지 않는 이유: control 모드에서 그걸 고르면
> 모터 cmd read-back 도 ctr_mode read-back 도 없는 채로 제어가 돌아
> "왜 검증이 실패하지" 가 된다.

### 4.5 불변식 — `static_assert` 가 지킨다

프리셋을 고치면 **컴파일이 먼저 막는다.** 지금 걸려 있는 것들:

- 모든 프리셋이 SYS 전체(`{16,17}`)를 읽는다 → `NodeStatus` 매핑이 보드·모드 분기 없이 성립
- `control` 은 진단 채널 6개의 `STATE_t` 를 **전부** 덮는다
  (RC 87 / RS485 86 / IMU 69 / 모터 127 / 엔코더 85 / 로드셀 47)
- `control` 은 `{128,4}` ctr_mode read-back 을 갖는다 → 없으면 `SET_CTR_MODE` 검증이 **무조건 통과**
- `control` 은 `{164,16}` cmd_current read-back 을 갖는다 → 없으면 `ControlFeedback.cmd` 가 낡는다
- `project` 는 엔코더를 **읽지 않는다** (의도. 실수로 들어오면 응답이 커진다)
- `control_test` 는 ctr_mode read-back 이 **없다** (auto_mode:none 전용이라는 뜻)
- 전 프리셋 `RespPayload() ≤ kMaxRespPayload`

---

## 5. `RdMap` — 인코딩 / 디코딩

### 5.1 `TaskConfig_t` — 트랜잭션 1건의 명세

```cpp
struct TaskConfig_t {
    uint8_t    target_id;      // TARGET::{ECU,DPC,PCU}
    PacketInst inst;           // READ / WRITE / RW / REBOOT
    uint16_t   start_addr;     // READ: 대표 주소 / WRITE·RW: write 시작 주소
    uint16_t   data_len;       // WRITE·RW: write 바이트 수

    static constexpr int MAX_READ_SEGS = 8;
    Segment_t  segs[MAX_READ_SEGS];    // READ/RW 의 읽기 세그먼트
    uint8_t    seg_count;              // 0 이면 단일 구간(start_addr/data_len) 모드
};
```

생성자 3종:

```cpp
TaskConfig_t(tid, inst, addr, len)                       // 단일 구간 READ / WRITE
TaskConfig_t(tid, inst, {seg, seg, ...})                 // 멀티세그 READ
TaskConfig_t(tid, inst, waddr, wlen, {seg, seg, ...})    // RW (write 구간 + read 세그)
```

### 5.2 `Encode` — inst 별 Data 레이아웃

```cpp
RD_RET Encode(const TaskConfig_t& config, RobotState_t* state,
              PACKET_comm_t* pkt, size_t* out_data_len);   // 내부에서 state_mutex 를 잡는다
```

| inst | Data 내용 | 길이 |
|---|---|---|
| `READ` (seg) | `[addrL,addrH,lenL,lenH] × n` | `4n` |
| `READ` (단일) | `[addrL,addrH,lenL,lenH]` | 4 |
| `WRITE` | `[addrL,addrH] + 섀도 바이트` | `2 + len` |
| `RW` | `[n] + 세그×4 + [waddrL,waddrH] + 섀도 write 바이트` | `1+4n+2+wlen` |
| `REBOOT`/`PING` | 없음 | 0 |

> **WRITE/RW 의 데이터 출처는 언제나 섀도다.** "무엇을 쓸까" 를 여기서 정하지 않는다 —
> 상위 계층(`RdControl::PrepareWrite`, `RdCarrierApi::GetRosInputs`, `RdCommand`)이
> 미리 섀도에 써 두면 이 함수는 그 구간을 그대로 퍼 나른다.

**연결 판정**: `EncodeNode` 가 매번 `comm.timeout_cnt++` 하고, `3` 을 넘으면
`is_connected = false`. `DecodeNode` 가 0 으로 되돌리고 `true` 로 만든다.

### 5.3 `Decode` — 응답 → 섀도

```cpp
RD_RET Decode(PACKET_comm_t* pkt, const TaskConfig_t& sent_config, RobotState_t* state);
```

1. **inst echo 검증** — STM 은 별도 RESPONSE inst 없이 보낸 inst 를 되돌려 준다.
2. **ID 검증** — 응답 ID 는 반드시 `ORIN`(0x01).
3. inst 별 처리:

| sent inst | 처리 |
|---|---|
| `RW` | `Data[0]` 이 **니블 분리**: `read_err \| (write_err << 4)`. write 에러는 read 스트림을 끊지 않는다 |
| `READ` | `Data[0]` 에러 확인 후 `ScatterReadSegs` |
| `WRITE`/`REBOOT`/`PING` | `Data[0]` 만 확인 |

**`ScatterReadSegs`** — 응답의 연접 데이터 `Data[1..]` 를 각 세그먼트의 제 주소로 흩뿌린다.
길이가 안 맞으면(`received != expected`) `RD_ERROR` — 조용히 어긋난 자리에 쓰지 않는다.

### 5.4 RW write 에러 카운터 — 상위가 소비한다

```cpp
uint64_t RwWriteErrStreak() const;   // 연속 거부 tick 수 (write 성공 시 0)
uint64_t RwWriteErrTotal()  const;   // 누적 (리셋 없음) — action result 의 구간 차분용
uint8_t  LastRwErr()        const;   // 직전 응답의 에러 니블 → ControlFeedback.rw_err
```

`RwWriteErrStreak()` 값이 곧 "연속 거부 tick 수" 이므로 제어 FSM 의 `LOCKED` 래치 판정에
그대로 쓰인다 (`RdControl::NoteWriteErrStreak`, 임계 50 tick = 0.25초).

### 5.5 `ShadowBase` — 섀도 포인터 (인라인 자유 함수)

```cpp
inline uint8_t* ShadowBase(RobotState_t* st, uint8_t target_id, uint16_t* total_size);
```

**여기 하나만 둔다.** 종전에 `rd_command.cpp` 의 static 함수였고, 다른 곳에서 필요해지는
순간 복사될 자리였다 (실제로 웹 Tab3 에서 필요해졌다).
포인터만 돌려주며 **호출자가 `state_mutex` 를 잡아야 한다.**

---

## 6. 레지스터 맵

### 6.1 ECU (`rd_register_ecu.hpp`) — 256B

| addr | 크기 | 구조체 | 내용 |
|---|---|---|---|
| 0 | 16 | `DEFINE_t` | `sys_write_mode`, timeout/cnt 설정, `hw_reset`(5) |
| **16** | **17** | `DATA_SYSTEM_t` | `degraded_cnt[8]`, `hw_error`(24), `hw_fatal`(25), `hw_reset`(26), `sys_state`(27), `realtime_tick`(28, u32), `rs485_proc_delta`(32) |
| 33 | 9 | — | RSVD0 |
| 42 | 6 | `DATA_LOADCELL_t` | `avg[2]`, `delta_tick`(46), `state`(47) |
| 48 | 22 | `DATA_IMU_t` | quat z/y/x/w, gyro, acc, `delta_tick`(68), `state`(69) |
| 70 | 16 | `DATA_ENCODER_t` | AS5600 5ch raw, `delta_tick[5]`(80), `state`(85) |
| 86 | 1 | `DATA_UART2_t` | RS485 `state` |
| 87 | 1 | `DATA_RC_t` | RC 수신기 `state` |
| 88 | 40 | `DATA_MOTOR_t` | pos/vel/cur/temp ×4, `delta_tick[4]`, `cmd_delta_tick[4]`, `error_code`(124), `comm_err`(126), `state`(127) |
| **128** | **52** | `CMD_MOTOR_t` | `ctr_mode[4]`(128), `cmd_position[4]`(132), `cmd_velocity[4]`(148), `cmd_current[4]`(164) |
| **180** | **13** | `CMD_SYSTEM_t` | `cmd_lin_vel`(180), `cmd_ang_vel`(184), `auto_mode`(188), `soft_estop`(189), `mode`(190), `use_lpf`(191), `motor_mask`(192) |
| 193 | 31 | — | RSVD1 |
| 224 | 32 | `DIAG_t` | `cmd_write_tick` |

> `static_assert(sizeof(REGISTER_t) == 256)` 등 12개가 레이아웃을 고정한다.
> STM `Core/Inc/rd_register_ecu.h` 와 **바이트 단위로 동일**해야 하며,
> 그 대조는 `test_stm_mirror` 가 한다 (`stm_ws` 가 함께 있을 때만 빌드).

**주요 상수**

```cpp
// ctr_mode (128~131) — AK_Control_Mode_t
CTR_MODE_ESTOP=0, CTR_MODE_CURRENT=1, CTR_MODE_VELOCITY=3,
CTR_MODE_POSITION=4, CTR_MODE_SET_ORIGIN=5      // 5 는 엣지 명령 — 펄스로만

// auto_mode (188) — AUTO 경로 선택. **write 범위가 여기서 파생된다**
AUTO_MODE_KINEMATIC=0   // 금지: ECU 가 100Hz 로 ctr_mode 를 VELOCITY 로 덮어씀
AUTO_MODE_CURRENT  =1   // write 164:16
AUTO_MODE_DIRECT   =2   // write 128:52 — ECU 미개입, bridge 가 ctr_mode 소유
AUTO_MODE_CONTROL  =3   // 금지: ECU 미구현 (motor off)
AUTO_MODE_VELOCITY =4   // write 148:16
AUTO_MODE_POSITION =5   // write 132:16

SOFT_ESTOP_ACTIVE=0, SOFT_ESTOP_RELEASE=1       // addr 189
MODE_MANUAL=0, MODE_AUTO=1                      // addr 190
DELTA_STALE = 0xFF                              // ≥25.5ms 또는 미갱신
```

**이름·범위 헬퍼 — 한 곳에서만 만든다**

```cpp
const char* AutoModeName(uint8_t am);            // "CURRENT" / "DIRECT" / ... / "NONE"(0xFF)
const char* AutoModeWriteSpan(uint8_t am);       // "164:16" / "128:52" / "-"
uint8_t     AutoModeForcedCtrMode(uint8_t am);   // 비-DIRECT 에서 ECU 가 강제하는 ctr_mode
```

> 이것들이 함수인 이유: VELOCITY/POSITION 이 추가됐을 때 실제 셀렉터는 갱신됐지만
> 여섯 군데에 흩어진 표시용 삼항식 `am == DIRECT ? "DIRECT" : "CURRENT"` 는 그대로 남았다.
> 그래서 `config auto_mode 5` 가 성공해 놓고 **"auto_mode=1(CURRENT)"** 라고 답했다 —
> 동작은 옳고 보고만 틀린, 조작자가 알아챌 방법이 없는 상태다.

### 6.2 DPC-B (`rd_register_dpc.hpp`) — 256B, **ECU 와 배치가 다르다**

| addr | 크기 | 내용 |
|---|---|---|
| 0 | 16 | DEFINE |
| **46** | **16** | SYS — `sys_state`(57, R/O) 포함. **ECU 는 16 인데 여기는 46** |
| 62 | 3 | DPC-A 센서 (UART4 중계) |
| 65 | 1 | UART2 state |
| 66 | 8 | DPC-B 센서 |
| 74 | 37 | 모터 데이터 (Dynamixel ×3) |
| 120 | 2 | `dpca_locker_en`(120), `dpca_boot_en`(121) |
| 122 | 6 | `dpcb_locker_en`(122), `boot_en`(123), `light_en`(124), `servo_cmd`(125), `mode`(126), **`sys_state_target`(127, R/W)** |
| 128 | 15 | 모터 명령 |
| 175 | 32 | DIAG |

```cpp
enum {  // DPCB_STATE_e — sys_state(57) / sys_state_target(127)
    STATE_CTRL=0, STATE_HOLD=1,
    STATE_INIT=2, STATE_DESCEND_1=3, STATE_DESCEND_2=4,
    STATE_WAIT=5,          // ★ 카메라 위치 — Orin 이 target=ASCEND_1 을 써야 진행
    STATE_ASCEND_1=6, STATE_ASCEND_2=7, STATE_FINISH=8,
    STATE_RSVD=9, STATE_ERROR=10,
};
inline bool IsDeployState(uint8_t s);      // INIT ~ FINISH
inline bool IsWritableTarget(uint8_t s);   // CTRL/HOLD/INIT/ASCEND_1 만
```

**브리지가 `sys_state_target` 에 쓰는 것은 두 번뿐이다** — 전개 시작(`INIT`)과 회수 시작
(`ASCEND_1`). 나머지는 DPC 가 스스로 밟고 브리지는 `sys_state` 로 따라간다.

> ⚠ **ECU 와 공용 매핑을 쓸 수 없다**: SYS 위치가 다르고, `hw_*` 순서가 역순이며,
> `realtime_tick` 단위가 ms 다(ECU 는 ×0.1ms). `RdTelemetry` 가 보드별로 따로 적는다.

### 6.3 PCU (`rd_register_pcu.hpp`)

DEFINE(0:16) / SYS(46:16) / POWER(96:8) / CMD(128:4) / DIAG(224:32).
**레지스터 상당 부분이 미확정**이라 `enable_pcu_read` 기본 off 이고, 발행 시
`connected` 를 뺀 전 필드가 미판독(`0xFF`/`NaN`)으로 나간다.

---

## 7. `RdClockSync` — 시계 동기 추정기

ECU 의 TIM5(10kHz free-run) tick 과 Orin epoch 을 잇는다. NTP 유사지만
wire 시간이 결정적이라 대칭 가정이 필요 없다.

```
lb = t_req  + wire_up                    ≤ t_latch
ub = t_resp - wire_down - proc_delta     ≥ t_latch
offset = t_latch - tick_s   →  [lb - tick_s, ub - tick_s] 구간 추정
```

```cpp
struct TxnTiming_t { double t_req, t_resp; size_t req_bytes, resp_bytes; bool valid; };

struct Result_t { double offset, drift_ppm, quality, wire_up, wire_down; bool valid; };
Result_t Update(const TxnTiming_t& txn, uint32_t ecu_tick, double proc_delta_prev);
double   UnwrapTickSec(uint32_t ecu_tick);      // wrap(~4.97일) 처리한 단조 [s]
```

| 상수 | 값 | 의미 |
|---|---|---|
| `kByteTime` | `10/921600` | 8N1 바이트 시간 |
| `kWindowSec` | 20.0 | 피팅 창 |
| `kQualMargin` | 2e-4 | 창 내 최소 quality + 0.2ms 이내 샘플만 채택 |
| `kMinFitSamples` / `kMinFitSpan` | 50 / 5.0s | 수렴 조건 |
| `kDiscontinuity` | 0.1s | offset 점프 한계 — 초과 시 창 리셋 (ECU 리부트/NTP 스텝) |

**추정기는 항상 돈다.** `comm_diag_enable` 은 원자료 발행만 가린다 —
`ControlFeedback.header.stamp` 가 이 결과를 쓰기 때문이다.
미수렴이면 stamp 가 Orin 수신 시각으로 fallback 하고 `stamp_valid=false` 로 알린다
(**시간축 정밀도 등급이 다른 런**이라 분석이 섞으면 안 된다).

---

## 8. `STATE_t` — 채널 상태 (`rd_common.hpp`)

```cpp
typedef union {
    uint8_t raw;
    struct { uint8_t lifecycle : 4; uint8_t health : 4; } bits;
} STATE_t;

// lifecycle: LS_INIT(0) READY(1) RUNNING(2) DEGRADED(3) RECOVERING(4) OFFLINE(15)
// health   : HC_OK(0) INFO TIMEOUT CRC_ERR FRAMING_ERR OVERRUN DATA_RANGE PROTOCOL_ERR
//            ACK_FAIL PARAM_ERR HW_FAULT BUS_WARNING BUS_PASSIVE BUS_OFF UNRECOVERABLE FATAL(15)
```

ECU 각 센서 블록의 **끝바이트**에 하나씩 박혀 있다 (로드셀 47 / IMU 69 / 엔코더 85 /
RS485 86 / RC 87 / 모터 127). 그래서 프리셋이 블록 끝을 안 덮으면 그 채널의 `lc`/`hs` 가
미판독이 된다 — `kPresetControl` 의 `static_assert` 6줄이 그것을 막는다.

> ⚠ **채널 상태 보고보다 `delta_tick`(취득 시각)이 믿을 만하다.**
> 실기에서 IMU 가 `lc=1/hs=0`("정상")을 보고하면서 데이터는 하나도 안 오는 상태가 관측됐다.
> 발행 경로는 `Reads()` → `delta_tick != DELTA_STALE` → `lifecycle != LS_OFFLINE`
> 세 조건을 **모두** 본다 (`rd_telemetry.cpp:239`).

> ⚠ `HW_BIT_*` 매크로군은 **정의만 있고 사용처가 0** 이다. 그래서 슬롯이 uart4→uart6 으로
> 옮겨졌을 때 이름이 틀린 채로 오래 남았다. 쓰기 시작할 때 그 주석부터 지울 것.
