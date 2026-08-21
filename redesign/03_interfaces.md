# 03 — 인터페이스 계약 (topic / service / action) · 시간축 통일

> 선행: [00](00_overview.md) · [01](01_modes.md) · [02](02_layers.md).
> 결정: 2026-07-26 사용자 확정 (B1~B6).
> 이 문서가 정하는 것: 메시지 패키지 구성, 전체 topic/service/action 목록, 메시지 필드,
> **ECU↔Orin 시간축 통일 설계**, 진단 인덱스 규약, 커맨드 의미 단위 스펙.
> 이 문서가 정하지 않는 것: 슬롯 구현 상세(04), 프로파일 스키마(05), 마이그레이션 순서(06).

---

## 1. 확정 결정 (B1~B6)

| # | 결정 |
|---|------|
| B1 | `CommLatency` **흡수**. 단 목적을 재정의 — 지연 계산이 아니라 **시간축 통일**. 모든 센서 값이 Orin 시간축의 취득시각을 갖는다 (§2) |
| B2 | 진단은 **배열 status 하나**로. `lc[8]`/`hs[8]`/`degraded_cnt[8]` + 비트필드 3개 (§3) |
| B3 | `project` = HANDOFF 지정 토픽명 규칙 준수 / `control` = **단일 custom 200Hz 피드백** (§4) |
| B4 | 전체 인터페이스 구성 (§5) |
| B5 | `SETPOINT` → 서비스 아님, **`cmd_motor` 토픽에 통합**. `SET_ORIGIN` → 커맨드 슬롯으로 (§6) |
| B6 | 커맨드 슬롯은 **의미 단위 명령**. raw 주소 접근은 `manual` 전용 (§7) |

---

## 2. B1 — 시간축 통일 (이번 재설계의 핵심 인터페이스)

### 2.1 목적 재정의

기존 `CommLatency`(14필드, 200Hz 별도 토픽)는 **지연 분해**가 목적이었다:
`RTT − wire_up − wire_down − proc_delta` 로 Orin USB 지연을 고립시키는 계산.

**진짜 필요한 것은 그 계산 결과가 아니라 시계 매핑이다.** ECU가 취득한 센서 값이 Orin 시간축에서
정확히 몇 시에 측정된 값인지 알아야 제어가 성립한다. 지연 분해는 그 매핑을 만드는 **수단**이었지
목적이 아니다.

→ **200Hz 피드백에는 "시각"만 싣는다. 계산 원자료는 디버그 토픽으로 내린다.**

### 2.2 시각 복원 공식

ECU는 TIM5 free-run 32bit @10kHz — `realtime_tick` = 카운터 × 0.1ms.
각 센서는 `delta_tick`(uint8, ×0.1ms, `0xFF`=stale)을 갖는다 = **발행 시점 tick − 취득 시점 tick**.

```
① ECU 도메인 취득 tick
     acquire_tick[s] = realtime_tick − delta_tick[s]

② ECU tick → Orin 시간축 (선형 매핑)
     t_orin = a · ecu_tick + b
       a = 1e-4 × (1 + drift)     ← 드리프트 보정된 tick 주기 [s/tick]
       b = offset                  ← rd_clock_sync 가 min-RTT 선별 + 선형 피팅으로 상시 추정

③ 센서 취득 절대시각
     t_sensor[s] = a · (realtime_tick − delta_tick[s]) + b
```

> **드리프트를 delta 에도 적용해야 한다**: 실측 `drift_ppm ≈ −19,600` = **−1.96%**
> (ECU 클럭이 crystal 이 아니라 HSI 내부 RC — testbed_spec §2.5). `delta_tick` 최대 255 = 공칭 25.5ms 인데
> 1.96% 오차면 **0.5ms** 로 정확도 목표(±0.5ms)를 그대로 잡아먹는다. `a` 에 드리프트를 포함시켜
> offset·delta 를 **하나의 선형식으로 일관 처리**한다.

### 2.3 방향 선택 — Orin 시간축으로 변환한다

사용자 제기: "realtime_tick 에 맞게 orin timer 를 수정하든 반대로 수정하든".

| 방향 | 장점 | 단점 |
|---|---|---|
| **(A) ECU tick → Orin ROS time** ✅ | rosbag·tf·rviz·타 노드와 호환. `header.stamp` 가 정상 동작 | 변환 오차가 개입 |
| (B) ECU tick 을 마스터로, Orin 이 따라감 | 센서 시각이 원본 그대로 | ROS time 과 어긋나 tf/bag 재생이 깨짐. ROS 생태계 이탈 |

**(A) 채택.** 단 두 가지로 (B)의 장점을 회수한다:
1. **`ecu_tick` raw 를 필드로 유지** — 분석은 이것을 권위 소스로 쓸 수 있다 (완전 복원 가능)
2. **`stamp_quality` 를 함께 발행** — 변환의 불확도를 소비자가 안다

### 2.4 `header.stamp` 의 의미를 바꾼다

| | Before | After |
|---|---|---|
| `header.stamp` | Orin **수신** 시각 (DDS 지터 포함) | **ECU 취득 기준시각을 Orin 시간축으로 변환한 값** |

이것이 B1의 실질이다. 지금은 `header.stamp` 가 "언제 메시지를 만들었나"라 제어에 쓸 수 없다.
바꾸면 **그대로 제어 시간축이 된다.**

`stamp_valid == false`(추정기 미수렴)일 때만 Orin 수신 시각으로 fallback 하고, 그 사실을 플래그로 알린다.
**조용히 틀린 시각을 주지 않는다.**

### 2.5 센서별 취득시각을 어떻게 실을 것인가

절대 시각을 센서마다 `float64` 로 실으면 15개 × 8B = 120B — 200Hz 에 과하다.
**기준시각 1개 + 센서별 상대 지연**으로 표현한다.

```
header.stamp        = a · realtime_tick + b          # 배치 기준시각
float32 dt_<sensor> = a · delta_tick[sensor]         # 기준시각 대비 얼마나 이전인가 [s]

→ 센서 절대 취득시각 = header.stamp − dt_<sensor>
```

- 단위를 `float32` 초로 두어 소비자가 raw tick 을 다시 변환할 필요가 없다 (드리프트 보정 완료 상태)
- 해당 프리셋이 읽지 않은 센서는 **`NaN`** — 0 을 넣으면 "동시 취득"으로 오해된다.
  `delta_tick == 0xFF`(stale)도 `NaN`
- 비용: 15개 × 4B = 60B

---

## 3. B2 — 진단 인덱스 규약 통일

### 3.1 인덱스가 하나로 정렬된다 (발견)

현재 진단 정보는 세 갈래로 흩어져 있는데, 인덱스 규약을 맞출 수 있다:

| idx | 통신 채널 | 서브시스템 | `hw_*` 비트 | `STATE_t` 위치 |
|-----|----------|-----------|------------|---------------|
| 0 | uart1 | RC 수신기 | `HW_BIT_UART1` | `rc.state` (87) |
| 1 | uart2 | RS485 | `HW_BIT_UART2` | `uart2.state` (86) |
| 2 | uart6 | IMU | `HW_BIT_UART4`* | `imu.state` (69) |
| 3 | can1 | 모터 ×4 | `HW_BIT_CAN1` | `motor_data.state` (127) |
| 4 | i2c1 | 엔코더 ×5 | `HW_BIT_I2C1` | `encoder.state` (85) |
| 5 | (adc) | 로드셀 | — | `loadcell.state` (47) |
| 6,7 | RSVD | | | |

\* 매크로 이름이 `HW_BIT_UART4` 지만 실제 슬롯은 uart6 (IMU) — 구 이름 잔재.
**개명 대상**: `HW_BIT_UART4` → `HW_BIT_UART6`.

> **핵심**: `degraded_cnt` 의 기존 인덱스(uart1/uart2/uart6/can1/i2c1)와 `hw_error` 비트 순서가
> **이미 같다.** 여기에 `STATE_t` 를 같은 인덱스로 배열화하면 **모든 진단이 하나의 인덱스 규약**을
> 쓴다. 토픽 30개가 배열 필드 3개로 접히는 근거다.

### 3.2 `STATE_t` 분해

`STATE_t` = 1바이트 (`lifecycle` 4bit | `health` 4bit). 배열 2개로 분해해 발행한다:

```
uint8[8] lc   # lifecycle : 0=INIT 1=READY 2=RUNNING 3=DEGRADED 4=RECOVERING 15=OFFLINE
uint8[8] hs   # health    : 0=OK 2=TIMEOUT 3=CRC 4=FRAMING 5=OVERRUN 6=DATA_RANGE
              #             7=PROTOCOL 8=ACK_FAIL 9=PARAM 10=HW_FAULT 11=BUS_WARN
              #             12=BUS_PASSIVE 13=BUS_OFF 14=UNRECOVERABLE 15=FATAL
```

`raw` 그대로 두지 않고 나누는 이유: HANDOFF 요구사항이며, 소비자가 비트 마스킹 없이 바로 읽는다.

### 3.3 접히는 규모

| Before (토픽) | 개수 | After (필드) |
|---|---|---|
| `error/degraded_cnt/{5채널}` | 5 | `uint8[8] degraded_cnt` |
| `error/hw_reset/{5}` | 5 | `uint8 hw_reset` (비트필드) |
| `error/hw_error/{5}` | 5 | `uint8 hw_error` |
| `error/hw_fatal/{5}` | 5 | `uint8 hw_fatal` |
| `{lc,hs}/{motor,encoder,rc}` | 6 | `uint8[8] lc` / `uint8[8] hs` |
| `motor/comm_err/{1..4}` | 4 | `uint8 comm_err` (2bit×4) |
| `connected` / `fsm` / `alive_time` | 3 | 필드 3개 |
| **합계** | **33** | **메시지 1개** |

---

## 4. B3 — 모드별 발행 세트

### 4.1 `project` — HANDOFF 지정 토픽명 규칙 준수

외부 팀이 쓰는 **계약 이름**이므로 변경하지 않는다 (네임스페이스 없는 언더스코어 표기).

| 방향 | 토픽 | 타입 | 주기 |
|---|---|---|---|
| Sub | `/carrier_cmd_vel` | `geometry_msgs/Twist` | — |
| Sub | `/jeongae` | `mgs01_base_msgs/JeonGae` | — |
| Pub | `/carrier_battery` | `std_msgs/UInt8` | 10Hz |
| Pub | `/carrier_imu` | `sensor_msgs/Imu` | 100Hz |

진단용(우리 팀 전용)은 별도 네임스페이스:

| 토픽 | 타입 | 주기 |
|---|---|---|
| `/carrier/ecu/status` | `mgs_tp_msgs/NodeStatus` | 20Hz |
| `/carrier/dpc/status` | `mgs_tp_msgs/NodeStatus` | 20Hz |
| `/carrier/pcu/status` | `mgs_tp_msgs/NodeStatus` | 20Hz |
| `/carrier/ecu/motor` | `mgs_tp_msgs/MotorStatus` | 20Hz |

> `/carrier/ecu/motor` 하나가 기존 `motor/{current/raw, current/filtered, pose, speed, temp, error}` 6개 +
> `motor/comm_err/{1..4}` 4개 = **10개를 대체**한다. 모터 데이터는 프로젝트 계약이 아니라 진단이므로
> 이쪽으로 옮긴다.

### 4.2 `control` — 단일 200Hz custom

| 토픽 | 타입 | 주기 |
|---|---|---|
| `/carrier/control/feedback` | `mgs_tp_msgs/ControlFeedback` | **200Hz** |
| `/carrier/control/comm_diag` | `mgs_tp_msgs/CommDiag` | 5Hz — **기본 off** |

**1 RW 트랜잭션 = 1 메시지** (R5). `comm_diag` 는 `comm_diag_enable` 파라미터(기본 `false`)로만 켠다 —
클럭 추정기 자체는 항상 돌지만 원자료 발행은 디버그 상황에서만 한다.

### 4.3 토픽 총량

| | Before | After |
|---|---|---|
| `project` | 47 | **6** |
| `control` | 47 + 2 | **1** (+디버그 1) |

---

## 5. B4 — 전체 인터페이스 구성

### 5.1 패키지 분리 (Q5)

```
mgs01_base_msgs/          # 프로젝트 전용 — 이 로봇 과제에서만 의미 있는 것
└── msg/JeonGae.msg

mgs_tp_msgs/              # 플랫폼 공용 — 다른 로봇에도 재사용 가능
├── msg/
│   ├── NodeStatus.msg          # ECU/DPC/PCU 공용 상태
│   ├── MotorStatus.msg         # 모터 4채널 진단
│   ├── ControlFeedback.msg     # 200Hz 제어 피드백 (시간축 포함)
│   ├── CommDiag.msg            # 통신 지연 원자료 (디버그)
│   └── CmdMotor.msg            # 스트림 명령 입구 (웹 슬라이더 + MPC 공용)
├── srv/
│   ├── ControlConfig.srv        # 구 TestbedConfig
│   └── CommandSet.srv           # 의미 단위 커맨드 슬롯 (§7)
└── action/
    └── RunProfile.action
```

### 5.2 전체 목록

| 종류 | 이름 | 타입 | 모드 | 비고 |
|---|---|---|---|---|
| **Sub** | `/carrier_cmd_vel` | `geometry_msgs/Twist` | project | 계약명 고정 |
| Sub | `/jeongae` | `mgs01_base_msgs/JeonGae` | project | 계약명 고정 |
| Sub | `/carrier/control/cmd_motor` | `mgs_tp_msgs/CmdMotor` | control | 웹 슬라이더 + MPC 공용 (§6) |
| **Pub** | `/carrier_battery` | `std_msgs/UInt8` | project | 10Hz, 계약명 고정 |
| Pub | `/carrier_imu` | `sensor_msgs/Imu` | project | 100Hz, 계약명 고정 |
| Pub | `/carrier/ecu/status` | `mgs_tp_msgs/NodeStatus` | project·manual | 20Hz |
| Pub | `/carrier/dpc/status` | `mgs_tp_msgs/NodeStatus` | project·manual | 20Hz |
| Pub | `/carrier/pcu/status` | `mgs_tp_msgs/NodeStatus` | project·manual | 20Hz |
| Pub | `/carrier/ecu/motor` | `mgs_tp_msgs/MotorStatus` | project·manual | 20Hz |
| Pub | `/carrier/control/feedback` | `mgs_tp_msgs/ControlFeedback` | control | **200Hz** |
| Pub | `/carrier/control/comm_diag` | `mgs_tp_msgs/CommDiag` | control | 5Hz, 기본 off |
| **Srv** | `/carrier/control/config` | `mgs_tp_msgs/ControlConfig` | control | **7 op** (§6.4) |
| Srv | `/carrier/command_set` | `mgs_tp_msgs/CommandSet` | 전 모드 | 의미 단위 (§7) |
| Srv | `/carrier/jeongae_lock` | `std_srvs/SetBool` | project | 유지 |
| **Act** | `/carrier/control/run_profile` | `mgs_tp_msgs/RunProfile` | control | 유지 |

**삭제**: `/carrier/testbed/comm_latency`(흡수), `/carrier/status`, `/carrier/{ecu,dpc,pcu}/connected`,
`/carrier/ecu/{fsm,alive_time}`, `/carrier/ecu/error/**`(20), `/carrier/ecu/{lc,hs}/**`(6),
`/carrier/ecu/motor/**`(10), `/carrier/ecu/sensor/linkage_angle`.

> **`linkage_angle` 은 `control` 전용이다 (2026-07-26 확정)**: 링크가 몸체에 대해 기울어진 각도이며
> IMU·GPS 와 함께 **global position 추정(MPC)의 입력**이다. 그 제어기는 `control` 모드에서 돌고,
> `project` 프리셋은 엔코더(70~85)를 읽지도 않는다 (01 §5.3).
> → `ControlFeedback.link_angle` 로만 나간다. `project` 모드에는 링크 각도 토픽이 **없다**.

### 5.3 메시지 정의

#### `NodeStatus.msg` (20Hz — ECU/DPC/PCU 공용)

```
std_msgs/Header header

bool     connected          # 브리지가 판정한 링크 상태
uint8    fsm                # 노드 FSM (ECU sys_state: 0=INIT 1=MANUAL 2=AUTO
                            #   3=ESTOP_SW 4=ESTOP_HW 5=FAULT)
float32  alive_time         # [s] 노드 기동 후 경과

# ── 진단 배열 — 인덱스 규약 §3.1 공통 ──────────────────────
#   0=uart1/RC  1=uart2/RS485  2=uart6/IMU  3=can1/모터  4=i2c1/엔코더
#   5=adc/로드셀  6,7=RSVD
uint8[8] lc                 # lifecycle nibble
uint8[8] hs                 # health nibble
uint8[8] degraded_cnt       # 통신 오염도 [%]
uint8    hw_error           # 비트필드 (같은 인덱스)
uint8    hw_fatal
uint8    hw_reset
```

##### 발행 규칙 — 부분 갱신 금지

`degraded_cnt`(16~23) / `hw_error`(24) / `hw_fatal`(25) 은 **`{16,17}` 을 읽어야만 얻어진다.**

→ **`NodeStatus` 는 `{16,17}` 전체를 읽은 트랜잭션에서만 발행한다. 부분 발행 금지.**

**2026-07-27 확정으로 이 제약이 사라졌다**: 모든 읽기 프리셋이 `{16,17}` 로 시작하도록 통일했다
(01 §5.3 / 04 §2.3). 초안은 `control` 프리셋이 `{26,7}` 이라 세 필드를 못 읽었는데, 10B 를 더
읽어도 최악(DIRECT) 이 1.93ms 로 예산 2.5ms 안이다.

| `bridge_mode` | `NodeStatus` 발행 |
|---|---|
| `project` / `manual` | **상시 20Hz** |
| `control` **RUNNING** | **발행 없음** — 데이터는 있지만 `control` 은 토픽 1개 계약(B3)이므로 `ControlFeedback` 에 실어 보낸다 |
| `control` **IDLE** | 좌동. 필요 시 `diag` 프리셋으로 교체 (04 §2.4.2) |

`control` 모드의 상시 진단은 `ControlFeedback` 이 담당하며, `{16,17}` 통일로 이제
**`NodeStatus` 와 동일한 내용**(`lc`/`hs`/`degraded_cnt`/`hw_*`/`sys_state`)을 200Hz 로 싣는다 —
**판정 권위는 `sys_state`** 이며, 이것이 P6(진단 stale) 의 원래 해법이었다 (01 §5.6).

> **검토했으나 채택하지 않은 안 — `fields_valid` 비트마스크**: "이번엔 안 읽었다"를 필드별로
> 표현하는 방법. 부분 발행을 허용하면 필요하지만, **부분 발행을 금지하면 불필요하다.**
> "안 읽음"과 "정상"을 구분하는 문제는 **메시지를 안 보내는 것**으로 이미 해결된다 —
> 소비자는 `header.stamp` 의 나이로 stale 을 판정한다. 필드를 늘리지 않는 쪽을 택했다.

#### `MotorStatus.msg` (20Hz)

```
std_msgs/Header header
uint8      motor_mask       # 활성 모터 비트필드 (bit0~3 = M1~M4)
float32[4] position         # [deg]
float32[4] velocity         # [RPM]
float32[4] current          # [A]
float32[4] current_filtered # [A] EMA (alpha=0.05)
int8[4]    temp             # [°C]
uint16     error_code       # 4bit×4 packed (LSB=M1) — AK 에러코드
uint8      comm_err         # 2bit×4 packed (LSB=M1): bit0=RX err, bit1=TX err
uint8      state            # STATE_t raw (4모터 중 worst)
```

> **엔코더는 여기 없다**: 링크 각도는 모터 데이터가 아니고, `project` 프리셋이 엔코더(70~85)를
> 읽지도 않는다 (01 §5.3). `control` 모드의 `ControlFeedback.link_angle` 로만 나간다.

#### `ControlFeedback.msg` (200Hz — 이번 재설계의 중심 메시지)

```
# control 모드 전용. 1 RW 트랜잭션 = 1 메시지 (보간·합성 금지).
std_msgs/Header header
  # ⚠ stamp = **ECU 취득 기준시각을 Orin 시간축으로 변환한 값** (§2.4)
  #     = a·realtime_tick + b     (Orin 수신 시각이 아니다)
  #   stamp_valid==false 일 때만 Orin 수신 시각으로 fallback

# ── 시간축 (B1) ────────────────────────────────────────────
uint32  ecu_tick            # realtime_tick raw [×0.1ms] — 분석의 권위 소스, 완전 복원용
float32 stamp_quality       # header.stamp 불확도 ± [s]  (0.0005 = ±0.5ms)
bool    stamp_valid         # 클럭 추정기 수렴 여부
float32 drift_ppm           # 현재 드리프트 추정 (참고. 실측 ≈ -19600)

# 센서별 취득 지연 [s] — 절대 취득시각 = header.stamp - dt_*
#   미판독 프리셋 / delta_tick==0xFF(stale) → NaN
float32    dt_imu
float32    dt_loadcell
float32[5] dt_encoder
float32[4] dt_motor         # CAN 피드백 수신 지연
float32[4] dt_motor_cmd     # CAN 명령 송출 지연

# ── 진단 (2026-07-27 추가) ─────────────────────────────────
# 인덱스 규약 = §3.1 (0=uart1/RC 1=uart2/RS485 2=uart6/IMU 3=can1/모터 4=i2c1/엔코더 5=로드셀)
#   프리셋이 그 채널을 읽지 않으면 **0xFF** (= "미판독"). OK(0) 와 반드시 구분한다
uint8[8] lc                 # lifecycle nibble (§3.2)
uint8[8] hs                 # health nibble    (§3.2)
uint8[8] degraded_cnt       # 통신 오염도 [%]
uint8   hw_error            # 현재 활성 에러 비트필드
uint8   hw_fatal            # 재초기화 필요 치명 에러
uint8   hw_reset            # 소프트 리셋 요구 비트필드

# ── 상태 ───────────────────────────────────────────────────
uint8   ecu_state           # ECU sys_state
uint8   control_state       # 0=INIT 1=IDLE 2=RUNNING 3=LOCKED
uint8   write_source        # 0=NONE 1=CMD_VEL 2=PROFILE 3=STREAM
uint8   auto_mode           # 현재 유효 auto_mode (write 범위의 근거)
uint8   motor_mask
uint32  goal_id             # 활성 프로파일 (0=없음) — 분석 자동 분할 키
float32 profile_time        # [s]
uint16  segment_index

# ── 명령·피드백 ────────────────────────────────────────────
float32[4] cmd              # 명령값. 단위는 auto_mode 가 결정 (A / RPM / deg)
float32[4] fb_current       # [A]
float32[4] fb_velocity      # [RPM]
float32[4] fb_position      # [deg]
int32[2]   loadcell_raw     # raw (캘리브레이션은 오프라인)
float32[4] imu_quat         # x,y,z,w — control 프리셋만, 없으면 NaN
float32[3] imu_gyro         # [rad/s]
float32[3] imu_acc          # [m/s²]
float32[5] link_angle       # [deg] linkage encoder — 링크가 몸체에 대해 기울어진 각
                            #   AS5600 12bit raw × 360/4096. control 프리셋만, 없으면 NaN
                            #   ※ IMU·GPS 와 함께 global position 추정(MPC)의 입력 (§8.2)

# ── 품질 ───────────────────────────────────────────────────
uint8   rw_err              # read_err | write_err<<4
uint32  drop_cnt            # 발행 큐 드롭 누적 (02 §6.3) — 시계열 구멍의 위치·크기
float32 rtt                 # 이번 트랜잭션 왕복 [s] — 유일하게 남기는 지연 지표
```

**약 246B / 200Hz ≈ 49KB/s.** `depth 100` 유지.

> **진단 필드를 200Hz 메시지에 넣은 근거**: `control` RUNNING 중에는 `NodeStatus` 를 발행하지
> 않으므로(B3, 토픽 1개 계약), 이것이 없으면 긴 실험 도중 어느 채널이 언제 degrade 됐는지가
> **사후에 복원 불가능**하다. 값 자체는 이미 와이어에 실려 오고 있었고(`{16,17}` + `{47,81}`),
> 발행만 안 하던 것이다 — 추가 wire 비용은 `{48,80}`→`{47,81}` 의 **1바이트**뿐이다 (04 §2.3).
> 같은 트랜잭션에서 나오므로 `header.stamp` 와 시각이 정합한다 (B1 의 "1 트랜잭션 = 1 메시지").

#### `CommDiag.msg` (5Hz — 디버그 전용, 기본 off)

```
# 지연 분해 원자료. comm_diag_enable:=true 일 때만 발행.
# 상시 필요한 시각 정보는 ControlFeedback 이 이미 갖고 있다 (§2.1).
std_msgs/Header header
float64 t_req               # 요청 write 직전 [s, epoch]
float64 t_resp              # 응답 수신 완료 [s, epoch]
float32 rtt
float32 wire_up             # 계산치 (요청 bytes × 10.85µs)
float32 wire_down
float32 proc_delta          # STM 처리시간 (직전 트랜잭션, -1=stale)
float32 residual            # rtt - wire_up - wire_down - proc_delta = Orin USB 지연
float64 clock_offset        # b
float32 tick_period         # a [s/tick] — 드리프트 반영된 실효 tick 주기
float32 drift_ppm
float32 quality             # 이번 샘플 불확도 (ub-lb) [s]
uint32  sample_cnt          # 추정기 누적 샘플
uint32  drop_cnt
```

#### `CmdMotor.msg` (스트림 입력 — 웹 슬라이더 + MPC 공용)

```
# **arm=on 일 때만** 소비 (01 §6.1.3 — 웹 run/stop). arm=off 면 구독하되 버린다.
# 스테일 > timeout → IDLE + arm 자동 off. LOCKED 는 비정상 경로 전용.
std_msgs/Header header      # 명령 생성 시각 — 스테일 판정 기준
uint8[4]   ctr_mode         # 0=ESTOP(해당 모터 송신 skip) 1=CURRENT 2=CURRENT_BRAKE
                            # 3=VELOCITY 4=POSITION.  auto_mode=DIRECT 에서만 유효
float32[4] position         # [deg]
float32[4] velocity         # [RPM]
float32[4] current          # [A]
```

---

## 6. B5 — `SETPOINT` / `SET_ORIGIN` 을 어디에 둘 것인가

### 6.1 단일 서비스 + op enum vs 서비스 분리 — 일반론

| | 단일 서비스 + `op` | 서비스 분리 |
|---|---|---|
| 클라이언트 | 타입 1개만 import | 여러 타입 import |
| **타입 안전성** | ✗ — 필드 의미가 op 마다 다름 (`motors[]`, `value` 가 op 별 재해석) | ○ — 각 서비스가 자기 필드를 가짐 |
| **자기 문서화** | ✗ — 문서 없이는 무슨 값을 넣을지 모름 | ○ — `set_motor_mask "{motors: [2,3]}"` 로 읽힌다 |
| 확장 | op 추가 = msg 재빌드 = **모든 클라이언트 재빌드** | 서비스 추가는 기존 클라이언트에 무영향 |
| 서버 코드 | 하나의 큰 `switch` | 콜백 여러 개 |

일반론으로는 **분리가 낫다.** 다만 현행 `ControlConfig` 6개 op 는 이미 실기 검증을 통과했고
CLI·테스트가 붙어 있어, 지금 쪼개는 것은 이득 대비 변경 비용이 크다 → **현행 유지**.

### 6.2 그런데 `SETPOINT` 는 서비스여선 안 된다

웹 슬라이더는 **초당 수~수십 회 연속**으로 값을 보낸다. 서비스는 요청마다 응답을 기다리므로
UI 가 버벅이고, 브리지 콜백 큐도 막힌다. **성질이 스트림이다.**

그리고 스트림이면 `/carrier/control/cmd_motor`(`CmdMotor.msg`, Task 3 예약분)와 **완전히 같다** —
차이는 발신자가 웹이냐 MPC 냐 뿐이고, 브리지 입장에서는 구분할 이유가 없다.

→ **통합한다.** 01 §6.1 의 `write_source` 가 4개에서 3개로 줄어든다:

| Before | After |
|---|---|
| `CMD_VEL` / `PROFILE` / `STREAM` / `SETPOINT` | `CMD_VEL` / `PROFILE` / **`STREAM`** |

웹 슬라이더 = `CmdMotor` 발행자. 스테일 timeout 이 곧 "슬라이더를 놓았을 때" 의 안전 복귀가 된다
(→ `IDLE` + arm off. 01 §6.1.3 에서 `LOCKED` 가 아님을 확정했다).

> **웹의 범위 (2026-07-27 확정)**: 실시간 자세 표시 → (선택) 원점 → 슬라이더 정렬 → **run/stop** →
> 조작. 그 이상의 자동화·검증·보정은 넣지 않는다. 01 §6.1.3 운용 흐름 참조.

### 6.3 `SET_ORIGIN` 은 커맨드 슬롯으로 — 단 `control` 에는 별도 경로가 필요하다

`SET_ORIGIN`(AK mode 5, 엔코더 원점)은 1회성 셋업 작업이다. `manual` 에서는 모든 조작이 커맨드
슬롯으로 이뤄지므로, 여기만 서비스 op 로 두면 일관성이 깨진다.

→ **`CommandSet` 의 의미 단위 명령 `CMD_SET_ORIGIN`** 으로 (§7).

> **정정 (2026-07-27) — "`manual` 전용" 제약 해제**: 위치 실험의 기준점을 잡는 동작이므로
> `control` 의 `IDLE` 에서도 필요하다. 그런데 `control` 은 커맨드 슬롯이 0개라(04 §2.4.1)
> `CommandSet` 경로가 없고, `ctr_mode`(128)는 `auto_mode=DIRECT` 에서 **in-span** 이라
> shadow 에 넣으면 매 tick 반복 전송된다.
> → **01 §6.3 (C-1) 의 `in-span 1회성 WRITE`(펄스)** 가 `control` 에서의 유일한 경로다.
> **자동화하지 않는다** — 조작자가 웹에서 실시간 pose 를 보고 명시적으로 누르는 동작이다.

### 6.4 결론

**`ControlConfig` 는 현행 6 op 유지 + `SET_READ_PRESET` 1개 추가 = 7 op.**
(`SETPOINT`·`SET_ORIGIN` 은 §6.2·§6.3 대로 op 로 만들지 않는다 — 추가되는 둘은 성격이 다르다:
`SET_READ_PRESET` 은 브리지 내부 상태, `SET_STREAM_ARM` 은 FSM 전이 게이트다.)

```
0=SET_ACTIVE_MOTORS  1=SET_CTR_MODE  2=SET_MODE
3=REARM              4=GET_STATUS    5=SET_AUTO_MODE
6=SET_READ_PRESET    7=SET_STREAM_ARM     ← 신규 (2026-07-27)
```

**`SET_STREAM_ARM`(7)** — 웹의 **run / stop 버튼**. `STREAM` 진입을 명시적 조작으로 만든다
(01 §6.1.3). 이것이 없으면 남아 있던 발행자가 조작자 모르게 모터를 움직인다.
`value` 0=stop / 1=run, **`IDLE` 에서만 on 가능**, 스테일 timeout 시 브리지가 자동으로 off 로 내린다.

`GET_STATUS` 는 `--json` 계약(testbed_spec §5.1 TODO ⓐ)의 데이터 원천이므로 유지하되,
응답을 정형 JSON 문자열로 바꾼다 (04 §4 에서 스키마 확정).

**`SET_READ_PRESET`(6) 을 추가하는 이유**: 04 §2.4.2 가 "정기 센서 변경 = 읽기 프리셋 교체" 로
정하면서 그 진입점이 필요해졌다. `control` 은 커맨드 슬롯이 0개라(04 §2.4.1) **이것이 정기 READ
구간을 바꾸는 유일한 경로**다.

- 인자: 프리셋 이름 문자열 (`control` / `control_test` / `diag` / `project`)
- **`IDLE` 에서만 허용.** RUNNING 중 교체는 발행 메시지의 필드 구성을 바꿔 실험 시계열을
  중간에 갈라놓는다 (R6 이 지키려던 것)
- 와이어로 나가는 write 가 없다 — **브리지 내부 상태 변경**이다. `GET_STATUS` 와 같은 성질이므로
  §6.1 의 "op 추가 = 전 클라이언트 재빌드" 비용을 한 번 더 치를 값어치가 있다고 판단
- `constexpr` 프리셋 배열의 인덱스를 atomic 으로 바꾸는 것이 전부다 (01 §3.3 write 범위 셀렉터와 동형)

---

## 7. B6 — 커맨드 슬롯의 "의미 단위 명령"

### 7.1 무엇을 묻는 질문인가

Q4 에서 **CLI 에 raw `reg` 서브커맨드를 만들지 않기로** 했다 ("대부분 상위에서 명령을 만들어서 넣을 것").
그런데 현행 `CommandSet.srv` 는 `start_addr` / `data_len` / `data[]` 를 그대로 받는다.

→ **CLI 에서 뺐으면 서비스 인터페이스도 무엇으로 바꿀 것인가?** 가 B6 이다.

### 7.2 세 가지 선택지

> ### ⚠ 정정 (2026-08-06, 09 §5.4 ②)
>
> 아래 (c) 가 정한 *"raw 는 manual 전용"* 은 **해제됐다.** 웹 TAB4 가 조작자가 주소를
> 직접 보고 만지는 **Advanced 탭**이 되면서 모드로 잠그면 탭이 성립하지 않기 때문이다.
>
> **(c) 혼합의 나머지는 그대로다** — 기본형은 여전히 의미 단위 명령이고, raw 는 여전히
> "주소를 아는 주체를 늘리는 통로" 라는 예외적 위치다. 없어진 것은 *모드로 잠그는* 부분뿐이며,
> `raw_write` 의 `safe_stop` 요구와 target 축 검증(09 §6)은 남는다.

| 안 | 내용 | 평가 |
|---|---|---|
| (a) raw 유지 | 서비스는 주소를 받고 CLI 만 안 노출 | 웹·스크립트가 주소를 알아야 함. 잘못된 주소로 쏘는 사고 경로가 남는다 |
| (b) 의미 단위 전용 | 주소 개념을 인터페이스에서 완전히 제거 | 디버깅 시 새 레지스터를 볼 방법이 없다 |
| **(c) 혼합** ✅ | 의미 단위가 기본, raw 는 `manual` 전용 | 01 §4.1 의 manual 권한 정의와 일관 |

**(c) 채택.** 주소를 아는 주체가 브리지 하나로 좁혀지고, 개발 경로도 남는다.

### 7.3 `CommandSet.srv` 재정의

```
# 커맨드 슬롯 SET/RESET. 주소 번역은 브리지가 담당한다.

# --- action ---
uint8 ACTION_RESET = 0
uint8 ACTION_SET   = 1

# --- cmd: READ 계열 (safe_stop 불필요) ---
uint8 CMD_READ_SYS     = 0    # SYS 영역 {16,17}
uint8 CMD_READ_MOTOR   = 1    # 모터 데이터 {88,40}
uint8 CMD_READ_SENSOR  = 2    # 로드셀+IMU+엔코더 {42,44}
uint8 CMD_READ_DIAG    = 3    # DEFINE+SYS+DIAG
uint8 CMD_READ_ALL     = 4    # 전 범위 {0,256}  (버퍼 확장 후 1 트랜잭션)

# --- cmd: WRITE 계열 (safe_stop 필요 — 01 §6.2) ---
uint8 CMD_SET_MOTOR_MASK = 10 # args = 모터번호 목록 (1~4)
uint8 CMD_SET_MODE       = 11 # args[0] = 0(MANUAL) / 1(AUTO)
uint8 CMD_SET_AUTO_MODE  = 12 # args[0] = auto_mode (01 §3)
uint8 CMD_SET_SOFT_ESTOP = 13 # args[0] = 0(작동) / 1(해제)   ※ 0 은 감속 방향 → 무조건 허용
uint8 CMD_SET_USE_LPF    = 14 # args[0] = 0 / 1
uint8 CMD_SET_ORIGIN     = 15 # args = 모터번호 목록. manual 전용 (§6.3)

# --- cmd: 제어 ---
uint8 CMD_REBOOT         = 20 # 대상 보드 리셋 + 3초 blackout

# --- cmd: raw (manual 전용) ---
uint8 CMD_RAW_READ       = 30 # start_addr, data_len
uint8 CMD_RAW_WRITE      = 31 # start_addr, data[]

# --- target ---
uint8 TARGET_ECU = 225        # 0xE1
uint8 TARGET_DPC = 210        # 0xD2
uint8 TARGET_PCU = 161        # 0xA1

# --- duration ---
uint16 DURATION_FOREVER = 0
uint16 DURATION_ONCE    = 1   # RET_OK 까지 반복, 2s timeout
                              # 2~100 : 지속 시간 [sec]

uint8   slot          # 슬롯 번호. 255 = 자동 배정
uint8   action        # ACTION_*
uint8   target_id     # TARGET_*
uint8   cmd           # CMD_*
int32[] args          # cmd 별 인자
uint16  start_addr    # CMD_RAW_* 전용
uint16  data_len      # CMD_RAW_READ 전용
uint8[] data          # CMD_RAW_WRITE 전용
uint16  duration      # DURATION_* 또는 2~100
---
bool   accepted
string message        # 거부 사유 (safe_stop 위반 시 위반 조건 명시 — 01 §6.2)
uint8  assigned_slot  # 자동 배정 결과
```

> **정직한 트레이드오프**: `args` 가 `int32[]` 라 §6.1 이 지적한 "필드 의미가 cmd 마다 다름" 문제를
> 그대로 갖는다. 다만 커맨드 슬롯은 본질적으로 이종 명령의 집합이고 **진단·셋업 경로**이지 주 계약이
> 아니므로 수용한다. 주 계약(`ControlConfig`, `RunProfile`, `ControlFeedback`)은 타입이 명확하다.

### 7.4 브리지의 번역 책임

```
CMD_SET_MOTOR_MASK, args=[2,3]
   → 브리지: mask = 0b0110 → (ECU, WRITE, addr 192, len 1, data=[0x06])
   → safe_stop 검사 → out-of-span 경로 (01 §6.3 C) → write_err 확인 → shadow 커밋/롤백
```

주소 상수는 `core/rd_register_ecu.hpp` 한 곳에만 존재한다. CLI·웹·스크립트 어디에도 주소가 없다.

---

## 8. 시간축 통일의 파급

### 8.1 분석 파이프라인

```python
# Before: ecu_tick 을 10kHz 로 가정 → -1.96% 드리프트로 796초 bag 에서 ~15초 오차
t = ecu_tick * 1e-4

# After: header.stamp 가 이미 보정된 Orin 시각
t = msg.header.stamp
t_imu = t - msg.dt_imu          # 센서별 실제 취득 시각
```
`analysis/latency/` 의 독립 회귀 교차검증은 유지한다 — 추정기 검증 수단으로.

### 8.2 제어 — MPC 상태추정 입력이 한 메시지에 정합된다

`ControlFeedback` 하나가 **같은 RW 트랜잭션에서 나온** 상태추정 입력을 전부 담는다:

| 입력 | 필드 | 취득시각 |
|---|---|---|
| 자세 | `imu_quat` / `imu_gyro` / `imu_acc` | `header.stamp − dt_imu` |
| **링크 각** | `link_angle[5]` | `header.stamp − dt_encoder[i]` (I2C MUX 순차 → 채널별로 다름) |
| 모터 상태 | `fb_position` / `fb_velocity` / `fb_current` | `header.stamp − dt_motor[m]` |
| 명령 반영 시각 | — | `header.stamp − dt_motor_cmd[m]` (CAN 송출) |

- **엔코더 5채널의 취득시각이 서로 다르다** — I2C MUX 를 순차 스캔하므로 채널당 지연이 붙는다.
  `dt_encoder[5]` 를 채널별 배열로 둔 이유이며, 링크 각속도를 수치미분할 때 이 차이를 보정해야 한다
- MPC 지연 보상의 입력(`dt_motor_cmd`)이 **초 단위로 직접 제공**된다. 지금은 raw `cmd_delta_tick`
  (×0.1ms, 드리프트 미보정)을 소비자가 변환해야 했다
- GPS 는 아직 없다. 추가되면 Orin 시간축에서 이미 동작하므로 `header.stamp` 기준으로 바로 융합된다

### 8.3 `stamp_valid == false` 일 때

기동 직후 추정기가 수렴하기 전 구간이다. 이때:
- `header.stamp` = Orin 수신 시각 (fallback)
- `dt_*` = **전부 `NaN`** (드리프트 미상이라 변환 불가)
- `stamp_quality` = `NaN`

**분석·제어는 `stamp_valid` 를 반드시 검사해야 한다.** 이 구간의 데이터는 시간축이 없다.

---

## 9. 04 에서 결정할 것

| # | 질문 |
|---|------|
| C1 | 슬롯 테이블 구현 — `rd_slot_table` 의 데이터 형식, 모드별 프레임 정의 방식 |
| C2 | `duration` 만료·재시도·blackout 의 tick 단위 처리 규칙 |
| C3 | `GET_STATUS` 의 JSON 스키마 확정 (testbed_spec §5.1 TODO ⓐ) |
| C4 | `control_cli` 명령 체계 — `command_cli` 흡수분 포함 |
| C5 | DPC/PCU 레지스터 확정 시점과 `NodeStatus` 매핑 |

---

## 부록: 결정 요약 카드

```
시간축 (B1)  header.stamp = a·realtime_tick + b   ← ECU 취득 기준시각을 Orin 시간축으로
             a = 1e-4×(1+drift), b = offset       ← 드리프트를 delta 에도 적용 (25.5ms×1.96%=0.5ms)
             센서 절대시각 = header.stamp - dt_<sensor>
             ecu_tick raw 유지(권위 소스) + stamp_quality(불확도) + stamp_valid(수렴)
             미판독/stale → NaN.  지연 분해 원자료는 CommDiag(5Hz, 기본 off)로 분리

진단 (B2)   인덱스 규약 통일: 0=uart1/RC 1=uart2 2=uart6/IMU 3=can1/모터 4=i2c1/엔코더 5=adc/로드셀
             lc[8] hs[8] degraded_cnt[8] + hw_error/fatal/reset 비트필드
             → 토픽 33개가 NodeStatus 1개로

토픽 (B3)   project 47 → 6   (계약명 고정: /carrier_cmd_vel /jeongae /carrier_battery /carrier_imu
                              + /carrier/{ecu,dpc,pcu}/status + /carrier/ecu/motor)
             control 49 → 1   (/carrier/control/feedback, +디버그 comm_diag)
             linkage encoder 는 control 전용 (MPC 상태추정) → ControlFeedback.link_angle
             NodeStatus 는 {16,17} 전체를 읽은 트랜잭션에서만 발행 (부분 발행 금지)
               → control RUNNING 중에는 발행 없음 (토픽 1개 계약).
                 모든 프리셋이 {16,17} 로 통일되어 ControlFeedback 이 lc/hs/degraded_cnt/hw_*
                 를 200Hz 로 싣는다. 미판독 채널은 0xFF

패키지 (B4) mgs01_base_msgs = JeonGae 만
             mgs_tp_msgs = NodeStatus MotorStatus ControlFeedback CommDiag CmdMotor
                           ControlConfig CommandSet RunProfile

SETPOINT(B5) 서비스 아님 → cmd_motor 토픽에 통합. write_source 4개 → 3개
             STREAM 진입은 명시적 arm(op 7). 스테일 → IDLE + arm off (LOCKED 아님)
             SET_ORIGIN → 커맨드 슬롯. ControlConfig 는 6 op + SET_READ_PRESET(6) = 7 op

커맨드 (B6) 의미 단위 명령 (CMD_SET_MOTOR_MASK 등) + raw 는 manual 전용
             주소 상수는 rd_register_ecu.hpp 한 곳에만 존재
```
