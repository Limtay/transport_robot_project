# Testbed Task 상세 스펙 — 입력 인터페이스·FSM·프로파일

> [TASKS.md](TASKS.md) §2(Testbed Task) TODO 의 구체화 문서 (Phase 2 산출물, 2026-07-18).
> 대상: `orin_firmware_bridge` control_mode 확장 + 신규 msg/CLI/웹. **STM(ECU_V3) 변경은 §2.5 proc_delta 레지스터 1건뿐** (⚠ CubeIDE 빌드 필요).
> 구현 시 이 문서가 스펙의 진실 원천 — 작업 로그는 [Code_modify.md](Code_modify.md)에 append.

## 0. 설계 결정 요약 (토론 확정, 2026-07-18)

| # | 결정 | 근거 |
|---|------|------|
| D1 | 프로파일 재생기(player)는 **bridge 내장** — 200Hz tick 안에서 배열 보간 | DDS 지터·스테일 원천 제거, 실험 재현성. CLI/AI/웹은 200Hz 스트림을 직접 만들 수 없음 |
| D2 | 외부 입력은 **action(프로파일) + service(단발 설정)**, 스트림 토픽은 입력 경로에서 제외 | 단발은 응답(성공/실패) 필요 → service. 장기 작업은 진행·취소 필요 → action |
| D3 | `/carrier/cmd_current` **폐기** → Task 3용 typed 스트림 `/carrier/cmd_motor` 로 재정의(예약) | Float32MultiArray 는 무정형. 테스트베드는 스트림 입력 자체가 불필요 |
| D4 | 피드백 토픽 **전면 개편**: TractionTest → `TestbedFeedback.msg` (goal_id 태그 포함) | 분석 파이프라인이 bag 을 실험 단위로 자동 분할 → AI 자동화 기반 |
| D5 | 프로파일 기술 포맷 = **YAML 파일** | 사람·AI 모두 작성 용이, bag 과 함께 보관 시 실험 기록이 됨 |
| D6 | **Single Writer 원칙**: control_mode 에서 wire 접근은 200Hz RW 가 유일. 모든 입력은 bridge shadow 만 수정 | 단발 입력 패킷 삽입 문제(타이밍 충돌)의 원천 제거 |

## 1. 아키텍처

```
[입력자]   웹 UI ──┐
           CLI  ──┼──→  Action  /carrier/testbed/run_profile   (프로파일 재생)
           AI   ──┘     Service /carrier/testbed/config        (단발 설정 + 검증 응답)
                              │  (bridge shadow 상태만 수정)
[bridge]              테스트베드 FSM → write 소스 선택 → 200Hz RW ──RS485──→ ECU
                              │
[관측]                Topic /carrier/testbed/feedback (200Hz)
                        └→ 웹 시각화 / rosbag / AI 모니터링 공용
```

## 2. 테스트베드 FSM (bridge 내부)

RW 자체는 **모든 상태에서 계속 돈다** (read 스트림 = 피드백 발행 유지). 상태는 **write 소스**만 결정한다.

| 상태        | write 값                 | 진입                        | 이탈                                   | 허용 입력                                 |
| --------- | ----------------------- | ------------------------- | ------------------------------------ | ------------------------------------- |
| `INIT`    | (루프 시작 전)               | 노드 기동                     | motor_mask 검증 성공 → IDLE / 실패 → 노드 종료 | —                                     |
| `IDLE`    | **0A 고정**               | 기본 상태                     | run_profile 수락 → RUNNING             | config service 전부 허용                  |
| `RUNNING` | profile(t)              | 프로파일 goal 수락              | 완료/취소/에러 → IDLE                      | **config 거부** (abort=action cancel 만) |
| `STREAM`  | 최신 `/carrier/cmd_motor` | (Task 3) 스트림 fresh        | 스테일 > timeout → **LOCKED**           | config 거부                             |
| `LOCKED`  | **0A 래치**               | 스트림 스테일, RW write 연속 거부 등 | **config `REARM` 명시 호출** → IDLE      | REARM 만                               |

- LOCKED 가 "모터락" 아이디어의 구현: 스테일 시 0A 로 조용히 자동 복귀하지 않고, 원인 확인 후 명시적 재무장을 요구.
- ECU 쪽 안전장치(mtr_lock: AUTO 상태에서만 모터 write 수락, AUTO_TIMEOUT 100ms)는 그대로 최후방 방어선으로 유지 — bridge FSM 은 그 앞단.

### 단발 입력의 두 경로 (메모 "IDLE 에서 read 만"의 확정판)

| 대상 레지스터                                                      | 방법                                                                               | 허용 상태                                              |
| ------------------------------------------------------------ | -------------------------------------------------------------------------------- | -------------------------------------------------- |
| RW write 범위 **안** (CMD_MOTOR 128–179: ctr_mode, pos/vel/cur) | service 가 shadow 수정 → **다음 RW tick 이 자연 반영** + 같은 트랜잭션 read-back 으로 검증. 패킷 삽입 없음 | 정책상 IDLE 만 (기술적으론 언제든 가능)                          |
| RW write 범위 **밖** (motor_mask 192, soft_estop 189, DEFINE 등) | **RW 1 tick 을 일반 WRITE 패킷으로 대체** — IDLE 은 write 값이 0A 라 1 tick 대체 무해             | **IDLE 만** (RUNNING 중 tick 훔치기 금지 → service 거부 응답) |

검증 규칙: service 응답은 read-back 확인 후 반환. **10 tick(50ms) 내 미확인 → 실패 응답** (재시도는 호출자 몫).

## 2.5 시간 동기·지연 계측 (최우선 선행 작업 — §6 #0)

목적: ① Orin 토픽 수신 → STM 모터 CAN TX 까지의 명령 지연, ② STM 센서 취득 → Orin 수신까지의
상행 지연을 정량화하고, ECU tick(10kHz) ↔ Orin 시계 매핑(offset+drift)을 상시 추정한다.
결과는 분석 시간축(ECU tick 마스터)과 이후 MPC 지연 보상 파라미터가 된다.

### 타임라인 모델 (측정 / 계산 / 특성화 구분)

```
t_topic (Orin 측정)  토픽 수신
t_req   (Orin 측정)  시리얼 write 직전
   │  wire_up   = 요청 bytes × 10.85µs (계산: 921600bps 결정적) + Orin USB 송신 지연 (특성화)
t_rx  ≈ realtime_tick (STM 기존 — rs485Task 기상 직후 latch, 오차 < 0.1ms 분해능)
   │  proc_delta = t_tx − t_rx (STM 측정 — 신규)
t_tx    (STM 신규)   응답 TX 시작 직전 latch
   │  wire_down = 응답 bytes × 10.85µs (계산) + Orin USB 수신 지연 (latency_timer≈1ms, 특성화)
t_resp  (Orin 측정)  응답 수신 완료
```

- **잔차 고립**: `RTT − wire_up − wire_down − proc_delta` = Orin USB/드라이버 지연 — 유일한 미지수로
  고립됨 (대칭 가정 불필요). min-RTT 샘플 선별 + 선형 피팅으로 offset·drift(ppm) 상시 추정.
- **명령 경로 분해**: 하행 E2E = t_topic→t_req(200Hz tick 양자화) + wire + reg 반영→CAN TX
  (`cmd_delta_tick` 기존 레지스터, controlTask 200Hz 양자화). 이론 예상 ~1–11ms — 분포 실측이 목표.
- **STM 변경 (유일)** ⚠ CubeIDE 빌드 필요: 응답 TX 직전 tick latch → `proc_delta`(uint8, ×0.1ms)를
  DIAG 레지스터에 저장 — **직전 트랜잭션 값 보고 방식** (스냅샷이 처리 전에 찍히므로 이번 응답엔 못 실음.
  200Hz 스트림에서 Orin 이 소급 매칭). 프로토콜 변경 없음.
- **TestbedFeedback 연동**: `rtt`, `clock_offset_est`(+유효 플래그) 필드 (§3.4) — 분석은 ECU tick 을
  마스터 시계로 사용해 DDS/시리얼 지터를 시간축에서 제거.
- **특성화 실험 1회**: control_mode 스텝 명령 반복 → 구간별 지연 분포(평균/p99/지터) + 장시간 offset
  drift 리포트. 본 실험 전 200Hz 루프 건전성 검증 겸함.
- 정확도 목표: **±0.5ms** (tick 분해능 0.1ms 가 하한, MPC 5ms 주기 대비 충분).
- **드리프트 실측 (2026-07-21 확정, V1 796s bag)**: `drift_ppm ≈ -19,600` = **-1.96%** — "수십 ppm" 이 아니라 **% 스케일**이다. 원인은 ECU 시계가 crystal 이 아니라 **HSI(내부 RC, PLL 소스, `main.c` RCC 확인)** 이기 때문(HSI 정확도 ±1~2%). 추정기 출력과 독립 회귀 교차검증(`t_resp = a·ecu_tick + b`, `analysis/latency`)이 -19,579 / -19,211 ppm 으로 일치 → 추정기 정상, 하드웨어 특성. **함의**: ① 기능 결함 아님(offset 추정이 이 drift 를 흡수), ② 분석·MPC 는 **raw ecu_tick 을 10kHz 로 가정 금지**, 반드시 보정된 `clock_offset` 으로 Orin 시각에 매핑. ③ crystal(HSE) 전환은 선택적 개선(보드에 HSE 실장 시) — 소프트웨어 보정으로 충분하므로 낮은 우선순위.

## 2.6 auto_mode ↔ write 범위 연동 (2026-07-19 확정 — C-8)

**배경 (실제 버그)**: `auto_mode`(addr188) 기본값 0=KINEMATIC 이면 ECU `ACTION_STATE_AUTO` 가 100Hz 로
`reg.cmd_motor.ctr_mode[i] = MODE_VELOCITY` 를 덮어쓴다 (`rd_system.c:195`). bridge 는 200Hz 로
CURRENT 를 쓰므로 **어느 쪽도 이기지 못하는 경쟁** — controlTask 가 읽는 값이 tick 마다 1/3 을 오가
전류 실험이 무의미해진다. 초기 구현이 매 tick 52B 전체를 쓴 것은 이 덮어쓰기에 맞서려던 것이었으나,
올바른 해법은 **덮어쓰기의 원인(auto_mode)을 제거**하는 것이다.

**원칙**: write 범위는 독립 설정이 아니라 **auto_mode 에서 파생**된다 —
"bridge 가 책임지는 영역 = ECU 가 건드리지 않는 영역".

| auto_mode | ECU 동작 | bridge write 범위 | 대역폭(5ms 중) |
|-----------|---------|------------------|---------------|
| **1 = CURRENT** (테스트베드 기본) | ctr_mode 를 MODE_CURRENT 로 강제 (자가치유) | **164:16** (cmd_current 만) | ~1.39ms (27.8%) |
| **2 = DIRECT** (확장 — TEST4 속도 실험 등) | 무가공 통과 (ECU 미개입) | **128:52** (전체, bridge 가 ctr_mode 소유) | ~1.78ms (35.6%) |
| 0 = KINEMATIC | ctr_mode 덮어씀 | **금지** — SET_AUTO_MODE 거부 | — |
| 3 = CONTROL | motor off (ECU 미구현) | **금지** | — |

**IDLE 에서도 write 는 계속 나간다** (0A): ECU `AUTO_TIMEOUT`(100ms) 워치독이
`reg.diag.cmd_write_tick` 미갱신 시 motor_on=0 으로 내리기 때문 (`rd_system.c:220`).
164:16 은 워치독 판정 범위(132~187)에 포함되므로 축소해도 갱신은 유지된다.

**구현 제약 4건**:
0. **shadow 소유권 원칙 (2026-07-19 C-8 하네스 발견 — 위 원칙의 읽기 쪽 대칭)**:
   **shadow 에는 bridge 가 소유한 것(= write 범위 안)만 쓴다.** write 범위 밖의 shadow 자리
   (auto_mode=1 에서의 `ctr_mode` 128~131)에는 read 세그 `{128,4}` 로 들어온 **ECU 실값**이 들어 있으며,
   이것이 §2.6-3 가드와 read-back 검증의 유일한 진실 원천이다. 매 tick 낙관적으로 덮어쓰면
   **ECU 가 VELOCITY 인데 가드가 CURRENT 로 보고 goal 을 통과시키는** 실패가 발생한다 (하네스에서 산발 재현).
   → `PrepareControlCommand` 는 auto_mode=1 이면 `cmd_current` 만, DIRECT 에서만 ctr_mode/pos/vel 까지 쓴다.
1. **범위 전환 = 선택, 변경 아님**: `task_control_current_` / `task_control_direct_` 두 개를 생성자에서
   미리 만들고 매 tick atomic 셀렉터로 고른다. 런타임 구조체 변경 금지 (200Hz 스레드와 레이스).
2. **1→2 전환 시 shadow 소독 선행**: ① shadow 의 ctr_mode=CURRENT / cmd_position=0 / cmd_velocity=0
   → ② auto_mode=2 WRITE+검증 → ③ 범위 확장. 소독 없이 확장하면 shadow 잔재(과거 값·read 유입)가
   그대로 ECU 로 나간다. 2→1 은 ① auto_mode=1 WRITE+검증 → ② 범위 축소.
3. **프로파일 재생 가드**: goal 수락 시 **활성 모터 전부 ctr_mode==CURRENT** 확인, 아니면 reject
   (v1 player 는 CURRENT 전용 — §7-0ⓑ). DIRECT+VELOCITY 조합에서 프로파일이 조용히 무효화되는
   실패를 막는다. 최신 ctr_mode 는 RW read 세그 `{128,4}`(C-5)가 이미 보유.

## 3. 인터페이스 스펙

### 3.1 기동 파라미터 (control_mode 전용)

```
ros2 run orin_firmware_bridge comm_test_node --ros-args \
  -p control_mode:=true -p active_motors:="[2,3]"
```

- `active_motors` (int 리스트, 모터 번호 1~4): 기본값 `[1,2,3,4]`, 테스트베드에서는 명시 권장.
  - **운용 불변식 (2026-07-21 실기 확정 — V2 블로커)**: `active_motors` 는 **실제로 전원·CAN 이 살아 있는 모터와 정확히 일치**해야 한다. AUTO 진입 시 ECU 모드전환 안전장치가 `ACTION_STATE_ESTOP_SW` 를 강제 경유하며 그 제동 프레임을 마스크된 모터에 CAN 송신하는데(`CAN_AK_ESTOP`), 부재 모터가 마스크에 포함되면 ACK 실패 → CAN fatal → **FAULT(sticky)** 로 기동이 막힌다. 제동 TX 는 마스크를 존중하므로(masked-out skip, `rd_can_motor.c:100`), 마스크를 실연결 모터에만 맞추면 무FAULT AUTO 진입. 예: 모터 2번만 연결 시 `active_motors:="[2]"`. `ALL_READY` 신선도 게이트는 ESTOP 경로 TX 를 막지 않으므로 마스크 일치가 유일한 방어다.
- INIT 플로우: RS485 개통 → ① `motor_mask`(addr192) WRITE+검증 → ② **`auto_mode`(addr188)=1(CURRENT) WRITE+검증** → ③ `mode`(addr190)=1(AUTO) WRITE+검증 → 성공 시 IDLE. 각 단계 **0.2s 간격 10회 재시도, 전부 실패 시 노드 종료** (exit ≠ 0).
  - **순서 필수**: auto_mode 를 AUTO 진입 **전에** 확정해야 KINEMATIC 이 활성인 순간(ctr_mode 덮어쓰기)을 한 tick 도 거치지 않는다 — motor_mask 를 AUTO 전에 쓰는 것과 같은 논리 (§2.6).
  - 기동 파라미터 `auto_mode`(기본 1) 로 DIRECT(2) 시작도 가능. 0·3 은 기동 거부.
  - AUTO 진입 근거: `reg.cmd_system.mode` 가 모드의 단일 진실원천 — GPIO 스위치·Orin write 양쪽이 갱신 (`rd_system.c:26`). RC 없이 레지스터 write 로 AUTO 진입 가능 (ECU 기구현).
- 런타임 변경은 §3.3 config service (`SET_ACTIVE_MOTORS`, `SET_MODE`) — 웹/앱 대응.

### 3.2 Action `/carrier/testbed/run_profile` (`mgs01_base_msgs/action/RunProfile`)

```
# Goal
string name           # 실험 라벨 (피드백 goal_id 와 함께 기록)
string profile_yaml   # 프로파일 YAML 전문 (§4) — 파싱·검증은 bridge
---
# Result
bool   success
string message        # 거부/중단 사유 (YAML 오류 위치 포함)
uint32 goal_id        # 피드백 토픽에 태그된 id
uint32 ticks_executed
uint32 write_err_cnt  # RW write 거부 횟수
uint32 clamp_cnt      # 클램프 발동 횟수
---
# Feedback (5Hz)
float32 t             # 프로파일 경과 [s]
uint16  segment_index
float32 progress      # 0~1
```

- goal 수락 시: YAML 파싱 → 검증(§4.3) → 전 구간 **사전 샘플링(200Hz 배열)** → RUNNING 전이. 검증 실패 시 즉시 reject.
- cancel → 즉시 0A → IDLE (result: success=false, "canceled").
- 동시 goal 불가 — RUNNING 중 새 goal 은 reject (기존 실험 오염 방지).

### 3.3 Service `/carrier/testbed/config` (`mgs01_base_msgs/srv/TestbedConfig`)

```
# Request
uint8   op        # 0=SET_ACTIVE_MOTORS / 1=SET_CTR_MODE / 2=SET_MODE / 3=REARM / 4=GET_STATUS / 5=SET_AUTO_MODE
uint8[] motors    # 대상 모터 번호 (1~4)
int32   value     # op 별 의미 (ctr_mode 값 / mode 0·1)
---
bool   ok
string message    # read-back 결과 or 실패 사유 ("RUNNING 중 거부" 포함)
```

- `SET_ACTIVE_MOTORS`: motor_mask 재기록 (out-of-span 경로, IDLE 만).
- `SET_CTR_MODE`: 모터별 제어 모드 (in-span 경로). 비활성 모터 지정 시 거부 — 메모의 "비활성 모터는 체크 불가" 규칙.
  - **auto_mode 의존 (2026-07-19 확정)**: `auto_mode=1(CURRENT)` 에서는 **사유를 명시해 거부**한다.
    그 모드에선 ctr_mode 가 ECU 소유(100Hz 강제)이고 bridge write 범위(164:16) 밖이라 wire 로 나가지도 않는다 —
    조용히 10 tick 타임아웃되면 원인 불명의 실패가 되므로, "DIRECT 로 전환 후 사용" 안내와 함께 즉시 거부.
    실제 동작은 `auto_mode=2(DIRECT)` 에서만 유효.
  - 허용 값 (`AK_Control_Mode_t`, can_ak.h): **1=CURRENT(±60A) / 2=CURRENT_BRAKE(0~60A) / 3=VELOCITY(RPM) / 4=POSITION(deg)** + 0=ESTOP(해당 모터만 송신 skip — 마스크 변경 없는 모터별 임시 정지 용). 5~7(SET_ORIGIN/POS_VEL/MIT)은 거부.
- `SET_MODE`: ECU AUTO(1)/MANUAL(0) 전환 (addr190, out-of-span 경로, IDLE 만).
- `SET_AUTO_MODE`: **1=CURRENT / 2=DIRECT 만 허용** (0·3 거부). addr188 out-of-span 경로, IDLE 만.
  성공 시 write 범위가 §2.6 표대로 자동 전환된다 (1→2 는 shadow 소독 선행 — §2.6 제약 2).
  응답 message 에 전환 후 write 범위를 명시할 것 (예: `auto_mode=2(DIRECT), write 128:52`).
- `REARM`: LOCKED → IDLE.

### 3.4 Topic `/carrier/testbed/feedback` (`mgs01_base_msgs/msg/TestbedFeedback`, 200Hz, depth 100)

```
std_msgs/Header header
uint32  ecu_tick_ms       # ECU realtime_tick 기반 (기존 TractionTest 동일)
uint8   sys_state         # ECU FSM
uint8   testbed_state     # bridge FSM (IDLE/RUNNING/STREAM/LOCKED)
uint8   motor_mask
uint32  goal_id           # 활성 프로파일 (0 = 없음) ← 분석 자동 분할 키
                          #   채번: 세션 내 단조증가 seq (1~). 전역 유일성은 bag 폴더명이 담당
                          #   → 실험의 전역 키 = (폴더명, goal_id) 쌍
float32 profile_time      # 프로파일 경과 [s]
uint16  segment_index
float32[4] cmd_current
float32[4] fb_current
float32[4] fb_velocity
float32[4] fb_position
int32[2]   loadcell_raw
uint8   rw_err            # 최근 RW 에러 니블 (read_err | write_err<<4)
float32 rtt               # 이번 트랜잭션 왕복 시간 [s] (§2.5)
float64 clock_offset      # ECU tick → Orin 시각 offset 추정치 [s] (§2.5)
bool    clock_offset_valid # 추정기 수렴 여부
```

기존 `/carrier/ecu/traction_test` + `TractionTest.msg` 는 폐기. 분석 파이프라인은 구버전 bag 분기 유지(§6).

### 3.5 (Task 3 예약) Topic `/carrier/cmd_motor` (`mgs01_base_msgs/msg/CmdMotor`)

```
std_msgs/Header header
uint8[4]   ctr_mode
float32[4] position
float32[4] velocity
float32[4] current
```

MPC 등 순수 온라인 제어기용 스트림 입구. STREAM 상태에서만 소비, 스테일 → LOCKED. **이번 구현 범위 밖** — 인터페이스만 정의.

## 4. 프로파일 YAML 스키마

### 4.1 구조

```yaml
name: hysteresis_ramp_v1        # 실험 라벨
description: p2 위치, 20kg, 상승-홀드-하강 히스테리시스   # 자유 기술
seed: 42                        # random 계열 재현성 (미지정 시 시각 기반, result 에 기록)
limits:
  max_current: 25.0             # 이 프로파일의 클램프 (전역 cmd_current_max 와 min 적용)
  slew_rate: 50.0               # [A/s] 옵션 — custom/노이즈 안전용
motors:
  m2:                           # 모터 번호. active_motors 에 없으면 goal 거부
                                # 샘플 값의 단위 = 해당 모터의 ctr_mode 를 따름 (CURRENT=A / VELOCITY=RPM / POSITION=deg)
    - {type: hold, duration: 3.0, value: 0}          # 무부하 영점 구간 (분석 tare 용)
    - {type: ramp, duration: 20.0, from: 0, to: 10}
    - {type: hold, duration: 3.0, value: 10}
    - {type: ramp, duration: 20.0, from: 10, to: 0}
  m3:
    - {type: hold, duration: 46.0, value: 0}
```

### 4.2 세그먼트 타입 (초기 셋 — "다양할수록 좋다" 방침, Phase 3 에서 보강)

| type | 파라미터 | 용도 |
|------|---------|------|
| `hold` | value, duration | 정지·유지·영점 구간 |
| `ramp` | from, to, duration | 상승/하강 (히스테리시스, 기존 RC 램프 대체) |
| `stair` | values[], step_duration | 계단 (데드밴드 탐색, 정상상태 포인트) |
| `step` | from, to, t_step, duration | 단일 스텝 (동특성) |
| `sine` | amp, freq, offset, duration | 주기 응답 |
| `chirp` | amp, f0, f1, offset, duration | 주파수 스윕 (레이트 의존성) |
| `prbs` | low, high, bit_duration, duration | 의사랜덤 이진 (시스템 식별) |
| `noise` | mean, std, duration | 시드 기반 랜덤 (slew_rate 필수) |
| `custom` | samples[], rate(기본 200) | 임의 파형 — **웹 그래프 드로잉 → 샘플 배열 변환 경로** |

### 4.3 검증·안전 규칙 (goal 수락 시)

1. 지정 모터 ⊆ active_motors, 아니면 reject.
2. 미지정 활성 모터 = 전 구간 0A. 모터별 총 길이 불일치 시 짧은 쪽 끝을 0A hold 로 패딩.
3. 전 샘플 클램프: |I| ≤ min(profile limits.max_current, `cmd_current_max`).
4. slew_rate 지정 시 위반 샘플 자동 성형이 아니라 **reject** (프로파일 = 실험 기록이므로 몰래 수정 금지).
5. 사전 샘플링 완료 후에만 RUNNING 전이 — 재생 중 파싱·검증 연산 없음 (200Hz 예산 보호).

## 5. CLI / 웹 / AI 자동화

### 5.1 `testbed_cli` — 신규 **ament_python** 패키지 (2026-07-19 상세 확정)

**형식: 원샷 명령어** (REPL 아님). 기존 `command_cli`(C++ REPL, 대화형 디버깅용)와 용도가 다르다 —
이쪽은 **AI 자동화(test_plan §3)의 실행 수단**이라 한 번 실행 → 종료코드 반환이 기본 계약이다.

**현재 구현 (2026-07-21 실기 확정 — cli.py 실측)**: 인자는 **정수 코드**만 받는다. 단어형(`current`/`auto`)·`--json`·`--timeout`·`--label` 은 아래 "미구현" 참조.

```bash
testbed_cli status                         # 서비스 message 원문을 텍스트로 출력 (--json 아직 없음)
testbed_cli config motors 2                # SET_ACTIVE_MOTORS (모터번호 나열)
testbed_cli config ctr_mode 2 1            # SET_CTR_MODE — <모터...> <mode>; ESTOP=0/CURRENT=1/VELOCITY=3
testbed_cli config auto_mode 2             # SET_AUTO_MODE — 1=CURRENT / 2=DIRECT (§2.6)
testbed_cli config mode 1                  # SET_MODE — 0=MANUAL / 1=AUTO
testbed_cli rearm                          # LOCKED 해제
testbed_cli run <profile.yaml> [--record] [--name NAME] [--bag-dir DIR]
testbed_cli abort                          # 실행 중 goal 취소 (별도 셸에서)
```

> **미구현 (AI 자동화 마일스톤 TODO)**: 아래 세 항목은 설계 계약으로 **유지**하되 현재 cli.py 에 없다 —
> AI 자동화 스크립트를 이들에 의존해 작성하지 말 것. ⓐ `--json`(결과 JSON 1건 출력) — AI 파싱 계약이라
> 삭제하지 않음, 구현 필요. ⓑ 단어형 인자(`current`/`direct`/`auto`/`manual`) — 사람 가독성용, 정수 코드와
> 병행 허용으로 추후 추가. ⓒ `run --timeout SEC` (기본 = 프로파일 길이 + 30s) — 종료코드 5 경로. 현재
> `run` 옵션은 `--record` / `--name`(라벨, §5.2) / `--bag-dir`(기록 루트) 뿐이며 `--label` 은 없다(→`--name`).

**`run` 동작 순서** (--record 시):
1. `status` 확인 — IDLE 아니면 즉시 실패 (exit 2)
2. **bag 녹화 시작** → 토픽 구독 확인까지 대기(≤2s)
   — goal 보다 먼저 떠야 프로파일 첫 hold(분석 tare 창)가 온전히 잡힌다
3. YAML 읽어 goal 전송 → accept 확인
4. 5Hz 피드백으로 진행률 출력 (`--json` 시 억제)
5. result 수신 → **+1s 여유 후 bag 종료** (마지막 tick 의 피드백 유실 방지)
6. 폴더에 `profile.yaml` 사본 + `result.json` 기록 (§5.3)

**Ctrl+C 안전 규칙 (필수)**: `run` 중 SIGINT 를 받으면 **프로세스를 즉시 죽이지 말고**
① action cancel 전송 → ② result 수신 대기(≤3s) → ③ bag 정상 종료 → ④ exit 3.
CLI 만 죽으면 bridge 의 goal 은 계속 재생되어 **모터에 전류가 계속 나간다** — 이 경로가 안전 구멍이다.
2회 연속 SIGINT 시에는 즉시 종료하되 "goal 이 계속 돌 수 있음 — `testbed_cli abort` 실행" 경고 출력.

**종료 코드** (AI 분기용):

| 코드 | 의미 |
|------|------|
| 0 | 성공 (run 완주 / 명령 수락) |
| 1 | 사용법·파일 오류 (인자, YAML 파일 없음) |
| 2 | 거부 — goal reject(YAML 검증 실패·ctr_mode 불일치·IDLE 아님) 또는 service ok=false |
| 3 | 중단 — Ctrl+C / abort / 재생 중 LOCKED 전이 |
| 4 | bridge 미실행 (action/service 서버 없음, 5s 탐색 실패) |
| 5 | 타임아웃 (`--timeout`, 기본 = 프로파일 길이 + 30s) |

**`--json`** (⚠ 미구현 — 위 TODO ⓐ): 사람용 텍스트 대신 결과 JSON 1건만 stdout 에 출력 (진행률·장식 억제).
AI 는 이것만 파싱한다 — 텍스트 정규식 파싱 금지. 구현 시 `status --json` 목표 스키마:

```json
{"testbed_state":"IDLE","motor_mask":6,"active_motors":[2,3],"auto_mode":"current",
 "write_span":"164:16","goal_id":0,"ecu_sys_state":2,"clock_offset_valid":true,
 "rtt_ms":1.7,"lock_reason":null}
```

> 현재는 `status` 가 config 서비스의 `message` 문자열을 그대로 출력한다(정형 JSON 아님). AI 자동화 착수 전
> `--json` 구현이 선행 조건이다 (텍스트 파싱을 시키지 않는다는 §5.1 원칙 준수).

### 5.2 기록 폴더 규격 — **CLI·웹 공통 계약** (재구현 대상)

`--record` 산출물. **실험 1회 = 폴더 1개 = 자기완결 기록**이며, 웹 UI(#9)도 동일 규격을 만들어야 한다
(분석 파이프라인·AI 자동화가 이 구조를 전제로 동작).

```
data/rosbags/<label>_<YYYY-MM-DD_HHMMSS>/
  bag/            # ros2 bag (metadata.yaml + *.db3)
  profile.yaml    # 제출한 YAML 원본 사본 (즉석 수정본도 그대로)
  result.json     # 아래 스키마
  console.log     # CLI stdout 사본 (선택)
```

- `label` = `--label` 인자 > YAML `name:` > `run` (우선순위).
- **녹화 토픽**: `/carrier/testbed/feedback`, `/carrier/testbed/comm_latency` (2개 고정).
- `result.json` 필수 키: `label`, `goal_id`, `success`, `message`, `ticks_executed`,
  `write_err_cnt`, `clamp_cnt`, `seed`(프로파일 실효 시드), `started_at`/`ended_at`(ISO8601),
  `profile_sha256`, `node_params`(active_motors·auto_mode·cmd_current_max), `bag_dir`.

### 5.3 웹 UI (신규 패키지 `testbed_web`, 작업 #9 — 요구사항 수준, 상세 설계는 CLI 실기 검증 후)

- rosbridge(웹소켓) 기반 별도 노드 — bridge 와 완전 분리, action/service/feedback 만 사용 (CLI 와 동일 진입점).
- 기능: 상태 대시보드(피드백 실시간 플롯) / config 버튼(비활성 모터는 UI 에서 체크 불가) / 프로파일 편집기(세그먼트 폼 + **그래프 드로잉 → custom 샘플**) → YAML 생성·제출 / run·abort.
- **기록은 §5.2 규격 준수 필수** — 폴더 구조·result.json 스키마가 CLI 와 달라지면 분석·AI 자동화가 깨진다.

## 6. 구현 작업 분해 (진행 현황 — 2026-07-21 전면 갱신)

> **상태**: #0~#9 **완료 (실기 검증 포함)**. 실기 검증 V0~V6 완주(2026-07-20~21, 이력: [Code_modify.md](Code_modify.md)),
> Stage 0(시간동기)·Stage 1(로드셀 캘리) 완료. 잔여: **#10 웹 / #11 리팩터링 / #12 CLI 보강**.
> 다음 작업 지시는 [HANDOFF_260721.md](HANDOFF_260721.md).

| # | 작업 | 상태 |
|---|------|------|
| 0 | 시간 동기·지연 계측 (§2.5): STM proc_delta(228) + `rd_clock_sync` + `CommLatency.msg` | ✅ 실기 검증 (V1, drift 실측 §2.5) |
| 1 | `mgs01_base_msgs`: `TestbedFeedback.msg` / `RunProfile.action` / `TestbedConfig.srv` / `CmdMotor.msg`(예약) | ✅ |
| 2 | bridge: 테스트베드 FSM + write 소스 선택 + LOCKED 래치 | ✅ 실기 검증 (V2~V5) |
| 3 | bridge: `active_motors` INIT 플로우 (mask→auto_mode→mode 3단계 검증) | ✅ 실기 검증 (V2, §3.1 불변식 준수 필수) |
| 4 | bridge: 프로파일 player (`rd_profile`, 세그 9종 + 사전 샘플링) + action 서버 | ✅ 실기 검증 (V4·V5) |
| 5 | bridge: config service (in-span shadow / out-of-span IDLE tick 대체 + read-back) | ✅ 실기 검증 (V3, 단 DIRECT 전환 크래시 이월 — 아래) |
| 6 | bridge: `TestbedFeedback` 발행 개편, `/carrier/cmd_current`·`TractionTest.msg` 폐기 | ✅ |
| 7 | auto_mode ↔ write 범위 연동 (§2.6), KINEMATIC 덮어쓰기 버그 수정 | ✅ (CURRENT 경로 실기 검증) |
| 7.5 | 하네스 레포 이관 — `orin_firmware_bridge/test/` colcon test 65케이스 | ✅ (신규 테스트는 반드시 이 디렉터리에) |
| 8 | `testbed_cli` (원샷 CLI §5.1) | ✅ 실기 검증 (V5 `--record` 폴더 규격 충족). 미구현분은 #12 |
| 9 | 분석: `analysis/latency/`(지연·시간동기) + `analysis/traction/` 신 msg 디코더·goal_id 분할·cnt→N 캘리 연동 | ✅ (2026-07-21, V1·V5 bag 검증) |
| 10 | `testbed_web` (§5.3, 기록은 §5.2 규격 준수) | 📋 미착수 |
| 11 | **rd_bridge 분할 리팩터링** — God-object 해소: `rd_bridge`(셸) / `rd_telemetry` / `rd_testbed_api` / `rd_carrier_api`. 전제(실기 통과) 충족됨. "동작 불변, 구조만"을 colcon test 로 증명. **이때 함께**: control 배치 READ 에 sys 세그(16~31) 포함(진단 토픽 stale 해소) + DIRECT 전환 크래시 조사 | 📋 미착수 |
| 12 | `testbed_cli` 보강 (§5.1 TODO): `--json` / 단어형 인자 / `run --timeout` — **AI 자동화 착수 전 `--json` 필수** | 📋 미착수 |

**이월 버그 (비블로킹, 담당 미정)**:
- **DIRECT 전환 크래시**: `config auto_mode 2` → 128:52 RW 가 RD_FATAL → 브리지 재시작 → SEGFAULT (재현 2/2).
  트랙션은 CURRENT 전용이라 비크리티컬. TEST4(속도 실험) 전에 해소 필요. 용의: STM RS485 52B RX 수용 여부 + 브리지 재시작 경합. → #11 때 조사.
- **control 모드 진단 토픽 stale**: `hw_reset/hw_error/hw_fatal`·`degraded_cnt` 가 control 배치 READ 범위 밖이라 거짓 정상.
  **판정 권위는 TestbedFeedback `sys_state`**. → #11 때 sys 세그 포함으로 해소 (방향 확정됨).
- ~~**STM 소스 미커밋**: CAN SJW 1→3TQ + BS1/BS2 재배분~~ → ✅ **이미 커밋돼 있었다** (2026-07-29 확인). `532ec69 UPDATE: Control mode`(7/25) 에 `main.c:413-415` 로 들어가 있고 워킹트리·플래시본과 일치한다 (SJW 3TQ / BS1 10TQ / BS2 3TQ).

## 7. 미해결 질문 → 확정 결과 (문답 2026-07-19, 코드 검증 완료)

0. **(2026-07-19 C-4 왕복 확정)** ⓐ `segment_index`(§3.4) = **기준 모터(세그 수 최다, 동수면 낮은 번호)의
   세그 인덱스, 패딩 구간은 마지막 값 유지** — 실시간 모니터링용 편의 태그. **분석의 권위 소스는
   YAML + profile_time** (사전 샘플링이 결정적이므로 세그 경계는 YAML 에서 재계산).
   ⓑ 프로파일 샘플 단위: **v1 player 는 CURRENT[A] 전용** (TEST3 전 계획이 전류 프로파일).
   velocity/position 프로파일은 TEST4 확장 예약 — YAML 세그 `mode:` 키 예약.
   ⓒ run_profile action 서버는 **INIT 성공 후에만 오픈** (검증 전 재생 창 제거 — 구현 판단 소급 반영).
1. **ECU AUTO 진입 경로 — 해결 (ECU 기구현)**: `reg.cmd_system.mode`(addr190) 가 모드의 단일 진실원천으로 GPIO 스위치·Orin write 양쪽이 갱신 (`rd_system.c:26`). → INIT 에서 `mode=1` write (§3.1) + config service `SET_MODE` op (§3.3). **STM 변경 없음 가정 유지.**
2. **SET_CTR_MODE 도메인 — 확정**: CURRENT(1) / CURRENT_BRAKE(2) / VELOCITY(3) / POSITION(4) + ESTOP(0, 모터별 송신 skip). §3.3 반영. 프로파일 샘플 값 단위는 ctr_mode 를 따름 (§4.1).
3. **goal_id 채번 — 확정**: 세션 내 단조증가 seq. 노드 재시작 시 1 부터 재시작하나, `run --record` 가 실험 1회 = 폴더 1개를 만들므로 전역 유일성은 폴더명이 담당 — 실험의 전역 키 = (폴더명, goal_id) 쌍. bag 간 goal_id 값 자체를 비교하지 말 것 (분석 파이프라인 규칙).
4. **motor_mask ECU 처리 — 코드 검증 완료** (2026-07-17 작업에 모두 포함):
   - CAN TX skip: `rd_can_motor.c:100` (마스크 제외 모터 송신 안 함)
   - motor_on 게이트: `RD_CAN_MOTOR_ALL_READY` — 마스크된 모터만 신선도 검사 (미연결 모터 있어도 기동 가능)
   - 에러 진단 제외: `RD_CAN_MOTOR_CHECKER` (`rd_can_motor.c:202`)
   - ESTOP 제동 경로에도 마스크 적용: `rd_peripheral_ecu.c:107`
   - 잔여: **실기 확인 1회** — 예: `active_motors:=[2,3]`, M1·M4 미연결 상태로 기동 → FAULT 없이 IDLE 도달 확인.

## 8. Phase 3 연결

- 실험 계획 확정본: **[test_plan_traction.md](test_plan_traction.md)** (TEST3 캠페인, Stage 0~5 + AI 자동화 프로토콜).
- 세그먼트 타입 셋과 `run --record` 폴더 규격이 실험 계획의 어휘 — 실험 1회 = YAML 1개. 표준 프로파일은 plan §2 (구현 시 `fw/profiles/` 파일화).
