# 12 — Orin Position Control 구체안 (2026-11-05)

> 범위: `feature/ecu_ctr`, `orin_ws/src` — 브리지 + CLI. 웹은 §8 에 계약만 적고 13 문서에서 구체화한다.
> 전제: ECU 쪽은 [11_ecu_position_261030.md](11_ecu_position_261030.md). **아직 플래시 전이라** 실물 ECU 시험은 뒤로 미루고, 가짜 ECU 테스트로 먼저 검증한다 (§9).
> 실험 주체가 AI 이므로 **CLI 만으로 준비·실행·판정이 끝나야 한다** (§7).

---

## 1. 한눈에

```
./run.sh position   = control + auto_mode:position + position_inst:true   (기동 시 고정)

매 tick (200Hz)
  ① 플래너 샘플 s (시작점 기준 변위)
  ② cmd_abs = start_abs + s          ┐ RdControl.PrepareWrite
  ③ cmd_rel = cmd_abs − offset       ┘
  ④ INST_POS(0x05) 송신, POS_FB 88B 수신
  ⑤ POS_FB → 기존 섀도 자리에 미러      → 기존 코드(SafeStop·피드백·시계 동기)가 그대로 동작
  ⑥ RdPosFrame.Update: origin_seq 가 바뀌면 offset += pre_origin_pose
  ⑦ 안전 판정 (FRAME_LOST / seq 건너뜀 / ECU 리부트 / ESTOP / 추종 오차) → 위반 시 LOCKED
  ⑧ /carrier/control/pos_feedback 발행
```

| 결정 | 내용 | 이유 |
|---|---|---|
| Q1 | INST_POS 사용은 **기동 파라미터로 고정** (런타임 전환 없음) | RW↔INST_POS 전환이 사라져 11 §9 의 좌표 어긋남 위험이 원천 차단된다 |
| Q2 | POS_FB 를 **기존 섀도 자리에 미러** | 소비자마다 분기하면 SafeStop·IDLE 재시드가 낡은 값을 쓸 위험. 함수 1개로 해결 |
| Q3 | 새 값은 **새 토픽 `pos_feedback`** — `ControlFeedback` 은 불변 | 기존 bag 분석이 메시지 정의로 디코드하므로(`rosbags`), 필드를 늘리면 옛 bag 이 깨진다 |
| Q4 | 프로파일은 **시작점 기준 변위** (`position_ref: start` 명시) | 플랜은 변위이고 모터 절대값은 매번 다르다. 같은 YAML 이 설정에 따라 뜻이 달라지지 않게 키를 필수로 |
| Q5 | **추종 오차 한계 필수** (`limits.max_track_err`) | AI 가 무인으로 돌린다. 걸림·미끄러짐으로 오차가 커지면 스스로 멈춰야 한다 |

---

## 2. 레지스터 미러 (`core/rd_register_ecu.hpp`) — ECU 헤더와 맞춤

| 항목 | 변경 |
|---|---|
| `REG_TOTAL_SIZE` | 256 → 344. `PAGE_POS_FB_t pos_fb` 추가, `REG_POS_FB_OFFSET=256`, `SIZE=88` |
| `CMD_SYSTEM_t` | `lin_zero_req`(193). `REG_CMD_SYSTEM_SIZE` 14, `REG_RSVD1_OFFSET` 194, `SIZE` 30 |
| `DIAG_t` | `origin_jump_rej`(228), `reserved[24]` |
| `rd_common.hpp` | `HW_BIT_UART4 (1u<<6)`. 진단 인덱스 6 = RLS (`kIdxRls`), 7 = RSVD |

256 을 전제한 곳도 같이 고친다: `ReadAgeMap::kBytes`(344), `WriteEcho::kSize`(344 — 쓰기는 256 미만이지만 주석·상수를 맞춘다), `test_regmap.py`, `rd_test_fixture.hpp` 의 `ecu_reg_[256]`. DPC·PCU 는 256 그대로다.

**미러 함수** (`rd_register_ecu.hpp`, 순수 함수):
```cpp
inline void MirrorPosFb(REGISTER_t* r) {
    const auto& f = r->pos_fb;
    r->sys.realtime_tick = f.realtime_tick;   r->sys.rs485_proc_delta = f.rs485_proc_delta;
    r->sys.sys_state     = f.sys_state;       r->sys.hw_reset         = f.hw_reset;
    memcpy(r->sys.degraded_cnt, f.degraded_cnt, 8);
    for i: r->motor_data.position/velocity/current[i] = f.mtr_*[i];
    r->motor_data.state = f.motor_state;
    r->loadcell.avg[0..1] = f.loadcell_avg;   r->loadcell.state = f.loadcell_state;
}
```
미러 안 되는 값 → **미판독 유지**: `hw_error`, `hw_fatal`, 모터 `temp`·`error_code`·`delta_tick`, IMU, AS5600.
실행 중 원인 확인은 불가하고, LOCKED 후 IDLE 에서 `command set auto 1 ecu read_all` 로 읽는다 (§7).

---

## 3. 와이어·스케줄러

| 파일 | 변경 |
|---|---|
| `core/rd_comm.hpp` | `PacketInst::POS = 0x05` |
| `core/rd_map.cpp` Encode | POS: `Data = waddr(2) + 섀도 write 바이트`. read 세그는 **보내지 않는다** |
| `core/rd_map.cpp` Decode | POS 를 RW 분기에 합류 (에러 니블·거부 카운터 공유) → scatter → `MirrorPosFb` |
| `sched/rd_schedule.cpp` | `task_pos_inst_` 를 생성자에서 1회 굽기: `TaskConfig_t(ECU, POS, 132, 16, {{256,88}})` |
| 〃 `SelectControlTask` | `cfg_.position_inst` 면 `task_pos_inst_` |
| 〃 `ExecuteTask` | `has_write`, `ClassifyWrite(is_rw)`, `MarkRead` 조건에 POS 추가. `sent[]` 는 16B 라 그대로 |
| 〃 `MarkRead` | 256:88 + **미러된 구간도 같이** 도장 (Tab4 가 미판독으로 그리지 않게) |
| 〃 control 분기 | 성공 시 `control_->OnPosFb()` (§4). 나머지 순서·`OnRwErr`·`OnTxn`·`OnFeedback` 은 그대로 |

- **쓰기는 항상 132:16 (4모터 전체).** 11 §3 은 부분 쓰기도 허용하지만, 활성 모터가 M1·M4 면 어차피 연속이 아니고 "런타임에 태스크를 조립하지 않는다"(04 §2.4.2) 규칙과 맞춘다. 비활성 모터 값은 ECU 가 mask 로 무시한다. 요청 26B, 응답 97B ≈ 1.3ms 로 5ms 주기에 충분하다 (기존 control 프리셋 응답 106B 보다 작다).
- **기동 프로브**: `InitControl` 에서 `position_inst` 면 INIT 전에 `READ 256:88` 1회. 실패 → `init_fatal_` + "ECU 펌웨어가 POS_FB 미지원 — 재플래시 필요" 후 exit≠0. 구 펌웨어에 0x05 를 보내면 에러 바이트가 어긋나 원인을 알 수 없게 되므로 입구에서 막는다.

---

## 4. 좌표 추적 — `policy/rd_pos_frame.hpp/.cpp` (신규, ROS 무의존)

**모터별 상태**: `offset_deg`(double), `last_seq`, `abs_deg`, `start_abs_deg`, `cmd_abs_deg`, `origin_cnt`, `max_track_err`.
`RdControl` 이 멤버로 소유하고 `mutex_` 로 보호한다.

`Update(pos_fb, active_mask)` — 응답마다 1회:

| 조건 | 동작 |
|---|---|
| 첫 응답 | `last_seq = seq`, `offset = 0`. 절대좌표의 0 = 이 세션 시작 위치. 호밍 없음 (프로파일이 변위이므로 필요 없다) |
| `seq` 차이 1 | `offset += pre_origin_pose·0.1`, `origin_cnt++` |
| `seq` 차이 ≥ 2 | **LOCK** "원점 이벤트 유실". ECU 는 PEND 중 재원점을 금지하므로 정상이면 불가능 |
| PEND=1 이고 `seq` 불변 | `abs_deg` 를 갱신하지 않는다 (11 §9-2). 이 샘플의 fb 는 어느 좌표인지 모른다 |
| 그 외 | `abs_deg = offset + mtr_position·0.1` |
| `realtime_tick` 역행 또는 `seq` 감소 | ECU 리부트 → 프레임 무효. RUNNING 이면 **LOCK**, IDLE 이면 재초기화 + WARN |
| `FRAME_LOST`(활성 모터) | **LOCK** "FRAME_LOST m*". ECU 는 POSITION 이탈 전까지 제동을 유지한다 → 아래 재무장 필요 |

**재무장 (`OP_REARM`, FRAME_LOST 일 때)**: ECU 래치는 `ctr_mode` 가 POSITION 을 벗어나야 풀리는데, `auto_mode=5` 를 다시 써서는 벗어나지 않는다.
절차: `auto_mode=1(CURRENT)` 검증 쓰기 → 30ms 대기 (ECU 100Hz 가 `ctr_mode` 를 바꿔 래치 해제) → `auto_mode=5` 검증 쓰기 → 프레임 재초기화 → IDLE.
그 사이 전류 명령은 0A 이고 INST_POS 쓰기는 거부되지만 6 tick 이라 LOCK 임계(50 tick)에 안 닿는다. 기동 프로브에서 FRAME_LOST 가 보여도 같은 절차를 탄다 (노드 재시작만으로는 안 풀린다).

`RdControl::OnPosFb()` 는 추가로 RUNNING 중에만 본다:

| 조건 | 동작 |
|---|---|
| `sys_state != AUTO` | **LOCK** — ECU 가 ESTOP·FAULT 인데 플랜 시간만 흐르는 것을 막는다 |
| `\|cmd_abs − abs\| > max_track_err` (활성, PEND 제외) | **LOCK** + `max_track_err` 기록 |

**`PrepareWrite` POSITION 분기**:
```cpp
if (pos_inst_) {
    cmd_abs = RUNNING ? start_abs[i] + clamp_prof(sample)   // 프로파일 = 변위
            : STREAM  ? stream_position[i]                  // 스트림 = 절대
            :           hold_abs[i];                        // IDLE·LOCKED — 잡아 둔 값
    cm.cmd_position[i] = cmd_abs - offset[i];
}
```
- **IDLE 은 재시드하지 않는다 (2026-11-05 실기로 변경).** fb 는 0.1° 단위로 잘려 와서 매 tick cmd 로 되먹이면 목표가 한 방향으로 밀렸다 (61.1→56.6°, 0.14°/s, -5.3A, 프로파일 없음). 한 번 잡은 cmd 만 보낸다. 잡는 시점: 첫 응답 = fb, 런 종료 = 마지막 cmd, LOCK·REARM = 그 시점 fb. RW 경로의 재시드는 그대로다.
- `BeginProfile`: 활성 모터에 PEND 가 있으면 거부, 없으면 `start_abs = hold_abs` 래치 (재생 첫 cmd 가 직전 cmd 와 같다).
- `SET_AUTO_MODE`·`SET_READ_PRESET`·`SET_ORIGIN` 은 `position_inst` 에서 **거부**한다 (사유 포함).

---

## 5. 프로파일 (`policy/rd_profile.*`)

```yaml
name: climb_200mm_v1
mode: position
position_ref: start          # 신규. 값은 시작점 기준 변위 [deg]
limits:
  range: [-500, 8000]        # 변위 기준
  max_track_err: 30.0        # [deg] 신규. 넘으면 LOCKED
motors:
  m1: [ {type: custom, rate: 200, samples: [0.0, 0.1, ...]} ]
  m2: [ ... ]
```
수락 규칙 (`HandleProfileGoal`):

| `position_inst` | `position_ref` | 결과 |
|---|---|---|
| true | `start` | 수락. `max_track_err` 없으면 거부, 활성 모터 첫 샘플 \|v\| > 1° 면 거부 (변위는 0 에서 시작) |
| true | 없음 | 거부 — 절대값으로 해석되면 시작 위치에 따라 모터가 튄다 |
| false | `start` | 거부 — RW 경로는 절대값이다 |
| false | 없음 | 기존 동작 그대로 |

**거부 사유를 저장한다**: action 거부에는 문자열을 실을 수 없어 지금은 브리지 로그에만 남는다. `last_reject` 로 보관해 `GET_STATUS` 로 내보낸다 (AI 가 로그를 안 보고도 원인을 안다).

`RunProfile.Result` 추가: `uint8[4] origin_cnt`, `float32[4] start_abs_deg`, `float32[4] end_abs_deg`, `float32[4] max_track_err_deg`.

---

## 6. 텔레메트리 — 새 토픽 `/carrier/control/pos_feedback` (`PosFeedback.msg`, 200Hz)

`position_inst` 에서만 생성. 응답 1건 = 메시지 1건, `ControlFeedback` 과 같은 `header.stamp`·`ecu_tick`.

```
std_msgs/Header header
uint32     ecu_tick
uint32     goal_id
float32    profile_time
float64[4] cmd_abs          # [deg] 플랜이 요구한 절대 위치. IDLE 은 NaN
float64[4] fb_abs           # [deg] offset + fb. PEND 구간은 직전 값
float64[4] offset           # [deg]
bool       fb_abs_held      # 이번 샘플이 PEND 로 유지된 값인가
uint8[4]   origin_seq
int16[4]   pre_origin_pose  # x0.1 deg
uint8      origin_flag      # bit0~3 PEND / bit4~7 FRAME_LOST
uint32[3]  rls_raw          # psi1, psi2 (18bit) / theta3 (14bit)
float32[3] rls_deg          # 해당 채널 Error 면 NaN
uint8      rls_ew
uint8      rls_state
int32[2]   lin_xy           # [count]. 채널 Error 면 INT32_MIN (0 은 유효값)
uint8      lin_zero_result
uint8      lin_zero_seq
uint8      odiag_motor
uint8      odiag_dt
int16      odiag_e_track
int16      odiag_residual
int16      odiag_vel
```

**`ControlFeedback` 동작 (정의는 불변)**: `RdTelemetry` 에 `pos_inst` 판정을 두어 `motor_vals`·`loadcell_vals` 를 fresh 로 본다. 그 값은 **모터 좌표 그대로**(`cmd`, `fb_position`)이고 절대값은 `pos_feedback` 에 있다. `dt_*`, IMU, 링크각은 NaN, `hw_error`/`hw_fatal` 은 255.

**`NodeStatus` 수정**: 지금은 `lc/hs` 를 프리셋과 무관하게 채워, 안 읽은 채널이 "정상"(0)으로 나간다. `Reads()` 또는 미러 구간일 때만 채우도록 고치고, `kIdxRls = 6` 을 추가한다.

**기록**: `record_topics()` 가 `position_inst` 일 때 `pos_feedback` 을 포함. `result.json` 은 schema v2 유지 + 선택 키 `position` (origin_cnt, start/end_abs, max_track_err).

---

## 7. CLI — AI 가 혼자 돌릴 수 있게

원칙: **stdout 한 덩어리 JSON + 의미 있는 종료 코드.** 기존 규약(0 성공 / 1 거부·실패 / 2 인프라 / 3 기록 무효)을 유지한다.

| 명령 | 내용 |
|---|---|
| `status --json` (확장) | `position_inst`, `pos{abs_deg, offset_deg, seq, pend, frame_lost, rls{deg,ew,state}, lin{x,y,zero_result,zero_seq}, loadcell}`, `last_reject` |
| `preflight [--json]` (신규) | 전 항목 통과 여부 + 항목별 사유. 실패 시 exit 1: 브리지 `position_inst` / ECU 연결 / `sys_state=AUTO` / FSM IDLE / 활성 모터 mask / 시계 수렴 / `safe_stop` / PEND·FRAME_LOST 없음 / `rls_ew`(지정 채널) / 로드셀 상태 |
| `lin-zero [--json]` (신규) | `lin_zero_req=1` → `lin_zero_seq` 변화 대기(1s) → 결과별 exit. 무응답은 "불확정"으로 exit 1 |
| `run <yaml> --record --json` (확장) | 진행 표시 끔, 결과 JSON 1개 (`success`, `message`, `run_dir`, 카운터, `position`). 거부 시 `last_reject` 를 실어 exit 1 |
| `abort`, `rearm` | 기존 그대로 |
| LOCK 진단 | LOCKED 면 `run`·`status` 가 `lock_reason` 을 내고, `command set auto 1 ecu read_all` 로 온도·에러코드를 읽을 수 있다 |

**AI 실험 절차**: `run.sh position` → `preflight` → (`lin-zero`) → `run --record --json` → 결과 판정 → 실패면 `status --json` → `read_all`.
`lin-zero` 는 ECU 가 레지스터를 곧바로 지우므로 **read-back 검증 경로(`RdOos::Request`)를 쓸 수 없다** — 검증 없는 단발 쓰기 `OP_LIN_ZERO`(=10)를 추가한다. IDLE 전용.

`run.sh` 에 `position)` 케이스 추가. `PARAMETERS.md`, `DOC/03_cli.md`, `README.txt` 갱신.

---

## 8. 웹 (13 문서에서 구체화) — 브리지가 지금 보장할 것

- 기동 패널: `position_inst` 파라미터. `GET_STATUS` 에 그대로 노출.
- `pos_feedback` 구독 → Tab1/2 는 `fb_abs`·`cmd_abs` 표시. **슬라이더 초기값은 `fb_position` 이 아니라 `fb_abs`** (`server.py:192,269` — 스트림이 절대값이므로).
- Tab2: `mode: position` 에 `position_ref`, `max_track_err` 입력 (`profile_run.py` `build_yaml`).
- Tab4: POS_FB 페이지 디코딩 (`regmap.js`, 전체 344B).

---

## 9. 검증

**실물 ECU 없이**: `rd_test_fixture` 의 가짜 ECU 에 0x05 + 원점 시퀀스를 구현해 통합 테스트한다. 와이어 호환은 실물 전까지 증명되지 않는다.

| 테스트 | 확인 |
|---|---|
| `test_stm_mirror` | 344B, `pos_fb` 전 필드 오프셋, `lin_zero_req`, `origin_jump_rej` (STM 헤더 직접 대조) |
| `test_golden_wire` | INST_POS 요청 바이트, 응답 scatter + 미러 |
| `test_pos_frame` (신규) | seq +1 → offset 1회만 / PEND 유지 / seq 건너뜀·역행·FRAME_LOST → 잠금 / offset 누적 |
| `test_pos_inst` (신규) | 가짜 ECU 로 3000° 통과: `cmd_rel` 전환 tick, `fb_abs` 연속, `start_abs + s` 일치, 추종 오차·ESTOP 잠금 |
| `test_profile_parser` | §5 수락 표 4칸, `max_track_err` 누락, 첫 샘플 |
| `test_interface_inventory` | `pos_feedback` 은 `position_inst` 때만 |
| 회귀 | 기존 전체 — `position_inst=false` 면 와이어가 **바이트 단위로 동일** (golden) |
| CLI | `preflight`·`lin-zero`·`run --json` 출력 스키마, 종료 코드 |

⚠ 이 PC 에는 ROS 2·컴파일러가 없다. 빌드·테스트는 Orin 에서 `./build.sh --test`.

실물 시험은 ECU 플래시 후 11 §8 T0~T11 과 함께 한다.

---

## 10. 구현 순서

1. 레지스터 미러 + `test_stm_mirror` (§2)
2. 와이어·스케줄러 + golden (§3)
3. `RdPosFrame`·`RdControl` + 단위 테스트 (§4)
4. 프로파일 + 파서 테스트 (§5)
5. 메시지·텔레메트리 (§6)
6. GET_STATUS·OP_LIN_ZERO·CLI (§7)
7. 가짜 ECU 통합 테스트 (§9), 문서

---

## 11. 정해야 할 것

1. ~~IMU 를 기록할까?~~ **넣지 않는다 (2026-11-05 사용자 결정).** POS_FB 는 88B 유지, position 런의 IMU·AS5600 은 NaN. 나중에 필요하면 ECU 수정·재플래시.
2. ~~실기 기동 권한~~ **런마다 사용자 승인 (2026-11-05 사용자 결정).** 사용자가 ESTOP 옆에 있을 때만 AI 가 CLI 로 실행한다.
3. **플랜 → YAML 변환** (`tools/plan_to_profile.py`). 트랙 변위 → 모터 각도 스케일과 모터별 부호, 활성 모터 2개 지정이 필요하다. T1 실측 후 별도 작성.
