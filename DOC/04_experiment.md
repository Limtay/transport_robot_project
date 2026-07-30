# 04 — 실험 1회를 규격대로 남기기

> **실험 1회 = 폴더 1개 = 자기완결 기록.** 폴더만 보고 무엇을 어떻게 돌렸는지 재현할 수
> 있어야 한다. 그것이 분석 파이프라인의 입력 계약이다.

## 1. 폴더 규격

```
data/rosbags/<이름>_<MM-DD_HH-MM>/
├── bag/            # ros2 bag record 산출물
├── profile.yaml    # 제출한 프로파일 원문 사본
└── result.json     # schema v2 요약
```

전역 유일 키는 **(폴더명, goal_id)** 쌍이다. `goal_id` 는 세션 내 단조증가일 뿐이므로
폴더명이 전역 유일성을 담당한다.

이 폴더는 웹으로 돌려도 CLI 로 돌려도 **같은 모듈**(`control_cli/record.py`)이 만든다.

## 2. 프로파일 YAML

```yaml
name: step_5A
mode: current              # current | velocity | position
limits:
  max_abs: 5.0             # 절대값 상한
  slew_rate: 50.0          # [단위/초] 레이트 제한 (선택)
motors:
  m1:
    - {type: hold, duration: 1.0, value: 1.0}
    - {type: ramp, duration: 1.0, from: 1.0, to: 2.0}
    - {type: hold, duration: 2.0, value: 2.0}
```

position 모드는 **`range` 가 필수**다:

```yaml
mode: position
limits: {range: [-10.0, 90.0]}
```

### 세그먼트 타입과 인자

| 타입 | 인자 | 비고 |
|---|---|---|
| `hold` | duration, value | |
| `ramp` | duration, from, to | |
| `step` | duration, from, to, t_step | `t_step` 에서 전환 |
| `sine` | duration, amp, freq | freq 상한 있음 |
| `chirp` | duration, amp, f0, f1 | |
| `prbs` | duration, low, high, bit_duration | `low == high` 는 거부 |
| `noise` | duration, mean, std | **slew 검사 면제** |
| `stair` | step_duration, values | |
| `custom` | samples | 샘플 레이트 상한 있음 |

- **200Hz(5ms) 로 사전 샘플링**된다. 재생 중에는 배열 인덱싱만 하므로 tick 안에서
  연산이 일어나지 않는다.
- 총 길이 상한 1시간.
- `noise`/`prbs` 를 쓰면 **`seed` 를 기록해야 재현된다.** 미지정이면 브리지가 시각 기반으로
  정하고 그 값이 `result.json` 에 남는다 — 기록되지 않으면 그 파형은 영구 소실이다.

거부 예시는 `orin_ws/src/orin_firmware_bridge/test/profiles/reject_*.yaml` 에 있다
(파일명이 곧 거부 사유다).

### ⚠ `mode` 는 브리지 `auto_mode` 와 같아야 한다

다르면 goal 이 **거부된다**. 브리지를 `auto_mode:=current` 로 띄우고 `mode: position`
프로파일을 던지면 그렇게 된다.

```bash
./cli.sh status --json | grep auto_mode
```

## 3. 실행

### CLI

```bash
cd orin_ws
./run.sh control current &
./cli.sh run data/profiles/step_5A.yaml --record --name s1_w20
```

`arm` 은 **필요하지 않다.** arm 게이트는 STREAM(실시간 조작) 전용이고, 프로파일은 action
goal 로 들어와 RUNNING 으로 간다. 단 **FSM 이 IDLE 이어야 goal 이 수락된다** — STREAM 중이면
먼저 `arm off` 한다.

### 웹

Tab2 에서 세그먼트를 만들고 `기록` 을 켠 뒤 `재생`. 브리지 `auto_mode` 는 같은 화면의
기동 패널에서 확인한다.

## 4. `result.json` (schema v2)

**실패한 런도 반드시 남는다** — 왜 실패했는지가 데이터다. 거부로 끝나면
`ticks_executed=0`, `message` 에 사유가 들어간다.

| 키 | 뜻 |
|---|---|
| `schema_version` | 2 |
| `name` / `goal_id` / `success` / `message` | 식별·결과 |
| `mode` | **해석된 실효 mode** (프로파일에 없어도 브리지가 정한 값) |
| `profile_source` / `profile_sha256` | 원본 경로 + 해시 (사본이 유실돼도 동일성 판정 가능) |
| `seed` | noise·prbs 재현용 |
| `ticks_executed` | 실제 재생 tick |
| `write_err_cnt` | ECU 가 write 를 거부한 횟수 |
| `clamp_cnt` | 한계에 걸려 잘린 횟수 |
| `irregular_tick_cnt` | 정규 RW 가 대체된 tick — **그 tick 은 피드백이 결손** |
| `late_tick_cnt` | 주기를 넘겨 위상이 리셋된 tick |
| `drop_cnt` | 발행 큐 드롭 — **시계열에 구멍이 있었는지 아는 유일한 수단** |
| `slew_exempt_ticks` | slew 검사를 건너뛴 tick (= noise 구간 길이) |
| `clock_converged` / `drift_ppm` | 시간축 등급 |
| `started_at` / `finished_at` | ISO8601 |
| `node_params` | 그 런의 브리지 파라미터 전체 |
| `bag_dir` | 폴더를 옮겨도 원 위치를 안다 |

### 세 카운터는 서로 겹치지 않는다

| | 무슨 일 |
|---|---|
| `irregular_tick_cnt` | **자리를 뺏김** — out-of-span write 등이 그 tick 을 대체했다 |
| `late_tick_cnt` | **늦음** — 주기를 넘겨 시간축이 뒤로 밀렸다 |
| `drop_cnt` | **기록을 놓침** — 발행 큐가 가득 차 버렸다 |

`late_tick_cnt` 가 크면 **기록된 `t`(=tick×5ms)와 실제 경과가 벌어진 것**이다. 이 루프는
tick 번호를 건너뛰지 않는다 — 따라잡지 않고 시간축을 미룬다.

### 분석 전 확인할 것

```bash
python3 -c "
import json; d=json.load(open('data/rosbags/<폴더>/result.json'))
print('success       ', d['success'], d['message'])
print('ticks         ', d['ticks_executed'])
print('write_err     ', d['write_err_cnt'])
print('drop/late/irr ', d['drop_cnt'], d['late_tick_cnt'], d['irregular_tick_cnt'])
print('clock         ', d['clock_converged'], d['drift_ppm'])
"
```

- `drop_cnt > 0` → 시계열에 구멍이 있다.
- `clock_converged == false` → `header.stamp` 가 Orin 수신 시각 fallback 이다.
  **수렴 런과 섞지 않는다.**
- `write_err_cnt` 가 크면 ECU 가 명령을 거부하고 있었다 — LOCKED 직전 상태일 수 있다.

## 5. 수동 조작을 남길 때

프로파일이 아닌 조작(`stream`, teleop)은 실험 폴더 규격이 만들어지지 않는다.

```bash
./rosbag_test.sh              # data/rosbags/manual_<MM-DD_HH-MM>/bag
./rosbag_test.sh s1_w20       # 라벨 지정
./rosbag_test.sh s1_w20 --diag
```

기록 토픽은 `record.py` 에게 물어서 정한다 (하드코딩하지 않는다 — 종전에 개명 전 토픽이
박혀 있어서 **빈 bag** 이 남았다).

**Ctrl-C 로 종료한다.** SIGKILL 하면 메타데이터가 안 써져 bag 이 열리지 않는다.

## 6. 분석으로 넘기기

```bash
# 1) bag 을 data/rosbags/ 에 둔다 (기본 경로면 이미 거기 있다)
# 2) analysis/traction/test_index.csv 에 행 추가
# 3) 재분석
python analysis/traction/traction_analysis.py
```

> ### ⚠ 현재 분석 파이프라인은 신 계약을 아직 못 읽는다
>
> 토픽·메시지 개명(`/carrier/testbed/feedback` → `/carrier/control/feedback`,
> `TestbedFeedback` → `ControlFeedback`)이 분석 스크립트에 반영되지 않았다.
> `traction_analysis.py` 는 구 타입 디코더만, `latency_analysis.py` 는 폐지된
> `comm_latency` 토픽을 전제한다.
>
> **기록은 규격대로 쌓이고 있다** — 나중에 파이프라인을 갱신하면 그대로 들어간다.
> 지금 남기는 런을 버릴 필요는 없다. (`redesign/08_audit_260730.md` §2.3 이 이 항목이다.)

## 7. 체크리스트

**실험 전**
- [ ] `latency_timer == 1` (재부팅했으면 다시)
- [ ] `./cli.sh status` — `control_state` 가 LOCKED 가 아니다
- [ ] 프로파일 `mode` == 브리지 `auto_mode`
- [ ] `active_motors` 가 실제로 연결된 모터와 같다
- [ ] `hw_error` / `degraded_cnt` 확인 (Tab3 또는 `config preset 1`)

**실험 후**
- [ ] `result.json` 의 `success` / `message`
- [ ] `drop_cnt` / `late_tick_cnt` / `irregular_tick_cnt`
- [ ] `clock_converged`
- [ ] 종료 코드 3(sha256 불일치)이 아니었다
