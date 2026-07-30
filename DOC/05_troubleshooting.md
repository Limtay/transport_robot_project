# 05 — 안 될 때

증상 → 원인 → 조치. **가장 흔한 것부터.**

## 상태값 읽는 법 먼저

### 브리지 FSM (`control_state`)

| 값 | 뜻 | 명령이 나가는가 |
|---|---|---|
| `INIT` | 기동 검증 중 | 아니다 |
| `IDLE` | 대기 — 안전값(0)을 계속 쓴다 | 안전값만 |
| `RUNNING` | 프로파일 재생 중 | 그렇다 |
| `STREAM` | 실시간 조작 중 (`arm on` 필요) | 그렇다 |
| `LOCKED` | **래치** — `rearm` 없이는 안 나온다 | 아니다 |

### ECU 상태 (`ecu_sys_state`)

| 값 | 이름 | 뜻 |
|---|---|---|
| 0 | INIT | 부팅 중 |
| 1 | MANUAL | RC 조작 |
| **2** | **AUTO** | **정상 — 이 값일 때만 모터 명령을 받는다** |
| 3 | ESTOP_SW | 소프트 정지 (모터 fault 등) — **자동 복구된다** |
| 4 | ESTOP_HW | 물리 스위치 |
| 5 | FAULT | — |

`ecu_sys_state != 2` 면 **모든 모터 명령이 거부된다.**

---

## LOCKED 가 떴다

### 조건은 하나뿐이다

브리지가 LOCKED 로 가는 경로는 **정확히 하나**다:

> `bridge_mode: control` 에서 RW 트랜잭션이 성공했는데 ECU 가 **write 를 거부**했고,
> 그것이 **50 tick 연속**(0.25초) 이어졌다.

**통신 끊김으로는 LOCKED 가 되지 않는다.** 통신이 안 되면 트랜잭션 자체가 실패하고
브리지는 재시도한다. LOCKED 는 "말은 통하는데 ECU 가 거부한다" 는 뜻이다.

### 가장 흔한 원인 — 모터 fault (연쇄)

```
슬라이더/stream 을 빠르게 움직임
   ↓  tick(5ms)당 명령이 크게 점프 — 실시간 경로에는 레이트 제한이 없다
AK 모터가 과전류/락업 → 자기 error_code 를 세운다
   ↓
ECU 가 AUTO 를 벗어난다 (ecu_sys_state = 3 ESTOP_SW)
   ↓
ECU 가 모든 모터 명령을 거부한다 (mtr_lock)
   ↓
50 tick 연속 거부 → 브리지 LOCKED
```

거부 판정 범위는 **주소 132~187** 이다 — position(132)·velocity(148)·current(164) 가
전부 그 안이라 **모드와 무관하게** 막힌다.

### 확인

```bash
./cli.sh status --json | python3 -m json.tool | grep -E "ecu_sys_state|control_state|rw_err"
ros2 topic echo /carrier/ecu/motor --once      # error_code / temp 확인
```

브리지 stderr 에 200 tick 마다 한 줄 나온다:
```
[Map Warn] RW write rejected: [ACCESS] (cnt=...)
```

| 사유 | 뜻 |
|---|---|
| `ACCESS` | ECU 가 잠갔다 — 모터 fault 또는 AUTO 아님 |
| `DATA_LEN` | 주소·길이가 잘못됐다 |

> **LOCKED 사유 문자열에는 "RW write 연속 거부" 까지만 적힌다.** 진짜 원인(모터 fault)은
> 안 적히므로, 위 두 곳을 직접 봐야 한다.

### 조치

1. **원인을 먼저 본다** — 모터 온도·`error_code`. 과열이면 식힌다.
2. ECU 는 fault 가 사라지면 **스스로 AUTO 로 돌아온다.** `ecu_sys_state` 가 2 인지 확인.
3. 브리지는 자동 복구하지 않는다:
   ```bash
   ./cli.sh rearm
   ```
4. 재발 방지: position 을 나눠서 올린다 (0 → 15 → 30 → 45). 프로파일 재생은
   `limits.slew_rate` 가 있으므로 실시간 조작보다 안전하다.

---

## 브리지가 안 뜬다

### `UnknownROSArgsError: ['']`

빈 인자가 `--ros-args` 뒤에 붙었다. 스크립트에서 `"${ARR[@]:-}"` 를 쓰면 **빈 배열이 빈
문자열 하나로 전개된다.** 현재 스크립트들은 배열 append 방식으로 고쳐져 있다 — 직접
`ros2 run` 을 조립할 때 같은 함정을 조심한다.

### `AMENT_TRACE_SETUP_FILES: unbound variable`

`set -u` 아래에서 ROS `setup.bash` 를 source 했다. `_env.sh` 의 `rd_source_quietly()`
가 source 구간에서만 `-u` 를 푼다.

### 즉시 종료 (exit≠0), "Waiting for USB" 없이

기동 파라미터 오타다. 브리지는 **하드웨어를 시도하기 전에** 파라미터를 전수 검증하고
거부한다 (기본값으로 떨어뜨리면 의도하지 않은 모드로 뜨는데, 모터가 도는지 아닌지가
갈리는 문제다). stderr 에 어느 파라미터인지 나온다.

폐지된 것: `traction_test_mode`. 대체는 `./run.sh traction`.

### `No executable found`

그 패키지가 빌드되지 않았다.
```bash
./build.sh orin_firmware_bridge
```

## 브리지가 떴는데 ECU 와 말이 안 통한다

### `Open Failed: Bad file descriptor` 반복

**포트 권한**이다. USB 가 재열거되면 조용히 바뀐다.
```bash
ls -l /dev/ttyUSB0
./setup_rt.sh --perm    # → 재로그인
```

### "Waiting for USB..." 만 반복

장치가 없다. `ls /dev/ttyUSB*` / 전원 / 케이블. 다른 포트면 `RD_PORT=/dev/ttyUSB1`.

### 응답은 오는데 tick 이 밀린다 (`late_tick_cnt` 증가)

`latency_timer` 가 16 이다 (재부팅하면 되돌아간다).
```bash
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer   # 1 이어야 한다
./setup_rt.sh
```

### 다른 프로세스가 포트를 잡고 있다

```bash
pgrep -a comm_test_node
```
웹이 띄운 것일 수 있다 — 웹에서 정지시킨다. `pkill` 로 강제 종료하면 웹의 supervisor 가
자식을 잃고 상태가 어긋난다.

---

## 웹이 안 보인다

### API 는 200 인데 페이지가 404 / 빈 화면

정적 파일 서빙 문제다. 과거 원인: 경로 탈출 가드가 `os.path.realpath` 로 검사해서
`--symlink-install` 로 깐 트리에서 심링크를 따라가 root 밖으로 판정됐다. 지금은
`normpath` 로 고쳐졌고 회귀 테스트가 있다(`test_static_serving.py`).

```bash
curl -s -o /dev/null -w '%{http_code}\n' localhost:8080/          # 200
curl -s -o /dev/null -w '%{http_code}\n' localhost:8080/api/state # 200
```

새 파일을 `www/` 에 추가했다면 **`setup.py` 의 `data_files` 에도 넣어야 한다** —
개발 트리에서는 되고 설치본에서만 404 가 난다. 테스트가 이것을 잡는다.

### 포트가 이미 쓰인다

`web.sh` 가 미리 알려준다. `--port 9000`.

### `브리지가 이미 떠 있다 (이 웹이 띄운 것이 아니다)`

터미널에서 `comm_test_node` 를 먼저 올렸다. 그것을 내리거나, 웹 대신 `./run.sh` +
`./cli.sh` 를 쓴다.

---

## 값이 이상하다

### 전부 NaN 이다

**미판독이다** — 지금 읽기 프리셋이 그 구간을 안 읽는다. 0 이 아니라 NaN 인 이유:
0 은 "정지"·"고장 없음" 이라는 유효한 관측값이므로 섞으면 판별이 불가능해진다.

```bash
./cli.sh status --json | grep read_preset
./cli.sh config preset 1     # diag — hw_error·DIAG 블록이 실값으로
```

| 프리셋 | 읽는 것 |
|---|---|
| `control` (0) | SYS 16:17 + 42~127 (로드셀·IMU·엔코더·모터) + ctr_mode + cmd_current |
| `diag` (1) | 0~32 + ctr_mode + DIAG 224:32 |
| `control_test` (2) | SYS + 로드셀 + 모터 + cmd_current (견인 실험) |

### `link_angle` 이 NaN 이다 (엔코더)

**엔코더 I2C 버스가 고장 상태다** (2026-07-29 확인). ch0 만 응답하고 ch1~4 는 타임아웃이다
(`delta_tick = 0xFF`). I2C MUX 전원·배선·풀업 점검이 필요하고, **소프트웨어로 더
알아낼 것은 없다** — ECU 가 이미 채널별 stale 플래그로 정확히 보고하고 있다.

```
state(addr 85) = 0x2F   → lifecycle=15 LS_OFFLINE / health=2 HC_TIMEOUT
degraded_cnt[4] = 100
hw_error = hw_fatal = 0x10   (bit4 = i2c1/엔코더)
```

> **⚠ raw 값이 그럴듯하다고 살아 있다고 읽으면 안 된다.** ch1~4 의 raw 는 12bit 범위 안의
> 값이 들어 있지만 `delta_tick` 이 stale 이라 **언제 찍힌 값인지 모른다.** 판정 기준은
> 값의 그럴듯함이 아니라 stale 플래그다. 브리지는 채널별로 판정해 stale 채널을 NaN 으로
> 낸다 — 그것이 옳은 동작이다.

### `hw_error = 16` 이 계속 뜬다

위와 같은 것이다 (bit4 = 엔코더). **"원래 그런 것" 으로 넘기지 않는다** — 상시 표시돼야
하는 값이다.

### `rtt` / `clock_offset` 이 전부 0

시계 추정기가 죽었을 때의 증상이다. `comm_diag_enable` 없이 원자료를 보려 한 것일 수도
있다. `status --json` 의 `stamp_valid` / `rtt_ms` 를 본다.

### 슬라이더를 움직였는데 아무 일도 없다

`arm on` 을 하지 않았다 (웹의 ④ run). `status` 의 `control_state` 가 `STREAM` 이어야 한다.

### `origin` 이 거부된다

`auto_mode: direct` + `safe_stop` 이 필요하다. 사유가 직접 나온다:
```bash
./cli.sh status --json | grep safe_stop
```

---

## 프로파일이 거부된다

| 증상 | 원인 |
|---|---|
| `goal 거부됨` | 프로파일 `mode` ≠ 브리지 `auto_mode` |
| position 인데 거부 | `limits.range` 가 없다 (필수) |
| `sine` 거부 | `freq` 상한 초과 |
| `prbs` 거부 | `low == high` |
| `custom` 거부 | 샘플 레이트 상한 초과 |
| FSM 이 IDLE 이 아니라 거부 | `abort` 또는 `rearm` 후 재시도 |

거부 사유는 브리지가 문장으로 준다 — CLI stderr / 웹 result 영역에 그대로 나온다.
`orin_ws/src/orin_firmware_bridge/test/profiles/reject_*.yaml` 이 실례다.

---

## 빌드·테스트

### `colcon build` 가 exit 1

**`orin_ws` 루트에서 실행해야 한다.** `build.sh` 는 어디서 호출해도 스크립트 위치에서
워크스페이스를 찾으므로 그것을 쓴다.

`src/` 안에서 실수로 돌렸다면 유령 트리를 지운다 — 그것이 나중에 import 돼서
**고친 코드가 아닌 옛 코드가 도는** 상황을 만든다:
```bash
rm -rf orin_ws/src/*/build orin_ws/src/*/install orin_ws/src/*/log
```

### 첫 빌드가 메시지 헤더를 못 찾는다

메시지 패키지를 먼저 빌드하고 `install/setup.bash` 를 재-source 해야 한다.
`build.sh` 가 이미 그렇게 한다.

### `carrier_teleop` 테스트가 `pytest.missing_result` 로 실패

**코드 문제가 아니다.** pytest 플러그인 충돌로 colcon 이 결과 파일 없이 죽는 알려진
현상이다. 다른 패키지 테스트는 정상이다.

### 파이썬 테스트를 직접 돌리려면

```bash
cd orin_ws && source install/setup.bash
cd src/control_cli && python3 -m pytest test/ -q
```
overlay 를 source 하지 않으면 `ModuleNotFoundError: mgs_tp_msgs` 가 난다.

### `test_cli_e2e` 가 간헐적으로 실패

가짜 action 서버의 DDS discovery 타이밍이다. 단독으로 돌리면 통과한다.

---

## 무엇을 봐야 할지 모를 때

```bash
# 1. 브리지가 살아 있나
pgrep -a comm_test_node

# 2. 상태 전체
./cli.sh status --json | python3 -m json.tool

# 3. 하드웨어 진단
ros2 topic echo /carrier/ecu/status --once     # lc/hs/degraded_cnt/hw_error
ros2 topic echo /carrier/ecu/motor --once      # error_code / temp

# 4. 데이터가 흐르나
ros2 topic hz /carrier/control/feedback        # control 모드에서 200Hz

# 5. 레지스터를 직접
./cli.sh config preset 1                       # diag 로 갈아끼우고 다시 status
```

이 순서로 좁혀지지 않으면 브리지 stderr 를 그대로 본다 — 거부·경고는 전부 거기 남는다
(웹은 기동 패널 아래 로그창에 같은 것을 보여준다).
