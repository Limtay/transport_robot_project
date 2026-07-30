# 02 — 웹 UI (`control_web`)

```bash
cd orin_ws
./web.sh                        # 0.0.0.0:8080 — 다른 기기에서도 접속된다
./web.sh --port 9000 --local    # 포트 변경 + 이 기기만
./web.sh -p bag_dir:=/data/bags # 그 외 파라미터는 그대로 통과
```

브라우저에서 `http://<호스트>:8080`.

## 0. 먼저 — 브리지를 터미널에서 띄우지 않는다

**웹이 브리지를 띄우는 주체다.** `comm_test_node` 가 이미 돌고 있으면 웹이 기동을
거부한다 (*"브리지가 이미 떠 있다 — 이 웹이 띄운 것이 아니다"*). 고아 프로세스를
웹이 조작하면 누가 그것을 내릴 책임인지가 사라지기 때문이다.

브리지를 직접 띄우고 싶으면 웹 대신 `./run.sh` + `./cli.sh` 를 쓴다.

## 1. 공용 — 브리지 기동 패널

세 탭이 **같은 패널 하나**를 공유한다 (탭을 옮기면 패널도 따라 이동한다). 탭마다 따로
두면 select 가 두 벌이 되어 "Tab1 에서 고른 mode 와 Tab2 에서 고른 mode 가 다르다" 가
생긴다.

| 항목 | 값 | 뜻 |
|---|---|---|
| `bridge_mode` | project / **control** / manual | 기동 시 **고정**. 바꾸려면 재시작 |
| `auto_mode` | none / **current** / direct / velocity / position | write 범위가 여기서 파생된다 |
| `read_preset` | **control** / diag / control_test | 무엇을 읽는가 |
| `active_motors` | `1` `1,2` … | 쉼표 구분. 여기 없는 모터는 명령을 받지 않는다 |

**웹이 파라미터를 검증하지 않는다.** 브리지의 기동 게이트가 이미 전수 검증하고 사유까지
로그로 남긴다. 오타를 넣으면 브리지가 뜨지 않고 그 이유가 아래 로그창에 그대로 나온다 —
웹에서 한 번 더 검사하면 두 벌이 되고, 둘이 갈라지면 웹이 통과시킨 값을 브리지가
거부하거나 그 반대가 된다.

기동 실패는 로그창(마지막 줄에 실행된 커맨드가 그대로 찍힌다)에서 확인한다.

## 2. Tab1 — 실시간

### ① 현재 자세

모터별 `fb_position` / `fb_velocity` / `fb_current` / 온도. **NaN 은 미판독이다** —
현재 읽기 프리셋이 그 구간을 안 읽으면 0 이 아니라 NaN 으로 나온다 (0 은 "정지"라는
유효한 관측값이므로 섞으면 안 된다).

### ② 원점 · ④ run · ⑥ stop

| 버튼 | 하는 일 | 조건 |
|---|---|---|
| **② 원점 잡기** | `SET_ORIGIN` 펄스 | `auto_mode: direct` + `safe_stop` |
| **④ run** | `arm on` — STREAM 게이트를 연다 | — |
| **⑥ stop** | `arm off` — 게이트를 닫고 IDLE 로 | — |

**④ run 을 누르지 않으면 슬라이더가 아무 일도 하지 않는다.** 값은 브리지에 도착하지만
FSM 이 STREAM 으로 가지 않아 write 소스가 되지 않는다.

- ④ run 은 **IDLE 에서만** 먹는다. ⑥ stop 은 언제나 먹는다.
- 슬라이더 값이 0.1초 이상 안 오면 IDLE 로 내려가고 **arm 도 꺼진다** — 브라우저를 닫거나
  네트워크가 끊기면 자동으로 멈춘다는 뜻이다. 재개하려면 ④ run 을 다시 누른다.
- Tab2 의 프로파일 재생은 **arm 과 무관하다** (action goal 이라 별 경로다).

### ③ 슬라이더 · ⑤ 조작

슬라이더의 **단위는 모드가 정한다**:

| `auto_mode` | 슬라이더 단위 | UI 범위 · step |
|---|---|---|
| `current` | A | −20 ~ 20, 0.1 |
| `velocity` | RPM | −300 ~ 300, 1.0 |
| `position` | deg | −180 ~ 180, 0.5 |
| `direct` | **모터마다 다르다** — 그 모터의 `ctr_mode` 가 정한다 | 위 표에 따름 |
| `none` | 조작 대상이 아니다 | — |

- **`자세로`** 버튼: 그 모터를 다시 현재 피드백을 따라가게 한다 (건드리기 전 상태로).
- **`throttle`** 체크박스: 발행 주기를 낮춘다.

> ### ⚠ 슬라이더에는 레이트 제한이 없다
>
> 위 범위·step 은 **UI 값이지 안전 한계가 아니다.** 프로파일 재생에는 `limits.slew_rate`
> 가 있지만 실시간 경로에는 대응물이 없다.
>
> position 모드에서 슬라이더를 빠르게 끌면 tick 당 명령이 크게 점프하고, 모터가 그것을
> 따라가려 큰 전류를 뽑아 자기 `error_code` 를 세운다. 그러면 ECU 가 AUTO 를 벗어나
> (`ESTOP_SW`) 모든 모터 명령을 거부하고, 브리지는 0.25초 뒤 **LOCKED** 로 래치한다.
>
> **position 은 천천히 움직인다.** current 모드는 명령이 곧 전류라 상한이 명확하지만
> (`cmd_current_max`), position 은 "거기로 가라"는 명령이고 그 과정의 전류는 모터가
> 정한다 → [05_troubleshooting.md](05_troubleshooting.md#locked-가-떴다)

### 모니터 (그래프)

- 필드 검색창에 `fb_current`, `dt_` 처럼 넣어 계열을 고른다. 계열 목록은 **메시지 정의에서
  자동으로 뽑는다** (손으로 적지 않으므로 필드를 추가하면 그래프에도 바로 나온다).
- 시간축은 **브리지가 준 `header.stamp`** 다 (ECU 취득 시각을 Orin 축으로 변환한 값).
  웹 수신 시각을 쓰면 200Hz 스트림의 지터가 그대로 x축에 실린다.
- 값이 **`null` 이면 선을 끊는다** — 미판독이라 보간하면 없는 데이터를 그리게 된다.
- 다운샘플 구간은 `[min, max]` 로 보존한다 (가운데 점 하나로 그리면 스파이크가 사라진다).

상단 pill 의 `clock` 이 미수렴이면 `header.stamp` 가 Orin 수신 시각 fallback 이다 —
**시간축 등급이 다른 런**이므로 분석에서 수렴 런과 섞지 않는다.

## 3. Tab2 — 계획형 (프로파일)

### 흐름

```
세그먼트 편집 → [YAML 보기] 로 확인 → [재생] → 진행바 → result 표시
                                        └ 기록 체크 시 실험 폴더 생성
```

브라우저는 **구조(JSON)** 만 보내고 서버가 PyYAML 로 덤프한다. 브라우저에서 문자열을
이어 붙이면 따옴표·부동소수 표기·들여쓰기 때문에 "웹으로 만든 프로파일만 파싱이 다르다"
가 생긴다.

### 상단 설정

| 항목 | 뜻 |
|---|---|
| **이름** | 실험 라벨 → 폴더명이 된다 |
| **mode** | current / velocity / position — **브리지 `auto_mode` 와 같아야 goal 이 수락된다** |
| **max_abs** | 절대값 상한 |
| **range** | position 모드에서 **필수** (lo ~ hi) |
| **slew_rate** | 단위/초 레이트 제한 (선택) |
| **seed** | noise·prbs 재현용 (미지정이면 브리지가 정하고 result.json 에 남는다) |

### 세그먼트 타입

| 타입 | 인자 |
|---|---|
| `hold` | duration, value |
| `ramp` | duration, from, to |
| `step` | duration, from, to, t_step |
| `sine` | duration, amp, freq |
| `chirp` | duration, amp, f0, f1 |
| `prbs` | duration, low, high, bit_duration |
| `noise` | duration, mean, std |
| `stair` | step_duration, values |
| `custom` | samples |

UI 의 인자 검사는 **안내용**이다 — 진짜 검증은 브리지가 하고(`rd_profile.cpp`) 거부 사유가
그대로 올라온다. 여기서 막는 것은 "인자를 안 채우고 보낸 것"뿐이다.

`noise` 구간은 **slew 검사가 면제**되고, 그 tick 수가 `result.json` 의
`slew_exempt_ticks` 에 남는다 (그 구간엔 레이트 제한이 없었다는 사실을 분석이 알아야 한다).

### 기록

`기록` 체크박스를 켜면 CLI 와 **같은 모듈**로 실험 폴더가 만들어진다:

```
data/rosbags/<이름>_<MM-DD_HH-MM>/
  bag/          profile.yaml          result.json
```

**거부된 런도 기록한다** — 왜 거부됐는지가 데이터다.

진행 상태는 폴링으로 온다(1~10Hz). 재생 중 새로고침해도 상태가 이어진다.

## 4. Tab3 — 레지스터 맵

ECU 256B 레지스터를 주소·타입·값으로 그대로 본다.

- **`전체 읽기`** — 스냅샷 1회. `2초마다 자동` 으로 반복.
- **회색 + 기울임** 행은 **stale** 이다 — 지금 프리셋이 그 구간을 안 읽었거나
  센서 `delta_tick` 이 `0xFF` 다. **값이 그럴듯해도 믿지 않는다** (12bit 범위 안의
  잔값이 남아 있는 경우가 실제로 있었다 — 엔코더 ch1~4).
- 값을 고쳐 `선택한 값 전송` 으로 쓴다. `취소` 로 되돌린다.

### 이 탭이 무엇을 처음 드러냈는가

`degraded_cnt[4] = 100%` (엔코더 I2C 전량 실패)가 **여기서 처음 보였다.** 종전 control
프리셋이 SYS 블록(16:17)을 읽지 않아 진단값이 전부 미판독으로 나가고 있었고, 그래서
버스 고장이 몇 주 동안 아무에게도 보이지 않았다. 지금은 **전 프리셋이 SYS 를 읽는다.**

상시로 봐야 하는 것: `hw_error` / `hw_fatal` / `degraded_cnt[]` / 채널별 `lc`·`hs`.

## 5. HTTP API (자동화용)

웹 UI 없이 스크립트로 붙일 때. 응답은 전부 JSON.

| 메서드 | 경로 | 뜻 |
|---|---|---|
| GET | `/api/state` | FSM·모드·피드백·브리지 상태 (1급) |
| GET | `/api/stream` | SSE — 50Hz 프레임 |
| GET | `/api/fields` | 그래프 계열 목록 |
| GET | `/api/registers` | 256B 스냅샷 + 신선도 |
| GET | `/api/profile/status` | 재생 진행 |
| POST | `/api/setpoint` | 슬라이더 값 |
| POST | `/api/release` | `자세로` |
| POST | `/api/arm` | run/stop |
| POST | `/api/origin` | 원점 |
| POST | `/api/command` | 커맨드 슬롯 |
| POST | `/api/bridge/start` `/stop` | 브리지 기동·정지 |
| POST | `/api/profile/preview` `/run` `/abort` | 프로파일 |

POST 본문은 **`{"params": {...}}`** 로 감싼다.

```bash
curl -s localhost:8080/api/state | python3 -m json.tool
curl -s -X POST localhost:8080/api/arm -d '{"params":{"on":true}}'
```

## 6. 알아 두면 좋은 것

- **웹은 브리지 없이도 산다.** 브리지 부재와 "대답이 늦다" 를 구분해서 보여준다.
- QoS 는 브리지에 맞춘 RELIABLE 이다. 여기를 BEST_EFFORT 로 바꾸면 호환되지 않아
  구독이 조용히 붙지 않는다.
- 그래프는 자체 캔버스 렌더러다 (외부 라이브러리 없음) — `null`(선 끊기)과
  `[min,max]`(극값 보존)를 1급으로 다뤄야 했기 때문이다.
- `bag_dir` 기본값은 **레포 루트 기준** `data/rosbags` 다. `web.sh` 가 CWD 를 레포 루트로
  옮긴다 — 안 그러면 웹은 `orin_ws/data/rosbags`, CLI 는 `tp_ws/data/rosbags` 에 쌓여
  기록이 두 곳으로 갈라지고 그 사실은 분석할 때가 되어서야 드러난다.
