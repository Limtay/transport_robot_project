# 03 — CLI (`control_cli`)

터미널에서 조작한다. 웹이 하는 일 전부를 여기서도 할 수 있고, 스크립트에 넣기 쉽다.

```bash
cd orin_ws
./run.sh control current &   # 브리지를 먼저 띄운다 (웹과 달리 CLI 는 브리지를 안 띄운다)
./cli.sh status
```

> **웹과 동시에 쓰지 않는다.** 둘 다 `/dev/ttyUSB0` 을 열려 하고, 웹은 자기가 띄우지 않은
> 브리지를 조작하기를 거부한다. 하나를 고른다.

## 서브커맨드 한 장 정리

| 커맨드 | 하는 일 |
|---|---|
| `status [--json]` | FSM·모드·mask·ctr_mode·safe_stop 요약 |
| `arm on\|off` | STREAM 게이트 (웹의 ④ run / ⑥ stop) |
| `rearm` | LOCKED → IDLE |
| `abort` | 진행 중 프로파일 취소 |
| `origin` | `SET_ORIGIN` 펄스 |
| `config <what> <값...>` | 단발 설정 (**IDLE 에서만**) |
| `stream <what> <값...>` | 주기 발행 (웹 슬라이더 대응물) |
| `run <yaml> [--record]` | 프로파일 재생 |
| `command set\|reset ...` | 커맨드 슬롯 |

---

## `status`

```bash
./cli.sh status
```
```
state=IDLE write=NONE bridge=control ecu=AUTO(2)
  auto_mode=current write=164:16 preset=control mask=0x01 motors=[1]
  ctr_mode=['current', 'unknown', 'unknown', 'unknown']
  safe_stop=True
```

`--json` 은 정형 JSON 원문이다. **자동화는 반드시 이쪽을 쓴다** — 사람용 요약은 JSON 을
파싱해서 만든 것이고, 문장 형식은 보장하지 않는다.

```bash
./cli.sh status --json | python3 -c 'import json,sys; print(json.load(sys.stdin)["control_state"])'
```

주요 키: `control_state` `write_source` `bridge_mode` `ecu_mode` `ecu_sys_state`
`auto_mode` `write_span` `read_preset` `motor_mask` `active_motors` `ctr_mode`
`safe_stop` `safe_stop_detail` `stamp_valid` `rtt_ms` `slots[]`.

`safe_stop` 이 false 면 **사유가 같이 나온다** — `origin` 이 왜 거부되는지 여기서 바로 보인다.

## `arm` — 게이트

```bash
./cli.sh arm on      # STREAM 진입 허용
./cli.sh arm off     # 닫고 IDLE 로
```

`on|1|true|yes` 를 켜짐으로 읽는다. **`arm` 없이는 `stream` 이 아무 일도 하지 않는다.**

- `arm on` 은 **IDLE 에서만** 된다. `arm off` 는 언제나 된다 (감속 방향이므로).
- `arm on` 시점에 남아 있던 스트림 값은 무효화된다 — 안 그러면 arm 하자마자 옛 명령이 나간다.
- 스트림이 끊기면(기본 0.1초) IDLE 로 내려가고 **arm 도 함께 꺼진다.** 재개하려면 다시 `arm on`.
- **프로파일 재생(`run`)에는 arm 이 필요 없다** — action goal 은 별 경로다.

## `config` — 단발 설정 (IDLE 에서만)

```bash
./cli.sh config motors 1 2          # active_motors = M1,M2
./cli.sh config ctr_mode 2 3 1      # M2·M3 의 ctr_mode = 1   (마지막 인자가 mode)
./cli.sh config auto_mode 2         # auto_mode 교체
./cli.sh config mode 2              # ECU mode 레지스터
./cli.sh config preset 1            # 읽기 프리셋 — 0=control 1=diag 2=control_test
```

`preset 1`(diag) 로 갈아끼우면 `hw_error`·`hw_fatal`·`DIAG` 블록이 실값으로 바뀐다.

## `stream` — 실시간 조작

```bash
./cli.sh arm on
./cli.sh stream current 2.0                 # 전 모터 2.0 A
./cli.sh stream position 0 45 0 0           # 모터별 4개
./cli.sh stream velocity 100 --rate 50 --duration 10
```

`--rate` 기본 20Hz, `--duration` 기본 **5초** (`duration` 은 초다 — 시도 횟수가 아니다).

> **⚠ 레이트 제한이 없다.** 웹 슬라이더와 같은 경로이며 같은 위험을 갖는다. position 에서
> 큰 값을 갑자기 주면 모터 fault → ECU `ESTOP_SW` → LOCKED 로 갈 수 있다.
> 값을 나눠서 올린다 (예: 0 → 15 → 30 → 45).

## `run` — 프로파일 재생

```bash
./cli.sh run data/profiles/step_5A.yaml --record
./cli.sh run p.yaml --record --name s1_w20 --bag-dir /data/bags
./cli.sh run p.yaml --record --diag        # comm_diag 도 기록
```

| 옵션 | 뜻 |
|---|---|
| `--record` | bag + YAML 사본 + `result.json` 을 실험 폴더로 |
| `--name` | 실험 라벨 (기본: 파일명) |
| `--bag-dir` | 기록 루트 (기본 `data/rosbags`) |
| `--diag` | `comm_diag` 토픽도 기록 — **브리지를 `comm_diag_enable:=true` 로 띄운 경우만** |

`--diag` 없이는 `/carrier/control/feedback` **1개**만 기록한다. `comm_diag` 발행자는
`comm_diag_enable` 이 true 일 때만 만들어지므로, 무조건 요청하면 bag 메타데이터에 0건
토픽이 남아 나중에 "계측이 왜 비었나" 를 브리지가 아니라 bag 에서 찾게 된다.

### 종료 코드

| 코드 | 뜻 |
|---|---|
| 0 | 성공 |
| 1 | 재생 실패 (goal 거부 / 중단) |
| 2 | 인자·환경 오류 (파일 없음, action 서버 없음, 타임아웃) |
| 3 | **기록 검증 실패** — 특히 `profile.yaml` 사본의 sha256 이 재생된 것과 다르다 |

**코드 3 이 나온 런은 분석에 쓰지 않는다.** 무엇을 재생했는지 확정할 수 없다는 뜻이다.

## `command` — 커맨드 슬롯

의미 단위 명령을 슬롯에 걸어 주기적으로 보낸다.

```bash
./cli.sh command set auto 3 ecu read_all       # 슬롯 자동배정, 3회, ECU 전 구간 읽기
./cli.sh command set 0 5 ecu read_motor        # 슬롯 0, 5회
./cli.sh command reset 0
```

형식: `set <slot|auto> <dur> <target> <cmd> [인자...]`

| cmd | 뜻 | 제약 |
|---|---|---|
| `read_sys` `read_motor` `read_sensor` `read_diag` `read_all` | 의미 단위 읽기 | 어느 모드에서든 안전 |
| `set_soft_estop` `set_use_lpf` `reboot` | 설정·재시작 | — |
| `raw_read` `raw_write` | **주소 직접 지정** | **`manual` 모드 전용** |

위험은 **cmd 이름**이 말한다. `raw_*` 만 manual 전용이고 나머지 의미 단위 명령은 안전하다.
`raw` 서브커맨드는 `command` 의 폐지 예정 별칭이다.

레지스터를 직접 읽는 예 (엔코더 블록 70~85):

```bash
./run.sh manual &
./cli.sh command set auto 3 ecu raw_read 70 16
./cli.sh status --json     # 결과는 레지스터 스냅샷으로 확인
```

## `origin` / `rearm` / `abort`

```bash
./cli.sh origin      # DIRECT + safe_stop 필요. status 의 safe_stop_detail 이 사유를 준다
./cli.sh rearm       # LOCKED 해제 — 원인을 확인한 뒤에 한다
./cli.sh abort       # 진행 중 프로파일 취소
```

## 자동화 예시

```bash
#!/bin/bash
set -euo pipefail
REPO="$(cd "$(dirname "$0")" && pwd)"      # 레포 루트
cd "$REPO/orin_ws"

./run.sh control current & BR=$!
trap 'kill -INT $BR' EXIT
until ./cli.sh status --json >/dev/null 2>&1; do sleep 0.5; done

for f in "$REPO"/data/profiles/*.yaml; do   # ← 절대경로. 이유는 아래 ⚠
    st=$(./cli.sh status --json | python3 -c 'import json,sys; print(json.load(sys.stdin)["control_state"])')
    [ "$st" = "LOCKED" ] && { echo "LOCKED — 중단"; break; }
    ./cli.sh run "$f" --record --name "$(basename "$f" .yaml)" || echo "실패: $f (계속)"
done
```

**매 런 전에 `control_state` 를 본다.** LOCKED 인 채로 계속 돌리면 전부 거부되면서
쓸모없는 폴더만 쌓인다.

> ### ⚠ 경로는 절대경로로 넘긴다
>
> `cli.sh` 는 **CWD 를 레포 루트로 옮긴 뒤** `control_cli` 를 실행한다 (`data/rosbags` 가
> 상대경로라서, 옮기지 않으면 웹과 CLI 가 서로 다른 곳에 bag 을 쌓는다).
>
> 그래서 `orin_ws` 에서 `./cli.sh run ../data/profiles/x.yaml` 은 **동작하지 않는다** —
> 셸이 `../` 를 `orin_ws` 기준으로 확장해 넘기지만, `control_cli` 는 레포 루트에서 그것을
> 다시 해석해 한 단계 더 위를 본다.
>
> 규칙: **`cli.sh` 에 넘기는 파일 경로는 레포 루트 기준이거나 절대경로여야 한다.**
> `orin_ws` 에서라면 `./cli.sh run data/profiles/x.yaml` 이 맞다 (`../` 없이).
