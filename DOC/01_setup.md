# 01 — 설치 · 빌드 · 실기 세팅

## 1. 무엇이 필요한가

| 항목 | 값 | 확인 |
|---|---|---|
| ROS 2 | Humble | `ls /opt/ros/humble/setup.bash` |
| ECU 포트 | `/dev/ttyUSB0` (FTDI) | `ls -l /dev/ttyUSB*` |
| 시스템 패키지 | `libserial-dev`, `libyaml-cpp-dev` | 빌드가 없으면 알려준다 |
| STM 툴체인 | **불필요** (이 워크스페이스에서 ECU 펌웨어를 빌드하지 않는다) | — |

다른 ROS 배포판이면 `ROS_SETUP=/opt/ros/<distro>/setup.bash` 로 지정한다.

## 2. 빌드

```bash
cd orin_ws
./build.sh                          # 전체 (의존 순서대로)
./build.sh control_web control_cli  # 일부만
./build.sh --test                   # 전체 + 단위 테스트
```

### 왜 순서가 중요한가

`build.sh` 는 **메시지 패키지를 먼저, 따로** 빌드하고 `install/setup.bash` 를 다시
source 한 뒤 나머지를 빌드한다. colcon 이 의존 순서를 잡아 주긴 하지만, 생성된
헤더·파이썬 모듈은 setup.bash 를 재-source 해야 다음 패키지가 본다 — 그래서 깨끗한
트리에서 한 번에 몰아 빌드하면 첫 빌드가 실패한다.

```
mgs01_base_msgs, mgs_tp_msgs          (메시지)
  ↓
orin_firmware_bridge                  (브리지)
control_cli                            (CLI — control_web 이 이걸 import 한다)
control_web                            (웹)
carrier_teleop
```

### ⚠ 반드시 `orin_ws` 에서 실행한다

`colcon` 은 실행한 디렉터리를 워크스페이스 루트로 본다. `src/` 안에서 돌리면
`src/<pkg>/build/`·`install/` 이 생기고, 그 유령 install 이 나중에 import 돼서
**고친 코드가 아닌 옛 코드가 도는** 상황이 만들어진다. `build.sh` 는 스크립트 자기
위치에서 워크스페이스를 찾으므로 어디서 호출해도 안전하다.

## 3. 실기 세팅 (최초 1회 + 재부팅마다)

```bash
./setup_rt.sh          # latency_timer + 포트 권한 점검
./setup_rt.sh --perm   # + dialout 그룹 추가 (재로그인 필요)
```

### latency_timer 가 왜 중요한가

FTDI 기본값은 **16ms** 다. 200Hz 는 5ms 주기이므로, 응답이 최대 16ms 늦게 올라와
tick 이 통째로 밀린다. 1로 낮춘다.

```bash
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer   # 1 이어야 한다
```

**재부팅하면 16으로 되돌아간다.** 그래서 이것이 빌드가 아니라 별도 스크립트에 있다.

### 포트 권한

USB 가 재열거되면 권한이 조용히 바뀐다. 권한이 없으면 브리지는
`Open Failed: Bad file descriptor` 만 남기며 무한 재시도한다 — 로그만 보면
"ECU 가 응답하지 않는다" 처럼 보인다.

```bash
ls -l /dev/ttyUSB0        # crw-rw---- root dialout 이면 그룹 확인
groups                    # dialout 이 있어야 한다
sudo usermod -aG dialout $USER && echo "재로그인 필요"
```

### SCHED_FIFO

브리지가 스스로 자기 스레드에 `SCHED_FIFO pri=80` + CPU 코어 고정을 건다. 권한이
없으면 **경고만 남기고 일반 우선순위로 계속 돈다** — 즉 조용히 성능만 떨어진다.

```bash
ulimit -r        # 80 이상이어야 한다. 0 이면 아래 설정
```
```
# /etc/security/limits.conf
<사용자>  -  rtprio  99
```

Orin 신규 이관은 [ORIN_SET_GUIDE.md](../ORIN_SET_GUIDE.md) 를 그대로 따른다.

## 4. Orin 으로 배포

개발 머신에서 코드를 쏘고, 빌드는 Orin 에서 한다.

```bash
./deploy.sh                       # src/ 전체 + 스크립트 전송
./deploy.sh --host 192.168.55.1   # USB 이더넷
./deploy.sh --dry                 # 무엇이 갈지만
./deploy.sh --build               # 전송 후 원격 빌드까지
```

환경변수로도 지정된다: `ORIN_HOST` / `ORIN_USER` / `ORIN_WS`.

`src/` 를 **통째로** 보낸다. 패키지를 나열하면 새 패키지가 생길 때 여기를 고쳐야 하고,
안 고치면 원격 빌드가 깨질 때까지 아무도 모른다 (실제로 그런 상태였다 — 5개 중 2개만
보내고 있었다). `build/`·`install/`·`__pycache__` 는 제외한다.

## 5. 스크립트 한 장 정리

| 스크립트 | 하는 일 |
|---|---|
| `_env.sh` | 공통 — 워크스페이스 위치·ROS source·실행파일/포트 점검. **직접 실행하지 않는다** |
| `build.sh` | 빌드 (`--test` 로 단위 테스트) |
| `setup_rt.sh` | sudo 가 필요한 런타임 세팅 (`--perm`) |
| `web.sh` | 웹 기동 (`--port` `--local` `--bind`) |
| `run.sh` | 브리지 **직접** 기동 — 웹 없이 CLI 로 쓸 때 |
| `cli.sh` | `control_cli` 실행 |
| `rosbag_test.sh` | 수동 조작용 임시 bag (프로파일 실험은 `cli.sh run --record`) |
| `deploy.sh` | Orin 전송 |

모든 스크립트가 **자기 위치에서 워크스페이스를 찾는다.** 종전에는
`/home/swarm/tp_ws/...` 가 하드코딩돼 있어 개발 머신에서 돌지 않았다.

## 6. 기동 확인

```bash
./run.sh project                     # 가장 단순한 모드
ros2 topic hz /carrier/ecu/status    # 10Hz 가 나와야 한다
./cli.sh status                      # control 모드에서만 의미가 있다
```

`run.sh` 의 모드:

```bash
./run.sh                        # project (기본 주행)
./run.sh control current        # 제어 실험 — auto_mode 지정
./run.sh traction               # 견인 실험 = control + auto_mode:none + control_test 프리셋
./run.sh manual                 # 자동 설정이 전무한 백지 모드
./run.sh control direct -p active_motors:=1,2   # 그 외 파라미터는 그대로 통과
```

`traction_test_mode` 파라미터는 **폐지됐다.** 지금 그것을 주면 브리지가 기동 게이트에서
거부하고 종료한다(exit≠0). 같은 배치는 위의 `traction` 프리셋이다.
