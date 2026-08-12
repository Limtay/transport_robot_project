# Orin 세팅 가이드 — orin_firmware_bridge (comm_test_node)

> **이 문서 = 일회성 시스템 세팅**(빌드·RT·latency 영구화). 노드를 **직접 띄우고 실험 돌리는 운용법**은
> → [ORIN_USER_GUIDE.md](ORIN_USER_GUIDE.md). 둘은 짝 문서다(세팅 ↔ 운용).
>
> 대상: Jetson Orin (12코어, JetPack/L4T, 커널 5.15-tegra `CONFIG_PREEMPT=y`)
> 목적: 200Hz 제어 루프(`/dev/ttyUSB0`, FTDI RS485 921600)가 통신 끊김/주기 밀림 없이
> 도는 상태로 만들기 위한 전체 설정 절차. 신규 Orin 이관/재설치 시 이 문서대로 진행.
>
> 최종 검증 상태: `over-period 0.0%`, max 사이클 ~1.9ms, Loss≈0, ECU ON, FATAL 0.

---

## 0. 한눈에 — 필요/불필요

| 항목 | 판정 | 비고 |
|---|---|---|
| 코드 수정 (tcdrain→상한대기, RT 적용) | **필수** | 소스에 반영됨. `build.sh`로 빌드 |
| `latency_timer = 1` | **필수** | 통신 손실 해결. udev로 영구화 권장 |
| `rtprio 99` (limits.conf) | **필수** | SCHED_FIFO 권한 |
| SCHED_FIFO + core11 affinity (코드 내장) | **유지** | 선점 방지 보험. 공짜. rtprio 필요 |
| 전원모드 MAXN (`nvpmodel -m 0`) | 권장 | 성능 일관성 |
| `isolcpus=11` 커널 격리 | **불필요** | 스파이크 원인 아님. 재부팅 위험. 보류 |
| `jetson_clocks` | **불필요** | 효과 없음(오히려 악화) |
| `taskset` 코어 수동 제한 | **불필요** | 디버깅용 임시조치였음 |

---

## 0.5 빌드 선행 의존성 (신규 머신 공통 — Orin·개발머신)

```bash
sudo apt install libserial-dev      # RS485 시리얼 (rd_uart, pkg-config `libserial`)
sudo apt install libyaml-cpp-dev    # 프로파일 YAML 파싱 (rd_profile, testbed_spec §4)
```

없으면 colcon 이 pkg-config / find_package 단계에서 실패한다 (2026-07-19 개발머신에서 libserial 미설치로 실측).

---

## 1. 시스템 구성 (참고)

- 노드: `comm_test_node` (ROS2 노드명 `firmware_bridge_node`), 패키지 `orin_firmware_bridge`.
- 역할: Orin = 마스터(ID 0x01), RS485/USB-시리얼(`/dev/ttyUSB0`, FTDI `ftdi_sio`, 921600 8N1)로
  ECU/DPC/PCU에 req→resp.
- 제어 루프: `RdSchedule::RunLoop()` — 200Hz tick(5ms), 100Hz READ / 50Hz WRITE / 10Hz sys / 5Hz cmd×4.
- 핵심 파일:
  - `src/.../rd_schedule.cpp` — 루프/타이밍/RT/하트비트 통계
  - `src/.../rd_uart.cpp` — 시리얼 open/read/write, RS485 턴어라운드
  - `src/.../rd_comm.cpp` — 패킷 조립/CRC
  - `src/.../main.cpp` — `RdUart("/dev/ttyUSB0")`, `scheduler.MainLoopStart()`

---

## 2. 일회성 시스템 설정 (신규 Orin에서 1번만, sudo 필요)

### 2-1. rtprio 권한 (SCHED_FIFO 필수)
SCHED_FIFO를 sudo 없이 쓰려면 유저에 rtprio 한도를 줘야 한다. 없으면 코드가
`SCHED_FIFO 설정 실패` WARN 후 일반 우선순위로 떨어진다(부하 시 밀림 발생).
```bash
echo 'swarm  -  rtprio  99' | sudo tee -a /etc/security/limits.conf
# 로그아웃→재로그인 (또는 재부팅) 후:
ulimit -r        # 99 확인
```

### 2-2. latency_timer = 1 (통신 손실 방지 필수)
FTDI는 수신 바이트를 호스트로 보내기 전 기본 16ms 버퍼링 → read 타임아웃(2ms) 초과 →
Loss 폭증. **재부팅/USB 재연결 시 16으로 리셋되므로 udev로 영구 고정** 권장.
```bash
echo 'ACTION=="add", SUBSYSTEM=="usb-serial", ATTR{latency_timer}="1"' | \
  sudo tee /etc/udev/rules.d/99-ftdi-latency.rules
sudo udevadm control --reload && sudo udevadm trigger
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer   # 1 확인
```
> udev 적용 전 임시값: `echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer`
> (현 `build.sh`에 이 임시 설정이 들어 있음. udev 적용 후엔 제거 가능.)

### 2-3. 전원 모드 MAXN (권장)
```bash
sudo nvpmodel -m 0     # MAXN
nvpmodel -q            # 확인
```
> `jetson_clocks`(클럭 고정)는 이 루프 타이밍엔 **도움이 안 됐으므로 적용하지 않는다.**

---

## 3. 코드에 반영된 수정 (참고 — 이미 소스에 있음)

### 3-1. RS485 턴어라운드: tcdrain → 계산된 상한 대기  ★주된 해결
`rd_uart.cpp RdUart::Write()`. FTDI USB-시리얼에서 `tcdrain()`이 데이터량과 무관하게
**8~13ms 블록**(12B 실제 전송은 ~130µs) → 200Hz 주기 초과의 단독 원인이었다.
tcdrain을 제거하고 `전송 비트시간 + USB 마진(kTxUsbMarginUs)` 만큼만 대기.
```
const int64_t tx_bits_us = length * 10 * 1000000 / 921600;   // 8N1, 921600
std::this_thread::sleep_for(microseconds(tx_bits_us + kTxUsbMarginUs)); // 마진 1500us
```
- **튜닝**: `rd_uart.cpp`의 `kTxUsbMarginUs` (기본 1500). Loss 늘면 ↑(2000~3000),
  더 조이려면 ↓(1000)하며 Loss=0 유지되는 최소값 탐색.

### 3-2. 제어 스레드 RT (선점 방지 보험)
`rd_schedule.cpp MainLoopStart()`가 `ApplyRtScheduling(pthread_self())` 호출 →
`SCHED_FIFO(pri=80)` + `CPU affinity(core 11)`. rtprio 권한(2-1) 있어야 적용됨.
- `rd_schedule.hpp`: `kRtPriority=80`, `kCpuCore=11`(12코어 0~11 기준).
  **코어 수가 12 미만인 보드면 `kCpuCore`를 (nproc-1)로 수정** 후 재빌드.

### 3-3. 하트비트 타이밍 통계 (진단 가시성)
RT 루프 안 per-tick `RCLCPP_WARN` 제거(동기 로깅 지터원), 대신 400tick(~2s) 구간 통계 출력.
`평균>4ms`면 구간당 1회 경고. (진단용 `Spike:`/`I/O:` 줄은 원하면 제거 가능, 기능 영향 없음.)

---

## 4. 빌드 & 실행

```bash
cd ~/orin_ws
./build.sh        # realsense + mgs01_base_msgs + orin_firmware_bridge 빌드
                  # (latency_timer 임시설정 위해 sudo 암호 물을 수 있음)
./run.sh          # comm_test_node 실행 — taskset/추가옵션 불필요
```
- 코드만 고쳤을 때 빠르게:
  `colcon build --packages-select orin_firmware_bridge && source install/setup.bash`

---

## 5. 검증 — 하트비트 읽는 법

정상 시작 로그:
```
[INFO] ... SCHED_FIFO pri=80 적용됨.
[INFO] ... CPU affinity core 11 고정됨.
```
하트비트(약 2초마다):
```
====== [RdSchedule Heartbeat] ======
 Ticks : 2455
 Timing: avg 1000 us / max 1900 us | over-period 0/400 (0.0%)   ← 목표
 Spike : wake_max 15 us / proc_max 1850 us
 I/O   : clear_max 12 / write_max 1670 / read_max 400 us
 Comm  : Tx .. / Rx .. (Loss: 0~수개)                            ← 목표
 Nodes : ECU[ON]  DPC[OFF]  PCU[OFF]
====================================
```
목표치: `over-period ≈ 0%`, `max < 5000us`, `Loss ≈ 0`, `ECU[ON]`, FATAL 0.

---

## 6. 트러블슈팅 (증상 → 원인 → 조치)

| 증상 | 원인 | 조치 |
|---|---|---|
| `SCHED_FIFO 설정 실패` WARN | rtprio 권한 없음 | §2-1 (limits.conf + 재로그인) |
| `CPU affinity 설정 실패` WARN | 코어 수 부족 | `kCpuCore`=nproc-1로 수정 후 재빌드 |
| `Loss` 폭증(수백) | latency_timer=16 | §2-2 (udev / 임시 echo 1) |
| `over-period` 다시 증가 | turnaround 마진 과소/과대 부하 | `kTxUsbMarginUs` 점검, 다른 노드 core11 점유 확인 |
| `write_max`가 8~13ms | tcdrain이 다시 들어옴 | rd_uart.cpp 수정 반영/빌드 확인 |
| CRC/Sync/RX Timeout 빈발 | turnaround 마진 과소 | `kTxUsbMarginUs` ↑ (2000~3000) |
| 노드가 USB 끊김 후 사망 | 포트 자동 reopen 없음 | (개선과제 §8) |

---

## 7. 불필요/보류 항목 (혼동 방지)

- **isolcpus=11 / nohz_full**: 13ms 스파이크는 스케줄링이 아니라 tcdrain이었으므로 **불필요**.
  커널 cmdline 편집은 부팅 위험이 있어 보류. 안전 적용 스크립트 `apply_isolcpus.py`는
  나중에 다른 노드가 core11과 경쟁할 때를 대비해 **보관만** (지금 실행 안 함).
  - 만약 훗날 적용한다면: `sudo python3 apply_isolcpus.py` (primary 라벨 보존 + rt 라벨 추가,
    부팅 메뉴 30초 내 primary 선택으로 원복 가능). `nohz_full`은 이 커널 미지원.
- **jetson_clocks**: 타이밍에 도움 안 됨(측정상 악화). 재부팅 시 자동 해제되며 재적용 불필요.
- **taskset 코어 수동 제한**: 디버깅 중 측정 격리용이었음. 평시 `./run.sh`엔 불필요.

---

## 8. 남은 개선 과제 (추후, 선택)

- `RdUart`에 USB 끊김 시 **포트 자동 reopen** 로직 추가(현재 끊기면 노드 사망).
- `latency_timer` 설정을 udev로 옮긴 뒤 `build.sh`의 임시 echo 라인 제거.
- 하트비트 진단줄(`Spike:`/`I/O:`) 운영 빌드에선 정리.

---

## 9. 기타 Orin 설정 (확장 — 프로젝트별 추가 기재)

> 이 절은 firmware_bridge 외 Orin 전반 설정을 모으는 곳. 필요한 항목을 추가 기재.

- [ ] RealSense 카메라 (`run_camera.sh`) — 실행/네트워크/USB 대역 메모
- [ ] ROS2 DDS 설정 (도메인 ID, Fast DDS 프로파일, 공유메모리/멀티캐스트)
- [ ] 부팅 시 자동 실행 (systemd 서비스 등록)
- [ ] 네트워크/시간동기(chrony) 설정
- [ ] 사용자/그룹 권한 (dialout, gpio, i2c, video 등)
- [ ] 스토리지/스왑/로그 로테이션
- [ ] (추가...)

---

## 10. 개발 머신 ↔ 로봇 SSH 등록 (`deploy.sh` 선행 조건)

`deploy.sh` 는 rsync/ssh 로 코드를 밀어넣는다. 그 전에 **개발 머신이 로봇에 비밀번호 없이
붙을 수 있어야** 한다 — `deploy.sh` 의 사전 점검이 `BatchMode=yes`(비밀번호 입력 불가)라
키 인증이 안 되면 무조건 "SSH 로 못 붙는다" 로 막힌다.

### 10-0. 왜 별칭을 쓰는가 (로봇 2대 이상이면 필수)

Jetson 의 USB 이더넷 가젯 주소는 **기종 고정 `192.168.55.1`** 이다. 로봇이 여러 대면
전부 같은 주소를 쓴다. `known_hosts` 는 "주소 → 호스트키" 표라서, 로봇을 바꿔 꽂는 순간

```
@ WARNING: REMOTE HOST IDENTIFICATION HAS CHANGED! @
```

가 뜨고 접속이 막힌다. **키를 지우고 다시 받는 것으로 때우면 안 된다** — 로봇을 바꿀 때마다
반복되고, 그러다 경고를 습관적으로 무시하게 되어 진짜 사고를 놓친다.

해결은 **별칭마다 `known_hosts` 를 분리**하는 것. 검증을 끄는 게 아니라 보관함을 나누는
것이라, 진짜로 호스트키가 바뀌면(재설치 등) 여전히 경고가 뜬다.

### 10-1. 클라이언트 키 생성 — **노트북당 1회**

내 신분증을 만든다. 로봇 수와 무관하게 노트북마다 한 번.

```bash
ls ~/.ssh/id_ed25519 2>/dev/null && echo "이미 있음 — 건너뛴다"
ssh-keygen -t ed25519 -N "" -f ~/.ssh/id_ed25519
```

- `-N ""` = **passphrase 없음**. `deploy.sh` 가 매번 암호를 묻지 않고 돌게 하려면 필요하다.
  대신 이 노트북을 손에 넣은 사람은 로봇에 그대로 들어갈 수 있다 — 개발 노트북 기준의 선택.
- 키가 이미 있으면 **다시 만들지 말 것.** 덮어쓰면 기존에 등록해 둔 모든 서버와의 인증이 끊긴다.

### 10-2. `~/.ssh/config` 에 별칭 작성 — **노트북당 1회**

```
Host orin1
    HostName 192.168.55.1
    User swarm
    UserKnownHostsFile ~/.ssh/known_hosts.orin1

Host orin2
    HostName 192.168.55.1
    User swarm
    UserKnownHostsFile ~/.ssh/known_hosts.orin2

# 유선/무선 주소는 IP 가 서로 달라 충돌하지 않으므로 기본 known_hosts 를 쓴다.
Host orin1-lan
    HostName 10.251.24.214
    User swarm
```

```bash
chmod 600 ~/.ssh/config
ssh -G orin1 | grep -E '^(hostname|user|userknownhostsfile) '   # 해석 결과 확인
```

### 10-3. 로봇 호스트키 등록 — **로봇마다 1회**

등록할 로봇 **한 대만** USB 케이블에 연결한 상태에서:

```bash
ssh orin2          # 새로 세팅하는 로봇의 별칭
```

`Are you sure you want to continue connecting?` 에서 지문을 **확인한 뒤** `yes`.

> **지문을 제대로 검증하는 법** — 로봇에 모니터/키보드를 붙여 직접 읽는다. 이게 유일하게
> 확실한 방법이다 (네트워크 너머로 받은 값을 그 네트워크로 검증할 수는 없다).
> ```bash
> # 로봇 쪽 콘솔에서
> ssh-keygen -lf /etc/ssh/ssh_host_ed25519_key.pub
> ```
> 이 값이 노트북에 뜬 `SHA256:...` 과 같아야 한다. 직결 USB 케이블이라 위험이 낮다고 보고
> 생략할 수도 있지만, **최초 1회는 확인해 두는 편이 좋다.**

`yes` 하면 `~/.ssh/known_hosts.orin2` 가 생기고 그 로봇의 키가 들어간다. `exit` 로 나온다.

### 10-4. 공개키 등록 — **로봇마다 1회**

```bash
ssh-copy-id orin2
```

로봇의 `swarm` 비밀번호를 **한 번** 입력한다. 이후로는 비밀번호 없이 붙는다.
(내부적으로 `~/.ssh/id_ed25519.pub` 을 로봇의 `~/.ssh/authorized_keys` 에 덧붙인다.)

### 10-5. 검증

```bash
ssh orin2 true && echo "OK"                 # 아무것도 안 물으면 성공
ssh -o BatchMode=yes orin2 true && echo "BatchMode OK"   # deploy.sh 가 쓰는 방식
./deploy.sh --host orin2 --dry              # 무엇이 갈지/지워질지 미리보기
```

**첫 배포 전에 `--dry` 를 반드시 한 번 볼 것.** 로봇마다 `src/` 구성이 달라서, `*deleting`
줄에 그 로봇에만 있는 패키지가 잡히지 않는지 확인해야 한다.

### 10-6. 새 노트북에서 처음 세팅할 때 (체크리스트)

| 단계 | 명령 | 빈도 |
|---|---|---|
| 1 | `ssh-keygen -t ed25519 -N "" -f ~/.ssh/id_ed25519` | 노트북당 1회 |
| 2 | `~/.ssh/config` 에 별칭 작성 + `chmod 600` | 노트북당 1회 |
| 3 | `ssh orinN` → 지문 확인 → `yes` → `exit` | 로봇마다 1회 |
| 4 | `ssh-copy-id orinN` → 비밀번호 1회 | 로봇마다 1회 |
| 5 | `./deploy.sh --host orinN --dry` | 배포 전마다 |

3·4 는 **로봇 대수만큼 반복**한다. 로봇을 한 대 더 추가하면 `config` 에 `Host orin3` 블록을
추가하고 3·4 만 다시 하면 된다.

### 10-7. 트러블슈팅

| 증상 | 원인 | 조치 |
|---|---|---|
| `REMOTE HOST IDENTIFICATION HAS CHANGED!` | 같은 IP 에 다른 로봇이 붙음 (또는 로봇 재설치) | 10-2 별칭 분리. 이미 분리했는데도 나면 **진짜 호스트키가 바뀐 것** — 로봇 쪽에서 지문을 직접 확인할 것 |
| `Permission denied (publickey,password)` | 공개키 미등록 | `ssh-copy-id orinN` (10-4) |
| `deploy.sh` 가 "SSH 로 못 붙는다" 로 즉시 종료 | 사전 점검이 `BatchMode=yes` 라 비밀번호·호스트키 승인을 못 받음 | 10-3, 10-4 를 먼저 끝낼 것. `ssh orinN` 이 아무것도 안 물어야 통과한다 |
| `ssh: Could not resolve hostname orin2` | `~/.ssh/config` 미작성 또는 오타 | `ssh -G orin2` 로 해석 결과 확인 |
| 접속은 되는데 매번 passphrase 를 물음 | 키를 passphrase 있게 만듦 | `ssh-add ~/.ssh/id_ed25519` (세션 한정) 또는 키 재발급 |

### 10-8. 기존 `known_hosts` 를 별칭별로 나눌 때 (이미 쓰던 노트북)

로봇 1대로 쓰다가 2대가 된 경우, 기존 항목을 로봇 A 보관함으로 옮긴다.

```bash
cd ~/.ssh
cp -p known_hosts "known_hosts.bak.$(date +%Y%m%d-%H%M%S)"   # 백업 먼저
cp -p known_hosts known_hosts.orin1        # 기존 항목 = 로봇 A
ssh-keygen -R 192.168.55.1 -f known_hosts  # 본 파일에서 충돌 주소 제거
: > known_hosts.orin2                       # 로봇 B 는 빈 파일로 시작
chmod 600 known_hosts known_hosts.orin1 known_hosts.orin2
```

확인:
```bash
ssh-keygen -lf ~/.ssh/known_hosts.orin1     # 로봇 A 키 목록
ssh-keygen -F 192.168.55.1                  # 본 known_hosts 에 남아있지 않아야 함
```

> 어느 항목이 어느 로봇 것인지 헷갈리면, **로봇을 한 대씩만 꽂고** `ssh-keyscan -t ed25519
> 192.168.55.1 | ssh-keygen -lf -` 로 지문을 뽑아 대조한다.
