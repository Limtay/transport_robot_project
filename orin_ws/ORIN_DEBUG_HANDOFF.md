# Orin firmware_bridge 디버그 핸드오프

> 작성: 2026-06-15 / 노트북(swarm-Legion, x86 16코어)에서 진단·수정 후 Orin 이관용.
> Orin에는 STM 펌웨어 폴더 등 일부 컨텍스트가 없으므로, 이 문서 하나로 상황을 이어받을 수 있게 정리함.

## 0. 한 줄 요약
200Hz 제어 루프가 밀리고 통신이 끊기는 문제 → **원인 2개(통신 손실 / 루프 선점)가 겹친 것**.
- 통신 손실(Loss 폭증) = **USB-시리얼 `latency_timer` 기본 16ms** → `latency_timer=1`로 해결.
- 루프 밀림(`exceeded period`) = **제어 스레드에 RT 우선순위 미적용 → 선점당함** → 코드 수정 + **rtprio 권한** 필요.

---

## 1. 시스템 구조 (요점)
- 노드: `comm_test_node` (= ROS2 노드명 `firmware_bridge_node`), 패키지 `orin_firmware_bridge`.
- 역할: Orin이 마스터(ID 0x01), RS485/USB-시리얼(`/dev/ttyUSB0`, FTDI `ftdi_sio`, 921600 8N1)로 ECU/DPC/PCU에 req→resp.
- 제어 루프: `RdSchedule::RunLoop()` — **200Hz tick(5ms)**, 100Hz READ / 50Hz WRITE(cmd_vel) / 10Hz sys / 5Hz 커맨드 슬롯x4.
- 트랜잭션 예산: 헤더 2ms + 바디 2ms = 최악 4ms < 5ms 주기.
- CLI: `command_cli` (`/carrier/command_set`, `/carrier/jeongae_lock` 서비스 호출).
- 핵심 파일:
  - `src/orin_firmware_bridge/src/rd_schedule.cpp` — 루프/타이밍/RT (이번에 수정)
  - `src/orin_firmware_bridge/src/rd_uart.cpp` — 시리얼 open/read/write (`DrainWriteBuffer`=tcdrain)
  - `src/orin_firmware_bridge/src/rd_comm.cpp` — 패킷 조립/CRC, `Read(header_ms, body_ms)`
  - `src/orin_firmware_bridge/src/main.cpp` — `RdUart("/dev/ttyUSB0")`, `scheduler.MainLoopStart()`

---

## 2. 발견한 문제 3가지

### (A) CLI "서비스 응답 timeout" — sudo/유저 불일치 *(과거 이슈, 해결됨)*
- bridge를 **sudo(root)** 로, `command_cli`를 **일반 유저**로 띄우면 Fast DDS 공유메모리(/dev/shm)가 root↔user 간 막혀 **discovery는 되는데 응답 데이터가 안 옴** → "서비스 없음"이 아니라 "응답 timeout".
- 죽은 bridge의 유령 DDS 등록이 남아도 같은 증상.
- **해결: bridge와 CLI를 같은 유저로.** swarm은 `dialout` 그룹이라 `/dev/ttyUSB0`를 sudo 없이 연다 → sudo 불필요.

### (B) 통신 손실(Loss 폭증) — `latency_timer` 기본 16ms  ★주범1
- FTDI가 수신 바이트를 호스트로 보내기 전 버퍼링하는 시간이 기본 **16ms**. read 타임아웃은 2ms라 응답이 제때 안 와 **대부분 read 타임아웃 → Loss**.
- 코드/`run.sh` 어디에도 설정이 없고 `main.cpp` 주석에만 "수동 1ms 권장"으로 적혀 있음 → **리부팅/USB 재연결 때마다 16으로 리셋**.
- 노트북 실측(아래 §5)에서 16→1 바꾸자 **Loss 533→2, WARN 15→1**로 즉시 해결.

### (C) 루프 밀림(`exceeded period`) — RT 우선순위 미적용  ★주범2
- `RunLoop`은 `time_elapsed = now()-next_cycle`(깨어나서 일 끝낼 때까지 벽시계)를 5ms와 비교. **Exec Time이 ~2ms로 짧은데도** 초과 → 일이 무거운 게 아니라 **스레드가 늦게 깨어남(선점)**.
- RT 보호 코드(SCHED_FIFO+코어고정)는 원래 `ThreadStart()`에만 있었는데 `main.cpp`는 `MainLoopStart()`를 호출 → **RT가 한 번도 안 걸림**(git상 초기 커밋부터).
- 노트북(빠른 x86, 가벼운 부하)은 latency만 잡아도 거의 사라지나, **Orin(ARM + RealSense 등 부하)은 선점이 심해 latency=1이어도 밀림** → 그래서 "Orin은 latency 1인데도 문제"였던 것.

---

## 3. 이번에 한 코드 수정 (노트북 `tp_ws/orin_ws`에 반영·빌드 검증 완료)
`rd_schedule.cpp` / `rd_schedule.hpp`:
- `MainLoopStart()`가 실제 도는 스레드에 RT를 걸도록 `ApplyRtScheduling(pthread_self())` 추가.
- 헬퍼 `ApplyRtScheduling(pthread_t)`: `SCHED_FIFO(pri=kRtPriority=80)` + `CPU affinity(kCpuCore=11)`, 성공/실패 로그 출력. `ThreadStart()`도 이 헬퍼 사용.
- 헤더에 `#include <pthread.h>` 및 선언 추가.
- (별도로 사용자가) FATAL 임계를 `2*period`→`5*period`로 완화해 단발 지터로 통신이 끊기지 않게 함. ※임시 완화이며 근본 해결은 (B)(C).

> ⚠️ 이 수정은 노트북 `tp_ws/orin_ws/src/...`에 있음. **Orin에서 `~/orin_ws`로 빌드한다면 같은 두 파일 변경을 Orin 쪽 소스에도 반영** 후 `./build.sh`.

---

## 4. Orin에서 할 일 (체크리스트)

1. **latency_timer=1 (영구)** — 일회성 `echo`는 재부팅/재연결에 날아감. udev 룰로 고정:
   ```bash
   echo 'ACTION=="add", SUBSYSTEM=="usb-serial", ATTR{latency_timer}="1"' | \
     sudo tee /etc/udev/rules.d/99-ftdi-latency.rules
   sudo udevadm control --reload && sudo udevadm trigger
   cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer   # 1 확인
   ```
2. **rtprio 권한 부여 (sudo 없이 SCHED_FIFO 쓰려면 필수)** — 안 하면 `SCHED_FIFO 설정 실패` WARN 뜨고 일반 우선순위로 떨어져 밀림 유지:
   ```bash
   echo 'swarm  -  rtprio  99' | sudo tee -a /etc/security/limits.conf
   # 로그아웃→재로그인 (또는 리부팅)
   ulimit -r     # 99 확인
   ```
3. **코어 수 확인** — `kCpuCore=11`은 12코어(0~11) 가정. `nproc`가 12 미만이면 affinity 실패:
   ```bash
   nproc
   ```
   - 8코어(예: Orin NX)면 `rd_schedule.hpp`의 `kCpuCore = 11` → `7` 등으로 수정 후 재빌드.
4. **빌드·실행**: `./build.sh && ./run.sh`
5. **전원/클럭 고정**(선택, 지터 추가 완화): `sudo nvpmodel -m 0 && sudo jetson_clocks`

---

## 5. 검증 기준 (실행 로그에서 확인)
정상 적용 시 시작 로그:
```
[INFO] ... SCHED_FIFO pri=80 적용됨.
[INFO] ... CPU affinity core 11 고정됨.
```
Heartbeat 목표치:
- `Loss` ≈ 0~수개 (수백이면 latency_timer 미적용 의심)
- `exceeded period` WARN 거의 없음, FATAL/`Hardware Error` 0
- `Nodes : ECU[ON]`

진단 신호:
- `SCHED_FIFO 설정 실패` WARN → rtprio 권한 없음(§4-2 재확인).
- `CPU affinity(core 11) 설정 실패` WARN → 코어 수 부족(§4-3).
- `Loss` 폭증 → latency_timer 16(§4-1).

---

## 6. 노트북 실측 데이터 (참고 baseline)
- HW: x86 16코어(0~15), 커널 6.8 PREEMPT_DYNAMIC, governor `powersave`, `ulimit -r`=0.
- 시리얼: `/dev/ttyUSB0`, 드라이버 `ftdi_sio`.
- 같은 코드(새 RT)·rtprio 미설정 상태에서 latency_timer만 변경:

| latency_timer | Loss (Tx 691 기준) | exceeded period (9초) | FATAL | ECU |
|---|---|---|---|---|
| 16 | 533 (~77%) | 15 | 0 | 늦게 ON |
| **1** | **2 (~0.3%)** | **1** | 0 | 즉시 ON |

- RT 로그: `CPU affinity core 11 고정됨` ✅ / `SCHED_FIFO 설정 실패`(rtprio=0이라) ⚠️.
- 해석: 노트북은 latency=1만으로 거의 깨끗(빠른 CPU). **Orin은 여기에 더해 rtprio+RT가 있어야 선점 밀림이 잡힘.**

---

## 7. 남은 개선 과제 (근본책, 추후)
- USB 시리얼 끊김 시 `RdUart`에 **포트 자동 reopen** 로직 없음 → 재연결되면 노드가 죽음. 에러 시 reopen 추가 권장.
- `latency_timer` 설정을 코드(`RdUart::Init`)나 `run.sh`에 내장해 udev 의존 제거.
- 타이밍 측정이 "깨어난 지연 + 처리시간"을 합쳐 보고 → WARN 메시지를 "wake latency"로 분리하면 진단이 명확.
- (선택) 커널 `isolcpus=11`로 제어 코어를 스케줄러에서 완전 격리.
