# tools/ — 실기 관문 계측기

`redesign/06_migration.md` §4.2 의 G1~G3 관문을 **브리지 노드 없이** 확인한다.
L0/L1(`RdUart`/`RdComm`/`RdMap`)만 쓰므로 FSM·ROS 토픽이 개입하지 않는다 — 와이어만 본다.

**CMakeLists 에 등록하지 않는다.** 진단용이고 배포물이 아니다. 필요할 때 직접 컴파일한다.

## 컴파일

```bash
cd ~/tp_ws/orin_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
g++ -std=c++17 -O2 -o /tmp/rd_probe src/orin_firmware_bridge/tools/rd_probe.cpp \
    -I install/orin_firmware_bridge/include -I /opt/ros/humble/include \
    $(pkg-config --cflags libserial) \
    -L install/orin_firmware_bridge/lib -lrd_core_lib \
    -Wl,-rpath,$PWD/install/orin_firmware_bridge/lib \
    $(pkg-config --libs libserial) -lpthread
```

`rd_probe2.cpp` 도 파일명만 바꿔 동일하게.

> 6단계(CMake 2 라이브러리 분리)부터 `rd_bridge_lib` 은 INTERFACE 타깃이라
> `.so` 가 없다. 계측기는 L0/L1 만 쓰므로 **`rd_core_lib`** 에 링크한다.
> 실행 시 `install/setup.bash` 를 source 해야 런타임 경로가 잡힌다.
> 포트 권한도 필요하다: `sudo chmod 666 /dev/ttyUSB0`

## rd_probe — 기본 관문

```bash
/tmp/rd_probe /dev/ttyUSB0 200      # 인자: 포트, P8 반복 횟수
```

| 단계 | 확인 |
|---|---|
| A | 링크 + 펌웨어 레이아웃 (addr 32 vs 228) |
| B | `realtime_tick` 증가 = ECU 살아있음 |
| C | **P8 게이트** — DIRECT RW 87B 반복, `RD_FATAL` 0 이 통과 조건 |
| D | `auto_mode` 4/5 수용 + `ctr_mode` read-back |

모든 WRITE 는 shadow 를 `memset(0)` 한 뒤 나간다 (cmd_motor 전 구간 0 = 무토크).
종료 시 `auto_mode` 를 1(CURRENT)로 원복한다.

> ⚠ **D 의 함정**: `auto_mode=4` 는 `MODE_VELOCITY(3)` 을 강제하는데 `DEF_CTR_MODE` 도 3 이라
> **기본값과 구분되지 않는다.** 분기가 실제로 돌았는지는 `auto_mode=5` → `ctr_mode==4`
> (`MODE_POSITION`) 로만 알 수 있다. 06 §4.5-나 참조.

## rd_probe2 — 레이아웃 인과 판별

```bash
/tmp/rd_probe2 /dev/ttyUSB0 150     # 인자: 포트, 측정 쌍 수
```

`rs485_proc_delta` 는 `rd_delta_tick`(0.1ms 단위)이라 평시엔 대개 0 이다. 그래서
**"비영이면 신 펌웨어"** 는 성립하지 않는다 — 구 펌웨어의 `reserved0[0]`(항상 0)과 구분이 안 된다.

대신 **부하 반응**을 본다. 직전 트랜잭션의 처리시간이 다음 응답에 실리므로
(`rd_system.c:643`), READ 3B 직후와 RW 87B 직후를 짝지어 비교한다.
구 펌웨어면 `addr32` 는 부하와 무관하게 영원히 0 이다 — **반응 자체가 판별**이다.

2026-07-28 실측: `addr32` 26% → **76%**, `addr228` 0% → 0% → **신 펌웨어 확정**.

## 모터 미연결 시 알아둘 것

모터가 없으면 CAN 노드가 없어 ECU 가 **반드시 `SYS_STATE_FAULT`(5)** 로 떨어지고, 그 상태는
sticky 다 (`rd_system.c:393`). 여기서 두 가지가 따라온다 — **버그가 아니다**:

- `mtr_lock=1` → addr `132~187` 과 겹치는 WRITE 는 전량 `Access Error`
  (`auto_mode` 188 은 범위 밖이라 통과한다)
- `ACTION_STATE_AUTO()` 미실행 → `auto_mode` 분기(`case 0/1/2/4/5`)가 한 번도 안 돈다

자세한 내용과 모터 연결 후 할 일은 `redesign/06_migration.md` §4.5~§4.6.
