# 키보드 teleop (carrier_teleop)

실행은 **`./cmd_vel.sh`** 로 한다 — 이 문서에 있던 절차를 그대로 옮긴 스크립트다.
(경로·환경 세팅은 `_env.sh` 가 스크립트 위치에서 찾으므로 계정/머신이 달라도 그대로 돈다.)

```bash
cd ~/tp_ws/orin_ws
./cmd_vel.sh                     # 기본값
./cmd_vel.sh fast                # max 0.5/0.5, 부스트 x5 (아래 예시와 같은 값)
./cmd_vel.sh slow                # max 0.15/0.3, 부스트 x1.5
./cmd_vel.sh --no-idle-stop      # 촬영용 — 무입력에도 계속 발행
./cmd_vel.sh -p max_linear:=0.4  # 그 외 파라미터는 그대로 통과
./cmd_vel.sh --help
```

조작: `W/A/S/D` 이동 · `Ctrl` 홀드 = 부스트 램프 · `Space` 정지 · `J` = jeongae 펄스 ·
`R` = idle-stop 토글 · `Q`/`ESC` 종료.

## idle-stop (무입력 정지) — 촬영할 때 쓰는 토글

입력이 `idle_timeout` **기본 5초**(종전 2초) 동안 없으면 `/carrier_cmd_vel` 발행을 멈춘다.
bridge 의 0.5s 워치독이 그때 모터를 0 으로 잡는다.

촬영 중 토픽이 끊겨 bag/그래프가 조각나는 게 싫으면 **`R` 로 끈다.** 끈 동안은 입력이
없어도 `cmd_vel(0,0)` 이 계속 나간다. 처음부터 꺼진 채 띄우려면:

```bash
./cmd_vel.sh --no-idle-stop
# 같은 뜻: ./cmd_vel.sh -p idle_stop_enabled:=false
# 시간만 늘리려면:  ./cmd_vel.sh -p idle_timeout:=10.0
```

창의 `idle-stop:` 줄이 현재 상태다 — 꺼져 있으면 주황색으로 표시된다.

> ⚠ **끄면 bridge 의 0.5s 워치독이 영영 트리거되지 않는다.** "명령이 끊기면 멈춘다" 는
> 마지막 안전망이 걷힌 상태가 된다. 발행 값 자체는 0 이라 바퀴가 도는 것은 아니지만,
> 촬영 구간에서만 끄고 끝나면 `R` 로 다시 켠다. 재시작하면 항상 ON 으로 돌아온다.

## 1. pygame 설치 (한 번만)

```bash
pip3 install --user --upgrade pygame
```

> ⚠ apt 의 `python3-pygame` 은 시스템 libsdl2(2.24+)와 버전이 안 맞아 `pygame.init()` 에서
> `SDL compiled with ... linked to ...` 로 죽는다. **SDL 을 자체 번들하는 pip 휠**로 설치할 것.
> `cmd_vel.sh` 가 실행 전에 pygame 유무와 `DISPLAY` 를 먼저 확인해 준다.

## 2. 실행 위치

pygame 창이 필요하므로 **디스플레이가 있는 개발 노트북**에서 실행한다.
Orin 에는 필요 없어서 `deploy.sh` 가 `carrier_teleop` 패키지와 `cmd_vel.sh` 를 둘 다
전송 제외 목록(`SKIP_PKGS` / `SKIP_SCRIPTS`)에 두고 있다.

## 3. 스크립트 없이 직접 실행할 때

```bash
source install/setup.bash
ros2 run carrier_teleop keyboard_teleop --ros-args \
  -p max_linear:=0.5 -p max_angular:=0.5 \
  -p boost_factor:=5.0 -p boost_ramp_time:=1.0 \
  -p idle_timeout:=2.0 -p jeongae_pulse:=1.0
```
