#!/bin/bash
# 키보드 teleop (carrier_teleop) 기동 — WASD 로 /carrier_cmd_vel 을 만들어 넣는다.
#
#   ./cmd_vel.sh                       # 기본값
#   ./cmd_vel.sh fast                  # 빠른 프리셋 (max 0.5/0.5, 부스트 x5)
#   ./cmd_vel.sh slow                  # 느린 프리셋 (max 0.15/0.3, 부스트 x1.5)
#   ./cmd_vel.sh --no-idle-stop        # 촬영용 — 무입력에도 cmd_vel 을 계속 발행
#   ./cmd_vel.sh -p max_linear:=0.4    # 그 외 파라미터는 그대로 통과
#   ./cmd_vel.sh fast -p idle_timeout:=8.0   # 프리셋 + 개별 덮어쓰기 (뒤가 이긴다)
#
#   조작: W/A/S/D 이동, Ctrl 홀드=부스트 램프, Space=정지, J=jeongae 펄스,
#         R=idle-stop 토글, Q/ESC=종료
#
# ## idle-stop (무입력 정지)
#
# 입력이 `idle_timeout`(기본 5초) 동안 없으면 발행을 멈춘다 → bridge 의 0.5s 워치독이
# 모터를 0 으로 잡는다. 촬영 중 토픽이 끊겨 bag 이 조각나는 게 싫으면 실행 중 `R` 로
# 끄거나 `--no-idle-stop` 으로 꺼진 채 띄운다.
#
# ⚠ 끄면 **그 워치독이 영영 동작하지 않는다** (0 명령이 계속 흐르므로). 값이 0 이라
#   바퀴가 도는 건 아니지만, 촬영 구간에서만 끄고 끝나면 `R` 로 다시 켤 것.
#
# ## ⚠ 이건 **개발 머신(노트북)에서 실행하는 도구**다
#
# pygame 창이 필요하므로 디스플레이가 있어야 한다. Orin 에는 필요 없어서 `deploy.sh` 의
# 제외 목록(SKIP_PKGS carrier_teleop / SKIP_SCRIPTS cmd_vel.sh)에 들어 있다 — 전송되지 않는다.
#
# ## ⚠ pygame 은 apt 말고 pip 로 깔 것
#
# apt 의 `python3-pygame` 은 시스템 libsdl2(2.24+)와 버전이 안 맞아 `pygame.init()` 에서
# "SDL compiled with ... linked to ..." 로 죽는다. SDL 을 자체 번들하는 pip 휠을 쓴다:
#
#     pip3 install --user --upgrade pygame

set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/_env.sh"
rd_source_ws

usage() { awk 'NR>1 { if (!/^#/) exit; sub(/^# ?/, ""); print }' "${BASH_SOURCE[0]}"; }

ARGS=()
IDLE_STOP=""

# 프리셋보다 먼저 훑는다 — `./cmd_vel.sh fast --no-idle-stop` 처럼 섞어 쓸 수 있어야 한다.
REST=()
for a in "$@"; do
    case "$a" in
        --no-idle-stop) IDLE_STOP="false" ;;
        --idle-stop)    IDLE_STOP="true" ;;
        *) REST+=("$a") ;;
    esac
done
set -- ${REST[@]+"${REST[@]}"}

case "${1:-}" in
    fast)
        ARGS=(-p max_linear:=0.5 -p max_angular:=0.5
              -p boost_factor:=5.0 -p boost_ramp_time:=1.0)
        shift ;;
    slow)
        ARGS=(-p max_linear:=0.15 -p max_angular:=0.3
              -p boost_factor:=1.5 -p boost_ramp_time:=2.0)
        shift ;;
    default)
        shift ;;
    -h|--help)
        usage; exit 0 ;;
    -*|'')
        ;;                     # `-p ...` 로 시작하면 프리셋 이름이 아니다
    *)
        echo "ERROR: 모르는 프리셋 '$1' — default|fast|slow" >&2
        exit 2 ;;
esac

# 파라미터는 뒤가 이긴다 — 프리셋 뒤에 붙여 `-p idle_stop_enabled:=...` 로 통과시킨다.
if [ -n "$IDLE_STOP" ]; then
    ARGS+=(-p idle_stop_enabled:="$IDLE_STOP")
fi

rd_need_exe carrier_teleop keyboard_teleop

# 두 함정을 미리 잡는다. 둘 다 노드가 뜬 **뒤에** 죽어서, 로그를 봐야 원인이 보인다.
if [ -z "${DISPLAY:-}" ] && [ -z "${WAYLAND_DISPLAY:-}" ]; then
    echo "ERROR: DISPLAY 가 없다 — pygame 창을 열 수 없다 (SSH 세션?)." >&2
    echo "       디스플레이 있는 머신에서 실행할 것. Orin 에서는 쓰지 않는다." >&2
    exit 1
fi
if ! python3 -c 'import pygame' 2>/dev/null; then
    echo "ERROR: pygame 이 없다 — pip3 install --user --upgrade pygame" >&2
    echo "       ⚠ apt 의 python3-pygame 은 libsdl2 버전 불일치로 init 에서 죽는다." >&2
    exit 1
fi

# 브리지가 없어도 teleop 은 뜬다 (토픽만 발행). 다만 아무 반응이 없는 이유는 말해 준다.
if ! pgrep -x comm_test_node >/dev/null 2>&1; then
    echo "WARN: comm_test_node 가 안 보인다 — 로봇이 움직이려면 ./run.sh 나 ./web.sh 가 필요하다." >&2
fi

echo ">>> keyboard_teleop  ${ARGS[*]:-(기본값)} $*"
echo "    W/A/S/D 이동 · Ctrl 부스트 · Space 정지 · J jeongae 펄스"
echo "    R idle-stop 토글 · Q/ESC 종료"
if [ "$IDLE_STOP" = "false" ]; then
    echo "    ⚠ idle-stop OFF 로 시작한다 — 무입력에도 계속 발행(워치독 동작 안 함)" >&2
fi

exec ros2 run carrier_teleop keyboard_teleop --ros-args ${ARGS[@]+"${ARGS[@]}"} "$@"
