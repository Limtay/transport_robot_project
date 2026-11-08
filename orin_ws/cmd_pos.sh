#!/bin/bash
# 위치 조그 (carrier_teleop keyboard_pos) — position_inst 브리지에 W/S 로 위치 명령을 스트림한다.
#
#   ./rec.sh -m "이동량_1" -a 1,2      # 다른 터미널 — 브리지 + bag
#   ./cmd_pos.sh                       # 기본: 모터 1,2, 최대 60°/s, Ctrl 가속 20°/s²
#   ./cmd_pos.sh slow                  # 최대 30°/s, 가속 10°/s²
#   ./cmd_pos.sh fast                  # 최대 180°/s, 가속 60°/s², max_lead 40°
#   ./cmd_pos.sh -p signs:="[1.0,-1.0]"        # 모터별 전진 방향 (motors 와 같은 길이)
#   ./cmd_pos.sh -p s_range:="[-500.0,500.0]"  # 전진량 s 제한 [deg] (처음 arm 한 자리 기준)
#   ./cmd_pos.sh slow -p motors:="[1]"          # 프리셋 + 개별 덮어쓰기 (뒤가 이긴다)
#
#   조작: W/S 전진·후진 (기본 속도 0) · Ctrl 홀드 = 가속 (떼면 속도 유지) · Shift 홀드 = 감속
#         Space 정지 · E arm (LOCKED 면 REARM) · Q/ESC 종료 (정지 후 arm 해제)
#
# 파라미터: motors, signs, v_max [deg/s], accel / decel [deg/s²], max_lead [deg],
#           s_range [deg], rate [Hz], feedback_timeout [s]. 속도·각도는 모터 출력축 기준이다.
#
# ⚠ 브리지는 스트림 중 추종 오차를 검사하지 않는다. 목표가 실제 위치보다 max_lead 넘게
#   앞서면 이 도구가 더 밀지 않는다 — 모터가 안 따라오면 화면 "알림" 에 뜬다.
# ⚠ pygame 창이 필요하다 (cmd_vel.sh 와 같음 — 개발 머신 전용, pip 로 설치한 pygame).

set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/_env.sh"
rd_source_ws

usage() { awk 'NR>1 { if (!/^#/) exit; sub(/^# ?/, ""); print }' "${BASH_SOURCE[0]}"; }

ARGS=()
case "${1:-}" in
    slow) ARGS=(-p v_max:=30.0 -p accel:=10.0); shift ;;
    fast) ARGS=(-p v_max:=180.0 -p accel:=60.0 -p max_lead:=40.0); shift ;;
    default) shift ;;
    -h|--help) usage; exit 0 ;;
    -*|'') ;;
    *) echo "ERROR: 모르는 프리셋 '$1' — default|slow|fast" >&2; exit 2 ;;
esac
# -p 앞에 --ros-args 가 붙어야 하므로 나머지 인자를 그대로 넘긴다.

rd_need_exe carrier_teleop keyboard_pos

if [ -z "${DISPLAY:-}" ] && [ -z "${WAYLAND_DISPLAY:-}" ]; then
    echo "ERROR: DISPLAY 가 없다 — pygame 창을 열 수 없다." >&2
    exit 1
fi
if ! python3 -c 'import pygame' 2>/dev/null; then
    echo "ERROR: pygame 이 없다 — pip3 install --user --upgrade pygame" >&2
    exit 1
fi
if ! pgrep -x comm_test_node >/dev/null 2>&1; then
    echo "WARN: comm_test_node 가 안 보인다 — 먼저 ./rec.sh -m \"이름\" -a 1,2 로 브리지를 띄울 것." >&2
fi

echo ">>> keyboard_pos  ${ARGS[*]:-(기본값)} $*"
echo "    W/S 이동 · Ctrl 가속 · Shift 감속 · Space 정지 · E arm · Q/ESC 종료"
exec ros2 run carrier_teleop keyboard_pos --ros-args ${ARGS[@]+"${ARGS[@]}"} "$@"
