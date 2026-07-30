#!/bin/bash
# 웹 UI 기동 (redesign/07). 브라우저에서 http://<호스트>:8080
#
#   ./web.sh                                  # 기본 (0.0.0.0:8080)
#   ./web.sh --port 9000 --local              # 포트 변경 + 로컬만
#   ./web.sh -p bag_dir:=/data/bags           # 그 외 파라미터는 그대로 통과
#
# ## ⚠ 브리지를 먼저 띄우지 않는다
#
# 07 §1 부터 **웹이 브리지를 띄우는 주체**다. 브라우저의 "브리지 기동" 패널에서
# bridge_mode·auto_mode·read_preset·active_motors 를 고르고 시작을 누른다.
#
# 터미널에서 `comm_test_node` 를 먼저 올려 두면 웹이 기동을 **거부한다** —
# "브리지가 이미 떠 있다 (이 웹이 띄운 것이 아니다)". 고아 프로세스를 웹이 조작하면
# 누가 그것을 내릴 책임인지가 사라지기 때문이다 (supervisor.py `_find_orphan_bridge`).
# 브리지를 직접 띄우고 싶으면 웹 대신 ./run.sh 를 쓴다.

set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/_env.sh"
rd_source_ws

PORT=8080
BIND=0.0.0.0
PASS=()
while [ $# -gt 0 ]; do
    case "$1" in
        --port)  PORT="$2"; shift 2 ;;
        --local) BIND=127.0.0.1; shift ;;
        --bind)  BIND="$2"; shift 2 ;;
        *)       PASS+=("$1"); shift ;;
    esac
done

rd_need_exe control_web control_web
# 브리지도 있어야 한다 — 웹이 그것을 자식으로 띄우므로, 없으면 "시작" 을 누른 뒤에야 안다.
rd_need_exe orin_firmware_bridge comm_test_node
rd_check_port

# 포트가 이미 잡혀 있으면 파이썬이 OSError 로 죽는다. 먼저 말해 준다.
if command -v ss >/dev/null && ss -ltn 2>/dev/null | grep -q ":$PORT "; then
    echo "ERROR: 포트 $PORT 이 이미 사용 중이다 — --port 로 바꾸거나 기존 웹을 내릴 것" >&2
    exit 1
fi

# ⚠ CWD 를 레포 루트로 옮긴다 (cli.sh 와 같은 이유). `record.DEFAULT_BAG_BASE` 가
#    `data/rosbags` **상대경로**라, 여기서 옮기지 않으면 웹은 `orin_ws/data/rosbags` 에,
#    CLI 는 `tp_ws/data/rosbags` 에 쌓는다 — 기록이 두 곳으로 갈라지고 그 사실은
#    분석할 때가 되어서야 드러난다. 기동 로그에 해석된 절대경로가 찍히므로 확인 가능하다.
cd "$WS/.."

echo ">>> control_web  http://${BIND/0.0.0.0/localhost}:$PORT"
echo "    (브리지는 브라우저의 '브리지 기동' 패널에서 띄운다)"

# ⚠ `"${PASS[@]:-}"` 로 쓰면 **빈 배열이 빈 문자열 하나로 전개된다.** 그 `''` 가
#    `--ros-args` 뒤에 붙어 rclpy 가 `UnknownROSArgsError: ['']` 로 죽는다 (실제로 그랬다).
#    배열에 append 하는 형태는 빈 배열이면 무연산이라 안전하다 (bash 4.4+).
ARGS=(-p port:="$PORT" -p bind:="$BIND")
ARGS+=("${PASS[@]}")

exec ros2 run control_web control_web --ros-args "${ARGS[@]}"
