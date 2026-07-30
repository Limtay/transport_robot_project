#!/bin/bash
# 브리지를 **직접** 띄운다 — 웹 없이 CLI 로 실험할 때. 웹을 쓸 거면 ./web.sh 를 쓴다
# (웹이 브리지를 자식으로 띄우므로, 둘을 같이 올리면 웹이 기동을 거부한다).
#
#   ./run.sh                 # project (기본 주행 모드)
#   ./run.sh traction        # 견인 실험  = control + auto_mode:none + control_test
#   ./run.sh control current  # 제어 실험 = control + auto_mode:current
#   ./run.sh manual          # 수동 (브리지가 자동으로 쓰지 않는다)
#   ./run.sh control direct -p active_motors:=1,2   # 그 외는 그대로 통과
#
# ## ⚠ `traction_test_mode` 는 폐지됐다
#
# 종전 이 스크립트는 `-p traction_test_mode:=true` 를 썼다. 06 §9.10 에서 그 불리언은
# 사라지고 `bridge_mode` enum 3개(project/control/manual)로 접혔다. 지금 그 인자를 주면
# 브리지가 **기동 게이트에서 거부하고 종료한다** (exit≠0).
#
# 같은 wire 를 내는 조합은 아래 `traction` 프리셋이다:
#   bridge_mode:=control  auto_mode:=none  read_preset:=control_test
# (`auto_mode:none` = 아무것도 쓰지 않는다. 전류 명령은 STM RC 램프가 만든다.)

set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/_env.sh"
rd_source_ws

MODE="${1:-project}"
shift || true

case "$MODE" in
    project)
        ARGS=(-p bridge_mode:=project)
        ;;
    manual)
        ARGS=(-p bridge_mode:=manual)
        ;;
    traction)
        # 구 traction_test_mode:=true 와 같은 배치. read_preset 이 로드셀을 읽는다.
        ARGS=(-p bridge_mode:=control -p auto_mode:=none -p read_preset:=control_test)
        ;;
    control)
        # 두 번째 인자가 auto_mode. 안 주면 current.
        AUTO="${1:-current}"
        case "${AUTO}" in
            none|kinematic|current|direct|velocity|position) shift || true ;;
            -*|'') AUTO=current ;;    # `-p ...` 로 시작하면 auto_mode 가 아니다
            *) echo "ERROR: 모르는 auto_mode '$AUTO'" >&2
               echo "       none|kinematic|current|direct|velocity|position" >&2; exit 2 ;;
        esac
        ARGS=(-p bridge_mode:=control -p auto_mode:="$AUTO")
        ;;
    -h|--help)
        sed -n '2,20p' "${BASH_SOURCE[0]}" | sed 's/^# \?//'; exit 0 ;;   # 1행은 shebang
    *)
        echo "ERROR: 모르는 모드 '$MODE' — project|control|manual|traction" >&2
        exit 2 ;;
esac

rd_need_exe orin_firmware_bridge comm_test_node
rd_check_port

# 웹이 이미 브리지를 띄워 뒀으면 포트를 두고 싸운다 (둘 다 /dev/ttyUSB0 을 연다).
# 먼저 확인해 준다 — 그냥 띄우면 "Open Failed" 만 반복되고 원인이 안 보인다.
if pgrep -x comm_test_node >/dev/null 2>&1; then
    echo "ERROR: comm_test_node 가 이미 돌고 있다 (pid $(pgrep -x comm_test_node | tr '\n' ' '))." >&2
    echo "       웹(./web.sh)이 띄운 것일 수 있다 — 그 쪽에서 정지하거나 kill -INT 할 것." >&2
    exit 1
fi

echo ">>> comm_test_node  ${ARGS[*]} $*"
exec ros2 run orin_firmware_bridge comm_test_node --ros-args "${ARGS[@]}" "$@"
