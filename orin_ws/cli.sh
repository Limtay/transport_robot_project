#!/bin/bash
# control_cli 통과 실행. 인자를 그대로 넘긴다.
#
#   ./cli.sh status
#   ./cli.sh command set auto 1 ecu read_all
#   ./cli.sh run profiles/step_5A.yaml --name step_5A
#   ./cli.sh --help
#
# ## ⚠ `command_cli` 는 없어졌다
#
# 종전 이 스크립트는 `ros2 run orin_firmware_bridge command_cli` 를 불렀다. 그 C++ 대화형
# 도구는 06 Q4/C4 에서 `control_cli` 로 흡수됐다 — 같은 브리지를 조작하는 도구가 둘이면
# 조작자가 어느 쪽을 쓰는지에 따라 가능한 일이 달라지기 때문이다. **하나로 합치는 것이
# 결정이었다.** 지금 그 이름으로 부르면 "No executable found" 가 난다.

set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/_env.sh"
rd_source_ws
rd_need_exe control_cli control_cli

# CLI 는 서비스/액션으로 브리지에 붙는다 — 브리지가 없으면 wait_for_service 타임아웃까지
# 조용히 기다린다. 미리 말해 준다 (권한 문제로 브리지가 안 뜬 경우가 흔하다).
if ! pgrep -x comm_test_node >/dev/null 2>&1; then
    echo "WARN: comm_test_node 가 안 보인다 — ./run.sh 나 ./web.sh 로 먼저 띄울 것" >&2
fi

# ⚠ CWD 를 레포 루트로 옮긴다. `record.py` 의 기본 bag 경로가 `data/rosbags`(상대경로)라,
#    어디서 실행하느냐에 따라 기록이 다른 곳에 쌓인다 — 그건 분석할 때가 되어서야 드러난다.
cd "$WS/.."

exec ros2 run control_cli control_cli "$@"
