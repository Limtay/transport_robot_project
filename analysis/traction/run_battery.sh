#!/bin/bash
# 자율 배터리 러너 진입점 — ROS 소싱 후 run_battery.py <manifest> 실행.
# 사용: bash run_battery.sh data/profiles/<manifest>.json
# 진행 로그는 stdout(호출측 리다이렉트), 세션 요약은 data/rosbags/battery_<label>_*.json.
set -o pipefail
MANIFEST="${1:?사용: run_battery.sh <manifest.json>}"
source /opt/ros/humble/setup.bash
source /home/swarm/tp_ws/orin_ws/install/setup.bash
cd /home/swarm/tp_ws
exec python3 analysis/traction/run_battery.py "$MANIFEST"
