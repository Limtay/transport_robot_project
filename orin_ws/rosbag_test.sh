#!/bin/bash
# 임시 bag 기록 — **프로파일 재생이 아닌** 수동 조작(stream·teleop)을 남길 때만.
#
#   ./rosbag_test.sh                 # data/rosbags/manual_<MM-DD_HH-MM>/bag
#   ./rosbag_test.sh s1_w20          # 라벨 지정
#   ./rosbag_test.sh s1_w20 --diag   # comm_diag 도 (브리지를 comm_diag_enable 로 띄운 경우)
#
# ## ⚠ 프로파일 실험이면 이걸 쓰지 않는다
#
# 이 스크립트는 **bag 만** 남긴다. 분석 파이프라인이 요구하는 실험 폴더 규격
# (`profile.yaml` + schema v2 `result.json` + sha256) 은 만들지 않는다 — 05 §5.1.
# 프로파일 실험은 이쪽을 쓴다:
#
#     ./cli.sh run <profile.yaml> --record      # CLI
#     웹 Tab2 의 "기록" 체크박스                  # 웹 (같은 record.py 를 쓴다)
#
# ## 기록 토픽을 여기 적지 않는 이유
#
# 종전 이 스크립트는 `/carrier/testbed/feedback` 을 **하드코딩**하고 있었다. 01 §8.3 에서
# 토픽이 `/carrier/control/feedback` 으로 개명된 뒤에도 그대로여서, 이걸로 남긴 bag 은
# **빈 bag** 이 된다 (없는 토픽을 구독하니 0건). 목록은 `control_cli/record.py` 가 단독으로
# 갖고, 여기서는 물어본다 — 두 곳에 적으면 갈라진다.

set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/_env.sh"
rd_source_ws

LABEL="manual"
DIAG=0
for a in "$@"; do
    case "$a" in
        --diag) DIAG=1 ;;
        -h|--help) sed -n '2,30p' "${BASH_SOURCE[0]}" | sed 's/^# \?//'; exit 0 ;;
        -*) echo "ERROR: 모르는 옵션 '$a'" >&2; exit 2 ;;
        *)  LABEL="$a" ;;
    esac
done

# ⚠ CWD 를 레포 루트로 (cli.sh·web.sh 와 같은 이유) — `data/rosbags` 는 상대경로다.
cd "$WS/.."

TOPICS=$(python3 -c "
from control_cli import record
print(' '.join(record.record_topics(diag=$DIAG)))
")
if [ -z "$TOPICS" ]; then
    echo "ERROR: 기록 토픽 목록을 못 읽었다 — control_cli 가 빌드됐는지 확인할 것" >&2
    exit 1
fi

OUT="data/rosbags/${LABEL}_$(date +%m-%d_%H-%M)/bag"
echo ">>> $OUT"
echo "    토픽: $TOPICS"
echo "    (Ctrl-C 로 종료 — SIGKILL 하면 메타데이터가 안 써져 bag 이 열리지 않는다)"
exec ros2 bag record -o "$OUT" $TOPICS
