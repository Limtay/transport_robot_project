#!/bin/bash
# dpy_camera 가 찍은 사진을 RViz2 로 본다. (run_dpy_camera.sh 와 짝)
#
#   ./run_dpy_rviz.sh                  # 기본 설정으로 열기
#   ./run_dpy_rviz.sh --config x.rviz  # 다른 설정 파일
#   ./run_dpy_rviz.sh --no-config      # 빈 RViz2 (설정 파일 없이)
#
#   환경변수: DPY_RVIZ_CONFIG   (옵션을 주면 옵션이 이긴다)
#
# 사진 토픽은 TRANSIENT_LOCAL(래치)이라 rqt_image_view 로는 안 보인다 —
# QoS 를 지정할 수 있는 RViz2 + 아래 설정 파일이 필요하다. (CAMERA_ACTION.md §14)
#
# ## 종전 스크립트와 달라진 점
#
# `/home/bridge/tp_ws/orin_ws/...` 를 하드코딩하고 있었다 — 그 계정이 아닌 머신에서는
# 없는 설정 파일을 RViz2 에 넘겨 아무 Display 도 없는 빈 창이 떴고, 원인은 화면에
# 나오지 않았다. `_env.sh` 가 스크립트 위치에서 워크스페이스를 찾는다.

set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/_env.sh"

# 빌드 전에도 토픽만 보면 되므로 오버레이는 있으면 쓴다 (없어도 진행).
if [ -f "$WS/install/setup.bash" ]; then
    rd_source_quietly "$WS/install/setup.bash"
fi

# 원본이 있는 src 폴더의 설정을 직접 읽는다 (빌드 필요 없음 — run_dpy_camera.sh 와 동일)
CONFIG="${DPY_RVIZ_CONFIG:-$WS/src/dpy_camera/config/dpy_camera.rviz}"
PASS=()

usage() { awk 'NR>1 { if (!/^#/) exit; sub(/^# ?/, ""); print }' "${BASH_SOURCE[0]}"; }

while [ $# -gt 0 ]; do
    case "$1" in
        --config)    CONFIG="$2"; shift 2 ;;
        --no-config) CONFIG=""; shift ;;
        -h|--help)   usage; exit 0 ;;
        *) PASS+=("$1"); shift ;;
    esac
done

if ! command -v rviz2 >/dev/null 2>&1; then
    echo "ERROR: rviz2 가 없다 — sudo apt install ros-humble-rviz2" >&2
    exit 1
fi

# RViz2 는 없는 설정 파일을 받아도 경고만 내고 빈 창을 띄운다. 여기서 잡아 준다.
if [ -n "$CONFIG" ] && [ ! -f "$CONFIG" ]; then
    echo "ERROR: RViz 설정 파일이 없다: $CONFIG" >&2
    echo "       다른 파일이면 --config <파일>, 설정 없이 열려면 --no-config" >&2
    exit 1
fi

if [ -z "${DISPLAY:-}" ] && [ -z "${WAYLAND_DISPLAY:-}" ]; then
    echo "WARN: DISPLAY 가 없다 — 헤드리스(SSH)에서는 창이 안 뜬다." >&2
    echo "      디스플레이 있는 머신에서 실행하거나 ssh -X 로 접속할 것." >&2
fi

echo "Orin Environment: dpy_camera 촬영 결과 뷰어 (RViz2) Start"
echo "----------------------------------------------------------------------"
echo " 설정: ${CONFIG:-(없음)}"
echo " 왼쪽 Displays 패널 > Shot > Topic > Value 의 끝 숫자만 바꿔 장을 넘긴다."
echo "   /dpy_camera/image_raw_1  →  _2  →  _3 ...   (1-base, num_shots 만큼)"
echo " 래치 토픽이라 촬영이 한참 지난 뒤에 띄워도 사진이 나온다."
echo " Fixed Frame(TF) 경고는 3D 뷰를 안 쓰므로 무시해도 된다."
echo "----------------------------------------------------------------------"

if [ -n "$CONFIG" ]; then
    exec rviz2 -d "$CONFIG" ${PASS[@]+"${PASS[@]}"}
else
    exec rviz2 ${PASS[@]+"${PASS[@]}"}
fi
