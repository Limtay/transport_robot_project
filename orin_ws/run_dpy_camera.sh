#!/bin/bash
# Arducam USB 카메라 on-demand 캡처 노드(dpy_camera) 기동.
#
#   ./run_dpy_camera.sh                       # 기본
#   ./run_dpy_camera.sh --list                # 카메라 노드 목록만 보고 끝낸다
#   ./run_dpy_camera.sh --device /dev/video0  # 카메라 노드 지정
#   ./run_dpy_camera.sh --save-dir ~/shots    # 저장 폴더 지정 ('' 이면 저장 안 함)
#   ./run_dpy_camera.sh -p num_shots:=5       # 그 외 파라미터는 그대로 통과
#
#   환경변수: DPY_VIDEO / DPY_SAVE_DIR / DPY_PARAMS  (옵션을 주면 옵션이 이긴다)
#
# ## ⚠ `/dev/videoN` 번호를 믿지 말 것 — `by-id` 를 쓴다
#
# 번호는 USB **열거 순서**다. RealSense 같은 다른 카메라가 같이 꽂혀 있으면 재부팅·재연결
# 마다 밀린다. 2026-08-19 Orin 에서 yaml 의 `/dev/video2` 가 Arducam 이 아니라 RealSense 를
# 가리켜, 열리기는 했지만 `VIDIOC_REQBUFS: errno=19` 로 한 프레임도 못 받았다.
# 그래서 아래 사전 점검이 **장치 이름을 찍어 준다** — 촬영 로그를 보기 전에 어긋난 걸 안다.
#
#   ./run_dpy_camera.sh --list                       # 어느 노드가 Arducam 인지 본다
#   ./run_dpy_camera.sh --device /dev/v4l/by-id/...  # 번호가 아닌 안정 경로로 지정
#
# ## 종전 스크립트와 달라진 점
#
#   ① 경로를 하드코딩했다 (`/home/bridge/tp_ws/orin_ws/...` 3곳). 그 계정이 없는 머신에서는
#      `source` 가 조용히 실패하고 python 이 "No such file" 로 죽었다 —
#      `_env.sh` 가 스크립트 자기 위치에서 워크스페이스를 찾는다 (run.sh/cli.sh 와 동일).
#   ② `save_dir` 을 주지 않았다. yaml 의 기본값은 **빈 문자열**이고(계정마다 홈이 달라
#      절대경로를 박을 수 없다), 빈 값은 "저장하지 않는다" 는 뜻이다. 즉 이 스크립트로 띄우면
#      토픽만 나오고 **사진 파일이 안 남았다** — launch 파일만 save_dir 을 주입하고 있었다.
#      여기서도 같은 기본값(`~/dpy_camera_shots`)을 주입한다.
#
# ## 왜 launch 대신 src 의 .py 를 직접 부르는가
#
# 빌드 없이 `capture_node.py`/yaml 수정이 바로 반영되게 하려는 것이다. (dpy_camera 는
# `--symlink-install` 로 깔리므로 빌드해 뒀다면 `ros2 launch dpy_camera dpy_camera.launch.py`
# 도 같은 파일을 본다 — 그 쪽은 install/ 이 있어야 한다.)

set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/_env.sh"

# 오버레이는 있으면 쓴다. 이 노드는 rclpy/sensor_msgs/std_srvs 만 쓰므로 **빌드 전에도 뜬다**.
# (`&&` 한 줄로 쓰면 안 된다 — `set -e` 아래에서 install 이 없을 때 그 자리에서 죽는다.)
if [ -f "$WS/install/setup.bash" ]; then
    rd_source_quietly "$WS/install/setup.bash"
fi

NODE="$WS/src/dpy_camera/scripts/capture_node.py"
PARAMS="${DPY_PARAMS:-$WS/src/dpy_camera/config/dpy_camera_params.yaml}"
# launch 파일(dpy_camera.launch.py)과 같은 기본값 — 두 경로로 띄워도 사진이 한 곳에 쌓인다.
SAVE_DIR="${DPY_SAVE_DIR-$HOME/dpy_camera_shots}"
VIDEO="${DPY_VIDEO:-}"
PASS=()

usage() { awk 'NR>1 { if (!/^#/) exit; sub(/^# ?/, ""); print }' "${BASH_SOURCE[0]}"; }

# `/sys/class/video4linux/<노드>/name` = 그 노드가 실제로 어느 카메라인지. by-id 심볼릭
# 링크로 들어와도 실제 노드로 풀어서 읽는다 (노드도 realpath 로 푼다).
rd_v4l_name() {
    local real
    real="$(readlink -f "$1" 2>/dev/null || echo "$1")"
    cat "/sys/class/video4linux/$(basename "$real")/name" 2>/dev/null || true
}

rd_list_video() {
    local d n
    echo "  /dev/video* :"
    if ! compgen -G "/dev/video*" >/dev/null; then
        echo "    (없다 — 카메라가 연결되지 않았다)"
    else
        for d in /dev/video*; do
            n="$(rd_v4l_name "$d")"
            printf '    %-14s %s\n' "$d" "${n:-?}"
        done
    fi
    # 이쪽이 번호와 달리 재열거에도 안 바뀐다 — 실제로 써야 할 경로다.
    echo "  /dev/v4l/by-id/ (안정 경로 — 이걸 쓸 것) :"
    if [ -d /dev/v4l/by-id ] && compgen -G "/dev/v4l/by-id/*" >/dev/null; then
        for d in /dev/v4l/by-id/*; do
            printf '    %s -> %s\n' "$d" "$(readlink -f "$d")"
        done
    else
        echo "    (없다)"
    fi
}

while [ $# -gt 0 ]; do
    case "$1" in
        --list)     rd_list_video; exit 0 ;;
        --device)   VIDEO="$2"; shift 2 ;;
        --save-dir) SAVE_DIR="$2"; shift 2 ;;
        --params)   PARAMS="$2"; shift 2 ;;
        -h|--help)  usage; exit 0 ;;
        *) PASS+=("$1"); shift ;;
    esac
done

[ -f "$NODE" ]   || { echo "ERROR: 노드 스크립트가 없다: $NODE" >&2; exit 1; }
[ -f "$PARAMS" ] || { echo "ERROR: 파라미터 파일이 없다: $PARAMS" >&2; exit 1; }

# 파라미터는 **뒤에 온 것이 이긴다** — yaml 을 먼저, 개별 -p 를 나중에 놓는다.
ARGS=(--params-file "$PARAMS")
if [ -n "$SAVE_DIR" ]; then
    mkdir -p "$SAVE_DIR"
    ARGS+=(-p save_dir:="$SAVE_DIR")
fi
if [ -n "$VIDEO" ]; then
    ARGS+=(-p video_device:="$VIDEO")
fi

# 실제로 열 장치를 미리 본다 — 없거나 권한이 없으면 노드는 "열기 실패" 만 반복한다.
# 옵션/환경변수로 덮지 않았으면 yaml 에 적힌 값이 그 장치다.
DEV="$VIDEO"
if [ -z "$DEV" ]; then
    DEV="$(awk -F'"' '/^[[:space:]]*video_device:/ {print $2; exit}' "$PARAMS")"
fi
DEV_NAME=""
if [ -n "$DEV" ] && [ ! -e "$DEV" ]; then
    echo "WARN: $DEV 이 없다 — 카메라 연결/전원 확인." >&2
    rd_list_video >&2
    echo "      다른 노드면: ./run_dpy_camera.sh --device <위 경로>" >&2
elif [ -n "$DEV" ] && { [ ! -r "$DEV" ] || [ ! -w "$DEV" ]; }; then
    echo "WARN: $DEV 에 읽기/쓰기 권한이 없다 ($(stat -c '%A %U:%G' "$DEV"))." >&2
    echo "      해결: sudo usermod -aG video $USER  후 재로그인(로그아웃/로그인)" >&2
    echo "      현재 그룹: $(id -nG)" >&2
elif [ -n "$DEV" ]; then
    # **번호가 맞는지가 아니라 그 노드가 어느 카메라인지**를 본다. 번호는 열거 순서라
    # 존재하고 권한이 있어도 다른 카메라일 수 있다 — 그러면 촬영이 0장으로 끝난다.
    DEV_NAME="$(rd_v4l_name "$DEV")"
    if [ -z "$DEV_NAME" ]; then
        echo "WARN: $DEV 의 이름을 못 읽었다 — v4l2 노드가 아닐 수 있다." >&2
    else
        echo "     장치 이름: $DEV_NAME"
        echo "     (의도한 카메라가 아니면 --list 로 확인하고 --device 로 지정할 것)"
    fi
    # by-id 경로가 있는데 번호로 지정했으면 알려 준다. 다음 재열거에 또 어긋난다.
    case "$DEV" in
        /dev/video*)
            for l in /dev/v4l/by-id/*; do
                [ -e "$l" ] || continue
                if [ "$(readlink -f "$l")" = "$(readlink -f "$DEV")" ]; then
                    echo "     안정 경로: $l"
                    echo "     (번호는 재열거마다 밀린다 — yaml 의 video_device 를 이 경로로 두는 편이 낫다)"
                    break
                fi
            done ;;
    esac
fi

echo "Orin Environment: Arducam USB Camera (dpy_camera on-demand capture node) Start"
echo "----------------------------------------------------------------------"
echo " 노드가 대기 상태로 뜹니다. (평소 CPU ~0%, 명령 시에만 캡처)"
echo " 장치     : ${DEV:-(yaml 기본값)}${DEV_NAME:+  [$DEV_NAME]}"
echo " 저장     : ${SAVE_DIR:-(저장 안 함)}"
echo " 사진 캡처: ros2 service call /dpy_camera/capture std_srvs/srv/Trigger {}"
echo " 결과 보기: ./run_dpy_rviz.sh"
echo "----------------------------------------------------------------------"

exec python3 "$NODE" --ros-args "${ARGS[@]}" ${PASS[@]+"${PASS[@]}"}
