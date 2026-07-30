# 공통 환경 — 다른 .sh 들이 `source` 해서 쓴다. 직접 실행하지 않는다.
#
# ## 왜 파일 하나로 모으는가
#
# 종전 run.sh / cli.sh 는 `/home/swarm/tp_ws/orin_ws/install/setup.bash` 를 **각각 하드코딩**
# 하고 있었다. 개발 머신은 `/home/limtay/tp_ws` 라서 그 스크립트들은 개발 머신에서 동작하지
# 않았고, 경로를 고칠 때마다 두 곳을 같이 고쳐야 했다.
#
# 여기서는 **스크립트 자기 위치에서 워크스페이스를 찾는다** — 레포를 어디에 두든,
# 어느 계정이든 그대로 돈다.

# 이 파일이 있는 디렉터리 = orin_ws. `source` 로 불려도 BASH_SOURCE 는 이 파일을 가리킨다.
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export WS

ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"

if [ ! -f "$ROS_SETUP" ]; then
    echo "ERROR: ROS 를 못 찾았다: $ROS_SETUP" >&2
    echo "       다른 배포판이면 ROS_SETUP=/opt/ros/<distro>/setup.bash 로 지정할 것" >&2
    return 1 2>/dev/null || exit 1
fi

# ⚠ ROS 의 setup.bash 는 `set -u`(unbound variable = 에러) 아래에서 죽는다 —
#   `AMENT_TRACE_SETUP_FILES` 같은 변수를 정의 없이 참조한다. 호출 스크립트가 `set -euo
#   pipefail` 을 쓰는 것은 옳으므로, **source 구간에서만** 잠시 풀고 원래대로 되돌린다.
#   (이걸 안 하면 스크립트가 "AMENT_TRACE_SETUP_FILES: unbound variable" 로 즉사한다.)
rd_source_quietly() {
    local restore=""
    case "$-" in *u*) restore="u" ;; esac
    set +u
    # shellcheck disable=SC1090
    source "$1"
    local rc=$?
    [ -n "$restore" ] && set -u
    return $rc
}

rd_source_quietly "$ROS_SETUP"

# 워크스페이스 오버레이. **아직 빌드 안 했으면 없는 것이 정상**이라 여기서 죽이지 않는다
# (build.sh 는 이 파일을 source 한 뒤에 빌드하므로).
rd_source_ws() {
    if [ -f "$WS/install/setup.bash" ]; then
        rd_source_quietly "$WS/install/setup.bash"   # 위와 같은 이유로 set -u 를 잠시 푼다
        return 0
    fi
    echo "ERROR: $WS/install 이 없다 — 먼저 ./build.sh 를 돌릴 것" >&2
    return 1
}

# `ros2 run` 이 실행 파일을 찾을 수 있는지 미리 본다. 없으면 빌드가 덜 된 것이고,
# 그 상태로 띄우면 "No executable found" 만 나와 원인이 안 보인다.
rd_need_exe() {
    local pkg="$1" exe="$2"
    if ! ros2 pkg executables "$pkg" 2>/dev/null | grep -qx "$pkg $exe"; then
        echo "ERROR: $pkg 의 실행 파일 '$exe' 가 없다 — 그 패키지가 빌드되지 않았다." >&2
        echo "       ./build.sh $pkg" >&2
        return 1
    fi
    return 0
}

# ECU 포트. **없어도 멈추지 않는다** — 브리지는 USB 를 기다리며 뜨는 것이 정상 동작이다.
# 다만 흔한 두 함정(장치 없음 / 권한 없음)은 미리 말해 준다. 특히 권한은 USB 가
# 재열거되면 조용히 바뀌어서, 로그에 "Open Failed: Bad file descriptor" 만 남는다.
RD_PORT="${RD_PORT:-/dev/ttyUSB0}"
export RD_PORT

rd_check_port() {
    if [ ! -e "$RD_PORT" ]; then
        echo "WARN: $RD_PORT 이 없다 — ECU 가 연결/전원 상태인지 확인할 것" >&2
        return 0
    fi
    if [ ! -r "$RD_PORT" ] || [ ! -w "$RD_PORT" ]; then
        echo "WARN: $RD_PORT 에 읽기/쓰기 권한이 없다 ($(stat -c '%A %U:%G' "$RD_PORT"))." >&2
        echo "      브리지는 'Open Failed: Bad file descriptor' 만 남기고 계속 재시도한다." >&2
        echo "      해결: ./setup_rt.sh   (또는 sudo usermod -aG dialout $USER 후 재로그인)" >&2
    fi
    return 0
}
