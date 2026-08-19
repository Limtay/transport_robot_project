#!/bin/bash
# 빌드. 인자 없으면 전체, 있으면 그 패키지만.
#
#   ./build.sh                          # 전체 (의존 순서대로)
#   ./build.sh control_web control_cli  # 일부만
#   ./build.sh --test                   # 전체 빌드 + 단위 테스트
#
# ## 종전 build.sh 와 달라진 점
#
#   ① `mgs_tp_msgs` · `control_cli` · `control_web` 을 **빌드하지 않았다.** 재설계로 생긴
#      패키지 3개가 빠져 있어서, 웹을 띄우려 해도 실행 파일이 없었다.
#   ② `latency_timer` 를 여기서 건드렸다. 그건 빌드가 아니라 **런타임 세팅**이고 sudo 가
#      필요하다 — `setup_rt.sh` 로 옮겼다. 빌드가 비밀번호를 묻는 것은 이상하다.
#   ③ 경로를 하드코딩했다 (`cd ~/tp_ws/orin_ws`) — `_env.sh` 가 스크립트 위치에서 찾는다.

set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/_env.sh"
cd "$WS"

RUN_TEST=0
PKGS=()
for a in "$@"; do
    case "$a" in
        --test) RUN_TEST=1 ;;
        -*) echo "ERROR: 모르는 옵션 $a" >&2; exit 2 ;;
        *)  PKGS+=("$a") ;;
    esac
done

# ⚠ **메시지 패키지를 먼저, 따로 빌드한다.** colcon 이 의존을 보고 순서를 잡아 주지만,
#   생성된 헤더/파이썬 모듈은 `install/setup.bash` 를 **다시 source 해야** 다음 패키지가
#   본다. 한 번에 몰아 빌드하면 깨끗한 트리에서 첫 빌드가 실패한다.
MSG_PKGS=(mgs01_base_msgs mgs_tp_msgs)
# control_web 은 control_cli.record 를 **import** 한다 (기록을 CLI 와 한 모듈로 두기 위해 —
# 07 §5 W5). 그래서 control_cli 가 먼저 install 돼 있어야 한다.
# dpy_camera 는 우리 msgs 에 의존하지 않으므로(rclpy/sensor_msgs/std_srvs 만) 순서는 뒤여도 된다.
CODE_PKGS=(orin_firmware_bridge control_cli control_web carrier_teleop dpy_camera)

if [ ${#PKGS[@]} -gt 0 ]; then
    # 오타를 조용히 넘기지 않는다 — colcon 은 없는 패키지 이름을 그냥 무시한다.
    for q in "${PKGS[@]}"; do
        [ -d "src/$q" ] || { echo "ERROR: src/$q 가 없다 (오타?)" >&2; exit 2; }
    done
    # 요청받은 것만 남기되 **순서는 위 목록이 정한다** (인자 순서를 믿으면 의존이 깨진다).
    keep() {
        local -a out=()
        for p in "$@"; do
            for q in "${PKGS[@]}"; do [ "$p" = "$q" ] && out+=("$p") && break; done
        done
        # ⚠ `"${out[@]:-}"` 는 빈 배열을 **빈 문자열 하나**로 전개한다 — 호출부가 그걸
        #    패키지 이름으로 받으면 colcon 에 `''` 가 넘어간다. 개수를 보고 나간다.
        [ ${#out[@]} -eq 0 ] && return 0
        printf '%s\n' "${out[@]}"
    }
    mapfile -t MSG_PKGS  < <(keep "${MSG_PKGS[@]}"  | grep -v '^$' || true)
    mapfile -t CODE_PKGS < <(keep "${CODE_PKGS[@]}" | grep -v '^$' || true)
fi

# 파이썬 패키지는 symlink 로 깔아 `.py`/`.js`/`.html` 수정이 **재빌드 없이** 반영되게 한다.
# 웹 UI 를 만질 때 이게 없으면 매번 빌드해야 한다.
# (C++ 는 어차피 컴파일이 필요하므로 이득이 없다.)
# dpy_camera 는 ament_cmake 이지만 C++ 컴파일 대상이 없다 (파이썬 스크립트·launch·config 설치만).
# symlink 로 깔아야 capture_node.py / launch / params yaml 수정이 재빌드 없이 반영된다.
SYMLINK_PKGS=" control_cli control_web carrier_teleop dpy_camera "

build_group() {
    local -a group=("$@") plain=() symlink=()
    [ ${#group[@]} -eq 0 ] && return 0
    for p in "${group[@]}"; do
        [ -z "$p" ] && continue
        if [[ "$SYMLINK_PKGS" == *" $p "* ]]; then symlink+=("$p"); else plain+=("$p"); fi
    done
    if [ ${#plain[@]} -gt 0 ]; then
        echo ">>> build: ${plain[*]}"
        colcon build --packages-select "${plain[@]}" \
            --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
    fi
    if [ ${#symlink[@]} -gt 0 ]; then
        echo ">>> build (symlink): ${symlink[*]}"
        colcon build --packages-select "${symlink[@]}" --symlink-install
    fi
}

# 빈 배열 전개는 append 로 받는다 — 직접 `"${a[@]:-}"` 하면 빈 문자열 인자가 생긴다.
build_group ${MSG_PKGS[@]+"${MSG_PKGS[@]}"}
# 메시지가 새로 깔렸으면 오버레이를 다시 잡아야 다음 그룹이 헤더를 본다.
[ -f install/setup.bash ] && rd_source_ws
build_group ${CODE_PKGS[@]+"${CODE_PKGS[@]}"}

echo ">>> build 완료"

if [ "$RUN_TEST" = 1 ]; then
    rd_source_ws
    ALL=()
    ALL+=(${MSG_PKGS[@]+"${MSG_PKGS[@]}"})
    ALL+=(${CODE_PKGS[@]+"${CODE_PKGS[@]}"})
    [ ${#ALL[@]} -eq 0 ] && { echo ">>> 테스트할 패키지가 없다"; exit 0; }
    echo ">>> test: ${ALL[*]}"
    # 실패해도 결과 표를 먼저 보여 준다 — 어느 스위트가 깨졌는지가 종료 코드보다 유용하다.
    colcon test --packages-select "${ALL[@]}" --event-handlers console_direct- || true
    colcon test-result --all || true
    colcon test-result >/dev/null   # 실패가 하나라도 있으면 여기서 exit≠0
fi
