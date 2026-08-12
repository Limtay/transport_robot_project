#!/bin/bash
# 개발 머신 → Orin 코드 전송 (rsync). 빌드는 Orin 에서 한다.

#===========================================================================
# ## 사용법
#---------------------------------------------------------------------------
#   ./deploy.sh                  # 전송
#   ./deploy.sh --host orin1     # 대상 지정 (~/.ssh/config 별칭 또는 IP)
#   ./deploy.sh --dry            # 미리보기 — 무엇이 가고 무엇이 지워질지만 본다
#   ./deploy.sh --build          # 전송 후 원격에서 ./build.sh 까지
#
#   옵션      : --host <주소>  --user <계정>  --ws <원격경로>  --dry  --build
#   환경변수  : ORIN_HOST / ORIN_USER / ORIN_WS   (옵션을 주면 옵션이 이긴다)
#   기본값    : swarm@10.251.24.214 : ~/orin_ws
#
#===========================================================================
# ## 복제 목록
#---------------------------------------------------------------------------
#   1) src/ 아래 패키지 디렉터리  →  <원격WS>/src/<패키지>/
#        · 목록은 로컬 src/ 를 훑어 만든다 (PKG_DIRS) — 이름을 여기 적지 않는다
#        · SKIP_PKGS 에 적힌 패키지는 뺀다
#   2) 워크스페이스 루트 스크립트  →  <원격WS>/
#        _env.sh  build.sh  run.sh  web.sh  cli.sh  setup_rt.sh
#
#===========================================================================
# ## 전송 제외 패키지 (SKIP_PKGS)
#---------------------------------------------------------------------------
#   carrier_teleop   — 개발 머신에서 토픽을 만들어 넣는 용도. Orin 에는 필요 없다.
#
#===========================================================================
# ## 복제·삭제 제외 목록 (--exclude)
#---------------------------------------------------------------------------
#   build/   install/   log/   __pycache__/   .pytest_cache/   *.pyc
#
#   · 보내지 않는다. 그리고 Orin 에 있어도 지우지 않는다 — rsync 는 exclude 된 경로를
#     수신측에서 삭제로부터 보호한다.
#
#===========================================================================
# ## 삭제 규칙
#---------------------------------------------------------------------------
#   · 패키지를 **하나씩 따로** 보낸다. `--delete` 는 그 패키지 디렉터리 안에서만 돈다.
#       → 로컬 패키지에 없는 파일이 Orin 의 같은 패키지에 있으면 지운다 (낡은 파일 정리)
#       → src/ 아래 **전송 대상이 아닌 디렉터리는 rsync 가 보지 못하므로 지울 수 없다**
#   · 루트 스크립트 전송에는 `--delete` 가 없다. 덮어쓰기만 하고 지우지 않는다.
#   · 개발 머신 쪽 파일은 어떤 경우에도 바뀌지 않는다. 방향은 dev → Orin 뿐이다.
#===========================================================================
# ## 실행 순서
#---------------------------------------------------------------------------
#   1. _env.sh 로 워크스페이스 경로(WS)를 잡는다
#   2. SSH 로 대상에 닿는지 먼저 확인한다 (안 되면 전송 없이 멈춘다)
#   3. 패키지별 rsync → 루트 스크립트 rsync
#   4. --dry 면 여기서 끝, --build 면 원격에서 ./build.sh

set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/_env.sh"

HOST="${ORIN_HOST:-10.251.24.214}"
USER_="${ORIN_USER:-swarm}"
REMOTE_WS="${ORIN_WS:-~/orin_ws}"
DRY=""
DO_BUILD=0

# 위 헤더 주석을 그대로 출력한다. 줄 번호를 박지 않는다 — 주석이 늘면 어긋나기 때문.
usage() { awk 'NR>1 { if (!/^#/) exit; sub(/^# ?/, ""); print }' "${BASH_SOURCE[0]}"; }

while [ $# -gt 0 ]; do
    case "$1" in
        --host)  HOST="$2"; shift 2 ;;
        --user)  USER_="$2"; shift 2 ;;
        --ws)    REMOTE_WS="$2"; shift 2 ;;
        # `-vi` 를 같이 준다 — rsync 는 verbose 없이 dry-run 하면 아무것도 출력하지 않는다.
        # `-i` 가 변경 사유를 찍어 준다: `*deleting` 으로 시작하는 줄이 삭제될 파일이다.
        --dry)   DRY="--dry-run -vi"; shift ;;
        --build) DO_BUILD=1; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "ERROR: 모르는 옵션 '$1'" >&2; exit 2 ;;
    esac
done

TARGET="$USER_@$HOST"

# 먼저 닿는지 본다. 안 되면 rsync 가 30초쯤 매달린 뒤 애매한 에러를 낸다.
if ! ssh -o ConnectTimeout=5 -o BatchMode=yes "$TARGET" true 2>/dev/null; then
    echo "ERROR: $TARGET 에 SSH 로 못 붙는다." >&2
    echo "       ping 확인 → 키 등록 확인 (ssh-copy-id $TARGET)" >&2
    echo "       로봇이 2대 이상이면 별칭을 쓸 것: ./deploy.sh --host orin1" >&2
    echo "       등록 절차: ORIN_SET_GUIDE.md §10" >&2
    exit 1
fi

# 전송 제외 패키지. 앞뒤 공백으로 감싸 부분일치를 막는다.
SKIP_PKGS=" carrier_teleop "

# 복제 목록은 로컬 src/ 를 훑어 만든다 — 이름을 손으로 나열하면 새 패키지가 생길 때마다
# 여기를 고쳐야 하고, 안 고치면 원격 빌드가 깨질 때까지 아무도 모른다.
PKG_DIRS=()
for d in "$WS"/src/*/; do
    n="$(basename "$d")"
    [[ "$SKIP_PKGS" == *" $n "* ]] && continue
    PKG_DIRS+=("$n")
done
[ ${#PKG_DIRS[@]} -eq 0 ] && { echo "ERROR: $WS/src 에 보낼 패키지가 없다" >&2; exit 1; }

echo ">>> $TARGET:$REMOTE_WS/src${DRY:+   (dry-run)}"
echo "    보냄: ${PKG_DIRS[*]}"
echo "    제외:$SKIP_PKGS"

# 패키지를 하나씩 따로 보낸다 — `--delete` 의 범위를 그 패키지 안으로 가두기 위해서다.
# src/ 를 통째로 보내면 로컬에 없는 디렉터리가 전부 삭제 대상이 된다.
for n in "${PKG_DIRS[@]}"; do
    rsync -az --delete $DRY \
        --exclude='build/' --exclude='install/' --exclude='log/' \
        --exclude='__pycache__/' --exclude='.pytest_cache/' --exclude='*.pyc' \
        "$WS/src/$n/" "$TARGET:$REMOTE_WS/src/$n/"
done

# 스크립트도 같이 보낸다 — Orin 쪽 build.sh 가 낡으면 빌드하는 패키지 목록이 갈라진다.
# 여기엔 `--delete` 가 없다.
rsync -az $DRY "$WS"/{_env.sh,build.sh,run.sh,web.sh,cli.sh,setup_rt.sh} \
    "$TARGET:$REMOTE_WS/"

[ -n "$DRY" ] && exit 0

echo ">>> 전송 완료"
if [ "$DO_BUILD" = 1 ]; then
    echo ">>> 원격 빌드"
    # `bash -lc` — 로그인 셸이어야 /opt/ros 가 잡힌다.
    ssh "$TARGET" "bash -lc 'cd $REMOTE_WS && ./build.sh'"
else
    echo "    빌드:  ssh $TARGET 'cd $REMOTE_WS && ./build.sh'"
    echo "    또는:  ./deploy.sh --build"
fi
