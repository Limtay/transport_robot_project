#!/bin/bash
# 개발 머신 → Orin 코드 전송 (rsync). 빌드는 Orin 에서 한다.
#
#   ./deploy.sh                          # 전 패키지 전송
#   ./deploy.sh --host 192.168.55.1      # USB 이더넷으로
#   ./deploy.sh --dry                    # 무엇이 갈지만 보기
#   ./deploy.sh --build                   # 전송 후 원격 빌드까지
#
# 환경변수로도 지정된다: ORIN_HOST / ORIN_USER / ORIN_WS
#
# ## 종전 deploy.sh 와 달라진 점 (08 §1.3)
#
#   ① **패키지 5개 중 2개만 보냈다** (`orin_firmware_bridge`, `mgs01_base_msgs`).
#      재설계로 생긴 `mgs_tp_msgs`·`control_cli`·`control_web` 이 빠져 있어서, 이걸로
#      전송하면 **Orin 에서 빌드가 실패한다** — 브리지가 `mgs_tp_msgs` 를 의존한다.
#      패키지를 손으로 나열하지 않고 `src/` 를 통째로 보내 그 실수를 구조적으로 없앤다.
#   ② IP·사용자·경로가 하드코딩돼 있었다 (`swarm@10.251.24.214`, `~/tp_ws`).
#   ③ `--delete` 를 켠 채 산출물 제외가 없어서, Orin 의 `build/`·`install/` 이
#      개발 머신에 그것이 없다는 이유로 지워질 수 있었다 (매번 전체 재빌드).

set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/_env.sh"

HOST="${ORIN_HOST:-10.251.24.214}"
USER_="${ORIN_USER:-swarm}"
REMOTE_WS="${ORIN_WS:-~/orin_ws}"
DRY=""
DO_BUILD=0

while [ $# -gt 0 ]; do
    case "$1" in
        --host)  HOST="$2"; shift 2 ;;
        --user)  USER_="$2"; shift 2 ;;
        --ws)    REMOTE_WS="$2"; shift 2 ;;
        --dry)   DRY="--dry-run"; shift ;;
        --build) DO_BUILD=1; shift ;;
        -h|--help) sed -n '2,20p' "${BASH_SOURCE[0]}" | sed 's/^# \?//'; exit 0 ;;
        *) echo "ERROR: 모르는 옵션 '$1'" >&2; exit 2 ;;
    esac
done

TARGET="$USER_@$HOST"

# 먼저 닿는지 본다. 안 되면 rsync 가 30초쯤 매달린 뒤 애매한 에러를 낸다.
if ! ssh -o ConnectTimeout=5 -o BatchMode=yes "$TARGET" true 2>/dev/null; then
    echo "ERROR: $TARGET 에 SSH 로 못 붙는다." >&2
    echo "       ping 확인 → 키 등록 확인 (ssh-copy-id $TARGET)" >&2
    echo "       다른 주소면: ./deploy.sh --host <ip>   (USB 이더넷은 보통 192.168.55.1)" >&2
    exit 1
fi

echo ">>> $WS/src  →  $TARGET:$REMOTE_WS/src${DRY:+   (dry-run)}"

# ⚠ `src/` 를 통째로 보낸다 — 패키지를 나열하면 새 패키지가 생길 때마다 여기를 고쳐야
#   하고, 안 고치면 **원격 빌드가 깨질 때까지 아무도 모른다** (실제로 그랬다).
# ⚠ 산출물은 제외한다. `--delete` 와 함께 보내면 Orin 의 build/install 이 지워진다.
rsync -az --delete $DRY \
    --exclude='build/' --exclude='install/' --exclude='log/' \
    --exclude='__pycache__/' --exclude='*.pyc' \
    "$WS/src/" "$TARGET:$REMOTE_WS/src/"

# 스크립트도 같이 보낸다 — Orin 쪽 build.sh 가 낡으면 패키지 목록이 갈라진다.
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
