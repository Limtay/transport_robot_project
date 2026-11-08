#!/bin/bash
# 측정 기록 — 브리지를 position 으로 띄우고 곧바로 bag 을 받는다. Ctrl-C 한 번이면 둘 다 내린다.
#
#   ./rec.sh -m "로드셀_20kg"          # data/rosbags/s<날짜시간>_로드셀_20kg/
#   ./rec.sh -m "서스펜션_1" -a 1,2    # 활성 모터 지정 (기본 1)
#   ./rec.sh -m "시험" -p comm_diag_enable:=true   # 그 외 브리지 파라미터는 그대로 통과
#
# 폴더: bag/ (pos_feedback 하나 — PosInstFeedback, redesign/14), bridge.log, bag.log
#   로드셀 = loadcell_raw, 리니어 엔코더 = lin_xy (x 10 um, y 20 um), RLS = rls_*, 모터 = fb_* / mtr_*
#
# ⚠ 활성 모터는 기록 내내 현재 위치 유지 전류를 받는다 — 손으로 밀면 되돌아가려 하고 열이 오른다.
#
# 종료 순서는 bag → 브리지다. 반대면 bag 끝에 브리지가 사라진 구간이 남는다.
# bag 은 SIGKILL 하면 메타데이터가 안 써져 열리지 않으므로 SIGINT 로만 끝낸다.

set -uo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/_env.sh"
rd_source_ws || exit 1

READY_MARK='run_profile action 준비됨'     # INIT(POS_FB 프로브 포함) 통과 뒤에 나온다
READY_TIMEOUT_SEC=30
BAG_SETTLE_SEC=1                         # record.BAG_SETTLE_SEC 와 같은 이유 — 구독이 붙을 시간

usage() { sed -n '2,14p' "${BASH_SOURCE[0]}" | sed 's/^# \?//'; }

NAME=""
MOTORS="1"
EXTRA=()
while [ $# -gt 0 ]; do
    case "$1" in
        -m) NAME="${2:-}"; shift 2 || exit 2 ;;
        -a) MOTORS="${2:-}"; shift 2 || exit 2 ;;
        -p) EXTRA+=(-p "${2:-}"); shift 2 || exit 2 ;;
        -h|--help) usage; exit 0 ;;
        *) echo "ERROR: 모르는 옵션 '$1'" >&2; usage >&2; exit 2 ;;
    esac
done
if [ -z "$NAME" ]; then
    echo "ERROR: 실험 이름이 없다 — ./rec.sh -m \"이름\"" >&2
    exit 2
fi
if ! [[ "$MOTORS" =~ ^[1-4](,[1-4])*$ ]]; then
    echo "ERROR: -a 는 1~4 를 쉼표로 (예: 1,2): '$MOTORS'" >&2
    exit 2
fi

if pgrep -x comm_test_node >/dev/null 2>&1; then
    echo "ERROR: comm_test_node 가 이미 돌고 있다 (pid $(pgrep -x comm_test_node | tr '\n' ' ')) — 먼저 내릴 것." >&2
    exit 1
fi

LAT_FILE="/sys/bus/usb-serial/devices/$(basename "$RD_PORT")/latency_timer"
if [ -r "$LAT_FILE" ] && [ "$(cat "$LAT_FILE")" != "1" ]; then
    echo "WARN: latency_timer=$(cat "$LAT_FILE") — 200Hz 가 안 나온다. echo 1 | sudo tee $LAT_FILE" >&2
fi

# ⚠ CWD 를 레포 루트로 (cli.sh·web.sh 와 같은 이유) — `data/rosbags` 는 상대경로다.
cd "$WS/.." || exit 1

read -r LABEL TOPICS < <(NAME="$NAME" python3 -c "
import os
from control_cli import record
print(record.sanitize_name(os.environ['NAME']), ' '.join(record.record_topics(pos=True)))
")
if [ -z "${TOPICS:-}" ]; then
    echo "ERROR: 기록 토픽 목록을 못 읽었다 — control_cli 가 빌드됐는지 확인할 것" >&2
    exit 1
fi

STEM="s$(date +%Y%m%d_%H%M%S)_${LABEL}"
OUT="data/rosbags/$STEM"
mkdir -p "$OUT" || exit 1
LOG="$OUT/bridge.log"

BRIDGE=""
BAG=""
STOPPING=0

# 그룹 전체에 SIGINT → 안 끝나면 SIGTERM. `ros2 run` 이 실행 파일을 자식으로 띄우므로 그룹으로 보낸다.
stop_group() {
    local pid="$1" name="$2" grace="$3"
    [ -n "$pid" ] || return 0
    kill -INT -- "-$pid" 2>/dev/null || return 0
    for _ in $(seq 1 $((grace * 10))); do
        kill -0 "$pid" 2>/dev/null || return 0
        timeout 0.1 tail -f /dev/null
    done
    echo "WARN: $name 가 SIGINT 에 안 끝나 SIGTERM 을 보낸다" >&2
    kill -TERM -- "-$pid" 2>/dev/null
    for _ in $(seq 1 30); do
        kill -0 "$pid" 2>/dev/null || return 0
        timeout 0.1 tail -f /dev/null
    done
    echo "WARN: $name 가 안 끝나 SIGKILL 을 보낸다" >&2
    kill -KILL -- "-$pid" 2>/dev/null
}

shutdown_all() {
    [ "$STOPPING" = 1 ] && return 0
    STOPPING=1
    trap '' INT TERM HUP              # 종료 중 Ctrl-C 를 더 눌러도 순서를 깨지 않는다
    if [ -n "$BAG" ]; then
        echo ">>> bag 닫는 중…"
        stop_group "$BAG" "bag" 15
    fi
    if [ -n "$BRIDGE" ]; then
        echo ">>> 브리지 내리는 중…"
        stop_group "$BRIDGE" "브리지" 3
    fi
    BAG=""
    BRIDGE=""
}

on_signal() { shutdown_all; RC=0; }
RC=""
trap on_signal INT TERM HUP
trap shutdown_all EXIT                # 어떤 경로로 끝나도 브리지를 남기지 않는다 (모터 유지 전류)

# 자식을 띄우는 순간에만 잡 컨트롤을 켠다. 자식이 각자 프로세스 그룹을 가져 터미널의 Ctrl-C 를
# 같이 받지 않고, 끄고 띄우면 bash 가 자식의 SIGINT 를 무시로 두어 bag 이 SIGINT 로 안 끝날 수 있다.
# 계속 켜 두면 전경 명령이 터미널을 가져가 Ctrl-C 가 이 스크립트에 안 온다.
echo ">>> 브리지 기동 (active_motors=[$MOTORS])  로그: $LOG"
set -m
"$WS/run.sh" position -p "active_motors:=[$MOTORS]" ${EXTRA[@]+"${EXTRA[@]}"} >"$LOG" 2>&1 </dev/null &
BRIDGE=$!
set +m

for _ in $(seq 1 $((READY_TIMEOUT_SEC * 5))); do
    [ -n "$RC" ] && exit "$RC"
    grep -q "$READY_MARK" "$LOG" 2>/dev/null && break
    if ! kill -0 "$BRIDGE" 2>/dev/null; then
        BRIDGE=""
        echo "ERROR: 브리지가 기동 중에 끝났다 — bag 없음. 마지막 로그:" >&2
        tail -n 5 "$LOG" | cut -c1-160 >&2
        exit 1
    fi
    timeout 0.2 tail -f /dev/null
done
if ! grep -q "$READY_MARK" "$LOG" 2>/dev/null; then
    echo "ERROR: 브리지가 ${READY_TIMEOUT_SEC}초 안에 준비되지 않았다 — ECU 연결·전원 확인. bag 없음. 마지막 로그:" >&2
    tail -n 5 "$LOG" | cut -c1-160 >&2
    exit 1
fi

set -m
# shellcheck disable=SC2086
ros2 bag record -o "$OUT/bag" $TOPICS >"$OUT/bag.log" 2>&1 </dev/null &
BAG=$!
set +m
timeout "$BAG_SETTLE_SEC" tail -f /dev/null

echo ">>> 기록 중: $OUT"
echo "    Ctrl-C 한 번으로 bag 과 브리지를 함께 내린다"

while [ -z "$RC" ]; do
    if ! kill -0 "$BRIDGE" 2>/dev/null; then
        BRIDGE=""
        echo "ERROR: 브리지가 기록 중에 끝났다. 마지막 로그:" >&2
        tail -n 5 "$LOG" | cut -c1-160 >&2
        RC=1
    elif ! kill -0 "$BAG" 2>/dev/null; then
        BAG=""
        echo "ERROR: bag 기록이 먼저 끝났다 — $OUT/bag.log 확인" >&2
        RC=1
    else
        wait -n 2>/dev/null
        timeout 0.2 tail -f /dev/null
    fi
done
shutdown_all
trap - EXIT

# 빈 bag 은 끝난 직후에 알아야 다시 측정할 수 있다.
python3 - "$OUT/bag" 2>/dev/null <<'PY' || echo "WARN: 요약 실패 — bag 이 열리지 않을 수 있다 ($OUT/bag)" >&2
import sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from mgs_tp_msgs.msg import PosInstFeedback

UNREAD = -2147483648
reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='sqlite3'),
            rosbag2_py.ConverterOptions('cdr', 'cdr'))
load, lin = [], []
stamps = []
while reader.has_next():
    topic, data, t = reader.read_next()
    stamps.append(t)
    if topic.endswith('/pos_feedback'):
        m = deserialize_message(data, PosInstFeedback)
        load.append(list(m.loadcell_raw))
        lin.append(list(m.lin_xy))


def show(label, rows):
    if not rows:
        print('  %s: ⚠ 메시지 0건 — 다시 측정할 것' % label)
        return
    a = np.array(rows, dtype=np.int64)
    for ch in range(a.shape[1]):
        ok = a[:, ch][a[:, ch] != UNREAD]
        if not ok.size:
            print('  %s[%d]: ⚠ %d건 전부 미판독' % (label, ch, a.shape[0]))
        else:
            print('  %s[%d]: %d건 min=%d max=%d 평균=%.1f 미판독=%d' % (
                label, ch, a.shape[0], ok.min(), ok.max(), ok.mean(), a.shape[0] - ok.size))


dur = (max(stamps) - min(stamps)) * 1e-9 if stamps else 0.0
print('--- 기록 요약 (%.1f초)' % dur)
show('loadcell_raw', load)
show('lin_xy', lin)
PY
echo ">>> 저장: $OUT"
exit "$RC"
