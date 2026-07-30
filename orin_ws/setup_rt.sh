#!/bin/bash
# 200Hz 실기 조건 세팅 (ORIN_SET_GUIDE.md). **sudo 가 필요한 것만 여기 모은다.**
#
# 종전에는 이 중 latency_timer 가 build.sh 안에 있었다. 빌드가 비밀번호를 묻는 것은
# 이상하고, 더 중요하게는 **재부팅하면 사라지는 세팅을 빌드 시점에 하고 있었다** —
# 빌드는 한 번, 재부팅은 여러 번이다.
#
#   ./setup_rt.sh          # latency_timer + 포트 권한 확인
#   ./setup_rt.sh --perm   # 위 + dialout 그룹에 사용자 추가 (재로그인 필요)

set -uo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/_env.sh"

PORT_NAME="$(basename "$RD_PORT")"
LAT="/sys/bus/usb-serial/devices/$PORT_NAME/latency_timer"

echo "=== 1) latency_timer"
# FTDI 기본값 16ms 는 200Hz(5ms 주기)에 치명적이다 — 응답이 최대 16ms 늦게 올라와
# tick 이 밀린다. 1ms 로 낮춘다. **재부팅하면 되돌아간다.**
if [ ! -e "$LAT" ]; then
    echo "  SKIP: $LAT 이 없다 (장치 미연결이거나 FTDI 가 아니다)"
else
    cur="$(cat "$LAT")"
    if [ "$cur" = "1" ]; then
        echo "  이미 1 — 건드리지 않는다"
    else
        echo "  $cur -> 1 (sudo 필요)"
        echo 1 | sudo tee "$LAT" >/dev/null && echo "  이제 $(cat "$LAT")"
    fi
fi

echo "=== 2) 포트 권한"
if [ ! -e "$RD_PORT" ]; then
    echo "  SKIP: $RD_PORT 이 없다 — ECU 연결/전원 확인"
else
    echo "  $RD_PORT  $(stat -c '%A %U:%G' "$RD_PORT")"
    if [ -r "$RD_PORT" ] && [ -w "$RD_PORT" ]; then
        echo "  읽기/쓰기 가능"
    else
        echo "  ⚠ 권한 없음 — 브리지가 'Open Failed: Bad file descriptor' 로 무한 재시도한다."
        if [ "${1:-}" = "--perm" ]; then
            # 영구 해결. udev 가 장치를 dialout 그룹으로 만들므로 그룹에 들어가면 끝난다.
            echo "  usermod -aG dialout $USER (sudo 필요)"
            sudo usermod -aG dialout "$USER" \
              && echo "  → **재로그인(또는 재부팅) 해야 반영된다.** 지금 당장 쓰려면 아래 임시 방법."
        else
            echo "  영구 해결: ./setup_rt.sh --perm   (그룹 추가 후 재로그인)"
            echo "  임시 해결: sudo chmod 666 $RD_PORT   (USB 재열거되면 되돌아간다)"
        fi
    fi
fi

echo "=== 3) RT 스케줄링 권한"
# 브리지는 SCHED_FIFO 를 시도하고, 권한이 없으면 WARN 만 남기고 일반 우선순위로 계속 돈다
# (rd_schedule.cpp ApplyRtScheduling). 즉 조용히 지터가 늘어난다 — 미리 알려 준다.
soft="$(ulimit -Hr 2>/dev/null || echo 0)"
if [ "${soft:-0}" = "0" ] || [ "${soft:-0}" = "unlimited" ] 2>/dev/null; then :; fi
if [ "$(ulimit -r 2>/dev/null || echo 0)" = "0" ]; then
    echo "  ⚠ rtprio 한도가 0 — SCHED_FIFO 설정이 실패하고 일반 우선순위로 돈다 (지터 증가)."
    echo "    /etc/security/limits.conf 에 다음을 넣고 재로그인:"
    echo "      $USER  -  rtprio  99"
else
    echo "  rtprio 한도 $(ulimit -r) — SCHED_FIFO 가능"
fi

echo "=== 완료. 1)·2)는 재부팅/USB 재열거 후 다시 필요할 수 있다."
