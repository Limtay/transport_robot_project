#!/usr/bin/env python3
"""rosbag2 → numpy. **`ControlFeedback` 전용** (재설계 후 포맷).

## 왜 손으로 CDR 을 파싱하지 않는가

구 `traction_analysis.py` 는 바이트 오프셋을 손으로 계산해 `TestbedFeedback` 을 읽었다.
토픽이 `/carrier/testbed/feedback` → `/carrier/control/feedback` 으로 개명되고 메시지가
`ControlFeedback` 으로 바뀐 뒤에도 **토픽 이름 끝이 `/feedback` 이라는 이유로 계속 매칭돼**
엉뚱한 오프셋을 읽었다. 크래시하지 않고 조용히 틀린 값(lc 상수 −0.1 N, cmd NaN)을 냈다.

지금은 **메시지 타입으로 판별하고 rosidl 역직렬화를 쓴다.** `ControlFeedback` 은 첫 필드가
`std_msgs/Header`(가변길이 `frame_id` 문자열)라 **고정 오프셋 파싱이 원리적으로 불가능**하다.
타입이 안 맞으면 조용히 틀리는 대신 **거부한다.**

의존성: ROS 2 환경 source 필요 (`source /opt/ros/humble/setup.bash && source install/setup.bash`).
"""
import glob
import os
import sqlite3

import numpy as np

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

SUPPORTED_TYPE = 'mgs_tp_msgs/msg/ControlFeedback'
INT32_MIN = -2147483648


class BagFormatError(RuntimeError):
    """지원하지 않는 bag — 조용히 틀린 값을 내는 대신 여기서 멈춘다."""


def _find_db3(bag_dir):
    """bag 폴더 안의 .db3 — `<run>/bag/*.db3` 중첩도 허용 (control_cli 기록 규격)."""
    for pat in ('*.db3', os.path.join('bag', '*.db3')):
        hits = sorted(glob.glob(os.path.join(bag_dir, pat)))
        if hits:
            return hits
    raise FileNotFoundError(f'.db3 없음: {bag_dir}')


def _pick_topic(con):
    """topics 테이블 → (topic_id, msg_class). **타입으로** 고른다 (이름 아님)."""
    rows = con.execute('SELECT id, name, type FROM topics').fetchall()
    for tid, name, typ in rows:
        if typ == SUPPORTED_TYPE:
            return tid, get_message(typ)
    listed = ', '.join(f'{n}({t})' for _, n, t in rows) or '(토픽 없음)'
    raise BagFormatError(
        f'{SUPPORTED_TYPE} 토픽이 없다 — 이 bag 은 구 포맷이다.\n'
        f'  담긴 토픽: {listed}\n'
        f'  구 TestbedFeedback bag 은 analysis/_old/ 의 구 분석기로만 읽을 수 있다.')


def load_bag(bag_dir):
    """bag 폴더 → dict of arrays. 중복 tick 제거, 시간축은 0 기준 [s].

    반환 키:
      t             [s] header.stamp 기준 (ECU 취득시각을 Orin 축으로 보정한 값)
      ecu_tick      raw ×0.1ms (분석 권위 소스)
      cmd, fb_current, fb_velocity, fb_position   (n,4)
      lc            (n,2) [cnt] — 미판독은 NaN
      control_state, goal_id, segment_index, profile_time, rw_err
      stamp_valid
    """
    msgs = []
    msg_cls = None
    for path in _find_db3(bag_dir):
        con = sqlite3.connect(path)
        try:
            tid, msg_cls = _pick_topic(con)
            rows = con.execute(
                'SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp',
                (tid,)).fetchall()
        finally:
            con.close()
        try:
            msgs += [deserialize_message(blob, msg_cls) for (blob,) in rows]
        except Exception as e:                       # noqa: BLE001 — 원인을 사람 말로 바꿔 준다
            raise BagFormatError(
                f'{SUPPORTED_TYPE} 역직렬화 실패: {path}\n'
                f'  타입 이름은 맞지만 **필드 구성이 지금 빌드와 다르다** — 그 사이 msg 가\n'
                f'  바뀐 옛 bag 이다. 그때의 mgs_tp_msgs 로만 읽을 수 있다.\n'
                f'  원인: {e}') from e

    if not msgs:
        raise BagFormatError(f'{SUPPORTED_TYPE} 메시지가 0건이다: {bag_dir}')

    n = len(msgs)
    out = {
        'ecu_tick':      np.array([m.ecu_tick for m in msgs], np.int64),
        'stamp_valid':   np.array([m.stamp_valid for m in msgs], bool),
        'control_state': np.array([m.control_state for m in msgs], np.uint8),
        'goal_id':       np.array([m.goal_id for m in msgs], np.int64),
        'segment_index': np.array([m.segment_index for m in msgs], np.int64),
        'profile_time':  np.array([m.profile_time for m in msgs], np.float64),
        'rw_err':        np.array([m.rw_err for m in msgs], np.uint8),
        'cmd':           np.array([m.cmd for m in msgs], np.float64),
        'fb_current':    np.array([m.fb_current for m in msgs], np.float64),
        'fb_velocity':   np.array([m.fb_velocity for m in msgs], np.float64),
        'fb_position':   np.array([m.fb_position for m in msgs], np.float64),
    }
    stamp = np.array([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for m in msgs])

    lc = np.array([m.loadcell_raw for m in msgs], np.float64)
    lc[lc == INT32_MIN] = np.nan            # 미판독 — 0 은 "하중 없음" 이라 sentinel 이 못 된다
    out['lc'] = lc

    # 중복 tick 제거 (같은 RW 트랜잭션이 두 번 발행되는 경우)
    keep = np.concatenate([[True], np.diff(out['ecu_tick']) != 0])
    for k, v in out.items():
        out[k] = v[keep]
    out['t'] = stamp[keep] - stamp[keep][0]
    out['n'] = int(keep.sum())
    out['bag_dir'] = bag_dir
    return out


def summary(d):
    """한 줄 요약 — 로딩이 제대로 됐는지 사람이 눈으로 보는 용도."""
    lc0 = d['lc'][:, 0]
    ok = np.isfinite(lc0)
    rate = d['n'] / max(1e-9, d['t'][-1] - d['t'][0])
    return (f"n={d['n']:6d}  t={d['t'][-1]:7.2f}s (~{rate:.0f} Hz)  "
            f"cmd[{np.nanmin(d['cmd']):.2f},{np.nanmax(d['cmd']):.2f}]A  "
            f"lc0[{np.nanmin(lc0) if ok.any() else float('nan'):.0f},"
            f"{np.nanmax(lc0) if ok.any() else float('nan'):.0f}]cnt  "
            f"미판독 {int((~ok).sum())}")
