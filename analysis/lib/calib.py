#!/usr/bin/env python3
"""로드셀 cnt ↔ N 환산 — 정준 상수는 `analysis/loadcell_cal.json` 하나뿐이다.

절대력이 필요하면 `to_N(cnt)` (offset 차감 포함), 상대력(tare 후)이면 `scale_N(dcnt)`.
구 분석기가 offset 을 안 빼고 곱하기만 해서 절대력을 틀리게 낸 적이 있다 — 두 함수를
나눠 둔 이유다.
"""
import json
import os

CAL_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'loadcell_cal.json')


def load(path=None):
    with open(path or CAL_FILE) as f:
        c = json.load(f)
    ch = c.get('traction_channel', 0)
    p = c['channels'][f'ch{ch}']
    return {'channel': ch, 'N_per_count': float(p['N_per_count']),
            'counts_per_N': float(p['counts_per_N']), 'offset_cnt': float(p['offset_cnt']),
            'linear_range_N': p.get('linear_range_N'), 'created': c.get('created')}


CAL = load()


def to_N(cnt):
    """**절대력** [N] — offset 차감 후 환산."""
    return (cnt - CAL['offset_cnt']) * CAL['N_per_count']


def scale_N(dcnt):
    """**상대력** [N] — 이미 tare 된 차분에 스케일만."""
    return dcnt * CAL['N_per_count']


def to_cnt(force_N):
    """힘 [N] → raw cnt (abort 임계를 cnt 로 환산할 때)."""
    return force_N * CAL['counts_per_N'] + CAL['offset_cnt']
