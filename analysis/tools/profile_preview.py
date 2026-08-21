#!/usr/bin/env python3
"""프로파일 YAML → **cmd current 미리보기 그래프** (실행 승인용).

RULES.md §2 의 승인 절차가 쓰는 도구다. 실험을 돌리기 **전에** 어떤 전류를 넣을 것인지
그림으로 만들어 사용자가 보고 승인/수정하게 한다.

  python3 tools/profile_preview.py profiles/t4_ramp_cycle.yaml
  python3 tools/profile_preview.py P.yaml --motor m1 -o profiles/preview/x.png

세그먼트 전개는 브리지(`rd_profile.cpp`)와 **같은 200 Hz 격자**로 재현한다 — 미리보기가
실제 재생과 다르면 승인이 무의미하기 때문이다. 단 `prbs`/`noise` 는 브리지 쪽 난수열을
재현할 수 없어 **띠(band)로만** 표시한다.

힘 예측은 `lib/ref_curve_w40_m2.csv` (payload 40 kg·모터 m2, 2026-07-22) 기준의 **참고값**이다.
모터·하중이 다르면 그대로 맞지 않는다 — 안전선 확인용으로만 본다.
"""
import argparse
import os
import sys

import numpy as np
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'lib'))
import calib                                    # noqa: E402
from plotstyle import plt, PALETTE, TEXT2       # noqa: E402

TICK_HZ = 200.0
DT = 1.0 / TICK_HZ
REF_CSV = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'lib',
                       'ref_curve_w40_m2.csv')

ABORT_N = 190.0                                 # test_plan §5 — 검증 상한 200.7 N 아래 10 N
VERIFIED_N = calib.CAL['linear_range_N'][1]


def expand(segs):
    """세그먼트 리스트 → (t, i_cmd, band) — band 는 (lo,hi) 또는 None (난수 구간 표시용)."""
    t_all, i_all, band = [], [], []
    t0 = 0.0
    for s in segs:
        typ = s['type']
        dur = float(s.get('duration', 0.0))
        if typ == 'stair':
            vals = [float(v) for v in s['values']]
            step = float(s['step_duration'])
            dur = step * len(vals)
            n = int(round(dur * TICK_HZ))
            t = np.arange(n) * DT
            idx = np.minimum((t / step).astype(int), len(vals) - 1)
            y = np.array(vals)[idx]
            bd = None
        elif typ == 'step':
            n = int(round(dur * TICK_HZ)); t = np.arange(n) * DT
            y = np.where(t < float(s['t_step']), float(s['from']), float(s['to']))
            bd = None
        else:
            n = int(round(dur * TICK_HZ)); t = np.arange(n) * DT
            if typ == 'hold':
                y = np.full(n, float(s['value'])); bd = None
            elif typ == 'ramp':
                a, b = float(s['from']), float(s['to'])
                y = a + (b - a) * (t / dur if dur > 0 else 0.0); bd = None
            elif typ == 'sine':
                off, amp = float(s.get('offset', 0.0)), float(s['amp'])
                y = off + amp * np.sin(2 * np.pi * float(s['freq']) * t); bd = None
            elif typ == 'chirp':
                off, amp = float(s.get('offset', 0.0)), float(s['amp'])
                f0, f1 = float(s['f0']), float(s['f1'])
                k = (f1 - f0) / dur if dur > 0 else 0.0
                y = off + amp * np.sin(2 * np.pi * (f0 * t + 0.5 * k * t * t)); bd = None
            elif typ in ('prbs', 'noise'):
                # 브리지 난수열은 재현 불가 — 중심선 + 띠로만 표시한다 (거짓 파형 금지)
                lo, hi = ((float(s['low']), float(s['high'])) if typ == 'prbs' else
                          (float(s['mean']) - 2 * float(s['std']),
                           float(s['mean']) + 2 * float(s['std'])))
                y = np.full(n, (lo + hi) / 2.0); bd = (lo, hi)
            elif typ == 'custom':
                v = [float(x) for x in s['values']]
                rate = float(s.get('rate', TICK_HZ))
                y = np.interp(t, np.arange(len(v)) / rate, v); bd = None
            else:
                raise ValueError(f'모르는 세그먼트 타입: {typ}')
        t_all.append(t + t0); i_all.append(y)
        band += [bd] * n
        t0 += dur
    return np.concatenate(t_all), np.concatenate(i_all), band


def predict_force(i_cmd):
    """참고곡선으로 힘 예측 — 상승은 rise 분기, 하강은 **반전점에 앵커한** fall 분기.

    참고표의 fall 열은 **14 A 에서 반전한** 런들의 평균이다. 그걸 10 A 반전 프로파일에 그대로
    쓰면 하행이 실제 도달 못 한 힘까지 올라간다 (07-22 FORC: 하강경로는 반전점에 따라 갈린다).
    그래서 하강 분기를 `F_rise(I_rev)` 에 맞춰 평행이동한다 — 반전점에서 두 분기가 만나고,
    이력 폭(fall > rise)은 그대로 보존된다. 1차 근사이며 안전선 판정용이다.
    """
    ref = np.genfromtxt(REF_CSV, delimiter=',', names=True)
    top = ref['I_A'][-1]
    slope = (ref['F_rise_N'][-1] - ref['F_rise_N'][-4]) / (top - ref['I_A'][-4])

    def rise(x):
        y = np.interp(x, ref['I_A'], ref['F_rise_N'])
        ext = np.asarray(x) > top
        return np.where(ext, ref['F_rise_N'][-1] + slope * (np.asarray(x) - top), y)

    def fall(x):
        return np.interp(x, ref['I_A'], ref['F_fall_N'])

    i_rev = float(np.max(i_cmd))                       # 이 궤적의 반전점
    shift = float(rise(i_rev) - fall(i_rev))           # 하강 분기를 피크에 앵커
    rising = np.gradient(i_cmd) >= 0
    return np.where(rising, rise(i_cmd), fall(i_cmd) + shift), i_rev > top


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('profile')
    ap.add_argument('--motor', default=None, help='기본: YAML 의 첫 모터')
    ap.add_argument('-o', '--out', default=None,
                    help='기본: profiles/preview/<name>.png')
    a = ap.parse_args()

    prof = yaml.safe_load(open(a.profile))
    motors = prof['motors']
    mkey = a.motor or sorted(motors)[0]
    t, i_cmd, band = expand(motors[mkey])
    f_pred, extrapolated = predict_force(i_cmd)

    name = prof.get('name', os.path.splitext(os.path.basename(a.profile))[0])
    out = a.out or os.path.join(os.path.dirname(os.path.abspath(__file__)), '..',
                                'profiles', 'preview', f'{name}.png')
    os.makedirs(os.path.dirname(out), exist_ok=True)

    lim = prof.get('limits', {})
    lim_v = lim.get('max_current', lim.get('max_abs'))
    peak_i, peak_f = float(np.max(np.abs(i_cmd))), float(np.nanmax(f_pred))

    fig, (ax, ax2) = plt.subplots(2, 1, figsize=(11, 6.4), sharex=True,
                                  gridspec_kw={'height_ratios': [1, 1], 'hspace': 0.16})

    ax.plot(t, i_cmd, color=PALETTE[0], lw=1.6)
    for k, bd in enumerate(band):                      # 난수 구간 띠
        if bd and (k == 0 or band[k - 1] != bd):
            k2 = k
            while k2 < len(band) and band[k2] == bd:
                k2 += 1
            ax.fill_between(t[k:k2], bd[0], bd[1], color=PALETTE[0], alpha=0.18, lw=0)
    if lim_v:
        ax.axhline(float(lim_v), color=PALETTE[4], ls='--', lw=1.0)
        ax.annotate(f'limits {float(lim_v):.1f} A', (t[-1], float(lim_v)), ha='right',
                    va='bottom', fontsize=8, color=PALETTE[4])
    t0 = 0.0                                            # 세그먼트 경계
    for s in motors[mkey]:
        d = (float(s['step_duration']) * len(s['values']) if s['type'] == 'stair'
             else float(s.get('duration', 0.0)))
        t0 += d
        ax.axvline(t0, color=TEXT2, lw=0.5, alpha=0.35)
    ax.set_ylabel('cmd current [A]')
    ax.set_title(f'{name} — {mkey}   (총 {t[-1]:.1f} s, 피크 {peak_i:.2f} A)', loc='left')

    ax2.plot(t, f_pred, color=PALETTE[1], lw=1.6)
    ax2.axhline(ABORT_N, color=PALETTE[4], ls='--', lw=1.2)
    ax2.annotate(f'abort {ABORT_N:.0f} N', (t[-1], ABORT_N), ha='right', va='top',
                 fontsize=8, color=PALETTE[4])
    ax2.axhline(VERIFIED_N, color=PALETTE[2], ls=':', lw=1.2)
    ax2.annotate(f'검증 상한 {VERIFIED_N:.1f} N', (t[-1], VERIFIED_N), ha='right', va='bottom',
                 fontsize=8, color=PALETTE[2])
    ax2.set_ylabel('예측 힘 [N] (참고)'); ax2.set_xlabel('t [s]')
    verdict = ('⚠ abort 선 초과 예측' if peak_f > ABORT_N else 'abort 선 아래')
    ax2.set_title(f'예측 피크 {peak_f:.0f} N — {verdict}'
                  + ('   ※ 참고곡선 범위(13.75 A) 밖 외삽 포함' if extrapolated else ''),
                  loc='left', color=(PALETTE[4] if peak_f > ABORT_N else TEXT2))
    ax2.text(0.005, -0.32, '예측은 payload 40 kg·모터 m2 (2026-07-22) 참고곡선 기준 — '
             '모터·하중이 다르면 그대로 맞지 않는다. 안전선 확인용.',
             transform=ax2.transAxes, fontsize=8, color=TEXT2)

    fig.savefig(out, dpi=140, bbox_inches='tight')
    print(f'{name}: {t[-1]:.1f}s  피크 {peak_i:.2f} A  예측피크 {peak_f:.0f} N  [{verdict}]')
    print(f'-> {os.path.normpath(out)}')
    return 1 if peak_f > ABORT_N else 0


if __name__ == '__main__':
    sys.exit(main())
