#!/usr/bin/env python3
"""램프 사이클 bag → F(I) 곡선 · 지표 · 플롯.

`t4_ramp_cycle` / `t4_probe` 처럼 **hold → ramp up → hold → ramp down → hold** 구조의 런을
읽어 상행·하행을 나누고 지표를 뽑는다.

  python3 tools/ramp_analysis.py data/rosbags/t4_p1_w40_r01_08-21_21-00 [-o out.png]
  python3 tools/ramp_analysis.py RUN1 RUN2 RUN3 --label p1_w40   # 셀 = 반복 묶음

상행/하행 분리는 **`segment_index` 로** 한다 (전류 미분 부호가 아니라) — 홀드 구간의 노이즈로
부호가 흔들리는 문제를 원천적으로 피한다. 브리지가 세그먼트 번호를 200 Hz 로 실어 준다.
"""
import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'lib'))
import bagio                                     # noqa: E402
import calib                                     # noqa: E402
from plotstyle import plt, PALETTE, TEXT2, MONO  # noqa: E402

MOTOR = 0                 # m1 = 인덱스 0
PAYLOAD_KG = None         # --payload 로 주면 이용률(참고)을 같이 낸다
LC_CH = 0
BIN_A = 0.5               # F(I) 빈 폭
SLOPE_BAND = (8.0, 13.0)  # 기울기 산출 구간 [A] — 07-22 와 같은 밴드
GAP_AT_A = 7.0            # 이력 갭을 읽는 전류
DEADBAND_N = 5.0          # 이 힘을 넘는 첫 전류 = 데드밴드 I0
# 슬립 판정은 **위치 누적 이동**으로 한다. fb_velocity 는 10 RPM 단위로 양자화돼 있어
# (실측 고유값 …,-20,-10,0,10,20,…) max|vel| 은 최소 검출단위 한두 칸에 쉽게 걸린다 —
# 2026-08-21 프로브에서 max 20 RPM 이 떴지만 전 런 위치 이동은 +1.6°(벨트 0.8 mm)로
# 탄성 take-up 이었다. 실제로 슬립하면 위치가 계속 밀린다.
SLIP_HOLD_DEG = 5.0       # 정점 홀드 중 이만큼 밀리면 슬립 의심
# 리프트오프: 견인력의 모멘트가 트랙 전방을 들어올려 접지가 끊기는 현상. 마찰 포화와 달리
# 힘이 **평평해지는 게 아니라 무너지고** 모터가 헛돈다. 2026-08-22 A1(18.3kg)·14A 실측:
# 로드셀 최대 1.0 N, |v| 6650 RPM. 마찰 슬립(수십 RPM)과 자릿수가 다르다.
FREESPIN_RPM = 200.0      # 램프 중 이 이상 = 리프트오프/헛돎
VEL_QUANT_RPM = 10.0      # 관측된 속도 양자 (참고용)
ABORT_N = 190.0
# ── 접선 컴플라이언스(와인드업) ─────────────────────────────────────────────
# θ(F) = 스프로킷 각 vs 견인력. 벨트·기어의 **탄성**만이면 같은 힘에서 같은 각이 나온다
# (장력이 같으므로 하중 무관). 접지 패치가 부분슬립에 들어가면 그 초과분만큼 더 감긴다.
#   고착 구간 → θ(F) 선형 / 부분슬립 성장 → 아래로 휨(연화)
# 2026-08-21 실측: 100 N 에서 W1 +3.10° vs W2 +1.50° — 같은 힘인데 2배. 슬립 여유의
# **μ 를 몰라도 되는** 관측량이다.
# 고착 기준선은 **전류 구간**으로 잡는다 (힘 구간이 아니라). 2026-08-22: 힘 기준(20~60 N)으로
# 잡으니 경하중 A1(피크 47 N)에서 창이 안 잡혀 판정 불가였다. 전 하중이 겹치는 저전류
# 공통 구간(§1.1: ~6 A 까지 겹침)을 쓰면 하중과 무관하게 같은 기준이 된다.
STIFF_FIT_A = (2.0, 5.0)     # 이 전류 구간을 '고착 기준선'으로 본다
SOFTEN_TOL = 0.10            # 기준선 대비 각이 이만큼 초과하면 '연화 시작'
MU_MEMO = 0.4                # 이용률 참고용 마찰계수 — **가정이지 측정값이 아니다**


def analyze(run_dir):
    d = bagio.load_bag(run_dir)
    i_cmd = d['cmd'][:, MOTOR]
    lc = d['lc'][:, LC_CH]
    seg = d['segment_index']
    vel = d['fb_velocity'][:, MOTOR]

    segs = np.unique(seg)
    if len(segs) < 5:
        raise ValueError(f'세그먼트 {len(segs)}개 — 램프 사이클(5개)이 아니다: {run_dir}')
    s_tare, s_rise, s_top, s_fall, s_end = segs[0], segs[1], segs[2], segs[3], segs[4]

    tare = float(np.nanmean(lc[seg == s_tare]))          # 첫 hold = 영점
    F = calib.scale_N(lc - tare)                         # 상대력 [N]

    def curve(mask):
        """전류 빈별 힘 평균 → (I, F)."""
        ii, ff = i_cmd[mask], F[mask]
        edges = np.arange(0, np.nanmax(i_cmd) + BIN_A, BIN_A)
        idx = np.digitize(ii, edges) - 1
        out_i, out_f = [], []
        for b in range(len(edges) - 1):
            m = idx == b
            if m.sum() >= 3:
                out_i.append(float(np.nanmean(ii[m]))); out_f.append(float(np.nanmean(ff[m])))
        return np.array(out_i), np.array(out_f)

    Ir, Fr = curve(seg == s_rise)
    If, Ff = curve(seg == s_fall)

    band = (Ir >= SLOPE_BAND[0]) & (Ir <= SLOPE_BAND[1])
    slope = float(np.polyfit(Ir[band], Fr[band], 1)[0]) if band.sum() >= 3 else float('nan')
    over = Ir[Fr > DEADBAND_N]
    i0 = float(over[0]) if len(over) else float('nan')
    peak = float(np.nanmax(F[(seg == s_rise) | (seg == s_top)]))
    # curve() 는 전류 빈 순서대로 내므로 If/Ir 은 **둘 다 오름차순**이다. np.interp 는 xp 가
    # 오름차순일 것을 요구한다 — 하행이라고 뒤집으면 보간이 망가진다(갭 부호가 뒤집혔던 자리).
    gap = (float(np.interp(GAP_AT_A, If, Ff) - np.interp(GAP_AT_A, Ir, Fr))
           if len(If) and len(Ir) else float('nan'))
    ret = float(np.nanmean(F[seg == s_end]))
    hold = (seg == s_top)
    pos = d['fb_position'][:, MOTOR]
    slip_deg = float(pos[hold][-1] - pos[hold][0]) if hold.any() else float('nan')
    travel_deg = float(pos[-1] - pos[0])
    vel_max = float(np.nanmax(np.abs(vel[hold]))) if hold.any() else float('nan')
    drive = (seg == s_rise) | (seg == s_top)
    vel_run_max = float(np.nanmax(np.abs(vel[drive])))
    liftoff = bool(vel_run_max > FREESPIN_RPM)

    # ── θ(F) 접선 컴플라이언스 ──────────────────────────────────────────
    th = pos - float(np.nanmean(pos[seg == s_tare]))
    rise_m = seg == s_rise
    Fr_t, th_t = F[rise_m], th[rise_m]
    Ir_t = i_cmd[rise_m]
    fit = (Ir_t >= STIFF_FIT_A[0]) & (Ir_t <= STIFF_FIT_A[1])
    if fit.sum() >= 20:
        kt = float(np.polyfit(Fr_t[fit], th_t[fit], 1)[0])        # [deg/N] 고착 기준선
        base = np.polyval(np.polyfit(Fr_t[fit], th_t[fit], 1), Fr_t)
        excess = th_t - base                                       # 기준선 초과 = 누적 미세슬립
        tol = SOFTEN_TOL * np.maximum(base, 1e-6)
        bad = np.where((excess > tol) & (Ir_t > STIFF_FIT_A[1]))[0]
        soften_F = float(Fr_t[bad[0]]) if len(bad) else float('nan')
        soften_I = float(Ir_t[bad[0]]) if len(bad) else float('nan')
        theta_excess = float(excess[np.argmax(Fr_t)])
    else:
        kt = soften_F = soften_I = theta_excess = float('nan')

    return {
        'run': os.path.basename(os.path.normpath(run_dir)),
        'n': d['n'], 'dur_s': round(float(d['t'][-1]), 2),
        'tare_cnt': round(tare, 1),
        'peak_N': round(peak, 2), 'peak_I_A': round(float(np.nanmax(i_cmd)), 2),
        'slope_N_per_A': round(slope, 3), 'deadband_I0_A': round(i0, 2),
        f'gap_at_{GAP_AT_A:.0f}A_N': round(gap, 2),
        'baseline_return_N': round(ret, 2),
        'slip_hold_deg': round(slip_deg, 2),     # ★ 슬립 판정은 이것
        'travel_deg': round(travel_deg, 2),
        'vel_max_rpm': round(vel_max, 1),        # 참고 (10 RPM 양자)
        'slope_band_A': [round(float(Ir[band].min()), 2), round(float(Ir[band].max()), 2)]
                        if band.sum() >= 3 else None,
        'windup_deg_per_N': round(kt, 5),        # 고착 기준선 기울기 (20~60 N)
        'soften_onset_N': round(soften_F, 1),    # 여기서부터 미세슬립이 자란다 (NaN=끝까지 고착)
        'soften_onset_A': round(soften_I, 2),
        'theta_excess_deg': round(theta_excess, 2),   # 피크에서 기준선 초과 각
        'theta_max_deg': round(float(np.nanmax(th)), 2),
        'theta_return_deg': round(float(np.nanmean(th[seg == s_end])), 2),
        'utilization': round(peak / (MU_MEMO * PAYLOAD_KG * 9.80665), 3)
                       if PAYLOAD_KG else None,
        'vel_run_max_rpm': round(vel_run_max, 0),
        'liftoff': liftoff,      # True = 트랙 들림/헛돎 — 그 런은 무효
        'abort_exceeded': bool(peak > ABORT_N),
        'rw_err_nonzero': int((d['rw_err'] != 0).sum()),
        '_curves': {'I_rise': Ir.tolist(), 'F_rise': Fr.tolist(),
                    'I_fall': If.tolist(), 'F_fall': Ff.tolist()},
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('runs', nargs='+')
    ap.add_argument('--label', default=None)
    ap.add_argument('-o', '--out', default=None)
    ap.add_argument('--json', dest='js', default=None)
    ap.add_argument('--payload', type=float, default=None,
                    help='총 질량 [kg] — 이용률 F/(μWg) 참고값 산출용 (μ=0.4 가정)')
    a = ap.parse_args()

    global PAYLOAD_KG
    PAYLOAD_KG = a.payload
    res = [analyze(r) for r in a.runs]
    label = a.label or res[0]['run']

    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(11.5, 4.6), gridspec_kw={'wspace': 0.26})
    for k, r in enumerate(res):
        c = r['_curves']; col = PALETTE[k % len(PALETTE)]
        ax.plot(c['I_rise'], c['F_rise'], color=col, lw=1.5, label=f"{r['run']} 상행")
        ax.plot(c['I_fall'], c['F_fall'], color=col, lw=1.5, ls='--', alpha=0.8)
    ax.axhline(ABORT_N, color=PALETTE[4], ls=':', lw=1.0)
    ax.set_xlabel('cmd current [A]'); ax.set_ylabel('힘 [N] (tare 후)')
    ax.set_title(f'{label} — 실선 상행 / 파선 하행', loc='left')
    if len(res) <= 3:
        ax.legend(fontsize=8, loc='upper left')

    keys = ['peak_N', 'slope_N_per_A', 'deadband_I0_A', f'gap_at_{GAP_AT_A:.0f}A_N',
            'baseline_return_N', 'slip_hold_deg', 'windup_deg_per_N',
            'soften_onset_N', 'theta_excess_deg']
    ax2.axis('off')
    ax2.text(0, 1.0, f'{"지표":<22}' + ''.join(f'{r["run"][-6:]:>11}' for r in res),
             family=MONO, fontsize=8.5, va='top', transform=ax2.transAxes)
    for j, k in enumerate(keys):
        vals = [r[k] for r in res]
        line = f'{k:<22}' + ''.join(f'{v:>11.2f}' for v in vals)
        if len(vals) > 1:
            line += f'   CV {100 * np.std(vals) / max(1e-9, abs(np.mean(vals))):5.2f}%'
        ax2.text(0, 0.90 - j * 0.085, line, family=MONO, fontsize=8.5,
                 va='top', transform=ax2.transAxes, color=TEXT2)
    warn = [f"{r['run']}: **리프트오프** ({r['vel_run_max_rpm']:.0f} RPM 헛돎)"
            for r in res if r.get('liftoff')]
    warn += [f"{r['run']}: abort 초과" for r in res if r['abort_exceeded']]
    warn += [f"{r['run']}: 슬립 의심 (홀드 중 {r['slip_hold_deg']}°)" for r in res
             if abs(r['slip_hold_deg']) > SLIP_HOLD_DEG]
    warn += [f"{r['run']}: rw_err {r['rw_err_nonzero']}건" for r in res if r['rw_err_nonzero']]
    ax2.text(0, 0.90 - len(keys) * 0.085 - 0.06,
             '\n'.join(warn) if warn else '이상 없음',
             fontsize=9, va='top', transform=ax2.transAxes,
             color=(PALETTE[4] if warn else PALETTE[1]), weight='bold')

    out = a.out or os.path.join('result', f'{label}.png')
    os.makedirs(os.path.dirname(out) or '.', exist_ok=True)
    fig.savefig(out, dpi=140, bbox_inches='tight')

    for r in res:
        print(json.dumps({k: v for k, v in r.items() if k != '_curves'}, ensure_ascii=False))
    print(f'-> {out}')
    if a.js:
        json.dump(res, open(a.js, 'w'), ensure_ascii=False, indent=1)


if __name__ == '__main__':
    main()
