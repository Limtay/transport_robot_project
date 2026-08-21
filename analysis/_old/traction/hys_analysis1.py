#!/usr/bin/env python3
"""히스테리시스 분석 1단계 — 반복성 / 율의존성 / FORC 분기족.

분기 추출: cmd 의 구간별 기울기 부호로 rise/fall 을 나누고, fb(실측 전류) 격자에 binning.
출력: 콘솔 표 + analysis/result/ 플롯 3장.
"""
import json, os, re
import numpy as np
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt

CACHE = '/home/swarm/tp_ws/analysis/traction/hys_cache'
OUT = '/home/swarm/tp_ws/analysis/result'
IDX = json.load(open(os.path.join(CACHE, 'index.json')))

GRID = np.arange(0.25, 14.0, 0.5)   # 전류 bin 중심

def runs(pat):
    return sorted([i for i in IDX if re.match(pat, i['name'])], key=lambda i: i['name'])

def branches(z, smooth=41):
    """cmd 기울기 부호로 (rise, fall) 마스크 목록 반환. hold 는 제외."""
    cmd = z['cmd']; d = np.gradient(cmd)
    k = np.ones(smooth) / smooth
    ds = np.convolve(d, k, 'same')
    rise = ds > 1e-4; fall = ds < -1e-4
    return rise, fall

def binned(x, y, grid=GRID, half=0.25):
    out = np.full(len(grid), np.nan)
    for j, g in enumerate(grid):
        m = (x >= g - half) & (x < g + half)
        if m.sum() >= 5: out[j] = y[m].mean()
    return out

def slope(x, y, lo=8.0, hi=13.0):
    m = (x >= lo) & (x <= hi) & np.isfinite(y)
    if m.sum() < 20: return np.nan
    return np.polyfit(x[m], y[m], 1)[0]

def load(i): return np.load(i['npz'])

# ── 1) 반복성: forc_rev14 (2x3세트) + camp_ramp_med (x4) ──
print("=" * 70)
print("1) 반복성 — rise slope [N/A] (fit 8-13A) / 상승분기 곡선 RMS 산포")
rows = []
for group, pat in [('forc_rev14(6run,3세트)', r'h_forc_rev14_'), ('camp_ramp_med(4run,세트1)', r'c_ramp_med_')]:
    rs = runs(pat); sl = []; curves = []
    for i in rs:
        z = load(i); rise, fall = branches(z)
        sl.append(slope(z['fb'][rise], z['Ft'][rise]))
        curves.append(binned(z['fb'][rise], z['Ft'][rise]))
    sl = np.array(sl); C = np.vstack(curves)
    spread = np.nanstd(C, axis=0)
    print(f"  {group:28s} slopes={np.round(sl,2)}")
    print(f"  {'':28s} mean={np.nanmean(sl):.2f}  CV={100*np.nanstd(sl)/np.nanmean(sl):.1f}%  "
          f"curveRMS(8-13A)={np.nanmean(spread[(GRID>=8)&(GRID<=13)]):.2f}N")
    rows.append((group, C))

# ── 2) 율의존성: desc_slow/med/fast 하강분기 (각 6run) ──
print("=" * 70)
print("2) 율의존성 — 하강 분기 곡선 (rate 0.3 / 0.6 / 1.2 A/s, forc_rev14 fall=1.0)")
rate_curves = {}
for lab, pat in [('0.3', r'h_desc_slow_'), ('0.6', r'h_desc_med_'), ('1.2', r'h_desc_fast_'), ('1.0', r'h_forc_rev14_')]:
    C = []
    for i in runs(pat):
        z = load(i); rise, fall = branches(z)
        C.append(binned(z['fb'][fall], z['Ft'][fall]))
    rate_curves[lab] = np.vstack(C)
    m = np.nanmean(rate_curves[lab], axis=0); s = np.nanstd(rate_curves[lab], axis=0)
    band = (GRID >= 3) & (GRID <= 12)
    print(f"  rate {lab} A/s: n={len(C)}  intra-RMS(3-12A)={np.nanmean(s[band]):.2f}N")
means = {k: np.nanmean(v, axis=0) for k, v in rate_curves.items()}
band = (GRID >= 3) & (GRID <= 12)
inter = np.nanstd(np.vstack([means['0.3'], means['0.6'], means['1.2']]), axis=0)
intra = np.nanmean([np.nanmean(np.nanstd(rate_curves[k], axis=0)[band]) for k in ('0.3','0.6','1.2')])
print(f"  → inter-rate 곡선차 RMS(3-12A) = {np.nanmean(inter[band]):.2f}N  vs  intra-rate 반복산포 = {intra:.2f}N")
print(f"  → 판정: {'율의존성 유의 (점성항 필요)' if np.nanmean(inter[band]) > 2*intra else '속도무관 (rate-independent) — 이력연산자만으로 충분'}")

# ── 3) FORC: 반전점별 하강분기족 ──
print("=" * 70)
print("3) FORC — 반전점(8/10/12/14A)별 하강 분기")
forc = {}
for rev in (8, 10, 12, 14):
    C = []
    for i in runs(rf'h_forc_rev{rev:02d}_'):
        z = load(i); rise, fall = branches(z)
        C.append(binned(z['fb'][fall], z['Ft'][fall]))
    forc[rev] = np.nanmean(np.vstack(C), axis=0)
    v = forc[rev]; g = GRID[np.isfinite(v)]
    print(f"  rev{rev:2d}: F@rev={np.nanmax(v):6.1f}N   F@3A={v[np.argmin(abs(GRID-3))]:6.1f}N")

# ── 플롯 ──
fig, axes = plt.subplots(1, 3, figsize=(16, 4.6))
ax = axes[0]
for lab, C in [('forc_rev14', rows[0][1]), ('ramp_med(s1)', rows[1][1])]:
    for c in C: ax.plot(GRID, c, lw=0.7, alpha=0.6)
ax.set_title('repeatability: rise branches (all repeats)'); ax.set_xlabel('I [A]'); ax.set_ylabel('F tared [N]')
ax = axes[1]
for lab, color in [('0.3','tab:blue'), ('0.6','tab:green'), ('1.0','tab:orange'), ('1.2','tab:red')]:
    ax.plot(GRID, means[lab], lw=1.6, color=color, label=f'fall {lab} A/s')
ax.legend(); ax.set_title('rate dependence: mean fall branches'); ax.set_xlabel('I [A]')
ax = axes[2]
rise14 = np.nanmean(rows[0][1], axis=0)
ax.plot(GRID, rise14, 'k--', lw=1.5, label='main rise')
for rev, color in zip((8,10,12,14), ('tab:blue','tab:green','tab:orange','tab:red')):
    ax.plot(GRID, forc[rev], lw=1.5, color=color, label=f'fall from {rev}A')
ax.legend(fontsize=8); ax.set_title('FORC: fall branches by reversal point'); ax.set_xlabel('I [A]')
for a in axes: a.grid(alpha=0.3)
fig.tight_layout(); fig.savefig(os.path.join(OUT, 'hys1_rep_rate_forc.png'), dpi=110)
print("saved", os.path.join(OUT, 'hys1_rep_rate_forc.png'))
