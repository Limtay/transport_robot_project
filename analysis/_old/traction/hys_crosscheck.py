#!/usr/bin/env python3
"""교차검증 — (1) 상승기울기: 같은날 rate 0.2/0.375/0.7 비교(rate vs 날짜효과 분리)
             (2) 07-22 학습 Preisach 를 07-21 bag(c_prbs, c_stair, c_ramp_med)에 적용(이전성)."""
import json, os, re
import numpy as np

CACHE = '/home/swarm/tp_ws/analysis/traction/hys_cache'
IDX = json.load(open(os.path.join(CACHE, 'index.json')))
def runs(pat): return sorted([i for i in IDX if re.match(pat, i['name'])], key=lambda i: i['name'])
def load(i): return np.load(i['npz'])
GRID = np.arange(0.25, 14.0, 0.5)

def branches(z, smooth=41):
    d = np.convolve(np.gradient(z['cmd']), np.ones(smooth)/smooth, 'same')
    return d > 1e-4, d < -1e-4

def binned(x, y):
    out = np.full(len(GRID), np.nan)
    for j, g in enumerate(GRID):
        m = (x >= g-0.25) & (x < g+0.25)
        if m.sum() >= 5: out[j] = y[m].mean()
    return out

def slope(x, y, lo=8, hi=13):
    m = (x>=lo)&(x<=hi)&np.isfinite(y)
    return np.polyfit(x[m], y[m], 1)[0] if m.sum()>20 else np.nan

print("1) 같은날(07-21) 상승속도별 rise slope [N/A] — rate 효과 분리")
for lab, pat in [('0.2 A/s', r'c_ramp_slow_'), ('0.375', r'c_ramp_med_'), ('0.7', r'c_ramp_fast_')]:
    sl = []
    for i in runs(pat):
        z = load(i); r, f = branches(z)
        sl.append(slope(z['fb'][r], z['Ft'][r]))
    print(f"  {lab:8s}: {np.round(sl,2)}  mean={np.nanmean(sl):.2f}")
print("  (07-22 forc_rev14 rise@1.0A/s = 18.62) → 같은날 rate 간 차이가 작으면 18.6vs20.1 은 날짜/재적재 효과")

print("\n2) 모델 이전성 — 07-22 학습 Preisach 를 07-21 런에 적용")
M = np.load(os.path.join(CACHE, 'preisach_model.npz'))
A, B, W, SUB = M['A'], M['B'], M['W'], int(M['sub'])
def predict(u):
    T = len(u); s = np.zeros(len(A)); out = np.empty(T)
    for k in range(T):
        s = np.where(u[k] >= A, 1.0, np.where(u[k] <= B, 0.0, s))
        out[k] = s @ W[:-2] + W[-2] - W[-1]
    return out
for pat in (r'c_prbs_', r'c_deadband_stair_', r'c_ramp_med_', r'c_sine_lo_'):
    for i in runs(pat):
        z = load(i); u = z['fb'][::SUB]; F = z['Ft'][::SUB]
        e = predict(u) - F
        print(f"  {i['name'][:28]:28s} RMSE = {np.sqrt((e**2).mean()):5.2f} N   bias(mean err) = {e.mean():+5.2f} N")
        break   # 각 패밀리 첫 런만
