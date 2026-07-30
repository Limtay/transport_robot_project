#!/usr/bin/env python3
"""프로파일 YAML 린터 — rd_profile.cpp 파서 규칙 미러링.

goal-reject(런 낭비)를 실행 전에 잡는다. 필수 키·양수 제약·active_mask·전류 클램프·음수전류를
검사하고, 각 세그먼트를 200Hz 로 펼쳐 실제 파형의 min/max/clamp 수를 예측한다.
사용: python3 lint_profiles.py <active_motor_no> <max_current> profile.yaml [profile.yaml ...]
"""
import sys, math, yaml, numpy as np

REQ = {
    'hold':  ['duration', 'value'],
    'ramp':  ['duration', 'from', 'to'],
    'step':  ['duration', 'from', 'to', 't_step'],
    'sine':  ['duration', 'amp', 'freq'],
    'chirp': ['duration', 'amp', 'f0', 'f1'],
    'prbs':  ['duration', 'low', 'high', 'bit_duration'],
    'noise': ['duration', 'mean', 'std'],
    'stair': ['step_duration', 'values'],
    'custom':['samples'],
}
HZ = 200.0; DT = 1.0 / HZ

def expand(seg, seed=42):
    t = seg['type']
    if t == 'stair':
        per = int(round(seg['step_duration'] * HZ)); out = []
        for v in seg['values']: out += [float(v)] * per
        return np.array(out)
    if t == 'custom':
        rate = seg.get('rate', HZ); src = [float(x) for x in seg['samples']]
        n = int(round(len(src) / rate * HZ))
        return np.array([src[min(int(i*DT*rate), len(src)-1)] for i in range(n)])
    n = int(round(seg['duration'] * HZ)); i = np.arange(n)
    if t == 'hold':  return np.full(n, float(seg['value']))
    if t == 'ramp':
        a = i/(n-1) if n > 1 else np.array([1.0]); return seg['from'] + (seg['to']-seg['from'])*a
    if t == 'step':
        k = int(round(seg['t_step']*HZ)); return np.where(i < k, seg['from'], seg['to']).astype(float)
    if t == 'sine':
        return seg.get('offset',0.0) + seg['amp']*np.sin(2*np.pi*seg['freq']*i*DT)
    if t == 'chirp':
        T = seg['duration']; tt = i*DT
        ph = 2*np.pi*(seg['f0']*tt + (seg['f1']-seg['f0'])*tt*tt/(2*T))
        return seg.get('offset',0.0) + seg['amp']*np.sin(ph)
    if t == 'prbs':
        per = max(1, int(round(seg['bit_duration']*HZ)))
        rng = np.random.default_rng(seed); out = []; cur = seg['high'] if rng.random()<0.5 else seg['low']
        for j in range(n):
            if j % per == 0: cur = seg['high'] if rng.random()<0.5 else seg['low']
            out.append(cur)
        return np.array(out, float)
    if t == 'noise':
        rng = np.random.default_rng(seed); return rng.normal(seg['mean'], seg['std'], n)
    raise ValueError(f"unknown type {t}")

def type_constraints(key, si, seg, t):
    """05 §3.3 확정표 — 각 제약은 **조용한 오해**를 막는다.

    제약이 없으면 사용자가 그렸다고 믿는 것과 실제 재생되는 것이 달라진다.
    이 함수는 `rd_profile.cpp` 의 검사와 **한 글자도 다르면 안 된다** — 다르면
    lint 를 통과한 프로파일이 실기에서 거부되고, 원인을 찾는 데 시간이 든다.
    """
    e = []
    if t == 'step':
        d, ts = seg.get('duration'), seg.get('t_step')
        if d and ts is not None and not (0 < ts < d):
            e.append(f"{key}[{si}] step: t_step({ts}) 은 0 < t_step < duration({d})")
    elif t == 'sine':
        f = seg.get('freq')
        if f is not None and not (0 < f <= 25):
            # 25Hz = 200Hz 체인에서 주기당 8샘플. 넘으면 앨리어싱으로 다른 파형이 나온다.
            e.append(f"{key}[{si}] sine: freq({f}) 은 0 < freq <= 25 Hz")
        elif f is not None and f > 5:
            e.append(f"WARN {key}[{si}] sine: freq({f}) > 5Hz — 기구 대역을 넘을 수 있다")
    elif t == 'chirp':
        for k in ('f0', 'f1'):
            v = seg.get(k)
            if v is not None and not (0 <= v <= 25):
                e.append(f"{key}[{si}] chirp: {k}({v}) 은 0~25 Hz")
        if seg.get('f0') == seg.get('f1'):
            e.append(f"WARN {key}[{si}] chirp: f0==f1 — sine 을 쓸 것")
    elif t == 'prbs':
        bd = seg.get('bit_duration')
        if bd is not None and bd < DT:
            e.append(f"{key}[{si}] prbs: bit_duration({bd}) 은 1 tick({DT}s) 이상")
        if seg.get('low') == seg.get('high'):
            e.append(f"{key}[{si}] prbs: low==high — hold 를 쓸 것")
    elif t == 'noise':
        sd = seg.get('std')
        if sd is not None and not sd > 0:
            e.append(f"{key}[{si}] noise: std 는 양수 (0 이면 hold)")
    elif t == 'stair':
        n = len(seg.get('values') or [])
        if not (1 <= n <= 1000):
            e.append(f"{key}[{si}] stair: values 길이({n}) 는 1~1,000")
    elif t == 'custom':
        r = seg.get('rate', HZ)
        if not (1 <= r <= HZ):
            e.append(f"{key}[{si}] custom: rate({r}) 은 1~{HZ} — "
                     f"초과분은 내보내기 단계에서 다운샘플할 것")
        n = len(seg.get('samples') or [])
        if not (1 <= n <= 100000):
            e.append(f"{key}[{si}] custom: samples 길이({n}) 는 1~100,000")
        if seg.get('interp', 'linear') not in ('linear', 'nearest'):
            e.append(f"{key}[{si}] custom: interp 는 linear|nearest")
    return e


def lint(path, active_no, gmax):
    errs = []
    doc = yaml.safe_load(open(path))
    # 05 §2.2~§2.4 — mode 와 모드 의존 limits
    mode = doc.get('mode', 'current')
    if mode not in ('current', 'velocity', 'position'):
        errs.append(f"mode: '{mode}' 는 current|velocity|position")
        mode = 'current'
    lim = doc.get('limits') or {}
    if mode == 'velocity' and 'max_abs' not in lim and 'range' not in lim:
        errs.append("mode: velocity 는 limits.max_abs 필수 (전역 기본값이 없다)")
    if mode == 'position' and 'range' not in lim:
        errs.append("mode: position 은 limits.range 필수 (관절 가동범위는 비대칭)")
    if 'max_current' in lim and mode != 'current':
        errs.append("limits.max_current 는 mode: current 전용 (deprecated) — max_abs/range 를 쓸 것")

    limit = gmax
    if 'range' in lim:
        limit = max(abs(float(lim['range'][0])), abs(float(lim['range'][1])))
    elif 'max_abs' in lim:
        limit = abs(float(lim['max_abs']))
    elif lim.get('max_current') is not None:
        limit = min(limit, float(lim['max_current']))
    seed = int(doc.get('seed', 42))
    motors = doc.get('motors', {})
    if not motors: errs.append("motors 맵 없음")
    allmin, allmax, clamp = 1e9, -1e9, 0; total_ticks = 0; max_jump = 0.0; prev_last = None
    for key, segs in motors.items():
        mno = int(key[1:])
        if mno != active_no:
            errs.append(f"{key}: active_motors({active_no})에 없음 → reject")
        for si, seg in enumerate(segs):
            t = seg.get('type')
            if t not in REQ: errs.append(f"{key}[{si}]: unknown type {t}"); continue
            for k in REQ[t]:
                if k not in seg: errs.append(f"{key}[{si}] {t}: 필수 키 '{k}' 누락")
            for k in ('duration','step_duration','bit_duration'):
                if k in seg and seg[k] <= 0: errs.append(f"{key}[{si}] {t}: {k}>0 이어야")
            # ── 05 §3.3 타입별 제약 (확정표). **rd_profile.cpp 와 같은 표를 본다** ──
            #    두 구현이 갈라지면 CLI 가 통과시킨 프로파일을 브리지가 거부한다.
            errs += type_constraints(key, si, seg, t)
            try:
                w = expand(seg, seed)
                total_ticks += len(w)
                allmin = min(allmin, w.min()); allmax = max(allmax, w.max())
                clamp += int((np.abs(w) > limit + 1e-6).sum())
                # 세그먼트 경계 급단차(모터 저크) 검사 — step 타입은 의도적이라 제외
                if prev_last is not None and t not in ('step',):
                    max_jump = max(max_jump, abs(float(w[0]) - prev_last))
                prev_last = float(w[-1])
            except Exception as e:
                errs.append(f"{key}[{si}] {t}: expand 실패 {e}")
    dur = total_ticks * DT
    return errs, dict(limit=limit, min=allmin, max=allmax, clamp=clamp, dur=dur,
                      neg=allmin < -1e-6, jump=max_jump)

if __name__ == '__main__':
    active_no = int(sys.argv[1]); gmax = float(sys.argv[2]); paths = sys.argv[3:]
    bad = 0
    for p in paths:
        errs, info = lint(p, active_no, gmax)
        big_jump = info['jump'] > 0.5
        tag = 'FAIL' if errs else ('WARN' if (info['clamp'] or info['neg'] or big_jump) else 'OK')
        print(f"[{tag}] {p.split('/')[-1]:26s} dur={info['dur']:6.1f}s  A[{info['min']:+.2f},{info['max']:+.2f}]  "
              f"limit={info['limit']}  clamp={info['clamp']}  neg={info['neg']}  seg_jump={info['jump']:.2f}A")
        for e in errs: print(f"       ! {e}");
        if errs: bad += 1
    print(f"\n{'ALL PASS' if not bad else str(bad)+' FILE(S) FAILED'}")
    sys.exit(1 if bad else 0)
