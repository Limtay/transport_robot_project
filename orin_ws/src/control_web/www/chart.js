// 시계열 캔버스 렌더러 (redesign/07 §4.4)
//
// uPlot 을 쓰려면 내려받아야 하는데 로봇은 오프라인이고, 우리 데이터는 형태가 특수하다:
//
//   ① 값이 `null` 일 수 있다 — **미판독**이다. 선을 이으면 안 되고 끊어야 한다.
//   ② 값이 `[min, max]` 일 수 있다 — 50Hz 로 내리면서 창 안의 극값을 보존한 것이다.
//      가운데 점 하나로 그리면 다운샘플에서 스파이크를 지키려던 이유가 사라진다.
//
// 그래서 그 두 가지를 1급으로 다루는 렌더러를 직접 둔다. 의존이 없으므로 vendoring
// 문제도 없다. 나중에 uPlot 으로 갈아끼우려면 이 파일의 인터페이스(setSeries/push/draw)만
// 맞추면 된다.

const PALETTE = ['#4ea1ff', '#ff8a4e', '#5ad18a', '#e05d8a', '#c48cff',
                 '#f0c419', '#4ecdc4', '#ff6b6b', '#9aa5b1', '#7ed957'];

export class TimeChart {
  constructor(canvas, opts = {}) {
    this.cv = canvas;
    this.ctx = canvas.getContext('2d');
    this.windowSec = opts.windowSec || 10;
    this.keys = [];                 // 그릴 계열 (순서 = 색 순서)
    this.t = [];                    // 공통 시간축
    this.d = new Map();             // key -> 값 배열 (number | [lo,hi] | null)
    this.maxPoints = opts.maxPoints || 4000;
    this.hover = null;
    this._bindHover();
  }

  setSeries(keys) {
    this.keys = keys.slice();
    for (const k of keys) if (!this.d.has(k)) this.d.set(k, new Array(this.t.length).fill(null));
    for (const k of [...this.d.keys()]) if (!keys.includes(k)) this.d.delete(k);
  }

  // 서버 history 를 통째로 얹는다 (구독 직후 1회).
  load(hist) {
    this.t = (hist.t || []).slice();
    this.d = new Map();
    for (const k of this.keys) {
      const col = (hist.d && hist.d[k]) ? hist.d[k].slice() : [];
      // 길이가 다르면 앞을 null 로 채운다 — 없는 과거를 지어내지 않는다.
      while (col.length < this.t.length) col.unshift(null);
      this.d.set(k, col);
    }
    this._trim();
  }

  push(frame) {
    this.t.push(frame.t);
    for (const k of this.keys) {
      if (!this.d.has(k)) this.d.set(k, new Array(this.t.length - 1).fill(null));
      const v = frame.d[k];
      this.d.get(k).push(v === undefined ? null : v);
    }
    this._trim();
  }

  _trim() {
    const over = this.t.length - this.maxPoints;
    if (over > 0) {
      this.t.splice(0, over);
      for (const col of this.d.values()) col.splice(0, over);
    }
  }

  // ── 그리기 ────────────────────────────────────────────────────────────
  draw() {
    const cv = this.cv, ctx = this.ctx;
    const dpr = window.devicePixelRatio || 1;
    const w = cv.clientWidth, h = cv.clientHeight;
    if (cv.width !== w * dpr || cv.height !== h * dpr) {
      cv.width = w * dpr; cv.height = h * dpr;
    }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);

    const PAD = {l: 58, r: 10, t: 8, b: 20};
    const pw = w - PAD.l - PAD.r, ph = h - PAD.t - PAD.b;
    if (pw <= 0 || ph <= 0) return;

    if (!this.t.length || !this.keys.length) {
      ctx.fillStyle = '#888'; ctx.font = '13px sans-serif';
      ctx.fillText(this.keys.length ? '데이터 대기 중…' : '왼쪽에서 필드를 고르면 그려진다',
                   PAD.l, PAD.t + ph / 2);
      return;
    }

    const tMax = this.t[this.t.length - 1];
    const tMin = tMax - this.windowSec;

    // y 범위 — **보이는 구간만** 본다. 전체로 잡으면 지나간 스파이크가 축을 눌러 놓는다.
    let lo = Infinity, hi = -Infinity, seen = 0;
    for (const k of this.keys) {
      const col = this.d.get(k) || [];
      for (let i = 0; i < col.length; i++) {
        if (this.t[i] < tMin) continue;
        const v = col[i];
        if (v === null || v === undefined) continue;
        const a = Array.isArray(v) ? v[0] : v, b = Array.isArray(v) ? v[1] : v;
        if (a < lo) lo = a;
        if (b > hi) hi = b;
        seen++;
      }
    }
    if (!seen) { lo = 0; hi = 1; }
    if (lo === hi) { lo -= 1; hi += 1; }
    const pad = (hi - lo) * 0.08; lo -= pad; hi += pad;

    const X = (t) => PAD.l + (t - tMin) / (tMax - tMin || 1) * pw;
    const Y = (v) => PAD.t + (1 - (v - lo) / (hi - lo)) * ph;

    // 격자 + y 눈금
    ctx.strokeStyle = '#2a2f3a'; ctx.fillStyle = '#8b93a1';
    ctx.font = '11px monospace'; ctx.lineWidth = 1;
    for (let i = 0; i <= 4; i++) {
      const v = lo + (hi - lo) * i / 4, y = Math.round(Y(v)) + 0.5;
      ctx.beginPath(); ctx.moveTo(PAD.l, y); ctx.lineTo(PAD.l + pw, y); ctx.stroke();
      ctx.fillText(fmt(v), 4, y + 3);
    }
    ctx.fillText('-' + this.windowSec + 's', PAD.l, h - 6);
    ctx.fillText('now', PAD.l + pw - 22, h - 6);

    // 계열
    this.keys.forEach((k, ci) => {
      const col = this.d.get(k) || [];
      const color = PALETTE[ci % PALETTE.length];

      // ① [min,max] 밴드를 먼저 — 선 아래에 깔린다.
      ctx.fillStyle = color + '55';
      for (let i = 0; i < col.length; i++) {
        const v = col[i];
        if (!Array.isArray(v) || this.t[i] < tMin) continue;
        const x = X(this.t[i]), y0 = Y(v[1]), y1 = Y(v[0]);
        ctx.fillRect(x - 1, y0, 2, Math.max(1, y1 - y0));
      }

      // ② 선 — **null 에서 끊는다.** 이어 그리면 미판독 구간이 보간처럼 보인다.
      ctx.strokeStyle = color; ctx.lineWidth = 1.5;
      ctx.beginPath();
      let pen = false;
      for (let i = 0; i < col.length; i++) {
        if (this.t[i] < tMin) continue;
        const v = col[i];
        if (v === null || v === undefined) { pen = false; continue; }
        const mid = Array.isArray(v) ? (v[0] + v[1]) / 2 : v;
        const x = X(this.t[i]), y = Y(mid);
        if (!pen) { ctx.moveTo(x, y); pen = true; } else { ctx.lineTo(x, y); }
      }
      ctx.stroke();
    });

    if (this.hover !== null) {
      const x = Math.max(PAD.l, Math.min(PAD.l + pw, this.hover));
      ctx.strokeStyle = '#666'; ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(x + 0.5, PAD.t); ctx.lineTo(x + 0.5, PAD.t + ph); ctx.stroke();
    }
  }

  /** 범례용 — 각 계열의 최신 값. 미판독이면 null 을 그대로 돌려준다. */
  latest() {
    const out = {};
    for (const k of this.keys) {
      const col = this.d.get(k) || [];
      let v = null;
      for (let i = col.length - 1; i >= 0; i--) {
        if (col[i] !== null && col[i] !== undefined) { v = col[i]; break; }
      }
      // 마지막 표본 자체가 미판독이면 "미판독" 이 최신 상태다 — 옛 값을 보여주면 안 된다.
      const last = col.length ? col[col.length - 1] : null;
      out[k] = (last === null || last === undefined) ? null
             : (Array.isArray(v) ? (v[0] + v[1]) / 2 : v);
    }
    return out;
  }

  colorOf(k) { return PALETTE[this.keys.indexOf(k) % PALETTE.length]; }

  _bindHover() {
    this.cv.addEventListener('mousemove', (e) => {
      this.hover = e.offsetX;
    });
    this.cv.addEventListener('mouseleave', () => { this.hover = null; });
  }
}

export function fmt(v) {
  if (v === null || v === undefined) return '미판독';
  const a = Math.abs(v);
  if (a === 0) return '0';
  if (a >= 1e6 || a < 1e-3) return v.toExponential(2);
  return v.toFixed(a >= 100 ? 1 : a >= 1 ? 3 : 4);
}
