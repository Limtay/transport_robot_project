// Tab2 계획형 — 세그먼트 편집 + 미리보기 + 재생 (redesign/07 §2)
//
// **그래프는 미리보기지 진실이 아니다.** 실제로 나가는 값은 브리지가 YAML 에서 펼친 것이고,
// 여기 곡선은 같은 식을 브라우저에서 다시 계산한 근사다. 그래서:
//   ① 보내기 전에 **YAML 원문을 보여준다** — 무엇이 나가는지 눈으로 확인할 수 있어야 한다.
//   ② 검증은 흉내내지 않는다. 브리지가 거부하면 그 사유를 그대로 띄운다
//      (limits 필수 조건·slew·주파수 상한은 전부 브리지가 정본이다 — 05 §2.4).

const $ = (id) => document.getElementById(id);

// 세그 타입별 인자. 서버의 SEG_ARGS 와 같아야 한다 (거기가 조립을 하고 여기는 입력을 만든다).
const SEG = {
  hold:   ['duration', 'value'],
  ramp:   ['duration', 'from', 'to'],
  step:   ['duration', 'from', 'to', 't_step'],
  sine:   ['duration', 'amp', 'freq'],
  chirp:  ['duration', 'amp', 'f0', 'f1'],
  prbs:   ['duration', 'low', 'high', 'bit_duration'],
  noise:  ['duration', 'mean', 'std'],
  stair:  ['step_duration', 'values'],
  custom: ['samples'],
};
const DEFAULTS = {
  hold:  {duration: 1, value: 0},
  ramp:  {duration: 5, from: 0, to: 5},
  step:  {duration: 4, from: 0, to: 5, t_step: 2},
  sine:  {duration: 5, amp: 3, freq: 1},
  chirp: {duration: 10, amp: 3, f0: 0.2, f1: 5},
  prbs:  {duration: 5, low: -2, high: 2, bit_duration: 0.2},
  noise: {duration: 5, mean: 0, std: 0.5},
  stair: {step_duration: 1, values: '0,2,4,2,0'},
  custom: {samples: '0,1,2,1,0', rate: 50},
};
const UNIT = {current: 'A', velocity: 'RPM', position: 'deg'};

function post(path, body) {
  return fetch(path, {method: 'POST', headers: {'Content-Type': 'application/json'},
                      body: JSON.stringify(body || {})}).then(r => r.json());
}
const num = (v) => (typeof v === 'string' && v.includes(',')
  ? v.split(',').map(x => Number(x.trim())).filter(x => !Number.isNaN(x))
  : Number(v));

/** 미리보기용 샘플링. 200Hz 로 펼치면 브라우저가 무거워서 200Hz/4 로 그린다. */
const PREVIEW_HZ = 50;
function expand(seg) {
  const out = [];
  const n = (k) => Number(seg[k]);
  const push = (dur, f) => {
    const c = Math.max(1, Math.round(dur * PREVIEW_HZ));
    for (let i = 0; i < c; i++) out.push(f(i / PREVIEW_HZ));
  };
  switch (seg.type) {
    case 'hold': push(n('duration'), () => n('value')); break;
    case 'ramp': {
      const d = n('duration');
      push(d, (t) => n('from') + (n('to') - n('from')) * (d > 0 ? t / d : 1));
      break;
    }
    case 'step': push(n('duration'), (t) => (t < n('t_step') ? n('from') : n('to'))); break;
    case 'sine': push(n('duration'), (t) =>
      (Number(seg.offset) || 0) + n('amp') * Math.sin(2 * Math.PI * n('freq') * t)); break;
    case 'chirp': {
      const d = n('duration'), f0 = n('f0'), f1 = n('f1');
      push(d, (t) => (Number(seg.offset) || 0) +
        n('amp') * Math.sin(2 * Math.PI * (f0 * t + (f1 - f0) * t * t / (2 * d))));
      break;
    }
    case 'prbs': {
      // 값은 브리지의 seed 로 정해진다 — 여기 난수는 **모양만** 보여준다.
      let cur = n('low'), next = 0;
      push(n('duration'), (t) => {
        if (t >= next) { next = t + n('bit_duration'); cur = Math.random() < 0.5 ? n('low') : n('high'); }
        return cur;
      });
      break;
    }
    case 'noise': push(n('duration'), () =>
      n('mean') + n('std') * (Math.random() + Math.random() + Math.random() - 1.5) * 1.1547); break;
    case 'stair': {
      const vals = num(seg.values);
      for (const v of (Array.isArray(vals) ? vals : [vals])) push(n('step_duration'), () => v);
      break;
    }
    case 'custom': {
      const s = num(seg.samples);
      const arr = Array.isArray(s) ? s : [s];
      const rate = Number(seg.rate) || 50;
      // rate 로 준 샘플을 PREVIEW_HZ 로 다시 찍는다 (브리지는 선형보간한다).
      const dur = arr.length / rate;
      push(dur, (t) => {
        const x = t * rate, i = Math.floor(x);
        if (i >= arr.length - 1) return arr[arr.length - 1];
        return arr[i] + (arr[i + 1] - arr[i]) * (x - i);
      });
      break;
    }
  }
  return out;
}

export class ProfileTab {
  constructor() {
    this.motors = {m1: []};
    this.poll = null;
  }

  init() {
    $('pf-add').onclick = () => this.addSeg();
    $('pf-motor').onchange = () => this.render();
    $('pf-mode').onchange = () => this.render();
    $('pf-preview').onclick = () => this.showYaml();
    $('pf-run').onclick = () => this.run();
    $('pf-abort').onclick = () => post('/api/profile/abort').then(r =>
      $('pf-notice').textContent = r.message);
    for (const t of Object.keys(SEG)) {
      const o = document.createElement('option'); o.textContent = t; $('pf-type').appendChild(o);
    }
    this.render();
    setInterval(() => this.pollStatus(), 500);
  }

  cur() {
    const k = $('pf-motor').value;
    if (!this.motors[k]) this.motors[k] = [];
    return this.motors[k];
  }

  addSeg() {
    const t = $('pf-type').value;
    this.cur().push(Object.assign({type: t}, DEFAULTS[t]));
    this.render();
  }

  render() {
    const list = $('pf-segs');
    list.innerHTML = '';
    const segs = this.cur();
    if (!segs.length) {
      list.innerHTML = '<p class="steps">세그먼트가 없다 — 위에서 타입을 골라 추가할 것.</p>';
    }
    segs.forEach((s, i) => {
      const d = document.createElement('div');
      d.className = 'pf-seg';
      const fields = SEG[s.type].concat(
        s.type === 'custom' ? ['rate'] : (['sine', 'chirp'].includes(s.type) ? ['offset'] : []));
      d.innerHTML = `<b>${i}</b> <span class="pf-t">${s.type}</span> ` +
        fields.map(a =>
          `<label>${a} <input data-i="${i}" data-a="${a}" value="${s[a] ?? ''}" size="${
            ['values', 'samples'].includes(a) ? 22 : 6}"></label>`).join(' ') +
        ` <button data-del="${i}">삭제</button>` +
        ` <button data-up="${i}" ${i === 0 ? 'disabled' : ''}>↑</button>`;
      list.appendChild(d);
    });
    list.querySelectorAll('input').forEach(inp => {
      inp.onchange = () => {
        const s = segs[+inp.dataset.i];
        s[inp.dataset.a] = inp.value;
        this.drawPreview();
      };
    });
    list.querySelectorAll('[data-del]').forEach(b => {
      b.onclick = () => { segs.splice(+b.dataset.del, 1); this.render(); };
    });
    list.querySelectorAll('[data-up]').forEach(b => {
      b.onclick = () => {
        const i = +b.dataset.up;
        [segs[i - 1], segs[i]] = [segs[i], segs[i - 1]];
        this.render();
      };
    });
    this.drawPreview();
  }

  spec() {
    const mode = $('pf-mode').value;
    const limits = {};
    if ($('pf-maxabs').value !== '') limits.max_abs = Number($('pf-maxabs').value);
    if ($('pf-lo').value !== '' && $('pf-hi').value !== '')
      limits.range = [Number($('pf-lo').value), Number($('pf-hi').value)];
    if ($('pf-slew').value !== '') limits.slew_rate = Number($('pf-slew').value);
    const motors = {};
    for (const [k, segs] of Object.entries(this.motors)) {
      if (!segs.length) continue;
      motors[k] = segs.map(s => {
        const o = {type: s.type};
        for (const a of Object.keys(s)) {
          if (a === 'type') continue;
          o[a] = ['values', 'samples'].includes(a) ? num(s[a]) : Number(s[a]);
        }
        return o;
      });
    }
    const sp = {name: $('pf-name').value || 'web', mode, limits, motors};
    if ($('pf-seed').value !== '') sp.seed = Number($('pf-seed').value);
    return sp;
  }

  async showYaml() {
    const r = await post('/api/profile/preview', {spec: this.spec()});
    $('pf-yaml').textContent = r.ok ? r.yaml : '조립 실패: ' + r.message;
  }

  async run() {
    // 되돌릴 수 없다 — 모터가 실제로 돈다. 무엇이 나가는지 먼저 보여주고 확인을 받는다.
    const r = await post('/api/profile/preview', {spec: this.spec()});
    if (!r.ok) { $('pf-notice').textContent = '조립 실패: ' + r.message; return; }
    $('pf-yaml').textContent = r.yaml;
    if (!window.confirm('이 프로파일로 모터를 구동한다. 계속?\n\n' + r.yaml)) return;
    const res = await post('/api/profile/run',
                           {spec: this.spec(), record: $('pf-record').checked});
    $('pf-notice').textContent = res.message;
  }

  async pollStatus() {
    const s = await (await fetch('/api/profile/status')).json();
    const running = s.state === 'running';
    $('pf-run').disabled = running;
    $('pf-abort').disabled = !running;
    $('pf-bar').style.width = (s.progress * 100).toFixed(1) + '%';
    $('pf-prog').textContent = running
      ? `${(s.progress * 100).toFixed(1)}%  t=${s.t.toFixed(1)}s  seg=${s.segment}`
      : (s.message || '대기');
    const r = s.result;
    if (!r) { $('pf-result').textContent = ''; return; }
    const warn = [];
    // 성공했는데 데이터가 온전하지 않은 두 경우 — 조용히 지나가면 안 된다.
    if (r.drop) warn.push(`⚠ 발행 큐 드롭 ${r.drop}건 — 시계열에 구멍이 있다`);
    if (!r.clock_converged) warn.push('⚠ 클럭 미수렴 — stamp 가 Orin 수신 시각 fallback이다. '
                                      + '수렴한 런과 시간축을 섞지 말 것');
    $('pf-result').innerHTML =
      `<b>${r.success ? '완료' : '실패'}</b> goal=${r.goal_id} · ${r.ticks} tick · ` +
      `write_err=${r.write_err} · clamp=${r.clamp}` +
      (r.run_dir ? `<br>기록: <code>${r.run_dir}</code>` : '') +
      (warn.length ? '<br><span class="bad">' + warn.join('<br>') + '</span>' : '');
  }

  drawPreview() {
    const cv = $('pf-chart'), ctx = cv.getContext('2d');
    const dpr = window.devicePixelRatio || 1;
    const w = cv.clientWidth, h = cv.clientHeight;
    if (cv.width !== w * dpr || cv.height !== h * dpr) { cv.width = w * dpr; cv.height = h * dpr; }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);

    let vals = [], bounds = [];
    try {
      for (const s of this.cur()) {
        const v = expand(s);
        bounds.push(vals.length + v.length);
        vals = vals.concat(v);
      }
    } catch (e) { vals = []; }

    const PAD = {l: 54, r: 10, t: 8, b: 20};
    const pw = w - PAD.l - PAD.r, ph = h - PAD.t - PAD.b;
    if (!vals.length) {
      ctx.fillStyle = '#888'; ctx.font = '13px sans-serif';
      ctx.fillText('세그먼트를 추가하면 여기에 그려진다', PAD.l, PAD.t + ph / 2);
      return;
    }
    const good = vals.filter(v => Number.isFinite(v));
    let lo = Math.min(...good), hi = Math.max(...good);
    if (lo === hi) { lo -= 1; hi += 1; }
    const pad = (hi - lo) * 0.1; lo -= pad; hi += pad;
    const X = (i) => PAD.l + i / (vals.length - 1 || 1) * pw;
    const Y = (v) => PAD.t + (1 - (v - lo) / (hi - lo)) * ph;

    ctx.strokeStyle = '#2a2f3a'; ctx.fillStyle = '#8b93a1';
    ctx.font = '11px monospace';
    for (let i = 0; i <= 4; i++) {
      const v = lo + (hi - lo) * i / 4, y = Math.round(Y(v)) + 0.5;
      ctx.beginPath(); ctx.moveTo(PAD.l, y); ctx.lineTo(PAD.l + pw, y); ctx.stroke();
      ctx.fillText(v.toFixed(2), 4, y + 3);
    }
    // 세그 경계 — 어디서 무엇이 바뀌는지 보이게
    ctx.strokeStyle = '#3a4150';
    ctx.setLineDash([3, 3]);
    for (const b of bounds.slice(0, -1)) {
      const x = Math.round(X(b)) + 0.5;
      ctx.beginPath(); ctx.moveTo(x, PAD.t); ctx.lineTo(x, PAD.t + ph); ctx.stroke();
    }
    ctx.setLineDash([]);

    ctx.strokeStyle = '#4ea1ff'; ctx.lineWidth = 1.5;
    ctx.beginPath();
    vals.forEach((v, i) => (i ? ctx.lineTo(X(i), Y(v)) : ctx.moveTo(X(i), Y(v))));
    ctx.stroke();

    const secs = (vals.length / PREVIEW_HZ).toFixed(1);
    ctx.fillStyle = '#8b93a1';
    ctx.fillText(`${secs}s · ${UNIT[$('pf-mode').value] || ''}`, PAD.l + pw - 70, h - 6);
  }
}
