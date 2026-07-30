// 모니터 패널 — 필드 트리 + SSE 구독 + 플롯 (redesign/07 §4)
//
// plotjuggler 처럼 **왼쪽 트리에서 고르면 오른쪽에 그려진다.** 고른 순간 과거부터 보이는데,
// 서버가 구독 시 링버퍼(30s)를 먼저 보내 주기 때문이다 (07 §4 ④) — 체크한 순간부터
// 그리기 시작하면 "방금 무슨 일이 있었나" 를 볼 수 없다.

import { TimeChart, fmt } from './chart.js';

const $ = (id) => document.getElementById(id);

export class Monitor {
  constructor() {
    this.fields = [];
    this.selected = [];
    this.es = null;                 // EventSource
    this.chart = new TimeChart($('chart'), {windowSec: 10});
    this.pending = null;            // 구독 재시작 디바운스
  }

  async init() {
    const r = await fetch('/api/fields');
    const j = await r.json();
    this.fields = j.fields;
    $('mstat').textContent = `${j.fields.length}계열 · ${j.stream_hz}Hz`;
    this.renderTree();

    $('mfilter').addEventListener('input', () => this.renderTree());
    $('mwindow').addEventListener('change', (e) => {
      this.chart.windowSec = parseInt(e.target.value, 10);
    });
    $('mclear').addEventListener('click', () => {
      this.selected = [];
      this.renderTree();
      this.resubscribe();
    });

    const tick = () => { this.chart.draw(); this.renderLegend(); requestAnimationFrame(tick); };
    requestAnimationFrame(tick);
  }

  renderTree() {
    const q = ($('mfilter').value || '').toLowerCase();
    const groups = new Map();
    for (const f of this.fields) {
      if (q && !f.key.toLowerCase().includes(q) && !f.group.toLowerCase().includes(q)) continue;
      if (!groups.has(f.group)) groups.set(f.group, []);
      groups.get(f.group).push(f);
    }
    const el = $('mtree');
    el.innerHTML = '';
    for (const g of [...groups.keys()].sort()) {
      const box = document.createElement('div');
      box.className = 'mgroup';
      const h = document.createElement('div');
      h.className = 'mghead';
      h.textContent = g;
      box.appendChild(h);
      for (const f of groups.get(g)) {
        const lab = document.createElement('label');
        lab.className = 'mitem';
        const cb = document.createElement('input');
        cb.type = 'checkbox';
        cb.checked = this.selected.includes(f.key);
        cb.onchange = () => this.toggle(f.key, cb.checked);
        const sw = document.createElement('span');
        sw.className = 'swatch';
        sw.style.background = cb.checked ? this.chart.colorOf(f.key) : 'transparent';
        lab.append(cb, sw, document.createTextNode(f.key));
        box.appendChild(lab);
      }
      el.appendChild(box);
    }
  }

  toggle(key, on) {
    const i = this.selected.indexOf(key);
    if (on && i < 0) this.selected.push(key);
    if (!on && i >= 0) this.selected.splice(i, 1);
    this.renderTree();
    this.resubscribe();
  }

  /** 선택이 바뀌면 스트림을 다시 연다. 체크를 여러 개 빠르게 켜는 경우가 흔해서 묶는다. */
  resubscribe() {
    clearTimeout(this.pending);
    this.pending = setTimeout(() => this._open(), 120);
  }

  _open() {
    if (this.es) { this.es.close(); this.es = null; }
    this.chart.setSeries(this.selected);
    if (!this.selected.length) { this.chart.load({t: [], d: {}}); return; }

    // 대괄호는 URL 에서 그대로 못 쓴다 (`fb_current[0]`).
    const qs = this.selected.map(encodeURIComponent).join(',');
    const es = new EventSource('/api/stream?fields=' + qs);
    es.addEventListener('history', (e) => this.chart.load(JSON.parse(e.data)));
    es.addEventListener('f', (e) => this.chart.push(JSON.parse(e.data)));
    es.onerror = () => { $('mstat').textContent = '스트림 끊김 — 재접속 중…'; };
    es.onopen = () => { $('mstat').textContent = `${this.selected.length}계열 구독 중`; };
    this.es = es;
  }

  renderLegend() {
    const el = $('mlegend');
    const vals = this.chart.latest();
    const rows = this.selected.map((k) => {
      const v = vals[k];
      // **미판독을 0 으로 적지 않는다** — 값이 없다는 것 자체가 정보다 (07 §4.3).
      const cls = v === null ? 'unread' : '';
      return `<span class="lg ${cls}"><i style="background:${this.chart.colorOf(k)}"></i>` +
             `${k} <b>${fmt(v)}</b></span>`;
    });
    el.innerHTML = rows.join('');
  }
}
