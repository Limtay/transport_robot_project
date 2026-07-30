// Tab3 — 레지스터 맵 / 커맨드 슬롯 (redesign/07 §2)
//
// CLI 의 `command` 로 하던 것을 시각화한다. 두 가지가 이 화면의 규칙이다:
//
//   ① **읽지 않는 자리는 회색이다.** 섀도는 언제나 값을 갖고 있어서, 신선도를 표시하지
//      않으면 한 번도 읽은 적 없는 자리의 0 이 방금 읽은 값처럼 보인다. 서버가 주는
//      `fresh` 구간 밖은 값 대신 "미판독" 을 쓴다 (07 §4.3 과 같은 규칙).
//   ② **쓰기 칸은 write 영역에만 생긴다.** 읽기 전용 블록에 입력칸을 만들면 조작자가
//      "썼는데 안 먹는다" 를 겪는다. rw 는 regmap.json 이 정하고 테스트가 헤더와 대조한다.

const $ = (id) => document.getElementById(id);

// CommandSet.srv 의 CMD_* — **브리지 카탈로그와 같은 번호여야 한다.**
const CMD = {
  read_sys: 0, read_motor: 1, read_sensor: 2, read_diag: 3, read_all: 4,
  set_soft_estop: 13, set_use_lpf: 14, reboot: 20, raw_read: 30, raw_write: 31,
};
const TARGET = {ecu: 225, dpc: 210, pcu: 161};

function post(path, body) {
  return fetch(path, {method: 'POST', headers: {'Content-Type': 'application/json'},
                      body: JSON.stringify(body || {})}).then(r => r.json());
}

// 리틀엔디언 — STM 과 Orin 이 둘 다 LE 다 (구조체를 그대로 memcpy 하는 것이 계약이다).
function decode(bytes, addr, type) {
  const dv = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  switch (type) {
    case 'u8':  return dv.getUint8(addr);
    case 'i8':  return dv.getInt8(addr);
    case 'u16': return dv.getUint16(addr, true);
    case 'i16': return dv.getInt16(addr, true);
    case 'u32': return dv.getUint32(addr, true);
    case 'i32': return dv.getInt32(addr, true);
    case 'f32': return dv.getFloat32(addr, true);
  }
  return null;
}
const SIZE = {u8: 1, i8: 1, u16: 2, i16: 2, u32: 4, i32: 4, f32: 4};

/** 신선 구간은 **겹친다** — 프리셋 구간과 슬롯 READ 구간이 같은 자리를 덮을 수 있다.
 *  그냥 더하면 "284 / 256B 신선" 같은 말이 안 되는 수가 나온다. 합집합으로 센다. */
function unionSize(spans) {
  const s = [...(spans || [])].map(([a, l]) => [a, a + l]).sort((x, y) => x[0] - y[0]);
  let n = 0, end = -1;
  for (const [a, b] of s) {
    if (a > end) { n += b - a; end = b; }
    else if (b > end) { n += b - end; end = b; }
  }
  return n;
}

function inFresh(fresh, addr, len) {
  // 04 §2.3 의 Covers() 와 같은 규칙 — **부분만 걸치면 신선하지 않다.**
  // 한 값의 일부만 읽혔으면 그 값 전체가 못 믿을 것이다.
  return (fresh || []).some(([a, l]) => addr >= a && addr + len <= a + l);
}

export class RegMap {
  constructor() {
    this.map = null;
    this.fresh = [];
    this.bytes = null;
    this.edits = new Map();       // addr -> 입력값 (전송 전)
    this.timer = null;
  }

  async init() {
    this.map = await (await fetch('/regmap.json')).json();
    this.renderTable();
    this.renderSlots();

    $('rg-refresh').onclick = () => this.issueRead('read_all');
    $('rg-auto').onchange = (e) => this.setAuto(e.target.checked);
    for (const name of ['read_sys', 'read_motor', 'read_sensor', 'read_diag', 'read_all']) {
      const b = document.createElement('button');
      b.textContent = name;
      b.onclick = () => this.issueRead(name);
      $('rg-reads').appendChild(b);
    }
    $('rg-send').onclick = () => this.sendEdits();
    $('rg-discard').onclick = () => { this.edits.clear(); this.renderTable(); this.renderEdits(); };
    $('rg-reboot').onclick = () => this.reboot();
    this.poll();
    setInterval(() => this.poll(), 1000);
  }

  setAuto(on) {
    clearInterval(this.timer);
    this.timer = null;
    // duration=0(forever)로 걸면 슬롯을 붙잡고 있게 된다. **once 를 주기적으로** 넣는다 —
    // 웹이 죽어도 슬롯이 남지 않는다.
    if (on) this.timer = setInterval(() => this.issueRead('read_all'), 2000);
  }

  async issueRead(name) {
    const r = await post('/api/command', {
      slot: 255, action: 1, target_id: TARGET.ecu, cmd: CMD[name], duration: 1});
    $('rg-notice').textContent = (r.ok ? '' : '거부: ') + r.message;
  }

  async reboot() {
    // 되돌릴 수 없는 조작이라 확인을 받는다. 브리지도 safe_stop 을 요구하지만,
    // 게이트가 있다고 해서 실수로 누르는 것을 막아 주지는 않는다.
    if (!window.confirm('ECU 를 리부트한다. 3초간 통신이 끊기고 진행 중인 것은 멈춘다. 계속?')) return;
    const r = await post('/api/command', {
      slot: 255, action: 1, target_id: TARGET.ecu, cmd: CMD.reboot, duration: 1});
    $('rg-notice').textContent = (r.ok ? '' : '거부: ') + r.message;
  }

  async poll() {
    const res = await fetch('/api/registers');
    if (!res.ok) {
      $('rg-stat').textContent = '브리지 없음';
      this.bytes = null;
      this.renderTable();
      return;
    }
    const d = await res.json();
    this.fresh = d.fresh || [];
    const hex = d.bytes || '';
    const b = new Uint8Array(hex.length / 2);
    for (let i = 0; i < b.length; i++) b[i] = parseInt(hex.substr(i * 2, 2), 16);
    this.bytes = b;
    const covered = unionSize(this.fresh);
    $('rg-stat').textContent =
      `${covered}/${d.total}B 신선` +
      (d.read_age_s === null ? ' (슬롯 READ 없음 — 프리셋 구간만)'
                             : ` · 마지막 READ ${d.read_age_s.toFixed(1)}s 전`);
    this.renderTable();
  }

  fmt(f, v) {
    if (v === null) return '—';
    if (f.stale !== undefined && v === f.stale) return '미판독';
    if (f.enum && f.enum[v] !== undefined) return `${f.enum[v]} (${v})`;
    if (f.hex) return '0x' + v.toString(16).padStart(2, '0').toUpperCase();
    if (f.scale) return (v * f.scale).toFixed(3) + (f.unit ? ' ' + f.unit : '');
    if (f.type === 'f32') return v.toFixed(4) + (f.unit ? ' ' + f.unit : '');
    return v + (f.unit ? ' ' + f.unit : '');
  }

  renderTable() {
    const el = $('rg-table');
    el.innerHTML = '';
    for (const blk of this.map.blocks) {
      const h = document.createElement('div');
      h.className = 'rg-block';
      h.innerHTML = `<b>${blk.name}</b> <span class="unit">${blk.addr}:${blk.size}</span>` +
                    ` <span class="rw rw-${blk.rw}">${{r: '읽기전용', w: '쓰기가능', x: '예약'}[blk.rw]}</span>`;
      el.appendChild(h);
      if (blk.rw === 'x') continue;            // 예약 구간은 행을 만들지 않는다

      const tbl = document.createElement('table');
      tbl.className = 'rg';
      for (const f of blk.fields) {
        if (f.name.startsWith('reserved')) continue;
        const n = f.count || 1;
        for (let i = 0; i < n; i++) {
          const addr = f.addr + i * SIZE[f.type];
          const fresh = this.bytes && inFresh(this.fresh, addr, SIZE[f.type]);
          const v = fresh ? decode(this.bytes, addr, f.type) : null;
          const tr = document.createElement('tr');
          if (!fresh) tr.className = 'stale';
          const label = n > 1 ? `${f.name}[${i}]` : f.name;
          const editable = blk.rw === 'w';
          const pend = this.edits.get(addr);
          tr.innerHTML =
            `<td class="a">${addr}</td><td>${label}</td><td class="t">${f.type}</td>` +
            `<td class="v">${fresh ? this.fmt(f, v) : '미판독'}</td>` +
            `<td>${editable
                ? `<input class="rg-in" data-addr="${addr}" data-type="${f.type}" ` +
                  `value="${pend !== undefined ? pend : ''}" placeholder="쓸 값">`
                : ''}</td>`;
          tbl.appendChild(tr);
        }
      }
      el.appendChild(tbl);
    }
    el.querySelectorAll('.rg-in').forEach(inp => {
      inp.onchange = () => {
        const a = +inp.dataset.addr;
        if (inp.value === '') this.edits.delete(a);
        else this.edits.set(a, inp.value);
        this.renderEdits();
      };
    });
    this.renderEdits();
  }

  renderEdits() {
    const n = this.edits.size;
    $('rg-send').disabled = n === 0;
    $('rg-discard').disabled = n === 0;
    $('rg-pending').textContent = n === 0 ? '' :
      `보낼 값 ${n}개: ` + [...this.edits.entries()].map(([a, v]) => `${a}=${v}`).join(', ');
  }

  /** 쓰기는 **연속 구간 단위**로 나간다 — 흩어진 주소를 한 번에 쓰는 프로토콜이 없다. */
  async sendEdits() {
    if (!this.edits.size) return;
    const bytes = new Map();
    for (const [addr, raw] of this.edits) {
      const f = this.fieldAt(addr);
      if (!f) continue;
      const buf = new ArrayBuffer(SIZE[f.type]);
      const dv = new DataView(buf);
      const v = Number(raw);
      if (Number.isNaN(v)) { $('rg-notice').textContent = `${addr}: 숫자가 아니다`; return; }
      switch (f.type) {
        case 'u8':  dv.setUint8(0, v); break;
        case 'i8':  dv.setInt8(0, v); break;
        case 'u16': dv.setUint16(0, v, true); break;
        case 'i16': dv.setInt16(0, v, true); break;
        case 'u32': dv.setUint32(0, v, true); break;
        case 'i32': dv.setInt32(0, v, true); break;
        case 'f32': dv.setFloat32(0, v, true); break;
      }
      new Uint8Array(buf).forEach((b, i) => bytes.set(addr + i, b));
    }
    // 인접 바이트를 묶어 구간으로.
    const addrs = [...bytes.keys()].sort((a, b) => a - b);
    const runs = [];
    for (const a of addrs) {
      const last = runs[runs.length - 1];
      if (last && a === last.addr + last.data.length) last.data.push(bytes.get(a));
      else runs.push({addr: a, data: [bytes.get(a)]});
    }
    const msgs = [];
    for (const r of runs) {
      // raw_write 는 **manual 전용**이다 (B6). control 이면 브리지가 거부하고, 그 사유가
      // 그대로 여기 뜬다 — 웹이 조건을 흉내내지 않는다.
      const res = await post('/api/command', {
        slot: 255, action: 1, target_id: TARGET.ecu, cmd: CMD.raw_write,
        start_addr: r.addr, data: r.data, duration: 1});
      msgs.push((res.ok ? 'OK ' : '거부 ') + r.addr + ': ' + res.message);
      if (!res.ok) break;
    }
    $('rg-notice').textContent = msgs.join(' | ');
    if (msgs.every(m => m.startsWith('OK'))) { this.edits.clear(); this.renderTable(); }
  }

  fieldAt(addr) {
    for (const blk of this.map.blocks) {
      for (const f of blk.fields) {
        const n = f.count || 1;
        for (let i = 0; i < n; i++) {
          if (f.addr + i * SIZE[f.type] === addr) return f;
        }
      }
    }
    return null;
  }

  renderSlots() {
    const el = $('rg-slots');
    el.innerHTML = '';
    for (let i = 0; i < 4; i++) {
      const d = document.createElement('div');
      d.className = 'rg-slot';
      d.innerHTML =
        `<b>슬롯 ${i}</b> <span class="unit">낮을수록 우선</span><br>` +
        `<select id="sc${i}">${Object.keys(CMD).map(k => `<option>${k}</option>`).join('')}</select> ` +
        `<input id="sa${i}" size="14" placeholder="인자 (addr len / 값)"> ` +
        `<select id="sd${i}"><option value="1">once</option><option value="0">forever</option>` +
        `<option value="5">5s</option><option value="30">30s</option></select> ` +
        `<button id="ss${i}">SET</button> <button id="sr${i}">RESET</button>`;
      el.appendChild(d);
    }
    for (let i = 0; i < 4; i++) {
      $('ss' + i).onclick = async () => {
        const name = $('sc' + i).value;
        const parts = ($('sa' + i).value || '').trim().split(/\s+/).filter(x => x);
        const body = {slot: i, action: 1, target_id: TARGET.ecu, cmd: CMD[name],
                      duration: +$('sd' + i).value};
        if (name === 'raw_read') { body.start_addr = +parts[0]; body.data_len = +parts[1]; }
        else if (name === 'raw_write') { body.start_addr = +parts[0]; body.data = parts.slice(1).map(Number); }
        else if (name.startsWith('set_')) body.args = [+parts[0]];
        const r = await post('/api/command', body);
        $('rg-notice').textContent = (r.ok ? '' : '거부: ') + r.message;
      };
      $('sr' + i).onclick = async () => {
        const r = await post('/api/command', {slot: i, action: 0});
        $('rg-notice').textContent = (r.ok ? '' : '거부: ') + r.message;
      };
    }
  }
}
