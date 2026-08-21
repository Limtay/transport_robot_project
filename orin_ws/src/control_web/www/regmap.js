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
  // ⚠ **이게 빠져 있어서 DPC 전체 읽기가 안 됐다** (2026-08-07).
  //   `CMD.dpc_read_all` 이 undefined 면 `JSON.stringify` 가 그 키를 **통째로 빼고**,
  //   서버의 `req.get('cmd', 0)` 이 0(=read_sys)으로 떨어진다. 그러면 브리지 카탈로그가
  //   "read_sys 는 ECU 전용" 으로 거부한다 — 화면에는 엉뚱한 사유가 뜬다.
  dpc_read_all: 45,
};

// 보드마다 **의미 단위 READ 가 다르다** (09 §6). ECU 는 다섯, DPC 는 전 구간 하나뿐이다.
// 종전에는 ECU 목록을 그대로 그려 놓고 DPC 일 때 전부 dpc_read_all 로 바꿔 보냈다 —
// 버튼 다섯 개가 같은 일을 하는 화면이라 조작자가 무엇이 다른지 알 수 없었다.
const READS = {
  ecu: ['read_sys', 'read_motor', 'read_sensor', 'read_diag', 'read_all'],
  dpc: ['dpc_read_all'],
};
// `전체 읽기` 버튼이 보내는 것 — 보드마다 이름이 다르다.
const READ_ALL = {ecu: 'read_all', dpc: 'dpc_read_all'};
// ⚠ DPC 는 **209(0xD1)** 다. 210(0xD2) 은 낡은 값이었고 브리지가 target_id 로 거부한다
// (09 §0.2 — ID 변경 때 이 한 줄이 누락됐다).
const TARGET = {ecu: 225, dpc: 209, pcu: 161};
// 09 §5.4 (U13) — 보드별 표. PCU 는 레지스터 미확정이라 자리만 둔다.
const MAPS = {ecu: '/regmap.json', dpc: '/regmap.dpc.json'};

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

/* U13 — `unionSize`/`inFresh` 를 지웠다.
 *
 * 둘 다 구 `fresh` 배열(= "지금 주기적으로 읽히는 구간")을 다뤘다. U8 의 `spans` 는
 * **겹치지 않게 이미 쪼개져** 오므로 합집합을 셀 이유가 없고, 신선도 판정도
 * `spanAt()` 하나로 끝난다. 안 쓰는 채로 두면 다음 사람이 어느 쪽이 진짜인지 묻게 된다. */

// U8 — 경과시간 표기. 초 단위로 다 찍으면 "0.0s" 가 줄줄이 나와 실시간 구간과
// 기동 스냅샷의 차이가 눈에 안 들어온다.
function fmtAge(s) {
  if (s === null || s === undefined) return '미판독';
  if (s < 1) return `${Math.round(s * 1000)}ms`;
  if (s < 60) return `${s.toFixed(1)}s`;
  return `${Math.round(s / 60)}분`;
}

// 이 주소를 마지막으로 읽은 구간 (U8 `spans`). 없으면 null = 한 번도 안 읽음.
function spanAt(spans, addr, len) {
  for (const s of spans || []) {
    if (addr >= s.addr && addr + len <= s.addr + s.len) return s;
  }
  return null;
}

export class RegMap {
  constructor() {
    this.map = null;
    this.spans = [];              // U8 — 구간별 age
    this.bytes = null;
    this.edits = new Map();       // addr -> 입력값 (전송 전)
    this.timer = null;
    this.target = 'ecu';
    this.maps = {};               // 보드별 표 (한 번 읽으면 캐시)
  }

  async loadMap(which) {
    if (!this.maps[which]) this.maps[which] = await (await fetch(MAPS[which])).json();
    this.map = this.maps[which];
  }

  async setTarget(which) {
    // 보드를 바꾸면 **편집 중인 값을 버린다.** 남겨 두면 ECU 주소로 찍은 값이 DPC 로
    // 나간다 — 주소가 같아도 뜻이 전혀 다르다.
    this.edits.clear();
    this.target = which;
    this.bytes = null; this.spans = [];
    await this.loadMap(which);
    this.renderReadButtons();
    $('rg-reboot').textContent = which.toUpperCase() + ' 리부트';
    this.renderTable(); this.renderEdits();
    await this.poll();
  }

  async init() {
    await this.loadMap(this.target);
    this.renderTable();
    this.renderSlots();

    $('rg-refresh').onclick = () => this.issueRead(READ_ALL[this.target]);
    $('rg-auto').onchange = (e) => this.setAuto(e.target.checked);
    this.renderReadButtons();
    $('rg-send').onclick = () => this.sendEdits();
    $('rg-discard').onclick = () => { this.edits.clear(); this.renderTable(); this.renderEdits(); };
    $('rg-reboot').onclick = () => this.reboot();
    for (const el of document.querySelectorAll('input[name="rg-target"]')) {
      el.onchange = () => { if (el.checked) this.setTarget(el.value); };
    }
    this.poll();
    // 09 §5.4 ① — **10Hz 폴링.** `/api/registers` 는 섀도 덤프라 **버스를 안 건드린다**
    // (통신 부하 0). 화면이 안 움직이던 이유는 폴링을 안 해서였지 비용 때문이 아니었다.
    setInterval(() => this.poll(), 100);
  }

  setAuto(on) {
    clearInterval(this.timer);
    this.timer = null;
    // duration=0(forever)로 걸면 슬롯을 붙잡고 있게 된다. **once 를 주기적으로** 넣는다 —
    // 웹이 죽어도 슬롯이 남지 않는다.
    if (on) this.timer = setInterval(() => this.issueRead('read_all'), 2000);
  }

  // 보드에 맞는 READ 버튼만 그린다.
  renderReadButtons() {
    const box = $('rg-reads');
    box.innerHTML = '';
    for (const name of (READS[this.target] || [])) {
      const b = document.createElement('button');
      b.textContent = name;
      b.onclick = () => this.issueRead(name);
      box.appendChild(b);
    }
  }

  async issueRead(name) {
    const cmd = CMD[name];
    if (cmd === undefined) {          // 표에 없는 이름 — 조용히 0 으로 떨어지지 않게 막는다
      $('rg-notice').textContent = `알 수 없는 읽기 명령: ${name}`;
      return;
    }
    const r = await post('/api/command', {
      slot: 255, action: 1, target_id: TARGET[this.target], cmd, duration: 1});
    $('rg-notice').textContent = (r.ok ? '' : '거부: ') + r.message;
  }

  async reboot() {
    // 되돌릴 수 없는 조작이라 확인을 받는다. 브리지도 safe_stop 을 요구하지만,
    // 게이트가 있다고 해서 실수로 누르는 것을 막아 주지는 않는다.
    //
    // ⚠ **선택된 보드로 보낸다** (2026-08-07). 종전에는 `TARGET.ecu` 고정이라 DPC 표를
    //   띄워 놓고 리부트를 눌러도 **ECU 가 재부팅됐다.** 브리지 카탈로그의 REBOOT 은
    //   원래 `kTargetAny` 라 보드를 가리지 않는다 — 막고 있던 것은 웹뿐이었다.
    const name = this.target.toUpperCase();
    if (!window.confirm(
        `${name} 를 리부트한다. 3초간 해당 보드 통신이 끊기고 진행 중인 것은 멈춘다. 계속?`)) return;
    const r = await post('/api/command', {
      slot: 255, action: 1, target_id: TARGET[this.target], cmd: CMD.reboot, duration: 1});
    $('rg-notice').textContent = (r.ok ? '' : '거부: ') + r.message;
  }

  async poll() {
    const res = await fetch('/api/registers?target=' + TARGET[this.target]);
    if (!res.ok) {
      $('rg-stat').textContent = '브리지 없음';
      this.bytes = null;
      this.renderTable();
      return;
    }
    const d = await res.json();
    this.spans = d.spans || [];      // U8 — 구간별 (age_s, src)
    const hex = d.bytes || '';
    const b = new Uint8Array(hex.length / 2);
    for (let i = 0; i < b.length; i++) b[i] = parseInt(hex.substr(i * 2, 2), 16);
    this.bytes = b;
    // U8 — **신선/미판독 2값에서 경과시간으로.** 종전에는 "30초 전 기동 스냅샷" 과
    // "한 번도 안 읽음" 이 같은 회색이었다 (09 §5.4 ①).
    const read = this.spans.reduce((n, s) => n + s.len, 0);
    const newest = this.spans.length ? Math.min(...this.spans.map(s => s.age_s)) : null;
    const oldest = this.spans.length ? Math.max(...this.spans.map(s => s.age_s)) : null;
    $('rg-stat').textContent = this.spans.length
      ? `${read}/${d.total}B 읽음 · 최신 ${fmtAge(newest)} / 가장 낡은 것 ${fmtAge(oldest)}`
      : `0/${d.total}B — 한 번도 안 읽었다`;
    this.renderTable();
  }

  fmt(f, v) {
    if (v === null) return '—';
    // A14 — **일회성 소비 트리거** (10_defects C4). DPC 126·127 은 펌웨어가 소비한 뒤
    // 0xFF 로 되돌리므로 **255 가 정상(=소비됨)** 이다. 그냥 숫자로 그리면 조작자가
    // 정상 동작을 오류로 읽는다. 뒤집으면 진단 신호이기도 하다 — 255 가 아니면
    // "썼는데 아직 소비되지 않았다" 이고, 지금은 그것이 CONSUME 이 꺼져 있다는 뜻이다.
    if (f.oneshot !== undefined) {
      return (v === f.oneshot) ? '소비됨'
           : `대기 ${f.enum && f.enum[v] !== undefined ? f.enum[v] + ' ' : ''}(${v}) — 미소비`;
    }
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
          // U8 — **읽은 적이 있는가**로 값을 그릴지 정하고, 얼마나 낡았는지는 따로 적는다.
          // 종전 판정(`inFresh`)은 "지금 주기적으로 읽히는가" 라서, 기동 전체읽기로
          // 채워진 구간이 값이 있는데도 '미판독' 으로 나왔다.
          const sp = this.bytes ? spanAt(this.spans, addr, SIZE[f.type]) : null;
          const seen = sp !== null;
          const v = seen ? decode(this.bytes, addr, f.type) : null;
          const tr = document.createElement('tr');
          // 1초 넘게 안 읽힌 값은 흐리게 — 실시간 구간과 스냅샷을 눈으로 가른다.
          if (!seen) tr.className = 'stale';
          else if (sp.age_s >= 1.0) tr.className = 'aged';
          const label = n > 1 ? `${f.name}[${i}]` : f.name;
          const editable = blk.rw === 'w';
          const pend = this.edits.get(addr);
          tr.innerHTML =
            `<td class="a">${addr}</td><td>${label}</td><td class="t">${f.type}</td>` +
            `<td class="v">${seen ? this.fmt(f, v) : '미판독'}</td>` +
            `<td class="age">${seen ? fmtAge(sp.age_s) : ''}</td>` +
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
      // raw_write 는 **정지 상태에서만** 된다 (모드 제한은 09 §5.4 에서 풀렸다).
      // 달리는 중이면 브리지가 거부하고, 그 사유가
      // 그대로 여기 뜬다 — 웹이 조건을 흉내내지 않는다.
      // ⚠ **선택된 보드로 보낸다.** 여기가 `TARGET.ecu` 로 박혀 있었다 (U13 에서 발견):
      //   DPC 표를 띄우고 편집하면 같은 주소의 **ECU 레지스터**로 나갔다. 주소가 같아도
      //   뜻이 전혀 달라서 — 예컨대 DPC 126(mode) 을 고치려던 값이 ECU 126(mode) 에 쓰인다 —
      //   화면에는 "OK" 가 뜨고 엉뚱한 보드가 바뀐다.
      const res = await post('/api/command', {
        slot: 255, action: 1, target_id: TARGET[this.target], cmd: CMD.raw_write,
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
