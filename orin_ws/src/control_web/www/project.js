// TAB3 프로젝트 — 보드 상태 + ECU/DPC 레지스터 조작 (redesign/09 §5.3 ①②③, U11)
//
// ## ⚠ 지금 DPC 레지스터는 구현이 미완이다 (사용자 확인, 2026-08-06)
//
// **읽기는 된다 — 그래서 값이 갱신되는 것처럼 보인다. 그러나 그 값을 의미 있는 상태로
// 읽으면 안 된다.** 펌웨어가 레지스터를 완전히 구현하지 않았고, 쓴 값을 반영하지도 않는다
// (`RD_MAP_MARSHAL_CONSUME` 주석 처리).
//
// 이 화면은 그래서 **관측을 진실로 승격시키지 않는다.** 표는 계획서(09 §5.3)가 정한
// 구조대로 그리고, 각 칸이 무엇을 근거로 하는지만 말한다. 지금 실제로 말할 수 있는 것은
// "DPC 가 이 바이트를 갖고 있다" 하나뿐이고, 나머지는 전부 **미확인**이다.
//
// ⚠ 이 파일을 고칠 때 **실기에서 읽힌 값으로 설계를 되돌리지 말 것.** 예를 들어
//   `sys_state`(57)가 HOLD 로 읽힌다고 해서 "DPC 는 HOLD 상태다" 라고 그리면 안 된다 —
//   그 레지스터도 구현 대상이고, 지금 값은 보증되지 않는다. 실제로 이 파일의 첫 판이
//   그 오류를 저질렀다(관측값에서 UI 결론을 끌어냄).
//
// ## 그래서 열을 둘로 나눈다
//
//   ① 수신  — CMD 영역 에코 ({120,8} 20Hz). **"DPC 가 이 바이트를 갖고 있다" 까지만.**
//   ② 반영  — 독립된 관측 필드가 **설계상** 있는 자리. 지금은 그 값도 미보증이다.
//
// 설계상 독립 관측이 있는 것은 둘뿐이다:
//   · 127 sys_state_target → `sys_state`(57)
//   · 122 locker_en        → `lock_contact`(66)
// 나머지(123 boot / 124 light / 125 servo / 126 mode)는 되먹임 선 자체가 없다.
//
// 하나로 합치면 에코가 동작으로 읽힌다 — 이 프로젝트가 반복해서 지워 온 결함 형태다.
//
// ## 계획서 §5.3 ③ 표와 다른 곳 (사실 정정)
//
// 계획서는 *"126 은 0xFF 만 읽힌다"* 고 적었으나 실제로는 쓴 값이 에코된다. 그 자체는
// 사실 정정이라 반영했다 — 다만 **에코된다는 것이 그 값이 유효하다는 뜻은 아니다.**

const $ = id => document.getElementById(id);

const TARGET = {ecu: 225, dpc: 209};

// CommandSet.srv 의 cmd 번호. **여기 숫자를 늘리지 않는다** — 새 조작이 필요하면
// 카탈로그(브리지)에 의미 명령을 만들고 그 번호를 여기 적는다.
const CMD = {
  set_soft_estop: 13, set_use_lpf: 14,
  dpc_set_boot: 40, dpc_set_light: 41, dpc_set_servo: 42,
  dpc_set_mode: 43, dpc_set_seq: 44, dpc_read_all: 45,
};
// ControlConfig.srv — ECU mode(190)만 이 경로다 (신규 명령을 만들지 않는다, 09 §5.3 ②).
const OP_SET_MODE = 2;

// ⚠ index 5 는 **WAIT** 다 (DESCEND_3 이 아니다). 처음에 그렇게 적었다가 U13 에서 잡혔다 —
// 헤더 `SysStateName()` 이 정본이고 `test_regmap.py` 가 그것과 대조한다.
// WAIT 는 "Orin 이 target=ASCEND_1 을 써야 진행" 하는 지점이라, 이름을 틀리면 화면이
// 전개가 멈춘 이유를 엉뚱하게 설명한다.
const DPC_STATE = ['CTRL', 'HOLD', 'INIT', 'DESCEND_1', 'DESCEND_2', 'WAIT',
                   'ASCEND_1', 'ASCEND_2', 'FINISH', 'RSVD', 'ERROR'];
// 펌웨어가 값을 검증 없이 대입하므로 브리지가 넷만 허용한다 (rd_register_dpc.hpp
// IsWritableTarget). 중간 상태를 쓰면 단계를 건너뛴다.
const WRITABLE_TARGETS = [0, 1, 2, 6];

// DPC 섀도에서 읽는 자리
const A = {lock_contact: 66, sys_state: 57,
           locker_a: 120, boot_a: 121,
           locker: 122, boot: 123, light: 124, servo: 125, mode: 126, seq: 127};

async function post(path, body) {
  const r = await fetch(path, {method: 'POST', headers: {'Content-Type': 'application/json'},
                               body: JSON.stringify(body)});
  return r.json();
}

export class ProjectTab {
  constructor() {
    this.dpc = null;        // 마지막 DPC 섀도 바이트열
    this.dpcAge = null;     // 구간별 age (U8) — 화면이 "언제 읽은 값인가" 를 말할 수 있다
    this.busy = false;
    this.sent = {};         // 마지막으로 보낸 값 (에코가 오기 전 표시용)
  }

  init() {
    // 조작 버튼. 모두 같은 경로(`/api/command`)라 표로 돌린다 — 버튼마다 핸들러를
    // 손으로 적으면 cmd 번호와 target 이 흩어지고, 그게 갈라지는 자리다.
    for (const b of document.querySelectorAll('#pj-panel [data-cmd]')) {
      b.onclick = () => this.sendCmd(b.dataset.cmd, Number(b.dataset.val), b);
    }
    $('pj-ecu-mode-manual').onclick = () => this.setEcuMode(0);
    $('pj-ecu-mode-auto').onclick   = () => this.setEcuMode(1);
    $('pj-seq-send').onclick = () => {
      const v = Number($('pj-seq-val').value);
      if (!WRITABLE_TARGETS.includes(v)) {
        this.notice(`sys_state_target 은 ${WRITABLE_TARGETS.join('/')} 만 쓸 수 있다 — ` +
                    `FSM 중간 상태를 쓰면 단계를 건너뛴다`);
        return;
      }
      this.sendCmd('dpc_set_seq', v, $('pj-seq-send'));
    };
    $('pj-refresh').onclick = () => this.readDpc();

    // 09 §5.3 ④ (U12) — jeongae.
    $('pj-seq-start').onclick = async () => {
      const r = await post('/api/jeongae/trigger', {});
      this.notice(r.message || '');
    };
    $('pj-lock-on').onclick  = async () => this.notice(
      (await post('/api/jeongae/lock', {on: true})).message || '');
    $('pj-lock-off').onclick = async () => this.notice(
      (await post('/api/jeongae/lock', {on: false})).message || '');

    // 2026-08-07 — cmd_vel 0 수렴 스킵 (운용 스위치). 기본 off.
    $('zs-on').onclick  = async () => this.notice(
      (await post('/api/cmdvel/zero_skip', {on: true})).message || '');
    $('zs-off').onclick = async () => this.notice(
      (await post('/api/cmdvel/zero_skip', {on: false})).message || '');

    // 탭 진입 시 1회 전체 읽기 (09 §5.3 ③). 주기 슬롯이 {120,8} 만 덮으므로
    // 나머지(sys_state 는 {46,20} 안이라 덮이지만 lock_contact 66 은 아니다)를 채운다.
    document.querySelector('.tabs button[data-tab="t3"]')
      .addEventListener('click', () => this.onEnter());
    if (document.querySelector('.tabs button[data-tab="t3"]').classList.contains('active')) {
      this.onEnter();
    }
  }

  notice(msg) { $('pj-notice').textContent = msg || ''; }

  async onEnter() {
    // 브리지가 없으면 조용히 넘어간다 — 없는 것이 정상인 상태다.
    if (!window.state || !window.state.bridge ||
        window.state.bridge.state !== 'running') return;
    await this.readAllOnce();
  }

  async readAllOnce() {
    // dpc_read_all(45) — 한 트랜잭션 136B. 주기 슬롯이 안 읽는 구간까지 채운다.
    await post('/api/command', {slot: 255, action: 1, target_id: TARGET.dpc,
                                cmd: CMD.dpc_read_all, duration: 1});
    await new Promise(r => setTimeout(r, 400));   // 슬롯이 한 바퀴 돌 시간
    await this.readDpc();
  }

  async readDpc() {
    try {
      const r = await fetch('/api/registers?target=' + TARGET.dpc);
      const d = await r.json();
      if (d.bytes) {
        this.dpc = Uint8Array.from(d.bytes.match(/../g).map(h => parseInt(h, 16)));
        this.dpcAge = d.spans || [];
      }
    } catch (e) { /* 폴링이라 조용히 넘어간다 — 다음 주기에 다시 온다 */ }
    this.render();
  }

  // 이 주소를 마지막으로 읽은 지 몇 초 됐나 (U8 `spans`). 못 읽었으면 null.
  ageOf(addr) {
    for (const s of (this.dpcAge || [])) {
      if (addr >= s.addr && addr < s.addr + s.len) return s.age_s;
    }
    return null;
  }

  async sendCmd(name, val, btn) {
    if (this.busy) return;
    this.busy = true;
    if (btn) btn.disabled = true;
    try {
      const r = await post('/api/command', {
        slot: 255, action: 1, target_id: TARGET.dpc, cmd: CMD[name],
        args: [val], duration: 1,
      });
      this.notice(r.message || '');
      if (r.ok) this.sent[name] = val;
      // 보낸 직후 한 번 읽어 에코를 확인한다. **낙관적으로 그리지 않는다** —
      // 버튼을 눌렀다는 사실로 상태를 칠하면 거부됐을 때 화면이 거짓말을 한다.
      await new Promise(res => setTimeout(res, 300));
      await this.readDpc();
    } finally {
      this.busy = false;
      if (btn) btn.disabled = false;
    }
  }

  async setEcuMode(v) {
    // ECU 190 — ControlConfig 경로 (09 §5.3 ②). 브리지가 IDLE 게이트를 건다.
    const r = await post('/api/config', {op: OP_SET_MODE, value: v});
    this.notice(r.message || '');
  }

  // ── 그리기 ────────────────────────────────────────────────────────────────
  render() {
    this.renderNodes();
    this.renderDpc();
    this.renderSeq();
    this.renderZeroSkip();
  }

  // 2026-08-07 — 0 수렴 스킵 상태. **모르면 켜짐으로 그리지 않는다** — 기본이 off 이므로
  // "모른다" 를 ON 으로 칠하면 화면이 없는 위험을 있다고 말하게 된다.
  renderZeroSkip() {
    const s = window.state || {};
    const on = s.cmd_vel_zero_skip;
    const el = $('zs-state');
    el.textContent = (on === null || on === undefined) ? '—' : (on ? 'ON' : 'OFF');
    // 켜져 있으면 경고색 — 이 스위치는 켠 쪽이 위험한 방향이다.
    el.className = on ? 'bad' : '';
    const t = s.cmd_vel_zero_timeout_s;
    $('zs-timeout').textContent = (t === null || t === undefined) ? '—' : `${t}s`;
    $('zs-on').disabled  = (on === true);
    $('zs-off').disabled = (on === false);
  }

  // 09 §5.3 ④ — 전개 단계 + **배타 잠금** (U12).
  renderSeq() {
    const q = (window.state || {}).sequence;
    const st = $('pj-seqstate'), wt = $('pj-seqwait'), lk = $('pj-lockstate');
    if (!q) {                       // 브리지가 없거나 상태를 아직 못 받았다 — **모른다**
      st.textContent = '—'; wt.textContent = '—'; lk.textContent = '—';
      // 모를 때는 **잠근다.** 시퀀스가 도는 중일 수도 있는데 열어 두면 겹쳐 쓰게 된다.
      // 06 §4.11 에서 "모르면 관대하게" 가 낡은 값을 신선하게 보이게 만든 적이 있다.
      this.setManualSeqEnabled(false, '브리지 상태를 모른다');
      return;
    }
    st.textContent = q.state;
    wt.textContent = q.busy ? `${q.wait_ticks}/${q.wait_max} tick` : '—';
    lk.textContent = q.locked ? 'ON' : 'OFF';

    // **`busy` 는 브리지가 준 값을 그대로 쓴다.** 여기서 `state !== 'IDLE'` 로 흉내내면
    // 단계 이름이 하나 늘 때마다 이 파일도 같이 고쳐야 하고, 안 고치면 새 단계에서
    // 수동 입력이 열린 채로 남는다.
    this.setManualSeqEnabled(!q.busy,
      q.busy ? `자동 전개 진행 중 (${q.state}) — 겹쳐 쓰면 시퀀스가 Abort 한다` : '');

    // 전개 시작은 lock 중이면 무시되므로 버튼도 같이 잠근다 (눌리는데 아무 일도
    // 안 일어나는 버튼이 가장 나쁘다).
    $('pj-seq-start').disabled = q.locked || q.busy;
  }

  setManualSeqEnabled(on, why) {
    const el = $('pj-seq-val'), btn = $('pj-seq-send');
    if (!el || !btn) return;
    el.disabled = !on;
    btn.disabled = !on;
    btn.title = on ? '' : why;
    el.title = on ? '' : why;
  }

  renderNodes() {
    const n = (window.state || {}).nodes;
    for (const k of ['ecu', 'dpc', 'pcu']) {
      const el = $('pj-node-' + k);
      if (!el) continue;
      if (!n) {                       // **모른다** — 브리지가 없거나 상태를 아직 못 받았다
        el.className = 'nodedot unknown';
        el.title = '브리지 상태를 아직 모른다';
        el.nextElementSibling.textContent = k.toUpperCase() + ' —';
        continue;
      }
      const d = n[k] || {};
      // `enabled`(설정으로 껐다) 와 `connected`(켰는데 응답이 없다)는 **다른 상황**이다.
      // 하나로 합치면 "안 붙였다" 와 "고장" 이 같은 회색이 된다.
      const cls = !d.enabled ? 'off' : (d.connected ? 'on' : 'bad');
      el.className = 'nodedot ' + cls;
      const why = !d.enabled ? '읽기 꺼짐 (기동 파라미터)'
                : d.connected ? '응답 정상'
                : `응답 없음 (연속 실패 ${d.fail_streak})`;
      el.title = why;
      el.nextElementSibling.textContent = k.toUpperCase() + ' ' +
        (!d.enabled ? '꺼짐' : d.connected ? '정상' : '끊김');
    }
  }

  renderDpc() {
    const b = this.dpc;
    const val = a => (b ? b[a] : null);
    const show = (id, v, fmt) => {
      const el = $(id);
      if (!el) return;
      el.textContent = (v === null || v === undefined) ? '미판독' : (fmt ? fmt(v) : String(v));
      el.className = 'rv' + ((v === null || v === undefined) ? ' unread' : '');
    };

    const onoff = v => (v ? 'ON' : 'OFF');
    show('pj-v-boot',  val(A.boot),  onoff);
    show('pj-v-light', val(A.light), onoff);
    show('pj-v-servo', val(A.servo), v => ['IDLE', 'LOCK', 'UNLOCK'][v] || ('? ' + v));
    // A14 — 126·127 은 **일회성 소비 트리거**다 (10_defects C4). 펌웨어가 소비하면
    // 0xFF 로 되돌리므로 **255 가 정상(소비됨)**. 255 가 아니면 "아직 소비 안 됨" 이고,
    // 지금 그것이 읽힌다면 CONSUME 이 꺼져 있다는 뜻이다 — 진단 신호로 그린다.
    const oneshot = (label) => (v) =>
      (v === 255) ? '소비됨' : `대기 ${label(v)} — 미소비`;
    show('pj-v-mode', val(A.mode), oneshot(v => (v ? 'AUTO' : 'MANUAL') + ` (${v})`));
    show('pj-v-seq',  val(A.seq),  oneshot(v => (DPC_STATE[v] || '?') + ` (${v})`));

    // **반영** 쪽 — 독립 관측이 있는 둘만.
    show('pj-v-sysstate', val(A.sys_state), v => (DPC_STATE[v] || ('? ' + v)) + ` (${v})`);
    show('pj-v-lock', val(A.lock_contact),
         v => '접점 ' + [0, 1, 2, 3].map(i => (v >> i) & 1).join(''));

    // 나이 — "언제 읽은 값인가". 주기 슬롯이 덮는 구간은 ms 대, 나머지는 기동 스냅샷이다.
    const age = this.ageOf(A.light);
    $('pj-age').textContent = (age === null) ? '미판독'
      : (age < 1 ? `${(age * 1000) | 0}ms 전` : `${age.toFixed(1)}s 전`);
  }
}
