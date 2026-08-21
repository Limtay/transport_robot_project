"""TAB3 프로젝트 — 주소·명령 번호 대조와 "수신 ≠ 반영" 표시 (09 §5.3, U11).

## 왜 대조하나

`project.js` 는 DPC 레지스터 주소(123~127·57·66)와 `CommandSet` 의 cmd 번호(40~45)를
**손으로 적어 들고 있다.** 손으로 옮겨 적은 표는 반드시 갈라지고, 이 프로젝트에서 이미
세 번 겪었다(07 §5.1). 그래서 브리지 헤더/srv 를 **정본으로 삼아 대조**한다.

갈라졌을 때의 증상이 고약하다: 주소가 하나 밀리면 조명 버튼이 서보를 움직인다.
"버튼을 눌렀는데 엉뚱한 게 동작한다" 는 화면 어디에도 안 나온다.

## "수신 ≠ 반영" 을 왜 테스트하나

2026-08-06 실측으로 DPC 123~127 이 **전부 리드백된다.** 그런데 펌웨어의
`RD_MAP_MARSHAL_CONSUME` 이 주석 처리돼 있어 **DPC 는 저장만 하고 아무것도 안 한다.**
에코를 그대로 초록불로 그리면 이 상태가 "정상 동작" 으로 보인다 — 그게 이 탭이
막아야 하는 유일한 거짓말이다. 표에 그 구분이 살아 있는지 본다.
"""

import os
import re

import pytest

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # .../src/control_web
WS = os.path.dirname(HERE)                                           # .../src
JS = open(os.path.join(HERE, 'www', 'project.js'), encoding='utf-8').read()
INDEX = open(os.path.join(HERE, 'www', 'index.html'), encoding='utf-8').read()

DPC_HDR = os.path.join(WS, 'orin_firmware_bridge', 'include', 'orin_firmware_bridge',
                       'core', 'rd_register_dpc.hpp')
CMDSET = os.path.join(WS, 'mgs_tp_msgs', 'srv', 'CommandSet.srv')
CFG = os.path.join(WS, 'mgs_tp_msgs', 'srv', 'ControlConfig.srv')


def code_only(js):
    """`//` 주석을 걷어낸 JS.

    ⚠ **이 프로젝트에서 같은 실수를 두 번 했다.** 코드에 대한 단언을 원문에 그대로 걸면
    *"이렇게 하면 안 된다"* 고 적어 둔 **설명 문구가 걸려서** 테스트가 실패한다
    (U10 에서 한 번, U12 에서 또 한 번). 지키려는 것은 낱말이 아니라 **동작**이므로
    주석을 뺀 뒤에 본다. 설명은 오히려 있어야 좋다.
    """
    return '\n'.join(re.sub(r'//.*$', '', ln) for ln in js.splitlines())


def js_addr_table():
    """`const A = {...}` 를 뜯는다."""
    m = re.search(r'const A = \{(.*?)\};', JS, re.S)
    assert m, 'project.js 의 주소표(const A)를 못 찾았다'
    return {k: int(v) for k, v in re.findall(r'(\w+):\s*(\d+)', m.group(1))}


def hdr_addrs():
    """`/* addr NNN */ type name;` 주석에서 (이름 → 주소)."""
    src = open(DPC_HDR, encoding='utf-8').read()
    return re.findall(r'/\*\s*addr\s+(\d+)\s*\*/\s*\w[\w:]*\s+(\w+)', src)


# ── 주소 대조 ──────────────────────────────────────────────────────────────
@pytest.mark.parametrize('js_key,hdr_name', [
    ('boot', 'boot_en'), ('light', 'light_en'), ('servo', 'servo_cmd'),
    ('mode', 'mode'), ('seq', 'sys_state_target'),
    ('sys_state', 'sys_state'), ('lock_contact', 'lock_contact'),
])
def test_dpc_addresses_match_the_firmware_header(js_key, hdr_name):
    a = js_addr_table()
    pairs = hdr_addrs()
    # `boot_en`·`locker_en` 은 DPC-A/DPC-B 양쪽에 있다 — CMD/DPCB(122~127) 쪽을 쓴다.
    cands = [int(addr) for addr, name in pairs if name == hdr_name]
    assert cands, '헤더에 %s 가 없다' % hdr_name
    assert a[js_key] in cands, (
        'project.js A.%s = %d 인데 헤더의 %s 는 %s — **주소가 갈라졌다.** '
        '한 칸 밀리면 조명 버튼이 서보를 움직인다' % (js_key, a[js_key], hdr_name, cands))


def test_dpcb_command_block_is_where_we_think():
    """122~127 이 CMD/DPCB 연속 블록인가 — U3 의 `{120,8}` 리드백이 성립하는 근거다."""
    a = js_addr_table()
    assert a['locker'] == 122 and a['boot'] == 123 and a['light'] == 124
    assert a['servo'] == 125 and a['mode'] == 126 and a['seq'] == 127
    assert a['seq'] < 120 + 8, '127 이 {120,8} 밖이면 에코가 안 온다'


# ── 명령 번호 대조 ─────────────────────────────────────────────────────────
def test_command_numbers_match_the_srv():
    srv = open(CMDSET, encoding='utf-8').read()
    want = dict(re.findall(r'uint8 CMD_(\w+)\s*=\s*(\d+)', srv))
    m = re.search(r'const CMD = \{(.*?)\};', JS, re.S)
    got = {k: v for k, v in re.findall(r'(\w+):\s*(\d+)', m.group(1))}
    for name, num in got.items():
        key = name.upper()
        assert key in want, 'CommandSet.srv 에 CMD_%s 가 없다' % key
        assert want[key] == num, (
            'project.js CMD.%s=%s 인데 srv 는 %s — 번호가 갈라졌다' % (name, num, want[key]))


def test_op_set_mode_matches_the_srv():
    """ECU 190 은 **새 명령을 만들지 않고** `OP_SET_MODE` 를 쓴다 (09 §5.3 ②).

    같은 레지스터를 바꾸는 길이 둘이 되면 게이트가 갈라진다.
    """
    srv = open(CFG, encoding='utf-8').read()
    want = int(re.search(r'uint8 OP_SET_MODE\s*=\s*(\d+)', srv).group(1))
    got = int(re.search(r'const OP_SET_MODE = (\d+);', JS).group(1))
    assert got == want, 'OP_SET_MODE 가 %d 인데 project.js 는 %d' % (want, got)


def test_ecu_mode_does_not_go_through_the_command_catalog():
    """ECU 190 버튼이 `/api/command` 로 새지 않는가."""
    m = re.search(r'setEcuMode\(v\)\s*\{(.*?)\n  \}', code_only(JS), re.S)
    assert m, 'setEcuMode 를 못 찾았다'
    assert '/api/config' in m.group(1)
    assert '/api/command' not in m.group(1), (
        'ECU mode 가 커맨드 카탈로그로 나간다 — auto_mode 를 바꾸는 길이 둘이 된다')


def test_writable_seq_targets_match_the_firmware_gate():
    """127 에 쓸 수 있는 값 — `IsWritableTarget()` 과 같아야 한다."""
    src = open(DPC_HDR, encoding='utf-8').read()
    body = re.search(r'IsWritableTarget\(uint8_t s\)\s*\{(.*?)\}', src, re.S).group(1)
    names = set(re.findall(r'STATE_(\w+)', body))
    want_names = {'CTRL', 'HOLD', 'INIT', 'ASCEND_1'}
    assert names == want_names, '펌웨어 게이트가 바뀌었다: %s' % names
    got = re.search(r'const WRITABLE_TARGETS = \[([\d,\s]+)\]', JS).group(1)
    assert [int(x) for x in got.split(',')] == [0, 1, 2, 6]


# ── "수신 ≠ 반영" 이 화면에 남아 있는가 ────────────────────────────────────
def test_the_table_separates_receipt_from_effect():
    """표에 **수신**과 **반영** 열이 둘 다 있어야 한다.

    하나로 합치는 순간 에코가 동작으로 읽힌다 — 지금 펌웨어에서 그건 100% 거짓이다.
    """
    head = re.search(r'<thead>(.*?)</thead>', INDEX, re.S)
    assert head, 'TAB3 표의 thead 를 못 찾았다'
    cols = re.findall(r'<th>([^<]+)</th>', head.group(1))
    assert any('수신' in c for c in cols), cols
    assert any('반영' in c for c in cols), cols


def test_registers_without_feedback_carry_a_badge():
    """되먹임 선이 없는 것(ECU 189/191 · DPC 123/124/126)에 배지가 붙어 있는가."""
    assert INDEX.count('badge warn') >= 5, (
        '반영 확인이 안 되는 항목에 배지가 모자란다 — 에코만 보고 동작했다고 믿게 된다')


def test_it_warns_that_dpc_registers_are_incomplete():
    """**DPC 레지스터 구현이 미완**이라는 사실이 화면에 있어야 한다 (사용자 확인 2026-08-06).

    읽기가 되어서 값이 갱신되는 것처럼 보이는 것이 함정이다 — 갱신은 되지만 그 값이
    기계의 실제 상태를 뜻하지 않는다. 이 경고를 빼면 화면이 미완성 펌웨어의 출력을
    실상태처럼 단언하게 되고, 그게 이 탭이 만들 수 있는 가장 큰 거짓말이다.

    ⚠ 펌웨어 구현이 끝나면 **이 테스트를 지우고 경고도 같이 내려야 한다.** 남겨 두면
      이번엔 "멀쩡한 값을 못 믿게 만드는" 반대 방향 거짓말이 된다.
    """
    assert '구현 미완' in INDEX, (
        'TAB3 에 DPC 레지스터 미완 경고가 없다 — 표시값이 실상태로 읽힌다')
    # 눈에 띄는 자리여야 한다 (경고색). 회색 각주로 밀면 아무도 안 읽는다.
    assert 'notice bad' in INDEX


def test_it_warns_that_consume_is_commented_out():
    """지금 DPC 가 값을 저장만 한다는 사실이 화면에 적혀 있는가.

    이건 곧 풀릴 임시 조건이지만, **풀리기 전까지는 화면이 그렇게 말해야 한다.**
    """
    assert 'RD_MAP_MARSHAL_CONSUME' in INDEX


def test_project_js_is_declared_in_setup_py():
    setup_py = open(os.path.join(HERE, 'setup.py'), encoding='utf-8').read()
    assert "'www/project.js'" in setup_py


# ── U12 jeongae — 배타 잠금 ────────────────────────────────────────────────
def test_manual_seq_input_is_locked_from_the_bridge_busy_flag():
    """수동 127 입력 잠금이 **브리지의 `busy`** 로 결정되는가 (09 §5.3 ④).

    웹이 `state !== 'IDLE'` 로 흉내내면 단계 이름이 하나 늘 때마다 이 파일도 같이
    고쳐야 하고, 안 고치면 **새 단계에서 수동 입력이 열린 채로 남는다.** 그 상태로
    127 에 쓰면 시퀀스가 자기가 안 쓴 상태 전이를 보고 Abort 한다.
    """
    m = re.search(r'renderSeq\(\)\s*\{(.*?)\n  \}', code_only(JS), re.S)
    assert m, 'renderSeq 를 못 찾았다'
    body = m.group(1)
    assert 'q.busy' in body, 'busy 를 안 쓴다'
    assert "state !== 'IDLE'" not in body and 'state != "IDLE"' not in body, (
        '웹이 busy 를 직접 흉내낸다 — 브리지가 준 판정을 쓸 것')


def test_unknown_sequence_state_locks_rather_than_opens():
    """`sequence` 를 모를 때 **잠근다** — 열어 두면 전개 중일 수도 있는데 겹쳐 쓴다."""
    m = re.search(r'renderSeq\(\)\s*\{(.*?)\n  \}', code_only(JS), re.S)
    body = m.group(1)
    guard = re.search(r'if \(!q\) \{(.*?)\n    \}', body, re.S)
    assert guard, 'sequence 미수신 분기가 없다'
    assert 'setManualSeqEnabled(false' in guard.group(1), (
        '상태를 모를 때 수동 입력을 열어 둔다 — 모르면 안전측(잠금)이 맞다')


def _server_src():
    return open(os.path.join(HERE, 'control_web', 'server.py'), encoding='utf-8').read()


def test_trigger_uses_the_existing_topic_path():
    """전개 시작이 **기존 `/jeongae` 경로**로 가는가.

    브리지에 새 서비스를 파면 같은 동작에 입구가 둘이 되고 게이트가 갈라진다 —
    `auto_mode` 에서 이미 거부한 형태다(03 §7.3).
    """
    assert "'/jeongae'" in _server_src()


def test_jeongae_publisher_is_created_eagerly_not_lazily():
    """**publisher 를 `__init__` 에서 만든다** (2026-08-06 실기에서 물린 자리).

    지연 생성하면 만든 직후의 publish 가 DDS 디스커버리 전이라 구독자에게 안 간다.
    토픽이라 실패도 안 뜬다 — **첫 전개 요청이 통째로 유실되고** 화면에는
    "요청을 보냈다" 만 남는다. 실제로 그렇게 짰다가 실기에서 트리거가 안 먹었다.

    서비스는 `wait_for_service` 가 막아 주므로 지연 생성해도 된다. 토픽만 다르다.
    """
    src = _server_src()
    init = re.search(r'def __init__\(self\):(.*?)\n    # ── ROS', src, re.S)
    assert init, '__init__ 를 못 찾았다'
    assert re.search(r"self\.pub_jeongae = self\.create_publisher", init.group(1)), (
        'jeongae publisher 가 __init__ 에서 안 만들어진다 — 첫 요청이 유실된다')

    trig = re.search(r'def jeongae_trigger\(self\):(.*?)\n    def ', src, re.S)
    assert trig, 'jeongae_trigger 를 못 찾았다'
    assert 'create_publisher' not in trig.group(1), (
        'jeongae_trigger 안에서 publisher 를 만든다 — 첫 호출이 유실된다')


def test_seq_start_button_is_disabled_while_locked():
    """lock 중에는 시작 버튼도 잠근다 — **눌리는데 아무 일도 안 일어나는 버튼**이 가장 나쁘다."""
    m = re.search(r'renderSeq\(\)\s*\{(.*?)\n  \}', code_only(JS), re.S)
    assert re.search(r"pj-seq-start'\)\.disabled = q\.locked", m.group(1)), m.group(1)
