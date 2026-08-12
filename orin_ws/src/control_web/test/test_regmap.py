"""regmap.json 이 C++ 헤더와 갈라지지 않았는가 (redesign/07 §3.3).

`www/regmap.json` 은 `rd_register_ecu.hpp` 의 구조체를 **손으로 옮겨 적은 것**이다.
옮겨 적은 표는 반드시 갈라진다 — 04 §2.5 가 지적한 사고(43 을 손으로 옮겨 적어 payload 와
wire 를 섞었다)와 같은 형태이고, 이 세션에서만 `kMaxRespPayload = 88` 이 같은 이유로
stale 이었던 것을 찾았다.

**그래서 C++ 파서를 만들지 않는다.** 검증에 그만한 것이 필요 없다: 헤더의 `REG_*_OFFSET`
/ `REG_*_SIZE` 상수를 정규식으로 뽑아, 블록의 주소·크기와 대조하고 필드 바이트 합이
블록 크기와 같은지 본다. 현실적으로 일어나는 드리프트(필드 추가·크기 변경·블록 이동)는
전부 여기서 걸린다.
"""

import json
import os
import re

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
CORE = os.path.join(HERE, '..', '..', 'orin_firmware_bridge',
                    'include', 'orin_firmware_bridge', 'core')

# 09 §5.4 ③ (U13) — **보드마다 표가 따로다.** DPC 는 ECU 와 필드 순서·스케일이 다르고
# (SYS 는 hw_reset/fatal/error 역순, proc_delta 없음) 헤더가 공용 매핑을 금지한다.
# 그래서 대조도 보드별로 돈다 — 한쪽만 보면 다른 쪽이 조용히 갈라진다.
MAPS = {
    'ecu': (os.path.join(HERE, '..', 'www', 'regmap.json'),
            os.path.join(CORE, 'rd_register_ecu.hpp')),
    'dpc': (os.path.join(HERE, '..', 'www', 'regmap.dpc.json'),
            os.path.join(CORE, 'rd_register_dpc.hpp')),
}
TARGETS = sorted(MAPS)

CMDSET_SRV = os.path.join(HERE, '..', '..', 'mgs_tp_msgs', 'srv', 'CommandSet.srv')

TYPE_SIZE = {'u8': 1, 'i8': 1, 'u16': 2, 'i16': 2, 'u32': 4, 'i32': 4, 'f32': 4}


def load_map(which='ecu'):
    with open(MAPS[which][0]) as f:
        return json.load(f)


def header_consts(which='ecu'):
    """`constexpr uint16_t REG_X = 12;` 들을 그대로 긁어온다."""
    with open(MAPS[which][1]) as f:
        src = f.read()
    return {m.group(1): int(m.group(2))
            for m in re.finditer(r'constexpr\s+uint16_t\s+(REG_\w+)\s*=\s*(\d+)\s*;', src)}


@pytest.mark.parametrize('which', TARGETS)
def test_header_constants_are_readable(which):
    c = header_consts(which)
    assert c, '헤더에서 REG_* 상수를 하나도 못 뽑았다 — 정규식이 헤더 형식과 어긋났다'
    assert c['REG_TOTAL_SIZE'] == 256


# ★ 핵심 — 블록의 주소·크기가 헤더와 같은가.
def test_blocks_match_header_offsets_and_sizes():
    c = header_consts()
    m = load_map()
    # regmap 의 블록 이름 -> 헤더 상수 접두사
    PREFIX = {
        'DEFINE': 'REG_DEFINE', 'SYS': 'REG_SYS', 'RSVD0': 'REG_RSVD0',
        'LOADCELL': 'REG_LOADCELL', 'IMU': 'REG_IMU', 'ENCODER': 'REG_ENCODER',
        'UART2': 'REG_UART2', 'RC': 'REG_SENSOR_RC', 'MOTOR_DATA': 'REG_MOTOR_DATA',
        'CMD_MOTOR': 'REG_CMD_MOTOR', 'CMD_SYSTEM': 'REG_CMD_SYSTEM',
        'RSVD1': 'REG_RSVD1', 'DIAG': 'REG_DIAG',
    }
    names = [b['name'] for b in m['blocks']]
    assert set(names) == set(PREFIX), '블록 집합이 헤더와 다르다: {}'.format(
        set(names) ^ set(PREFIX))

    for b in m['blocks']:
        p = PREFIX[b['name']]
        assert b['addr'] == c[p + '_OFFSET'], \
            '{} 주소 {} != 헤더 {}'.format(b['name'], b['addr'], c[p + '_OFFSET'])
        assert b['size'] == c[p + '_SIZE'], \
            '{} 크기 {} != 헤더 {}'.format(b['name'], b['size'], c[p + '_SIZE'])


# ★ 필드를 하나 추가·삭제하면 여기서 걸린다.
@pytest.mark.parametrize('which', TARGETS)
def test_field_bytes_sum_to_block_size(which):
    for b in load_map(which)['blocks']:
        total = sum(TYPE_SIZE[f['type']] * f.get('count', 1) for f in b['fields'])
        assert total == b['size'], \
            '{}: 필드 합 {}B != 블록 {}B — 필드를 넣거나 뺀 뒤 크기를 안 고쳤다'.format(
                b['name'], total, b['size'])


@pytest.mark.parametrize('which', TARGETS)
def test_fields_are_contiguous_within_block(which):
    for b in load_map(which)['blocks']:
        addr = b['addr']
        for f in b['fields']:
            assert f['addr'] == addr, \
                '{}.{}: 주소 {} 인데 {} 여야 한다 (앞 필드에서 이어짐)'.format(
                    b['name'], f['name'], f['addr'], addr)
            addr += TYPE_SIZE[f['type']] * f.get('count', 1)


@pytest.mark.parametrize('which', TARGETS)
def test_blocks_tile_the_whole_map_without_gaps_or_overlap(which):
    m = load_map(which)
    blocks = sorted(m['blocks'], key=lambda b: b['addr'])
    addr = 0
    for b in blocks:
        assert b['addr'] == addr, \
            '{} 앞에 구멍이나 겹침이 있다 ({} 여야 하는데 {})'.format(b['name'], addr, b['addr'])
        addr += b['size']
    assert addr == m['total'] == 256, '전체가 256B 를 정확히 덮지 않는다 ({}B)'.format(addr)


# rw 는 Tab3 의 편집 가능 여부를 정한다 — 잘못되면 읽기 전용 자리에 쓰기 칸이 생긴다.
def test_writable_blocks_are_exactly_the_command_regions():
    m = load_map()
    writable = {b['name'] for b in m['blocks'] if b['rw'] == 'w'}
    assert writable == {'CMD_MOTOR', 'CMD_SYSTEM'}, \
        '쓰기 가능 블록이 바뀌었다: {}'.format(writable)
    reserved = {b['name'] for b in m['blocks'] if b['rw'] == 'x'}
    assert reserved == {'RSVD0', 'RSVD1'}
    for b in m['blocks']:
        assert b['rw'] in ('r', 'w', 'x'), b['name']


def test_known_addresses_land_where_the_bridge_expects():
    """브리지가 상수로 들고 있는 주소들이 표에서도 같은 자리인가.

    이 넷은 코드가 직접 주소를 쓰는 자리다 (auto_mode·soft_estop·mode·motor_mask).
    표가 어긋나면 Tab3 에서 엉뚱한 칸을 그 이름으로 보여준다.
    """
    c = header_consts()
    field_at = {}
    for b in load_map()['blocks']:
        for f in b['fields']:
            field_at[f['addr']] = f['name']
    assert field_at[c['REG_AUTO_MODE_OFFSET']] == 'auto_mode'
    assert field_at[c['REG_SOFT_ESTOP_OFFSET']] == 'soft_estop'
    assert field_at[c['REG_MODE_OFFSET']] == 'mode'
    assert field_at[c['REG_MOTOR_MASK_OFFSET']] == 'motor_mask'
    assert field_at[c['REG_USE_LPF_OFFSET']] == 'use_lpf'
    assert field_at[c['REG_PROC_DELTA_OFFSET']] == 'rs485_proc_delta'


def test_enums_and_stale_sentinels_are_declared_where_they_matter():
    """미판독 센티넬(255)이 표에 적혀 있어야 UI 가 회색으로 그린다."""
    stale_fields = []
    for b in load_map()['blocks']:
        for f in b['fields']:
            if f.get('stale') is not None:
                assert f['stale'] == 255, f['name']
                stale_fields.append(f['name'])
    assert 'delta_tick' in stale_fields
    assert 'rs485_proc_delta' in stale_fields


# ── DPC (09 §5.4 ③, U13) ───────────────────────────────────────────────────
#
# ⚠ **ECU 와 같은 함수로 검사하지 않는다.** 블록 이름 → 헤더 상수 대응이 다르고,
#   DPC 헤더는 `공용 매핑 함수를 쓰지 말 것` 이라고 명시적으로 경고한다.
DPC_PREFIX = {
    'DEFINE': 'REG_DEFINE', 'RSVD0': 'REG_RSVD0', 'SYS': 'REG_SYS',
    'SENSOR_DPCA': 'REG_SENSOR_DPCA', 'UART2': 'REG_UART2',
    'SENSOR_DPCB': 'REG_SENSOR_DPCB', 'MOTOR_DATA': 'REG_MOTOR_DATA',
    'RSVD1': 'REG_RSVD1', 'CMD_DPCA': 'REG_CMD_DPCA', 'CMD_DPCB': 'REG_CMD_DPCB',
    'CMD_MOT': 'REG_CMD_MOT', 'RSVD2': 'REG_RSVD2', 'DIAG': 'REG_DIAG',
    'RSVD3': 'REG_RSVD3',
}


def test_dpc_blocks_match_header_offsets_and_sizes():
    c, m = header_consts('dpc'), load_map('dpc')
    names = [b['name'] for b in m['blocks']]
    assert set(names) == set(DPC_PREFIX), '블록 집합이 헤더와 다르다: {}'.format(
        set(names) ^ set(DPC_PREFIX))
    for b in m['blocks']:
        p = DPC_PREFIX[b['name']]
        assert b['addr'] == c[p + '_OFFSET'], \
            '{} 주소 {} != 헤더 {}'.format(b['name'], b['addr'], c[p + '_OFFSET'])
        assert b['size'] == c[p + '_SIZE'], \
            '{} 크기 {} != 헤더 {}'.format(b['name'], b['size'], c[p + '_SIZE'])


def test_dpc_named_addresses_land_where_the_bridge_expects():
    """브리지·웹이 **상수로 들고 있는 주소**가 표에서도 같은 이름인가.

    TAB3 의 버튼들이 이 주소로 나간다 (`project.js` 의 `const A`). 표가 어긋나면
    TAB4 가 같은 바이트를 다른 이름으로 보여 주고, 그 순간 두 탭이 서로 다른 말을 한다.
    """
    c = header_consts('dpc')
    at = {}
    for b in load_map('dpc')['blocks']:
        for f in b['fields']:
            at.setdefault(f['addr'], []).append((b['name'], f['name']))

    def name_at(addr):
        return [n for _, n in at.get(addr, [])]

    assert 'sys_state' in name_at(c['REG_SYS_STATE_OFFSET'])
    assert 'sys_state_target' in name_at(c['REG_SYS_STATE_TARGET_OFFSET'])
    assert 'mode' in name_at(c['REG_MODE_OFFSET'])
    assert 'light_en' in name_at(c['REG_DPCB_LIGHT_EN_OFFSET'])
    assert 'boot_en' in name_at(c['REG_DPCB_BOOT_EN_OFFSET'])
    assert 'servo_cmd' in name_at(c['REG_DPCB_SERVO_CMD_OFFSET'])
    assert 'locker_en' in name_at(c['REG_DPCB_LOCKER_EN_OFFSET'])


def test_dpc_writable_blocks_are_exactly_the_command_regions():
    """rw='w' 가 CMD 영역과 정확히 일치하는가 — 읽기 전용 자리에 쓰기 칸이 생기면 안 된다."""
    m = load_map('dpc')
    writable = {b['name'] for b in m['blocks'] if b['rw'] == 'w'}
    assert writable == {'CMD_DPCA', 'CMD_DPCB', 'CMD_MOT'}, writable
    reserved = {b['name'] for b in m['blocks'] if b['rw'] == 'x'}
    assert reserved == {'RSVD0', 'RSVD1', 'RSVD2', 'RSVD3'}
    for b in m['blocks']:
        assert b['rw'] in ('r', 'w', 'x'), b['name']


def test_dpc_sys_block_is_not_the_ecu_layout():
    """**ECU 와 필드 순서가 다르다** — 헤더가 명시적으로 경고하는 지점이다.

    ECU: hw_error(24) → hw_fatal(25) → hw_reset(26)
    DPC: hw_reset(54) → hw_fatal(55) → hw_error(56)
    복붙으로 ECU 배치를 그대로 옮기면 hw_error 와 hw_reset 이 뒤바뀐 채로 표시된다 —
    값이 둘 다 작은 정수라 **화면만 봐서는 절대 모른다.**
    """
    sys_block = next(b for b in load_map('dpc')['blocks'] if b['name'] == 'SYS')
    order = [f['name'] for f in sys_block['fields']]
    assert order.index('hw_reset') < order.index('hw_fatal') < order.index('hw_error'), order
    assert 'rs485_proc_delta' not in order, 'DPC SYS 에는 proc_delta 가 없다'


def test_dpc_map_is_declared_in_setup_py():
    setup_py = open(os.path.join(HERE, '..', 'setup.py'), encoding='utf-8').read()
    assert "'www/regmap.dpc.json'" in setup_py


def test_raw_write_goes_to_the_selected_board():
    """편집 전송이 **화면에서 고른 보드**로 나가는가 (U13 에서 발견한 결함).

    `TARGET.ecu` 로 박혀 있었다. DPC 표를 띄우고 값을 고치면 같은 주소의 **ECU**
    레지스터로 나간다 — 주소가 같아도 뜻이 전혀 다르다. 예컨대 DPC 126(mode) 을
    고치려던 값이 ECU 126(mode) 에 쓰이고, **화면에는 "OK" 가 뜬다.**
    """
    js = open(os.path.join(HERE, '..', 'www', 'regmap.js'), encoding='utf-8').read()
    body = re.search(r'async sendEdits\(\).*?\n  \}', js, re.S)
    assert body, 'sendEdits 를 못 찾았다'
    code = '\n'.join(re.sub(r'//.*$', '', ln) for ln in body.group(0).splitlines())
    assert 'TARGET[this.target]' in code, '전송 대상이 선택 보드를 따르지 않는다'
    assert 'TARGET.ecu' not in code, 'ECU 로 박혀 있다'


def test_dpc_target_id_is_209_not_the_stale_210():
    """09 §0.2 — ID 변경 때 이 한 줄이 누락돼 있었다. 210 이면 브리지가 거부한다."""
    js = open(os.path.join(HERE, '..', 'www', 'regmap.js'), encoding='utf-8').read()
    m = re.search(r'const TARGET = \{([^}]*)\}', js)
    assert m, 'TARGET 표를 못 찾았다'
    assert re.search(r'dpc:\s*209', m.group(1)), m.group(1)
    assert '210' not in m.group(1), '낡은 210 이 남아 있다'


def test_dpc_sys_state_enum_matches_the_header_names():
    """`sys_state` enum 이 헤더 `SysStateName()` 과 **이름까지** 같은가.

    ⚠ 이 테스트가 없어서 실제로 틀렸다 (U13, 2026-08-07): index 5 를 `DESCEND_3` 으로
    적었는데 실제는 **`WAIT`** 다. 주소 대조만으로는 안 잡힌다 — 주소는 맞고 이름만
    틀리기 때문이다.

    하필 5 는 **"Orin 이 target=ASCEND_1 을 써야 진행하는 지점"** 이라, 이름을 틀리면
    화면이 *전개가 멈춘 이유*를 엉뚱하게 설명한다. 조작자는 "하강 3단계 중" 이라고 읽고
    기다리지만 실제로는 **자기 입력을 기다리는 중**이다.
    """
    src = open(MAPS['dpc'][1], encoding='utf-8').read()
    body = re.search(r'inline const char\* SysStateName.*?\n\}', src, re.S)
    assert body, '헤더에서 SysStateName 을 못 찾았다'
    want = {int(re.search(r'STATE_\w+\s*=\s*(\d+)', src[src.index('STATE_' + n + ' '):]).group(1)): nm
            for n, nm in re.findall(r'case STATE_(\w+):\s*return "(\w+)";', body.group(0))}

    sys_field = next(f for b in load_map('dpc')['blocks'] for f in b['fields']
                     if f['name'] == 'sys_state')
    got = sys_field['enum']
    for idx, name in want.items():
        assert idx < len(got), 'enum 이 짧다 — %d(%s) 가 없다' % (idx, name)
        assert got[idx] == name, (
            'sys_state[%d] 가 %r 인데 헤더는 %r — **이름이 갈라졌다**' % (idx, got[idx], name))


def test_project_js_state_names_match_the_regmap():
    """TAB3(`project.js`)과 TAB4(`regmap.dpc.json`)가 **같은 이름**을 쓰는가.

    두 탭이 같은 바이트를 다른 이름으로 부르면, 어느 쪽이 맞는지 화면만 봐서는 모른다.
    """
    js = open(os.path.join(HERE, '..', 'www', 'project.js'), encoding='utf-8').read()
    m = re.search(r'const DPC_STATE = \[(.*?)\];', js, re.S)
    assert m, 'project.js 의 DPC_STATE 를 못 찾았다'
    got = re.findall(r"'([^']+)'", m.group(1))
    sys_field = next(f for b in load_map('dpc')['blocks'] for f in b['fields']
                     if f['name'] == 'sys_state')
    assert got == sys_field['enum'], 'TAB3 과 TAB4 의 상태 이름이 다르다\n%s\n%s' % (
        got, sys_field['enum'])


def test_oneshot_triggers_are_marked(  ):
    """A14 — 126·127 은 **일회성 소비 트리거**이므로 255 가 정상(소비됨)이다.

    표시 규칙이 없으면 CONSUME 이 켜지는 순간 화면이 정상값 255 를 오류로 그린다.
    """
    oneshot = {f['name']: f.get('oneshot')
               for b in load_map('dpc')['blocks'] for f in b['fields']}
    assert oneshot['mode'] == 255, 'mode 에 oneshot 표시가 없다'
    assert oneshot['sys_state_target'] == 255, 'sys_state_target 에 oneshot 표시가 없다'
    # 일반 CMD 는 붙으면 안 된다 — 얘들은 유지값이라 255 가 정상이 아니다.
    for n in ('light_en', 'boot_en', 'servo_cmd'):
        assert oneshot[n] is None, '%s 에 oneshot 이 잘못 붙었다' % n


# ── 보드별 명령 (2026-08-07) ───────────────────────────────────────────────
def _regmap_js():
    return open(os.path.join(HERE, '..', 'www', 'regmap.js'), encoding='utf-8').read()


def test_every_read_button_name_exists_in_the_cmd_table():
    """`READS` 가 부르는 이름이 **전부 `CMD` 표에 있어야** 한다.

    ⚠ 이게 실제로 깨져 있었다: `dpc_read_all` 이 `CMD` 에 없어 `CMD['dpc_read_all']` 이
    `undefined` 였고, `JSON.stringify` 가 그 키를 **통째로 빼서** 서버의
    `req.get('cmd', 0)` 이 0(=read_sys)으로 떨어졌다. 그러면 브리지가
    "read_sys 는 ECU 전용" 으로 거부하고 **화면에는 엉뚱한 사유가 뜬다.**

    JS 는 없는 키를 조용히 `undefined` 로 주므로 런타임에 안 터진다 — 표로 막는다.
    """
    js = _regmap_js()
    cmd = dict(re.findall(r'(\w+):\s*(\d+)', re.search(r'const CMD = \{(.*?)\};', js, re.S).group(1)))
    reads = re.search(r'const READS = \{(.*?)\};', js, re.S).group(1)
    names = re.findall(r"'([a-z_]+)'", reads)
    assert names, 'READS 를 못 읽었다'
    for n in names:
        assert n in cmd, "READS 의 '%s' 가 CMD 표에 없다 — undefined 가 되어 0 으로 떨어진다" % n

    all_map = re.search(r'const READ_ALL = \{(.*?)\};', js, re.S).group(1)
    for n in re.findall(r"'([a-z_]+)'", all_map):
        assert n in cmd, "READ_ALL 의 '%s' 가 CMD 표에 없다" % n


def test_read_command_numbers_match_the_srv():
    """`CMD` 표의 번호가 `CommandSet.srv` 와 같은가 (손으로 옮겨 적은 표)."""
    srv = open(CMDSET_SRV, encoding='utf-8').read()
    want = dict(re.findall(r'uint8 CMD_(\w+)\s*=\s*(\d+)', srv))
    js = _regmap_js()
    got = dict(re.findall(r'(\w+):\s*(\d+)',
                          re.search(r'const CMD = \{(.*?)\};', js, re.S).group(1)))
    for name, num in got.items():
        key = name.upper()
        assert key in want, 'CommandSet.srv 에 CMD_%s 가 없다' % key
        assert want[key] == num, 'regmap.js CMD.%s=%s 인데 srv 는 %s' % (name, num, want[key])


def test_dpc_read_uses_the_dpc_only_command():
    """DPC 의 의미 단위 READ 는 `dpc_read_all` 하나뿐이다 (09 §6).

    ECU 목록(read_sys 등)을 DPC 에 그대로 그리면 브리지가 target 으로 전부 거부한다.
    """
    js = _regmap_js()
    reads = re.search(r'const READS = \{(.*?)\};', js, re.S).group(1)
    m = re.search(r'dpc:\s*\[(.*?)\]', reads, re.S)
    assert m, 'READS.dpc 가 없다'
    assert re.findall(r"'([a-z_]+)'", m.group(1)) == ['dpc_read_all'], m.group(1)


def test_reboot_follows_the_selected_board():
    """리부트가 **화면에서 고른 보드**로 가는가 (2026-08-07).

    종전에는 `TARGET.ecu` 고정이라 DPC 표를 띄우고 눌러도 **ECU 가 재부팅됐다.**
    브리지 카탈로그의 REBOOT 은 `kTargetAny` 라 원래 보드를 가리지 않는다 —
    막고 있던 것은 웹뿐이었다.
    """
    js = _regmap_js()
    body = re.search(r'async reboot\(\).*?\n  \}', js, re.S)
    assert body, 'reboot 를 못 찾았다'
    code = '\n'.join(re.sub(r'//.*$', '', ln) for ln in body.group(0).splitlines())
    assert 'TARGET[this.target]' in code, '리부트 대상이 선택 보드를 따르지 않는다'
    assert 'TARGET.ecu' not in code, 'ECU 로 박혀 있다'
