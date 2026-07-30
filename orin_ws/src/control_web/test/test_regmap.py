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

HERE = os.path.dirname(os.path.abspath(__file__))
REGMAP = os.path.join(HERE, '..', 'www', 'regmap.json')
HEADER = os.path.join(HERE, '..', '..', 'orin_firmware_bridge',
                      'include', 'orin_firmware_bridge', 'core', 'rd_register_ecu.hpp')

TYPE_SIZE = {'u8': 1, 'i8': 1, 'u16': 2, 'i16': 2, 'u32': 4, 'i32': 4, 'f32': 4}


def load_map():
    with open(REGMAP) as f:
        return json.load(f)


def header_consts():
    """`constexpr uint16_t REG_X = 12;` 들을 그대로 긁어온다."""
    with open(HEADER) as f:
        src = f.read()
    return {m.group(1): int(m.group(2))
            for m in re.finditer(r'constexpr\s+uint16_t\s+(REG_\w+)\s*=\s*(\d+)\s*;', src)}


def test_header_constants_are_readable():
    c = header_consts()
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
def test_field_bytes_sum_to_block_size():
    for b in load_map()['blocks']:
        total = sum(TYPE_SIZE[f['type']] * f.get('count', 1) for f in b['fields'])
        assert total == b['size'], \
            '{}: 필드 합 {}B != 블록 {}B — 필드를 넣거나 뺀 뒤 크기를 안 고쳤다'.format(
                b['name'], total, b['size'])


def test_fields_are_contiguous_within_block():
    for b in load_map()['blocks']:
        addr = b['addr']
        for f in b['fields']:
            assert f['addr'] == addr, \
                '{}.{}: 주소 {} 인데 {} 여야 한다 (앞 필드에서 이어짐)'.format(
                    b['name'], f['name'], f['addr'], addr)
            addr += TYPE_SIZE[f['type']] * f.get('count', 1)


def test_blocks_tile_the_whole_map_without_gaps_or_overlap():
    m = load_map()
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
