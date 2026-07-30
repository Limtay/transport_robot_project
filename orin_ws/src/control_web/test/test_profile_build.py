"""브라우저 JSON → 프로파일 YAML 조립 (redesign/07 §2 Tab2).

**여기서 지키는 것은 하나다: 웹이 만든 YAML 이 CLI 가 쓰는 것과 같은 문서여야 한다.**
브라우저에서 문자열을 이어 붙여 만들었다면 따옴표·부동소수 표기·들여쓰기가 미묘하게 달라
"웹으로 만든 프로파일만 파싱이 다르다" 가 생긴다. 그래서 구조(JSON)만 받아 PyYAML 로 덤프하고,
그 결과를 다시 파싱해 원래 구조와 같은지 본다.

⚠ **브리지 검증을 흉내내지 않는다.** limits 필수 조건·주파수 상한·slew 위반은 전부
   브리지(rd_profile.cpp)가 정본이고, 여기서 다시 판정하면 두 벌이 되어 갈라진다.
   이 층이 막는 것은 "인자를 안 채우고 보낸 것" 뿐이다.
"""

import yaml

from control_web.profile_run import build_yaml, SEG_ARGS


def spec(**kw):
    base = {'name': 't', 'mode': 'current', 'limits': {'max_abs': 10.0},
            'motors': {'m1': [{'type': 'hold', 'duration': 1.0, 'value': 0.0}]}}
    base.update(kw)
    return base


def test_round_trips_through_yaml():
    text, err = build_yaml(spec())
    assert err is None, err
    d = yaml.safe_load(text)
    assert d['name'] == 't'
    assert d['mode'] == 'current'
    assert d['limits'] == {'max_abs': 10.0}
    assert d['motors']['m1'] == [{'type': 'hold', 'duration': 1.0, 'value': 0.0}]


def test_matches_the_shape_the_bridge_test_profiles_use():
    """브리지 테스트 프로파일(ok_position_range.yaml)과 같은 문서가 나오는가."""
    text, err = build_yaml(spec(
        name='ok_position_range', mode='position',
        limits={'range': [-10.0, 90.0]},
        motors={'m1': [{'type': 'hold', 'duration': 0.5, 'value': 0.0},
                       {'type': 'ramp', 'duration': 1.0, 'from': 0.0, 'to': 45.0}]}))
    assert err is None, err
    assert yaml.safe_load(text) == {
        'name': 'ok_position_range', 'mode': 'position',
        'limits': {'range': [-10.0, 90.0]},
        'motors': {'m1': [{'type': 'hold', 'duration': 0.5, 'value': 0.0},
                          {'type': 'ramp', 'duration': 1.0, 'from': 0.0, 'to': 45.0}]}}


def test_missing_argument_is_refused_with_the_field_name():
    """"거부됨" 만으로는 다음 수가 없다 — 어느 세그의 어느 인자인지 말해야 한다."""
    _, err = build_yaml(spec(motors={'m1': [{'type': 'ramp', 'duration': 1.0, 'from': 0.0}]}))
    assert err is not None
    assert 'to' in err and 'm1' in err and 'ramp' in err


def test_unknown_segment_type_is_refused():
    _, err = build_yaml(spec(motors={'m1': [{'type': 'wobble', 'duration': 1}]}))
    assert err and 'wobble' in err


def test_empty_profile_is_refused():
    assert build_yaml(spec(motors={}))[1]
    assert build_yaml(spec(motors={'m1': []}))[1]


# ★ 기본값을 우리가 적어 넣으면 브리지 기본값과 갈라진다.
def test_optional_args_are_omitted_when_empty():
    text, err = build_yaml(spec(motors={'m1': [
        {'type': 'sine', 'duration': 5, 'amp': 1, 'freq': 2, 'offset': ''}]}))
    assert err is None
    seg = yaml.safe_load(text)['motors']['m1'][0]
    assert 'offset' not in seg, 'offset 을 비웠는데 값이 들어갔다 — 브리지 기본값을 덮는다'

    text, _ = build_yaml(spec(motors={'m1': [
        {'type': 'sine', 'duration': 5, 'amp': 1, 'freq': 2, 'offset': 0.5}]}))
    assert yaml.safe_load(text)['motors']['m1'][0]['offset'] == 0.5


def test_empty_limits_block_is_omitted_not_written_as_null():
    """`limits:` 만 있고 내용이 없으면 브리지가 빈 맵을 보게 된다 — 아예 빼는 편이 낫다."""
    text, err = build_yaml(spec(limits={'max_abs': None, 'range': [], 'slew_rate': ''}))
    assert err is None
    assert 'limits' not in yaml.safe_load(text)


def test_seed_is_carried_only_when_given():
    assert 'seed' not in yaml.safe_load(build_yaml(spec())[0])
    assert yaml.safe_load(build_yaml(spec(seed=42))[0])['seed'] == 42


def test_every_declared_segment_type_can_be_built():
    """SEG_ARGS 에 타입을 추가하고 조립을 안 고치면 여기서 걸린다."""
    sample = {'duration': 1.0, 'value': 0.0, 'from': 0.0, 'to': 1.0, 't_step': 0.5,
              'amp': 1.0, 'freq': 1.0, 'f0': 0.1, 'f1': 1.0, 'low': -1.0, 'high': 1.0,
              'bit_duration': 0.1, 'mean': 0.0, 'std': 0.1, 'step_duration': 0.5,
              'values': [0, 1], 'samples': [0, 1, 0]}
    for t, args in SEG_ARGS.items():
        seg = {'type': t}
        seg.update({a: sample[a] for a in args})
        text, err = build_yaml(spec(motors={'m1': [seg]}))
        assert err is None, '{}: {}'.format(t, err)
        assert yaml.safe_load(text)['motors']['m1'][0]['type'] == t


def test_multiple_motors_are_kept_and_empty_ones_dropped():
    text, err = build_yaml(spec(motors={
        'm1': [{'type': 'hold', 'duration': 1, 'value': 0}],
        'm2': [],
        'm3': [{'type': 'hold', 'duration': 2, 'value': 1}]}))
    assert err is None
    d = yaml.safe_load(text)
    assert set(d['motors']) == {'m1', 'm3'}
