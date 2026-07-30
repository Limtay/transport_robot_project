"""슬라이더 단위 선택이 브리지 규칙(01 §6.1.2)과 일치하는지.

여기가 틀리면 조작자가 **각도 슬라이더로 암페어를 보내게 된다.** 브리지 쪽에서
같은 종류의 결함(DIRECT 분기가 단위를 무시하던 것)을 이미 한 번 겪었으므로,
웹에도 같은 계약을 테스트로 박아 둔다.

인자는 **04 §4 정형 JSON 의 문자열 enum** 이다. 종전에는 브리지의 한국어 문장을
정규식으로 뜯어 정수로 바꿔 썼는데, 그 방식이 웹의 기능 한계를 만들었다 (06 §9.1).
"""
from control_web.server import unit_for


def test_non_direct_modes_ignore_ctr_mode():
    # 비-DIRECT 에서는 ECU 가 ctr_mode 를 강제하므로 auto_mode 만으로 단위가 정해진다.
    for ctr in ('estop', 'current', 'velocity', 'position'):
        assert unit_for('current', ctr) == 'current'
        assert unit_for('velocity', ctr) == 'velocity'
        assert unit_for('position', ctr) == 'position'


def test_direct_follows_per_motor_ctr_mode():
    assert unit_for('direct', 'current') == 'current'
    assert unit_for('direct', 'velocity') == 'velocity'
    assert unit_for('direct', 'position') == 'position'
    # CURRENT_BRAKE 도 전류 단위다 (AcceptsAutoMode 가 current 로 인정하는 값)
    assert unit_for('direct', 'current_brake') == 'current'


def test_direct_with_non_command_ctr_mode_has_no_slider():
    # estop·set_origin 은 조작 대상이 아니다 — 슬라이더를 만들면 안 된다.
    assert unit_for('direct', 'estop') is None
    assert unit_for('direct', 'set_origin') is None


def test_forbidden_auto_modes_have_no_slider():
    # kinematic/control 은 control 모드에서 기동 자체가 막힌다.
    for ctr in ('current', 'position'):
        assert unit_for('kinematic', ctr) is None
        assert unit_for('control', ctr) is None


def test_unknown_values_yield_no_slider():
    """모르는 값에 **슬라이더를 만들지 않는다.**

    "모르면 관대하게" 가 안전한 기본이 아니다 — 단위를 모르는 채로 조작 UI 를 띄우면
    조작자가 어떤 단위로 명령을 보내는지 모른 채 모터를 움직이게 된다.
    """
    assert unit_for(None, None) is None
    assert unit_for('unknown', 'unknown') is None
    assert unit_for('direct', None) is None


# ── 04 §4 응답이 실제로 그대로 쓰이는지 (정규식 파싱이 사라졌는지) ──────────────
def test_server_has_no_regex_status_parser():
    """`_STATUS_RE` 가 되살아나면 이 테스트가 잡는다.

    브리지가 사람용 문장을 돌려주던 시절의 잔재이며, 그 방식으로 돌아가면
    웹이 알 수 있는 값이 다시 넷으로 줄어든다 (06 §9.1).
    """
    import control_web.server as srv
    assert not hasattr(srv, '_STATUS_RE'), '정규식 상태 파서가 되살아났다'


def test_status_defaults_cover_schema_keys():
    """첫 응답 전에도, 그리고 **브리지가 없을 때도** 렌더가 안 깨져야 한다.

    종전에는 `__init__` 의 소스를 문자열로 훑었다. 기본값을 모듈 상수로 빼는 것만으로
    깨지는 검사였고, 정작 값이 무엇인지는 보지 않았다 — 값을 직접 본다.
    """
    from control_web.server import UNKNOWN_STATUS
    for k in ('control_state', 'motor_mask', 'ctr_mode', 'auto_mode',
              'safe_stop', 'stamp_valid', 'lock_reason'):
        assert k in UNKNOWN_STATUS, '기본 상태에 {} 키가 없다'.format(k)


def test_unknown_status_does_not_fake_a_plausible_state():
    """07 §1.1 — 브리지가 없을 때 "모른다" 를 "정상" 처럼 그리면 안 된다.

    `control_state` 가 'IDLE' 이면 조작 UI 가 살아 있는 것처럼 보이고, 조작자는 명령이
    나가는 줄 안다. 모르는 값은 모른다고 적는다.
    """
    from control_web.server import UNKNOWN_STATUS
    assert UNKNOWN_STATUS['control_state'] == '?'
    assert UNKNOWN_STATUS['bridge_mode'] == '?'
    assert UNKNOWN_STATUS['motor_mask'] == 0
    assert UNKNOWN_STATUS['active_motors'] == []
