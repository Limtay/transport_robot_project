"""기동 파라미터 문자열화 (redesign/09 §4.3 · §5.1, U10).

## 왜 이게 테스트할 값어치가 있나

`active_motors` 는 **네 가지 형태가 전부 다른 뜻**이고, 그중 둘은 브리지를 죽인다.
실측(2026-08-06, rcl humble)으로 확인한 표다:

    -p active_motors:=[1]   → 정수 리스트 [1]      : 그 마스크를 ECU 에 **쓴다**
    -p active_motors:=""    → 빈 문자열            : 쓰지 않고 ECU 현재값을 **채택**한다
    -p active_motors:=[]    → NOT_SET              : rclcpp 가 **노드 생성 중 죽는다**(exit 134)
    -p active_motors:=      → rcl 파싱 실패        : "Couldn't parse parameter override rule"
    (파라미터 생략)          → 브리지 기본값 [1,2,3,4] 가 **쓰인다** — 빈칸과 정반대다

`Popen` 에 리스트로 넘기므로 셸이 없다 — 따옴표를 셸이 벗겨 주지 않는다. 그래서
빈칸은 따옴표 **두 글자를 값의 일부로** 넣어야 rcl 이 빈 문자열로 읽는다.

이 구분은 화면으로는 안 보인다. 잘못 보내면 조작자는 "빈칸으로 뒀는데 왜 마스크가
0x0F 가 됐지" 또는 "왜 브리지가 그냥 죽지" 를 보게 되고, 원인은 인자 한 글자다.
"""

import pytest

from control_web.supervisor import _fmt_param


def test_empty_string_becomes_quoted_empty():
    """빈칸의 유일한 정답. 이게 깨지면 브리지가 rcl 파싱에서 죽는다."""
    assert _fmt_param('') == '""'


def test_motor_list_is_bracketed():
    assert _fmt_param([1]) == '[1]'
    assert _fmt_param([2, 3]) == '[2,3]'


def test_empty_list_would_be_the_killer_form():
    """**빈 리스트를 여기 넘기면 안 된다** — 호출자가 빈칸을 `''` 로 바꿔서 준다.

    이 테스트는 `_fmt_param` 을 고치라는 뜻이 아니라, `[]` 가 만들어지는 형태가
    무엇인지 코드에 남겨 두려는 것이다: `-p active_motors:=[]` 는 rclcpp 를
    노드 생성에서 죽인다(try/catch 로도 못 막는다 — rd_node.cpp 주석).
    """
    assert _fmt_param([]) == '[]'          # 형태는 이렇게 나온다 — 그래서 안 보낸다


def test_bools_are_ros_lowercase():
    """`enable_dpc_read` 등 보드 on/off. 파이썬 `True` 를 그대로 쓰면 rcl 이 못 읽는다."""
    assert _fmt_param(True) == 'true'
    assert _fmt_param(False) == 'false'


@pytest.mark.parametrize('v,want', [('control', 'control'), ('project', 'project'), (5, '5')])
def test_plain_values_pass_through(v, want):
    assert _fmt_param(v) == want
