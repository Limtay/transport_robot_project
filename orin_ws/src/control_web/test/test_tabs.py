"""탭 4개 ↔ 브리지 모드 결속 (redesign/09 §5.1, U10).

## 왜 HTML 을 테스트하나

여기서 지키려는 것은 모양이 아니라 **거짓말을 안 하는 UI** 다. 두 가지가 있다.

① **TAB3(project)에 `auto_mode`·`read_preset` 을 두면 안 된다.** 브리지가 project 에서
   그 둘을 무시하기 때문이다 — `InitProject` 는 KINEMATIC 을 강제하고 read 배치는
   `kPresetProject` 고정이다. 고를 수 있는 것처럼 보여 주면 조작자는 고른 값이 적용됐다고
   믿는다. 이건 눈으로 보면 멀쩡해 보이는 종류의 결함이라 테스트가 아니면 안 잡힌다.

② **두 패널의 입력 id 가 겹치면 안 된다.** `getElementById` 는 먼저 나온 것만 집으므로,
   겹치는 순간 **안 보이는 패널의 값이 조용히 나간다.** 화면과 전송값이 갈리는데 화면
   어디에도 흔적이 없다.

브라우저 없이 본다 — 이 둘은 정적 구조의 성질이고, 헤드리스 브라우저를 끌어오면
테스트가 느려지고 CI 에서 흔들린다.
"""

import os
import re
from html.parser import HTMLParser

import pytest

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
INDEX = os.path.join(HERE, 'www', 'index.html')


class _Subtree(HTMLParser):
    """`id=want` 인 요소의 **바깥 HTML** 을 뽑는다.

    정규식으로 `<div id=...>...</div>` 를 잡으려 하면 중첩 div 에서 첫 닫힘에 걸려
    잘린 조각을 보게 된다 — 그러면 "없다" 가 "안 잡혔다" 와 구분되지 않는다.

    깊이는 **대상 요소와 같은 태그 이름만** 센다. 모든 태그를 세면 void 요소
    (`<input>`·`<br>`)와 닫지 않아도 되는 요소(`<p>`·`<option>`)에서 균형이 깨져
    끝을 못 찾고 문서 끝까지 삼킨다 — 실제로 처음 그렇게 짰다가 t4 가 t1 까지
    먹었다. 같은 이름만 세면 그 태그가 균형만 맞으면 정확하다.
    """

    def __init__(self, want):
        super().__init__(convert_charrefs=False)
        self.want, self.tag, self.depth, self.buf, self.done = want, None, 0, [], False

    def handle_starttag(self, tag, attrs):
        if self.done:
            return
        if self.tag is None:
            if dict(attrs).get('id') != self.want:
                return
            self.tag = tag                      # 여기서부터 담는다
        if tag == self.tag:
            self.depth += 1
        self.buf.append(self.get_starttag_text())

    def handle_startendtag(self, tag, attrs):
        if self.tag is not None and not self.done:
            self.buf.append(self.get_starttag_text())

    def handle_endtag(self, tag):
        if self.tag is None or self.done:
            return
        self.buf.append('</%s>' % tag)
        if tag == self.tag:
            self.depth -= 1
            if self.depth == 0:
                self.done = True

    def handle_data(self, data):
        if self.tag is not None and not self.done:
            self.buf.append(data)


def subtree(want):
    p = _Subtree(want)
    p.feed(open(INDEX, encoding='utf-8').read())
    html = ''.join(p.buf)
    assert html, 'id=%s 인 요소가 index.html 에 없다' % want
    return html


def ids_in(html):
    return set(re.findall(r'\bid="([^"]+)"', html))


@pytest.fixture(scope='module')
def doc():
    return open(INDEX, encoding='utf-8').read()


# ── 탭 구성 ────────────────────────────────────────────────────────────────
def test_there_are_exactly_four_tabs_in_order(doc):
    tabs = re.findall(r'<button data-tab="(t\d)"[^>]*>([^<]+)</button>', doc)
    assert [t[0] for t in tabs] == ['t1', 't2', 't3', 't4'], tabs
    labels = ' '.join(t[1] for t in tabs)
    for word in ('실시간', '계획형', '프로젝트', '레지스터'):
        assert word in labels, '%s 탭이 없다 — %s' % (word, labels)


def test_every_tab_button_has_a_pane(doc):
    tabs = re.findall(r'<button data-tab="(t\d)"', doc)
    panes = set(re.findall(r'<div id="(t\d)" class="tabpane', doc))
    assert set(tabs) == panes, '탭 버튼과 pane 이 안 맞는다: %s vs %s' % (tabs, panes)


def test_register_tab_is_never_locked_by_mode(doc):
    """TAB4 는 `data-mode` 가 **없어야** 한다 — 09 §5.1 "TAB4 는 언제나 열린다".

    있으면 게이트가 모드로 잠근다. 브리지가 무슨 모드든 레지스터는 봐야 한다.
    """
    m = re.search(r'<button data-tab="t4"([^>]*)>', doc)
    assert m, 't4 버튼이 없다'
    assert 'data-mode' not in m.group(1), 't4 에 data-mode 가 붙었다 — 모드로 잠기게 된다'


def test_control_tabs_and_project_tab_declare_their_mode(doc):
    got = dict(re.findall(r'<button data-tab="(t\d)"[^>]*data-mode="([^"]+)"', doc))
    assert got == {'t1': 'ctrl', 't2': 'ctrl', 't3': 'project'}, got


# ── ① project 패널이 무시되는 값을 노출하지 않는가 ──────────────────────────
def test_project_panel_offers_nothing_to_select():
    """project 패널에는 `<select>` 가 **하나도 없어야** 한다.

    project 에서 고를 것이 없기 때문이다 — `bridge_mode` 는 고정이고, `auto_mode`(KINEMATIC)
    와 `read_preset`(project)은 브리지가 정하며 파라미터로 준 값을 **무시한다**.

    ⚠ 처음에는 "'auto_mode' 라는 **문자열**이 패널에 없어야 한다" 로 짰다가 **설명 문구에
      걸려 실패했다.** 지키려는 것은 낱말이 아니라 *고를 수단이 없다* 는 것이므로,
      폼 컨트롤의 존재로 판정한다. 설명은 오히려 있어야 좋다.
    """
    html = subtree('bp-proj')
    assert '<select' not in html, (
        'project 기동 패널에 선택 컨트롤이 있다 — 브리지가 project 에서 그 값을 **무시한다**. '
        '고를 수 있는 것처럼 보여 주면 조작자는 적용됐다고 믿는다 (09 §5.1)')


def test_project_panel_has_no_auto_mode_or_preset_inputs():
    """혹시 select 가 아닌 다른 컨트롤로 들어와도 막는다 (id 규약으로 본다)."""
    ids = ids_in(subtree('bp-proj'))
    bad = {i for i in ids if re.search(r'auto|preset|bridge', i)}
    assert not bad, 'project 패널에 있으면 안 되는 입력: %s' % sorted(bad)


def test_project_panel_states_the_mode_is_fixed():
    assert 'project' in subtree('bp-proj'), 'project 고정이라는 표시가 없다'


def test_control_panel_does_expose_auto_mode_and_read_preset():
    """반대 방향 — control 계열에서는 **있어야** 한다.

    위 테스트만 있으면 "둘 다 지웠다" 도 통과한다.
    """
    html = subtree('bp-ctrl')
    assert 'auto_mode' in html and 'read_preset' in html


def test_control_panel_cannot_select_project():
    """control 패널의 bridge_mode 에 project 가 있으면 안 된다.

    있으면 TAB1 에서 project 브리지를 띄울 수 있고, 그 순간 자기가 서 있는 탭이
    게이트에 잠긴다 — 조작자 입장에서는 "시작을 눌렀더니 화면이 튕겼다" 가 된다.
    """
    html = subtree('bp-ctrl')
    m = re.search(r'<select id="c-bridge">(.*?)</select>', html, re.S)
    assert m, 'c-bridge select 가 없다'
    opts = re.findall(r'<option[^>]*>([^<]+)</option>', m.group(1))
    assert 'project' not in opts, opts
    assert set(opts) == {'control', 'manual'}, opts


# ── ② 두 패널의 id 가 겹치지 않는가 ────────────────────────────────────────
def test_the_two_bridge_panels_share_no_input_ids():
    a, b = ids_in(subtree('bp-ctrl')), ids_in(subtree('bp-proj'))
    dup = a & b
    assert not dup, (
        '두 기동 패널이 같은 id 를 쓴다: %s — getElementById 는 먼저 나온 것만 집으므로 '
        '**안 보이는 패널의 값이 조용히 나간다**' % sorted(dup))


def test_no_duplicate_ids_anywhere(doc):
    """문서 전체에서도 id 는 유일해야 한다 (탭을 넷으로 늘리며 복붙이 생기는 자리다)."""
    ids = re.findall(r'\bid="([^"]+)"', doc)
    dup = sorted({i for i in ids if ids.count(i) > 1})
    assert not dup, '중복 id: %s' % dup


# ── 기동 패널이 있어야 할 곳/없어야 할 곳 ──────────────────────────────────
def test_register_tab_has_no_bridge_slot():
    """09 §5.1 — TAB4 에는 기동 패널이 없다.

    모드와 무관하게 열리는 탭이 모드별 패널을 소유할 수는 없다.
    """
    assert 'bridge-slot' not in subtree('t4')


@pytest.mark.parametrize('pane', ['t1', 't2', 't3'])
def test_operational_tabs_have_a_bridge_slot(pane):
    assert 'bridge-slot' in subtree(pane), '%s 에 기동 패널 자리가 없다' % pane
