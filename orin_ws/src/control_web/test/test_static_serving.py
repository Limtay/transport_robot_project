"""정적 파일(www/) 서빙 — **심링크 트리에서도 되는가**, 그리고 경로 탈출은 여전히 막히는가.

## 왜 이 테스트가 있는가

`_static` 의 경로 탈출 가드가 `os.path.realpath` 로 검사하고 있었다. realpath 는
**심링크를 따라가므로**, `colcon build --symlink-install` 로 깐 트리에서는

    install/.../www/index.html  --(symlink)-->  src/control_web/www/index.html

realpath 가 후자를 돌려주고 그건 root 밖이라 가드가 전부 막았다. 결과는
**"API 는 200, UI 페이지는 404"** — 브라우저에는 빈 화면만 뜬다.

이 결함은 **소스만 봐서는 안 보인다.** 코드는 멀쩡하고, 빌드 방식(`--symlink-install`
여부)에 따라 되거나 안 된다. 그래서 심링크가 실제로 있는 임시 트리를 만들어 본다.

## 두 가지를 같이 봐야 한다

심링크를 허용하는 쪽으로 고치면서 탈출 가드를 같이 없애 버리면, 이 파일이 막으려던
보안 구멍(웹이 로봇 위에서 돌고 임의 파일을 읽힌다)이 열린다. 둘을 한 테스트 파일에서
같이 고정한다 — 한쪽만 고치면 다른 쪽이 깨지는 것을 바로 안다.
"""

import json
import os

import pytest

from control_web.server import make_handler


class FakeNode:
    """`_static` 은 노드를 쓰지 않는다 — 핸들러 생성에 필요한 자리만 채운다."""
    pass


class Probe:
    """`make_handler` 가 만든 클래스에서 `_static` 만 떼어 직접 부른다.

    HTTP 서버를 띄우지 않는 이유: 여기서 보려는 것은 **경로 판정**이고, 소켓·포트가
    끼면 테스트가 느려지고 CI 에서 포트 충돌로 흔들린다.
    """

    def __init__(self, www_dir):
        self.cls = make_handler(FakeNode(), www_dir)
        self.status = None
        self.body = None
        self.headers = {}

    def fetch(self, rel):
        probe = self

        class H(self.cls):
            def __init__(self):          # noqa: D107 - BaseHTTPRequestHandler 초기화를 건너뛴다
                pass

            def _json(self, code, obj):  # 404 경로
                probe.status = code
                probe.body = json.dumps(obj).encode()

            def send_response(self, code):
                probe.status = code

            def send_header(self, k, v):
                probe.headers[k] = v

            def end_headers(self):
                pass

            @property
            def wfile(self):
                class W:
                    @staticmethod
                    def write(b):
                        probe.body = b
                return W()

        H()._static(rel)
        return self.status, self.body


@pytest.fixture
def symlinked_www(tmp_path):
    """`--symlink-install` 이 만드는 모양을 재현한다.

        real/index.html          <- 실제 내용
        served/index.html  ->  ../real/index.html   (심링크)

    `served` 를 www_dir 로 주면, realpath 기반 가드는 여기서 404 를 낸다.
    """
    real = tmp_path / 'real'
    real.mkdir()
    (real / 'index.html').write_text('<h1>ok</h1>', encoding='utf-8')
    (real / 'app.js').write_text('console.log(1)', encoding='utf-8')

    served = tmp_path / 'served'
    served.mkdir()
    os.symlink(real / 'index.html', served / 'index.html')
    os.symlink(real / 'app.js', served / 'app.js')

    # 밖에 둔 비밀 파일 — 탈출 시도의 표적
    (tmp_path / 'secret.txt').write_text('do-not-serve', encoding='utf-8')
    return served


# ★ 이것이 핵심 — 심링크로 깔린 파일이 서빙돼야 한다.
def test_serves_files_reached_through_symlinks(symlinked_www):
    status, body = Probe(symlinked_www).fetch('index.html')
    assert status == 200, (
        'symlink 트리에서 404 가 났다 — realpath 로 검사하면 이렇게 된다. '
        '`colcon build --symlink-install` 로 빌드하면 UI 가 통째로 안 뜬다')
    assert b'ok' in body


def test_content_type_follows_extension(symlinked_www):
    p = Probe(symlinked_www)
    p.fetch('index.html')
    assert p.headers['Content-Type'].startswith('text/html')
    p2 = Probe(symlinked_www)
    p2.fetch('app.js')
    assert p2.headers['Content-Type'].startswith('application/javascript')


# ★ 심링크를 허용하면서 **탈출 가드를 잃지 않았는가.**
@pytest.mark.parametrize('attack', [
    '../secret.txt',
    '../../etc/passwd',
    'sub/../../secret.txt',
    '/etc/passwd',                  # lstrip('/') 후에도 절대경로처럼 들어오는 경우
    '....//secret.txt',
])
def test_path_traversal_is_still_refused(symlinked_www, attack):
    status, body = Probe(symlinked_www).fetch(attack)
    assert status == 404, '경로 탈출이 통과했다: %r' % attack
    assert b'do-not-serve' not in (body or b'')


def test_missing_file_is_404_not_an_exception(symlinked_www):
    status, _ = Probe(symlinked_www).fetch('nope.html')
    assert status == 404


def test_directory_is_not_served(symlinked_www):
    """디렉터리를 열면 open() 이 IsADirectoryError 로 터진다 — 404 여야 한다."""
    (symlinked_www / 'sub').mkdir()
    status, _ = Probe(symlinked_www).fetch('sub')
    assert status == 404


# 실제 설치본이 서빙 가능한지 — setup.py 의 data_files 에 파일을 안 적으면
# 개발 트리에서는 되고 **설치본에서만 404** 가 난다 (setup.py 주석의 경고).
def test_every_www_file_in_source_tree_is_declared_in_setup_py():
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    www = os.path.join(here, 'www')
    setup_py = open(os.path.join(here, 'setup.py'), encoding='utf-8').read()
    missing = [f for f in sorted(os.listdir(www))
               if not f.startswith('.') and ('www/' + f) not in setup_py]
    assert not missing, (
        'setup.py 의 data_files 에 없는 www 파일: %s — 개발 트리에서는 되고 '
        '설치본에서만 404 가 난다' % missing)
