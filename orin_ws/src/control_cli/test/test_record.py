"""C-7 실험 폴더 규격 (testbed_spec.md §5.1).

막는 것: run --record 가 만든 폴더가 규격에 안 맞아 나중에 분석 단계에서야 발견되는 것,
        같은 분에 두 번 돌렸을 때 앞 실험 기록이 덮어써지는 것, 이름에 경로 문자가 들어가
        폴더가 깨지거나 상위로 탈출하는 것.
ROS 비의존 — ECU 없이도 폴더 계약을 검증할 수 있어야 한다.
"""

import json
import os
from datetime import datetime

from control_cli import record


def test_sanitize_name_strips_dangerous_chars():
    assert record.sanitize_name('hysteresis_ramp_v1') == 'hysteresis_ramp_v1'
    assert record.sanitize_name('p2 20kg 상승') == 'p2_20kg_상승'
    # 경로 탈출 시도가 폴더명에 남으면 안 된다
    assert '/' not in record.sanitize_name('../../etc/passwd')
    assert '..' not in record.sanitize_name('../evil').strip('_')
    assert record.sanitize_name('') == 'run'
    assert record.sanitize_name('   ') == 'run'
    assert record.sanitize_name('///') == 'run'


def test_run_dir_name_format():
    when = datetime(2026, 7, 19, 14, 5)
    assert record.run_dir_name('TEST3_p2', when) == 'TEST3_p2_07-19_14-05'


def test_create_run_dir_never_overwrites(tmp_path):
    when = datetime(2026, 7, 19, 14, 5)
    a = record.create_run_dir(str(tmp_path), 'exp', when)
    b = record.create_run_dir(str(tmp_path), 'exp', when)
    c = record.create_run_dir(str(tmp_path), 'exp', when)
    assert a != b != c
    assert os.path.isdir(a) and os.path.isdir(b) and os.path.isdir(c)
    assert os.path.basename(b).endswith('_2')
    assert os.path.basename(c).endswith('_3')


def test_bag_path_is_not_precreated(tmp_path):
    # rosbag2 는 대상 경로가 이미 있으면 실패한다 — 미리 만들면 안 된다
    run_dir = record.create_run_dir(str(tmp_path), 'exp')
    bag = record.bag_path(run_dir)
    assert bag.endswith(os.path.join(record.BAG_SUBDIR))
    assert not os.path.exists(bag)


def test_copy_profile_preserves_text_verbatim(tmp_path):
    run_dir = record.create_run_dir(str(tmp_path), 'exp')
    text = 'name: x\nmotors:\n  m2:\n    - {type: hold, duration: 1.0, value: 0}\n'
    path = record.copy_profile(run_dir, text)
    # 재생된 것과 기록된 것이 반드시 같아야 한다
    assert open(path).read() == text


def test_write_result_roundtrip(tmp_path):
    run_dir = record.create_run_dir(str(tmp_path), 'exp')
    res = record.build_result('exp', '/abs/p.yaml', goal_id=3, success=True,
                              message='완료', ticks_executed=9200,
                              write_err_cnt=0, clamp_cnt=12)
    path = record.write_result(run_dir, res)
    loaded = json.load(open(path))
    assert loaded['goal_id'] == 3
    assert loaded['success'] is True
    assert loaded['ticks_executed'] == 9200
    assert loaded['clamp_cnt'] == 12
    assert loaded['name'] == 'exp'
    assert 'started_at' in loaded and 'finished_at' in loaded


def test_failed_run_is_still_recorded(tmp_path):
    # 실패한 실험도 기록한다 — 왜 실패했는지가 데이터다
    run_dir = record.create_run_dir(str(tmp_path), 'exp')
    res = record.build_result('exp', '/abs/p.yaml', 0, False, 'goal rejected')
    record.write_result(run_dir, res)
    loaded = json.load(open(os.path.join(run_dir, record.RESULT_FILE)))
    assert loaded['success'] is False
    assert loaded['message'] == 'goal rejected'


def test_verify_run_dir_reports_each_missing_piece(tmp_path):
    """schema v2 (05 §6.4) 이후로는 **파일 존재만으로 통과하지 않는다.**

    종전 이 테스트는 `build_result(...)` 에 profile_text 를 안 넘기고도 마지막에
    `== []` 를 기대했다 — 그 계약이 곧 "해시 없이도 규격 충족" 이었고, 사본이
    재생된 것과 같은지 아무도 확인하지 않는 상태를 통과시켰다.
    """
    run_dir = record.create_run_dir(str(tmp_path), 'exp')
    assert set(record.verify_run_dir(run_dir)) == {
        record.BAG_SUBDIR, record.PROFILE_COPY, record.RESULT_FILE}

    text = 'x'
    record.copy_profile(run_dir, text)
    assert record.PROFILE_COPY not in record.verify_run_dir(run_dir)

    # 해시 없이 쓰면 bag 누락과 **함께** sha256 미기재도 잡힌다.
    record.write_result(run_dir, record.build_result('e', 'p', 1, True, 'ok'))
    problems = record.verify_run_dir(run_dir)
    assert record.BAG_SUBDIR in problems
    assert any('profile_sha256' in p for p in problems)

    # 해시를 채우고 bag 을 실제 내용까지 갖추면 통과한다.
    record.write_result(run_dir, record.build_result(
        'e', 'p', 1, True, 'ok', profile_text=text, mode=0, seed=1,
        clock_converged=True, drift_ppm=0.0))
    os.makedirs(record.bag_path(run_dir))
    open(os.path.join(record.bag_path(run_dir), 'metadata.yaml'), 'w').close()
    open(os.path.join(record.bag_path(run_dir), 'exp_0.db3'), 'w').close()
    assert record.verify_run_dir(run_dir) == []   # 규격 충족


def test_verify_run_dir_on_missing_dir(tmp_path):
    assert record.verify_run_dir(str(tmp_path / 'nope')) == ['run_dir']


def test_profile_label_prefers_yaml_name():
    assert record.profile_label('name: hysteresis_ramp_v1\nmotors: {}\n', 'fallback') \
        == 'hysteresis_ramp_v1'
    # name 이 없거나 비었으면 파일명으로 물러난다
    assert record.profile_label('motors: {}\n', 'fallback') == 'fallback'
    assert record.profile_label('name: "   "\nmotors: {}\n', 'fallback') == 'fallback'
    assert record.profile_label('name: 42\nmotors: {}\n', 'fallback') == 'fallback'
    # 깨진 YAML 때문에 실행이 막히면 안 된다 (검증은 bridge 몫)
    assert record.profile_label('name: [unclosed\n', 'fallback') == 'fallback'
    assert record.profile_label('', 'fallback') == 'fallback'


# ── schema v2 (05 §5.2, §6.4) ─────────────────────────────────────────────────
#
# 이 검사들이 막는 것: **기록이 거짓인 채로 통과하는 것.** 종전 verify_run_dir 은
# 파일 존재만 봐서, result.json 이 비어 있어도 bag 폴더만 있고 rosbag2 가 실패했어도
# "규격 충족" 을 돌려줬다.

import json
import os

from control_cli import record


def _make_run(tmp_path, profile_text='name: t\nmotors: {}\n', **kw):
    d = record.create_run_dir(str(tmp_path), 'unit')
    record.copy_profile(d, profile_text)
    bag = os.path.join(d, record.BAG_SUBDIR)
    os.makedirs(bag, exist_ok=True)
    open(os.path.join(bag, 'metadata.yaml'), 'w').close()
    open(os.path.join(bag, 'unit_0.db3'), 'w').close()
    res = record.build_result('unit', '/p/unit.yaml', 1, True, '',
                              profile_text=profile_text, **kw)
    record.write_result(d, res)
    return d


def test_v2_has_every_required_key(tmp_path):
    d = _make_run(tmp_path)
    with open(os.path.join(d, record.RESULT_FILE)) as f:
        data = json.load(f)
    for k in record.REQUIRED_KEYS:
        assert k in data, '필수 키 누락: {}'.format(k)
    assert data['schema_version'] == 2


def test_unknown_values_are_null_not_zero(tmp_path):
    """**"안 채운 것" 을 0 으로 위장하지 않는다.** seed=0 은 유효한 시드이고,
    clock_converged=false 는 "미수렴" 이라는 관측이다 — 둘 다 "모른다" 와 다르다."""
    d = _make_run(tmp_path)          # mode/seed/clock 을 안 넘겼다
    with open(os.path.join(d, record.RESULT_FILE)) as f:
        data = json.load(f)
    assert data['seed'] is None
    assert data['mode'] is None
    assert data['clock_converged'] is None
    assert data['drift_ppm'] is None


def test_mode_number_becomes_name(tmp_path):
    d = _make_run(tmp_path, mode=2, seed=42)
    with open(os.path.join(d, record.RESULT_FILE)) as f:
        data = json.load(f)
    assert data['mode'] == 'position'
    assert data['seed'] == 42


def test_clean_run_dir_passes(tmp_path):
    d = _make_run(tmp_path, mode=0, seed=7, clock_converged=True, drift_ppm=-19600.0)
    assert record.verify_run_dir(d) == []


# ★ 이 파일에서 가장 중요한 검사 — 05 §6.4-4
def test_tampered_profile_copy_is_detected(tmp_path):
    """사본을 고치면 잡아야 한다. 이것이 "재생된 것과 기록된 것이 같다" 를
    런 종료 시점에 실제로 검증하는 유일한 지점이다."""
    d = _make_run(tmp_path, mode=0, seed=7, clock_converged=True, drift_ppm=0.0)
    assert record.verify_run_dir(d) == []
    with open(os.path.join(d, record.PROFILE_COPY), 'a') as f:
        f.write('# 한 글자만 덧붙여도 다른 프로파일이다\n')
    problems = record.verify_run_dir(d)
    assert any('sha256' in p for p in problems), problems


def test_missing_sha256_is_reported(tmp_path):
    """해시가 없으면 "일치한다" 고 말할 수 없다 — 통과시키면 안 된다."""
    d = _make_run(tmp_path, mode=0, seed=1, clock_converged=True, drift_ppm=0.0)
    path = os.path.join(d, record.RESULT_FILE)
    with open(path) as f:
        data = json.load(f)
    data['profile_sha256'] = None
    with open(path, 'w') as f:
        json.dump(data, f)
    assert any('profile_sha256' in p for p in record.verify_run_dir(d))


def test_empty_bag_dir_is_detected(tmp_path):
    """폴더만 생기고 rosbag2 가 실패한 경우 (05 §6.4-3)."""
    d = _make_run(tmp_path, mode=0, seed=1, clock_converged=True, drift_ppm=0.0)
    os.remove(os.path.join(d, record.BAG_SUBDIR, 'unit_0.db3'))
    assert any('db3' in p for p in record.verify_run_dir(d))
    os.remove(os.path.join(d, record.BAG_SUBDIR, 'metadata.yaml'))
    assert any('metadata.yaml' in p for p in record.verify_run_dir(d))


def test_unknown_schema_version_is_reported(tmp_path):
    """v1 기록 84런이 이미 있다 — 분석이 분기할 수 있어야 한다."""
    d = _make_run(tmp_path, mode=0, seed=1, clock_converged=True, drift_ppm=0.0)
    path = os.path.join(d, record.RESULT_FILE)
    with open(path) as f:
        data = json.load(f)
    data['schema_version'] = 1
    with open(path, 'w') as f:
        json.dump(data, f)
    assert any('schema_version' in p for p in record.verify_run_dir(d))


def test_missing_required_key_is_reported(tmp_path):
    d = _make_run(tmp_path, mode=0, seed=1, clock_converged=True, drift_ppm=0.0)
    path = os.path.join(d, record.RESULT_FILE)
    with open(path) as f:
        data = json.load(f)
    del data['drop_cnt']
    with open(path, 'w') as f:
        json.dump(data, f)
    assert any('drop_cnt' in p for p in record.verify_run_dir(d))


def test_corrupt_result_json_is_reported(tmp_path):
    d = _make_run(tmp_path, mode=0, seed=1, clock_converged=True, drift_ppm=0.0)
    with open(os.path.join(d, record.RESULT_FILE), 'w') as f:
        f.write('{ 깨진 json')
    assert any('읽을 수 없음' in p for p in record.verify_run_dir(d))


def test_failed_run_is_still_recorded(tmp_path):
    """실패한 런도 기록한다 — 왜 실패했는지가 데이터다 (05 §5.2)."""
    d = record.create_run_dir(str(tmp_path), 'rejected')
    text = 'name: r\n'
    record.copy_profile(d, text)
    res = record.build_result('rejected', '/p/r.yaml', 0, False, 'goal rejected',
                              profile_text=text)
    record.write_result(d, res)
    with open(os.path.join(d, record.RESULT_FILE)) as f:
        data = json.load(f)
    assert data['success'] is False
    assert data['message'] == 'goal rejected'
    assert data['ticks_executed'] == 0
    assert data['profile_sha256'] == record.sha256_text(text)
