"""C-7 CLI 인자 파싱 (testbed_spec.md §5.1).

막는 것: 스펙의 커맨드 형태가 파서와 어긋나 실기에서야 발견되는 것.
ROS 없이 파서만 검증한다 (rclpy import 를 피하려 build_parser 만 끌어온다).
"""

import pytest


def parser():
    from control_cli.cli import build_parser
    return build_parser()


def test_spec_command_forms_parse():
    p = parser()
    assert p.parse_args(['status']).cmd == 'status'
    assert p.parse_args(['rearm']).cmd == 'rearm'
    assert p.parse_args(['abort']).cmd == 'abort'

    a = p.parse_args(['config', 'motors', '2', '3'])
    assert (a.cmd, a.what, a.values) == ('config', 'motors', ['2', '3'])

    a = p.parse_args(['config', 'ctr_mode', '2', '3'])
    assert (a.what, a.values) == ('ctr_mode', ['2', '3'])

    a = p.parse_args(['config', 'auto_mode', '2'])
    assert (a.what, a.values) == ('auto_mode', ['2'])

    a = p.parse_args(['run', 'profile.yaml'])
    assert a.profile == 'profile.yaml' and a.record is False

    a = p.parse_args(['run', 'profile.yaml', '--record'])
    assert a.record is True


def test_run_options():
    p = parser()
    a = p.parse_args(['run', 'p.yaml', '--record', '--name', 'exp1', '--bag-dir', '/tmp/b'])
    assert a.name == 'exp1'
    assert a.bag_dir == '/tmp/b'
    # 기본 기록 루트는 스펙의 data/rosbags
    assert p.parse_args(['run', 'p.yaml']).bag_dir == 'data/rosbags'


def test_unknown_config_target_rejected():
    p = parser()
    with pytest.raises(SystemExit):
        p.parse_args(['config', 'bogus', '1'])


def test_subcommand_required():
    p = parser()
    with pytest.raises(SystemExit):
        p.parse_args([])
