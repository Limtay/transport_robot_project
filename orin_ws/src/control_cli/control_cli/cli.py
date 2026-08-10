"""트랙 테스트베드 CLI (testbed_spec.md §5.1).

    control_cli status
    control_cli config motors 2 3          # SET_ACTIVE_MOTORS
    control_cli config ctr_mode 2 3        # SET_CTR_MODE (모터 2 -> mode 3)
    control_cli config auto_mode 2         # SET_AUTO_MODE (§3.3, 1=CURRENT/2=DIRECT)
    control_cli config mode 1              # SET_MODE (ECU AUTO/MANUAL)
    control_cli rearm
    control_cli run profile.yaml [--record]
    control_cli abort

CLI 는 bridge 와 같은 진입점(action/service)만 쓴다 — 웹 UI 와 동일 경로(§5.2).
"""

import argparse
import ast
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import time
from datetime import datetime

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from mgs_tp_msgs.action import RunProfile
from mgs_tp_msgs.srv import ControlConfig
from action_msgs.srv import CancelGoal

from . import record

CONFIG_SRV = '/carrier/control/config'
BRIDGE_NODE = '/firmware_bridge_node'   # node_params 스냅샷 대상 (05 §5.2)


def _parse_param_value(text):
    """`ros2 param get` 의 사람용 출력에서 값만 뽑아 JSON 친화 타입으로.

    array('q', [1]) 같은 파이썬 repr 이 기록에 남으면 나중에 읽는 쪽이 그걸 파싱해야 한다.
    실패하면 **원문 문자열 그대로 남긴다** — 못 읽은 것을 지어내지 않는다.
    """
    text = text.strip()
    m = re.search(r'array\([^,]+,\s*(\[.*\])\s*\)', text)
    if m:
        text = m.group(1)
    if text in ('True', 'False'):
        return text == 'True'
    try:
        return json.loads(text)
    except ValueError:
        pass
    try:
        return ast.literal_eval(text)
    except (ValueError, SyntaxError):
        return text
RUN_ACTION = '/carrier/control/run_profile'
# 기록 관련 상수·헬퍼는 record 모듈이 소유한다 (웹도 같은 것을 쓴다 — 07 §2 Tab2).
# 토픽 목록이 두 벌이 되면 웹으로 돌린 런의 bag 에만 어떤 토픽이 빠지고, 분석 시점에야 드러난다.
RECORD_TOPICS = record.RECORD_TOPICS
DEFAULT_BAG_BASE = record.DEFAULT_BAG_BASE

# ControlConfig.srv 의 op 상수 (숫자 리터럴을 CLI 에 흩뿌리지 않는다)
OP = {
    'motors':    ControlConfig.Request.OP_SET_ACTIVE_MOTORS,
    'ctr_mode':  ControlConfig.Request.OP_SET_CTR_MODE,
    'mode':      ControlConfig.Request.OP_SET_MODE,
    'rearm':     ControlConfig.Request.OP_REARM,
    'status':    ControlConfig.Request.OP_GET_STATUS,
    'auto_mode': ControlConfig.Request.OP_SET_AUTO_MODE,
    'preset':    ControlConfig.Request.OP_SET_READ_PRESET,   # 04 §2.4.2 읽기 프리셋 교체
    'arm':       ControlConfig.Request.OP_SET_STREAM_ARM,    # 01 §6.1.3 웹 run/stop
    'origin':    ControlConfig.Request.OP_SET_ORIGIN,        # 01 §6.3 C-1 펄스
}


class CliNode(Node):
    def __init__(self):
        super().__init__('control_cli')
        self.cfg = self.create_client(ControlConfig, CONFIG_SRV)
        self.act = ActionClient(self, RunProfile, RUN_ACTION)

    def node_params(self):
        """브리지 기동 파라미터 스냅샷 — 구현은 record 가 소유한다 (웹도 같은 것을 쓴다)."""
        return record.bridge_node_params(BRIDGE_NODE)

    def call_config(self, op, motors=(), value=0, timeout=10.0):
        if not self.cfg.wait_for_service(timeout_sec=timeout):
            return None, 'config 서비스({}) 없음 — bridge 가 control_mode 로 떠 있는지 확인'.format(CONFIG_SRV)
        req = ControlConfig.Request()
        req.op = op
        req.motors = list(motors)
        req.value = int(value)
        fut = self.cfg.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if not fut.done():
            return None, '응답 시간초과'
        res = fut.result()
        return res.ok, res.message


def cmd_status(node, args):
    """브리지 상태. 응답은 **정형 JSON** 이다 (04 §4 C3).

    `--json` 은 원문 그대로, 기본은 사람이 읽는 요약이다.
    **요약은 JSON 을 파싱해서 만든다** — 브리지가 사람용 문장을 따로 만들어 주던 종전
    방식으로 돌아가면, 그 문장을 다시 정규식으로 뜯는 소비자가 생긴다 (06 §9.1).
    """
    ok, msg = node.call_config(OP['status'])
    if ok is None:
        print('ERROR: {}'.format(msg), file=sys.stderr)
        return 2
    if getattr(args, 'json', False):
        print(msg)
        return 0 if ok else 1
    try:
        d = json.loads(msg)
    except ValueError:
        print(msg)          # JSON 이 아니면 원문 — 구 브리지와 섞여 돌 때를 위한 대비
        return 0 if ok else 1

    print('state={} write={} bridge={} ecu={}({})'.format(
        d['control_state'], d['write_source'], d['bridge_mode'],
        d['ecu_mode'], d['ecu_sys_state']))
    print('  auto_mode={} write={} preset={} mask=0x{:02X} motors={}'.format(
        d['auto_mode'], d['write_span'], d['read_preset'],
        d['motor_mask'], d['active_motors']))
    print('  ctr_mode={}'.format(d['ctr_mode']))
    # safe_stop 은 **사유까지** 보여준다 — origin 이 왜 거부되는지 여기서 바로 보인다.
    print('  safe_stop={}{}'.format(
        d['safe_stop'], '' if d['safe_stop'] else '  ({})'.format(d['safe_stop_detail'])))
    if d['goal_id']:
        print('  goal_id={} t={:.2f}s'.format(d['goal_id'], d['profile_time']))
    if d['lock_reason']:
        print('  LOCKED: {}'.format(d['lock_reason']))
    # 시간축 등급 — 미수렴이면 stamp 가 fallback 이라 분석이 섞으면 안 된다 (05 §5.2 B1).
    if d['stamp_valid']:
        print('  clock: valid  q={}ms drift={}ppm rtt={}ms'.format(
            d['stamp_quality_ms'], d['drift_ppm'], d['rtt_ms']))
    else:
        print('  clock: **미수렴** — header.stamp 는 Orin 수신 시각 fallback')
    if d['rw_err'] or d['drop_cnt']:
        print('  rw_err=0x{:02X} drop_cnt={}'.format(d['rw_err'], d['drop_cnt']))
    active = [s for s in d['slots'] if s['active']]
    if active:
        print('  slots: ' + ', '.join(
            '#{} {}/{} dur={} att={}'.format(s['index'], s['target'], s['cmd'],
                                             s['duration'], s['attempts'])
            for s in active))
    return 0 if ok else 1


def cmd_rearm(node, _args):
    ok, msg = node.call_config(OP['rearm'])
    if ok is None:
        print('ERROR: {}'.format(msg), file=sys.stderr)
        return 2
    print(('OK: ' if ok else 'FAIL: ') + msg)
    return 0 if ok else 1


def cmd_config(node, args):
    if args.what == 'motors':
        if not args.values:
            print('ERROR: 모터 번호가 필요하다 (예: config motors 2 3)', file=sys.stderr)
            return 2
        ok, msg = node.call_config(OP['motors'], motors=[int(v) for v in args.values])
    elif args.what == 'ctr_mode':
        # 마지막 인자가 mode, 앞의 것들이 대상 모터 (예: config ctr_mode 2 3 1 → m2,m3 를 CURRENT)
        if len(args.values) < 2:
            print('ERROR: config ctr_mode <모터...> <mode> 형식 (예: config ctr_mode 2 3)', file=sys.stderr)
            return 2
        motors = [int(v) for v in args.values[:-1]]
        mode = int(args.values[-1])
        ok, msg = node.call_config(OP['ctr_mode'], motors=motors, value=mode)
    elif args.what == 'preset':
        # 04 §2.4.2 — 읽는 구간을 바꾼다. 슬롯을 쓰지 않으므로 tick 소모가 0 이고,
        # 커맨드 슬롯이 0개인 control 에서도 가능하다.
        # ⚠ 바꾸면 **그 프리셋이 안 읽는 필드는 미판독(0xFF/NaN)으로 발행된다.**
        #    낡은 값이 신선한 값처럼 나가지 않게 하기 위한 것이며, 분석이 그 구간을
        #    "센서 고장" 으로 오해하지 않도록 프리셋 전환 시점을 기록해 둘 것.
        if len(args.values) != 1:
            print('ERROR: config preset <id>  (0=control 기본 / 1=diag)', file=sys.stderr)
            return 2
        ok, msg = node.call_config(OP['preset'], value=int(args.values[0]))
    elif args.what in ('mode', 'auto_mode'):
        if len(args.values) != 1:
            print('ERROR: config {} <값> 형식'.format(args.what), file=sys.stderr)
            return 2
        ok, msg = node.call_config(OP[args.what], value=int(args.values[0]))
    else:
        print('ERROR: 알 수 없는 항목 {}'.format(args.what), file=sys.stderr)
        return 2

    if ok is None:
        print('ERROR: {}'.format(msg), file=sys.stderr)
        return 2
    print(('OK: ' if ok else 'FAIL: ') + msg)
    return 0 if ok else 1


def _require_manual(node):
    """raw 조작 전에 브리지 모드를 확인한다 (C4/B6).

    **게이트 자체는 브리지에 있다** — 클라이언트에만 있는 가드는 가드가 아니다.
    여기 검사는 사유를 먼저, 명확히 보여주기 위한 것이다. 상태를 못 읽으면 막지 않고
    보내 본다 — 브리지가 거부하면 그 사유가 그대로 나온다.
    """
    ok, msg = node.call_config(OP['status'])
    if ok is None:
        return True, None
    try:
        mode = json.loads(msg).get('bridge_mode')
    except ValueError:
        return True, None
    if mode == 'manual':
        return True, None
    return False, mode


def cmd_command(node, args):
    """커맨드 슬롯 SET/RESET — 구 `command_cli`(C++ 대화형) 흡수 (06 Q4/C4).

    별도 실행 파일을 두지 않는 이유: 같은 브리지를 조작하는 도구가 둘이면 조작자가
    어느 쪽을 쓰는지에 따라 가능한 일이 달라진다. **하나로 합치는 것이 결정이었다.**

    B6 이후 **주소 번역은 브리지가 한다.** 이름으로 부른다:

        command set auto 1 ecu read_all          # 예약 구간 뺀 전 범위 216B, 한 트랜잭션
        command set 0 0 ecu read_motor           # forever
        command set auto 1 ecu set_use_lpf 1     # WRITE 계열은 값을 인자로
        command set auto 1 ecu raw_read 16 8     # raw — 전 모드 허용, 쓰기는 정지 상태에서만
        command set auto 1 ecu raw_write 191 1

    `read_*` 는 **모든 bridge_mode 에서 된다** — 읽기는 아무것도 바꾸지 않는다.
    control 에서는 프레임의 양보 tick 으로 나간다 (07 §3.2, 최대 5Hz).
    """
    from mgs_tp_msgs.srv import CommandSet
    R = CommandSet.Request
    # 이름 -> cmd. **브리지 카탈로그와 같은 번호여야 한다** (rd_command_catalog.hpp).
    CMD = {
        'read_sys': R.CMD_READ_SYS, 'read_motor': R.CMD_READ_MOTOR,
        'read_sensor': R.CMD_READ_SENSOR, 'read_diag': R.CMD_READ_DIAG,
        'read_all': R.CMD_READ_ALL,
        'set_soft_estop': R.CMD_SET_SOFT_ESTOP, 'set_use_lpf': R.CMD_SET_USE_LPF,
        'reboot': R.CMD_REBOOT,
        'raw_read': R.CMD_RAW_READ, 'raw_write': R.CMD_RAW_WRITE,
        # DPC 의미 명령 (09 §6) — target 은 `dpc` 여야 한다 (브리지가 표로 검증).
        'dpc_set_boot': R.CMD_DPC_SET_BOOT, 'dpc_set_light': R.CMD_DPC_SET_LIGHT,
        'dpc_set_servo': R.CMD_DPC_SET_SERVO, 'dpc_set_mode': R.CMD_DPC_SET_MODE,
        'dpc_set_seq': R.CMD_DPC_SET_SEQ, 'dpc_read_all': R.CMD_DPC_READ_ALL,
    }
    # ⚠ **raw 의 모드 차단을 제거했다** (09 §5.4 ②, 2026-08-06). 종전에는 여기서
    #   `_require_manual` 로 먼저 걸렀는데, raw 가 전 모드에서 허용되므로 남겨 두면
    #   **브리지는 받는데 CLI 만 거부하는** 상태가 된다 — 클라이언트 가드가 서비스보다
    #   좁으면 그건 가드가 아니라 버그다.
    #   남은 게이트는 `needs_safe_stop`(raw_write) 이고 그 판정은 브리지가 한다.
    cli = node.create_client(CommandSet, '/carrier/command_set')
    if not cli.wait_for_service(timeout_sec=5.0):
        print('ERROR: /carrier/command_set 서비스 없음 — 브리지 실행 확인', file=sys.stderr)
        return 2

    req = CommandSet.Request()
    TARGET = {'ecu': 225, 'dpc': 209, 'pcu': 161}   # CommandSet.srv TARGET_* 와 동일 (dpc=0xD1)

    if args.action == 'reset':
        req.action = 0
        req.slot = int(args.args[0]) if args.args else 0
    else:
        if len(args.args) < 4:
            print('ERROR: command set <slot|auto> <dur> <target> <cmd> [인자...]\n'
                  '       cmd: {}'.format(' '.join(sorted(CMD))), file=sys.stderr)
            return 2
        req.action = 1
        req.slot = 255 if args.args[0] == 'auto' else int(args.args[0])
        req.duration = int(args.args[1])
        tgt = args.args[2].lower()
        if tgt not in TARGET:
            print('ERROR: target 은 ecu|dpc|pcu', file=sys.stderr)
            return 2
        req.target_id = TARGET[tgt]
        name = args.args[3].lower()
        if name not in CMD:
            print('ERROR: cmd 는 {}'.format(' '.join(sorted(CMD))), file=sys.stderr)
            return 2
        req.cmd = CMD[name]
        rest = args.args[4:]
        # ⚠ `dpc_set_*` 도 값 1개를 받는다 — `startswith('set_')` 만 보면 안 걸려서
        #   "인자를 받지 않는다" 로 거부된다 (09 §6).
        if name.startswith('set_') or name.startswith('dpc_set_'):
            if len(rest) != 1:
                print('ERROR: {} <값>'.format(name), file=sys.stderr)
                return 2
            req.args = [int(rest[0]) & 0xFF]
        elif name == 'raw_read':
            if len(rest) != 2:
                print('ERROR: raw_read <addr> <len>', file=sys.stderr)
                return 2
            req.start_addr, req.data_len = int(rest[0]), int(rest[1])
        elif name == 'raw_write':
            if len(rest) < 1:
                print('ERROR: raw_write <addr> [val...] (값이 없으면 브리지 섀도 값을 보낸다)',
                      file=sys.stderr)
                return 2
            req.start_addr = int(rest[0])
            if len(rest) > 1:
                req.data = [int(v) & 0xFF for v in rest[1:]]
                req.data_len = len(req.data)
            else:
                req.data_len = 1
        elif rest:
            print('ERROR: {} 는 인자를 받지 않는다'.format(name), file=sys.stderr)
            return 2

    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
    if not fut.done():
        print('ERROR: 서비스 응답 timeout', file=sys.stderr)
        return 2
    res = fut.result()
    print(('OK: ' if res.accepted else 'FAIL: ') + res.message)
    return 0 if res.accepted else 1


def cmd_arm(node, args):
    """웹의 run/stop 버튼과 **같은 경로**다 (01 §6.1.3).

    웹을 만들기 전에 CLI 로 이 경로를 먼저 검증한다 — 브라우저에서 디버깅하지 않기 위함이다.
    """
    ok, msg = node.call_config(OP['arm'], value=1 if args.on else 0)
    if ok is None:
        print('ERROR: {}'.format(msg), file=sys.stderr)
        return 2
    print(('OK: ' if ok else 'FAIL: ') + msg)
    return 0 if ok else 1


def cmd_origin(node, _args):
    """SET_ORIGIN 펄스 (01 §6.3 C-1).

    auto_mode=DIRECT + safe_stop 이 필요하다. 거부되면 **위반한 조건이 응답에 실린다**.
    결과는 즉시 확인되지 않는다 — 다음 트랜잭션의 fb_position 으로 본다 (`status` 참조).
    """
    ok, msg = node.call_config(OP['origin'])
    if ok is None:
        print('ERROR: {}'.format(msg), file=sys.stderr)
        return 2
    print(('OK: ' if ok else 'FAIL: ') + msg)
    if ok:
        print('  → 결과는 fb_position 으로 확인할 것 (control_cli status)')
    return 0 if ok else 1


def cmd_stream(node, args):
    """CmdMotor 를 주기 발행 — 웹 슬라이더의 CLI 대응물.

    **arm 이 켜져 있어야 소비된다** (01 §6.1.3). arm 없이 보내면 브리지가 받고 버린다 —
    그것이 정상이며, 조작자 모르게 모터가 도는 것을 막는 장치다.
    """
    from mgs_tp_msgs.msg import CmdMotor
    pub = node.create_publisher(CmdMotor, '/carrier/control/cmd_motor', 10)

    msg = CmdMotor()
    vals = [float(v) for v in args.values]
    if len(vals) == 1:
        vals = vals * 4
    if len(vals) != 4:
        print('ERROR: 값은 1개(전 모터 공통) 또는 4개', file=sys.stderr)
        return 2
    field = {'position': 'position', 'velocity': 'velocity', 'current': 'current'}[args.what]
    setattr(msg, field, vals)

    period = 1.0 / args.rate
    deadline = time.time() + args.duration
    n = 0
    print('{} = {} 를 {}Hz 로 {}초간 발행 (Ctrl-C 로 중단)'.format(
        args.what, vals, args.rate, args.duration))
    try:
        while time.time() < deadline:
            msg.header.stamp = node.get_clock().now().to_msg()
            pub.publish(msg)
            n += 1
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    print('발행 {}회 종료 — 브리지는 스테일 timeout 후 IDLE 로 돌아간다'.format(n))
    return 0


def cmd_abort(node, _args):
    """진행 중인 goal 전체 취소.

    UUID·stamp 를 0 으로 두면 action 규약상 '전부 취소'다 — 별도 프로세스인 CLI 는
    goal handle 을 갖고 있지 않으므로 이 경로를 쓴다.
    """
    cli = node.create_client(CancelGoal, RUN_ACTION + '/_action/cancel_goal')
    if not cli.wait_for_service(timeout_sec=10.0):
        print('ERROR: action 서버 없음', file=sys.stderr)
        return 2
    fut = cli.call_async(CancelGoal.Request())
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
    if not fut.done():
        print('ERROR: 응답 시간초과', file=sys.stderr)
        return 2
    n = len(fut.result().goals_canceling)
    print('취소 요청 완료 — 대상 {}건'.format(n) if n else '취소할 goal 이 없다')
    return 0


_start_bag = record.start_bag
_stop_bag  = record.stop_bag


def cmd_run(node, args):
    if not os.path.isfile(args.profile):
        print('ERROR: 프로파일 파일 없음: {}'.format(args.profile), file=sys.stderr)
        return 2
    with open(args.profile) as f:
        profile_text = f.read()

    # 라벨 우선순위: --name > YAML 의 name > 파일명
    name = args.name or record.profile_label(
        profile_text, os.path.splitext(os.path.basename(args.profile))[0])

    if not node.act.wait_for_server(timeout_sec=10.0):
        print('ERROR: action 서버({}) 없음'.format(RUN_ACTION), file=sys.stderr)
        return 2

    run_dir, bag_proc = None, None
    started = datetime.now()
    if args.record:
        run_dir = record.create_run_dir(args.bag_dir, name)
        record.copy_profile(run_dir, profile_text)
        bag_proc = _start_bag(record.bag_path(run_dir), diag=args.diag)
        time.sleep(1.0)   # rosbag2 가 구독을 붙일 시간 — 없으면 초반 샘플이 빠진다
        print('기록 폴더: {}'.format(run_dir))

    goal = RunProfile.Goal()
    goal.name = name
    goal.profile_yaml = profile_text

    last = {'progress': -1.0}

    def on_feedback(fb):
        p = fb.feedback.progress
        if p - last['progress'] >= 0.01:
            last['progress'] = p
            print('\r진행 {:5.1f}%  t={:.1f}s  seg={}'.format(
                p * 100.0, fb.feedback.t, fb.feedback.segment_index), end='', flush=True)

    send_fut = node.act.send_goal_async(goal, feedback_callback=on_feedback)
    rclpy.spin_until_future_complete(node, send_fut, timeout_sec=15.0)
    handle = send_fut.result() if send_fut.done() else None

    if handle is None or not handle.accepted:
        print('\ngoal 거부됨 — bridge 로그에서 사유 확인 (FSM 상태·프로파일 검증·ctr_mode 가드)',
              file=sys.stderr)
        _stop_bag(bag_proc)
        if run_dir:
            # 거부된 런도 기록한다 — 왜 거부됐는지가 데이터다 (05 §5.2).
            # 브리지가 값을 준 적이 없으므로 mode/seed 등은 null 로 남는다.
            res = record.build_result(
                name, os.path.abspath(args.profile), 0, False, 'goal rejected',
                started_at=started, profile_text=profile_text,
                node_params=node.node_params(), bag_dir=record.bag_path(run_dir))
            record.write_result(run_dir, res)
        return 1

    res_fut = handle.get_result_async()
    try:
        rclpy.spin_until_future_complete(node, res_fut)
    except KeyboardInterrupt:
        print('\n중단 요청 — goal 취소 중...')
        handle.cancel_goal_async()
        rclpy.spin_until_future_complete(node, res_fut, timeout_sec=15.0)

    print()
    result = res_fut.result().result if res_fut.done() else None
    _stop_bag(bag_proc)

    if result is None:
        print('ERROR: 결과 수신 실패', file=sys.stderr)
        return 2

    print('{} — {} (goal_id={}, {} tick, write_err={}, clamp={})'.format(
        '완료' if result.success else '실패', result.message, result.goal_id,
        result.ticks_executed, result.write_err_cnt, result.clamp_cnt))
    # 조용히 지나가면 안 되는 것들 — 전부 "성공했는데 데이터가 온전하지 않다" 는 뜻이다.
    # 셋은 서로 다른 사고다: 자리를 뺏김 / 늦음 / 기록을 놓침.
    if result.irregular_tick_cnt:
        print('WARN: 정규 RW 가 대체된 tick {}건 — 그 tick 은 피드백이 없다 '
              '(시계열에 구멍)'.format(result.irregular_tick_cnt), file=sys.stderr)
    if result.late_tick_cnt:
        print('WARN: 주기 초과로 위상이 리셋된 tick {}건 — tick 번호는 안 빠졌지만 '
              '기록된 t(=tick*5ms)와 실제 경과가 그만큼 벌어졌다'.format(result.late_tick_cnt),
              file=sys.stderr)
    if result.drop_cnt:
        print('WARN: 발행 큐 드롭 {}건 — 시계열에 구멍이 있다'.format(result.drop_cnt),
              file=sys.stderr)
    if not result.clock_converged:
        print('WARN: 클럭 추정기 미수렴 — header.stamp 가 Orin 수신 시각 fallback 이다. '
              '수렴한 런과 시간축을 섞지 말 것', file=sys.stderr)

    if run_dir:
        # 05 §5.3 — mode/seed/irregular/drop/clock 은 **브리지만 아는 값**이라
        # action Result 로 받아 그대로 옮긴다. CLI 가 지어낼 수 있는 값이 아니다.
        summary = record.result_from_action(
            name, run_dir, result, started, profile_text,
            node_params=node.node_params())
        record.write_result(run_dir, summary)
        problems = record.verify_run_dir(run_dir)
        if problems:
            # 규격 위반을 조용히 넘기면 분석 단계에서야 발견된다.
            # **sha256 불일치는 경고가 아니라 실패다** — 기록이 거짓이면 그 런은 못 쓴다
            # (05 §6.4). 나머지 누락은 경고로 남기고 런 자체의 성패는 유지한다.
            print('WARN: 기록 폴더 규격 미충족 — {}'.format('; '.join(problems)),
                  file=sys.stderr)
            if any('sha256' in p for p in problems):
                print('ERROR: 프로파일 사본이 재생된 것과 다르다 — 이 런은 분석에 쓸 수 없다',
                      file=sys.stderr)
                return 3
        else:
            print('기록 완료: {}'.format(run_dir))

    return 0 if result.success else 1


def build_parser():
    p = argparse.ArgumentParser(prog='control_cli', description='트랙 테스트베드 CLI (testbed_spec §5.1)')
    sub = p.add_subparsers(dest='cmd', required=True)

    st_p = sub.add_parser('status', help='FSM·motor_mask·ctr_mode·auto_mode 요약')
    st_p.add_argument('--json', action='store_true',
                      help='정형 JSON 원문 (04 §4 C3) — 자동화는 이쪽을 쓸 것')
    sub.add_parser('rearm', help='LOCKED -> IDLE')
    sub.add_parser('abort', help='진행 중 프로파일 취소')

    c = sub.add_parser('config', help='단발 설정 (IDLE 에서만)')
    c.add_argument('what', choices=['motors', 'ctr_mode', 'mode', 'auto_mode', 'preset'],
                   help='preset: 읽기 프리셋 교체 (0=control 기본, 1=diag/hw_error)')
    c.add_argument('values', nargs='*')

    # 웹 run/stop 과 같은 경로 (01 §6.1.3)
    a = sub.add_parser('arm', help='STREAM 진입 게이트 on/off (웹 run/stop)')
    a.add_argument('on', type=lambda v: v.lower() in ('1', 'on', 'true', 'yes'),
                   help='on|off (1|0)')

    sub.add_parser('origin', help='SET_ORIGIN 펄스 — DIRECT + safe_stop 필요 (01 §6.3 C-1)')

    # 구 command_cli 흡수 (06 Q4/C4).
    #
    # ⚠ **이름을 `raw` 에서 `command` 로 되돌린다.** C4 에서 `raw` 로 부른 이유는 "이 명령은
    #    레지스터 주소를 그대로 노출하므로 이름이 위험을 말해야 한다" 였다. B6 을 구현한
    #    지금은 기본형이 **의미 단위**(`read_all`·`read_motor`)이고 그것들은 어느 모드에서든
    #    안전하다. 서브커맨드 이름이 계속 `raw` 면 안전한 읽기까지 위험해 보인다.
    #    위험은 이제 **cmd 이름**이 말한다 — `raw_read`/`raw_write` 가 그것이다.
    #    (09 §5.4 ② 로 모드 제한은 풀렸고, `raw_write` 의 safe_stop 요구는 남는다.)
    #    `raw` 는 폐지 예정 별칭으로 남긴다.
    for _name, _help in (('command', '커맨드 슬롯 SET/RESET — 의미 단위 명령 (B6)'),
                         ('raw',     '(폐지 예정) command 의 구 이름')):
        cm = sub.add_parser(_name, help=_help)
        cm.add_argument('action', choices=['set', 'reset'])
        cm.add_argument('args', nargs='*',
                        help='set <slot|auto> <dur> <target> <cmd> [인자...] — '
                             'cmd: read_sys read_motor read_sensor read_diag read_all '
                             'set_soft_estop set_use_lpf reboot raw_read raw_write')

    st = sub.add_parser('stream', help='CmdMotor 주기 발행 (웹 슬라이더 대응물). arm 필요')
    st.add_argument('what', choices=['position', 'velocity', 'current'])
    st.add_argument('values', nargs='+', help='값 1개(전 모터 공통) 또는 4개')
    st.add_argument('--rate', type=float, default=20.0, help='발행 주기 [Hz] (기본 20)')
    st.add_argument('--duration', type=float, default=5.0, help='발행 지속 [s] (기본 5)')

    r = sub.add_parser('run', help='프로파일 재생')
    r.add_argument('profile')
    r.add_argument('--record', action='store_true',
                   help='bag + YAML 사본 + result 를 실험 폴더로 기록')
    r.add_argument('--name', default=None, help='실험 라벨 (기본: 파일명)')
    r.add_argument('--bag-dir', default=DEFAULT_BAG_BASE, help='기록 루트 (기본: data/rosbags)')
    # 05 §7 — 기본 기록 토픽은 feedback 1개다. comm_diag 는 브리지를
    # `comm_diag_enable:=true` 로 띄웠을 때만 존재하므로 **명시할 때만** 넣는다.
    r.add_argument('--diag', action='store_true',
                   help='comm_diag 토픽도 기록 (브리지를 comm_diag_enable:=true 로 띄운 경우)')
    return p


def main(argv=None):
    args = build_parser().parse_args(argv if argv is not None else sys.argv[1:])
    rclpy.init()
    node = CliNode()
    try:
        return {
            'status': cmd_status, 'rearm': cmd_rearm, 'abort': cmd_abort,
            'config': cmd_config, 'run': cmd_run,
            'arm': cmd_arm, 'origin': cmd_origin, 'stream': cmd_stream,
            'raw': cmd_command,
            'command': cmd_command,
        }[args.cmd](node, args)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
