"""합성 램프 사이클 bag 생성 — `test_pipeline.py` 가 쓴다.

참고곡선(`lib/ref_curve_w40_m2.csv`)을 **진짜 힘**으로 심어 bag 을 만든다. 분석기가 그 곡선을
되찾아내야 정상이다 — 정답을 아는 입력이라 회귀 판정이 가능하다.
"""
import os, sys, shutil
import numpy as np
import rosbag2_py
from rclpy.serialization import serialize_message
from mgs_tp_msgs.msg import ControlFeedback

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'lib'))
import calib

out = sys.argv[1]
shutil.rmtree(out, ignore_errors=True)
w = rosbag2_py.SequentialWriter()
w.open(rosbag2_py.StorageOptions(uri=out, storage_id='sqlite3'),
       rosbag2_py.ConverterOptions('', ''))
w.create_topic(rosbag2_py.TopicMetadata(
    name='/carrier/control/feedback', type='mgs_tp_msgs/msg/ControlFeedback',
    serialization_format='cdr'))

# 참고곡선으로 "진짜" 힘을 만들어 넣는다 — 분석기가 그걸 되찾아내는지 본다
REF = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'lib', 'ref_curve_w40_m2.csv')
ref = np.genfromtxt(REF, delimiter=',', names=True)
DT = 0.005
segs = [(5.0, 'hold', 0.0, 0.0), (37.33, 'ramp', 0.0, 14.0), (3.0, 'hold', 14.0, 14.0),
        (37.33, 'ramp', 14.0, 0.0), (5.0, 'hold', 0.0, 0.0)]
rng = np.random.default_rng(7)
t_abs, tick = 0.0, 0
TARE_CNT = 1027.0
for si, (dur, typ, a, b) in enumerate(segs):
    n = int(round(dur / DT))
    for k in range(n):
        u = k / max(1, n - 1)
        i_cmd = a + (b - a) * u
        rising = si <= 2
        f = np.interp(i_cmd, ref['I_A'], ref['F_rise_N'] if rising else ref['F_fall_N'])
        if not rising:                       # 하강 분기를 반전점에 앵커 (미리보기와 같은 근사)
            f += np.interp(14.0, ref['I_A'], ref['F_rise_N']) - np.interp(14.0, ref['I_A'], ref['F_fall_N'])
        cnt = TARE_CNT + f * calib.CAL['counts_per_N'] + rng.normal(0, 1.2)
        m = ControlFeedback()
        m.header.stamp.sec = int(t_abs); m.header.stamp.nanosec = int((t_abs % 1) * 1e9)
        m.ecu_tick = tick; m.stamp_valid = True
        m.control_state = 2; m.goal_id = 1; m.segment_index = si
        m.profile_time = float(t_abs)
        m.cmd = [float(i_cmd), 0.0, 0.0, 0.0]
        m.fb_current = [float(i_cmd * 0.98), 0.0, 0.0, 0.0]
        m.fb_velocity = [float(rng.normal(0, 0.8)), 0.0, 0.0, 0.0]
        m.fb_position = [float(12.3), 0.0, 0.0, 0.0]
        m.loadcell_raw = [int(round(cnt)), int(round(cnt * 0.63))]
        w.write('/carrier/control/feedback', serialize_message(m), int(t_abs * 1e9))
        t_abs += DT; tick += 50
print('합성 bag 생성:', out)
