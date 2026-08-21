#!/usr/bin/env python3
"""회귀 테스트 — 합성 bag 을 정답으로 삼아 bagio + ramp_analysis 를 검증한다.

  source /opt/ros/humble/setup.bash && source orin_ws/install/setup.bash
  python3 analysis/tests/test_pipeline.py

**왜 이게 있나**: 구 분석기는 토픽 이름만 보고 옛 디코더를 붙여 **크래시 없이 틀린 값**
(lc 상수 −0.1 N, cmd NaN)을 냈다. 조용히 틀리는 실패는 사람이 못 잡는다 — 정답을 아는
입력을 통과시키는 테스트만이 잡는다.
"""
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'lib'))
sys.path.insert(0, os.path.join(HERE, '..', 'tools'))

# 참고곡선에서 읽은 기대값 (합성 bag 에 심은 진짜 값) — 허용오차는 빈 폭·노이즈 기인
EXPECT = {
    'peak_N':          (163.2, 2.0),    # rise@14A
    'slope_N_per_A':   (18.6,  1.0),    # 8~13A 기울기 (07-22 실측 18.6~20)
    'deadband_I0_A':   (1.75,  0.6),    # F>5N 첫 전류
    'gap_at_7A_N':     (30.5,  3.0),    # 하행−상행 @7A — **양수여야 한다**
    'baseline_return_N': (0.0, 1.0),
}
fails = []


def check(name, got, want, tol):
    ok = abs(got - want) <= tol
    print(f'  {"OK  " if ok else "FAIL"} {name:22s} {got:9.2f}  (기대 {want} ±{tol})')
    if not ok:
        fails.append(name)


def main():
    tmp = tempfile.mkdtemp(prefix='analysis_test_')
    bag = os.path.join(tmp, 'synth_run')
    rc = subprocess.run([sys.executable, os.path.join(HERE, '_make_synth_bag.py'), bag],
                        capture_output=True, text=True)
    if rc.returncode != 0:
        print('합성 bag 생성 실패:\n' + rc.stdout + rc.stderr); return 2

    import bagio
    d = bagio.load_bag(bag)
    print('\n[1] bagio — 신 포맷 디코드')
    check('n_samples', d['n'], 17532, 5)
    check('cmd_peak_A', float(d['cmd'][:, 0].max()), 14.0, 0.05)
    check('lc0_유효비율', float((d['lc'][:, 0] == d['lc'][:, 0]).mean()), 1.0, 0.001)

    print('\n[2] bagio — 구 포맷은 거부해야 한다 (조용히 틀리면 안 된다)')
    old = os.path.join(HERE, '..', '..', 'data', 'rosbags', 'BASE_r1_07-28_15-46-14')
    if os.path.isdir(old):
        try:
            bagio.load_bag(old)
            print('  FAIL 구 포맷 bag 을 받아들였다'); fails.append('old_format_rejected')
        except bagio.BagFormatError:
            print('  OK   BagFormatError 로 거부')
    else:
        print('  SKIP 구 포맷 bag 없음')

    print('\n[3] ramp_analysis — 심어 둔 곡선을 되찾는가')
    import ramp_analysis
    r = ramp_analysis.analyze(bag)
    for k, (want, tol) in EXPECT.items():
        check(k, r[k], want, tol)
    print(f'  {"OK  " if r["gap_at_7A_N"] > 0 else "FAIL"} 이력 갭 부호(하행>상행)')
    if r['gap_at_7A_N'] <= 0:
        fails.append('gap_sign')

    print('\n' + ('전부 통과' if not fails else f'실패 {len(fails)}건: {fails}'))
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
