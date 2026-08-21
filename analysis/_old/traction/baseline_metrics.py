#!/usr/bin/env python3
"""기준런 지표 추출 — redesign/06 §4.3 통합 판정용.

재설계 7~9단계 전후를 같은 자로 재기 위한 도구. bag 에서 §4.3 표의 지표를 뽑아
JSON 으로 떨군다. 3회 반복런의 **중앙값**으로 판정한다 (1회 런의 이상치로 되돌리거나
통과시키지 않기 위함 — §4.3).

사용:
    python3 analysis/traction/baseline_metrics.py data/rosbags/BASE_r*  -o ref.json
    python3 analysis/traction/baseline_metrics.py data/rosbags/AFTER_r* --compare ref.json

⚠ 물리 결과(견인력 등)는 비교 대상이 아니다. 구조 개편이 지연·발행·통신을 건드렸는지만 본다.
"""

import argparse
import json
import statistics
import sys
from pathlib import Path

try:
    from rosbags.rosbag2 import Reader
    from rosbags.typesys import Stores, get_typestore, get_types_from_msg
except ImportError:
    print("pip install rosbags 필요", file=sys.stderr)
    sys.exit(2)

# rosbags 0.11 부터 deserialize_cdr 가 typestore 메서드로 옮겨졌다.
# 커스텀 메시지(mgs01_base_msgs)는 스토어에 없으므로 .msg 정의를 읽어 등록한다 —
# 이래야 ROS 환경 없이도(분석 전용 머신) 돌아간다.
_TYPESTORE = get_typestore(Stores.ROS2_HUMBLE)


def _register_custom_types(msg_root: Path) -> None:
    add = {}
    for msg_file in sorted(msg_root.rglob("*.msg")):
        pkg = msg_file.parents[1].name          # <pkg>/msg/Foo.msg
        name = f"{pkg}/msg/{msg_file.stem}"
        if name in _TYPESTORE.types:
            continue
        add.update(get_types_from_msg(msg_file.read_text(), name))
    if add:
        _TYPESTORE.register(add)

# 7단계 개명 (redesign/03 §5.2). 구 이름도 받아 **개명 전후 bag 을 같은 자로 잰다** —
# 그러지 않으면 개명 자체가 비교를 불가능하게 만든다.
FEEDBACK = ("/carrier/control/feedback", "/carrier/testbed/feedback")
LATENCY  = ("/carrier/control/comm_diag", "/carrier/testbed/comm_latency")


def read_run(bag_dir: Path) -> dict:
    """bag 하나에서 §4.3 지표를 뽑는다."""
    root = Path(__file__).resolve().parents[2] / "orin_ws" / "src"
    _register_custom_types(root / "mgs01_base_msgs")
    _register_custom_types(root / "mgs_tp_msgs")
    fb_stamps, rtts, rw_errs, states, goal_ids = [], [], [], [], []
    lat_rtts = []

    with Reader(bag_dir) as reader:
        conns = {c.topic: c for c in reader.connections}
        for conn, ts, raw in reader.messages():
            if conn.topic in FEEDBACK:
                m = _TYPESTORE.deserialize_cdr(raw, conn.msgtype)
                fb_stamps.append(ts * 1e-9)
                # 구 TestbedFeedback 은 rtt 를 들고 있었고, 신 ControlFeedback 은
                # rtt 를 CommDiag 로 넘겼다. 있으면 쓰고 없으면 CommDiag 에 의존한다.
                if hasattr(m, "rtt"):
                    rtts.append(float(m.rtt))
                rw_errs.append(int(m.rw_err))
                states.append(int(getattr(m, "control_state", getattr(m, "testbed_state", 0))))
                goal_ids.append(int(m.goal_id))
            elif conn.topic in LATENCY:
                m = _TYPESTORE.deserialize_cdr(raw, conn.msgtype)
                lat_rtts.append(float(m.rtt))

    if not fb_stamps:
        raise RuntimeError(f"{bag_dir}: {FEEDBACK} 샘플이 없다")

    fb_stamps.sort()
    dt = [b - a for a, b in zip(fb_stamps, fb_stamps[1:])]
    # 200Hz = 5ms. 7.5ms(1.5배) 넘으면 결손으로 센다 — 한 tick 이 통째로 빠진 것.
    dropped = sum(1 for d in dt if d > 0.0075)
    dur = fb_stamps[-1] - fb_stamps[0]

    src = lat_rtts or rtts
    src_sorted = sorted(src)

    def pct(p):
        if not src_sorted:
            return 0.0
        i = min(int(len(src_sorted) * p), len(src_sorted) - 1)
        return src_sorted[i]

    return {
        "bag": bag_dir.name,
        "duration_s": round(dur, 3),
        "feedback_samples": len(fb_stamps),
        "publish_hz": round(len(fb_stamps) / dur, 2) if dur > 0 else 0.0,
        # §4.3 지표
        "missing_ticks": dropped,          # 결손 tick — 기대 0
        "write_err_samples": sum(1 for e in rw_errs if (e >> 4) != 0),
        "read_err_samples": sum(1 for e in rw_errs if (e & 0x0F) != 0),
        "locked_samples": sum(1 for s in states if s == 4),   # TestbedState::LOCKED (STREAM=3)
        "distinct_goal_ids": len(set(g for g in goal_ids if g)),
        "rtt_p50_ms": round(pct(0.50) * 1000, 4),
        "rtt_p99_ms": round(pct(0.99) * 1000, 4),
        "rtt_max_ms": round(max(src) * 1000, 4) if src else 0.0,
        "latency_samples": len(lat_rtts),
    }


def summarize(runs: list) -> dict:
    """3회 반복런의 중앙값 (§4.3)."""
    keys = ["publish_hz", "missing_ticks", "write_err_samples", "read_err_samples",
            "locked_samples", "rtt_p50_ms", "rtt_p99_ms", "rtt_max_ms"]
    return {k: round(statistics.median([r[k] for r in runs]), 4) for k in keys}


def compare(now: dict, ref: dict) -> int:
    """§4.3 판정. 하나라도 미달이면 비-0 반환."""
    print("\n=== §4.3 통합 판정 (중앙값 기준) ===")
    fails = 0

    def check(key, ok, detail):
        nonlocal fails
        mark = "OK " if ok else "FAIL"
        if not ok:
            fails += 1
        print(f"  [{mark}] {key:22s} 기준 {ref[key]:>10}  현재 {now[key]:>10}   {detail}")

    check("missing_ticks", now["missing_ticks"] <= ref["missing_ticks"], "결손 tick — 증가 불가")
    check("write_err_samples", now["write_err_samples"] <= ref["write_err_samples"], "증가는 곧 회귀")
    check("read_err_samples", now["read_err_samples"] <= ref["read_err_samples"], "")
    check("locked_samples", now["locked_samples"] <= ref["locked_samples"], "LOCKED 진입")
    for k in ("rtt_p50_ms", "rtt_p99_ms"):
        lim = ref[k] * 1.20
        check(k, now[k] <= lim, f"+20% 이내 (<= {lim:.4f})")
    # 발행 주기는 양방향으로 본다 — 느려도 빨라도 이상이다
    ok_hz = abs(now["publish_hz"] - ref["publish_hz"]) <= ref["publish_hz"] * 0.02
    check("publish_hz", ok_hz, "±2% 이내")

    print(f"\n  => {'통과' if fails == 0 else f'{fails}개 미달 — 06 §5.2 에 따라 롤백 후 원인 분석'}")
    return fails


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bags", nargs="+", type=Path)
    ap.add_argument("-o", "--out", type=Path, help="결과 JSON 저장")
    ap.add_argument("--compare", type=Path, help="기준 JSON 과 비교 (§4.3 판정)")
    args = ap.parse_args()

    runs = [read_run(b) for b in args.bags]
    for r in runs:
        print(f"{r['bag']}: {r['feedback_samples']:6d} 샘플 / {r['publish_hz']:7.2f} Hz / "
              f"결손 {r['missing_ticks']:3d} / rtt p50 {r['rtt_p50_ms']:.3f}ms p99 {r['rtt_p99_ms']:.3f}ms")

    med = summarize(runs)
    print("\n중앙값:", json.dumps(med, indent=2, ensure_ascii=False))

    if args.out:
        args.out.write_text(json.dumps({"runs": runs, "median": med}, indent=2, ensure_ascii=False))
        print(f"\n저장: {args.out}")

    if args.compare:
        ref = json.loads(args.compare.read_text())["median"]
        return compare(med, ref)
    return 0


if __name__ == "__main__":
    sys.exit(main())
