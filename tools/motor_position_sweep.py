#!/usr/bin/env python3
"""Measure single-turn position holding at equally spaced encoder angles."""
import argparse
import json
import math
from pathlib import Path
import statistics
import time

from motor_acceptance import wait_stopped
from motor_trace import check_state
from stlink_bench import Bench, ROOT, finite_between, position_distance, require_current_feedback


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--elf", type=Path, default=ROOT / "build/Bench/canopen_foc.elf")
    parser.add_argument("--points", type=int, default=16)
    parser.add_argument("--seconds", type=float, default=3)
    parser.add_argument("--current-limit", type=float, default=0.3)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    finite_between(args.points, 4, 64, "Angles")
    finite_between(args.seconds, 2, 15, "Duration per angle")
    finite_between(args.current_limit, 0.02, 1.4, "Current limit")
    report = dict(points=args.points, seconds=args.seconds, current_limit=args.current_limit,
                  completed=False, trials=[])
    try:
        with Bench(args.elf) as bench:
            try:
                bench.stop()
                state = bench.snapshot()
                require_current_feedback(state)
                if args.current_limit > state["current_limit"]:
                    raise ValueError("Requested current exceeds configured motor limit")
                bench.sdo(0x6072, value=int(1000*args.current_limit/state["current_limit"]), size=2)
                for index in (0x6083, 0x6084):
                    bench.sdo(index, value=2, fmt="<f")
                for index in range(args.points):
                    wait_stopped(bench)
                    state, cw = bench.enable_position(index/args.points, single_turn=True, speed=1)
                    target = state["position"] + position_distance(state["position"], index/args.points, True)
                    trial = dict(angle_rev=index/args.points, target=target, captures=[])
                    report["trials"].append(trial)
                    refresh = lambda: bench.sdo(0x6040, value=cw, size=2)
                    start = time.monotonic()
                    while time.monotonic()-start < args.seconds:
                        refresh()
                        rows = bench.capture(refresh)
                        trial["captures"].append(rows)
                        state = bench.snapshot()
                        check_state(state, rows, 1, args.current_limit)
                        if any(abs(r["position"]-target) > 0.6 for r in rows):
                            raise RuntimeError("Position exceeded the single-turn travel bound")
                    last = trial["captures"][-1][-1]["samples"]
                    tail = [r for c in trial["captures"] for r in c
                            if ((last-r["samples"]) & 0xFFFFFFFF) <= 20000]
                    errors = [360*(r["position"]-target) for r in tail]
                    trial["summary"] = dict(samples=len(tail), error_mean_deg=statistics.mean(errors),
                                            error_max_deg=max(map(abs, errors)),
                                            position_peak_to_peak_deg=max(errors)-min(errors),
                                            velocity_rms=math.sqrt(statistics.mean(r["velocity"]**2 for r in tail)))
                    print(json.dumps(dict(angle_deg=index/args.points*360, **trial["summary"])), flush=True)
                    if trial["summary"]["error_max_deg"] > 2 or trial["summary"]["velocity_rms"] > 0.1:
                        raise RuntimeError("Position holding failed (2 degrees / 0.1 rev/s RMS)")
                report["completed"] = True
            finally:
                bench.stop()
                report["final"] = bench.snapshot()
    except BaseException as error:
        report["error"] = str(error)
        raise
    finally:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2)+"\n", encoding="utf8")


if __name__ == "__main__":
    main()
