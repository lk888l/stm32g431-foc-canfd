#!/usr/bin/env python3
"""Run bounded, unloaded speed/position/MIT trials through the Bench SWD transport.

Requires successful current-zero and electrical calibration. Every trial stops
the drive, and the report distinguishes a rejected preflight from a motion test.
These low-rate snapshots do not measure current-loop bandwidth or cogging harmonics.
"""
import argparse
import json
import math
from pathlib import Path
import statistics
import time
from types import SimpleNamespace

from stlink_bench import Bench, ROOT, finite_between, require_current_feedback, run_motion


def summarize(records):
    if len(records) < 10:
        raise ValueError("At least 10 samples are required to assess a trial")
    tail = [r for r in records if r["seconds"] >= records[-1]["seconds"] * 0.65]
    if not all(math.isfinite(r[k]) for r in records for k in ("position", "velocity", "iq", "id")):
        raise ValueError("Nonfinite telemetry")
    result = dict(samples=len(records), steady_samples=len(tail),
                  velocity_mean=statistics.mean(r["velocity"] for r in tail),
                  velocity_std=statistics.pstdev(r["velocity"] for r in tail),
                  current_peak=max(max(abs(r["iq"]), abs(r["id"])) for r in records))
    if records[0]["operation"] == "velocity":
        target = records[0]["target"]
        result["target"] = target
        result["mean_error"] = result["velocity_mean"] - target
        result["passed"] = (abs(result["mean_error"]) <= max(0.03, abs(target) * 0.2) and
                            result["velocity_std"] <= max(0.05, abs(target) * 0.3))
    else:
        target = records[0].get("resolved_target", records[0]["target"])
        result["target"] = target
        result["position_error_max"] = max(abs(r["position"] - target) for r in tail)
        result["passed"] = (result["position_error_max"] <= 0.02 and
                            max(abs(r["velocity"]) for r in tail) <= 0.1)
    result["passed"] &= all(not r["fault"] and r["status"] & 0x6F == 0x27 for r in records)
    return result


def trials(current_limit):
    common = dict(seconds=5, current_limit=current_limit)
    for target in (0.1, -0.1, 0.5, -0.5):
        yield SimpleNamespace(operation="velocity", target=target, **common)
    for operation, target in (("position", 0.125), ("position", -0.125),
                              ("multi-position", 1.25), ("multi-position", -1.25)):
        yield SimpleNamespace(operation=operation, target=target, relative=True, speed=0.5, **common)
    for position in (0.08, -0.08):
        yield SimpleNamespace(operation="mit", position=position, relative=True, velocity=0,
                              torque=0, kp=1.0, kd=0.08, **common)


def wait_stopped(bench):
    bench.stop()
    consecutive = 0
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline:
        state = bench.snapshot()
        if state["fault"]:
            raise RuntimeError("Drive fault while waiting for standstill")
        consecutive = consecutive + 1 if abs(state["velocity"]) < 0.03 else 0
        if consecutive >= 5:
            return
        time.sleep(0.02)
    raise TimeoutError("Motor did not settle between trials")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--elf", type=Path, default=ROOT / "build/Bench/canopen_foc.elf")
    parser.add_argument("--port", type=int, default=6666)
    parser.add_argument("--node", type=int, choices=range(1, 128), default=1)
    parser.add_argument("--current-limit", type=float, default=0.3)
    parser.add_argument("--output", type=Path, default=ROOT / ".work/motor-acceptance.json")
    args = parser.parse_args(argv)
    try:
        finite_between(args.current_limit, 0.02, 1.4, "Bench current limit")
    except ValueError as error:
        parser.error(str(error))
    report = dict(completed=False, motion_attempted=False, telemetry_collected=False,
                  current_limit=args.current_limit, trials=[])
    try:
        with Bench(args.elf, args.port, args.node) as bench:
            try:
                report["initial"] = bench.snapshot()
                require_current_feedback(report["initial"])
                if report["initial"]["fault"] or report["initial"]["calibration"] != 4:
                    raise RuntimeError("A fault-free electrical calibration is required before acceptance")
                for trial in trials(args.current_limit):
                    wait_stopped(bench)
                    entry = dict(command=vars(trial), records=[])
                    report["trials"].append(entry)
                    # run_motion performs another preflight and applies the current
                    # limit before the 6/7/F handshake. A failed run retains samples.
                    try:
                        report["motion_attempted"] = True
                        run_motion(bench, trial, entry["records"])
                    finally:
                        report["telemetry_collected"] |= bool(entry["records"])
                        bench.stop()
                    entry["summary"] = summarize(entry["records"])
                    print(json.dumps(dict(operation=trial.operation, **entry["summary"])), flush=True)
                    if not entry["summary"]["passed"]:
                        raise RuntimeError(f"{trial.operation} tracking criteria failed; remaining trials skipped")
                report["completed"] = True
            finally:
                bench.stop()
                report["final"] = bench.snapshot()
    except Exception as error:
        report["error"] = str(error)
        raise
    finally:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf8")


if __name__ == "__main__":
    main()
