#!/usr/bin/env python3
"""Unloaded velocity trial with 1 kHz captures, ramp-down and tracking metrics.

Captures contain 96 consecutive samples; SWD readout leaves gaps between batches.
The report retains raw encoder position to distinguish motion from speed filtering.
"""
import argparse
import json
import math
from pathlib import Path
import statistics
import time

from stlink_bench import Bench, ROOT, finite_between, require_current_feedback
from motor_acceptance import wait_stopped


def validate(args):
    for value, low, high, name in ((args.target, -40, 40, "Velocity"),
                                  (args.seconds, 0.2, 120, "Duration"),
                                  (args.current_limit, 0.02, 1.4, "Current limit"),
                                  (args.acceleration, 0.1, 20, "Acceleration"),
                                  (args.settle, 0, 110, "Settling time")):
        finite_between(value, low, high, name)
    if args.seconds <= max(args.settle, abs(args.target) / args.acceleration + 1):
        raise ValueError("Duration must include the speed ramp and a steady measurement window")


def check_state(state, rows, target, limit):
    speed_bound = max(3, abs(target) * 1.2 + 0.5)
    if (state["fault"] or state["status"] & 0x6F != 0x27 or
            not math.isfinite(state["bus"]) or not 4.5 <= state["bus"] <= 35 or
            not math.isfinite(state["temperature"]) or state["temperature"] >= 100):
        raise RuntimeError("Trace trial faulted, left enabled, or exceeded supply/temperature bounds")
    for r in [state, *rows]:
        if (not all(math.isfinite(r[k]) for k in ("position", "velocity", "iq", "id")) or
                abs(r["velocity"]) > speed_bound or
                math.hypot(r["iq"], r["id"]) > limit * 1.5 + 0.1):
            raise RuntimeError("Trace trial exceeded motion/current bounds")


def summarize(report):
    captures = report["captures"]
    if not captures:
        raise ValueError("No captures")
    origin = captures[0][0]["samples"]
    elapsed = lambda r: ((r["samples"] - origin) & 0xFFFFFFFF) / 20000
    tail = [r for c in captures for r in c if elapsed(r) >= report["settle"]]
    if len(tail) < 96:
        raise ValueError("Insufficient steady samples")
    velocity = [r["velocity"] for r in tail]
    # Independently derive velocity from actual positions over 10 ms, strictly
    # within each contiguous capture. Never differentiate across a SWD gap.
    position_velocity = [(b["position"] - a["position"]) / 0.01
                         for c in captures for a, b in zip(c, c[10:])
                         if elapsed(a) >= report["settle"]]
    states = [r for r in report.get("snapshots", []) if elapsed(r) >= report["settle"]]
    result = dict(steady_samples=len(tail), velocity_mean=statistics.mean(velocity),
                  velocity_std=statistics.pstdev(velocity),
                  velocity_error_rms=math.sqrt(statistics.mean((v-report["target"])**2 for v in velocity)),
                  position_velocity_10ms_std=statistics.pstdev(position_velocity),
                  iq_error_rms=math.sqrt(statistics.mean((r["iq"]-r["iq_target"])**2 for r in tail)),
                  id_rms=math.sqrt(statistics.mean(r["id"]**2 for r in tail)),
                  current_peak=max(math.hypot(r["iq"], r["id"]) for c in captures for r in c),
                  current_limit_fraction=statistics.mean(abs(r["iq_target"]) >= report["current_limit"]*0.99
                                                        for r in tail))
    if states and "voltage_a" in states[0]:
        utilization = [math.sqrt(2*(r["voltage_a"]**2+r["voltage_b"]**2+r["voltage_c"]**2))/r["bus"]
                       for r in states]
        result.update(voltage_utilization_mean=statistics.mean(utilization),
                      voltage_utilization_peak=max(utilization))
    return result


def run_trace(bench, args, report):
    for name in ("roller_trace_request", "roller_trace_ack", "roller_trace_data"):
        if name not in bench.symbols:
            raise RuntimeError("Flash a Bench build with 1 kHz trace support")
    wait_stopped(bench)
    state = bench.snapshot()
    require_current_feedback(state)
    finite_between(state["bus"], 4.5, 35, "Measured bus voltage")
    finite_between(state["temperature"], -40, 99, "Measured MCU temperature")
    if not math.isfinite(state["current_limit"]) or state["current_limit"] < args.current_limit:
        raise ValueError("Requested current exceeds the configured motor limit")
    bench.sdo(0x6072, value=int(1000 * args.current_limit / state["current_limit"]), size=2)
    for index in (0x6083, 0x6084):
        bench.sdo(index, value=args.acceleration, fmt="<f")
    if args.cogging != "keep":
        bench.sdo(0x2100, 1, 0x206E6F63 if args.cogging == "on" else 0x66666F63)
    report["initial"] = bench.enable_velocity()
    refresh = lambda: bench.sdo(0x60FF, value=args.target, fmt="<f")
    start = time.monotonic()
    while time.monotonic() - start < args.seconds:
        refresh()
        rows = bench.capture(refresh)
        report["captures"].append(rows)
        state = bench.snapshot()
        report["snapshots"].append(state)
        check_state(state, rows, args.target, args.current_limit)
    report["summary"] = summarize(report)
    # Decelerate through the same velocity loop before disabling the bridge.
    deadline = time.monotonic() + abs(args.target) / args.acceleration + 4
    consecutive = 0
    while time.monotonic() < deadline:
        bench.sdo(0x60FF, value=0, fmt="<f")
        state = bench.snapshot()
        report["deceleration"].append(state)
        check_state(state, [], args.target, args.current_limit)
        consecutive = consecutive + 1 if abs(state["velocity"]) < 0.03 else 0
        if consecutive >= 5:
            report["completed"] = True
            return
        time.sleep(0.02)
    raise TimeoutError("Motor did not settle during ramp-down")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("target", type=float, help="Velocity in rev/s, -40..40; increase in measured stages")
    parser.add_argument("--seconds", type=float, default=10)
    parser.add_argument("--current-limit", type=float, default=0.3)
    parser.add_argument("--acceleration", type=float, default=2, help="Acceleration and deceleration in rev/s^2")
    parser.add_argument("--settle", type=float, default=5, help="Exclude this many seconds from steady metrics")
    parser.add_argument("--cogging", choices=("keep", "on", "off"), default="keep")
    parser.add_argument("--elf", type=Path, default=ROOT / "build/Bench/canopen_foc.elf")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    validate(args)
    report = dict(target=args.target, current_limit=args.current_limit, acceleration=args.acceleration,
                  settle=max(args.settle, abs(args.target)/args.acceleration + 1), seconds=args.seconds,
                  cogging=args.cogging, captures=[], snapshots=[], deceleration=[], completed=False)
    try:
        with Bench(args.elf) as bench:
            try:
                run_trace(bench, args, report)
            finally:
                bench.stop()
                report["final"] = bench.snapshot()
    except BaseException as error:
        report["completed"] = False
        report["error"] = str(error)
        raise
    finally:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf8")
    print(json.dumps(report.get("summary", {})), flush=True)
    print(f"Saved {len(report['captures'])} captures to {args.output}")


if __name__ == "__main__":
    main()
