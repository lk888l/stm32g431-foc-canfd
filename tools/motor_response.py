"""Measure unloaded response and same-PWM-frame current peaks on the guarded Bench firmware."""
import argparse
import json
import math
from pathlib import Path
import statistics
import struct
import time

from motor_acceptance import wait_stopped
from motor_trace import check_state
from stlink_bench import Bench, ROOT, finite_between, require_current_feedback


def mean_spectrum(series):
    import numpy as np
    n = len(series[0])
    window = np.hanning(n)
    freq = np.fft.rfftfreq(n, .001)
    power = np.zeros(len(freq))
    for rpm in series:
        bins = abs(np.fft.rfft((rpm - rpm.mean()) * window))**2 / (n * np.sum(window**2))
        bins[1:(n+1)//2] *= 2
        power += bins / len(series)
    return freq, power


def spectrum(captures):
    import numpy as np
    freq, power = mean_spectrum([np.array([60*r["velocity"] for r in rows]) for rows in captures])
    band = (freq >= 80) & (freq <= 100)
    result = dict(bin_hz=float(freq[1]), band_80_100_rms_rpm=float(np.sqrt(power[band].sum())),
                  band_peak_hz=float(freq[band][power[band].argmax()]),
                  largest_ac_line_hz=float(freq[1:][power[1:].argmax()]))
    # Both position differences end on the very same samples, within each capture.
    counts = [np.array([r["counts"] for r in rows], dtype=np.int64) for rows in captures]
    for interval in (1, 10):
        speeds = [(c[10:]-c[10-interval:len(c)-interval]) * (60000/(16384*interval)) for c in counts]
        f, p = mean_spectrum(speeds)
        result[f"encoder_{interval}ms_band_rms_rpm"] = float(np.sqrt(p[(f >= 80) & (f <= 100)].sum()))
    return result


def settling_ms(errors, tolerance, minimum=50):
    """First sample inside the band through the end, with >=minimum ms observed in band."""
    last_outside = max((i for i, x in enumerate(errors) if abs(x) > tolerance), default=-1)
    first = last_outside + 1
    return first if len(errors) - first >= minimum else None


def edge(rows, name, value):
    return next(i for i in range(1, len(rows)) if rows[i-1][name] != value and rows[i][name] == value)


def step_metrics(rows, test, value):
    if test == "reversal":
        after = rows[edge(rows, "marked", value < 0):]
        # Trailing 10 ms differentiation uses data acquired only after the edge.
        rpm = [(b["counts"]-a["counts"]) * 6000/16384 for a,b in zip(after, after[10:])]
        target = -60 * value
        sign = math.copysign(1, target)
        settled = settling_ms([v-target for v in rpm], 3.)
        return dict(target_rpm=target, overshoot_10ms_rpm=max(0., max(sign*(v-target) for v in rpm)),
                    overshoot_estimator_rpm=max(0., max(sign*(r["velocity"]*60-target) for r in after)),
                    within_3rpm_ms=None if settled is None else settled+10, observed_ms=len(after)-1)
    if test == "position":
        after = rows[edge(rows, "ack", True):]
        wire, = struct.unpack("<f", struct.pack("<f", value))
        increment = math.floor(abs(wire) * 16384 + .5)
        target = after[0]["counts"] + (increment if wire >= 0 else -increment)
        errors = [(r["counts"] - target)*360/16384 for r in after]
        return dict(target_counts=target, max_overshoot_deg=max(0., max(math.copysign(1,value)*e for e in errors)),
                    within_0p1deg_ms=settling_ms(errors,.1), final_error_deg=errors[-1],
                    observed_ms=len(after)-1)
    on = edge(rows, "marked", True)
    off = edge(rows[on:], "marked", False) + on
    baseline = statistics.median(r["counts"] for r in rows[:on])
    errors = [(r["counts"]-baseline)*360/16384 for r in rows[off:]]
    excursion = max(abs((r["counts"]-baseline)*360/16384) for r in rows[on:])
    return dict(command_nm=value, pulse_ms=off-on, pre_pulse_counts=baseline,
                max_excursion_deg=excursion, left_0p1deg=excursion > .1,
                return_within_0p1deg_ms=settling_ms(errors,.1) if excursion > .1 else None, final_error_deg=errors[-1],
                observed_after_release_ms=len(errors)-1)


def run(b, args, report):
    test, value, limit = args.test, args.value, args.current_limit
    if not {"roller_trace_kind", "roller_bench_guard"}.issubset(b.symbols):
        raise RuntimeError("Requires the motion/peak Bench build")
    wait_stopped(b)
    state = b.snapshot()
    require_current_feedback(state)
    finite_between(state["bus"], 4.5, 35, "Bus voltage")
    finite_between(state["temperature"], -40, 99, "MCU temperature")
    finite_between(limit, .02, state["current_limit"], "Motor current limit")
    b.sdo(0x6072, value=int(limit*1000/state["current_limit"]), size=2)
    acc = 10 if test == "reversal" else 4 if test == "position" else 2
    for index in (0x6083, 0x6084):
        b.sdo(index, value=acc, fmt="<f")
    speed_bound = abs(value) if test in ("reversal", "spectrum", "current") else 1
    demand = value

    def service():
        if test == "position":
            b.sdo(0x6040, value=demand, size=2)
        elif test == "mit-pulse":
            b.sdo(0x2003, 3, value=demand, fmt="<f")
        else:
            b.sdo(0x60FF, value=demand, fmt="<f")
        s = b.snapshot()
        report["snapshots"].append(s)
        check_state(s, [], speed_bound, limit)
        if s["encoder_missed"] != state["encoder_missed"]:
            raise RuntimeError("Encoder samples lost during response test")

    if test == "position":
        b.enable_position(0, relative=True, speed=.4)
        demand = 0x0f
        b.sdo(0x6040, value=demand, size=2)
        b.sdo(0x607A, value=value, fmt="<f")
        settle = .25
    elif test == "mit-pulse":
        b.enable_mit(kp=3, kd=.04)
        demand, settle = 0., .6
    else:
        b.enable_velocity()
        settle = abs(value) / acc + 2
    deadline = time.monotonic() + settle
    while time.monotonic() < deadline:
        service()
        time.sleep(.015)
    if test in ("spectrum", "current"):
        for _ in range(args.windows):
            if test == "spectrum":
                report["captures"].append(b.motion_capture(service))
            else:
                result = b.peak_capture(service)
                report["captures"].append(result)
                if max(p["peak"] for p in result["peaks"]) > 1.5*limit+.1:
                    raise RuntimeError("PWM-frame current exceeded the trial peak threshold")
        if test == "spectrum":
            report["summary"] = spectrum(report["captures"])
        else:
            peaks = [p for c in report["captures"] for p in c["peaks"]]
            report["summary"] = dict(frames=sum(p["frames"] for p in peaks), worst=max(peaks,key=lambda p:p["peak"]),
                                     frames_by_mask={m:sum(p["frames"] for p in peaks if p["mask"]==m)
                                                     for m in range(8)})
    else:
        start = time.monotonic()

        def step():
            nonlocal demand
            elapsed = time.monotonic() - start
            if elapsed >= .08:
                if test == "reversal":
                    demand = -value
                elif test == "position":
                    demand = 0x5f
                else:
                    demand = value if elapsed < .22 else 0.
            service()

        rows = b.motion_capture(step)
        report["captures"].append(rows)
        report["summary"] = step_metrics(rows, test, value)
    if test in ("spectrum", "current", "reversal"):
        demand = 0.
        deadline = time.monotonic() + abs(value)/acc + 4
        stopped = 0
        while stopped < 5:
            service()
            stopped = stopped+1 if abs(report["snapshots"][-1]["velocity"]) < .03 else 0
            if time.monotonic() > deadline:
                raise RuntimeError("Ramp-down timed out")
    report["completed"] = True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("test", choices=["spectrum", "current", "reversal", "position", "mit-pulse"])
    parser.add_argument("value", type=float, help="Initial rev/s, relative position in turns, or MIT pulse in Nm")
    parser.add_argument("--current-limit", type=float, default=.3)
    parser.add_argument("--windows", type=int, default=6)
    parser.add_argument("--elf", type=Path, default=ROOT / "build/Bench/canopen_foc.elf")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    bound = {"current":15, "spectrum":15, "reversal":2, "position":.1, "mit-pulse":.005}[args.test]
    finite_between(args.value, -bound, bound, args.test)
    finite_between(args.current_limit, .02, .5, "Current limit")
    if args.value == 0 or not 1 <= args.windows <= 200:
        parser.error("Choose a nonzero setpoint and 1..200 windows")
    if args.test == "spectrum":
        try:
            import numpy
        except ImportError:
            parser.error("Spectrum measurement requires NumPy")
    r = dict(test=args.test, value=args.value, current_limit=args.current_limit,
             captures=[], snapshots=[], completed=False)
    try:
        with Bench(args.elf) as b:
            try:
                run(b, args, r)
            finally:
                b.stop()
                r["final"] = b.snapshot()
    except BaseException as error:
        r["completed"] = False
        r["error"] = str(error)
        raise
    finally:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(r,indent=2)+"\n", encoding="utf8")
    print(json.dumps(r["summary"]))


if __name__ == "__main__":
    main()
