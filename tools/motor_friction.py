"""Compare equal MIT absolute targets with and without a signed approach-direction torque bias."""
import argparse
import json
from pathlib import Path
import statistics
from types import SimpleNamespace

from motor_acceptance import wait_stopped
from stlink_bench import Bench, ROOT, finite_between, run_motion


def measure(bench, phase, sign, bias, trial):
    wait_stopped(bench)
    setup = SimpleNamespace(operation="position", target=phase-sign*.08, relative=False,
                            speed=.5, seconds=1.4, current_limit=.3)
    run_motion(bench, setup, [])
    bench.stop()
    wait_stopped(bench)
    origin = bench.snapshot()["position"]
    target = round(origin-phase) + phase
    if not .035 < sign*(target-origin) < .125:
        raise RuntimeError("Prepositioning did not establish the requested approach")
    trial.update(target=target, origin=origin)
    command = SimpleNamespace(operation="mit", position=target, relative=False, velocity=0,
                              torque=bias, kp=1, kd=.08, seconds=2.6, current_limit=.3)
    rows = trial["records"]
    run_motion(bench, command, rows)
    bench.stop()
    tail = [s for s in rows if s["seconds"] >= rows[-1]["seconds"]-1]
    if len(tail) < 8:
        raise RuntimeError("Too few steady MIT observations")
    error = [(target-s["position"])*360 for s in tail]
    trial.update(mean_error_deg=statistics.mean(error), max_error_deg=max(map(abs, error)),
                 max_speed_tail_rpm=max(abs(s["velocity"])*60 for s in tail))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--elf", type=Path, default=ROOT / "build/Bench/canopen_foc.elf")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--bias", type=float, default=.005, help="Positive magnitude in Nm")
    args = parser.parse_args()
    finite_between(args.bias, .0001, .005, "MIT friction trial bias")
    r = dict(completed=False, trials=[], kp=1, kd=.08, current_limit=.3)
    try:
        with Bench(args.elf) as b:
            try:
                for repeat in range(2):
                    for i, phase in enumerate((.125,.625)):
                        for sign in (1,-1):
                            biases = [0., sign*args.bias]
                            if (repeat + i + int(sign < 0)) % 2:
                                biases.reverse()
                            for bias in biases:
                                trial = dict(repeat=repeat, phase=phase, sign=sign, ff_nm=bias, records=[])
                                r["trials"].append(trial)
                                measure(b, phase, sign, bias, trial)
                                print(phase, sign, bias, trial["max_error_deg"], flush=True)
                r["completed"] = True
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


if __name__ == "__main__":
    main()
