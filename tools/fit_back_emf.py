#!/usr/bin/env python3
"""Fit effective phase-peak back EMF from completed Bench velocity reports.

Uses steady, fully measured current frames and commanded voltage reconstructed
from PWM/bus data. This is an observer model estimate, not a torque calibration.
Does not connect to hardware or write firmware parameters.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import statistics


def plateau(path, resistance, inductance, voltage_delay):
    report = json.loads(path.read_text(encoding="utf8"))
    if not report.get("completed") or not report.get("captures"):
        raise ValueError(f"{path}: completed trace report required")
    origin = report["captures"][0][0]["samples"]
    rows = []
    for s in report["snapshots"]:
        if ((s["samples"]-origin) & 0xFFFFFFFF)/20000 < report["settle"]:
            continue
        if s["fault"] or s["status"] & 0x6F != 0x27:
            raise ValueError(f"{path}: fault or disabled sample")
        if s["warnings"] & 2 or abs(s["velocity"]) < 3:
            continue
        omega = s["velocity"] * math.tau
        electrical = s["position"] * math.tau * s["pole_pairs"] - s["offset"]
        electrical += omega * s["pole_pairs"] * voltage_delay
        alpha = (2*s["voltage_a"]-s["voltage_b"]-s["voltage_c"])/3
        beta = (s["voltage_b"]-s["voltage_c"])/math.sqrt(3)
        vq = -alpha*math.sin(electrical) + beta*math.cos(electrical)
        emf = vq - resistance*s["iq"] - omega*s["pole_pairs"]*inductance*s["id"]
        if not math.isfinite(emf) or not math.isfinite(omega):
            raise ValueError(f"{path}: nonfinite electrical data")
        rows.append((omega, emf))
    if len(rows) < 10:
        raise ValueError(f"{path}: at least 10 steady, fully measured snapshots required")
    return dict(path=path.as_posix(), sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                points=len(rows), omega=statistics.mean(x[0] for x in rows),
                emf=statistics.mean(x[1] for x in rows))


def fit(groups):
    if len(groups) < 4 or not (any(g["omega"] > 0 for g in groups) and
                               any(g["omega"] < 0 for g in groups)):
        raise ValueError("At least four plateaus including both directions are required")
    x = [abs(g["omega"]) for g in groups]
    y = [math.copysign(1, g["omega"])*g["emf"] for g in groups]
    if max(x)-min(x) < math.tau*2:
        raise ValueError("Use at least two distinct speeds separated by 2 rev/s")
    mx, my = statistics.mean(x), statistics.mean(y)
    ke = sum((a-mx)*(b-my) for a, b in zip(x, y))/sum((a-mx)**2 for a in x)
    offset = my-ke*mx
    return dict(ke_v_per_mechanical_rad_s=ke, odd_voltage_offset=offset,
                residual_rms_v=math.sqrt(statistics.mean((ke*a+offset-b)**2 for a, b in zip(x, y))))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("traces", type=Path, nargs="+")
    parser.add_argument("--resistance", type=float, default=1.8)
    parser.add_argument("--inductance", type=float, default=0.0013/3)
    parser.add_argument("--voltage-delay", type=float, default=0.00013)
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args()
    if not all(math.isfinite(v) and v > 0 for v in (args.resistance, args.inductance, args.voltage_delay)):
        parser.error("Electrical parameters and delay must be finite and positive")
    groups = [plateau(p, args.resistance, args.inductance, args.voltage_delay) for p in args.traces]
    report = dict(groups=groups, **fit(groups), resistance_ohm=args.resistance,
                  inductance_h=args.inductance, voltage_angle_delay_s=args.voltage_delay)
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2)+"\n", encoding="utf8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
