"""Refine an installed periodic feedforward from compensated, bidirectional velocity traces."""
import argparse
import hashlib
import json
from pathlib import Path
import re

import numpy as np


ORDERS = np.array([*range(1, 49), 55, 66, 72, 77, 88, 99, 110, 120, 132, 144, 165, 176, 198, 220, 264])
LEAD_S = 130e-6 + 1 / (2 * np.pi * 800)


def basis(phase):
    a = 2 * np.pi * phase[:, None] * ORDERS
    return np.column_stack((np.ones(len(a)), np.cos(a), np.sin(a)))


def load_trace(path, old, metadata):
    raw = path.read_bytes()
    r = json.loads(raw)
    state = r["initial"]
    if (not r["completed"] or r.get("error") or r["cogging"] != "on" or not 0 < abs(r["target"]) <= 0.25 or
            state["pole_pairs"] != metadata["pole_pairs"] or state["direction"] != metadata["direction"] or
            abs(np.angle(np.exp(1j * (state["offset"] - metadata["reference_offset"])))) > .1):
        raise ValueError(f"{path}: requires a completed low-speed trace with the matching map enabled")
    start = r["captures"][0][0]["samples"]
    rows = []
    for batch in r["captures"]:
        if any(((b["samples"]-a["samples"]) & 0xffffffff) != 20 for a, b in zip(batch, batch[1:])):
            raise ValueError(f"{path}: nonconsecutive trace")
        rows.extend(row for row in batch if ((row["samples"]-start) & 0xffffffff)/20000 >= r["settle"])
    position, v, iq = np.array([(x["position"], x["velocity"], x["iq_target"]) for x in rows]).T
    if (not np.isfinite([position, v, iq]).all() or np.ptp(position) < 1 or
            np.max(np.abs(iq)) >= r["current_limit"] * .99):
        raise ValueError(f"{path}: coverage, feedback or saturation invalidates fit")
    phase = (position + v * LEAD_S) % 1
    remaining = iq - np.interp(phase * len(old), np.arange(len(old)), old, period=len(old))
    return phase, remaining, r["target"], hashlib.sha256(raw).hexdigest()


def angle_fit(records, direction):
    blocks = [x for x in records if x[2] * direction > 0]
    if not blocks:
        raise ValueError("Need both directions")
    phase = np.concatenate([x[0] for x in blocks])
    current = np.concatenate([x[1] for x in blocks])
    bins = (phase * 128).astype(int)
    counts = np.bincount(bins, minlength=128)
    if np.count_nonzero(counts) < 120:
        raise ValueError("Need more coverage of mechanical angle in each training direction")
    # Give angular bins equal weight, even when the motor sticks briefly.
    w = 1 / counts[bins]
    w /= w.sum()
    a = basis(phase)
    penalty = np.r_[0., np.tile(.001 * (1 + (ORDERS/100)**2), 2)]
    return np.linalg.solve(a.T @ (a * w[:, None]) + np.diag(penalty), a.T @ (current * w))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--base", type=Path, required=True, help="Exact header compiled into the training firmware")
    p.add_argument("--train", nargs="+", type=Path, required=True)
    p.add_argument("--validate", nargs="+", type=Path, required=True, help="Independent compensated traces")
    p.add_argument("--fraction", type=float, default=.8)
    p.add_argument("--header", type=Path, required=True)
    p.add_argument("--report", type=Path, required=True)
    args = p.parse_args()
    if not 0 < args.fraction <= 1 or args.header.resolve() == args.base.resolve():
        p.error("Choose a fraction in (0,1] and a separate candidate header")
    source = args.base.read_text(encoding="utf8")
    table = re.search(r"(current_100ua\{\{)(.*?)(\}\};)", source, re.S)
    meta = {name: float(re.search(rf"\b{name} = ([-.\d]+)", source)[1])
            for name in ("gain", "direction", "reference_offset", "pole_pairs")}
    old = np.array([int(s) for s in re.findall(r"-?\d+", table[2])]) * (meta["gain"] * 1e-4)
    if len(old) != 1024:
        raise ValueError("Expected 1024 encoder-angle bins")
    train = [load_trace(f, old, meta) for f in args.train]
    test = [load_trace(f, old, meta) for f in args.validate]
    if set(x[3] for x in train) & set(x[3] for x in test) or {np.sign(x[2]) for x in test} != {-1, 1}:
        raise ValueError("Require independent validation in both directions")
    forward, reverse = (angle_fit(train, sign) for sign in (1, -1))
    shared = (forward + reverse) / 2
    shared[0] = 0  # Estimate direction bias separately from the periodic map.
    grid = np.arange(1024) / 1024
    requested = old + (basis(grid) @ shared) * args.fraction
    # Retain the installed peak bound and a zero-mean table after clipping.
    limit = max(abs(old))
    left, right = min(requested), max(requested)
    for _ in range(32):
        shift = (left + right) / 2
        if np.mean(np.clip(requested-shift, -limit, limit)) > 0:
            left = shift
        else:
            right = shift
    codes = np.rint(np.clip(requested-shift, -limit, limit) / (meta["gain"] * 1e-4)).astype(int)
    candidate = codes * (meta["gain"] * 1e-4)
    correction = candidate - old
    metrics = []
    for path, (phase, residual, speed, digest) in zip(args.validate, test):
        predicted = np.interp(phase * len(old), np.arange(len(old)), correction, period=len(old))
        metrics.append(dict(file=path.name, sha256=digest, speed=speed, before_a=float(np.std(residual)),
                            after_a=float(np.std(residual - predicted))))
    if max(abs(correction)) > .12 or max(abs(old + correction)) > .25:
        raise ValueError("Candidate exceeds low-speed feedforward bounds")
    summary = dict(base_sha256=hashlib.sha256(args.base.read_bytes()).hexdigest(),
                   train={f.name:x[3] for f,x in zip(args.train,train)}, validation=metrics,
                   fraction=args.fraction, forward_bias_a=forward[0], reverse_bias_a=reverse[0],
                   direction_odd_a=(forward[0]-reverse[0])/2, direction_even_a=(forward[0]+reverse[0])/2,
                   correction_peak_a=float(max(abs(correction))), correction_rms_a=float(np.std(correction)),
                   candidate_peak_a=float(max(abs(candidate))), candidate_mean_a=float(np.mean(candidate)),
                   dominant_orders=sorted([dict(order=int(h), amplitude_a=float(v)) for h,v in
                        zip(ORDERS, np.hypot(*np.split(shared[1:], 2)))], key=lambda x:-x["amplitude_a"])[:8])
    args.report.write_text(json.dumps(summary, indent=2)+"\n", encoding="utf8")
    print(json.dumps(summary, indent=2))
    if any(x["after_a"] >= x["before_a"] for x in metrics):
        raise ValueError("Held-out feedback did not improve in every validation run")
    rows = "\n".join("    " + ", ".join(map(str, codes[i:i+16])) + "," for i in range(0, 1024, 16))
    refined = source[:table.start(2)] + "\n" + rows + "\n" + source[table.end(2):]
    refined = refined.replace("inline constexpr float reference_offset",
                              "// Bidirectional velocity residual fit, base map SHA256: " +
                              summary["base_sha256"] + "\ninline constexpr float reference_offset", 1)
    args.header.write_text(refined, encoding="utf8")


if __name__ == "__main__":
    main()
