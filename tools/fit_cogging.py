#!/usr/bin/env python3
"""Fit a zero-mean periodic current map from a completed bidirectional slow scan.

Requires numpy. Calibration id/iq are in the commanded stator frame; rotate them
into the calibrated rotor frame before separating forward/reverse friction.
The generated map is specific to this motor and encoder mounting.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


def fit(rows):
    if not rows or rows[-1]["calibration"] != 4 or rows[-1]["fault"]:
        raise ValueError("A successfully completed slow calibration is required")
    end = rows[-1]
    poles, direction, offset = end["pole_pairs"], end["direction"], end["offset"]
    selected = [r for r in rows if r["calibration"] == 2 and not r["warnings"] & 3 and
                4000 < r["calibration_ticks"] < 3996000 and
                abs(r["calibration_ticks"] - 2000000) > 4000]
    if len(selected) < 1500:
        raise ValueError("Insufficient valid scan samples")
    p = np.array([r["position"] % 1 for r in selected])
    forward = np.array([r["calibration_ticks"] <= 2000000 for r in selected])
    delta = np.array([r["calibration_angle"] - r["position"] * 2 * np.pi * poles + offset for r in selected])
    current = np.array([r["id"] for r in selected]) * np.sin(delta) + np.array([r["iq"] for r in selected]) * np.cos(delta)
    harmonics = np.array(list(range(1, 97)) + [110, 132, 176, 220, 264])
    phase = 2 * np.pi * p[:, None] * harmonics
    basis = np.column_stack((np.sin(phase), np.cos(phase)))
    model = np.column_stack((np.ones(len(p)), np.where(forward, 1, -1), basis))
    bins = np.floor(p * 128).astype(int)
    counts = [np.bincount(bins[forward == flag], minlength=128) for flag in (True, False)]
    if min(np.count_nonzero(c) for c in counts) < 126:
        raise ValueError("Scan does not cover a complete revolution in both directions")
    weight = 1 / np.maximum(1, np.where(forward, counts[0][bins], counts[1][bins]))
    weight /= weight.mean()
    penalty = np.concatenate(([0, 0], np.tile(1 + (harmonics / 100) ** 4, 2))) * len(p) * 1e-4
    robust = np.ones(len(p))
    for _ in range(4):
        w = weight * robust
        coefficients = np.linalg.solve(model.T @ (model * w[:, None]) + np.diag(penalty),
                                       model.T @ (current * w))
        residual = current - model @ coefficients
        robust = np.minimum(1, 0.025 / np.maximum(np.abs(residual), 1e-8))
    angles = np.arange(1024) / 1024
    grid_phase = 2 * np.pi * angles[:, None] * harmonics
    grid = np.column_stack((np.sin(grid_phase), np.cos(grid_phase)))
    table = grid @ coefficients[2:]
    amplitudes = np.hypot(coefficients[2:2 + len(harmonics)], coefficients[2 + len(harmonics):])
    summary = dict(samples=len(p), pole_pairs=poles, direction=direction, electrical_offset=offset,
                   forward_samples=int(forward.sum()), reverse_samples=int((~forward).sum()),
                   bias_a=float(coefficients[0]), friction_a=float(coefficients[1]),
                   fit_rms_a=float(np.sqrt(np.mean(residual ** 2))),
                   map_rms_a=float(np.sqrt(np.mean(table ** 2))), map_peak_a=float(np.max(np.abs(table))),
                   harmonics=sorted([dict(order=int(h), amplitude_a=float(a)) for h, a in zip(harmonics, amplitudes)],
                                    key=lambda v: -v["amplitude_a"])[:15])
    if not np.isfinite(table).all() or summary["map_peak_a"] > 0.3 or summary["fit_rms_a"] > 0.04:
        raise ValueError(f"Map fit rejected: {summary}")
    return table, summary


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scan", type=Path)
    parser.add_argument("--header", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--gain", type=float, default=0.8)
    args = parser.parse_args(argv)
    if not 0 <= args.gain <= 1:
        parser.error("gain must be 0..1")
    table, summary = fit(json.loads(args.scan.read_text(encoding="utf8")))
    values = np.rint(table * 10000).astype(int)
    sha = hashlib.sha256(args.scan.read_bytes()).hexdigest()
    lines = ["#pragma once", "#include <array>", "#include <cstdint>", "namespace roller::control::cogging {",
             f"// Motor/encoder-specific slow scan SHA256: {sha}",
             f"inline constexpr float reference_offset = {summary['electrical_offset']:.9f}F;",
             f"inline constexpr unsigned pole_pairs = {summary['pole_pairs']};",
             f"inline constexpr int direction = {summary['direction']};",
             f"inline constexpr float gain = {args.gain:.6f}F;",
             "inline constexpr std::array<std::int16_t, 1024> current_100ua{{"]
    lines += ["    " + ", ".join(str(v) for v in values[i:i + 16]) + "," for i in range(0, 1024, 16)]
    lines += ["}};", "} // namespace roller::control::cogging", ""]
    args.header.parent.mkdir(parents=True, exist_ok=True)
    args.header.write_text("\n".join(lines), encoding="utf8")
    summary.update(scan_sha256=sha, gain=args.gain)
    args.report.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf8")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
