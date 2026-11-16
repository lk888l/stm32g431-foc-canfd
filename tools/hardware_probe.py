#!/usr/bin/env python3
"""Read v2 hardware diagnostics through OpenOCD; never sends a motion command.

The only writes request an immutable diagnostic snapshot. The MCU keeps running.
Works with Debug/Release as well as Bringup; defaults to the no-motion Bringup ELF.
"""
import argparse
import json
from pathlib import Path
import struct
import subprocess
import time

from stlink_bench import Bench


class HardwareProbe(Bench):
    def __init__(self, elf, port=6666):
        output = subprocess.check_output(["arm-none-eabi-nm", "--defined-only", str(elf)], text=True)
        self.symbols = {p[2]: int(p[0], 16) for line in output.splitlines() if len(p := line.split()) == 3}
        self.port = port
        self.state = self.symbols["roller_hardware_snapshot"]
        self.request = self.symbols["roller_hardware_request"]
        self.ack = self.symbols["roller_hardware_ack"]

    def snapshot(self):
        request = (self.read(self.request, 1)[0] + 1) & 0xFFFFFFFF
        self.write(self.request, [request])
        deadline = time.monotonic() + 2
        while self.read(self.ack, 1)[0] != request:
            if time.monotonic() > deadline:
                raise TimeoutError("MCU did not publish hardware diagnostics")
            time.sleep(0.002)
        words = self.read(self.state, 27)
        if self.read(self.ack, 1)[0] != request:
            raise RuntimeError("Another diagnostic reader replaced this snapshot")
        fields = (
            "sequence", "frames", "encoder_frames", "encoder_errors", "encoder_raw", "encoder_status",
            "adc_a", "adc_b", "adc_c", "adc_bus", "adc_ntc", "current_mask", "adc_errors", "missed_frames",
            "angle", "bus", "vdda", "temperature", "offset_a", "offset_b", "offset_c",
            "offsets_valid", "driver_awake", "no_motion", "pwm_deferred", "warnings", "can_loopback",
        )
        values = words[:14] + list(struct.unpack("<7f", struct.pack("<7I", *words[14:21]))) + words[21:]
        result = dict(zip(fields, values))
        result["gpio_b_odr"] = self.read(0x48000414, 1)[0]
        result["timer_ccer"] = self.read(0x40012C20, 1)[0]
        result["gpio_a_mode"] = self.read(0x48000000, 1)[0]
        result["gpio_a_odr"] = self.read(0x48000014, 1)[0]
        result["elapsed"] = time.monotonic()
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--elf", type=Path, default=Path("build/Bringup/canopen_foc.elf"))
    parser.add_argument("--port", type=int, default=6666)
    parser.add_argument("--samples", type=int, default=1)
    parser.add_argument("--interval", type=float, default=0.05)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if not 1 <= args.samples <= 10000 or not 0 <= args.interval <= 60:
        parser.error("samples must be 1..10000, interval 0..60 seconds")
    records = []
    with HardwareProbe(args.elf, args.port) as probe:
        for i in range(args.samples):
            records.append(probe.snapshot())
            if i + 1 < args.samples:
                time.sleep(args.interval)
    text = json.dumps(records, indent=2)
    if args.output:
        args.output.write_text(text + "\n", encoding="utf8")
        print(f"Saved {len(records)} snapshots to {args.output}")
    else:
        print(text)


if __name__ == "__main__":
    main()
