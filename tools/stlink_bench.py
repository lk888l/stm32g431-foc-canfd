#!/usr/bin/env python3
"""Attended motor tests through a running OpenOCD and matching Bench ELF.

The MCU processes real CANopen frames, but this transport bypasses physical CAN.
Inspection never enables power. Motion commands retain firmware fault/lease checks.
"""
import argparse
import json
from pathlib import Path
import socket
import struct
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]


class Bench:
    def __init__(self, elf, port=6666, node=1):
        output = subprocess.check_output(
            ["arm-none-eabi-nm", "--defined-only", str(elf)], text=True
        )
        self.symbols = {
            parts[2]: int(parts[0], 16)
            for line in output.splitlines()
            if len(parts := line.split()) == 3
        }
        self.rx = self.symbols["roller_bench_rx"]
        self.tx = self.symbols["roller_bench_tx"]
        self.state = self.symbols["roller_bench_state"]
        self.request = self.symbols["roller_bench_request"]
        self.port, self.node = port, node

    def __enter__(self):
        self.sock = socket.create_connection(("127.0.0.1", self.port), timeout=2)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.sock.settimeout(2)
        return self

    def __exit__(self, *_):
        self.sock.close()

    def command(self, command):
        self.sock.sendall(command.encode("ascii") + b"\x1a")
        reply = bytearray()
        while not reply.endswith(b"\x1a"):
            block = self.sock.recv(65536)
            if not block:
                raise ConnectionError("OpenOCD closed the connection")
            reply.extend(block)
            if len(reply) > 1000000:
                raise RuntimeError("Unexpected OpenOCD response size")
        return reply[:-1].decode("utf8", errors="replace")

    def read(self, address, count):
        result = self.command(f"read_memory 0x{address:x} 32 {count}")
        values = [int(word, 0) for word in result.split()]
        if len(values) != count:
            raise RuntimeError("Incomplete SWD read: " + result)
        return values

    def write(self, address, values):
        words = " ".join(str(word) for word in values)
        result = self.command(f"write_memory 0x{address:x} 32 {{{words}}}")
        if result.strip():
            raise RuntimeError("SWD write failed: " + result)

    def snapshot(self):
        old = self.read(self.state + 8, 1)[0]
        request = (self.read(self.request, 1)[0] + 1) & 0xFFFFFFFF
        self.write(self.request, [request])
        deadline = time.monotonic() + 1
        while time.monotonic() < deadline:
            words = self.read(self.state, 31)
            if words[:2] != [0x524C4142, 2]:
                raise RuntimeError("Bench ABI mismatch: flash the matching Bench ELF")
            sequence = words[2]
            if sequence == old or sequence & 1:
                continue
            if self.read(self.state + 8, 1)[0] != sequence:
                continue
            result = dict(
                zip(
                    (
                        "samples",
                        "status",
                        "fault",
                        "calibration",
                        "pole_pairs",
                        "direction",
                    ),
                    words[3:9],
                )
            )
            result["direction"] = struct.unpack("<i", struct.pack("<I", words[8]))[0]
            result.update(
                zip(
                    (
                        "bus",
                        "temperature",
                        "position",
                        "velocity",
                        "id",
                        "iq",
                        "ia",
                        "ib",
                        "ic",
                        "current_limit",
                        "kt",
                        "offset",
                    ),
                    struct.unpack("<12f", struct.pack("<12I", *words[9:21])),
                )
            )
            result.update(
                zip(
                    (
                        "outputs",
                        "sample_max",
                        "control_max",
                        "apply_max",
                        "total_max",
                        "interval_min",
                        "interval_max",
                        "isr_max",
                        "sampling_epoch",
                    ),
                    words[21:30],
                )
            )
            result["cordic_max_error"] = struct.unpack(
                "<f", struct.pack("<I", words[30])
            )[0]
            return result
        raise TimeoutError("Bench snapshot was not published")

    def send(self, cob_id, data):
        if len(data) > 8 or not 0 <= cob_id <= 0x7FF:
            raise ValueError("Only classic CAN standard frames are supported")
        deadline = time.monotonic() + 1
        while (sequence := self.read(self.rx, 1)[0]) != self.read(self.rx + 20, 1)[0]:
            if time.monotonic() > deadline:
                raise TimeoutError("RX mailbox busy")
        low, high = struct.unpack("<II", data.ljust(8, b"\0"))
        self.write(self.rx + 4, [cob_id, len(data), low, high])
        sequence = (sequence + 1) & 0xFFFFFFFF
        self.write(self.rx, [sequence])  # Commit after the entire frame body.
        while self.read(self.rx + 20, 1)[0] != sequence:
            if time.monotonic() > deadline:
                raise TimeoutError("RX mailbox was not consumed")

    def nmt(self, command):
        self.send(0, bytes([command, self.node]))

    def sdo(self, index, sub=0, value=None, size=4, fmt=None):
        old = self.read(self.tx, 1)[0]
        cs = 0x40 if value is None else {1: 0x2F, 2: 0x2B, 4: 0x23}[size]
        payload = b"\0" * 4
        if value is not None:
            payload = (
                struct.pack(fmt, value)
                if fmt
                else int(value).to_bytes(size, "little", signed=value < 0)
            ).ljust(4, b"\0")
        self.send(0x600 + self.node, struct.pack("<BHB", cs, index, sub) + payload)
        deadline = time.monotonic() + 1
        while time.monotonic() < deadline:
            words = self.read(self.tx, 5)
            if words[0] == old or words[0] & 1:
                continue
            if self.read(self.tx, 1)[0] != words[0]:
                continue
            data = struct.pack("<II", words[3], words[4])
            reply, received_index, received_sub = struct.unpack("<BHB", data[:4])
            if (words[1], words[2], received_index, received_sub) != (
                0x580 + self.node,
                8,
                index,
                sub,
            ):
                continue
            if reply == 0x80:
                abort = int.from_bytes(data[4:], "little")
                raise RuntimeError(f"SDO {index:04X}:{sub} abort 0x{abort:08X}")
            if value is not None and reply != 0x60:
                raise RuntimeError("Unexpected SDO download response")
            if value is None and reply not in (0x43, 0x47, 0x4B, 0x4F):
                raise RuntimeError("Only expedited SDO uploads are supported")
            return data[4:]
        raise TimeoutError(f"SDO {index:04X}:{sub} timed out")

    def stop(self):
        try:
            self.nmt(2)  # Also aborts calibration; SDO is unavailable in Stopped.
            deadline = time.monotonic() + 0.2
            while self.snapshot()["outputs"] & 4:
                if time.monotonic() > deadline:
                    raise TimeoutError("Power did not turn off")
            self.nmt(128)
            self.sdo(0x6040, value=0, size=2)
        except BaseException:
            # Last resort: gate and nSLEEP off. A reset is needed to wake again.
            self.write(0x48000418, [0x00060000])
            raise

    def reset_fault(self):
        self.stop()
        self.sdo(0x6040, value=128, size=2)
        time.sleep(0.02)
        self.sdo(0x6040, value=0, size=2)
        if self.snapshot()["fault"]:
            raise RuntimeError("Fault cause is still present")

    def enable_velocity(self):
        self.stop()
        if self.snapshot()["calibration"] != 4:
            raise RuntimeError("Calibrate before enabling motion")
        self.sdo(0x6060, value=3, size=1)
        self.sdo(0x60FF, value=0, fmt="<f")
        self.nmt(1)
        for controlword, expected in ((6, 0x21), (7, 0x23), (15, 0x27)):
            self.sdo(0x6040, value=controlword, size=2)
            deadline = time.monotonic() + 0.08
            while (self.snapshot()["status"] & 0x6F) != expected:
                if time.monotonic() > deadline:
                    raise TimeoutError("Drive did not reach the requested state")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--elf", type=Path, default=ROOT / "build/Bench/canopen_foc.elf"
    )
    parser.add_argument("--port", type=int, default=6666)
    parser.add_argument(
        "--node", type=int, choices=range(1, 128), default=1, metavar="1..127"
    )
    parser.add_argument("--output", type=Path, help="Write telemetry JSON")
    sub = parser.add_subparsers(dest="operation", required=True)
    for name in ("inspect", "stop", "fault-reset"):
        sub.add_parser(name)
    sub.add_parser("calibrate").add_argument("--save", action="store_true")
    speed = sub.add_parser("velocity")
    speed.add_argument(
        "target", type=float, help="Unloaded bench target, -5 to +5 rev/s"
    )
    speed.add_argument("--seconds", type=float, default=3, help="1 to 30 seconds")
    args = parser.parse_args()
    if args.operation == "velocity" and not (
        -5 <= args.target <= 5 and 1 <= args.seconds <= 30
    ):
        parser.error("Velocity trials are limited to +/-5 rev/s and 1..30 seconds")
    records = []
    with Bench(args.elf, args.port, args.node) as bench:
        # Resolve/check the ABI before writing any control command.
        initial = bench.snapshot()
        if args.operation == "inspect":
            print(json.dumps(initial, indent=2))
            return
        try:
            if args.operation == "fault-reset":
                bench.reset_fault()
            elif args.operation == "calibrate":
                bench.stop()
                bench.sdo(0x2100, 1, 0x696C6163)
                start = time.monotonic()
                started = False
                while time.monotonic() - start < 9:
                    state = bench.snapshot()
                    records.append(dict(state, seconds=time.monotonic() - start))
                    if state["fault"] or state["calibration"] == 5:
                        raise RuntimeError(f"Calibration failed: {state}")
                    started |= state["calibration"] in (1, 2, 3)
                    if started and state["calibration"] == 4:
                        break
                    time.sleep(0.02)
                else:
                    raise TimeoutError("Calibration did not finish")
                bench.stop()
                if args.save:
                    bench.sdo(0x1010, 1, 0x65766173)
                    time.sleep(0.02)
                    if bench.snapshot()["fault"]:
                        raise RuntimeError("Drive fault after parameter save")
            elif args.operation == "velocity":
                bench.enable_velocity()
                start = time.monotonic()
                while time.monotonic() - start < args.seconds:
                    bench.sdo(0x60FF, value=args.target, fmt="<f")
                    state = bench.snapshot()
                    records.append(
                        dict(
                            state, seconds=time.monotonic() - start, target=args.target
                        )
                    )
                    if state["fault"] or abs(state["velocity"]) > 7:
                        raise RuntimeError(f"Motion trial aborted: {state}")
                    time.sleep(0.005)
        finally:
            bench.stop()
            if args.output:
                args.output.parent.mkdir(parents=True, exist_ok=True)
                args.output.write_text(
                    json.dumps(records, indent=2) + "\n", encoding="utf8"
                )
        print(json.dumps(bench.snapshot(), indent=2))


if __name__ == "__main__":
    main()
