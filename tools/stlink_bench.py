#!/usr/bin/env python3
"""Attended motor tests through a running OpenOCD and matching Bench ELF.

The MCU processes real CANopen frames, but this transport bypasses physical CAN.
Inspection never enables power. Motion commands retain firmware fault/lease checks.
"""
import argparse
import json
import math
from pathlib import Path
import socket
import struct
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]
MODES = {"position": -1, "multi-position": 1, "velocity": 3, "torque": 4, "mit": 5}


def finite_between(value, low, high, label):
    if not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f"{label} must be finite and within {low}..{high}")
    return value


def position_distance(position, target, single_turn=False, relative=False):
    """Match the firmware's negative direction for an exact half turn."""
    distance = target if relative else target - position
    return (distance + 0.5) % 1.0 - 0.5 if single_turn else distance


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
            # The MCU may not have serviced the request yet. Poll one word first;
            # a stale 31-word SWD read can consume most of the 100 ms motion lease.
            sequence = self.read(self.state + 8, 1)[0]
            if sequence == old or sequence & 1:
                continue
            words = self.read(self.state, 31)
            if words[:2] != [0x524C4142, 2]:
                raise RuntimeError("Bench ABI mismatch: flash the matching Bench ELF")
            if words[2] != sequence:
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
            raise ValueError("SWD Bench ABI 2 supports only classic CAN standard frames; use physical SocketCAN for CAN-FD")
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

    def prepare_mode(self, mode):
        self.stop()
        state = self.snapshot()
        if state["calibration"] != 4:
            raise RuntimeError("Calibrate before enabling motion")
        if state["fault"]:
            raise RuntimeError(f"Reset the existing fault before motion: {state['fault']:#06x}")
        self.sdo(0x6060, value=mode, size=1)
        return state

    def statusword(self):
        return struct.unpack("<H", self.sdo(0x6041)[:2])[0]

    def activate(self):
        self.nmt(1)
        for controlword, expected in ((6, 0x21), (7, 0x23), (15, 0x27)):
            self.sdo(0x6040, value=controlword, size=2)
            deadline = time.monotonic() + 0.08
            while True:
                status = self.statusword()
                if status & 8:
                    raise RuntimeError(f"Drive fault while enabling, statusword={status:#06x}")
                if status & 0x6F == expected:
                    break
                if time.monotonic() > deadline:
                    raise TimeoutError("Drive did not reach the requested state")

    def enable_velocity(self):
        state = self.prepare_mode(MODES["velocity"])
        self.sdo(0x60FF, value=0, fmt="<f")
        self.activate()
        return state

    def enable_position(self, target, *, single_turn=False, relative=False, speed=1):
        finite_between(target, -100000, 100000, "Position")
        finite_between(speed, 0.05, 5, "Profile speed")
        state = self.prepare_mode(MODES["position" if single_turn else "multi-position"])
        distance = position_distance(state["position"], target, single_turn, relative)
        finite_between(distance, -5, 5, "Bench position travel")
        self.sdo(0x6081, value=speed, fmt="<f")
        self.sdo(0x607A, value=target, fmt="<f")
        self.activate()
        controlword = 0x1F | (0x40 if relative else 0)
        self.sdo(0x6040, value=controlword, size=2)
        deadline = time.monotonic() + 0.08
        while True:
            status = self.statusword()
            if status & 8:
                raise RuntimeError(f"Position setpoint rejected, statusword={status:#06x}")
            if status & 0x1000:
                break
            if time.monotonic() > deadline:
                raise TimeoutError("Position setpoint was not acknowledged")
        return state, controlword

    def enable_torque(self):
        state = self.prepare_mode(MODES["torque"])
        self.sdo(0x6071, value=0, size=2)
        self.activate()
        return state

    def enable_mit(self, *, position=None, relative=False, velocity=0, torque=0, kp=0.01, kd=0.002):
        for value, low, high, label in (
            (velocity, -5, 5, "MIT velocity"),
            (torque, -0.005, 0.005, "MIT feedforward torque"),
            (kp, 0, 1, "MIT Kp"),
            (kd, 0, 0.1, "MIT Kd"),
        ):
            finite_between(value, low, high, label)
        if position is not None:
            finite_between(position, -100000, 100000, "MIT position")
        state = self.prepare_mode(MODES["mit"])
        target = state["position"] if position is None else position + (state["position"] if relative else 0)
        finite_between(target, -100000, 100000, "MIT position")
        finite_between(target - state["position"], -2, 2, "Bench MIT travel")
        # Disable packed PDO interpretation before configuring the ordinary MIT objects.
        # All five fields are committed while disabled, so no partial command can move the motor.
        self.sdo(0x2004, 1, value=0, size=1)
        for sub, value in ((1, target), (2, velocity), (3, torque)):
            self.sdo(0x2003, sub, value=value, fmt="<f")
        self.sdo(0x2003, 4, value=round(kp * 1000), size=2)
        self.sdo(0x2003, 5, value=round(kd * 1000), size=2)
        self.activate()
        return state, target


def run_motion(bench, args, records):
    details = {"operation": args.operation, "mode": MODES[args.operation]}
    if args.operation == "velocity":
        initial = bench.enable_velocity()
        refresh = lambda: bench.sdo(0x60FF, value=args.target, fmt="<f")
        details["target"] = args.target
    elif args.operation in ("position", "multi-position"):
        initial, controlword = bench.enable_position(
            args.target, single_turn=args.operation == "position", relative=args.relative, speed=args.speed
        )
        # Keep bit4 high: rewriting an identical relative target must not trigger another move.
        refresh = lambda: bench.sdo(0x6040, value=controlword, size=2)
        resolved_target = initial["position"] + position_distance(
            initial["position"], args.target, args.operation == "position", args.relative
        )
        details.update(target=args.target, relative=args.relative, profile_velocity=args.speed,
                       resolved_target=resolved_target)
    elif args.operation == "torque":
        initial = bench.enable_torque()
        refresh = lambda: bench.sdo(0x6071, value=args.target, size=2)
        details.update(target=args.target, target_iq=initial["current_limit"] * args.target / 1000)
    else:
        initial, target = bench.enable_mit(
            position=args.position, relative=args.relative, velocity=args.velocity,
            torque=args.torque, kp=args.kp, kd=args.kd,
        )
        refresh = lambda: bench.sdo(0x2003, 3, value=args.torque, fmt="<f")
        details.update(target=target, target_velocity=args.velocity, torque=args.torque,
                       kp=round(args.kp * 1000) / 1000, kd=round(args.kd * 1000) / 1000)
    details["initial_position"] = initial["position"]
    start = time.monotonic()
    while time.monotonic() - start < args.seconds:
        refresh()
        state = bench.snapshot()
        records.append(dict(state, seconds=time.monotonic() - start, **details))
        motion_values = (state["position"], state["velocity"], state["iq"], state["id"])
        excessive_travel = args.operation != "velocity" and abs(state["position"] - initial["position"]) > 6
        if (state["fault"] or not all(map(math.isfinite, motion_values)) or
                abs(state["velocity"]) > 7 or excessive_travel):
            raise RuntimeError(f"Motion trial aborted: {state}")
        if state["status"] & 0x6F != 0x27:
            raise RuntimeError(f"Drive left operation enabled: {state}")
        time.sleep(0.005)


def main(argv=None):
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
    for name in ("position", "multi-position"):
        position = sub.add_parser(name, help="Single-turn shortest path" if name == "position" else "Accumulated multi-turn position")
        position.add_argument("target", type=float, help="Target in revolutions; travel limited to 5 turns")
        position.add_argument("--relative", action="store_true", help="Relative displacement; single-turn wraps to [-0.5, 0.5)")
        position.add_argument("--speed", type=float, default=1, help="Profile speed, 0.05 to 5 rev/s")
        position.add_argument("--seconds", type=float, default=3, help="1 to 30 seconds")
    torque = sub.add_parser("torque", help="Current-based torque target; estimated Kt, not calibrated Nm")
    torque.add_argument("target", type=int, help="-200 to +200 permille of configured current limit")
    torque.add_argument("--seconds", type=float, default=1, help="0.1 to 30 seconds")
    mit = sub.add_parser("mit", help="MIT impedance: position/velocity feedback plus feedforward torque")
    mit.add_argument("--position", type=float, help="Position in rev; omitted holds initial position")
    mit.add_argument("--relative", action="store_true", help="Interpret --position as displacement from initial position")
    mit.add_argument("--velocity", type=float, default=0, help="-5 to +5 rev/s")
    mit.add_argument("--torque", type=float, default=0, help="Feedforward torque, -0.005 to +0.005 Nm")
    mit.add_argument("--kp", type=float, default=0.01, help="0 to 1 Nm/rev, quantized to 0.001")
    mit.add_argument("--kd", type=float, default=0.002, help="0 to 0.1 Nm/(rev/s), quantized to 0.001")
    mit.add_argument("--seconds", type=float, default=3, help="0.1 to 30 seconds")
    args = parser.parse_args(argv)
    try:
        if args.operation in MODES:
            finite_between(args.seconds, 0.1 if args.operation in ("torque", "mit") else 1, 30, "Duration")
        if args.operation == "velocity":
            finite_between(args.target, -5, 5, "Velocity")
        elif args.operation == "torque":
            finite_between(args.target, -200, 200, "Torque permille")
        elif args.operation in ("position", "multi-position"):
            finite_between(args.target, -100000, 100000, "Position")
            finite_between(args.speed, 0.05, 5, "Profile speed")
        elif args.operation == "mit":
            for value, low, high, label in (
                (args.velocity, -5, 5, "MIT velocity"), (args.torque, -0.005, 0.005, "MIT torque"),
                (args.kp, 0, 1, "MIT Kp"), (args.kd, 0, 0.1, "MIT Kd"),
            ):
                finite_between(value, low, high, label)
            if args.position is not None:
                finite_between(args.position, -100000, 100000, "MIT position")
    except ValueError as error:
        parser.error(str(error))
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
            elif args.operation in MODES:
                run_motion(bench, args, records)
        finally:
            try:
                bench.stop()
            finally:
                # Preserve the last fault telemetry even if the stop transport also fails.
                if args.output:
                    args.output.parent.mkdir(parents=True, exist_ok=True)
                    args.output.write_text(
                        json.dumps(records, indent=2) + "\n", encoding="utf8"
                    )
        print(json.dumps(bench.snapshot(), indent=2))


if __name__ == "__main__":
    main()
