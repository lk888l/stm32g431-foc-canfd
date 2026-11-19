#!/usr/bin/env python3
"""Attended motor tests through a running OpenOCD and matching Bench ELF.

The MCU processes real CANopen frames, but this transport bypasses physical CAN.
Inspection never enables power. Motion commands require explicit enable; communication expiry is diagnostic.
"""
import argparse
import json
import math
from pathlib import Path
import socket
import struct
import subprocess
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
MODES = {"position": -1, "multi-position": 1, "velocity": 3, "torque": 4, "mit": 5}
ZERO_REASONS = ("none", "offset outside range", "noisy zero", "phase zero mismatch",
                "CSA unavailable", "cancelled")
CALIBRATION_REASONS = ("none", "prerequisites not ready", "current feedback missing",
                       "current polarity/phase response", "calibration current exceeded",
                       "encoder/nonfinite input", "sweep travel/slip", "offset concentration",
                       "return position or speed", "interrupted", "bus voltage")


def require_current_feedback(state):
    if state.get("no_motion"):
        raise RuntimeError("Bringup firmware cannot run motor tests; flash Bench first")
    if "offsets_valid" in state and not state["offsets_valid"]:
        reason = state.get("zero_reason", 0)
        label = ZERO_REASONS[reason] if reason < len(ZERO_REASONS) else str(reason)
        raise RuntimeError(f"Current-zero acquisition failed or is incomplete: {label}. "
                           "Inspect hardware diagnostics; fix the cause and reboot before motion.")


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
        self._guard_armed = False

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
        try:
            values = [int(word, 0) for word in result.split()]
        except ValueError as error:
            raise RuntimeError(f"SWD read at {address:#x} failed: {result}") from error
        if len(values) != count:
            raise RuntimeError("Incomplete SWD read: " + result)
        return values

    def frozen_words(self, address, count):
        previous = self.read(address, count)
        for _ in range(4):
            current = self.read(address, count)
            if current == previous:
                return current
            previous = current
        raise RuntimeError(f"Inconsistent frozen SWD words at {address:#x}")

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
            words = self.frozen_words(self.state, 31)
            if words[0] != 0x524C4142 or words[1] not in (2, 3, 4, 5):
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
            if words[1] >= 3:
                extra = self.frozen_words(self.state + 124, 17 if words[1] >= 4 else 16)
                if self.read(self.state + 8, 1)[0] != sequence:
                    continue
                result.update(zip(("warnings", "encoder_missed", "current_predicted", "calibration_reason",
                                   "calibration_ticks", "calibration_points"), extra[:6]))
                result.update(zip(("calibration_concentration", "calibration_travel", "iq_target"),
                                  struct.unpack("<3f", struct.pack("<3I", *extra[6:9]))))
                result.update(zip(("offsets_valid", "zero_state", "zero_reason", "zero_span",
                                   "driver_awake", "pwm_channels", "no_motion"), extra[9:]))
                if words[1] >= 4:
                    result["calibration_angle"] = struct.unpack("<f", struct.pack("<I", extra[16]))[0]
                if words[1] >= 5:
                    voltage = self.frozen_words(self.state + 192, 3)
                    if self.read(self.state + 8, 1)[0] != sequence:
                        continue
                    result.update(zip(("voltage_a", "voltage_b", "voltage_c"),
                                      struct.unpack("<3f", struct.pack("<3I", *voltage))))
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

    def arm_capture(self, kind=0):
        if "roller_trace_kind" in self.symbols:
            kind_address = self.symbols["roller_trace_kind"]
            self.write(kind_address, [kind])
            if self.frozen_words(kind_address, 1)[0] != kind:
                raise RuntimeError("SWD capture mode write did not verify")
        elif kind:
            raise RuntimeError("Flash a Bench ELF with motion/peak capture support")
        request = self.symbols["roller_trace_request"]
        generation = (self.frozen_words(request, 1)[0] + 1) & 0xFFFFFFFF
        self.write(request, [generation])
        if self.frozen_words(request, 1)[0] != generation:
            raise RuntimeError("SWD capture request did not verify")
        return generation

    def frozen_capture(self, generation, count, size, refresh=None):
        ack = self.symbols["roller_trace_ack"]
        deadline = time.monotonic() + 2
        while self.read(ack, 1)[0] != generation or self.frozen_words(ack, 1)[0] != generation:
            if time.monotonic() >= deadline:
                raise TimeoutError("Trace capture did not complete")
            if refresh:
                refresh()
            time.sleep(0.01)
        if self.frozen_words(self.symbols["roller_trace_count"], 1)[0] != count:
            raise RuntimeError("Trace ABI mismatch")
        # dump_image uses the ST-Link block-memory path. read_memory's Tcl
        # scalar loop is much slower for a full capture on HLA adapters.
        with tempfile.NamedTemporaryFile(suffix=".bin", delete=False) as stream:
            path = Path(stream.name)
        try:
            previous = None
            for _ in range(4):
                if refresh:
                    refresh()
                path.write_bytes(b"")
                self.command(f"dump_image {{{path.as_posix()}}} 0x{self.symbols['roller_trace_data']:x} {size}")
                data = path.read_bytes()
                if len(data) == size and data == previous:
                    break
                previous = data
            else:
                # A long adapter burst can contain a bad word. Re-read short
                # immutable blocks and require agreement for each block.
                chunks = []
                address = self.symbols["roller_trace_data"]
                for offset in range(0, size, 64):
                    if refresh and offset % 512 == 0:
                        refresh()
                    words = self.frozen_words(address + offset, min(64, size-offset)//4)
                    chunks.append(struct.pack(f"<{len(words)}I", *words))
                data = b"".join(chunks)
        finally:
            path.unlink(missing_ok=True)
        if (self.frozen_words(ack, 1)[0] != generation or
                self.read(self.symbols["roller_trace_request"], 1)[0] != generation):
            raise RuntimeError("Trace was replaced during readout")
        return data

    def capture(self, refresh=None):
        """Capture 96 consecutive 1 kHz samples; hardware stays running."""
        data = self.frozen_capture(self.arm_capture(), 96, 96 * 24, refresh)
        rows = [dict(zip(("samples", "position", "velocity", "iq", "id", "iq_target"), values))
                for values in struct.iter_unpack("<I5f", data)]
        if any((b["samples"] - a["samples"]) & 0xFFFFFFFF != 20 for a, b in zip(rows, rows[1:])):
            raise RuntimeError("Trace was replaced or contains nonconsecutive samples")
        return rows

    def capture_ticks(self, generation, intervals, stride):
        first = self.frozen_words(self.symbols["roller_trace_first"], 1)[0]
        last = self.frozen_words(self.symbols["roller_trace_last"], 1)[0]
        if ((last - first) & 0xFFFFFFFF != intervals * stride or
                self.read(self.symbols["roller_trace_request"], 1)[0] != generation):
            raise RuntimeError("Trace was replaced or has a discontinuous time base")
        return first, last

    def motion_capture(self, refresh=None):
        generation = self.arm_capture(1)
        data = self.frozen_capture(generation, 768, 768 * 4, refresh)
        words = self.frozen_words(self.symbols["roller_trace_origin"], 2)
        origin, = struct.unpack("<q", struct.pack("<II", *words))
        first, _ = self.capture_ticks(generation, 767, 20)
        rows = []
        position = origin
        for i, (angle, velocity) in enumerate(struct.iter_unpack("<Hh", data)):
            delta = ((angle - position + 8192) & 16383) - 8192
            if abs(delta) > 1700 or (i == 0 and delta):
                raise RuntimeError("Encoder discontinuity in motion capture")
            position += delta
            rows.append(dict(samples=(first + 20*i) & 0xFFFFFFFF, counts=position, velocity=velocity/512,
                             ack=bool(angle & 0x4000), marked=bool(angle & 0x8000)))
        return rows

    def peak_capture(self, refresh=None):
        generation = self.arm_capture(2)
        data = self.frozen_capture(generation, 2000, 8 * 56, refresh)
        first, last = self.capture_ticks(generation, 1999, 1)
        peaks = []
        for mask, values in enumerate(struct.iter_unpack("<II10f4H", data)):
            frames, tick, sum2, max2, id_, iq, target, ia, ib, ic, angle, bus, a, b, c, _ = values
            if frames:
                if (((tick-first) & 0xFFFFFFFF) >= 2000 or max(a, b, c) > 4000 or not 0 <= angle < math.tau or
                        not all(math.isfinite(v) for v in values[2:12]) or sum2 < max2 or
                        not math.isclose(max2, id_**2+iq**2, abs_tol=1e-6)):
                    raise RuntimeError("Invalid same-frame current peak")
                peaks.append(dict(mask=mask, frames=frames, samples=tick, sum_squares=sum2, peak=math.sqrt(max2),
                                  id=id_, iq=iq, iq_target=target, raw_abc=[ia, ib, ic],
                                  electrical_angle=angle, bus=bus, ccr=[a, b, c]))
        if sum(p["frames"] for p in peaks) != 2000:
            raise RuntimeError("Nonconsecutive current capture")
        return dict(first=first, last=last, peaks=peaks)

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
            while self.snapshot()["outputs"] & (1 << 10):
                if time.monotonic() > deadline:
                    raise TimeoutError("Power did not turn off")
            self.nmt(128)
            self.sdo(0x6040, value=0, size=2)
            if self._guard_armed:
                self.write(self.symbols["roller_bench_guard"], [0])
                self._guard_armed = False
        except BaseException:
            if self._guard_armed:
                # Leave the CPU running so the independent timeout can gate off
                # after a lost SWD link, including a failed host stop command.
                raise
            # Explicit stop fallback: halt first so the control ISR cannot wake
            # the driver again after a raw nSLEEP write. Resume requires operator action.
            self.command("halt")
            self.write(0x48000418, [0x04000000])
            self.write(0x40012C20, [self.read(0x40012C20, 1)[0] & ~0x111])
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
        require_current_feedback(state)
        if state["calibration"] != 4:
            raise RuntimeError("Calibrate before enabling motion")
        if state["fault"]:
            raise RuntimeError(f"Reset the existing fault before motion: {state['fault']:#06x}")
        self.sdo(0x6060, value=mode, size=1)
        return state

    def statusword(self):
        return struct.unpack("<H", self.sdo(0x6041)[:2])[0]

    def activate(self):
        if "roller_bench_guard" in self.symbols:
            address = self.symbols["roller_bench_guard"]
            self.write(address, [1])
            if self.frozen_words(address, 1) != [1]:
                raise RuntimeError("SWD motion guard did not arm")
            self._guard_armed = True
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
            (kp, 0, 5, "MIT Kp"),
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
    bench.stop()
    state = bench.snapshot()
    require_current_feedback(state)
    limit = getattr(args, "current_limit", 0.3)
    finite_between(limit, 0.02, 1.4, "Bench current limit")
    bench.sdo(0x6072, value=min(1000, int(1000 * limit / state["current_limit"])), size=2)
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
                max(abs(state["iq"]), abs(state["id"])) > limit * 1.5 + 0.1 or
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
    sub.add_parser("cogging").add_argument("setting", choices=("on", "off"))
    calibration = sub.add_parser("calibrate")
    calibration.add_argument("--save", action="store_true")
    calibration.add_argument("--slow", action="store_true", help="201.6 s bidirectional scan for cogging measurement; requires prior calibration")
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
    mit.add_argument("--kp", type=float, default=0.01, help="0 to 5 Nm/rev, quantized to 0.001")
    mit.add_argument("--kd", type=float, default=0.002, help="0 to 0.1 Nm/(rev/s), quantized to 0.001")
    mit.add_argument("--seconds", type=float, default=3, help="0.1 to 30 seconds")
    for motion in (speed, torque, mit):
        motion.add_argument("--current-limit", type=float, default=0.3, help="Bench limit in A, default 0.3")
    for name in ("position", "multi-position"):
        sub.choices[name].add_argument("--current-limit", type=float, default=0.3,
                                      help="Bench limit in A, default 0.3")
    args = parser.parse_args(argv)
    try:
        if args.operation in MODES:
            finite_between(args.seconds, 0.1 if args.operation in ("torque", "mit") else 1, 30, "Duration")
            finite_between(args.current_limit, 0.02, 1.4, "Bench current limit")
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
                (args.kp, 0, 5, "MIT Kp"), (args.kd, 0, 0.1, "MIT Kd"),
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
            elif args.operation == "cogging":
                bench.stop()
                bench.sdo(0x2100, 1, 0x206E6F63 if args.setting == "on" else 0x66666F63)
            elif args.operation == "calibrate":
                bench.stop()
                state = bench.snapshot()
                records.append(dict(state, seconds=0))
                require_current_feedback(state)
                bench.sdo(0x2100, 1, 0x776F6C73 if args.slow else 0x696C6163)
                start = time.monotonic()
                started = False
                while time.monotonic() - start < (220 if args.slow else 25):
                    state = bench.snapshot()
                    records.append(dict(state, seconds=time.monotonic() - start))
                    if state["fault"] or state["calibration"] == 5:
                        reason = state.get("calibration_reason", 0)
                        label = CALIBRATION_REASONS[reason] if reason < len(CALIBRATION_REASONS) else str(reason)
                        raise RuntimeError(f"Calibration failed ({label}): {state}")
                    started |= state["calibration"] in (1, 2, 3)
                    if started and state["calibration"] == 4:
                        break
                    if args.slow and len(records) % 200 == 0:
                        print(f"Scan {time.monotonic() - start:.1f} s, position {state['position']:.4f} rev", flush=True)
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
