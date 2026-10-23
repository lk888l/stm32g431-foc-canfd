#!/usr/bin/env python3
"""Linux SocketCAN client: classic SDO and automatic classic/CAN-FD PDOs."""
import argparse
import json
import math
import socket
import struct
import time
from typing import NamedTuple

TYPES = {"u8": "B", "i8": "b", "u16": "H", "i16": "h", "u32": "I", "i32": "i", "f32": "f"}
MODES = {"disabled": 0, "single-turn": -1, "multi-turn": 1, "position": 1,
         "velocity": 3, "torque": 4, "mit": 5}
FRAME = struct.Struct("=IB3x8s")
FD_FRAME = struct.Struct("=IBB2x64s")
FD_LENGTHS = (*range(9), 12, 16, 20, 24, 32, 48, 64)
NOMINAL_RATES = (125000, 250000, 500000, 1000000)
DATA_RATES = (1000000, 2000000, 4000000, 5000000, 8000000)
CAN_RAW_FD_FRAMES = getattr(socket, "CAN_RAW_FD_FRAMES", 5)
CANFD_BRS, CANFD_ESI = 1, 2


class CanFrame(NamedTuple):
    cob_id: int
    data: bytes
    is_fd: bool = False
    brs: bool = False
    esi: bool = False


def fd_wire_length(length):
    if not 0 <= length <= 64:
        raise ValueError("CAN-FD payload must contain 0..64 bytes")
    return next(size for size in FD_LENGTHS if size >= length)


def pack_frame(cob_id, data, *, fd=None, brs=True):
    """Encode Linux can_frame/canfd_frame; fd=None chooses FD only above 8 bytes."""
    if not 0 <= cob_id <= 0x7FF:
        raise ValueError("Only standard 11-bit data frames are supported")
    payload = bytes(data)
    fd = len(payload) > 8 if fd is None else fd
    if fd:
        size = fd_wire_length(len(payload))
        return FD_FRAME.pack(cob_id, size, CANFD_BRS if brs else 0, payload.ljust(64, b"\0"))
    if len(payload) > 8:
        raise ValueError("Classic CAN payload cannot exceed 8 bytes")
    return FRAME.pack(cob_id, len(payload), payload.ljust(8, b"\0"))


def unpack_frame(packet):
    """Decode either SocketCAN MTU, retaining FD/BRS/ESI and on-wire padding."""
    if len(packet) == FRAME.size:
        cob_id, size, payload = FRAME.unpack(packet)
        flags, is_fd, maximum = 0, False, 8
    elif len(packet) == FD_FRAME.size:
        cob_id, size, flags, payload = FD_FRAME.unpack(packet)
        is_fd, maximum = True, 64
    else:
        raise ValueError("SocketCAN packet must be 16 or 72 bytes")
    if cob_id > 0x7FF:
        raise ValueError("Extended, remote and error frames are not supported")
    if size > maximum:
        raise ValueError("SocketCAN payload length exceeds its frame format")
    return CanFrame(cob_id, payload[:size], is_fd, bool(flags & CANFD_BRS), bool(flags & CANFD_ESI))


def open_can_socket(interface="can0", *, filters=None, timeout=0.5):
    """Open FD-capable SocketCAN. filters=None accepts all standard data IDs."""
    if not hasattr(socket, "AF_CAN"):
        raise RuntimeError("Use Linux SocketCAN, or the configured can0 inside WSL")
    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    try:
        sock.setsockopt(socket.SOL_CAN_RAW, CAN_RAW_FD_FRAMES, 1)
        if filters is None:
            packed_filters = struct.pack("=II", 0, 0xC0000000)
        else:
            filters = tuple(filters)
            if any(not 0 <= cob_id <= 0x7FF for cob_id in filters):
                raise ValueError("SocketCAN filters must use standard 11-bit IDs")
            # CAN_ERR_FLAG in a filter mask selects the kernel's error receive list.
            # Match EFF/RTR only; errors have their own CAN_RAW_ERR_FILTER option.
            packed_filters = b"".join(struct.pack("=II", cob_id, 0xC00007FF) for cob_id in filters)
        sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, packed_filters)
        sock.settimeout(timeout)
        sock.bind((interface,))
    except BaseException:
        sock.close()
        raise
    return sock


def send_frame(sock, cob_id, data, *, fd=None, brs=True):
    packet = pack_frame(cob_id, data, fd=fd, brs=brs)
    if sock.send(packet) != len(packet):
        raise OSError("Incomplete SocketCAN datagram send")
    return unpack_frame(packet)


def recv_frame(sock, *, timeout=None):
    """Return a standard data frame, ignoring unrelated/malformed frames within one deadline."""
    budget = sock.gettimeout() if timeout is None else timeout
    deadline = None if budget is None else time.monotonic() + budget
    first = True
    while True:
        if deadline is not None:
            remaining = deadline - time.monotonic()
            if remaining <= 0 and not (first and budget == 0):
                raise TimeoutError("No valid standard CAN frame before the receive deadline")
            sock.settimeout(max(0, remaining))
        first = False
        packet = sock.recv(FD_FRAME.size)
        try:
            return unpack_frame(packet)
        except ValueError:
            # Kernel filters are an optimization, not a substitute for frame validation.
            # Error/RTR/extended traffic and malformed packets must not abort an SDO exchange.
            continue


def mapping_size(entries):
    if not 0 <= len(entries) <= 16:
        raise ValueError("A PDO supports at most 16 mapping entries")
    total = 0
    for entry in entries:
        if not 0 <= entry <= 0xFFFFFFFF or not 0 < (entry & 0xFF) <= 248 or entry & 7:
            raise ValueError("Mapping entries must be u32 index/sub/bit-length with whole bytes")
        total += (entry & 0xFF) // 8
    if total > 64:
        raise ValueError("Mapped PDO payload exceeds 64 bytes")
    return total


def pdo_indices(direction, number):
    if direction not in ("rpdo", "tpdo") or number not in range(1, 5):
        raise ValueError("Choose rpdo/tpdo and PDO number 1..4")
    return ((0x1400 if direction == "rpdo" else 0x1800) + number - 1,
            (0x1600 if direction == "rpdo" else 0x1A00) + number - 1)


class SdoClient:
    def __init__(self, interface, node, timeout=0.5):
        if not 1 <= node <= 127:
            raise ValueError("Node ID must be within 1..127")
        self.node = node
        self.timeout = timeout
        self.socket = open_can_socket(interface, filters=[0x580 + node], timeout=timeout)

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    def close(self):
        self.socket.close()

    def send(self, cob_id, data, *, fd=None, brs=True):
        return send_frame(self.socket, cob_id, data, fd=fd, brs=brs)

    def request(self, index, sub, cs, payload=b""):
        data = struct.pack("<BHB", cs, index, sub) + payload.ljust(4, b"\0")
        self.send(0x600 + self.node, data, fd=False)
        deadline = time.monotonic() + self.timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(f"SDO {index:04X}:{sub:02X} timed out")
            frame = recv_frame(self.socket, timeout=remaining)
            reply = frame.data
            if frame.is_fd or frame.cob_id != 0x580 + self.node or len(reply) != 8 or reply[1:4] != data[1:4]:
                continue
            if reply[0] == 0x80:
                raise RuntimeError(
                    f"SDO abort {index:04X}:{sub:02X}: 0x{int.from_bytes(reply[4:8], 'little'):08X}"
                )
            return reply

    def read(self, index, sub, typ):
        reply = self.request(index, sub, 0x40)
        fmt = "<" + TYPES[typ]
        size = struct.calcsize(fmt)
        if reply[0] != 0x43 + ((4 - size) << 2):
            raise RuntimeError("Unexpected SDO response size/type; use the EDS type")
        return struct.unpack(fmt, reply[4 : 4 + size])[0]

    def write(self, index, sub, typ, value):
        payload = struct.pack("<" + TYPES[typ], value)
        if self.request(index, sub, 0x23 + ((4 - len(payload)) << 2), payload)[0] != 0x60:
            raise RuntimeError("Unexpected SDO write response")

    def nmt(self, state):
        self.send(0, bytes([state, self.node]), fd=False)

    def preoperational_disabled(self):
        """Explicit configuration commands first revoke running permission."""
        self.nmt(128)
        self.write(0x6040, 0, "u16", 0)
        deadline = time.monotonic() + self.timeout
        while True:
            status = self.read(0x6041, 0, "u16") & 0x6F
            if status in (0x40, 0x08) and self.read(0x6040, 0, "u16") == 0:
                return
            if time.monotonic() >= deadline:
                raise TimeoutError("Drive did not disable before configuration")
            time.sleep(0.005)

    def pdo_configuration(self, direction, number):
        communication, mapping = pdo_indices(direction, number)
        cob_id = self.read(communication, 1, "u32")
        count = self.read(mapping, 0, "u8")
        if count > 16:
            raise ValueError("Firmware returned more than 16 PDO mapping entries")
        entries = [self.read(mapping, sub, "u32") for sub in range(1, count + 1)]
        size = mapping_size(entries)
        return dict(direction=direction, number=number, cob_id=cob_id & 0x7FF,
                    enabled=not bool(cob_id & 0x80000000), raw_cob_id=cob_id,
                    transmission_type=self.read(communication, 2, "u8"), entries=entries,
                    payload_bytes=size, wire_bytes=fd_wire_length(size), is_fd=size > 8)

    def map_pdo(self, direction, number, entries, *, cob_id=None, transmission_type=255,
                event_ms=None, timeout_ms=None, enabled=True):
        communication, mapping = pdo_indices(direction, number)
        mapping_size(entries)
        if not entries:
            raise ValueError("Provide at least one PDO mapping entry")
        if cob_id is not None and not 1 <= cob_id <= 0x7FF:
            raise ValueError("PDO COB-ID must be within 1..0x7FF")
        if not (0 <= transmission_type <= 240 or transmission_type in (254, 255)):
            raise ValueError("Transmission type must be 0..240, 254 or 255")
        for value in (event_ms, timeout_ms):
            if value is not None and not 0 <= value <= 65535:
                raise ValueError("PDO timer must be within 0..65535 ms")
        if (direction == "rpdo" and event_ms is not None) or (direction == "tpdo" and timeout_ms is not None):
            raise ValueError("event-ms applies to TPDO; timeout-ms applies to RPDO")
        self.preoperational_disabled()
        original = self.read(communication, 1, "u32")
        configured = (original & 0x40000000) | (original & 0x7FF if cob_id is None else cob_id)
        self.write(communication, 1, "u32", original | 0x80000000)
        # A failure leaves the PDO disabled, with no attempt to enable a partial map.
        self.write(mapping, 0, "u8", 0)
        for sub, entry in enumerate(entries, 1):
            self.write(mapping, sub, "u32", entry)
        self.write(mapping, 0, "u8", len(entries))
        self.write(communication, 2, "u8", transmission_type)
        if event_ms is not None:
            self.write(communication, 5, "u16", event_ms)
        if timeout_ms is not None:
            self.write(communication, 5, "u16", timeout_ms)
        self.write(communication, 1, "u32", configured | (0 if enabled else 0x80000000))
        return self.pdo_configuration(direction, number)

    def send_pdo(self, number, data):
        configuration = self.pdo_configuration("rpdo", number)
        if not configuration["enabled"] or configuration["payload_bytes"] == 0:
            raise ValueError("RPDO must be enabled and mapped before sending")
        if len(data) != configuration["payload_bytes"]:
            raise ValueError(f"RPDO needs exactly {configuration['payload_bytes']} mapped bytes before padding")
        return self.send(configuration["cob_id"], data)

    def configure_can(self, *, nominal=None, data=None, save=False, reset=False):
        if nominal is not None and nominal not in NOMINAL_RATES:
            raise ValueError("Unsupported nominal bitrate")
        if data is not None and data not in DATA_RATES:
            raise ValueError("Unsupported data bitrate")
        if reset and not save:
            raise ValueError("--reset requires --save so pending settings are not discarded")
        if nominal is not None or data is not None or save:
            self.preoperational_disabled()
            if nominal is not None:
                self.write(0x2001, 2, "u32", nominal)
            if data is not None:
                self.write(0x2001, 3, "u32", data)
            if save:
                self.write(0x1010, 1, "u32", 0x65766173)
        result = dict(nominal=self.read(0x2001, 2, "u32"),
                      data=self.read(0x2001, 3, "u32"), brs=self.read(0x2001, 4, "u8"),
                      saved=save, reset_sent=reset)
        if reset:
            self.nmt(129)
        return result


def integer(value):
    return int(value, 0)


def bitrate(value):
    lower = value.lower()
    multiplier = {"k": 1000, "m": 1000000}.get(lower[-1:], 1)
    return int(lower[:-1] if multiplier != 1 else lower, 0) * multiplier


def frame_description(frame):
    return dict(cob_id=f"0x{frame.cob_id:03X}", format="fd" if frame.is_fd else "classic",
                brs=frame.brs, esi=frame.esi, length=len(frame.data), data=frame.data.hex())


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", default="can0")
    parser.add_argument("--node", type=int, choices=range(1, 128), default=1, metavar="1..127")
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser(
        "inspect", help="Read identification, drive status and telemetry; no writes"
    )
    for command in ("read", "write"):
        p = commands.add_parser(command)
        p.add_argument("index", type=integer)
        p.add_argument("sub", type=integer)
        p.add_argument("type", choices=TYPES)
        if command == "write":
            p.add_argument("value")
    commands.add_parser("nmt").add_argument("state", choices=("start", "stop", "preop", "reset"))
    commands.add_parser(
        "mode", help="Select a mode while disabled; does not enable the drive. position is multi-turn"
    ).add_argument("mode", choices=MODES)
    listing = commands.add_parser("pdo-list", help="Read PDO mappings and derived classic/FD frame sizes")
    listing.add_argument("direction", nargs="?", choices=("rpdo", "tpdo"))
    mapping = commands.add_parser("pdo-map", help="Enter Pre-operational, disable drive and configure one PDO")
    mapping.add_argument("direction", choices=("rpdo", "tpdo"))
    mapping.add_argument("number", type=int, choices=range(1, 5))
    mapping.add_argument("entries", type=integer, nargs="+", help="u32 index/sub/bits, e.g. 0x20030120")
    mapping.add_argument("--cob-id", type=integer)
    mapping.add_argument("--type", dest="transmission_type", type=int, default=255)
    mapping.add_argument("--event-ms", type=int, help="TPDO event timer")
    mapping.add_argument("--timeout-ms", type=int, help="RPDO timeout")
    mapping.add_argument("--disabled", action="store_true", help="Leave this PDO disabled after mapping")
    pdo = commands.add_parser("pdo-send", help="Send one mapped RPDO; >8 bytes uses FD+BRS with zero padding")
    pdo.add_argument("number", type=int, choices=range(1, 5))
    pdo.add_argument("payload", help="Mapped little-endian payload as hex; exclude DLC padding")
    can = commands.add_parser("can-config", help="Read pending rates, or stop drive and update; no Linux link changes")
    can.add_argument("--nominal", type=bitrate, choices=NOMINAL_RATES, help="125k, 250k, 500k or 1m")
    can.add_argument("--data", type=bitrate, choices=DATA_RATES, help="1m, 2m, 4m, 5m or 8m")
    can.add_argument("--save", action="store_true", help="Explicitly save all pending persistent settings")
    can.add_argument("--reset", action="store_true", help="Send NMT Reset Node after --save; rates then change")
    listen = commands.add_parser("listen", help="Receive standard classic/FD frames without sending")
    listen.add_argument("--id", dest="ids", type=integer, action="append", help="Accept this COB-ID; may repeat")
    listen.add_argument("--count", type=int, default=1)
    listen.add_argument("--seconds", type=float, default=2)
    args = parser.parse_args(argv)
    try:
        if args.command == "pdo-map":
            mapping_size(args.entries)
        elif args.command == "pdo-send":
            args.payload = bytes.fromhex(args.payload)
            fd_wire_length(len(args.payload))
        elif args.command == "can-config" and args.reset and not args.save:
            raise ValueError("--reset requires --save; use nmt reset separately for an already saved configuration")
        elif args.command == "listen":
            if not 1 <= args.count <= 1000000 or not math.isfinite(args.seconds) or not 0 < args.seconds <= 3600:
                raise ValueError("listen requires count 1..1000000 and seconds within (0,3600]")
            if args.ids and any(not 0 <= cob_id <= 0x7FF for cob_id in args.ids):
                raise ValueError("Listen IDs must be standard 11-bit IDs")
    except ValueError as error:
        parser.error(str(error))
    if args.command == "listen":
        deadline = time.monotonic() + args.seconds
        with open_can_socket(args.interface, filters=args.ids) as receiver:
            for _ in range(args.count):
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                try:
                    frame = recv_frame(receiver, timeout=remaining)
                except TimeoutError:
                    break
                print(json.dumps(frame_description(frame)))
        return
    client = SdoClient(args.interface, args.node)
    try:
        if args.command == "inspect":
            for index, sub, typ, name in [
                (0x1000, 0, "u32", "device_type"),
                (0x6041, 0, "u16", "statusword"),
                (0x603F, 0, "u16", "fault"),
                (0x6060, 0, "i8", "selected_mode"),
                (0x6061, 0, "i8", "mode"),
                (0x6064, 0, "f32", "position_rev"),
                (0x606C, 0, "f32", "velocity_rev_s"),
                (0x2202, 0, "i32", "bus_mV"),
                (0x2100, 2, "u8", "calibration_state"),
                (0x2100, 9, "i16", "mcu_temperature_deciC"),
                (0x2100, 10, "u32", "isr_max_cycles"),
                (0x2001, 2, "u32", "configured_nominal_bitrate"),
                (0x2001, 3, "u32", "configured_data_bitrate"),
                (0x2001, 4, "u8", "brs"),
            ]:
                value = client.read(index, sub, typ)
                print(
                    f"{name}: {hex(value) if name in ('device_type', 'statusword', 'fault') else value}"
                )
        elif args.command == "read":
            print(client.read(args.index, args.sub, args.type))
        elif args.command == "write":
            value = float(args.value) if args.type == "f32" else integer(args.value)
            if args.type == "f32" and not math.isfinite(value):
                raise ValueError("Motion values must be finite")
            client.write(args.index, args.sub, args.type, value)
            print("SDO write acknowledged")
        elif args.command == "mode":
            client.write(0x6060, 0, "i8", MODES[args.mode])
            print(f"Selected {args.mode} (0x6060 = {MODES[args.mode]}); drive enable is a separate command")
        elif args.command == "pdo-list":
            for direction in (args.direction,) if args.direction else ("rpdo", "tpdo"):
                for number in range(1, 5):
                    print(json.dumps(client.pdo_configuration(direction, number)))
        elif args.command == "pdo-map":
            print(json.dumps(client.map_pdo(
                args.direction, args.number, args.entries, cob_id=args.cob_id,
                transmission_type=args.transmission_type, event_ms=args.event_ms,
                timeout_ms=args.timeout_ms, enabled=not args.disabled,
            )))
        elif args.command == "pdo-send":
            print(json.dumps(frame_description(client.send_pdo(args.number, args.payload))))
        elif args.command == "can-config":
            result = client.configure_can(nominal=args.nominal, data=args.data, save=args.save, reset=args.reset)
            print(json.dumps(result))
            if args.nominal is not None or args.data is not None or args.save:
                print("Rates apply after a saved reboot. Linux interface configuration is unchanged.")
        else:
            client.nmt({"start": 1, "stop": 2, "preop": 128, "reset": 129}[args.state])
            print("NMT sent (NMT has no acknowledgement)")
    finally:
        client.close()


if __name__ == "__main__":
    main()
