#!/usr/bin/env python3
"""Small Linux SocketCAN SDO client. No dependencies; commands are explicit."""
import argparse
import math
import socket
import struct
import time

TYPES = {"u8": "B", "i8": "b", "u16": "H", "i16": "h", "u32": "I", "i32": "i", "f32": "f"}
FRAME = struct.Struct("=IB3x8s")


class SdoClient:
    def __init__(self, interface, node, timeout=0.5):
        if not hasattr(socket, "AF_CAN"):
            raise SystemExit(
                "This client requires Linux SocketCAN (or a SocketCAN interface forwarded to WSL)."
            )
        self.node = node
        self.timeout = timeout
        self.socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        # Exact standard data response; reject extended frames and RTR.
        self.socket.setsockopt(
            socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, struct.pack("=II", 0x580 + node, 0xC00007FF)
        )
        self.socket.bind((interface,))

    def close(self):
        self.socket.close()

    def send(self, cob_id, data):
        self.socket.send(FRAME.pack(cob_id, len(data), data.ljust(8, b"\0")))

    def request(self, index, sub, cs, payload=b""):
        data = struct.pack("<BHB", cs, index, sub) + payload.ljust(4, b"\0")
        self.send(0x600 + self.node, data)
        deadline = time.monotonic() + self.timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(f"SDO {index:04X}:{sub:02X} timed out")
            self.socket.settimeout(remaining)
            cob_id, size, reply = FRAME.unpack(self.socket.recv(FRAME.size))
            if cob_id != 0x580 + self.node or size != 8 or reply[1:4] != data[1:4]:
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
        self.send(0, bytes([state, self.node]))


def integer(value):
    return int(value, 0)


def main():
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
    args = parser.parse_args()
    client = SdoClient(args.interface, args.node)
    try:
        if args.command == "inspect":
            for index, sub, typ, name in [
                (0x1000, 0, "u32", "device_type"),
                (0x6041, 0, "u16", "statusword"),
                (0x603F, 0, "u16", "fault"),
                (0x6061, 0, "i8", "mode"),
                (0x6064, 0, "f32", "position_rev"),
                (0x606C, 0, "f32", "velocity_rev_s"),
                (0x2202, 0, "i32", "bus_mV"),
                (0x2100, 2, "u8", "calibration_state"),
                (0x2100, 9, "i16", "mcu_temperature_deciC"),
                (0x2100, 10, "u32", "isr_max_cycles"),
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
        else:
            client.nmt({"start": 1, "stop": 2, "preop": 128, "reset": 129}[args.state])
            print("NMT sent (NMT has no acknowledgement)")
    finally:
        client.close()


if __name__ == "__main__":
    main()
