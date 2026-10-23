#!/usr/bin/env python3
"""Offline SocketCAN ABI, PDO mapping and explicit communication-setting tests."""
import contextlib
import io
import struct
import unittest
from unittest.mock import patch

import rollercan_cli as cli


class FakeSocket:
    def __init__(self, replies=()):
        self.replies = iter(replies)
        self.sent = []
        self.read_sizes = []
        self.options = []
        self.timeouts = []
        self.timeout = 0.5
        self.closed = False

    def send(self, packet):
        self.sent.append(packet)
        return len(packet)

    def recv(self, size):
        self.read_sizes.append(size)
        return next(self.replies)

    def settimeout(self, timeout):
        self.timeout = timeout
        self.timeouts.append(timeout)

    def gettimeout(self):
        return self.timeout

    def setsockopt(self, *args):
        self.options.append(args)

    def bind(self, address):
        self.address = address

    def close(self):
        self.closed = True


class DictionaryClient(cli.SdoClient):
    def __init__(self):
        self.node, self.timeout = 1, 0.1
        self.events = []
        self.dictionary = {
            (0x6040, 0): 0, (0x6041, 0): 0x40,
            (0x2001, 2): 1000000, (0x2001, 3): 5000000, (0x2001, 4): 1,
            (0x1400, 1): 0x80000201, (0x1400, 2): 255,
            (0x1600, 0): 3, (0x1600, 1): 0x60FF0020,
            (0x1600, 2): 0x60400010, (0x1600, 3): 0x60600008,
        }
        self.fail_write = None

    def read(self, index, sub, typ):
        self.events.append(("read", index, sub, typ))
        return self.dictionary[index, sub]

    def write(self, index, sub, typ, value):
        self.events.append(("write", index, sub, typ, value))
        if (index, sub) == self.fail_write:
            raise RuntimeError("mapping abort")
        self.dictionary[index, sub] = value

    def nmt(self, command):
        self.events.append(("nmt", command))

    def send(self, cob_id, data, *, fd=None, brs=True):
        frame = cli.unpack_frame(cli.pack_frame(cob_id, data, fd=fd, brs=brs))
        self.events.append(("send", frame))
        return frame


class SocketCanTest(unittest.TestCase):
    def test_all_payload_lengths_choose_classic_or_fd_and_zero_padding(self):
        for size in range(65):
            with self.subTest(size=size):
                payload = bytes(range(size))
                packet = cli.pack_frame(0x201, payload)
                frame = cli.unpack_frame(packet)
                wire = cli.fd_wire_length(size)
                self.assertEqual(len(packet), 16 if size <= 8 else 72)
                self.assertEqual(frame.is_fd, size > 8)
                self.assertEqual(frame.brs, size > 8)
                self.assertEqual(frame.data, payload + b"\0" * (wire - size))

    def test_explicit_fd_for_small_frame_and_no_brs(self):
        frame = cli.unpack_frame(cli.pack_frame(0x181, b"12345678", fd=True, brs=False))
        self.assertTrue(frame.is_fd)
        self.assertFalse(frame.brs)
        self.assertEqual(len(frame.data), 8)

    def test_invalid_frames_are_rejected(self):
        for identifier in (-1, 0x800, 0x80000201):
            with self.assertRaises(ValueError):
                cli.pack_frame(identifier, b"")
        with self.assertRaises(ValueError):
            cli.pack_frame(0x201, bytes(65))
        with self.assertRaises(ValueError):
            cli.pack_frame(0x201, bytes(9), fd=False)
        for packet in (b"", bytes(17), cli.FRAME.pack(0x201, 9, bytes(8)),
                       cli.FD_FRAME.pack(0x201, 65, 1, bytes(64)),
                       cli.FRAME.pack(0x40000201, 0, bytes(8))):
            with self.assertRaises(ValueError):
                cli.unpack_frame(packet)

    def test_receiver_reads_full_fd_mtu_and_keeps_esi(self):
        packet = cli.FD_FRAME.pack(0x181, 12, 3, bytes(range(64)))
        sock = FakeSocket([packet])
        frame = cli.recv_frame(sock, timeout=0.25)
        self.assertEqual(sock.read_sizes, [72])
        self.assertTrue(frame.esi and frame.brs and frame.is_fd)
        self.assertEqual(frame.data, bytes(range(12)))

    def test_receiver_skips_extended_remote_error_and_malformed_frames(self):
        invalid = [cli.FRAME.pack(flag | 0x581, 8, bytes(8))
                   for flag in (0x80000000, 0x40000000, 0x20000000)]
        invalid += [bytes(15), cli.FRAME.pack(0x581, 9, bytes(8)),
                    cli.FD_FRAME.pack(0x581, 65, 1, bytes(64))]
        sock = FakeSocket([*invalid, cli.pack_frame(0x581, b"accepted")])
        self.assertEqual(cli.recv_frame(sock).data, b"accepted")
        self.assertEqual(sock.read_sizes, [72] * 7)

    def test_invalid_traffic_cannot_extend_the_receive_deadline(self):
        sock = FakeSocket([cli.FRAME.pack(0x20000004, 0, bytes(8))] * 10)
        with patch.object(cli.time, "monotonic", side_effect=[100, 100.1, 100.3, 100.5]):
            with self.assertRaisesRegex(TimeoutError, "deadline"):
                cli.recv_frame(sock, timeout=0.5)
        self.assertEqual(len(sock.read_sizes), 2)
        self.assertAlmostEqual(sock.timeouts[0], 0.4)
        self.assertAlmostEqual(sock.timeouts[1], 0.2)

    def test_fd_option_and_standard_filters_are_set_before_bind(self):
        sock = FakeSocket()
        with patch.multiple(cli.socket, AF_CAN=29, CAN_RAW=1, SOL_CAN_RAW=101, CAN_RAW_FILTER=1, create=True), \
             patch.object(cli.socket, "socket", return_value=sock):
            result = cli.open_can_socket("can0", filters=iter([0x181, 0x581]))
        self.assertIs(result, sock)
        self.assertIn((101, cli.CAN_RAW_FD_FRAMES, 1), sock.options)
        self.assertIn((101, 1, struct.pack("=IIII", 0x181, 0xC00007FF, 0x581, 0xC00007FF)), sock.options)
        self.assertEqual(sock.address, ("can0",))

    def test_accept_all_standard_filter_does_not_select_error_receive_list(self):
        sock = FakeSocket()
        with patch.multiple(cli.socket, AF_CAN=29, CAN_RAW=1, SOL_CAN_RAW=101, CAN_RAW_FILTER=1, create=True), \
             patch.object(cli.socket, "socket", return_value=sock):
            cli.open_can_socket("can0")
        self.assertIn((101, 1, struct.pack("=II", 0, 0xC0000000)), sock.options)

    def test_sdo_remains_classic_and_rejects_matching_fd_response(self):
        reply = struct.pack("<BHBf", 0x43, 0x6064, 0, 1.25)
        sock = FakeSocket([
            cli.FRAME.pack(0x20000004, 0, bytes(8)),
            bytes(15),
            cli.pack_frame(0x581, reply, fd=True),
            cli.pack_frame(0x582, reply),
            cli.pack_frame(0x581, reply),
        ])
        client = cli.SdoClient.__new__(cli.SdoClient)
        client.node, client.timeout, client.socket = 1, 0.5, sock
        self.assertEqual(client.read(0x6064, 0, "f32"), 1.25)
        self.assertEqual(sock.read_sizes, [72] * 5)
        self.assertEqual(len(sock.sent[0]), 16)
        self.assertEqual(cli.unpack_frame(sock.sent[0]).cob_id, 0x601)
        client.nmt(128)
        self.assertEqual(len(sock.sent[-1]), 16)


class PdoAndConfigurationTest(unittest.TestCase):
    def test_mapping_limits(self):
        self.assertEqual(cli.mapping_size([0x30000320] * 16), 64)
        for mapping in ([0x30000108] * 17, [0x30000320] * 15 + [0x30000328],
                        [0x30000109], [0x30000100], [-1], [0x100000000]):
            with self.assertRaises(ValueError):
                cli.mapping_size(mapping)

    def test_mapping_failure_keeps_pdo_disabled(self):
        client = DictionaryClient()
        client.fail_write = (0x1600, 2)
        with self.assertRaisesRegex(RuntimeError, "mapping abort"):
            client.map_pdo("rpdo", 1, [0x20030120, 0x20030220, 0x20030320])
        self.assertTrue(client.dictionary[0x1400, 1] & 0x80000000)
        self.assertEqual(client.dictionary[0x1600, 0], 0)
        self.assertEqual(client.events[0], ("nmt", 128))

    def test_long_mapping_and_send_add_only_dlc_padding(self):
        client = DictionaryClient()
        entries = [0x20030120, 0x20030220, 0x20030320, 0x20030410,
                   0x20030510, 0x60400010, 0x60600008]
        result = client.map_pdo("rpdo", 1, entries)
        self.assertEqual(result["payload_bytes"], 19)
        self.assertEqual(result["wire_bytes"], 20)
        self.assertTrue(result["enabled"] and result["is_fd"])
        frame = client.send_pdo(1, bytes(range(19)))
        self.assertEqual(frame.data, bytes(range(19)) + b"\0")
        self.assertTrue(frame.is_fd and frame.brs)
        with self.assertRaisesRegex(ValueError, "exactly 19"):
            client.send_pdo(1, bytes(20))

    def test_short_pdo_stays_classic(self):
        client = DictionaryClient()
        client.dictionary[0x1400, 1] = 0x201
        frame = client.send_pdo(1, bytes(7))
        self.assertFalse(frame.is_fd)
        self.assertEqual(len(frame.data), 7)

    def test_disabled_pdo_cannot_be_sent(self):
        client = DictionaryClient()
        with self.assertRaisesRegex(ValueError, "enabled"):
            client.send_pdo(1, bytes(7))
        self.assertFalse(any(event[0] == "send" for event in client.events))

    def test_can_config_read_is_read_only(self):
        client = DictionaryClient()
        result = client.configure_can()
        self.assertEqual(result, dict(nominal=1000000, data=5000000, brs=1, saved=False, reset_sent=False))
        self.assertTrue(all(event[0] == "read" for event in client.events))

    def test_can_config_stops_then_writes_and_explicitly_saves_before_reset(self):
        client = DictionaryClient()
        client.configure_can(nominal=500000, data=2000000, save=True, reset=True)
        self.assertEqual(client.events[0], ("nmt", 128))
        writes = [event for event in client.events if event[0] == "write"]
        self.assertEqual(writes, [("write", 0x6040, 0, "u16", 0),
                                  ("write", 0x2001, 2, "u32", 500000),
                                  ("write", 0x2001, 3, "u32", 2000000),
                                  ("write", 0x1010, 1, "u32", 0x65766173)])
        self.assertEqual(client.events[-1], ("nmt", 129))

    def test_invalid_rates_or_unsaved_reset_do_not_send_anything(self):
        for options in (dict(nominal=800000), dict(data=3000000), dict(reset=True)):
            client = DictionaryClient()
            with self.assertRaises(ValueError):
                client.configure_can(**options)
            self.assertEqual(client.events, [])

    def test_invalid_cli_inputs_do_not_open_socket(self):
        for argv in (["can-config", "--reset"], ["can-config", "--data", "3m"],
                     ["pdo-send", "1", "nothex"], ["listen", "--seconds", "nan"]):
            with self.subTest(argv=argv), patch.object(cli, "SdoClient") as client:
                with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                    cli.main(argv)
                client.assert_not_called()


if __name__ == "__main__":
    unittest.main()
