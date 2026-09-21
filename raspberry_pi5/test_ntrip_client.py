"""Protocol tests for the Raspberry Pi NTRIP-to-STM32 bridge."""

import json
import pathlib
import tempfile
import unittest

from raspberry_pi5.ntrip_client import (BridgeFlow, F9pMsmAdapter, NtripConfig,
                                         NtripResponse, RtcmFrameParser, crc24q,
                                         parse_bridge_report, write_ntrip_control)


def frame(length=400, message=1077):
    payload = bytes([message >> 4, (message & 15) << 4]) + bytes(length - 2)
    data = b"\xd3" + length.to_bytes(2, "big") + payload
    return data + crc24q(data).to_bytes(3, "big")


def msm(message, epoch=1000, multiple=1, station=621):
    payload = ((message << (176 - 12)) | (station << (176 - 24)) |
               (epoch << (176 - 54)) | (multiple << (176 - 55))).to_bytes(22, "big")
    data = b"\xd3\x00\x16" + payload
    return data + crc24q(data).to_bytes(3, "big")


def report(ms=10000, rx=0, tx=0, dropped=0, errors=0):
    return [ms, 1, rx, tx, dropped, errors, 0, 0, 0, 0, 0, 0, 0xFFFFFFFF]


class NtripClientTest(unittest.TestCase):
    def test_crc_and_fragmented_rtcm(self):
        self.assertEqual(crc24q(b"123456789"), 0xCDE703)
        expected = frame(700)
        parser = RtcmFrameParser()
        actual = []
        for byte in expected:
            actual.extend(parser.feed(bytes([byte])))
        self.assertEqual(actual, [expected])

    def test_http_icy_and_chunked_streams(self):
        data = frame(40)
        for header in (b"HTTP/1.1 200 OK\r\nContent-Type: gnss/data\r\n\r\n",
                       b"ICY 200 OK\r\n"):
            decoder = NtripResponse()
            self.assertEqual(b"".join(decoder.feed(bytes([value]))
                                        for value in header + data), data)
        decoder = NtripResponse()
        wire = (b"HTTP/1.1 200 OK\r\nTransfer-Encoding: chunked\r\n\r\n" +
                f"{len(data):X}\r\n".encode() + data + b"\r\n0\r\n\r\n")
        self.assertEqual(b"".join(decoder.feed(wire[i:i+3])
                                  for i in range(0, len(wire), 3)), data)
        self.assertTrue(decoder.ended)

    def test_navic_terminator_is_removed_and_previous_mmi_closed(self):
        adapter = F9pMsmAdapter()
        gps, bds, navic = msm(1077), msm(1127), msm(1137, multiple=0)
        self.assertEqual(adapter.feed(gps, 1.0), [])
        self.assertEqual(adapter.feed(bds, 1.1), [])
        output = adapter.feed(navic, 1.2)
        self.assertEqual(len(output), 2)
        self.assertEqual(output[0], gps)
        self.assertEqual(output[1][9] & 2, 0)
        self.assertEqual(crc24q(output[1][:-3]), int.from_bytes(output[1][-3:], "big"))

    def test_bridge_report_and_credit_window(self):
        line = "#RTCM," + ",".join(map(str, report()))
        parsed = parse_bridge_report(line)
        self.assertEqual(parsed, report())
        flow = BridgeFlow(parsed)
        self.assertEqual(flow.allowance(), 1024)
        flow.record_write(200)
        self.assertEqual(flow.allowance(), 824)
        flow.update(report(ms=10020, rx=200, tx=200))
        self.assertEqual(flow.allowance(), 1024)

    def test_config_rejects_header_injection(self):
        good = NtripConfig.from_mapping({"host": "caster.example", "port": 2101,
                                         "mountpoint": "MOUNT", "username": "u",
                                         "password": "p"})
        self.assertEqual(good.mountpoint, "MOUNT")
        with self.assertRaises(ValueError):
            NtripConfig.from_mapping({"host": "caster.example\r\nX: bad",
                                      "mountpoint": "MOUNT"})

    def test_control_file_is_atomic_private_and_normalized(self):
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "ntrip.json"
            write_ntrip_control(path, {
                "host": "caster.example", "port": 2101,
                "mountpoint": "/MOUNT", "username": "user",
                "password": "secret",
            })
            value = json.loads(path.read_text(encoding="utf-8"))
            self.assertEqual(value["mountpoint"], "MOUNT")
            self.assertEqual(value["password"], "secret")
            self.assertEqual(path.stat().st_mode & 0o777, 0o600)
            self.assertEqual(list(path.parent.glob(".ntrip.json.*")), [])


if __name__ == "__main__":
    unittest.main()
