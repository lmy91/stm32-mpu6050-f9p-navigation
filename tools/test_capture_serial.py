"""Protocol-v3 tests for the headless serial capture tool."""

from __future__ import annotations

import pathlib
import tempfile
import unittest

from tools.capture_serial import (GNSS_COLUMNS, RAWX_COLUMNS, parse_gnss,
                                  create_session_directory, parse_rawx_header,
                                  parse_rawx_measurement)


class GnssProtocolTests(unittest.TestCase):
    def test_protocol_v3_fields_and_units(self) -> None:
        line = (
            "GNSS,2420,123000,1,987654321,3,25,195,0,2,"
            "399123456,1161234567,45000,1200,1800,10,20,-30,22,50,135"
        )
        row = parse_gnss(line.split(","))

        self.assertIsNotNone(row)
        assert row is not None
        self.assertEqual(len(row), len(GNSS_COLUMNS))
        self.assertEqual(row[8:11], [2, 1, 1])
        self.assertEqual(row[14:16], [1.2, 1.8])
        self.assertEqual(row[20], 0.05)

    def test_short_gnss_record_is_rejected(self) -> None:
        line = (
            "GNSS,2420,123000,1,3,25,399123456,1161234567,"
            "45000,10,20,-30,22,135"
        )
        row = parse_gnss(line.split(","))

        self.assertIsNone(row)

    def test_rawx_exact_hex_and_signal_decode(self) -> None:
        header = parse_rawx_header(
            "RAWX,2420,405EC00000000000,18,1,1,1,987654321".split(","))
        row = parse_rawx_measurement(
            ("RAWX_MEAS,3,19,0,0,417C9C3800000000,4038000000000000,"
             "C0200000,1500,45,2,3,4,3").split(","), header)
        self.assertIsNotNone(row)
        assert row is not None
        self.assertEqual(len(row), len(RAWX_COLUMNS))
        self.assertEqual(row[11:13], ["BDS_B1I_D1", 1561.098])
        self.assertEqual(row[13:16], [30_000_000.0, 24.0, -2.5])
        self.assertEqual(row[-4:], [1, 1, 0, 0])

    def test_glonass_channel_frequency(self) -> None:
        header = parse_rawx_header(
            "RAWX,2420,405EC00000000000,18,1,1,1,1".split(","))
        row = parse_rawx_measurement(
            ("RAWX_MEAS,6,7,0,8,0000000000000000,0000000000000000,"
             "00000000,0,0,0,15,0,1").split(","), header)
        assert row is not None
        self.assertEqual(row[11], "GLO_L1OF")
        self.assertEqual(row[12], 1602.5625)

    def test_session_directory_matches_qt_layout(self) -> None:
        with tempfile.TemporaryDirectory() as parent:
            root = pathlib.Path(parent)
            first = create_session_directory(root, "20260908180500")
            second = create_session_directory(root, "20260908180500")
            self.assertEqual(first.name, "20260908180500")
            self.assertEqual(second.name, "20260908180500_01")


if __name__ == "__main__":
    unittest.main()
