"""Protocol-v3 tests for the headless serial capture decoder."""

from __future__ import annotations

import unittest

from tools.capture_serial import GNSS_COLUMNS, parse_gnss


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


if __name__ == "__main__":
    unittest.main()
