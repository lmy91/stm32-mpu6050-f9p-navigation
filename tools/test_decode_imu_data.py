"""Protocol-v3 input tests for the offline IMU decoder."""

from __future__ import annotations

import unittest

from tools.decode_imu_data import parse_named, parse_typed


class ImuProtocolTests(unittest.TestCase):
    def test_typed_v3_record(self) -> None:
        row = "IMU,42,2420,123456789,1,987654321,1,2,3,4,5,6,7".split(",")
        sample = parse_typed(row)

        self.assertIsNotNone(sample)
        assert sample is not None
        self.assertEqual(sample["gps_tow_us"], 123456789)
        self.assertEqual(sample["timer_us"], 987654321)

    def test_named_v3_record(self) -> None:
        header = [
            "sample", "gps_week", "gps_tow_us", "time_valid", "timer_us",
            "ax_raw", "ay_raw", "az_raw", "temp_raw", "gx_raw", "gy_raw", "gz_raw",
        ]
        row = ["42", "2420", "123456789", "1", "987654321",
               "1", "2", "3", "4", "5", "6", "7"]

        sample = parse_named(row, header)
        self.assertIsNotNone(sample)
        assert sample is not None
        self.assertEqual(sample["gps_week"], 2420)

    def test_named_input_requires_v3_time_fields(self) -> None:
        header = [
            "sample", "timer_us", "ax_raw", "ay_raw", "az_raw", "temp_raw",
            "gx_raw", "gy_raw", "gz_raw",
        ]
        row = ["42", "987654321", "1", "2", "3", "4", "5", "6", "7"]

        self.assertIsNone(parse_named(row, header))


if __name__ == "__main__":
    unittest.main()
