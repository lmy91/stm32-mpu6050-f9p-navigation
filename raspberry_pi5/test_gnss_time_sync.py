import unittest

from raspberry_pi5.gnss_time_sync import (GPS_EPOCH_UNIX_S, GPS_WEEK_S,
                                           gnss_utc_unix_s)


class GnssTimeSyncTests(unittest.TestCase):
    def test_converts_gps_time_and_extrapolates_state_age(self):
        state = {
            "gnss": {"gps_week": 2436, "gps_tow_ms": 100_250,
                     "time_valid": 1},
            "gps_utc_leap_seconds": 18,
            "gnss_updated_monotonic_s": 50.0,
        }
        actual = gnss_utc_unix_s(state, 50.75)
        expected = GPS_EPOCH_UNIX_S + 2436 * GPS_WEEK_S + 100.250 - 18 + 0.75
        self.assertAlmostEqual(actual, expected)

    def test_rejects_stale_state(self):
        state = {
            "gnss": {"gps_week": 2436, "gps_tow_ms": 100_000,
                     "time_valid": 1},
            "gps_utc_leap_seconds": 18,
            "gnss_updated_monotonic_s": 10.0,
        }
        with self.assertRaisesRegex(ValueError, "stale"):
            gnss_utc_unix_s(state, 20.0)

    def test_rejects_unconfirmed_gnss_time(self):
        state = {
            "gnss": {"gps_week": 2436, "gps_tow_ms": 100_000,
                     "time_valid": 0},
            "gps_utc_leap_seconds": 18,
            "gnss_updated_monotonic_s": 10.0,
        }
        with self.assertRaisesRegex(ValueError, "not valid"):
            gnss_utc_unix_s(state, 10.1)


if __name__ == "__main__":
    unittest.main()
