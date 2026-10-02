import csv
import contextlib
import io
import tempfile
import unittest
from pathlib import Path

import decode_rawx


COLUMNS = [
    "gps_week", "rcv_tow_s", "rx_timer_us", "leap_s", "rec_stat",
    "epoch_num_meas", "epoch_total_meas", "gnss_id", "sv_id", "sig_id",
    "freq_id", "signal", "frequency_mhz", "pseudorange_m",
    "carrier_phase_cycles", "doppler_hz", "locktime_ms", "cno_dbhz",
    "pr_stdev_m", "cp_stdev_cycles", "do_stdev_hz", "pr_valid",
    "cp_valid", "half_cycle", "sub_half_cycle",
]


def row(week=2435, tow=100.0, timer=1, count=1, gnss=0, sv=1, sig=0,
        freq=0, cp=1000.0, doppler=10.0, lock=1000, cp_valid=1):
    values = {
        "gps_week": week, "rcv_tow_s": tow, "rx_timer_us": timer,
        "leap_s": 18, "rec_stat": 1, "epoch_num_meas": count,
        "epoch_total_meas": count, "gnss_id": gnss, "sv_id": sv,
        "sig_id": sig, "freq_id": freq, "signal": "", "frequency_mhz": "",
        "pseudorange_m": 20000000.0, "carrier_phase_cycles": cp,
        "doppler_hz": doppler, "locktime_ms": lock, "cno_dbhz": 40,
        "pr_stdev_m": 0.1, "cp_stdev_cycles": 0.01, "do_stdev_hz": 0.1,
        "pr_valid": 1, "cp_valid": cp_valid, "half_cycle": 1,
        "sub_half_cycle": 0,
    }
    return {key: values[key] for key in COLUMNS}


class SignalTests(unittest.TestCase):
    def test_constellation_specific_signal_ids(self):
        self.assertEqual(decode_rawx.signal_name(0, 0), "GPS_L1CA")
        self.assertEqual(decode_rawx.signal_name(3, 0), "BDS_B1I_D1")
        self.assertEqual(decode_rawx.signal_name(2, 6), "GAL_E5bQ")

    def test_glonass_frequency_channel(self):
        name, frequency = decode_rawx.signal_info(6, 0, 8)
        self.assertEqual(name, "GLO_L1OF")
        self.assertAlmostEqual(frequency, 1602.5625)


class EpochTests(unittest.TestCase):
    def write_rows(self, rows):
        temporary = tempfile.TemporaryDirectory()
        path = Path(temporary.name) / "rawx.csv"
        with path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=COLUMNS)
            writer.writeheader()
            writer.writerows(rows)
        return temporary, path

    def test_incomplete_epoch_is_reported(self):
        temporary, path = self.write_rows([row(count=2)])
        self.addCleanup(temporary.cleanup)
        issues = decode_rawx.validate_epochs(decode_rawx.decode(path))
        self.assertEqual(len(issues), 1)
        self.assertIn("声明 2 条，实际 1 条", issues[0]["reasons"])

    def test_week_rollover_is_continuous(self):
        rows = [row(week=2435, tow=604799.0, timer=1, cp=1000.0),
                row(week=2436, tow=0.0, timer=2, cp=990.0, lock=2000)]
        temporary, path = self.write_rows(rows)
        self.addCleanup(temporary.cleanup)
        with contextlib.redirect_stdout(io.StringIO()):
            result = decode_rawx.analyze(path)
        self.assertEqual(result["span_s"], 1.0)
        self.assertEqual(result["non_1hz_intervals"], 0)


class SlipTests(unittest.TestCase):
    def test_invalid_carrier_breaks_continuity(self):
        epochs = [
            ((2435, 100.0, 1), [row(tow=100.0, cp=1000.0)]),
            ((2435, 101.0, 2), [row(tow=101.0, cp=0.0, cp_valid=0)]),
            ((2435, 102.0, 3), [row(tow=102.0, cp=2000.0, lock=3000)]),
        ]
        self.assertEqual(dict(decode_rawx.detect_cycle_slips(epochs)), {})

    def test_phase_doppler_outlier_and_lock_reset_are_separate(self):
        epochs = [
            ((2435, 100.0, 1), [row(tow=100.0, cp=1000.0, lock=1000)]),
            ((2435, 101.0, 2), [row(tow=101.0, cp=992.0, lock=2000)]),
            ((2435, 102.0, 3), [row(tow=102.0, cp=982.0, lock=100)]),
        ]
        events = next(iter(decode_rawx.detect_cycle_slips(epochs).values()))
        self.assertEqual([event["kind"] for event in events],
                         ["phase_doppler_outlier", "lock_reset"])


if __name__ == "__main__":
    unittest.main()
