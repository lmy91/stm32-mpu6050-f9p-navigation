"""Protocol-v3 tests for the headless serial capture tool."""

from __future__ import annotations

import pathlib
import tempfile
import unittest

from tools.capture_serial import (CsvRecorder, F9pUbxTap, GNSS_COLUMNS, IMU_COLUMNS,
                                  RAWX_COLUMNS, imu_live_sample, parse_gnss, parse_imu,
                                  create_session_directory, parse_rawx_header,
                                  parse_rawx_measurement, parse_satellite,
                                  parse_satellite_end)


class GnssProtocolTests(unittest.TestCase):
    def test_live_imu_payload_uses_deg_s(self) -> None:
        parsed = parse_imu(
            "IMU,7,2435,123456000,1,999000,16384,0,-16384,0,131,-262,0".split(","),
            None, None)
        self.assertIsNotNone(parsed)
        assert parsed is not None
        live = imu_live_sample(parsed[0])
        self.assertAlmostEqual(live["ax_m_s2"], 9.80665)
        self.assertAlmostEqual(live["az_m_s2"], -9.80665)
        self.assertAlmostEqual(live["gx_deg_s"], 1.0)
        self.assertAlmostEqual(live["gy_deg_s"], -2.0)

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

    def test_recorder_can_start_and_stop_without_closing_source(self) -> None:
        with tempfile.TemporaryDirectory() as parent:
            recorder = CsvRecorder(pathlib.Path(parent), {"imu", "gnss"})
            session = recorder.start()
            self.assertTrue(recorder.active)
            self.assertIsNotNone(session)
            recorder.write("imu", [0] * len(IMU_COLUMNS))
            recorder.write("gnss", [0] * len(GNSS_COLUMNS))
            recorder.stop()
            self.assertFalse(recorder.active)
            assert session is not None
            self.assertTrue((session / "imu.csv").is_file())
            self.assertTrue((session / "gnss.csv").is_file())
            self.assertFalse((session / "rawx.csv").exists())

    def test_ubx_tap_writes_exact_binary_bytes_into_same_session(self) -> None:
        with tempfile.TemporaryDirectory() as parent:
            session = pathlib.Path(parent) / "20260912120000"
            session.mkdir()
            tap = F9pUbxTap("unused-test-port", 115200)
            path = tap.start_recording(session)
            data = b"\xb5\x62\x02\x15\x00\x00\x17\x4a"
            tap._record_data(data)  # pylint: disable=protected-access
            self.assertEqual(tap.snapshot()["recorded_bytes"], len(data))
            tap.stop_recording()
            self.assertEqual(path.read_bytes(), data)
            self.assertFalse(tap.active)

    def test_satellite_epoch_rows_for_live_sky_plot(self) -> None:
        satellite = parse_satellite(
            "SAT,2435,123000,1,3,19,47,35,-20,1".split(","))
        self.assertEqual(satellite, {
            "gps_week": 2435, "gps_tow_ms": 123000, "time_valid": 1,
            "gnss_id": 3, "sv_id": 19, "cno_dbhz": 47,
            "elev_deg": 35, "azim_deg": 340, "used": 1,
        })
        self.assertEqual(parse_satellite_end(
            "SAT_END,2435,123000,1,25".split(",")), (2435, 123000, 1, 25))
        self.assertIsNone(parse_satellite(
            "SAT,2435,123000,1,3,19,47,91,0,1".split(",")))


if __name__ == "__main__":
    unittest.main()
