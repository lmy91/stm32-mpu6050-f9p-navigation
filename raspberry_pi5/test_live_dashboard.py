import csv
import json
import pathlib
import tempfile
import time
import unittest
from unittest import mock

from raspberry_pi5.live_dashboard import (GnssStore, newest_gnss_file, normalize_gnss,
                                           normalize_imu, normalize_satellites)


COLUMNS = [
    "gps_week", "gps_tow_ms", "time_valid", "rx_timer_us", "fix", "num_sv",
    "flags", "flags2", "carr_soln", "gnss_fix_ok", "diff_soln", "lat_deg", "lon_deg",
    "hmsl_m", "h_acc_m", "v_acc_m", "vel_n_m_s", "vel_e_m_s", "vel_d_m_s",
    "ground_speed_m_s", "s_acc_m_s", "pdop",
]


class LiveDashboardTest(unittest.TestCase):
    def test_normalize_live_imu(self):
        value = normalize_imu({
            "sample": 123, "gps_week": 2435, "gps_tow_us": 456000000,
            "time_valid": 1, "timer_us": 987654321,
            "ax_m_s2": 0.1, "ay_m_s2": -0.2, "az_m_s2": 9.81,
            "temp_deg_c": 31.25,
            "gx_deg_s": 0.01, "gy_deg_s": -0.02, "gz_deg_s": 0.03,
        })
        self.assertIsNotNone(value)
        assert value is not None
        self.assertEqual(value["sample"], 123)
        self.assertEqual(value["temp_deg_c"], 31.25)

    def test_normalize_fixed_solution(self):
        row = dict.fromkeys(COLUMNS, "0")
        row.update(gps_week="2435", gps_tow_ms="123400", fix="3", carr_soln="2",
                   lat_deg="30.5", lon_deg="114.3", num_sv="25", pdop="1.2")
        value = normalize_gnss(row)
        self.assertEqual(value["fix_text"], "3D / RTK固定")
        self.assertEqual(value["num_sv"], 25)

    def test_newest_valid_session_and_latest_row(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            (root / "not-a-session").mkdir()
            for name, tow in (("20260911100000", 1000), ("20260911100100", 2000)):
                session = root / name
                session.mkdir()
                with (session / "gnss.csv").open("w", newline="", encoding="utf-8") as stream:
                    writer = csv.DictWriter(stream, fieldnames=COLUMNS)
                    writer.writeheader()
                    row = dict.fromkeys(COLUMNS, 0)
                    row.update(gps_week=2435, gps_tow_ms=tow, fix=3, carr_soln=1,
                               lat_deg=30.5, lon_deg=114.3)
                    writer.writerow(row)
            self.assertEqual(newest_gnss_file(root).parent.name, "20260911100100")
            result = GnssStore(root).latest()
            self.assertTrue(result["ok"])
            self.assertEqual(result["data"]["gps_tow_ms"], 2000)
            self.assertLess(result["age_s"], 1.0)

    def test_missing_data_is_offline(self):
        with tempfile.TemporaryDirectory() as directory:
            result = GnssStore(pathlib.Path(directory)).latest()
            self.assertFalse(result["ok"])
            self.assertFalse(result["online"])

    def test_latest_row_is_read_from_large_file_tail(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            session = root / "20260911120000"
            session.mkdir()
            with (session / "gnss.csv").open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=COLUMNS)
                writer.writeheader()
                for tow in range(1000):
                    row = dict.fromkeys(COLUMNS, 0)
                    row.update(gps_week=2435, gps_tow_ms=tow * 1000, fix=3,
                               carr_soln=2, lat_deg=30.5, lon_deg=114.3)
                    writer.writerow(row)
            result = GnssStore(root).latest()
            self.assertEqual(result["data"]["gps_tow_ms"], 999000)

    def test_live_state_distinguishes_service_and_recording(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            state_file = root / "live.json"
            control_file = root / "recording.enabled"
            gnss = dict.fromkeys(COLUMNS, 0)
            gnss.update(gps_week=2435, gps_tow_ms=123000, fix=3, carr_soln=2,
                        lat_deg=30.5, lon_deg=114.3)
            now_ms = round(time.time() * 1000)
            state_file.write_text(json.dumps({
                "service_active": True, "recording": False,
                "updated_unix_ms": now_ms, "gnss_updated_unix_ms": now_ms,
                "imu_updated_unix_ms": now_ms,
                "imu": {"sample": 100, "gps_week": 2435, "gps_tow_us": 123000000,
                        "time_valid": 1, "timer_us": 1000000,
                        "ax_m_s2": 0.1, "ay_m_s2": 0.2, "az_m_s2": 9.8,
                        "temp_deg_c": 32.0,
                        "gx_deg_s": 0.01, "gy_deg_s": 0.02, "gz_deg_s": 0.03},
                "session": None, "gnss": gnss, "counts": {"imu": 100},
            }), encoding="utf-8")
            store = GnssStore(root, state_file, control_file)
            result = store.latest()
            self.assertTrue(result["service_running"])
            self.assertFalse(result["recording"])
            self.assertTrue(result["online"])
            self.assertTrue(result["imu_online"])
            self.assertEqual(result["imu"]["sample"], 100)
            self.assertTrue(store.set_recording(True)["ok"])
            self.assertTrue(control_file.exists())
            self.assertTrue(store.set_recording(False)["ok"])
            self.assertFalse(control_file.exists())

    def test_ntrip_config_is_volatile_and_password_is_not_returned(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            ntrip_file = root / "ntrip.json"
            store = GnssStore(root, ntrip_file=ntrip_file)
            result = store.set_ntrip({"host": "caster.example", "port": 2101,
                                      "mountpoint": "/TEST", "username": "user",
                                      "password": "secret"})
            self.assertTrue(result["ok"])
            self.assertNotIn("secret", json.dumps(result))
            saved = json.loads(ntrip_file.read_text(encoding="utf-8"))
            self.assertEqual(saved["mountpoint"], "TEST")
            self.assertEqual(saved["password"], "secret")
            self.assertTrue(store.set_ntrip(None)["ok"])
            self.assertFalse(ntrip_file.exists())

    def test_pi_local_map_config_is_validated(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            config = root / "amap.json"
            store = GnssStore(root, map_config_file=config)
            self.assertFalse(store.map_config()["configured"])
            config.write_text(json.dumps({
                "key": "a" * 32, "securityJsCode": "b" * 32,
            }), encoding="utf-8")
            result = store.map_config()
            self.assertTrue(result["configured"])
            self.assertEqual(result["key"], "a" * 32)
            config.write_text('{"key":"bad value"}', encoding="utf-8")
            self.assertFalse(store.map_config()["configured"])

    def test_live_satellites_are_validated_for_sky_plot(self):
        result = normalize_satellites([
            {"gnss_id": 0, "sv_id": 3, "cno_dbhz": 45, "elev_deg": 30,
             "azim_deg": 120, "used": 1},
            {"gnss_id": 3, "sv_id": 19, "cno_dbhz": 40, "elev_deg": 91,
             "azim_deg": 20, "used": 1},
        ])
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0]["sv_id"], 3)

    def test_service_console_is_allowlisted(self):
        with tempfile.TemporaryDirectory() as directory:
            store = GnssStore(pathlib.Path(directory))
            self.assertTrue(store.run_console_command("help")["ok"])
            result = store.run_console_command("status; rm -rf /")
            self.assertFalse(result["ok"])
            self.assertIn("不支持", result["output"])

    def test_service_console_controls_recording_and_ntrip_without_secrets(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            recording = root / "recording.enabled"
            ntrip = root / "ntrip.json"
            store = GnssStore(root, control_file=recording, ntrip_file=ntrip)
            self.assertTrue(store.run_console_command("record start")["ok"])
            self.assertTrue(recording.exists())
            self.assertTrue(store.run_console_command("record stop")["ok"])
            self.assertFalse(recording.exists())
            self.assertTrue(store.set_ntrip({
                "host": "caster.example", "port": 2101, "mountpoint": "TEST",
                "username": "user", "password": "top-secret",
            })["ok"])
            result = store.run_console_command("base reconnect")
            self.assertTrue(result["ok"])
            self.assertNotIn("top-secret", json.dumps(result))
            self.assertTrue(store.run_console_command("base disconnect")["ok"])
            self.assertFalse(ntrip.exists())

    def test_service_console_refuses_logger_restart_while_recording(self):
        with tempfile.TemporaryDirectory() as directory:
            store = GnssStore(pathlib.Path(directory))
            with mock.patch.object(store, "latest", return_value={"recording": True}):
                result = store.run_console_command("service restart logger")
            self.assertFalse(result["ok"])
            self.assertIn("record stop", result["output"])


if __name__ == "__main__":
    unittest.main()
