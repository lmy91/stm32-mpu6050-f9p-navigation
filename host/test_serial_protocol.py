import csv
import json
import os
import pathlib
import tempfile
import unittest
from dataclasses import fields, replace
from unittest import mock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt5 import QtWidgets
import numpy as np

from imu_serial_qt import NavigationMonitor, SelfAimConfigDialog
from fusion import GnssObservation, NavigationSolution, RealtimeSelfAim


RAWX_HEADER = b"RAWX,2420,405EC00000000000,18,1,1,1,987654321"
RAWX_MEAS = (
    b"RAWX_MEAS,3,19,0,0,417C9C3800000000,4038000000000000,"
    b"C0200000,1500,45,2,3,4,3"
)


class SerialProtocolTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self):
        self.monitor = NavigationMonitor()

    def tearDown(self):
        self.monitor.close()

    def test_startup_rawx_fragment_is_ignored(self):
        self.monitor.process_line(RAWX_MEAS)
        self.assertEqual(self.monitor.invalid_lines, 0)
        self.assertEqual(self.monitor.total_rawx, 0)

        self.monitor.process_line(RAWX_HEADER)
        self.monitor.process_line(RAWX_MEAS)
        self.assertEqual(self.monitor.invalid_lines, 0)
        self.assertEqual(self.monitor.total_rawx, 1)

    def test_missing_header_after_sync_is_invalid(self):
        self.monitor.process_line(RAWX_HEADER)
        self.monitor.process_line(b"RAWX_END,1")
        self.monitor.process_line(RAWX_MEAS)
        self.assertEqual(self.monitor.invalid_lines, 1)
        self.assertEqual(self.monitor.total_rawx, 0)

    def test_malformed_record_is_invalid(self):
        self.monitor.process_line(b"GNSS,not,a,complete,record")
        self.assertEqual(self.monitor.invalid_lines, 1)

    def test_select_all_logs(self):
        for checkbox in self.monitor.save_checkboxes.values():
            checkbox.setChecked(False)
        self.monitor.select_all_logs()
        self.assertTrue(all(checkbox.isChecked()
                            for checkbox in self.monitor.save_checkboxes.values()))
        self.assertEqual(self.monitor.select_all_button.text(), "一键取消")
        self.monitor.select_all_logs()
        self.assertFalse(any(checkbox.isChecked()
                             for checkbox in self.monitor.save_checkboxes.values()))
        self.assertEqual(self.monitor.select_all_button.text(), "一键存储")

    def test_self_aim_page_contains_ten_time_series_plots(self):
        self.assertEqual(len(self.monitor.aim_series_plots), 10)
        self.assertEqual(len(self.monitor.aim_series_curves), 30)
        self.assertEqual(len(self.monitor.aim_value_labels), 5)
        self.assertEqual(set(self.monitor.aim_series_curves),
                         set(self.monitor.aim_history) - {"time"})
        self.assertEqual(self.monitor.aim_series_groups, [
            ("pn", "pe", "pd"), ("vn", "ve", "vd"),
            ("roll", "pitch", "heading"), ("gbx", "gby", "gbz"),
            ("abx", "aby", "abz"), ("std_pn", "std_pe", "std_pd"),
            ("std_vn", "std_ve", "std_vd"),
            ("std_roll", "std_pitch", "std_heading"),
            ("std_gbx", "std_gby", "std_gbz"),
            ("std_abx", "std_aby", "std_abz")])

    def test_loose_navigation_page_has_map_and_ned_frd_curves(self):
        self.assertEqual(set(self.monitor.nav_curves),
                         {"vn", "ve", "vd", "roll", "pitch", "heading"})
        self.assertIsNot(self.monitor.fusion_map_widget, self.monitor.map_widget)
        self.assertEqual(set(self.monitor.nav_history),
                         {"time", "vn", "ve", "vd", "roll", "pitch", "heading"})

    def test_manual_fine_alignment_button_uses_algorithm_heading(self):
        monitor = self.monitor
        monitor.self_aim_config = replace(
            monitor.self_aim_config, initial_heading_deg=123.4567)
        monitor.self_aim = RealtimeSelfAim(monitor.self_aim_config)
        monitor.start_self_aim()
        self.assertTrue(monitor.self_aim.running)
        monitor.self_aim.update_gnss(GnssObservation(
            100.0, 30.0, 114.0, 20.0, np.zeros(3), True, 0.1, 0.1, 1.0))
        monitor.self_aim.update_imu(100.0, (0, 0, -9.80665), (0, 0, 0), 0.01)
        monitor.self_aim.update_imu(100.01, (0, 0, -9.80665), (0, 0, 0), 0.01)
        monitor.last_aim_solution = monitor.self_aim.solution()
        monitor.start_fine_alignment()
        self.assertEqual(monitor.self_aim.stage, "精对准")
        self.assertFalse(monitor.aim_manual_fine_button.isEnabled())
        self.assertFalse(hasattr(monitor, "aim_navigation_button"))
        monitor._update_aim_labels(monitor.self_aim.solution())
        self.assertEqual((monitor.aim_progress.minimum(), monitor.aim_progress.maximum()),
                         (0, 0))
        self.assertEqual(monitor.aim_stop_button.text(), "停止对准")
        self.assertTrue(monitor.nav_start_button.isEnabled())

    def test_aim_csv_header_matches_full_state_row(self):
        monitor = self.monitor
        for checkbox in monitor.save_checkboxes.values(): checkbox.setChecked(False)
        monitor.save_checkboxes["AIM"].setChecked(True)
        with tempfile.TemporaryDirectory() as directory:
            with (mock.patch.object(QtWidgets.QFileDialog, "getExistingDirectory",
                                    return_value=directory),
                  mock.patch("imu_serial_qt.time.strftime", return_value="20260910120000")):
                self.assertTrue(monitor._open_logs())
            monitor.self_aim.start()
            monitor.self_aim.update_gnss(GnssObservation(
                100.0, 30.0, 114.0, 20.0, np.zeros(3), True, 0.1, 0.1, 1.0))
            solution = monitor.self_aim.update_imu(
                100.0, (0, 0, -9.80665), (0, 0, 0), 0.01)
            monitor._consume_aim_solution(solution)
            monitor._close_logs()
            path = pathlib.Path(directory) / "20260910120000" / "aim.csv"
            with path.open(encoding="utf-8") as stream:
                rows = list(csv.reader(stream))
            self.assertEqual(len(rows), 2)
            self.assertEqual(len(rows[0]), len(rows[1]))
            self.assertEqual(len(rows[0]), 52)

    def test_nav_csv_header_matches_solution_row(self):
        monitor = self.monitor
        for checkbox in monitor.save_checkboxes.values(): checkbox.setChecked(False)
        monitor.save_checkboxes["NAV"].setChecked(True)
        with tempfile.TemporaryDirectory() as directory:
            with (mock.patch.object(QtWidgets.QFileDialog, "getExistingDirectory",
                                    return_value=directory),
                  mock.patch("imu_serial_qt.time.strftime", return_value="20260910120100")):
                self.assertTrue(monitor._open_logs())
            solution = NavigationSolution(
                100.0, 1.0, True, "组合导航运行", 30.0, 114.0, 20.0,
                (1.0, 2.0, 3.0), (0.1, 0.2, 0.3), 1.0, 2.0, 3.0,
                (0.01, 0.02, 0.03), (0.1, 0.2, 0.3), tuple(range(15)),
                1, 0, "已更新", 0.15, 15, 0.8, 2.0)
            monitor._consume_navigation_solution(solution)
            monitor._close_logs()
            path = pathlib.Path(directory) / "20260910120100" / "nav.csv"
            with path.open(encoding="utf-8") as stream:
                rows = list(csv.reader(stream))
            self.assertEqual(len(rows), 2)
            self.assertEqual(len(rows[0]), len(rows[1]))
            self.assertEqual(len(rows[0]), 46)

    def test_nav_same_epoch_keeps_only_latest_corrected_solution(self):
        monitor = self.monitor
        for checkbox in monitor.save_checkboxes.values(): checkbox.setChecked(False)
        monitor.save_checkboxes["NAV"].setChecked(True)
        with tempfile.TemporaryDirectory() as directory:
            with (mock.patch.object(QtWidgets.QFileDialog, "getExistingDirectory",
                                    return_value=directory),
                  mock.patch("imu_serial_qt.time.strftime", return_value="20260910120200")):
                self.assertTrue(monitor._open_logs())
            base = NavigationSolution(
                100.0, 1.0, True, "惯导递推", 30.0, 114.0, 20.0,
                (1.0, 2.0, 3.0), (0.1, 0.2, 0.3), 1.0, 2.0, 3.0,
                (0.01, 0.02, 0.03), (0.1, 0.2, 0.3), tuple(range(15)),
                0, 0, "等待GNSS", float("nan"), 0, 0.0, 2.0)
            corrected = replace(base, status="GNSS校正", velocity_ned_m_s=(4.0, 5.0, 6.0),
                                gnss_updates=1, last_gnss_reason="已更新")
            following = replace(corrected, gps_time_s=100.1, elapsed_s=1.1,
                                velocity_ned_m_s=(7.0, 8.0, 9.0))
            monitor._consume_navigation_solution(base)
            monitor._consume_navigation_solution(corrected)
            self.assertEqual(len(monitor.nav_history["time"]), 1)
            self.assertEqual(monitor.nav_history["vn"][-1], 4.0)
            monitor._consume_navigation_solution(following)
            monitor._close_logs()
            path = pathlib.Path(directory) / "20260910120200" / "nav.csv"
            with path.open(encoding="utf-8") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 2)
            self.assertEqual([float(row["gps_time_s"]) for row in rows], [100.0, 100.1])
            self.assertEqual(float(rows[0]["vel_n_m_s"]), 4.0)
            self.assertEqual(int(rows[0]["gnss_updates"]), 1)
            self.assertEqual(rows[0]["output_source"], "GNSS_REPLAY_CORRECTED")
            self.assertEqual(rows[1]["output_source"], "INS_PROPAGATION")

    def test_algorithm_session_saves_exact_effective_config_without_secrets(self):
        monitor = self.monitor
        for checkbox in monitor.save_checkboxes.values(): checkbox.setChecked(False)
        monitor.save_checkboxes["AIM"].setChecked(True)
        monitor.self_aim_config = replace(monitor.self_aim_config, output_rate_hz=7.5)
        with tempfile.TemporaryDirectory() as directory:
            with (mock.patch.object(QtWidgets.QFileDialog, "getExistingDirectory",
                                    return_value=directory),
                  mock.patch("imu_serial_qt.time.strftime", return_value="20260910120300")):
                self.assertTrue(monitor._open_logs())
            session = pathlib.Path(directory) / "20260910120300"
            metadata = json.loads((session / "session.json").read_text(encoding="utf-8"))
            self.assertEqual(metadata["serial_protocol"], "v3")
            self.assertEqual(metadata["navigation_frame"], "NED")
            self.assertEqual(metadata["body_frame"], "FRD")
            self.assertEqual(metadata["effective_self_aim_config"]["output_rate_hz"], 7.5)
            serialized = json.dumps(metadata).lower()
            self.assertNotIn("password", serialized)
            self.assertNotIn("amap", serialized)
            monitor._close_logs()
            closed = json.loads((session / "session.json").read_text(encoding="utf-8"))
            self.assertIn("closed_local", closed)

    def test_algorithm_dialog_exposes_all_configuration_groups(self):
        monitor = self.monitor
        dialog = SelfAimConfigDialog(monitor.self_aim_config,
                                     monitor.self_aim_config_path, monitor)
        config = dialog.current_config()
        self.assertEqual(dialog.tabs.count(), 6)
        represented = (set(dialog.scalar_widgets) | set(dialog.vector_widgets)
                       | set(dialog.bool_widgets)
                       | {"initial_heading_deg", "body_from_sensor"})
        self.assertEqual(represented, {field.name for field in fields(type(config))})
        self.assertEqual(config.coarse_alignment_seconds,
                         monitor.self_aim_config.coarse_alignment_seconds)
        self.assertEqual(config.body_from_sensor,
                         tuple(tuple(row) for row in monitor.self_aim_config.body_from_sensor))
        self.assertEqual(config.lever_arm_body_m,
                         tuple(monitor.self_aim_config.lever_arm_body_m))
        self.assertEqual(config.output_rate_hz, monitor.self_aim_config.output_rate_hz)
        dialog.close()

    def test_only_selected_log_is_created(self):
        for checkbox in self.monitor.save_checkboxes.values():
            checkbox.setChecked(False)
        self.monitor.save_checkboxes["GNSS"].setChecked(True)
        with tempfile.TemporaryDirectory() as directory:
            with (mock.patch.object(QtWidgets.QFileDialog, "getExistingDirectory",
                                    return_value=directory),
                  mock.patch("imu_serial_qt.time.strftime",
                             return_value="20260908180500")):
                self.assertTrue(self.monitor._open_logs())
            self.assertIsNone(self.monitor.imu_writer)
            self.assertIsNotNone(self.monitor.gnss_writer)
            self.assertIsNone(self.monitor.rawx_writer)
            self.monitor._close_logs()
            sessions = list(pathlib.Path(directory).iterdir())
            self.assertEqual(len(sessions), 1)
            self.assertTrue(sessions[0].is_dir())
            self.assertEqual(sessions[0].name, "20260908180500")
            self.assertEqual([path.name for path in sessions[0].iterdir()],
                             ["gnss.csv"])

    def test_session_directory_does_not_overwrite_same_second(self):
        with tempfile.TemporaryDirectory() as parent:
            root = pathlib.Path(parent)
            first = self.monitor._create_session_directory(root, "20260908180500")
            second = self.monitor._create_session_directory(root, "20260908180500")
            self.assertEqual(first.name, "20260908180500")
            self.assertEqual(second.name, "20260908180500_01")

    def test_log_only_saves_full_session_independent_of_display(self):
        m = self.monitor
        for name, checkbox in m.save_checkboxes.items(): checkbox.setChecked(name == "LOG")
        m.log_event("测试", "OLD_SESSION")
        with tempfile.TemporaryDirectory() as directory:
            with mock.patch.object(QtWidgets.QFileDialog, "getExistingDirectory", return_value=directory):
                self.assertTrue(m._open_logs())
            session = next(pathlib.Path(directory).iterdir())
            for i in range(5005): m.log_event("测试", f"CURRENT_{i}")
            self.assertNotIn("CURRENT_0\n", m.event_log.toPlainText())
            m.clear_event_log()
            m.log_event("测试", "AFTER_CLEAR")
            # Flush does not depend on any IMU/GNSS rows arriving.
            self.assertIn("AFTER_CLEAR", (session / "event.log").read_text(encoding="utf-8"))
            m._close_logs()
            text = (session / "event.log").read_text(encoding="utf-8")
            self.assertIn("CURRENT_0\n", text)
            self.assertIn("CURRENT_5004", text)
            self.assertNotIn("OLD_SESSION", text)
            self.assertIn("LOG 记录结束", text)
            self.assertEqual({p.name for p in session.iterdir()}, {"event.log"})
            self.assertIsNone(m.event_stream)

    def test_select_all_creates_session_files_and_protects_active_log(self):
        m = self.monitor
        self.assertFalse(m.save_checkboxes["LOG"].isChecked())
        m.select_all_logs()
        with tempfile.TemporaryDirectory() as directory:
            with mock.patch.object(QtWidgets.QFileDialog, "getExistingDirectory", return_value=directory):
                self.assertTrue(m._open_logs())
            session = next(pathlib.Path(directory).iterdir())
            path = session / "event.log"
            before = path.read_text(encoding="utf-8")
            with (mock.patch.object(QtWidgets.QFileDialog, "getSaveFileName", return_value=(str(path), "")),
                  mock.patch.object(QtWidgets.QMessageBox, "warning") as warning):
                m.export_event_log()
                warning.assert_called_once()
            self.assertEqual(before, path.read_text(encoding="utf-8"))
            m._close_logs()
            self.assertEqual({p.name for p in session.iterdir()},
                             {"imu.csv", "gnss.csv", "rawx.csv", "aim.csv", "nav.csv",
                              "event.log", "session.json"})

    def test_missing_f9p_firmware_counters_are_not_reported_as_real_zeros(self):
        report = [3, 1, 1000, 1000, 0, 0, 0, 0, 0, 0, 621, 1077, 100]
        self.assertIn("--/--/--", self.monitor._format_f9p_counters(report))
        report[7:10] = [10, 9, 0]
        self.assertIn("10/9/0", self.monitor._format_f9p_counters(report))

    def test_log_write_failure_does_not_recurse_or_close_csv(self):
        m = self.monitor
        stream = mock.Mock()
        stream.write.side_effect = OSError("disk full")
        m.event_stream = stream
        csv_stream = mock.Mock()
        m.imu_stream = csv_stream
        m.log_event("测试", "hello")
        self.assertIsNone(m.event_stream)
        self.assertIn("LOG 保存失败", m.event_log.toPlainText())
        csv_stream.close.assert_not_called()
        stream.write.assert_called_once()
        m.imu_stream = None

    def test_log_connection_failure_is_saved(self):
        m = self.monitor
        for name, checkbox in m.save_checkboxes.items(): checkbox.setChecked(name == "LOG")
        m.port_combo.clear(); m.port_combo.addItem("TEST", "TEST")
        with tempfile.TemporaryDirectory() as directory:
            with (mock.patch.object(QtWidgets.QFileDialog, "getExistingDirectory", return_value=directory),
                  mock.patch("imu_serial_qt.serial.Serial", side_effect=OSError("port unavailable")),
                  mock.patch.object(QtWidgets.QMessageBox, "critical")):
                m.connect_serial()
            path = next(pathlib.Path(directory).iterdir()) / "event.log"
            self.assertIn("port unavailable", path.read_text(encoding="utf-8"))
            self.assertIsNone(m.event_stream)

    def test_best_view_keeps_equal_axis_scale(self):
        self.monitor.map_widget.set_position(30.0, 114.0)
        self.monitor.map_widget.set_position(30.0001, 114.0002)
        self.monitor.map_widget.fit_track()
        aspect = self.monitor.map_widget.local_plot.getViewBox().state["aspectLocked"]
        self.assertEqual(aspect, 1.0)

    def test_track_auto_fits_when_new_point_reaches_edge(self):
        map_widget = self.monitor.map_widget
        map_widget.set_position(30.0, 114.0)
        map_widget.set_position(30.0001, 114.0001)
        map_widget.local_plot.setRange(xRange=(-0.5, 0.5),
                                       yRange=(-0.5, 0.5), padding=0.0)
        map_widget.set_position(30.0002, 114.0002)
        (x_min, x_max), (y_min, y_max) = map_widget.local_plot.viewRange()
        self.assertLess(x_min, min(map_widget.east_m))
        self.assertGreater(x_max, max(map_widget.east_m))
        self.assertLess(y_min, min(map_widget.north_m))
        self.assertGreater(y_max, max(map_widget.north_m))


if __name__ == "__main__":
    unittest.main()
