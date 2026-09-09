import os
import pathlib
import tempfile
import unittest
from unittest import mock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt5 import QtWidgets

from imu_serial_qt import NavigationMonitor


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

    def test_select_all_creates_four_files_and_protects_active_log(self):
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
                             {"imu.csv", "gnss.csv", "rawx.csv", "event.log"})

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
