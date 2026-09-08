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

    def test_best_view_keeps_equal_axis_scale(self):
        self.monitor.map_widget.set_position(30.0, 114.0)
        self.monitor.map_widget.set_position(30.0001, 114.0002)
        self.monitor.map_widget.fit_track()
        aspect = self.monitor.map_widget.local_plot.getViewBox().state["aspectLocked"]
        self.assertEqual(aspect, 1.0)


if __name__ == "__main__":
    unittest.main()
