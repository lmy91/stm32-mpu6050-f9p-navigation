import os
import unittest

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


if __name__ == "__main__":
    unittest.main()
