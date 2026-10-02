"""Exercise Qt preview and the real child pipeline using temporary observations."""
import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from datetime import timedelta
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch
import re
import sys

from PyQt5.QtWidgets import QApplication
from station_downloader_qt import DownloaderWindow
import station_downloader_qt as gui
from test_wuh2_download_merge import DAY, header, observations
from test_broadcast_ephemeris import navigation_bytes

APP = QApplication.instance() or QApplication([])


def until(condition, timeout=30):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        APP.processEvents()
        if condition():
            return
        time.sleep(.01)
    raise AssertionError('Qt operation timed out')


class GuiTests(unittest.TestCase):
    def test_default_rover_folder_real_child_reuse_and_preserve_user_files(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            rover = root/'流动站.obs'
            rover.write_text(header(DAY+timedelta(seconds=10.3), DAY+timedelta(seconds=20.7)), encoding='ascii')
            daily = root/'WUH200CHN_R_20262660000_01D_01S_MO.rnx'
            daily.write_bytes(observations(86400))
            # These names used to collide with cleanup. All must survive.
            (root/'compressed').mkdir()
            (root/'compressed'/'user.txt').write_text('keep')
            (root/'rinex').mkdir()
            (root/'pipeline_summary.json').write_text('user metadata')
            window = DownloaderWindow(str(rover))
            try:
                until(lambda: window.valid_rover is not None)
                self.assertEqual(Path(window.output_edit.text()), root)
                self.assertIn('2026-09-23', window.time_label.text())
                self.assertIn('266', window.time_label.text())
                self.assertFalse(window.full_day_check.isChecked())
                window.advanced_group.setChecked(True)
                window.offline_check.setChecked(True)
                outcomes = []
                window.completed.connect(lambda ok, result: outcomes.append((ok, result)))
                window.start_download()
                self.assertFalse(window.start_button.isEnabled())
                until(lambda: len(outcomes) == 1)
                ok, result = outcomes[0]
                self.assertTrue(ok, window.log.toPlainText())
                self.assertEqual(result['downloaded_this_run'], 0)
                self.assertEqual(result['daily']['epochs'], 86400)
                self.assertEqual(result['session']['epochs'], 12)
                self.assertTrue(window.open_button.isEnabled())
                station_files = list(root.glob('WUH*.rnx'))
                self.assertEqual(len(station_files), 2)
                mtimes = {p.name:p.stat().st_mtime_ns for p in station_files}
                window.start_download()
                until(lambda: len(outcomes) == 2)
                self.assertTrue(outcomes[-1][0])
                self.assertEqual(mtimes, {p.name:p.stat().st_mtime_ns for p in station_files})
                self.assertTrue(rover.exists())
                self.assertTrue((root/'compressed'/'user.txt').exists())
                self.assertTrue((root/'rinex').is_dir())
                self.assertEqual((root/'pipeline_summary.json').read_text(), 'user metadata')
                self.assertTrue((root/'.wuh2_pipeline').is_dir())
                self.assertNotIn('broadcast',outcomes[-1][1])
                legacy = root/'brdc2660.26p'
                legacy.mkdir()
                nav = legacy/'BRDC00IGS_R_20262660000_01D_MN.rnx'
                nav.write_bytes(navigation_bytes())
                window.broadcast_check.setChecked(True)
                window.start_download()
                self.assertFalse(window.broadcast_check.isEnabled())
                until(lambda: len(outcomes)==3)
                self.assertTrue(outcomes[-1][0],window.log.toPlainText())
                self.assertEqual(outcomes[-1][1]['broadcast']['downloaded'],0)
                self.assertEqual(outcomes[-1][1]['broadcast']['path'],str(nav))
                self.assertIn('广播星历：复用已有文件',window.status_label.text())
                self.assertEqual(mtimes,{p.name:p.stat().st_mtime_ns for p in station_files})
                window.full_day_check.setChecked(True)
                window.start_download()
                until(lambda: len(outcomes)==4)
                self.assertTrue(outcomes[-1][0])
                self.assertTrue(outcomes[-1][1]['full_day_requested'])
                self.assertEqual(outcomes[-1][1]['required_segments'],96)
            finally:
                until(lambda: not window.running())
                window.close()

    def test_failure_offline_missing_day_enables_retry(self):
        with tempfile.TemporaryDirectory() as folder:
            rover = Path(folder)/'rover.obs'
            rover.write_text(header(DAY, DAY+timedelta(seconds=2)), encoding='ascii')
            window = DownloaderWindow(str(rover))
            try:
                until(lambda: window.valid_rover is not None)
                window.advanced_group.setChecked(True)
                window.offline_check.setChecked(True)
                outcomes = []
                window.completed.connect(lambda ok, result: outcomes.append((ok, result)))
                window.start_download()
                until(lambda: bool(outcomes))
                self.assertFalse(outcomes[0][0])
                self.assertIn('离线模式缺少必要文件', window.log.toPlainText())
                self.assertTrue(window.start_button.isEnabled())
                self.assertFalse(window.open_button.isEnabled())
                self.assertFalse(list(Path(folder).glob('WUH*.rnx')))
                self.assertTrue((Path(folder)/'.wuh2_work_WUH200CHN_20260923').exists())
            finally:
                until(lambda: not window.running())
                window.close()

    def test_invalid_and_cross_day_disable_start(self):
        with tempfile.TemporaryDirectory() as folder:
            rover = Path(folder)/'bad.obs'
            rover.write_text('invalid data')
            window = DownloaderWindow(str(rover))
            try:
                until(lambda: window.preview_thread.isFinished())
                APP.processEvents()
                self.assertFalse(window.start_button.isEnabled())
                self.assertIn('无法使用', window.time_label.text())
                rover.write_text(header(DAY, DAY+timedelta(days=1)), encoding='ascii')
                window.inspect_rover()
                until(lambda: window.preview_thread.isFinished())
                APP.processEvents()
                self.assertFalse(window.start_button.isEnabled())
                self.assertIn('跨日', window.time_label.text())
            finally:
                window.close()

    @unittest.skipUnless(os.name=='nt','Windows process-tree cancellation')
    def test_close_stops_worker_and_child_preserves_partial_and_completed_files(self):
        import ctypes
        from ctypes import wintypes
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            rover = root/'rover.obs'
            rover.write_text(header(DAY,DAY+timedelta(seconds=2)),encoding='ascii')
            (root/'valid.gz').write_bytes(b'completed download')
            worker = root/'wuh2_download_merge.py'
            worker.write_text("import subprocess,sys,time\nfrom pathlib import Path\n"
                              "Path(__file__).with_name('download.part').write_bytes(b'partial')\n"
                              "child=subprocess.Popen([sys.executable,'-c','import time; time.sleep(60)'])\n"
                              "print('CHILD_PID='+str(child.pid),flush=True)\ntime.sleep(60)\n",encoding='utf-8')
            window = DownloaderWindow(str(rover))
            try:
                until(lambda: window.valid_rover is not None)
                with patch.object(gui,'ROOT',root):
                    window.start_download()
                until(lambda: 'CHILD_PID=' in window.log.toPlainText())
                child_pid = int(re.search(r'CHILD_PID=(\d+)',window.log.toPlainText())[1])
                kernel = ctypes.WinDLL('kernel32',use_last_error=True)
                kernel.OpenProcess.argtypes = [wintypes.DWORD,wintypes.BOOL,wintypes.DWORD]
                kernel.OpenProcess.restype = wintypes.HANDLE
                kernel.WaitForSingleObject.argtypes = [wintypes.HANDLE,wintypes.DWORD]
                kernel.WaitForSingleObject.restype = wintypes.DWORD
                kernel.CloseHandle.argtypes = [wintypes.HANDLE]
                handle = kernel.OpenProcess(0x100000,False,child_pid)
                self.assertTrue(handle)
                try:
                    self.assertTrue(window.close())
                    self.assertFalse(window.running())
                    self.assertEqual(kernel.WaitForSingleObject(handle,1000),0)
                finally:
                    kernel.CloseHandle(handle)
                self.assertEqual((root/'valid.gz').read_bytes(),b'completed download')
                self.assertEqual((root/'download.part').read_bytes(),b'partial')
            finally:
                if window.running():
                    window.close()


if __name__ == '__main__':
    unittest.main()
