"""Qt front end. Downloads run in a separate process; credentials stay in memory."""
import codecs
from datetime import timedelta
import json
import os
from pathlib import Path
import re
import sys
import subprocess

from PyQt5.QtCore import QProcess, QProcessEnvironment, QThread, QUrl, Qt, pyqtSignal
from PyQt5.QtGui import QDesktopServices, QFont
from PyQt5.QtWidgets import (
    QApplication, QCheckBox, QFileDialog, QFormLayout, QGroupBox, QHBoxLayout,
    QLabel, QLineEdit, QMainWindow, QMessageBox, QPlainTextEdit, QProgressBar,
    QPushButton, QVBoxLayout, QWidget,
)

from wuh2_download_merge import rover_times, required_slots

ROOT = Path(__file__).resolve().parent


class PreviewThread(QThread):
    ready = pyqtSignal(str, object, object, str)

    def __init__(self, path, parent):
        super().__init__(parent)
        self.path = path

    def run(self):
        try:
            first, last = rover_times(Path(self.path),self.isInterruptionRequested)
            if first.date() != last.date() or last < first:
                raise ValueError('请选择同一个 GPS 日期内的观测文件；跨日文件请先按天分割。')
            end = last.replace(microsecond=0) + timedelta(seconds=int(last.microsecond > 0))
            if end.date() != first.date():
                raise ValueError('尾时刻需要次日历元，请先按天分割。')
            self.ready.emit(self.path, first, last, '')
        except Exception as error:
            self.ready.emit(self.path, None, None, str(error))


class DownloaderWindow(QMainWindow):
    # Exposed for automated checks and integration; no credentials in this signal.
    completed = pyqtSignal(bool, object)

    def __init__(self, rover=None):
        super().__init__()
        self.setWindowTitle('WUH2 · 1秒基站观测下载工具')
        self.resize(900, 780)
        self.setAcceptDrops(True)
        self.preview_thread = None
        self.valid_rover = None
        self.result = None
        self.process = QProcess(self)
        self.process.setProcessChannelMode(QProcess.MergedChannels)
        self.process.readyReadStandardOutput.connect(self.read_output)
        self.process.finished.connect(self.process_finished)
        self.process.errorOccurred.connect(self.process_error)
        self.process.started.connect(lambda: self.process.setProcessEnvironment(QProcessEnvironment()))
        self.decoder = codecs.getincrementaldecoder('utf-8')('replace')
        self.pending_output = ''
        self.failed_start = False
        self.closing = False
        self.rover_range = None

        content = QWidget()
        layout = QVBoxLayout(content)
        layout.setContentsMargins(24, 20, 24, 20)
        layout.setSpacing(12)
        title = QLabel('一键获取 WUH2 基站观测')
        title.setStyleSheet('font-size: 23px; font-weight: 600; color: #173b60;')
        layout.addWidget(title)
        subtitle = QLabel('选择流动站文件 → 自动识别 GPS 日期 → 下载与合并 → 提取匹配时段')
        layout.addWidget(subtitle)

        self.input_group = QGroupBox('1  选择流动站观测文件')
        form = QFormLayout(self.input_group)
        self.rover_edit = QLineEdit()
        self.rover_edit.setPlaceholderText('选择或拖入 RINEX 3 观测文件（.obs / .rnx / .26o）')
        self.rover_edit.editingFinished.connect(self.inspect_rover)
        browse = QPushButton('选择文件…')
        browse.clicked.connect(self.browse_rover)
        form.addRow('流动站文件', self.path_row(self.rover_edit, browse))
        self.time_label = QLabel('尚未选择文件')
        self.time_label.setWordWrap(True)
        self.time_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.time_label.setTextFormat(Qt.PlainText)
        self.time_label.setMinimumHeight(self.time_label.fontMetrics().lineSpacing()*2+8)
        form.addRow('自动识别', self.time_label)
        self.output_edit = QLineEdit()
        self.output_edit.setPlaceholderText('默认：流动站文件所在文件夹')
        output_browse = QPushButton('更改目录…')
        output_browse.clicked.connect(self.browse_output)
        form.addRow('保存位置', self.path_row(self.output_edit, output_browse))
        self.full_day_check = QCheckBox('下载全天观测（96个分段；默认只下载覆盖流动站时段的最少分段）')
        self.full_day_check.toggled.connect(self.refresh_mode)
        form.addRow('观测范围', self.full_day_check)
        self.broadcast_check = QCheckBox('同时下载当天广播星历（混合 GNSS，已有文件自动复用）')
        self.broadcast_check.setToolTip('下载流动站 GPS 日期对应的 BRDC00IGS 全天混合广播星历，用于 RTKLIB 后处理。')
        form.addRow('广播星历', self.broadcast_check)
        layout.addWidget(self.input_group)

        self.auth_group = QGroupBox('2  Earthdata 登录（本地数据完整时无需填写）')
        auth_form = QFormLayout(self.auth_group)
        self.username_edit = QLineEdit(os.environ.get('EARTHDATA_USERNAME', ''))
        self.username_edit.setPlaceholderText('Earthdata 用户名')
        self.password_edit = QLineEdit()
        self.password_edit.setEchoMode(QLineEdit.Password)
        self.password_edit.setPlaceholderText('密码只用于本次运行，不保存')
        auth_form.addRow('用户名', self.username_edit)
        auth_form.addRow('密码', self.password_edit)
        auth_hint = QLabel('首次下载请先在 Earthdata 网站完成 CDDIS 应用授权。')
        auth_hint.setWordWrap(True)
        auth_form.addRow(auth_hint)
        layout.addWidget(self.auth_group)

        self.advanced_group = QGroupBox('高级选项（可选）')
        self.advanced_group.setCheckable(True)
        self.advanced_group.setChecked(False)
        advanced_outer = QVBoxLayout(self.advanced_group)
        self.advanced_panel = QWidget()
        advanced_form = QFormLayout(self.advanced_panel)
        advanced_form.setContentsMargins(0, 0, 0, 0)
        advanced_outer.addWidget(self.advanced_panel)
        self.advanced_panel.setVisible(False)
        self.advanced_group.toggled.connect(self.advanced_panel.setVisible)
        self.offline_check = QCheckBox('只使用已有数据，不连接服务器')
        advanced_form.addRow(self.offline_check)
        self.converter_edit = QLineEdit()
        self.merger_edit = QLineEdit()
        for label, edit in [('CRX2RNX', self.converter_edit), ('GFZRNX', self.merger_edit)]:
            edit.setPlaceholderText('自动查找本机工具；其他电脑可选择程序路径')
            button = QPushButton('选择程序…')
            button.clicked.connect(lambda checked=False, field=edit: self.browse_tool(field))
            advanced_form.addRow(label, self.path_row(edit, button))
        layout.addWidget(self.advanced_group)

        actions = QHBoxLayout()
        self.start_button = QPushButton('一键下载 / 合并 / 提取')
        self.start_button.setMinimumHeight(42)
        self.start_button.setEnabled(False)
        self.start_button.clicked.connect(self.start_download)
        self.open_button = QPushButton('打开结果文件夹')
        self.open_button.setEnabled(False)
        self.open_button.clicked.connect(self.open_output)
        actions.addWidget(self.start_button, 2)
        actions.addWidget(self.open_button, 1)
        layout.addLayout(actions)
        self.progress = QProgressBar()
        self.progress.setRange(0, 100)
        self.progress.setValue(0)
        layout.addWidget(self.progress)
        self.status_label = QLabel('就绪：默认输出到流动站文件所在文件夹。')
        self.status_label.setTextFormat(Qt.PlainText)
        self.status_label.setWordWrap(True)
        layout.addWidget(self.status_label)
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMinimumHeight(150)
        self.log.setMaximumBlockCount(3000)
        layout.addWidget(self.log, 1)
        self.setCentralWidget(content)
        self.setStyleSheet('''
            QMainWindow { background: #f4f7fb; }
            QGroupBox { font-weight: 600; border: 1px solid #d4deea;
                        border-radius: 7px; margin-top: 12px; padding-top: 14px; }
            QGroupBox::title { subcontrol-origin: margin; left: 12px; }
            QLineEdit, QPlainTextEdit { background: white; border: 1px solid #c9d5e2;
                                     border-radius: 4px; padding: 7px; }
            QPushButton { padding: 7px 12px; }
            QProgressBar { border: 1px solid #d4deea; border-radius: 4px; text-align: center; }
            QProgressBar::chunk { background: #2879bd; }
        ''')
        if rover:
            self.rover_edit.setText(str(Path(rover).resolve()))
            self.inspect_rover()

    @staticmethod
    def path_row(edit, button):
        widget = QWidget()
        row = QHBoxLayout(widget)
        row.setContentsMargins(0, 0, 0, 0)
        row.addWidget(edit, 1)
        row.addWidget(button)
        return widget

    def running(self):
        return self.process.state() != QProcess.NotRunning

    def browse_rover(self):
        path, _ = QFileDialog.getOpenFileName(self, '选择流动站 RINEX 3 观测文件',
                                             self.rover_edit.text(),
                                             '观测文件 (*.obs *.rnx *.*o);;所有文件 (*)')
        if path:
            self.rover_edit.setText(path)
            self.inspect_rover()

    def browse_output(self):
        path = QFileDialog.getExistingDirectory(self, '选择结果保存文件夹', self.output_edit.text())
        if path:
            self.output_edit.setText(path)

    def browse_tool(self, edit):
        path, _ = QFileDialog.getOpenFileName(self, '选择工具程序', edit.text(), '程序 (*.exe);;所有文件 (*)')
        if path:
            edit.setText(path)

    def inspect_rover(self):
        if self.running():
            return
        # Keep the worker alive until finished; ignore stale preview results.
        if self.preview_thread and self.preview_thread.isRunning():
            return
        path = self.rover_edit.text().strip().strip('"')
        self.valid_rover = None
        self.rover_range = None
        self.start_button.setEnabled(False)
        self.open_button.setEnabled(False)
        if not path:
            self.time_label.setText('尚未选择文件')
            return
        self.time_label.setText('正在读取流动站日期和观测范围…')
        self.preview_thread = PreviewThread(path, self)
        self.preview_thread.ready.connect(self.preview_ready)
        self.preview_thread.finished.connect(self.preview_finished)
        self.preview_thread.start()

    def preview_ready(self, path, first, last, error):
        if self.closing:
            return
        current = self.rover_edit.text().strip().strip('"')
        if path != current:
            return
        if error:
            self.time_label.setText('无法使用此文件：' + error)
            self.status_label.setText('请重新选择有效的流动站观测文件。')
            return
        self.valid_rover = str(Path(path).resolve())
        self.rover_range = (first,last)
        self.output_edit.setText(str(Path(self.valid_rover).parent))
        self.time_label.setText(f'GPS 日期：{first:%Y-%m-%d}   年积日：{first:%j}   基站：WUH200CHN\n'
                                f'{first.strftime("%H:%M:%S.%f")[:-3]} ～ '
                                f'{last.strftime("%H:%M:%S.%f")[:-3]} (GPS)')
        self.refresh_mode()
        self.start_button.setEnabled(True)

    def preview_finished(self):
        if self.closing:
            return
        if self.preview_thread.path != self.rover_edit.text().strip().strip('"'):
            self.inspect_rover()

    def refresh_mode(self):
        if self.rover_range and not self.running():
            first,last = self.rover_range
            start = first.replace(microsecond=0)
            end = last.replace(microsecond=0)+timedelta(seconds=int(last.microsecond>0))
            count = len(required_slots(start,end,self.full_day_check.isChecked()))
            self.status_label.setText(f'已识别日期：本模式最多需要 {count} 个15分钟分段，已有数据自动复用。')

    def start_download(self):
        if self.running():
            return
        current = self.rover_edit.text().strip().strip('"')
        if not self.valid_rover or str(Path(current).resolve()) != self.valid_rover:
            self.inspect_rover()
            return
        destination = self.output_edit.text().strip()
        if not destination:
            destination = str(Path(self.valid_rover).parent)
            self.output_edit.setText(destination)
        arguments = ['-u', '-X', 'utf8', str(ROOT/'wuh2_download_merge.py'), self.valid_rover,
                     '--output', destination, '--non-interactive', '--json-summary']
        if self.broadcast_check.isChecked():
            arguments.append('--broadcast-ephemeris')
        if self.full_day_check.isChecked():
            arguments.append('--full-day')
        if self.advanced_group.isChecked():
            if self.offline_check.isChecked():
                arguments.append('--offline')
            for flag, field in [('--crx2rnx', self.converter_edit), ('--gfzrnx', self.merger_edit)]:
                if field.text().strip():
                    arguments.extend([flag, field.text().strip()])
        environment = QProcessEnvironment.systemEnvironment()
        environment.insert('PYTHONIOENCODING', 'utf-8')
        if self.username_edit.text().strip():
            environment.insert('EARTHDATA_USERNAME', self.username_edit.text().strip())
        if self.password_edit.text():
            environment.insert('EARTHDATA_PASSWORD', self.password_edit.text())
        self.process.setProcessEnvironment(environment)
        self.process.setWorkingDirectory(str(ROOT))
        self.result = None
        self.failed_start = False
        self.pending_output = ''
        self.decoder.reset()
        self.log.clear()
        self.progress.setRange(0, 0)
        self.status_label.setText('正在检查已有文件；缺少的数据将自动下载。')
        self.set_busy(True)
        self.process.start(sys.executable, arguments)
        self.password_edit.clear()

    def set_busy(self, busy):
        for group in (self.input_group, self.auth_group, self.advanced_group):
            group.setEnabled(not busy)
        self.start_button.setEnabled(not busy and bool(self.valid_rover))
        self.open_button.setEnabled(not busy and bool(self.result))

    def read_output(self):
        self.pending_output += self.decoder.decode(bytes(self.process.readAllStandardOutput()))
        while '\n' in self.pending_output:
            line, self.pending_output = self.pending_output.split('\n', 1)
            self.handle_line(line.rstrip('\r'))

    def handle_line(self, line):
        if line.startswith('WUH2_RESULT_JSON='):
            try:
                self.result = json.loads(line.split('=', 1)[1])
            except ValueError:
                self.result = None
            return
        self.log.appendPlainText(line)
        if line:
            self.status_label.setText(line)
        match = re.match(r'\[(\d+)/(\d+)\]', line)
        if match:
            self.progress.setRange(0, 100)
            self.progress.setValue(10 + int(int(match[1])*75/int(match[2])))
        elif '合并' in line or '提取' in line or '检查已有全天' in line or '广播星历' in line:
            self.progress.setRange(0, 0)

    def process_error(self, error):
        if error == QProcess.FailedToStart:
            self.failed_start = True
            self.finish(False, '无法启动 Python 下载程序：' + self.process.errorString())

    def process_finished(self, exit_code, exit_status):
        self.read_output()
        self.pending_output += self.decoder.decode(b'', final=True)
        if self.pending_output:
            self.handle_line(self.pending_output.rstrip('\r'))
            self.pending_output = ''
        if self.failed_start:
            return
        success = exit_code == 0 and exit_status == QProcess.NormalExit and bool(self.result)
        if success:
            info = self.result
            text = (f'完成：本次下载 {info["downloaded_this_run"]} 个分段；'
                    f'匹配时段 {info["session"]["epochs"]:,} 个历元。')
            if info.get('full_day_requested') and info.get('daily'):
                text += f' 全天 {info["daily"]["epochs"]:,} 个历元。'
            text += f'\n保存位置：{info["output_directory"]}'
            if info.get('broadcast'):
                navigation = info['broadcast']
                action = '下载并校验' if navigation['downloaded'] else '复用已有文件'
                text += f'\n广播星历：{action}，{navigation["records"]:,} 条记录。'
        else:
            self.result = None
            text = '未完成，请查看下方日志。已下载的有效分段保留，重试时自动复用。'
        self.finish(success, text)

    def finish(self, success, text):
        if self.closing:
            return
        self.process.setProcessEnvironment(QProcessEnvironment())
        self.progress.setRange(0, 100)
        self.progress.setValue(100 if success else 0)
        self.status_label.setText(text)
        self.set_busy(False)
        self.completed.emit(success, self.result)

    def open_output(self):
        if self.result:
            QDesktopServices.openUrl(QUrl.fromLocalFile(self.result['output_directory']))

    def dragEnterEvent(self, event):
        if not self.running() and event.mimeData().hasUrls() and len(event.mimeData().urls()) == 1:
            if event.mimeData().urls()[0].isLocalFile():
                event.acceptProposedAction()

    def dropEvent(self, event):
        if not self.running():
            self.rover_edit.setText(event.mimeData().urls()[0].toLocalFile())
            self.inspect_rover()

    def closeEvent(self, event):
        self.closing = True
        if self.preview_thread and self.preview_thread.isRunning():
            self.preview_thread.requestInterruption()
            self.preview_thread.wait(2000)
        if self.running():
            pid = int(self.process.processId())
            if os.name=='nt' and pid>0:
                try:
                    result = subprocess.run(['taskkill','/PID',str(pid),'/T','/F'],
                                   stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,
                                   creationflags=subprocess.CREATE_NO_WINDOW,timeout=3)
                    if result.returncode!=0:
                        self.process.kill()
                except (OSError,subprocess.TimeoutExpired):
                    self.process.kill()
            else:
                self.process.kill()
            self.process.waitForFinished(3000)
        event.accept()


def main():
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    app = QApplication(sys.argv)
    app.setFont(QFont('Microsoft YaHei UI', 10))
    window = DownloaderWindow(sys.argv[1] if len(sys.argv) > 1 else None)
    window.show()
    return app.exec_()


if __name__ == '__main__':
    raise SystemExit(main())
