"""Single owner of the full-duplex UART; no widgets or file I/O in this worker."""
import queue
import threading
import time
from collections import deque

from PyQt5 import QtCore
from ntrip_rtcm import BridgeFlow, RTCM_BACKLOG_WARN_SECONDS


class SerialWorker(QtCore.QThread):
    failed = QtCore.pyqtSignal(str)
    rtcm_failed = QtCore.pyqtSignal(object, str)
    diagnostic = QtCore.pyqtSignal(str)

    def __init__(self, port, parent=None):
        super().__init__(parent)
        self.port = port
        self.received = queue.Queue(maxsize=512)  # each chunk <=4096 bytes: 2 MiB
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.requested_source = None
        self.source = None
        self.flow = None
        self.pending = deque()
        self.report = None
        self.report_at = 0.0
        self.line_buffer = bytearray()
        self.snapshot = (0, 0, 0.0)  # sent, unacknowledged, maximum send-queue age
        self.max_send_age = 0.0
        self.timing_snapshot = (0.0, 0.0, 0.0)  # max assembly, send wait, total host residence
        self.backlog_warning_active = False
        self.write_stall_since = None

    def _capture_snapshot(self):
        now = time.monotonic()
        maxima = self.timing_snapshot
        for item in self.pending:
            maxima = tuple(max(a, b) for a, b in zip(maxima, item.ages(now)))
        self.timing_snapshot = maxima
        self.max_send_age = maxima[1]
        if self.flow is not None:
            self.snapshot = (self.flow.sent, self.flow.sent - self.flow.acked, self.max_send_age)

    def _warn_slow_backlog(self, now):
        """Report burst backlog once, without mistaking it for a dead link."""
        if not self.pending:
            self.backlog_warning_active = False
            return
        assembly, sending, total = self.pending[0].ages(now)
        if sending > RTCM_BACKLOG_WARN_SECONDS:
            if not self.backlog_warning_active:
                flow = self.flow
                outstanding = flow.sent - flow.acked if flow is not None else 0
                self.diagnostic.emit(
                    f"预警：RTCM本地排队 {sending:.3f}s（阈值{RTCM_BACKLOG_WARN_SECONDS:g}s），"
                    f"但STM32状态/转发确认保护未触发，继续下发；"
                    f"组包={assembly:.3f}s 总驻留={total:.3f}s "
                    f"未确认={outstanding}B 待发={sum(len(item.data) for item in self.pending)}B")
                self.backlog_warning_active = True
        elif sending < RTCM_BACKLOG_WARN_SECONDS / 2:
            self.backlog_warning_active = False

    def set_source(self, source):
        with self.lock: self.requested_source = source

    def stop(self):
        self.stop_event.set()

    def take_received(self, max_bytes=262144):
        result = bytearray()
        while len(result) < max_bytes:
            try: result.extend(self.received.get_nowait())
            except queue.Empty: break
        return result

    def _stop_forwarding(self, message):
        source = self.source
        self._capture_snapshot()
        if source is not None:
            now = time.monotonic()
            flow = self.flow
            ages = self.pending[0].ages(now) if self.pending else (0, 0, 0)
            queued, peak, queue_age = source.frames.snapshot()
            self.diagnostic.emit(
                f"停止前即时现场：{message}；已写/未确认={self.snapshot[0]}/{self.snapshot[1]}B "
                f"当前帧组包/发送等待/总驻留={ages[0]:.3f}/{ages[1]:.3f}/{ages[2]:.3f}s "
                f"串口待发={sum(len(item.data) for item in self.pending)}B "
                f"网络待发={queued}B/{queue_age:.3f}s 峰值={peak}B "
                f"距ACK报告={now-self.report_at:.3f}s "
                f"距转发进展={now-flow.progress_at if flow else 0:.3f}s "
                f"STM32即时报告={self.report}（本地计时，非差分龄期）")
        with self.lock:
            if self.requested_source is source: self.requested_source = None
        self.source = None; self.flow = None; self.pending.clear()
        if source is not None:
            source.stop()
            self.rtcm_failed.emit(source, message)

    def _read(self):
        waiting = self.port.in_waiting
        if not waiting: return
        data = self.port.read(min(waiting, 4096))
        if not data: return
        try: self.received.put_nowait(data)
        except queue.Full as error:
            raise OSError("界面/记录处理停顿，接收队列已达 2 MiB；停止采集以免静默丢数据") from error
        self.line_buffer.extend(data)
        while True:
            end = self.line_buffer.find(b"\n")
            if end < 0: break
            line = bytes(self.line_buffer[:end]).strip()
            del self.line_buffer[:end + 1]
            if line.startswith(b"#RTCM,"):
                try:
                    report = [int(v) for v in line.split(b",")[1:]]
                    if len(report) != 13 or any(v < 0 or v > 0xFFFFFFFF for v in report):
                        raise ValueError
                except ValueError:
                    continue  # GUI reports malformed lines; old ACK cannot grant credit
                self.report = report; self.report_at = time.monotonic()
                if self.flow is not None:
                    try: self.flow.update(report)
                    except OSError as error: self._stop_forwarding(str(error))
            elif line.startswith(b"# booting"):
                self.report = None
                self._stop_forwarding("STM32 重启，请等待就绪后重新连接")
        if len(self.line_buffer) > 65536:
            raise OSError("串口长时间没有换行，请检查波特率/协议")

    def _pump(self):
        with self.lock: requested = self.requested_source
        if requested is not self.source:
            self.source = requested; self.flow = None; self.pending.clear()
            self.backlog_warning_active = False
            self.write_stall_since = None
            self.max_send_age = 0.0
            self.timing_snapshot = (0.0, 0.0, 0.0)
            if requested is not None:
                if self.report is None or time.monotonic() - self.report_at > 1:
                    self._stop_forwarding("STM32 转发状态尚未就绪/过期")
                    return
                try: self.flow = BridgeFlow(self.report)
                except ValueError as error:
                    self._stop_forwarding(str(error)); return
        if self.source is None or self.flow is None: return
        try:
            credit = self.flow.allowance()
            # Assemble across frame boundaries; the credit cap also bounds
            # the number of loop iterations, including a burst of tiny frames.
            size = sum(len(item.data) for item in self.pending)
            while size < credit:
                item = self.source.take_item()
                if not item.data: break
                self.pending.append(item)
                size += len(item.data)
            if self.pending:
                self._capture_snapshot()
                now = time.monotonic()
                self._warn_slow_backlog(now)
                if credit:
                    if (self.write_stall_since is not None
                            and now - self.write_stall_since > RTCM_BACKLOG_WARN_SECONDS):
                        raise OSError(
                            f"串口连续 {now-self.write_stall_since:.3f} 秒未写入 RTCM，已停止下发")
                    batch = b"".join(item.data for item in self.pending)[:credit]
                    count = len(batch)
                    written = self.port.write(batch)
                    if not 0 <= written <= count: raise OSError("串口返回无效发送字节数")
                    if written:
                        self.write_stall_since = None
                    elif self.write_stall_since is None:
                        self.write_stall_since = now
                    self.flow.record_write(written)
                    remaining = written
                    while remaining:
                        item = self.pending.popleft()
                        if remaining < len(item.data):
                            self.pending.appendleft(item._replace(data=item.data[remaining:]))
                            break
                        remaining -= len(item.data)
            else:
                self.backlog_warning_active = False
                self.write_stall_since = None
            self.snapshot = (self.flow.sent, self.flow.sent - self.flow.acked, self.max_send_age)
        except (OSError, ValueError) as error:
            # On ambiguous write failure never replay a possibly written prefix.
            self._stop_forwarding(str(error))

    def run(self):
        try:
            while not self.stop_event.is_set():
                self._read()
                self._pump()
                self.stop_event.wait(0.002)
        except (OSError, ValueError) as error:
            self.failed.emit(str(error))
            self._stop_forwarding(str(error))
        finally:
            self.port.close()
