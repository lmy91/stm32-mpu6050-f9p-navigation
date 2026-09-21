"""Standard-library NTRIP/RTCM client for the Raspberry Pi serial owner.

The network thread validates and adapts whole RTCM3 frames but never touches
the UART. The serial-owning capture loop drains frames through ``NtripController``
using the STM32 ``#RTCM`` credit reports.
"""

from __future__ import annotations

import base64
import json
import os
import pathlib
import queue
import socket
import tempfile
import threading
import time
import urllib.parse
from dataclasses import dataclass


def crc24q(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
    return crc & 0xFFFFFF


class RtcmFrameParser:
    def __init__(self) -> None:
        self.buffer = bytearray()
        self.total_bytes = self.frame_count = self.crc_errors = self.resync_bytes = 0
        self.latest_type: int | None = None

    def feed(self, data: bytes) -> list[bytes]:
        self.buffer.extend(data)
        self.total_bytes += len(data)
        frames = []
        while True:
            start = self.buffer.find(b"\xd3")
            if start < 0:
                self.resync_bytes += len(self.buffer)
                self.buffer.clear()
                return frames
            if start:
                self.resync_bytes += start
                del self.buffer[:start]
            if len(self.buffer) < 3:
                return frames
            if self.buffer[1] & 0xFC:
                self.resync_bytes += 1
                del self.buffer[:1]
                continue
            length = ((self.buffer[1] & 3) << 8) | self.buffer[2]
            total = length + 6
            if len(self.buffer) < total:
                return frames
            frame = bytes(self.buffer[:total])
            if crc24q(frame[:-3]) == int.from_bytes(frame[-3:], "big"):
                payload = frame[3:-3]
                self.latest_type = ((payload[0] << 4) | (payload[1] >> 4)
                                    if len(payload) >= 2 else 0)
                self.frame_count += 1
                frames.append(frame)
                del self.buffer[:total]
            else:
                self.crc_errors += 1
                self.resync_bytes += 1
                del self.buffer[:1]


class NtripResponse:
    """Incremental HTTP/ICY and HTTP chunk decoder."""

    def __init__(self) -> None:
        self.buffer = bytearray()
        self.headers_done = self.chunked = self.need_crlf = self.ended = False
        self.remaining: int | None = None

    def feed(self, data: bytes) -> bytes:
        if self.ended:
            return b""
        self.buffer.extend(data)
        buf = self.buffer
        if not self.headers_done:
            icy = buf.startswith(b"ICY 200 OK\r\n")
            end = buf.find(b"\r\n") if icy else buf.find(b"\r\n\r\n")
            if end < 0:
                if len(buf) > 16384:
                    raise OSError("NTRIP响应头过大")
                return b""
            header = bytes(buf[:end])
            first = header.split(b"\r\n", 1)[0].split()
            if len(first) < 2 or first[1] != b"200":
                code = first[1].decode("ascii", "replace") if len(first) > 1 else "?"
                raise OSError(f"NTRIP HTTP {code}，请检查账号、挂载点或网络")
            if not (first[0].startswith(b"HTTP/") or icy):
                raise OSError("服务器未返回NTRIP数据流")
            headers = {}
            for line in header.split(b"\r\n")[1:]:
                key, separator, value = line.partition(b":")
                if separator:
                    headers[key.strip().lower()] = value.strip().lower()
            encoding = headers.get(b"transfer-encoding", b"")
            if encoding not in (b"", b"identity", b"chunked"):
                raise OSError("不支持的NTRIP传输编码")
            if headers.get(b"content-encoding", b"identity") != b"identity":
                raise OSError("不支持压缩的NTRIP流")
            content = headers.get(b"content-type", b"")
            if b"sourcetable" in content or b"text/html" in content:
                raise OSError("收到站点列表或网页，而不是RTCM")
            self.chunked = encoding == b"chunked"
            del buf[:end + (2 if icy else 4)]
            self.headers_done = True
        if not self.chunked:
            output = bytes(buf)
            buf.clear()
            return output
        output = bytearray()
        while True:
            if self.need_crlf:
                if len(buf) < 2:
                    break
                if buf[:2] != b"\r\n":
                    raise OSError("NTRIP chunk结尾错误")
                del buf[:2]
                self.need_crlf = False
            if self.remaining is None:
                end = buf.find(b"\r\n")
                if end < 0:
                    if len(buf) > 1024:
                        raise OSError("NTRIP chunk头过大")
                    break
                token = bytes(buf[:end]).split(b";", 1)[0]
                if not token or any(c not in b"0123456789abcdefABCDEF" for c in token):
                    raise OSError("NTRIP chunk长度错误")
                self.remaining = int(token, 16)
                if self.remaining > 16 * 1024 * 1024:
                    raise OSError("NTRIP chunk超过限制")
                del buf[:end + 2]
                if not self.remaining:
                    self.ended = True
                    buf.clear()
                    break
            count = min(len(buf), self.remaining)
            output.extend(buf[:count])
            del buf[:count]
            self.remaining -= count
            if self.remaining:
                break
            self.remaining = None
            self.need_crlf = True
        return bytes(output)


class F9pMsmAdapter:
    """Remove unsupported NavIC and close the last F9P-supported MSM frame."""

    def __init__(self) -> None:
        self.pending: list[tuple[float, bytes]] = []
        self.epochs: dict[int, int] = {}
        self.station: int | None = None
        self.bytes = self.filtered = self.rewritten = self.dropped_groups = 0

    def reset(self) -> None:
        self.pending.clear()
        self.epochs.clear()
        self.station = None
        self.bytes = 0

    @staticmethod
    def _header(frame: bytes) -> tuple[int, int, int, int, int]:
        message = (frame[3] << 4) | (frame[4] >> 4)
        family = message // 10
        payload = frame[3:-3]
        number = int.from_bytes(payload, "big")
        station = (number >> (len(payload) * 8 - 24)) & 4095
        epoch = (number >> (len(payload) * 8 - 54)) & ((1 << 30) - 1)
        multiple = (payload[6] >> 1) & 1
        return message, family, station, epoch, multiple

    def _release(self) -> list[bytes]:
        output = [data for _, data in self.pending
                  if ((data[3] << 4) | (data[4] >> 4)) // 10 != 113]
        self.filtered += len(self.pending) - len(output)
        if output and output[-1][9] & 2:
            modified = bytearray(output[-1][:-3])
            modified[9] &= ~2
            output[-1] = bytes(modified) + crc24q(modified).to_bytes(3, "big")
            self.rewritten += 1
        self.reset()
        return output

    def feed(self, frame: bytes, timestamp: float) -> list[bytes]:
        message = (frame[3] << 4) | (frame[4] >> 4)
        family = message // 10
        if not (107 <= family <= 113 and 1 <= message % 10 <= 7):
            return [frame]
        if len(frame) < 28:
            self.dropped_groups += 1
            self.reset()
            return []
        _, _, station, epoch, multiple = self._header(frame)
        outgoing = []
        if self.pending and station != self.station:
            self.dropped_groups += 1
            self.reset()
        elif self.pending and family in self.epochs and self.epochs[family] != epoch:
            outgoing = self._release()
        self.station = station
        self.epochs[family] = epoch
        self.pending.append((timestamp, frame))
        self.bytes += len(frame)
        if self.bytes > 65536:
            self.dropped_groups += 1
            self.reset()
            return outgoing
        return outgoing if multiple else outgoing + self._release()


@dataclass(frozen=True)
class NtripConfig:
    host: str
    port: int
    mountpoint: str
    username: str
    password: str

    @classmethod
    def from_mapping(cls, value: dict[str, object]) -> "NtripConfig":
        host = str(value.get("host", "")).strip()
        mountpoint = str(value.get("mountpoint", "")).strip().lstrip("/")
        username = str(value.get("username", ""))
        password = str(value.get("password", ""))
        try:
            port = int(value.get("port", 2101))
        except (TypeError, ValueError) as error:
            raise ValueError("基站端口无效") from error
        if (not host or len(host) > 253 or any(c in host for c in "\r\n/@:")
                or not mountpoint or len(mountpoint) > 255
                or any(c in mountpoint for c in "\r\n")
                or any(c in username + password for c in "\r\n")):
            raise ValueError("基站地址、挂载点或账号格式无效")
        if not 1 <= port <= 65535:
            raise ValueError("基站端口超出范围")
        return cls(host.encode("idna").decode("ascii"), port, mountpoint,
                   username, password)


def write_ntrip_control(path: pathlib.Path,
                        value: dict[str, object]) -> NtripConfig:
    """Validate and atomically replace the volatile NTRIP control file."""
    config = NtripConfig.from_mapping(value)
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", dir=path.parent)
    temporary = pathlib.Path(temporary_name)
    try:
        # When the command is accidentally run through sudo, keep the file
        # readable by the service account that owns the runtime directory.
        if hasattr(os, "fchown"):
            parent = path.parent.stat()
            os.fchown(descriptor, parent.st_uid, parent.st_gid)
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            descriptor = -1
            json.dump({
                "host": config.host, "port": config.port,
                "mountpoint": config.mountpoint, "username": config.username,
                "password": config.password,
            }, stream, ensure_ascii=False, separators=(",", ":"))
            stream.flush()
            os.fsync(stream.fileno())
        temporary.chmod(0o600)
        temporary.replace(path)
    finally:
        if descriptor >= 0:
            os.close(descriptor)
        temporary.unlink(missing_ok=True)
    return config


class BridgeFlow:
    WINDOW = 1024

    def __init__(self, report: list[int]):
        if len(report) != 13 or report[1] != 1 or report[2] != ((report[3] + report[4]) & 0xFFFFFFFF):
            raise ValueError("STM32 RTCM转发尚未就绪")
        self.previous_tx = report[3]
        self.errors = report[4:6]
        self.last_ms = report[0]
        self.sent = self.acked = 0
        self.progress_at = self.report_at = time.monotonic()

    def update(self, report: list[int]) -> None:
        if len(report) != 13 or not report[1]:
            raise OSError("STM32 RTCM转发状态无效")
        if ((report[0] - self.last_ms) & 0xFFFFFFFF) >= 0x80000000:
            raise OSError("STM32已重启，请重新连接基站")
        if report[4:6] != self.errors:
            raise OSError("STM32报告RTCM丢字节或串口错误")
        acked = self.acked + ((report[3] - self.previous_tx) & 0xFFFFFFFF)
        if acked < self.acked or acked > self.sent:
            raise OSError("STM32 RTCM转发计数不一致")
        now = time.monotonic()
        if acked > self.acked:
            self.progress_at = now
        self.last_ms, self.acked, self.report_at = report[0], acked, now
        self.previous_tx = report[3]

    def allowance(self) -> int:
        now = time.monotonic()
        if now - self.report_at > 2:
            raise OSError("STM32 RTCM转发状态超过2秒未更新")
        if self.sent > self.acked and now - self.progress_at > 2:
            raise OSError("STM32超过2秒未确认RTCM转发")
        return max(0, self.WINDOW - (self.sent - self.acked))

    def record_write(self, count: int) -> None:
        if count < 0 or count > self.WINDOW - (self.sent - self.acked):
            raise ValueError("RTCM写入超过STM32信用窗口")
        if self.sent == self.acked:
            self.progress_at = time.monotonic()
        self.sent += count


class NtripWorker(threading.Thread):
    def __init__(self, config: NtripConfig):
        super().__init__(name="ntrip-client", daemon=True)
        self.config = config
        self.stop_event = threading.Event()
        self.frames: queue.Queue[bytes] = queue.Queue(maxsize=128)
        self.lock = threading.Lock()
        self.phase = "准备连接"
        self.connected = False
        self.error = ""
        self.network_bytes = self.frame_count = self.crc_errors = self.reconnects = 0
        self.latest_type: int | None = None
        self.last_valid_unix_ms: int | None = None
        self.filtered_navic = self.rewritten_msm = 0

    def stop(self) -> None:
        self.stop_event.set()

    def snapshot(self) -> dict[str, object]:
        with self.lock:
            return {
                "requested": True, "connected": self.connected, "phase": self.phase,
                "error": self.error, "host": self.config.host,
                "port": self.config.port, "mountpoint": self.config.mountpoint,
                "network_bytes": self.network_bytes, "frames": self.frame_count,
                "crc_errors": self.crc_errors, "reconnects": self.reconnects,
                "latest_type": self.latest_type,
                "last_valid_unix_ms": self.last_valid_unix_ms,
                "filtered_navic": self.filtered_navic,
                "rewritten_msm": self.rewritten_msm,
                "queued_frames": self.frames.qsize(),
            }

    def take_frame(self) -> bytes | None:
        try:
            return self.frames.get_nowait()
        except queue.Empty:
            return None

    def _clear_frames(self) -> None:
        while True:
            try:
                self.frames.get_nowait()
            except queue.Empty:
                return

    def _put(self, frame: bytes) -> None:
        while not self.stop_event.is_set():
            try:
                self.frames.put(frame, timeout=0.1)
                return
            except queue.Full:
                continue

    def run(self) -> None:
        while not self.stop_event.is_set():
            try:
                self._session()
            except (OSError, ValueError) as error:
                with self.lock:
                    self.connected = False
                    self.phase = "等待重连"
                    self.error = str(error)
                    self.reconnects += 1
            self._clear_frames()
            if self.stop_event.wait(5):
                break
        with self.lock:
            self.connected = False
            self.phase = "已停止"

    def _session(self) -> None:
        parser = RtcmFrameParser()
        adapter = F9pMsmAdapter()
        decoder = NtripResponse()
        config = self.config
        mount = urllib.parse.quote(config.mountpoint, safe="")
        auth = base64.b64encode(
            f"{config.username}:{config.password}".encode()).decode("ascii")
        with self.lock:
            self.phase = "连接服务器"
            self.error = ""
        with socket.create_connection((config.host, config.port), timeout=5) as sock:
            sock.settimeout(0.5)
            request = (f"GET /{mount} HTTP/1.1\r\n"
                       f"Host: {config.host}:{config.port}\r\n"
                       "Ntrip-Version: Ntrip/2.0\r\n"
                       "User-Agent: NTRIP MPU6050F9P-Pi/1.0\r\n"
                       f"Authorization: Basic {auth}\r\n"
                       "Accept: */*\r\nConnection: close\r\n\r\n")
            sock.sendall(request.encode("ascii"))
            with self.lock:
                self.phase = "等待RTCM"
            last_frame = time.monotonic()
            while not self.stop_event.is_set():
                if time.monotonic() - last_frame > 15:
                    raise OSError("15秒未收到有效RTCM帧")
                try:
                    data = sock.recv(8192)
                except socket.timeout:
                    continue
                if not data:
                    raise OSError("基站断开连接")
                body = decoder.feed(data)
                before_crc = parser.crc_errors
                frames = parser.feed(body)
                with self.lock:
                    self.network_bytes += len(body)
                    self.crc_errors += parser.crc_errors - before_crc
                for frame in frames:
                    last_frame = time.monotonic()
                    message = (frame[3] << 4) | (frame[4] >> 4)
                    outgoing = adapter.feed(frame, last_frame)
                    for ready in outgoing:
                        self._put(ready)
                    with self.lock:
                        self.connected = True
                        self.phase = "已连接，收到有效RTCM"
                        self.error = ""
                        self.frame_count += 1
                        self.latest_type = message
                        self.last_valid_unix_ms = round(time.time() * 1000)
                        self.filtered_navic = adapter.filtered
                        self.rewritten_msm = adapter.rewritten
                if decoder.ended:
                    raise OSError("基站数据流结束")


class NtripController:
    """Coordinate volatile config, network worker, and serial credit flow."""

    def __init__(self, control_file: pathlib.Path | None):
        self.control_file = control_file
        self.worker: NtripWorker | None = None
        self.config: NtripConfig | None = None
        self.control_signature: tuple[int, int] | None = None
        self.flow: BridgeFlow | None = None
        self.pending = b""
        self.pending_offset = 0
        self.forwarded_bytes = 0
        self.delivery_error = ""

    def sync_control(self) -> None:
        if self.control_file is None or not self.control_file.exists():
            if self.worker is not None:
                self._stop_worker()
            self.control_signature = None
            return
        try:
            stat = self.control_file.stat()
            signature = (stat.st_mtime_ns, stat.st_size)
            if signature == self.control_signature:
                return
            value = json.loads(self.control_file.read_text(encoding="utf-8"))
            config = NtripConfig.from_mapping(value)
        except (OSError, ValueError, TypeError):
            return
        self._stop_worker()
        self.config = config
        self.control_signature = signature
        self.delivery_error = ""
        self.worker = NtripWorker(config)
        self.worker.start()

    def _stop_worker(self) -> None:
        if self.worker is not None:
            self.worker.stop()
            self.worker.join(timeout=1.0)
        self.worker = None
        self.config = None
        self.flow = None
        self.pending = b""
        self.pending_offset = 0

    def update_bridge(self, report: list[int]) -> None:
        if self.worker is None:
            return
        try:
            if self.flow is None:
                self.flow = BridgeFlow(report)
            else:
                self.flow.update(report)
        except (OSError, ValueError) as error:
            self.disable(str(error))

    def pump_serial(self, port) -> None:
        worker = self.worker
        if worker is None or self.flow is None:
            return
        try:
            allowance = self.flow.allowance()
            if allowance <= 0:
                return
            if not self.pending:
                frame = worker.take_frame()
                if frame is None:
                    return
                self.pending = frame
                self.pending_offset = 0
            chunk = self.pending[self.pending_offset:self.pending_offset + allowance]
            written = port.write(chunk)
            if written <= 0:
                raise OSError("树莓派UART写入RTCM无进展")
            self.flow.record_write(written)
            self.forwarded_bytes += written
            self.pending_offset += written
            if self.pending_offset == len(self.pending):
                self.pending = b""
                self.pending_offset = 0
        except (OSError, ValueError) as error:
            self.disable(str(error))

    def disable(self, reason: str) -> None:
        self.delivery_error = reason
        if self.control_file is not None:
            try:
                self.control_file.unlink(missing_ok=True)
            except OSError:
                pass
        self._stop_worker()

    def snapshot(self) -> dict[str, object]:
        if self.worker is None:
            return {"requested": False, "connected": False, "phase": "未连接",
                    "error": self.delivery_error, "forwarded_bytes": self.forwarded_bytes,
                    "bridge_ready": False, "queued_frames": 0}
        result = self.worker.snapshot()
        result.update({"forwarded_bytes": self.forwarded_bytes,
                       "bridge_ready": self.flow is not None,
                       "delivery_error": self.delivery_error})
        return result

    def close(self) -> None:
        self._stop_worker()


def parse_bridge_report(line: str) -> list[int] | None:
    if not line.startswith("#RTCM,"):
        return None
    try:
        report = [int(value) for value in line.split(",")[1:]]
    except ValueError:
        return None
    if len(report) != 13 or any(value < 0 or value > 0xFFFFFFFF for value in report):
        return None
    return report
