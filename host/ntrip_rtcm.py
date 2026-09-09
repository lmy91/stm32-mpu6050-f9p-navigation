"""NTRIP client and RTCM3 statistics for the MPU6050/F9P monitor.

The client connects to the WHU caster over a plain socket so system proxy
settings are never inherited. Validated RTCM3 frames enter a bounded queue;
an independent serial worker forwards them using STM32 credits.
"""

from __future__ import annotations

import base64
import pathlib
import queue
import re
import threading
import socket
import time
import urllib.parse
from collections import deque
from typing import NamedTuple

from PyQt5 import QtCore, QtWidgets

NTRIP_DEFAULT_HOST = "ntrip.gnsswhu.cn"
NTRIP_DEFAULT_PORT = 2101
NTRIP_DEFAULT_MOUNT = "WUH200CHN0"

RTCM_SEND_TIMEOUT = 2.0


class RtcmPacket(NamedTuple):
    ready_at: float
    data: bytes
    received_at: float | None = None

    def ages(self, now):
        received = self.ready_at if self.received_at is None else self.received_at
        return self.ready_at - received, now - self.ready_at, now - received

    def check_age(self, now):
        assembly, sending, total = self.ages(now)
        # Caster-side MSM delivery can legitimately pause for more than ten
        # seconds. Only local queue/UART waiting is a host blockage; assembly
        # age is diagnostic and the receiver decides whether a correction is
        # still usable from its GNSS epoch.
        if sending > RTCM_SEND_TIMEOUT:
            raise OSError(f"RTCM 发送等待超限：组包={assembly:.3f}s "
                          f"发送等待={sending:.3f}s(限{RTCM_SEND_TIMEOUT:g}s) "
                          f"总驻留={total:.3f}s，已停止下发")

# Observation-arrival interval is NOT the GNSS measurement correction age.
_OBSERVATION_TYPES = frozenset(
    list(range(1001, 1005)) + list(range(1009, 1013))
    + list(range(1071, 1078)) + list(range(1081, 1088))
    + list(range(1091, 1098)) + list(range(1101, 1108))
    + list(range(1111, 1118)) + list(range(1121, 1128))
    + list(range(1131, 1138))
)


def crc24q(data: bytes) -> int:
    """RTCM3 24-bit CRC (CRC-24Q), covering preamble through payload."""
    crc = 0
    for byte in data:
        crc ^= byte << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
    return crc & 0xFFFFFF


class RtcmFrameParser:
    """Incremental RTCM3 framing with CRC, type counts and correction age."""

    def __init__(self, window_seconds: float = 2.0) -> None:
        self._buffer = bytearray()
        self._window = window_seconds
        self.total_bytes = 0
        self.frame_count = 0
        self.crc_errors = 0
        self.resync_bytes = 0
        self.by_type: dict[int, int] = {}
        self.latest_type: int | None = None
        self.base_positions = {}
        self.msm_epochs = {}
        self._rate_samples: deque[tuple[float, int]] = deque()
        self._last_obs_monotonic: float | None = None

    def feed(self, data: bytes) -> list[bytes]:
        now = time.monotonic()
        self._buffer.extend(data)
        self.total_bytes += len(data)
        self._rate_samples.append((now, self.total_bytes))
        cutoff = now - self._window
        while self._rate_samples and self._rate_samples[0][0] < cutoff:
            self._rate_samples.popleft()
        return self._parse_all()

    def _parse_all(self) -> list[bytes]:
        buf = self._buffer
        frames = []
        while True:
            start = buf.find(b"\xd3")
            if start < 0:
                self.resync_bytes += len(buf)
                del buf[:]
                return frames
            if start > 0:
                self.resync_bytes += start
                del buf[:start]
            if len(buf) < 3:
                return frames
            if buf[1] & 0xFC:
                self.resync_bytes += 1
                del buf[:1]
                continue
            length = ((buf[1] & 0x03) << 8) | buf[2]
            total = 3 + length + 3
            if len(buf) < total:
                return frames
            frame = bytes(buf[:total])
            payload = frame[3:3 + length]
            if length >= 2:
                msg_type = (payload[0] << 4) | ((payload[1] & 0xF0) >> 4)
            else:
                msg_type = 0
            if crc24q(frame[:-3]) == int.from_bytes(frame[-3:], "big"):
                frames.append(frame)
                self.frame_count += 1
                self.by_type[msg_type] = self.by_type.get(msg_type, 0) + 1
                self.latest_type = msg_type
                self._metadata(payload, msg_type)
                if msg_type in _OBSERVATION_TYPES:
                    self._last_obs_monotonic = time.monotonic()
                del buf[:total]
            else:
                # Bad CRC: the assumed frame boundary may be wrong. Drop one
                # byte and resync instead of consuming the whole length.
                self.crc_errors += 1
                self.resync_bytes += 1
                del buf[:1]

    def _metadata(self, payload, message):
        """Diagnostic RTCM header fields only; never modify observations.

        Field layout checked against RTKLIB src/rtcm3.c decode_type1005 and
        decode_msm_head. TOW comparison must use the same constellation scale.
        """
        def bits(offset, width, signed=False):
            shift = len(payload) * 8 - offset - width
            value = (int.from_bytes(payload, "big") >> shift) & ((1 << width) - 1)
            return value - (1 << width) if signed and value & (1 << (width - 1)) else value
        if message in (1005, 1006) and len(payload) >= (19 if message == 1005 else 21):
            self.base_positions[bits(12, 12)] = tuple(bits(offset, 38, True) * 0.0001
                                                    for offset in (34, 74, 114))
        family = message // 10
        if 107 <= family <= 113 and 1 <= message % 10 <= 7 and len(payload) >= 22:
            self.msm_epochs[message] = {"station": bits(12, 12), "epoch_raw_ms": bits(24, 30),
                                       "multiple_message": bits(54, 1),
                                       "satellite_mask": bits(73, 64), "signal_mask": bits(137, 32),
                                       "arrived_at": time.monotonic()}

    def byte_rate(self) -> float:
        if len(self._rate_samples) < 2:
            return 0.0
        t0, b0 = self._rate_samples[0]
        t1, b1 = self._rate_samples[-1]
        span = t1 - t0
        if span <= 0:
            return 0.0
        return (b1 - b0) / span

    def observation_arrival_age(self) -> float | None:
        if self._last_obs_monotonic is None:
            return None
        return time.monotonic() - self._last_obs_monotonic


class NtripResponse:
    """Incremental HTTP/ICY and HTTP chunk decoding; never forward headers."""

    def __init__(self):
        self.buffer = bytearray()
        self.headers_done = False
        self.chunked = False
        self.remaining = None
        self.need_crlf = False
        self.ended = False

    def feed(self, data: bytes) -> bytes:
        if self.ended:
            return b""
        self.buffer.extend(data)
        buf = self.buffer
        if not self.headers_done:
            # Legacy NTRIP v1 can start the binary stream immediately after ICY.
            icy = buf.startswith(b"ICY 200 OK\r\n")
            end = buf.find(b"\r\n") if icy else buf.find(b"\r\n\r\n")
            if end < 0:
                if len(buf) > 16384:
                    raise OSError("NTRIP 响应头过大")
                return b""
            header = bytes(buf[:end])
            if end > 16384:
                raise OSError("NTRIP 响应头过大")
            first = header.split(b"\r\n", 1)[0].split()
            if len(first) < 2 or first[1] != b"200":
                # Do not echo server text, which could contain credentials.
                code = first[1].decode("ascii", "replace") if len(first) > 1 else "?"
                raise OSError(f"NTRIP HTTP {code}，请检查账号、挂载点或网络")
            if not (first[0].startswith(b"HTTP/") or icy):
                raise OSError("服务器未返回 NTRIP 数据流")
            headers = {}
            for line in header.split(b"\r\n")[1:]:
                key, sep, value = line.partition(b":")
                if sep:
                    headers[key.strip().lower()] = value.strip().lower()
            encoding = headers.get(b"transfer-encoding", b"")
            if encoding not in (b"", b"identity", b"chunked"):
                raise OSError("不支持的 NTRIP Transfer-Encoding")
            if headers.get(b"content-encoding", b"identity") != b"identity":
                raise OSError("不支持压缩的 NTRIP 流")
            content = headers.get(b"content-type", b"")
            if b"sourcetable" in content or b"text/html" in content:
                raise OSError("收到站点列表/网页，而非 RTCM 数据流")
            self.chunked = encoding == b"chunked"
            del buf[:end + (2 if icy else 4)]
            self.headers_done = True
        if not self.chunked:
            data = bytes(buf)
            buf.clear()
            return data
        output = bytearray()
        while True:
            if self.need_crlf:
                if len(buf) < 2:
                    break
                if buf[:2] != b"\r\n":
                    raise OSError("NTRIP chunk 结尾错误")
                del buf[:2]
                self.need_crlf = False
            if self.remaining is None:
                end = buf.find(b"\r\n")
                if end < 0:
                    if len(buf) > 1024:
                        raise OSError("NTRIP chunk 头过大")
                    break
                try:
                    token = bytes(buf[:end]).split(b";", 1)[0]
                    if not token or any(c not in b"0123456789abcdefABCDEF" for c in token):
                        raise ValueError
                    self.remaining = int(token, 16)
                except ValueError as error:
                    raise OSError("NTRIP chunk 长度错误") from error
                if self.remaining > 16 * 1024 * 1024:
                    raise OSError("NTRIP chunk 超过限制")
                del buf[:end + 2]
                if not self.remaining:
                    self.ended = True
                    buf.clear()
                    break
            count = min(len(buf), self.remaining)
            output.extend(buf[:count]); del buf[:count]
            self.remaining -= count
            if self.remaining:
                break
            self.remaining = None
            self.need_crlf = True
        return bytes(output)


class BridgeFlow:
    """Credit window bounds bytes in Windows UART + MCU queue, including bursts."""

    WINDOW = 1024  # firmware ring holds 2047; leave room for IRQ/UART bytes

    def __init__(self, report: list[int]):
        if len(report) != 13 or report[1] != 1 or report[2] != ((report[3] + report[4]) & 0xFFFFFFFF):
            raise ValueError("STM32 转发未就绪或仍有待发数据")
        self.previous_tx = report[3]
        self.errors = report[4:6]
        self.last_ms = report[0]
        self.sent = self.acked = 0
        self.progress_at = self.report_at = time.monotonic()

    def update(self, report: list[int]) -> None:
        if not report[1] or ((report[0] - self.last_ms) & 0xFFFFFFFF) >= 0x80000000:
            raise OSError("STM32 重启，基站已停止，请重新连接")
        if report[4:6] != self.errors:
            raise OSError("STM32 RTCM 丢字节/串口错误，已停止下发")
        acked = self.acked + ((report[3] - self.previous_tx) & 0xFFFFFFFF)
        if acked < self.acked or acked > self.sent:
            raise OSError("RTCM 转发计数不一致，请重新连接串口")
        now = time.monotonic()
        if acked > self.acked:
            self.progress_at = now
        self.last_ms, self.acked, self.report_at = report[0], acked, now
        self.previous_tx = report[3]

    def allowance(self) -> int:
        now = time.monotonic()
        if now - self.report_at > 2:
            raise OSError("STM32 转发状态超时，已停止下发")
        if self.sent > self.acked and now - self.progress_at > 2:
            raise OSError("STM32 未确认转发，请检查 USB-TTL TX → PA10")
        return max(0, self.WINDOW - (self.sent - self.acked))

    def record_write(self, count: int) -> None:
        if count < 0 or count > self.WINDOW - (self.sent - self.acked):
            raise ValueError("RTCM write exceeds credit")
        if self.sent == self.acked:
            self.progress_at = time.monotonic()
        self.sent += count


class F9pMsmAdapter:
    """Close a supported MSM group even when unsupported NavIC is its terminator.

    Buffer only a bounded, complete MSM group. Preserve all observation bits;
    omit NavIC (1131..1137), clear MMI on the last retained message and renew CRC.
    Non-MSM and unknown RTCM messages pass through immediately and never act as
    group delimiters. A repeated constellation with a new epoch is a reliable
    implicit boundary: release the preceding supported observations after
    repairing their final MMI, then start the new epoch. Wall-clock gaps are
    not protocol boundaries: WUH2 has been observed splitting one valid group
    across more than thirteen seconds. Capacity, malformed headers and station
    changes remain bounded failures without tearing down a healthy connection.
    """

    def __init__(self):
        self.pending = []
        self.epochs = {}
        self.station = None
        self.bytes = 0
        self.filtered = self.rewritten = 0
        self.dropped_groups = self.dropped_frames = self.dropped_bytes = 0
        self.expired_groups = self.discontinuous_groups = 0
        self.oversize_groups = self.malformed_groups = 0
        self.recovered_groups = 0
        self.drop_sequence = 0
        self.notice_sequence = 0
        self.last_drop_reason = ""
        self.last_notice_reason = ""

    def reset(self):
        self.pending.clear(); self.epochs.clear(); self.station = None; self.bytes = 0

    @staticmethod
    def _header(frame):
        message = (frame[3] << 4) | (frame[4] >> 4)
        family = message // 10
        p = frame[3:-3]
        number = int.from_bytes(p, "big")
        station = (number >> (len(p)*8-24)) & 4095
        epoch = (number >> (len(p)*8-54)) & ((1 << 30)-1)
        multiple = (p[6] >> 1) & 1
        return message, family, station, epoch, multiple

    def _summary(self, timestamp=None):
        entries = []
        for _, data in self.pending:
            message, _, station, epoch, multiple = self._header(data)
            entries.append(f"{message}(站{station},历元{epoch},MMI={multiple})")
        age = ""
        if timestamp is not None and self.pending:
            age = f"，组龄={timestamp-self.pending[0][0]:.3f}s"
        return ("→".join(entries) if entries else "空") + age

    def _discard(self, reason, category, extra_frames=0, extra_bytes=0, timestamp=None):
        frames = len(self.pending) + extra_frames
        byte_count = self.bytes + extra_bytes
        summary = self._summary(timestamp)
        self.dropped_groups += 1
        self.dropped_frames += frames
        self.dropped_bytes += byte_count
        if category == "expired": self.expired_groups += 1
        elif category == "discontinuous": self.discontinuous_groups += 1
        elif category == "oversize": self.oversize_groups += 1
        elif category == "malformed": self.malformed_groups += 1
        self.drop_sequence += 1
        self.notice_sequence += 1
        self.last_drop_reason = f"{reason}；缓存={summary}；丢弃 {frames} 帧/{byte_count} B"
        self.last_notice_reason = self.last_drop_reason
        self.reset()

    def _release(self):
        output = [(stamp, data) for stamp, data in self.pending
                  if ((data[3] << 4) | (data[4] >> 4)) // 10 != 113]
        self.filtered += len(self.pending) - len(output)
        if output and output[-1][1][9] & 2:
            stamp, data = output[-1]
            modified = bytearray(data[:-3]); modified[9] &= ~2
            output[-1] = (stamp, bytes(modified) + crc24q(modified).to_bytes(3, "big"))
            self.rewritten += 1
        self.reset()
        return output

    def feed(self, frame, timestamp):
        message = (frame[3] << 4) | (frame[4] >> 4)
        family = message // 10
        # RTCM framing and CRC validation have already succeeded. Unknown or
        # non-MSM payloads are independent messages, not MSM group boundaries.
        if not (107 <= family <= 113 and 1 <= message % 10 <= 7):
            return [(timestamp, frame)]
        if len(frame) < 28:
            self._discard("MSM 头长度不足", "malformed", 1, len(frame), timestamp)
            return []
        _, _, station, epoch, multiple = self._header(frame)
        outgoing = []
        if self.pending and station != self.station:
            self._discard(
                f"MSM 站号由 {self.station} 变为 {station}，上一组不能拼接",
                "discontinuous", timestamp=timestamp)
        elif self.pending and family in self.epochs and self.epochs[family] != epoch:
            summary = self._summary(timestamp)
            outgoing = self._release()
            self.recovered_groups += 1
            self.notice_sequence += 1
            self.last_notice_reason = (
                f"同星座 {family}x 新历元 {epoch} 到达，以历元边界结束上一组；"
                f"缓存={summary}；恢复 {len(outgoing)} 帧")
        self.station = station; self.epochs[family] = epoch
        self.pending.append((timestamp, frame)); self.bytes += len(frame)
        if self.bytes > 65536:
            self._discard("MSM 观测组超过 64 KiB 限制", "oversize", timestamp=timestamp)
            return outgoing
        if multiple: return outgoing
        return outgoing + self._release()


class RtcmQueue:
    """Byte-bounded whole-frame queue, with backpressure instead of burst loss."""

    def __init__(self, max_bytes=65536):
        self.max_bytes = max_bytes
        self.items = deque()
        self.bytes = self.peak_bytes = 0
        self.condition = threading.Condition()

    def put_nowait(self, item):
        item = RtcmPacket(*item)
        frame = item.data
        with self.condition:
            if self.bytes + len(frame) > self.max_bytes:
                raise queue.Full
            self.items.append(item)
            self.bytes += len(frame)
            self.peak_bytes = max(self.peak_bytes, self.bytes)
            self.condition.notify_all()

    def put(self, item, stop_event):
        item = RtcmPacket(*item)
        frame = item.data
        if len(frame) > self.max_bytes:
            raise ValueError("RTCM frame exceeds queue capacity")
        with self.condition:
            while self.bytes + len(frame) > self.max_bytes:
                if stop_event.is_set(): return
                item.check_age(time.monotonic())
                self.condition.wait(0.05)
            if not stop_event.is_set(): self.put_nowait(item)

    def get_nowait(self):
        with self.condition:
            if not self.items: raise queue.Empty
            item = self.items.popleft()
            self.bytes -= len(item[1])
            self.condition.notify_all()
            return item

    def qsize(self):
        with self.condition: return len(self.items)

    def snapshot(self):
        with self.condition:
            age = time.monotonic() - self.items[0][0] if self.items else 0.0
            return self.bytes, self.peak_bytes, age


class NtripClient(QtCore.QThread):
    """Direct sockets, bounded frame queue, no serial access from this thread."""

    status = QtCore.pyqtSignal(str)
    diagnostic = QtCore.pyqtSignal(str)

    def __init__(self, host, port, mountpoint, user, password, parent=None, *, f9p_compat=True):
        super().__init__(parent)
        self._host, self._port = host, int(port)
        self._mount = urllib.parse.quote(mountpoint.lstrip("/"), safe="")
        if not host or any(c in host for c in "\r\n/@"):
            raise ValueError("服务器格式错误（只填写域名或 IP）")
        if not 1 <= self._port <= 65535 or not self._mount:
            raise ValueError("端口或挂载点为空/超出范围")
        self._host = host.encode("idna").decode("ascii")
        self._auth = base64.b64encode(f"{user}:{password}".encode()).decode("ascii")
        self._stop_event = threading.Event()
        self.frames = RtcmQueue()
        self.parser = RtcmFrameParser()
        self.network_bytes = self.crc_errors = self.frame_count = 0
        self.reconnects = 0
        self.message_types = {}
        self.f9p_compat = f9p_compat
        self.msm_adapter = F9pMsmAdapter()
        self.session_id = 0
        self._reset_debug()

    def _reset_debug(self):
        self.session_started = time.monotonic()
        self.last_socket_at = self.last_body_at = self.last_valid_at = None
        self.socket_bytes = self.recv_calls = self.recv_timeouts = 0
        self.phase = "连接 TCP（含域名解析）"
        self.decoder = NtripResponse()
        self._debug_at = self.session_started
        self._debug_counts = (0, 0, 0)
        self._gap_reported = False

    def _emit_debug(self, reason):
        # Construct in the network owner thread; never emit payloads/auth headers.
        now = time.monotonic()
        def age(stamp):
            return "未收到" if stamp is None else f"{now - stamp:.2f}s"
        counts = (self.socket_bytes, self.parser.total_bytes, self.parser.frame_count)
        delta = tuple(a-b for a, b in zip(counts, self._debug_counts))
        queued, peak, queue_age = self.frames.snapshot()
        decoder = self.decoder
        msm = self.msm_adapter
        pending_age = age(msm.pending[0][0]) if msm.pending else "0s"
        self.diagnostic.emit(
            f"{reason} | 会话#{self.session_id} 阶段={self.phase} 运行={now-self.session_started:.1f}s "
            f"采样间隔={now-self._debug_at:.2f}s ΔTCP/正文/有效帧={delta[0]}B/{delta[1]}B/{delta[2]}帧 | "
            f"本连接TCP={self.socket_bytes}B recv={self.recv_calls}次 0.5s读等待超时={self.recv_timeouts}次 "
            f"距TCP/正文/有效帧={age(self.last_socket_at)}/{age(self.last_body_at)}/{age(self.last_valid_at)} | "
            f"HTTP头完成={int(decoder.headers_done)} chunked={int(decoder.chunked)} "
            f"HTTP缓存={len(decoder.buffer)}B chunk剩余={decoder.remaining} EOF={int(decoder.ended)} | "
            f"RTCM缓存={len(self.parser._buffer)}B CRC错={self.parser.crc_errors} "
            f"重同步丢弃={self.parser.resync_bytes}B 最近类型={self.parser.latest_type} | "
            f"MSM待组={len(msm.pending)}帧/{msm.bytes}B/{pending_age} "
            f"丢组/帧/字节={msm.dropped_groups}/{msm.dropped_frames}/{msm.dropped_bytes} "
            f"(恢复/过期/断续/超限/畸形={msm.recovered_groups}/{msm.expired_groups}/"
            f"{msm.discontinuous_groups}/{msm.oversize_groups}/{msm.malformed_groups}) "
            f"队列={queued}B/{queue_age:.2f}s 峰值={peak}B；以上间隔不是差分龄期")
        self._debug_at = now
        self._debug_counts = counts

    def stop(self):
        self._stop_event.set()

    def take_frame(self):
        return self.take_item()[1]

    def take_item(self):
        try:
            item = self.frames.get_nowait()
        except queue.Empty:
            return RtcmPacket(time.monotonic(), b"")
        item.check_age(time.monotonic())
        return item

    def _enqueue_ready(self, outgoing):
        # Timestamp once when the group is released, BEFORE any queue wait.
        # Original reception time remains attached for total residence checks.
        ready_at = time.monotonic()
        for received_at, data in outgoing:
            self.frames.put(RtcmPacket(ready_at, data, received_at), self._stop_event)
            if self._stop_event.is_set(): return

    def run(self):
        while not self._stop_event.is_set():
            try:
                self._session()
            except (OSError, ValueError) as error:
                if not self._stop_event.is_set():
                    self.reconnects += 1
                    self._emit_debug(f"异常现场 {type(error).__name__} errno={getattr(error, 'errno', None)} "
                                     f"winerror={getattr(error, 'winerror', None)}：{error}")
                    self.status.emit(f"基站: {error}；5 秒后重连")
            # Discard old whole frames; never replay stale corrections.
            while True:
                try: self.frames.get_nowait()
                except queue.Empty: break
            if self._stop_event.wait(5):
                break

    def _session(self):
        self.parser = RtcmFrameParser()
        self.msm_adapter.reset()
        self.session_id += 1
        self._reset_debug()
        decoder = self.decoder
        self._emit_debug("开始连接；直接 TCP，不使用系统 HTTP 代理；不排除 VPN 路由接管")
        # Does not consult HTTP_PROXY, Windows Internet Settings or a local proxy.
        with socket.create_connection((self._host, self._port), timeout=3) as sock:
            self.phase = "发送 NTRIP 请求"
            sock.settimeout(0.5)
            request = (f"GET /{self._mount} HTTP/1.1\r\n"
                       f"Host: {self._host}:{self._port}\r\n"
                       "Ntrip-Version: Ntrip/2.0\r\n"
                       "User-Agent: NTRIP MPU6050F9P/1.0\r\n"
                       f"Authorization: Basic {self._auth}\r\n"
                       "Accept: */*\r\nConnection: close\r\n\r\n")
            sock.sendall(request.encode("ascii"))
            self.phase = "等待 HTTP 响应头"
            last_frame = time.monotonic()
            announced = False
            while not self._stop_event.is_set():
                now = time.monotonic()
                if now - last_frame > 5 and not self._gap_reported:
                    self._emit_debug("预警：5 秒无有效 RTCM（尚未重连）")
                    self._gap_reported = True
                elif now - self._debug_at >= 10:
                    self._emit_debug("周期诊断")
                if time.monotonic() - last_frame > 15:
                    raise OSError("15 秒未收到有效 RTCM 帧")
                try: data = sock.recv(8192)
                except socket.timeout:
                    self.recv_timeouts += 1
                    continue
                if not data:
                    raise OSError("基站断开连接")
                self.last_socket_at = time.monotonic()
                self.socket_bytes += len(data); self.recv_calls += 1
                body = decoder.feed(data)
                if decoder.headers_done: self.phase = "接收 RTCM 正文"
                if body: self.last_body_at = time.monotonic()
                self.network_bytes += len(body)
                before = self.parser.crc_errors
                frames = self.parser.feed(body)
                self.crc_errors += self.parser.crc_errors - before
                for frame in frames:
                    last_frame = time.monotonic()
                    self.last_valid_at = last_frame
                    self.frame_count += 1
                    message = (frame[3] << 4) | (frame[4] >> 4) if len(frame) >= 8 else 0
                    self.message_types[message] = self.message_types.get(message, 0) + 1
                    notice_before = self.msm_adapter.notice_sequence
                    outgoing = (self.msm_adapter.feed(frame, last_frame) if self.f9p_compat
                                else [(last_frame, frame)])
                    if self.msm_adapter.notice_sequence != notice_before:
                        self.diagnostic.emit(
                            f"MSM兼容：{self.msm_adapter.last_notice_reason}；"
                            "RTCM流继续，NTRIP连接保持")
                    self._enqueue_ready(outgoing)
                    if self._stop_event.is_set(): return
                    if not announced:
                        self.status.emit("基站: 已直连 NTRIP v2，收到有效 RTCM")
                        self._emit_debug("首次有效 RTCM")
                        announced = True
                    elif self._gap_reported:
                        self._emit_debug("有效 RTCM 恢复（未重建连接）")
                    self._gap_reported = False
                if decoder.ended:
                    raise OSError("基站数据流结束")


class NtripSettingsDialog(QtWidgets.QDialog):
    """Caster endpoint and credentials, kept in QSettings (not the repo)."""

    def __init__(self, settings: QtCore.QSettings,
                 parent: QtWidgets.QWidget | None = None) -> None:
        super().__init__(parent)
        self.setWindowTitle("NTRIP 基站设置")
        self.settings = settings
        form = QtWidgets.QFormLayout(self)
        self.host_edit = QtWidgets.QLineEdit(
            str(settings.value("ntrip_host", NTRIP_DEFAULT_HOST)))
        self.port_spin = QtWidgets.QSpinBox()
        self.port_spin.setRange(1, 65535)
        self.port_spin.setValue(int(settings.value("ntrip_port", NTRIP_DEFAULT_PORT)))
        self.mount_edit = QtWidgets.QLineEdit(
            str(settings.value("ntrip_mount", NTRIP_DEFAULT_MOUNT)))
        self.user_edit = QtWidgets.QLineEdit(str(settings.value("ntrip_user", "")))
        self.pass_edit = QtWidgets.QLineEdit(str(settings.value("ntrip_password", "")))
        self.pass_edit.setEchoMode(QtWidgets.QLineEdit.Password)
        self.remember = QtWidgets.QCheckBox("记住密码（本机明文存储，不上传 Git）")
        self.remember.setChecked(bool(settings.value("ntrip_password", "")))
        form.addRow("服务器", self.host_edit)
        form.addRow("端口", self.port_spin)
        form.addRow("挂载点", self.mount_edit)
        form.addRow("账号", self.user_edit)
        form.addRow("密码", self.pass_edit)
        form.addRow(self.remember)
        import_button = QtWidgets.QPushButton("从 BNC 配置导入…")
        import_button.clicked.connect(self._import_bnc)
        form.addRow(import_button)
        buttons = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)
        buttons.accepted.connect(self._save)
        buttons.rejected.connect(self.reject)
        form.addRow(buttons)

    def _save(self) -> None:
        self.settings.setValue("ntrip_host", self.host_edit.text().strip())
        self.settings.setValue("ntrip_port", int(self.port_spin.value()))
        self.settings.setValue("ntrip_mount", self.mount_edit.text().strip())
        self.settings.setValue("ntrip_user", self.user_edit.text().strip())
        if self.remember.isChecked():
            self.settings.setValue("ntrip_password", self.pass_edit.text())
        else:
            self.settings.remove("ntrip_password")
        self.accept()

    def _import_bnc(self):
        filename, _ = QtWidgets.QFileDialog.getOpenFileName(self, "选择 BNC 配置", "", "BNC (*.bnc)")
        if not filename:
            return
        try:
            values = read_bnc_endpoint(pathlib.Path(filename))
            host, port, mount, user, password = values
        except (OSError, ValueError):
            QtWidgets.QMessageBox.warning(self, "导入失败", "未找到有效的 NTRIP mountPoints 配置，请手动填写。")
            return
        self.host_edit.setText(host); self.port_spin.setValue(port)
        self.mount_edit.setText(mount); self.user_edit.setText(user); self.pass_edit.setText(password)


def read_bnc_endpoint(path: pathlib.Path):
    """Read only a user-selected configuration. Never log its credentials."""
    text = path.read_text(encoding="utf-8-sig", errors="replace")
    match = re.search(r'''(?m)^mountPoints\s*=\s*["']?(//[^\s"',]+)''', text)
    if not match:
        raise ValueError("No mountPoints")
    parsed = urllib.parse.urlsplit(match[1])
    if not parsed.hostname or not parsed.path.strip("/"):
        raise ValueError("Invalid mountPoints")
    return (parsed.hostname, parsed.port or 2101, parsed.path.lstrip("/"),
            urllib.parse.unquote(parsed.username or ""), urllib.parse.unquote(parsed.password or ""))
