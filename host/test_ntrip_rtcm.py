import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pathlib
import csv
import queue
import socket
import tempfile
import time
import unittest
from unittest import mock

from PyQt5 import QtWidgets
from ntrip_rtcm import (BridgeFlow, NtripClient, NtripResponse, RtcmFrameParser,
                        crc24q, read_bnc_endpoint)
from imu_serial_qt import NavigationMonitor
from serial_worker import SerialWorker
from ntrip_rtcm import F9pMsmAdapter, RtcmPacket


def frame(length=400, message=1077):
    payload = bytes([message >> 4, (message & 15) << 4]) + bytes(length - 2)
    data = b"\xd3" + length.to_bytes(2, "big") + payload
    return data + crc24q(data).to_bytes(3, "big")


def report(ms=10000, rx=0, tx=0, dropped=0, errors=0):
    return [ms, 1, rx, tx, dropped, errors, 0, 0, 0, 0, 0, 0, 0xFFFFFFFF]


def msm(message, epoch=1000, multiple=1, station=621):
    p = ((message << (176-12)) | (station << (176-24)) |
         (epoch << (176-54)) | (multiple << (176-55))).to_bytes(22, "big")
    data = b"\xd3\x00\x16" + p
    return data + crc24q(data).to_bytes(3, "big")


class MsmAdapterTest(unittest.TestCase):
    def test_navic_terminator_rewritten_without_observation_changes(self):
        adapter = F9pMsmAdapter()
        gps, bds, navic = msm(1077), msm(1127, epoch=986000), msm(1137, multiple=0)
        self.assertEqual(adapter.feed(gps, 1), [])
        self.assertEqual(adapter.feed(bds, 1.01), [])
        output = adapter.feed(navic, 1.02)
        self.assertEqual(output[0], (1, gps))
        corrected = output[1][1]
        self.assertEqual(corrected[:9], bds[:9])
        self.assertEqual(corrected[9], bds[9] & ~2)
        self.assertEqual(corrected[10:-3], bds[10:-3])
        self.assertEqual(crc24q(corrected[:-3]), int.from_bytes(corrected[-3:], "big"))
        self.assertEqual((adapter.rewritten, adapter.filtered), (1, 1))
        self.assertEqual(adapter.pending, [])

    def test_normal_terminator_and_non_msm_unchanged(self):
        adapter = F9pMsmAdapter()
        data = msm(1077, multiple=0)
        self.assertEqual(adapter.feed(data, 1), [(1, data)])
        static = frame(19, 1005)
        self.assertEqual(adapter.feed(static, 1), [(1, static)])
        self.assertEqual(adapter.rewritten, 0)

    def test_missing_end_or_station_switch_rejected(self):
        for second, stamp in ((msm(1077, epoch=2000), 1.1),
                               (msm(1097, station=622), 1.1), (msm(1137, multiple=0), 4)):
            adapter = F9pMsmAdapter(); adapter.feed(msm(1077), 1)
            with self.assertRaises(OSError): adapter.feed(second, stamp)
            self.assertEqual(adapter.pending, [])

    def test_split_bds_messages_preserved(self):
        adapter = F9pMsmAdapter(); data = msm(1127)
        adapter.feed(data, 1); adapter.feed(data, 1)
        output = adapter.feed(msm(1137, multiple=0), 1)
        self.assertEqual(len(output), 2)
        self.assertEqual(output[0][1], data)


class NtripProtocolTest(unittest.TestCase):
    def test_crc_known_vector(self):
        self.assertEqual(crc24q(b"123456789"), 0xCDE703)

    def test_all_rtcm_lengths_and_byte_boundaries(self):
        for length in (2, 255, 256, 511, 512, 767, 768, 1023):
            with self.subTest(length=length):
                parser = RtcmFrameParser()
                data = frame(length)
                result = []
                for byte in data:
                    result.extend(parser.feed(bytes([byte])))
                self.assertEqual(result, [data])
                self.assertEqual(parser.by_type, {1077: 1})

    def test_rtcm_noise_and_crc_recovery(self):
        parser = RtcmFrameParser()
        broken = bytearray(frame()); broken[-1] ^= 1
        good = frame(700)
        self.assertEqual(parser.feed(b"junk\xd3\xff" + broken + good), [good])
        self.assertEqual(parser.crc_errors, 1)

    def test_http_and_icy_fragmentation(self):
        data = frame(500)
        for header in (b"HTTP/1.1 200 OK\r\nContent-Type: gnss/data\r\n\r\n",
                       b"ICY 200 OK\r\n"):
            decoder = NtripResponse()
            result = b"".join(decoder.feed(bytes([b])) for b in header + data)
            self.assertEqual(result, data)

    def test_chunked_stream_arbitrary_fragmentation(self):
        data = frame(1023) + frame(600, 1127)
        chunks = [data[:10], data[10:800], data[800:]]
        wire = b"HTTP/1.1 200 OK\r\nTransfer-Encoding: Chunked\r\n\r\n"
        for chunk in chunks:
            wire += f"{len(chunk):X};ext=yes\r\n".encode() + chunk + b"\r\n"
        wire += b"0\r\n\r\n"
        for step in (1, 2, 7, 1024, len(wire)):
            decoder = NtripResponse()
            result = b"".join(decoder.feed(wire[i:i + step]) for i in range(0, len(wire), step))
            self.assertEqual(result, data)
            self.assertTrue(decoder.ended)

    def test_http_errors_and_wrong_content(self):
        for header in (b"HTTP/1.1 401 Denied\r\n\r\n", b"HTTP/1.1 502 Bad Gateway\r\n\r\n",
                       b"HTTP/1.1 2000 Not200\r\n\r\n", b"SOURCETABLE 200 OK\r\n\r\n",
                       b"HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n",
                       b"HTTP/1.1 200 OK\r\nContent-Encoding: gzip\r\n\r\n"):
            with self.subTest(header=header), self.assertRaises(OSError):
                NtripResponse().feed(header)

    def test_bad_chunk(self):
        header = b"HTTP/1.1 200 OK\r\nTransfer-Encoding: chunked\r\n\r\n"
        for body in (b"xyz\r\n", b"-1\r\n", b"1\r\naXX", b"1000001\r\n"):
            with self.assertRaises(OSError): NtripResponse().feed(header + body)

    def test_queue_is_bounded_and_stale_frames_refused(self):
        client = NtripClient("localhost", 2101, "test", "", "")
        for _ in range(client.frames.max_bytes // len(frame())):
            client.frames.put_nowait((time.monotonic(), frame()))
        with self.assertRaises(queue.Full): client.frames.put_nowait((0, frame()))
        self.assertEqual(client.take_frame(), frame())
        other = NtripClient("localhost", 2101, "test", "", "")
        other.frames.put_nowait((time.monotonic() - 3, frame()))
        with self.assertRaises(OSError): other.take_frame()

    def test_v2_request_and_network_session_feed_valid_frames_only(self):
        data = frame(700)
        broken = bytearray(frame()); broken[-1] ^= 1
        body = bytes(broken) + data
        wire = (b"HTTP/1.1 200 OK\r\nTransfer-Encoding: chunked\r\n\r\n" +
                f"{len(body):x}\r\n".encode() + body + b"\r\n")
        sock = mock.MagicMock()
        sock.__enter__.return_value = sock
        sock.recv.side_effect = [wire[:80], wire[80:500], wire[500:], b""]
        client = NtripClient("localhost", 2101, "test", "example", "test-only")
        with (mock.patch("ntrip_rtcm.socket.create_connection", return_value=sock) as connect,
              mock.patch.dict(os.environ, {"HTTP_PROXY": "http://127.0.0.1:1"}),
              self.assertRaises(OSError)):
            client._session()
        connect.assert_called_once_with(("localhost", 2101), timeout=3)
        request = sock.sendall.call_args[0][0]
        self.assertIn(b"Ntrip-Version: Ntrip/2.0\r\n", request)
        self.assertEqual(client.take_frame(), data)
        self.assertEqual(client.crc_errors, 1)
        self.assertEqual(client.frame_count, 1)

    def test_bnc_import_only_selected_endpoint(self):
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "test.bnc"
            path.write_text('[General]\nmountPoints="//sample:p%40ss@localhost:2101/WUH200CHN0 RTCM_3.3 0 0"\n')
            self.assertEqual(read_bnc_endpoint(path), ("localhost", 2101, "WUH200CHN0", "sample", "p@ss"))


class BridgeFlowTest(unittest.TestCase):
    def test_credit_and_partial_writes(self):
        flow = BridgeFlow(report(rx=100, tx=100))
        flow.record_write(23); flow.record_write(1001)
        self.assertEqual(flow.allowance(), 0)
        flow.update(report(ms=10100, rx=1124, tx=500))
        self.assertEqual(flow.allowance(), 400)
        flow.record_write(400)
        self.assertEqual(flow.allowance(), 0)

    def test_refuse_pending_and_not_ready(self):
        for r in (report(rx=1), [100, 0] + report()[2:]):
            with self.assertRaises(ValueError): BridgeFlow(r)

    def test_reset_and_errors_stop(self):
        for r in (report(ms=1), report(dropped=1), report(errors=1), report(tx=1)):
            flow = BridgeFlow(report())
            with self.assertRaises(OSError): flow.update(r)

    def test_heartbeat_timeout(self):
        flow = BridgeFlow(report())
        flow.report_at -= 3
        with self.assertRaises(OSError): flow.allowance()

    def test_missing_pa10_ack_despite_live_heartbeat(self):
        flow = BridgeFlow(report()); flow.record_write(100)
        flow.progress_at -= 3
        flow.update(report(ms=11000))
        with self.assertRaises(OSError): flow.allowance()

    def test_counter_wrap(self):
        flow = BridgeFlow(report(ms=0xFFFFFFF0, rx=0xFFFFFFF0, tx=0xFFFFFFF0))
        flow.record_write(32)
        flow.update(report(ms=100, rx=16, tx=16))
        self.assertEqual(flow.acked, 32)
        self.assertEqual(flow.allowance(), 1024)


class SeparateStageTimingTest(unittest.TestCase):
    def test_slow_msm_then_partial_uart_send_does_not_false_stop(self):
        clock = [10.0]
        with mock.patch("ntrip_rtcm.time.monotonic", side_effect=lambda: clock[0]):
            client = NtripClient("localhost", 2101, "test", "", "")
            # A maximum-sized first RTCM frame straddles the 1024-byte credit window.
            payload = msm(1077)[3:-3] + bytes(1023 - 22)
            data = b"\xd3\x03\xff" + payload
            gps = data + crc24q(data).to_bytes(3, "big")
            self.assertEqual(client.msm_adapter.feed(gps, clock[0]), [])
            clock[0] = 11.7
            self.assertEqual(client.msm_adapter.feed(msm(1127), clock[0]), [])
            clock[0] = 11.8
            outgoing = client.msm_adapter.feed(msm(1137, multiple=0), clock[0])
            client._enqueue_ready(outgoing)
            port = mock.Mock(); port.write.side_effect = len
            worker = SerialWorker(port)
            worker.report = report(); worker.report_at = clock[0]
            worker.set_source(client); worker._pump()
            self.assertEqual(worker.flow.sent, 1024)
            self.assertEqual(worker.pending[0].received_at, 10.0)
            self.assertEqual(worker.pending[0].ready_at, 11.8)
            clock[0] = 12.1  # Old timer = 2.1 s; actual send wait = 0.3 s.
            wire = ("#RTCM," + ",".join(map(str, report(ms=10300, rx=1024, tx=1024))) + "\n").encode()
            port.in_waiting = len(wire); port.read.return_value = wire
            worker._read(); worker._pump()
            self.assertIs(worker.source, client)
            self.assertEqual(b"".join(call.args[0] for call in port.write.call_args_list),
                             b"".join(data for _, data in outgoing))
            self.assertFalse(worker.pending)
            self.assertAlmostEqual(worker.timing_snapshot[0], 1.8)
            self.assertAlmostEqual(worker.timing_snapshot[1], 0.3)
            self.assertAlmostEqual(worker.timing_snapshot[2], 2.1)

    def test_real_send_stall_still_stops_with_immediate_diagnostics(self):
        clock = [10.0]
        with mock.patch("ntrip_rtcm.time.monotonic", side_effect=lambda: clock[0]):
            client = NtripClient("localhost", 2101, "test", "", "")
            client.frames.put_nowait(RtcmPacket(10.0, frame(), 9.0))
            port = mock.Mock(); port.write.return_value = 0
            worker = SerialWorker(port)
            worker.report = report(); worker.report_at = clock[0]
            messages = []; worker.diagnostic.connect(messages.append)
            worker.set_source(client); worker._pump()
            clock[0] = 12.01
            worker.flow.update(report(ms=12010))  # Reports arrive, but writes cannot progress.
            worker.report_at = clock[0]
            worker._pump(); worker._pump()
            self.assertIsNone(worker.source)
            self.assertTrue(client._stop_event.is_set())
            self.assertEqual(port.write.call_count, 1)  # No replay after stop.
            self.assertIn("发送等待=2.010s", messages[0])
            self.assertIn("串口待发=406B", messages[0])

    def test_total_residence_guard_and_queue_wait_are_not_reset(self):
        packet = RtcmPacket(10.0, frame(), 7.5)
        with self.assertRaisesRegex(OSError, "总驻留=4.100s"):
            packet.check_age(11.6)
        client = NtripClient("localhost", 2101, "test", "", "")
        client.frames.max_bytes = len(frame())
        client.frames.put_nowait(RtcmPacket(10.0, frame(), 9.0))
        with mock.patch("ntrip_rtcm.time.monotonic", return_value=12.01):
            with self.assertRaisesRegex(OSError, "发送等待=2.010s"):
                client.frames.put(RtcmPacket(10.0, frame(), 9.0), client._stop_event)

    def test_missing_uart_ack_still_stops(self):
        clock = [10.0]
        with mock.patch("ntrip_rtcm.time.monotonic", side_effect=lambda: clock[0]):
            client = NtripClient("localhost", 2101, "test", "", "")
            client.frames.put_nowait(RtcmPacket(10.0, frame()))
            port = mock.Mock(); port.write.side_effect = len
            worker = SerialWorker(port)
            worker.report = report(); worker.report_at = clock[0]
            worker.set_source(client); worker._pump()
            clock[0] = 12.01
            worker.flow.update(report(ms=12010))  # Fresh status, no forwarded-byte increment.
            errors = []; worker.rtcm_failed.connect(lambda _, message: errors.append(message))
            worker._pump()
            self.assertIsNone(worker.source)
            self.assertIn("未确认转发", errors[0])


class NetworkDebugTest(unittest.TestCase):
    def test_timeout_debug_distinguishes_silent_socket_and_invalid_body(self):
        for mode in ("silent", "corrupt", "header"):
            with self.subTest(mode=mode):
                client = NtripClient("localhost", 2101, "test", "private-user", "private-secret")
                messages = []
                client.diagnostic.connect(messages.append)
                clock = [100.0]
                calls = [0]
                valid = frame(8, 1005)
                corrupt = valid[:-1] + bytes([valid[-1] ^ 1])
                def receive(_size):
                    clock[0] += 0.5
                    calls[0] += 1
                    if calls[0] == 1:
                        if mode == "header": return b"HTTP/1.1 200 OK\r\n"
                        return b"HTTP/1.1 200 OK\r\n\r\n" + (valid if mode == "silent" else corrupt)
                    if mode == "corrupt": return corrupt
                    raise socket.timeout()
                sock = mock.MagicMock()
                sock.__enter__.return_value = sock
                sock.recv.side_effect = receive
                client._stop_event = mock.Mock()
                client._stop_event.is_set.return_value = False
                client._stop_event.wait.return_value = True
                with (mock.patch("ntrip_rtcm.time.monotonic", side_effect=lambda: clock[0]),
                      mock.patch("ntrip_rtcm.socket.create_connection", return_value=sock)):
                    client.run()
                self.assertEqual(client.reconnects, 1)
                output = "\n".join(messages)
                self.assertIn("预警：5 秒", output)
                self.assertIn("异常现场", output)
                self.assertIn("15 秒未收到有效 RTCM", output)
                self.assertNotIn("private-user", output)
                self.assertNotIn("private-secret", output)
                self.assertNotIn(client._auth, output)
                if mode == "silent":
                    self.assertGreater(client.recv_timeouts, 0)
                    self.assertEqual(client.parser.frame_count, 1)
                    self.assertEqual(client.parser.crc_errors, 0)
                elif mode == "corrupt":
                    self.assertGreater(client.parser.crc_errors, 0)
                    self.assertGreater(client.parser.resync_bytes, 0)
                    self.assertEqual(client.last_valid_at, None)
                    self.assertGreater(client.last_body_at, client.session_started + 15)
                else:
                    self.assertIn("HTTP头完成=0", output)
                    self.assertEqual(client.last_body_at, None)

    def test_session_debug_counters_reset_without_resetting_total(self):
        client = NtripClient("localhost", 2101, "test", "", "")
        client.socket_bytes = 100
        client.last_valid_at = time.monotonic()
        client.network_bytes = 999
        client._reset_debug()
        self.assertEqual(client.socket_bytes, 0)
        self.assertIsNone(client.last_valid_at)
        self.assertEqual(client.network_bytes, 999)


class MonitorBridgeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self): self.monitor = NavigationMonitor()
    def tearDown(self): self.monitor.close()

    def test_link_debug_records_serial_snapshot_without_port_io(self):
        m = self.monitor
        worker = SerialWorker(mock.Mock())
        worker.snapshot = (1234, 16, 0.25)
        worker.report = report(rx=1218, tx=1218)
        worker.report_at = time.monotonic()
        m.serial_worker = worker
        m._log_link_debug("网络调试测试")
        output = m.event_log.toPlainText()
        self.assertIn("已写/未确认=1234/16B", output)
        self.assertIn("收/转发=1218/1218B", output)
        worker.port.read.assert_not_called()
        worker.port.write.assert_not_called()
        m.serial_worker = None

    def test_event_log_keeps_reconnect_reason_and_recovery(self):
        m = self.monitor
        m.clear_event_log()
        with mock.patch("imu_serial_qt.time.monotonic", side_effect=[10, 15, 22]):
            m._set_ntrip_status("基站: 基站断开连接；5 秒后重连")
            m._set_ntrip_status("基站: 连接超时；5 秒后重连")
            m._set_ntrip_status("基站: 已直连 NTRIP v2，收到有效 RTCM")
        text = m.event_log.toPlainText()
        self.assertIn("基站断开连接", text)
        self.assertIn("连接超时", text)
        self.assertIn("距首次报错 12.0 秒", text)
        self.assertNotIn("断开连接", m.ntrip_label.text())
        self.assertIsNone(m._ntrip_interrupted_at)

    def test_event_log_bounded_and_clear_does_not_clear_data(self):
        m = self.monitor
        for i in range(5010): m.log_event("测试", f"event-{i}")
        self.assertEqual(m.event_log.document().blockCount(), 5000)
        self.assertNotIn("event-0\n", m.event_log.toPlainText())
        m.total_imu = 123
        m.clear_event_log()
        self.assertEqual(m.total_imu, 123)
        self.assertEqual(m.event_log.toPlainText(), "")

    def test_event_log_export_and_copy(self):
        m = self.monitor
        m.log_event("测试", "连接恢复")
        m.copy_event_log()
        self.assertEqual(self.app.clipboard().text(), m.event_log.toPlainText())
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "events.log"
            with mock.patch("imu_serial_qt.QtWidgets.QFileDialog.getSaveFileName",
                            return_value=(str(path), "")):
                m.export_event_log()
            self.assertEqual(path.read_text(encoding="utf-8"), m.event_log.toPlainText() + "\n")

    def test_error_log_aggregates_and_does_not_repeat_unchanged_counts(self):
        m = self.monitor
        m.clear_event_log()
        m.invalid_lines = 27
        m.update_stats(); m.update_stats()
        self.assertEqual(m.event_log.document().blockCount(), 1)
        self.assertIn("无效行 27", m.event_log.toPlainText())

    def test_rtcm_reports_do_not_flood_event_log(self):
        m = self.monitor
        m.clear_event_log()
        for i in range(100):
            m.process_line(("#RTCM," + ",".join(map(str, report(ms=10000+i)))).encode())
        self.assertEqual(m.event_log.toPlainText(), "")

    def test_status_is_not_invalid_or_csv_record(self):
        self.monitor.process_line(("#RTCM," + ",".join(map(str, report()))).encode())
        self.assertEqual(self.monitor.bridge_report, report())
        self.assertEqual(self.monitor.invalid_lines, 0)
        self.assertEqual(self.monitor.total_gnss, 0)
        self.monitor.process_line(b"#RTCM,broken")
        self.assertEqual(self.monitor.invalid_lines, 1)

    def test_same_serial_handle_partial_write(self):
        port = mock.Mock(); port.write.return_value = 17
        worker = SerialWorker(port)
        client = NtripClient("localhost", 2101, "test", "", "")
        client.frames.put_nowait((time.monotonic(), frame()))
        worker.report = report(); worker.report_at = time.monotonic()
        worker.set_source(client); worker._pump()
        self.assertEqual(worker.flow.sent, 17)
        self.assertEqual(worker.pending[0][1], frame()[17:])
        port.write.assert_called_once_with(frame())

    def test_serial_timeout_stops_without_replay(self):
        port = mock.Mock(); port.write.side_effect = OSError("timeout")
        worker = SerialWorker(port)
        client = NtripClient("localhost", 2101, "test", "", "")
        client.frames.put_nowait((time.monotonic(), frame()))
        worker.report = report(); worker.report_at = time.monotonic()
        worker.set_source(client); worker._pump(); worker._pump()
        self.assertIsNone(worker.source)
        self.assertEqual(port.write.call_count, 1)

    def test_small_frame_burst_uses_byte_capacity_and_batches(self):
        port = mock.Mock(); port.write.side_effect = len
        worker = SerialWorker(port)
        client = NtripClient("localhost", 2101, "test", "", "")
        data = frame(2)
        for _ in range(100): client.frames.put_nowait((time.monotonic(), data))
        worker.report = report(); worker.report_at = time.monotonic()
        worker.set_source(client); worker._pump()
        port.write.assert_called_once_with(data * 100)
        self.assertEqual(worker.flow.sent, 800)
        self.assertEqual(client.frames.qsize(), 0)

    def test_forwarding_continues_without_gui_events(self):
        import threading
        class FakeSerial:
            def __init__(self):
                self.lock = threading.Lock()
                self.rx = bytearray(("#RTCM," + ",".join(map(str, report())) + "\n").encode())
                self.tx = bytearray()
                self.closed = False
            @property
            def in_waiting(self):
                with self.lock: return len(self.rx)
            def read(self, count):
                with self.lock:
                    data = bytes(self.rx[:count]); del self.rx[:count]; return data
            def write(self, data):
                with self.lock:
                    self.tx.extend(data)
                    r = report(ms=10000 + len(self.tx), rx=len(self.tx), tx=len(self.tx))
                    self.rx.extend(("#RTCM," + ",".join(map(str, r)) + "\n").encode())
                return len(data)
            def close(self): self.closed = True
        port = FakeSerial(); worker = SerialWorker(port)
        client = NtripClient("localhost", 2101, "test", "", "")
        data = frame(700)
        for _ in range(30): client.frames.put_nowait((time.monotonic(), data))
        worker.set_source(client); worker.start()
        try:
            time.sleep(0.5)  # deliberately no QApplication.processEvents()
        finally:
            worker.stop(); self.assertTrue(worker.wait(2000))
        self.assertEqual(bytes(port.tx), data * 30)
        self.assertTrue(port.closed)
        self.assertEqual(worker.snapshot[1], 0)

    def test_rtk_flag_not_fix_type_drives_display(self):
        # NAV-PVT fixType remains 3 for both float and fixed RTK.
        for carr in (1, 2):
            line = f"GNSS,2435,1000,1,1000000,3,25,{3 | (carr << 6)},0,{carr},300000000,1140000000,40000,20,30,0,0,0,0,10,100"
            self.monitor.process_line(line.encode())
            self.assertIn("RTK", self.monitor.fix_label.text())
        self.assertEqual(self.monitor.invalid_lines, 0)

    def test_worker_received_records_preserve_three_csv_files(self):
        from test_serial_protocol import RAWX_HEADER, RAWX_MEAS
        monitor = self.monitor
        port = mock.Mock()
        worker = SerialWorker(port)
        monitor.serial_port = port; monitor.serial_worker = worker
        wire = (b"IMU,1,2435,1000000,1,12345,0,0,16384,0,0,0,0\n"
                b"GNSS,2435,1000,1,12345,3,25,131,0,2,300000000,1140000000,40000,20,30,0,0,0,0,10,100\n"
                + RAWX_HEADER + b"\n" + RAWX_MEAS + b"\nRAWX_END,1\n")
        with tempfile.TemporaryDirectory() as directory:
            with mock.patch.object(QtWidgets.QFileDialog, "getExistingDirectory", return_value=directory):
                self.assertTrue(monitor._open_logs())
            worker.received.put(wire)
            monitor.poll_serial(); monitor.disconnect_serial("test")
            session = next(pathlib.Path(directory).iterdir())
            self.assertEqual({p.name for p in session.iterdir()}, {"imu.csv", "gnss.csv", "rawx.csv"})
            for name in ("imu", "gnss", "rawx"):
                with (session / f"{name}.csv").open(newline="", encoding="utf-8-sig") as source:
                    rows = list(csv.DictReader(source))
                self.assertEqual(len(rows), 1)
                if name == "gnss": self.assertEqual(rows[0]["carr_soln"], "2")
        self.assertEqual(monitor.invalid_lines, 0)


if __name__ == "__main__":
    unittest.main()
