"""Real-time synchronized MPU6050/ZED-F9P navigation data monitor."""

from __future__ import annotations

import csv
import json
import math
import os
import pathlib
import struct
import sys
import time
import urllib.parse
from collections import deque

# QtWebEngine/Chromium hardware acceleration is unstable with some Windows
# display drivers. The map is only 1 Hz, so software composition is preferable
# to allowing a GPU-process hang to affect acquisition.
os.environ.setdefault("QTWEBENGINE_CHROMIUM_FLAGS", "--disable-gpu")

import numpy as np
import pyqtgraph as pg
import serial
from PyQt5 import QtCore, QtGui, QtWidgets
from serial.tools import list_ports
from serial_worker import SerialWorker
from ntrip_rtcm import (BridgeFlow, NtripClient, NtripSettingsDialog,
                       NTRIP_DEFAULT_HOST, NTRIP_DEFAULT_PORT, NTRIP_DEFAULT_MOUNT)

try:
    from PyQt5 import QtWebEngineWidgets
except ImportError:
    QtWebEngineWidgets = None

G0 = 9.80665
ACCEL_SCALE = G0 / 16384.0
GYRO_DEG_H_SCALE = 3600.0 / 131.0
GPS_WEEK_SECONDS = 604800.0
PROJECT_ROOT = pathlib.Path(__file__).resolve().parent.parent
DEFAULT_DATA_DIR = PROJECT_ROOT / "data" / "decoded"
GNSS_NAMES = {0: "GPS", 1: "SBAS", 2: "GAL", 3: "BDS", 4: "IMES", 5: "QZSS", 6: "GLO"}
GNSS_COLORS = {
    0: QtGui.QColor("#45a3ff"), 1: QtGui.QColor("#aaaaaa"),
    2: QtGui.QColor("#5bd18b"), 3: QtGui.QColor("#ffad42"),
    4: QtGui.QColor("#dddddd"), 5: QtGui.QColor("#d98cff"),
    6: QtGui.QColor("#ff6174"),
}
RAWX_COLUMNS = [
    "gps_week", "rcv_tow_s", "rx_timer_us", "leap_s", "rec_stat",
    "epoch_num_meas", "epoch_total_meas", "gnss_id", "sv_id", "sig_id",
    "freq_id", "signal", "frequency_mhz", "pseudorange_m",
    "carrier_phase_cycles", "doppler_hz", "locktime_ms", "cno_dbhz",
    "pr_stdev_m", "cp_stdev_cycles", "do_stdev_hz", "pr_valid",
    "cp_valid", "half_cycle", "sub_half_cycle",
]
SIGNALS = {
    (0, 0): ("GPS_L1CA", 1575.42), (0, 3): ("GPS_L2CL", 1227.60),
    (0, 4): ("GPS_L2CM", 1227.60), (1, 0): ("SBAS_L1CA", 1575.42),
    (2, 0): ("GAL_E1C", 1575.42), (2, 1): ("GAL_E1B", 1575.42),
    (2, 5): ("GAL_E5bI", 1207.14), (2, 6): ("GAL_E5bQ", 1207.14),
    (3, 0): ("BDS_B1I_D1", 1561.098), (3, 1): ("BDS_B1I_D2", 1561.098),
    (3, 2): ("BDS_B2I_D1", 1207.14), (3, 3): ("BDS_B2I_D2", 1207.14),
    (5, 0): ("QZSS_L1CA", 1575.42), (5, 4): ("QZSS_L2CM", 1227.60),
    (5, 5): ("QZSS_L2CL", 1227.60),
}


def signal_name_frequency(gnss_id: int, sig_id: int,
                          freq_id: int) -> tuple[str, float | str]:
    if gnss_id == 6 and sig_id in (0, 2):
        channel = freq_id - 7
        if sig_id == 0: return "GLO_L1OF", 1602.0 + channel * 0.5625
        return "GLO_L2OF", 1246.0 + channel * 0.4375
    return SIGNALS.get((gnss_id, sig_id), (f"GNSS{gnss_id}_SIG{sig_id}", ""))


def float_from_hex(value: str, size: int) -> float:
    return struct.unpack("<d" if size == 8 else "<f",
                         int(value, 16).to_bytes(size, "little"))[0]


def _outside_china(lat: float, lon: float) -> bool:
    return not (72.004 <= lon <= 137.8347 and 0.8293 <= lat <= 55.8271)


def wgs84_to_gcj02(lat: float, lon: float) -> tuple[float, float]:
    """Convert F9P WGS-84 coordinates for display on AMap in mainland China."""
    if _outside_china(lat, lon):
        return lat, lon

    def transform_lat(x: float, y: float) -> float:
        value = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y
        value += 0.1 * x * y + 0.2 * math.sqrt(abs(x))
        value += (20.0 * math.sin(6.0 * x * math.pi) + 20.0 * math.sin(2.0 * x * math.pi)) * 2.0 / 3.0
        value += (20.0 * math.sin(y * math.pi) + 40.0 * math.sin(y / 3.0 * math.pi)) * 2.0 / 3.0
        return value + (160.0 * math.sin(y / 12.0 * math.pi) + 320.0 * math.sin(y * math.pi / 30.0)) * 2.0 / 3.0

    def transform_lon(x: float, y: float) -> float:
        value = 300.0 + x + 2.0 * y + 0.1 * x * x
        value += 0.1 * x * y + 0.1 * math.sqrt(abs(x))
        value += (20.0 * math.sin(6.0 * x * math.pi) + 20.0 * math.sin(2.0 * x * math.pi)) * 2.0 / 3.0
        value += (20.0 * math.sin(x * math.pi) + 40.0 * math.sin(x / 3.0 * math.pi)) * 2.0 / 3.0
        return value + (150.0 * math.sin(x / 12.0 * math.pi) + 300.0 * math.sin(x / 30.0 * math.pi)) * 2.0 / 3.0

    a = 6378245.0
    eccentricity = 0.006693421622965943
    d_lat = transform_lat(lon - 105.0, lat - 35.0)
    d_lon = transform_lon(lon - 105.0, lat - 35.0)
    rad_lat = lat / 180.0 * math.pi
    magic = 1.0 - eccentricity * math.sin(rad_lat) ** 2
    sqrt_magic = math.sqrt(magic)
    d_lat = d_lat * 180.0 / ((a * (1.0 - eccentricity) / (magic * sqrt_magic)) * math.pi)
    d_lon = d_lon * 180.0 / ((a / sqrt_magic * math.cos(rad_lat)) * math.pi)
    return lat + d_lat, lon + d_lon


class SkyPlotWidget(QtWidgets.QWidget):
    """Polar satellite sky view: north up, horizon outside, zenith center."""

    def __init__(self) -> None:
        super().__init__()
        self.setMinimumSize(330, 300)
        self.satellites: list[dict[str, int]] = []

    def set_satellites(self, satellites: list[dict[str, int]]) -> None:
        self.satellites = satellites
        self.update()

    def paintEvent(self, _event: QtGui.QPaintEvent) -> None:  # noqa: N802
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        painter.fillRect(self.rect(), QtGui.QColor("#101418"))
        side = min(self.width(), self.height()) - 50
        radius = max(20.0, side / 2.0)
        center = QtCore.QPointF(self.width() / 2.0, self.height() / 2.0 + 8.0)
        painter.setPen(QtGui.QPen(QtGui.QColor("#53606b"), 1))
        for scale in (1.0, 2.0 / 3.0, 1.0 / 3.0):
            painter.drawEllipse(center, radius * scale, radius * scale)
        painter.drawLine(QtCore.QPointF(center.x() - radius, center.y()), QtCore.QPointF(center.x() + radius, center.y()))
        painter.drawLine(QtCore.QPointF(center.x(), center.y() - radius), QtCore.QPointF(center.x(), center.y() + radius))
        painter.setPen(QtGui.QColor("#d8dee9"))
        for text, dx, dy in (("N", -5, -radius - 7), ("E", radius + 7, 5),
                             ("S", -5, radius + 18), ("W", -radius - 18, 5)):
            painter.drawText(QtCore.QPointF(center.x() + dx, center.y() + dy), text)
        for sat in self.satellites:
            elev, azim = sat["elev"], sat["azim"]
            if not (0 <= elev <= 90 and 0 <= azim <= 360):
                continue
            distance = radius * (90.0 - elev) / 90.0
            angle = math.radians(azim)
            point = QtCore.QPointF(center.x() + distance * math.sin(angle), center.y() - distance * math.cos(angle))
            color = GNSS_COLORS.get(sat["gnss"], QtGui.QColor("#eeeeee"))
            painter.setBrush(color if sat["used"] else QtCore.Qt.NoBrush)
            painter.setPen(QtGui.QPen(color, 2 if sat["used"] else 1))
            painter.drawEllipse(point, 9.0, 9.0)
            painter.setPen(QtGui.QColor("#f1f3f5"))
            painter.drawText(QtCore.QPointF(point.x() + 10, point.y() + 4),
                             f"{GNSS_NAMES.get(sat['gnss'], '?')[0]}{sat['sv']}")
        painter.end()


class NavigationMapWidget(QtWidgets.QWidget):
    """Local metric track with an optional AMap Web JS view."""

    TRACK_EDGE_MARGIN = 0.03

    def __init__(self, settings: QtCore.QSettings) -> None:
        super().__init__()
        self.settings = settings
        self.origin: tuple[float, float] | None = None
        self.east_m: deque[float] = deque(maxlen=10000)
        self.north_m: deque[float] = deque(maxlen=10000)
        self.amap_loaded = False
        self.amap_loading = False
        self.last_wgs_position: tuple[float, float] | None = None
        layout = QtWidgets.QVBoxLayout(self); layout.setContentsMargins(0, 0, 0, 0)
        bar = QtWidgets.QHBoxLayout()
        self.key_edit = QtWidgets.QLineEdit(str(settings.value("amap_key", "")))
        self.key_edit.setPlaceholderText("高德 Web API Key（稍后填写）")
        self.security_edit = QtWidgets.QLineEdit(str(settings.value("amap_security", "")))
        self.security_edit.setPlaceholderText("securityJsCode（如需要）"); self.security_edit.setEchoMode(QtWidgets.QLineEdit.Password)
        self.load_button = QtWidgets.QPushButton("加载高德地图")
        self.local_button = QtWidgets.QPushButton("本地轨迹")
        self.fit_button = QtWidgets.QPushButton("最佳窗口")
        self.map_status = QtWidgets.QLabel("当前：本地轨迹")
        self.load_button.clicked.connect(self.load_amap)
        self.local_button.clicked.connect(self.show_local_map)
        self.fit_button.clicked.connect(self.fit_track)
        bar.addWidget(self.key_edit, 2); bar.addWidget(self.security_edit, 2)
        bar.addWidget(self.load_button); bar.addWidget(self.local_button); bar.addWidget(self.fit_button)
        bar.addWidget(self.map_status); layout.addLayout(bar)
        self.stack = QtWidgets.QStackedWidget()
        self.local_plot = pg.PlotWidget(); self.local_plot.setBackground("#101418")
        self.local_plot.showGrid(x=True, y=True, alpha=0.3)
        self.local_plot.setAspectLocked(True, ratio=1.0)
        self.local_plot.setLabel("left", "北向", units="m"); self.local_plot.setLabel("bottom", "东向", units="m")
        self.local_plot.setTitle("WGS-84 本地轨迹（高德 Key 未加载时使用）")
        # PlotCurveItem produces long horizontal path artifacts with the
        # Windows/PyQt5/pyqtgraph combination when a short metric track bends
        # back in x. Independent points render the same observations reliably.
        self.track_curve = pg.ScatterPlotItem(
            size=4, pen=None, brush=pg.mkBrush("#45a3ff"))
        self.local_plot.addItem(self.track_curve)
        self.position_dot = self.local_plot.plot(pen=None, symbol="o", symbolSize=12, symbolBrush="#ff5c5c")
        self.stack.addWidget(self.local_plot)
        self.web_view = None
        layout.addWidget(self.stack, 1)
        self.ready_timer = QtCore.QTimer(self); self.ready_timer.setInterval(250)
        self.ready_timer.timeout.connect(self._check_map_ready)
        self.load_timeout = QtCore.QTimer(self); self.load_timeout.setSingleShot(True)
        self.load_timeout.setInterval(15000); self.load_timeout.timeout.connect(
            lambda: self._map_load_failed("地图加载超时，请检查网络、Key 和安全密钥。")
        )

    def show_local_map(self) -> None:
        self.stack.setCurrentWidget(self.local_plot)
        self.map_status.setText("当前：本地轨迹")

    def fit_track(self) -> None:
        """Restore an equal-scale view fitted to all collected positions."""
        self.show_local_map()
        self._fit_local_track()

    def _fit_local_track(self) -> None:
        self.local_plot.setAspectLocked(True, ratio=1.0)
        if self.east_m:
            self.local_plot.getViewBox().autoRange(padding=0.12)
        else:
            self.local_plot.setRange(xRange=(-5.0, 5.0), yRange=(-5.0, 5.0),
                                     padding=0.0)

    def _track_needs_fit(self) -> bool:
        if not self.east_m:
            return False
        (x_min, x_max), (y_min, y_max) = self.local_plot.viewRange()
        x_margin = max((x_max - x_min) * self.TRACK_EDGE_MARGIN, 1e-6)
        y_margin = max((y_max - y_min) * self.TRACK_EDGE_MARGIN, 1e-6)
        return (min(self.east_m) <= x_min + x_margin or
                max(self.east_m) >= x_max - x_margin or
                min(self.north_m) <= y_min + y_margin or
                max(self.north_m) >= y_max - y_margin)

    def clear_track(self) -> None:
        self.origin = None; self.east_m.clear(); self.north_m.clear()
        self.track_curve.clear(); self.position_dot.clear()
        if self.amap_loaded and self.web_view is not None:
            self.web_view.page().runJavaScript("clearTrack();")

    def set_position(self, lat: float, lon: float) -> None:
        self.last_wgs_position = lat, lon
        if self.origin is None:
            self.origin = lat, lon
        lat0, lon0 = self.origin
        east = (lon - lon0) * 111319.4908 * math.cos(math.radians(lat0))
        north = (lat - lat0) * 110574.0
        self.east_m.append(east); self.north_m.append(north)
        self.track_curve.setData(list(self.east_m), list(self.north_m)); self.position_dot.setData([east], [north])
        if self._track_needs_fit():
            self._fit_local_track()
        if self.amap_loaded and self.web_view is not None:
            gcj_lat, gcj_lon = wgs84_to_gcj02(lat, lon)
            self.web_view.page().runJavaScript(f"updatePosition({gcj_lon:.9f},{gcj_lat:.9f});")

    def load_amap(self) -> None:
        key, security = self.key_edit.text().strip(), self.security_edit.text().strip()
        if not key:
            QtWidgets.QMessageBox.information(self, "高德地图", "请先填写高德 Web API Key。")
            return
        if not security:
            QtWidgets.QMessageBox.information(
                self, "缺少安全密钥",
                "还需要填写与该 Web API Key 配套的 securityJsCode。\n"
                "未填写时继续加载会出现空白或鉴权失败。",
            )
            return
        if QtWebEngineWidgets is None:
            QtWidgets.QMessageBox.warning(self, "缺少组件", "请安装 PyQtWebEngine：\n\npip install PyQtWebEngine")
            return
        if self.web_view is None:
            self.web_view = QtWebEngineWidgets.QWebEngineView()
            self.web_view.loadFinished.connect(self._page_load_finished)
            self.stack.addWidget(self.web_view)
        if self.amap_loading:
            return
        self.settings.setValue("amap_key", key); self.settings.setValue("amap_security", security)
        self.amap_loaded = False; self.amap_loading = True
        self.load_button.setEnabled(False); self.map_status.setText("高德地图：正在加载…")
        key_url, security_json = urllib.parse.quote(key, safe=""), json.dumps(security)
        page = f"""<!doctype html><html><head><meta charset='utf-8'>
<style>html,body,#map{{width:100%;height:100%;margin:0;background:#101418}}</style>
<script>window._AMapSecurityConfig={{securityJsCode:{security_json}}};</script>
<script src='https://webapi.amap.com/maps?v=2.0&key={key_url}'></script></head>
<body><div id='map'></div><script>
window.mapReady=false;window.mapError='';let map=null,marker=null,path=[],line=null;
try{{
 if(typeof AMap==='undefined')throw new Error('AMap JavaScript API 未加载');
 map=new AMap.Map('map',{{zoom:17,viewMode:'2D'}});
 line=new AMap.Polyline({{strokeColor:'#1677ff',strokeWeight:5,map:map}});
 window.mapReady=true;
}}catch(error){{window.mapError=String(error);}}
function updatePosition(lon,lat){{if(!window.mapReady)return;let p=new AMap.LngLat(lon,lat);path.push(p);line.setPath(path);
 if(!marker){{marker=new AMap.Marker({{position:p,map:map}});map.setCenter(p);}}else marker.setPosition(p);
 if(path.length<3)map.setCenter(p);}}
function clearTrack(){{if(!window.mapReady)return;path=[];line.setPath(path);if(marker){{map.remove(marker);marker=null;}}}}
</script></body></html>"""
        self.web_view.setHtml(page, QtCore.QUrl("https://localhost/"))
        self.load_timeout.start()

    def _page_load_finished(self, ok: bool) -> None:
        if not self.amap_loading:
            return
        if not ok:
            self._map_load_failed("地图页面加载失败，请检查网络或高德 Key 配置。")
            return
        self.ready_timer.start()

    def _check_map_ready(self) -> None:
        if self.web_view is None or not self.amap_loading:
            return
        script = "JSON.stringify({ready:window.mapReady===true,error:window.mapError||''})"
        self.web_view.page().runJavaScript(script, self._map_ready_result)

    def _map_ready_result(self, result: object) -> None:
        if not self.amap_loading or not isinstance(result, str):
            return
        try:
            state = json.loads(result)
        except (TypeError, ValueError):
            return
        if state.get("ready"):
            self.ready_timer.stop(); self.load_timeout.stop()
            self.amap_loading = False; self.amap_loaded = True
            self.load_button.setEnabled(True); self.map_status.setText("高德地图：已加载")
            self.stack.setCurrentWidget(self.web_view)
            if self.last_wgs_position is not None:
                lat, lon = self.last_wgs_position
                gcj_lat, gcj_lon = wgs84_to_gcj02(lat, lon)
                self.web_view.page().runJavaScript(f"updatePosition({gcj_lon:.9f},{gcj_lat:.9f});")
        elif state.get("error"):
            self._map_load_failed(str(state["error"]))

    def _map_load_failed(self, reason: str) -> None:
        if not self.amap_loading and not self.amap_loaded:
            return
        self.ready_timer.stop(); self.load_timeout.stop()
        self.amap_loading = False; self.amap_loaded = False
        self.load_button.setEnabled(True); self.show_local_map()
        self.map_status.setText("高德地图：加载失败")
        QtWidgets.QMessageBox.warning(self, "高德地图加载失败", reason)


class NavigationMonitor(QtWidgets.QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.settings = QtCore.QSettings("MPU6050F9P", "NavigationMonitor")
        # Preserve keys entered with the predecessor application's settings
        # namespace while keeping all secrets outside the repository.
        legacy_settings = QtCore.QSettings("lmy91", "GnssImuNavigationMonitor")
        for setting_name in ("amap_key", "amap_security"):
            if not self.settings.contains(setting_name) and legacy_settings.contains(setting_name):
                self.settings.setValue(setting_name, legacy_settings.value(setting_name))
        self.setWindowTitle("MPU6050/F9P 实时同步导航采集系统"); self.resize(1560, 980)
        self.serial_port: serial.Serial | None = None
        self.serial_worker = None
        self.rx_buffer = bytearray(); self.discard_until_newline = False
        self.plot_paused = False
        self.imu_stream = self.gnss_stream = self.rawx_stream = None
        self.event_stream = None
        self.imu_writer: csv.writer | None = None
        self.gnss_writer: csv.writer | None = None
        self.rawx_writer: csv.writer | None = None
        self.rows_since_flush = 0
        self.imu = {name: deque() for name in ("time", "ax", "ay", "az", "gx", "gy", "gz", "temp")}
        self.speed = {name: deque() for name in ("time", "vn", "ve", "vd", "ground")}
        self.last_sample: int | None = None
        self.first_timer_us: int | None = None
        self.last_timer_us: int | None = None
        self.first_gnss_time: float | None = None
        self.total_imu = self.total_gnss = self.total_rawx = self.lost_imu = self.invalid_lines = 0
        self.last_dt_ms = 0.0; self.arrivals: deque[float] = deque()
        self.satellite_epoch: tuple[int, int] | None = None
        self.pending_satellites: list[dict[str, int]] = []
        self.rawx_epoch: dict[str, int | float] | None = None
        self.rawx_seen_header = False
        self.gnss_diagnostics = {}
        self.ntrip_client = None
        self.ntrip_workers = []
        self.ntrip_password = str(self.settings.value("ntrip_password", ""))
        self.bridge_report = None
        self.bridge_report_at = 0.0
        self._ntrip_interrupted_at = None
        self._logged_fix = None
        self._logged_errors = (0, 0)
        self._build_ui(); self._build_timers(); self.refresh_ports()
        self.log_event("系统", "程序已启动；界面保留最近 5000 条，勾选 LOG 可随采集保存，也可手动导出。")

    def _build_ui(self) -> None:
        pg.setConfigOptions(antialias=False, background="#101418", foreground="#d8dee9")
        central = QtWidgets.QWidget(); outer = QtWidgets.QVBoxLayout(central)
        outer.setContentsMargins(8, 8, 8, 8)
        controls = QtWidgets.QHBoxLayout()
        self.port_combo = QtWidgets.QComboBox(); self.port_combo.setMinimumWidth(210)
        self.refresh_button = QtWidgets.QPushButton("刷新串口"); self.refresh_button.clicked.connect(self.refresh_ports)
        self.baud_combo = QtWidgets.QComboBox(); self.baud_combo.addItems(["460800", "115200", "230400", "921600"])
        self.connect_button = QtWidgets.QPushButton("连接"); self.connect_button.clicked.connect(self.toggle_connection)
        self.pause_button = QtWidgets.QPushButton("暂停绘图"); self.pause_button.clicked.connect(self.toggle_pause); self.pause_button.setEnabled(False)
        self.clear_button = QtWidgets.QPushButton("清空曲线"); self.clear_button.clicked.connect(self.clear_data)
        self.window_spin = QtWidgets.QSpinBox(); self.window_spin.setRange(10, 3600); self.window_spin.setValue(120); self.window_spin.setSuffix(" s")
        self.save_checkboxes: dict[str, QtWidgets.QCheckBox] = {}
        for name in ("IMU", "GNSS", "RAWX", "LOG"):
            checkbox = QtWidgets.QCheckBox(name); checkbox.setChecked(name != "LOG")
            self.save_checkboxes[name] = checkbox
        self.save_checkboxes["LOG"].setToolTip("连接前勾选：本次日志保存到采集文件夹中的 event.log")
        self.select_all_button = QtWidgets.QPushButton("全选")
        self.select_all_button.clicked.connect(self.select_all_logs)
        self.connection_label = QtWidgets.QLabel("● 未连接"); self.connection_label.setStyleSheet("color:#ff6174;font-weight:bold")
        for text, widget in (("串口", self.port_combo), ("波特率", self.baud_combo), ("窗口", self.window_spin)):
            controls.addWidget(QtWidgets.QLabel(text)); controls.addWidget(widget)
        for widget in (self.refresh_button, self.connect_button, self.pause_button, self.clear_button):
            controls.addWidget(widget)
        controls.addWidget(QtWidgets.QLabel("保存:"))
        for checkbox in self.save_checkboxes.values(): controls.addWidget(checkbox)
        controls.addWidget(self.select_all_button)
        controls.addStretch(1); controls.addWidget(self.connection_label); outer.addLayout(controls)

        stats = QtWidgets.QHBoxLayout()
        self.rate_label = QtWidgets.QLabel("IMU: -- Hz"); self.frames_label = QtWidgets.QLabel("IMU/GNSS: 0 / 0")
        self.loss_label = QtWidgets.QLabel("丢帧: 0"); self.time_label = QtWidgets.QLabel("GPS时间: 无效")
        self.fix_label = QtWidgets.QLabel("定位: --"); self.sv_label = QtWidgets.QLabel("卫星数: --")
        self.pdop_label = QtWidgets.QLabel("PDOP: --")
        for label in (self.rate_label, self.frames_label, self.loss_label, self.time_label,
                      self.fix_label, self.sv_label, self.pdop_label):
            label.setMinimumWidth(135); stats.addWidget(label)
        stats.addStretch(1); outer.addLayout(stats)
        ntrip_controls = QtWidgets.QHBoxLayout()
        self.ntrip_settings_button = QtWidgets.QPushButton("基站设置…")
        self.ntrip_settings_button.clicked.connect(self.configure_ntrip)
        self.ntrip_button = QtWidgets.QPushButton("连接基站")
        self.ntrip_button.clicked.connect(self.toggle_ntrip)
        self.ntrip_label = QtWidgets.QLabel("基站: 未连接（Qt 直连，不使用系统 HTTP 代理）")
        ntrip_controls.addWidget(self.ntrip_settings_button); ntrip_controls.addWidget(self.ntrip_button)
        ntrip_controls.addWidget(self.ntrip_label, 1); outer.addLayout(ntrip_controls)
        self.rtcm_label = QtWidgets.QLabel("RTCM: 等待 STM32 转发状态；USB-TTL TX 须接 PA10")
        self.rtcm_label.setWordWrap(True)
        self.rtcm_label.setToolTip("STM32/F9P 计数自下位机启动累计，包括启动配置阶段。关注测试期间增量。距接收不是观测历元差分龄期。")
        outer.addWidget(self.rtcm_label)
        self.tabs = QtWidgets.QTabWidget(); self.tabs.addTab(self._build_navigation_tab(), "导航")
        self.tabs.addTab(self._build_imu_tab(), "IMU")
        self.tabs.addTab(self._build_event_log_tab(), "日志")
        outer.addWidget(self.tabs, 1)
        self.setCentralWidget(central)
        self.statusBar().showMessage("选择 PA9 对应的 USB-TTL 串口，默认 460800 bit/s。")

    def _build_event_log_tab(self) -> QtWidgets.QWidget:
        page = QtWidgets.QWidget(); layout = QtWidgets.QVBoxLayout(page)
        controls = QtWidgets.QHBoxLayout()
        self.log_follow = QtWidgets.QCheckBox("自动滚动"); self.log_follow.setChecked(True)
        controls.addWidget(self.log_follow)
        for title, callback in (("复制日志", self.copy_event_log),
                                ("导出日志…", self.export_event_log),
                                ("清空日志", self.clear_event_log)):
            button = QtWidgets.QPushButton(title); button.clicked.connect(callback)
            controls.addWidget(button)
        controls.addStretch(1)
        self.event_save_label = QtWidgets.QLabel("本机时间 · 界面最近 5000 条 · LOG 未保存")
        controls.addWidget(self.event_save_label)
        layout.addLayout(controls)
        self.event_log = QtWidgets.QPlainTextEdit()
        self.event_log.setReadOnly(True)
        self.event_log.setMaximumBlockCount(5000)
        self.event_log.setLineWrapMode(QtWidgets.QPlainTextEdit.NoWrap)
        layout.addWidget(self.event_log)
        return page

    def log_event(self, category: str, message: str) -> None:
        # UI-thread events only: never append every IMU/RTCM packet.
        message = " ".join(str(message).splitlines())[:2000]
        bar = self.event_log.verticalScrollBar(); previous = bar.value()
        stamp = QtCore.QDateTime.currentDateTime().toString("yyyy-MM-dd HH:mm:ss.zzz")
        line = f"{stamp} [{category}] {message}"
        self.event_log.appendPlainText(line)
        bar.setValue(bar.maximum() if self.log_follow.isChecked() else previous)
        if self.event_stream is not None:
            try:
                self.event_stream.write(line + "\n")
                self.event_stream.flush()  # Low-rate events only, independent of CSV row counts.
            except OSError as error:
                self._event_save_failed(error)

    def _event_save_failed(self, error: OSError) -> None:
        stream, self.event_stream = self.event_stream, None
        if stream is not None:
            try: stream.close()
            except OSError: pass
        self.event_save_label.setText("LOG 保存失败，请检查磁盘；采集继续")
        self.statusBar().showMessage(f"LOG 保存失败：{error}；请手动导出界面日志。")
        # The stream is disabled first, so this cannot recurse on a write failure.
        self.log_event("文件错误", f"LOG 保存失败：{error}；后续仅显示，CSV 采集继续。")

    def clear_event_log(self) -> None:
        self.event_log.clear()

    def copy_event_log(self) -> None:
        QtWidgets.QApplication.clipboard().setText(self.event_log.toPlainText())

    def export_event_log(self) -> None:
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "导出当前日志", f"navigation_{time.strftime('%Y%m%d%H%M%S')}.log",
            "日志文件 (*.log);;文本文件 (*.txt)")
        if not path: return
        if (self.event_stream is not None and
                pathlib.Path(path).resolve() == pathlib.Path(self.event_stream.name).resolve()):
            QtWidgets.QMessageBox.warning(self, "日志正在保存", "不能覆盖正在写入的 event.log，请选择其他文件。")
            return
        try:
            pathlib.Path(path).write_text(self.event_log.toPlainText() + "\n", encoding="utf-8")
        except OSError as error:
            self.log_event("错误", f"日志导出失败：{error}")
            QtWidgets.QMessageBox.warning(self, "导出失败", str(error))

    def _set_ntrip_status(self, message: str) -> None:
        self.ntrip_label.setText(message)
        now = time.monotonic()
        if "后重连" in message:
            if self._ntrip_interrupted_at is None: self._ntrip_interrupted_at = now
            self.log_event("基站重连", message)
        elif "收到有效 RTCM" in message:
            extra = ""
            if self._ntrip_interrupted_at is not None:
                extra = f"；距首次报错 {now - self._ntrip_interrupted_at:.1f} 秒（非差分龄期）"
            self._ntrip_interrupted_at = None
            self.log_event("基站", message + extra)
        else:
            self.log_event("基站", message)

    def _log_link_debug(self, message: str) -> None:
        self.log_event("NTRIP调试", message)
        worker = self.serial_worker
        if worker is None:
            self.log_event("链路调试", "无串口线程")
            return
        sent, pending, max_age = worker.snapshot
        detail = (f"串口线程运行={int(worker.isRunning())} 已写/未确认={sent}/{pending}B "
                  f"历史最大发送等待={max_age:.2f}s GUI接收待处理={worker.received.qsize()}块(每块≤4096B) "
                  f"GUI行缓存={len(self.rx_buffer)}B；IMU/GNSS/RAWX="
                  f"{self.total_imu}/{self.total_gnss}/{self.total_rawx} "
                  f"丢帧/无效行={self.lost_imu}/{self.invalid_lines}；{self.fix_label.text()}")
        assembly, sending, total = worker.timing_snapshot
        detail += f"；历史最大组包/发送等待/总驻留={assembly:.3f}/{sending:.3f}/{total:.3f}s"
        report = worker.report  # Replaced, never mutated by the serial owner.
        if report is not None:
            detail += (f"；距串口ACK={time.monotonic()-worker.report_at:.2f}s "
                       f"STM32运行={report[0]}ms 就绪={report[1]} 收/转发={report[2]}/{report[3]}B "
                       f"丢字节/串口错/GNSS溢出={report[4]}/{report[5]}/{report[6]} "
                       f"F9P收/使用/CRC错={report[7]}/{report[8]}/{report[9]} "
                       f"站号/类型={report[10]}/{report[11]}（设备计数自启动累计）")
        self.log_event("链路调试", detail + "；异步近似快照，非同一时刻原子采样")

    @staticmethod
    def _configure_plot(plot: pg.PlotItem, title: str, y_name: str, units: str) -> None:
        plot.setTitle(title); plot.setLabel("left", y_name, units=units)
        plot.showGrid(x=True, y=True, alpha=0.25); plot.setClipToView(True)
        plot.setDownsampling(auto=True, mode="peak")

    def _build_navigation_tab(self) -> QtWidgets.QWidget:
        tab = QtWidgets.QWidget(); grid = QtWidgets.QGridLayout(tab)
        self.map_widget = NavigationMapWidget(self.settings); grid.addWidget(self.map_widget, 0, 0, 1, 2)
        right = QtWidgets.QWidget(); right_layout = QtWidgets.QVBoxLayout(right)
        position_box = QtWidgets.QGroupBox("当前位置（F9P / WGS-84）"); form = QtWidgets.QFormLayout(position_box)
        self.lat_value = QtWidgets.QLabel("--°"); self.lon_value = QtWidgets.QLabel("--°")
        self.height_value = QtWidgets.QLabel("-- m"); self.speed_value = QtWidgets.QLabel("-- m/s")
        self.velocity_value = QtWidgets.QLabel("N --  E --  D -- m/s")
        form.addRow("纬度", self.lat_value); form.addRow("经度", self.lon_value)
        form.addRow("高程", self.height_value); form.addRow("地速", self.speed_value)
        form.addRow("NED速度", self.velocity_value); right_layout.addWidget(position_box)
        self.sky_plot = SkyPlotWidget(); right_layout.addWidget(self.sky_plot, 1); grid.addWidget(right, 0, 2, 2, 1)
        self.speed_graph = pg.PlotWidget(); self.speed_plot = self.speed_graph.getPlotItem()
        self._configure_plot(self.speed_plot, "GNSS 速度曲线（1 Hz）", "速度", "m/s")
        self.speed_plot.setLabel("bottom", "相对时间", units="s"); self.speed_plot.addLegend(offset=(8, 8))
        self.speed_curves = {
            "vn": self.speed_plot.plot(pen=pg.mkPen("#45a3ff", width=1.5), name="Vn"),
            "ve": self.speed_plot.plot(pen=pg.mkPen("#ffad42", width=1.5), name="Ve"),
            "vd": self.speed_plot.plot(pen=pg.mkPen("#d98cff", width=1.5), name="Vd"),
            "ground": self.speed_plot.plot(pen=pg.mkPen("#5bd18b", width=2), name="Ground"),
        }
        grid.addWidget(self.speed_graph, 1, 0, 1, 2)
        grid.setColumnStretch(0, 2); grid.setColumnStretch(1, 2); grid.setColumnStretch(2, 2)
        grid.setRowStretch(0, 3); grid.setRowStretch(1, 2); return tab

    def _build_imu_tab(self) -> QtWidgets.QWidget:
        tab = QtWidgets.QWidget(); layout = QtWidgets.QVBoxLayout(tab)
        graphics = pg.GraphicsLayoutWidget(); layout.addWidget(graphics, 1)
        self.accel_plot = graphics.addPlot(row=0, col=0); self.gyro_plot = graphics.addPlot(row=1, col=0)
        self.temp_plot = graphics.addPlot(row=2, col=0); self.gyro_plot.setXLink(self.accel_plot); self.temp_plot.setXLink(self.accel_plot)
        self._configure_plot(self.accel_plot, "三轴加速度", "加速度", "m/s²")
        self._configure_plot(self.gyro_plot, "三轴角速度", "角速度", "deg/h")
        self._configure_plot(self.temp_plot, "温度", "温度", "°C"); self.temp_plot.setLabel("bottom", "相对时间", units="s")
        self.accel_plot.addLegend(offset=(8, 8)); self.gyro_plot.addLegend(offset=(8, 8))
        self.accel_curves = {
            "ax": self.accel_plot.plot(pen="#45a3ff", name="Ax"), "ay": self.accel_plot.plot(pen="#ffad42", name="Ay"),
            "az": self.accel_plot.plot(pen="#5bd18b", name="Az")}
        self.gyro_curves = {
            "gx": self.gyro_plot.plot(pen="#ff6174", name="Gx"), "gy": self.gyro_plot.plot(pen="#d98cff", name="Gy"),
            "gz": self.gyro_plot.plot(pen="#d4a373", name="Gz")}
        self.temp_curve = self.temp_plot.plot(pen=pg.mkPen("#ff5c5c", width=1.5))
        axes = QtWidgets.QHBoxLayout(); axes.addWidget(QtWidgets.QLabel("显示轴:"))
        for name, curve in {**self.accel_curves, **self.gyro_curves}.items():
            check = QtWidgets.QCheckBox(name.upper()); check.setChecked(True); check.toggled.connect(curve.setVisible); axes.addWidget(check)
        axes.addStretch(1); layout.addLayout(axes); return tab

    def _build_timers(self) -> None:
        self.serial_timer = QtCore.QTimer(self); self.serial_timer.setInterval(10)
        self.serial_timer.timeout.connect(self.poll_serial); self.serial_timer.start()
        self.plot_timer = QtCore.QTimer(self); self.plot_timer.setInterval(100)
        self.plot_timer.timeout.connect(self.update_plots); self.plot_timer.start()
        self.stats_timer = QtCore.QTimer(self); self.stats_timer.setInterval(500)
        self.stats_timer.timeout.connect(self.update_stats); self.stats_timer.start()

    def refresh_ports(self) -> None:
        selected = self.port_combo.currentData(); self.port_combo.clear()
        ports = sorted(list_ports.comports(), key=lambda item: item.device); preferred = restored = -1
        for index, port in enumerate(ports):
            description = port.description or "未知设备"
            self.port_combo.addItem(f"{port.device} — {description}", port.device)
            if "USB" in description.upper() and "ZED" not in description.upper(): preferred = index
            if selected == port.device: restored = index
        if restored >= 0: self.port_combo.setCurrentIndex(restored)
        elif preferred >= 0: self.port_combo.setCurrentIndex(preferred)

    def toggle_connection(self) -> None:
        if self.serial_port is None: self.connect_serial()
        else: self.disconnect_serial("用户断开。")

    def select_all_logs(self) -> None:
        for checkbox in self.save_checkboxes.values():
            checkbox.setChecked(True)

    def _set_log_controls_enabled(self, enabled: bool) -> None:
        for checkbox in self.save_checkboxes.values():
            checkbox.setEnabled(enabled)
        self.select_all_button.setEnabled(enabled)

    @staticmethod
    def _create_session_directory(parent: pathlib.Path, stamp: str) -> pathlib.Path:
        for index in range(1000):
            name = stamp if index == 0 else f"{stamp}_{index:02d}"
            directory = parent / name
            try:
                directory.mkdir()
                return directory
            except FileExistsError:
                continue
        raise OSError(f"无法为采集时间 {stamp} 创建唯一文件夹")

    def _open_logs(self) -> bool:
        self.event_save_label.setText("本机时间 · 界面最近 5000 条 · LOG 未保存")
        self.event_save_label.setToolTip("")
        selected = {name for name, checkbox in self.save_checkboxes.items()
                    if checkbox.isChecked()}
        if not selected:
            self.log_event("记录", "未选择数据文件，仅实时显示。")
            self.statusBar().showMessage("未选择数据文件，仅实时显示。")
            return True
        DEFAULT_DATA_DIR.mkdir(parents=True, exist_ok=True)
        parent = QtWidgets.QFileDialog.getExistingDirectory(self, "选择数据保存目录", str(DEFAULT_DATA_DIR))
        if not parent: return False
        stamp = time.strftime("%Y%m%d%H%M%S")
        try:
            directory = self._create_session_directory(pathlib.Path(parent), stamp)
            created: list[str] = []
            if "LOG" in selected:
                path = directory / "event.log"
                self.event_stream = path.open("w", encoding="utf-8", newline="\n")
                self.event_save_label.setText("LOG 保存中：event.log（完整会话，不限 5000 条）")
                self.event_save_label.setToolTip(str(path))
                created.append(path.name)
            if "IMU" in selected:
                path = directory / "imu.csv"
                self.imu_stream = path.open("w", newline="", encoding="utf-8")
                self.imu_writer = csv.writer(self.imu_stream)
                self.imu_writer.writerow(["sample", "gps_week", "gps_tow_us", "time_valid", "timer_us", "time_s", "dt_s",
                    "ax_raw", "ay_raw", "az_raw", "temp_raw", "gx_raw", "gy_raw", "gz_raw",
                    "ax_m_s2", "ay_m_s2", "az_m_s2", "temp_deg_c", "gx_deg_h", "gy_deg_h", "gz_deg_h"])
                created.append(path.name)
            if "GNSS" in selected:
                path = directory / "gnss.csv"
                self.gnss_stream = path.open("w", newline="", encoding="utf-8")
                self.gnss_writer = csv.writer(self.gnss_stream)
                self.gnss_writer.writerow(["gps_week", "gps_tow_ms", "time_valid", "rx_timer_us", "fix", "num_sv",
                    "flags", "flags2", "carr_soln", "gnss_fix_ok", "diff_soln", "lat_deg", "lon_deg",
                    "hmsl_m", "h_acc_m", "v_acc_m", "vel_n_m_s", "vel_e_m_s", "vel_d_m_s",
                    "ground_speed_m_s", "s_acc_m_s", "pdop"])
                created.append(path.name)
            if "RAWX" in selected:
                path = directory / "rawx.csv"
                self.rawx_stream = path.open("w", newline="", encoding="utf-8")
                self.rawx_writer = csv.writer(self.rawx_stream)
                self.rawx_writer.writerow(RAWX_COLUMNS)
                created.append(path.name)
            self.log_event("记录", f"数据目录：{directory}；文件：" + "、".join(created))
            if "LOG" in selected and self.event_stream is None:
                raise OSError("LOG 初始写入失败，未开始采集")
            self.statusBar().showMessage(
                f"保存到 {directory.name}\\" + "、".join(created)); return True
        except OSError as error:
            self.log_event("错误", f"创建数据文件失败：{error}")
            self._close_logs(); QtWidgets.QMessageBox.critical(self, "文件错误", f"无法创建数据文件：\n{error}"); return False

    def _close_logs(self) -> None:
        for stream in (self.imu_stream, self.gnss_stream, self.rawx_stream):
            if stream is not None: stream.flush(); stream.close()
        self.imu_stream = self.gnss_stream = self.rawx_stream = None
        self.imu_writer = self.gnss_writer = self.rawx_writer = None; self.rows_since_flush = 0
        if self.event_stream is not None:
            self.log_event("记录", "本次采集文件关闭，LOG 记录结束。")
            if self.event_stream is not None:
                try:
                    self.event_stream.close()
                except OSError as error:
                    self._event_save_failed(error)
                else:
                    self.event_stream = None
                    self.event_save_label.setText("LOG 已保存并关闭")

    def connect_serial(self) -> None:
        device = self.port_combo.currentData()
        if not device:
            QtWidgets.QMessageBox.warning(self, "没有串口", "没有发现可用串口。"); return
        if not self._open_logs(): return
        try:
            port = serial.Serial(device, int(self.baud_combo.currentText()), timeout=0, write_timeout=0)
            port.dtr = False; port.rts = False; port.reset_input_buffer()
        except (serial.SerialException, OSError) as error:
            self.log_event("串口错误", f"无法打开 {device}：{error}")
            self._close_logs(); QtWidgets.QMessageBox.critical(self, "串口连接失败", f"无法打开 {device}：\n{error}"); return
        self.serial_port = port; self.rx_buffer.clear(); self.discard_until_newline = True
        self.bridge_report = None; self.bridge_report_at = 0.0
        # Each connection creates a new set of log files, so its live track
        # must not be connected to coordinates retained from an earlier run.
        self.clear_data()
        self._start_serial_worker(port)
        self.port_combo.setEnabled(False); self.baud_combo.setEnabled(False)
        self._set_log_controls_enabled(False); self.pause_button.setEnabled(True); self.connect_button.setText("断开")
        self.connection_label.setText(f"● 已连接 {device}"); self.connection_label.setStyleSheet("color:#5bd18b;font-weight:bold")
        self.log_event("串口", f"已连接 {device} / {self.baud_combo.currentText()} bit/s")
        self.statusBar().showMessage(f"正在接收 {device}；协议 IMU/GNSS/SAT/RAWX，460800 bit/s。")

    def disconnect_serial(self, reason: str) -> None:
        was_connected = self.serial_port is not None or self.serial_worker is not None
        if was_connected and "异常" in reason: self._log_link_debug(reason)
        self.stop_ntrip("基站: 已停止")
        if self.serial_worker is not None:
            worker = self.serial_worker
            worker.stop()
            if not worker.wait(1000):
                self.statusBar().showMessage("正在等待串口线程安全退出…")
                QtCore.QTimer.singleShot(100, lambda: self.disconnect_serial(reason))
                return
            # Drain already-received data before closing CSV files.
            while not worker.received.empty(): self.poll_serial()
            self.serial_worker = None
            worker.deleteLater()
        elif self.serial_port is not None:
            try: self.serial_port.close()
            except serial.SerialException: pass
        if was_connected: self.log_event("串口", reason + "；正在关闭数据文件。")
        self.serial_port = None; self._close_logs(); self.port_combo.setEnabled(True); self.baud_combo.setEnabled(True)
        self._set_log_controls_enabled(True); self.pause_button.setEnabled(False); self.connect_button.setText("连接")
        self.connection_label.setText("● 未连接"); self.connection_label.setStyleSheet("color:#ff6174;font-weight:bold")
        self.statusBar().showMessage(reason)

    def _start_serial_worker(self, port):
        self.serial_worker = SerialWorker(port, self)
        worker = self.serial_worker
        worker.diagnostic.connect(lambda message, source=worker:
                                  self.log_event("串口即时调试", message)
                                  if source is self.serial_worker else None)
        worker.failed.connect(lambda message, source=worker:
                              self.disconnect_serial(f"串口异常：{message}")
                              if source is self.serial_worker else None)
        worker.rtcm_failed.connect(lambda source, message:
                                   self.stop_ntrip(f"基站: {message}")
                                   if source is self.ntrip_client else None)
        worker.start()

    def configure_ntrip(self) -> bool:
        dialog = NtripSettingsDialog(self.settings, self)
        dialog.pass_edit.setText(self.ntrip_password)
        if dialog.exec_() != QtWidgets.QDialog.Accepted:
            return False
        self.ntrip_password = dialog.pass_edit.text()
        return True

    def toggle_ntrip(self) -> None:
        if self.ntrip_client is not None:
            self.stop_ntrip("基站: 已停止，IMU/GNSS 采集继续")
            return
        if self.serial_port is None or self.serial_worker is None:
            QtWidgets.QMessageBox.warning(self, "先连接采集串口", "请先连接 STM32 对应的 COM7。")
            return
        if self.bridge_report is None or time.monotonic() - self.bridge_report_at > 1:
            QtWidgets.QMessageBox.warning(self, "缺少转发状态", "请烧录支持 RTCM 的新固件，等待转发状态出现。")
            return
        if not self.settings.value("ntrip_user", "") and not self.configure_ntrip():
            return
        try:
            # Recheck freshness after the modal settings dialog.
            if time.monotonic() - self.bridge_report_at > 1:
                raise ValueError("STM32 状态已过期，请等待串口恢复")
            BridgeFlow(self.bridge_report)  # preliminary UI validation; worker rechecks live ACK
            client = NtripClient(str(self.settings.value("ntrip_host", NTRIP_DEFAULT_HOST)),
                                 int(self.settings.value("ntrip_port", NTRIP_DEFAULT_PORT)),
                                 str(self.settings.value("ntrip_mount", NTRIP_DEFAULT_MOUNT)),
                                 str(self.settings.value("ntrip_user", "")), self.ntrip_password, self)
        except (ValueError, OSError) as error:
            self.log_event("基站错误", f"无法启动基站：{error}")
            QtWidgets.QMessageBox.warning(self, "无法启动基站", str(error)); return
        self.ntrip_client = client; self.ntrip_workers.append(client)
        self.serial_worker.set_source(client)
        client.status.connect(lambda message, source=client:
                              self._set_ntrip_status(message) if source is self.ntrip_client else None)
        client.diagnostic.connect(lambda message, source=client:
                                  self._log_link_debug(message) if source is self.ntrip_client else None)
        client.finished.connect(lambda source=client: self._ntrip_finished(source))
        self.ntrip_button.setText("停止基站"); self.ntrip_settings_button.setEnabled(False)
        self._ntrip_interrupted_at = None
        self._set_ntrip_status("基站: 正在直连 NTRIP…")
        client.start()

    def _ntrip_finished(self, client) -> None:
        if client in self.ntrip_workers: self.ntrip_workers.remove(client)
        if self.ntrip_client is client: self.stop_ntrip("基站: 网络线程已退出")
        client.deleteLater()

    def stop_ntrip(self, reason: str) -> None:
        was_active = self.ntrip_client is not None
        if was_active: self._log_link_debug(reason)
        if self.serial_worker is not None: self.serial_worker.set_source(None)
        if self.ntrip_client is not None: self.ntrip_client.stop()
        self.ntrip_client = None
        self.ntrip_button.setText("连接基站"); self.ntrip_settings_button.setEnabled(True)
        self.ntrip_label.setText(reason)
        if was_active: self.log_event("基站", reason)
        self._ntrip_interrupted_at = None

    def _process_rtcm_status(self, line: str) -> None:
        try:
            report = [int(value) for value in line.split(",")[1:]]
            if len(report) != 13 or any(value < 0 or value > 0xFFFFFFFF for value in report):
                raise ValueError("Invalid RTCM status")
        except ValueError:
            self.invalid_lines += 1; return
        self.bridge_report = report; self.bridge_report_at = time.monotonic()
        # Credit is handled by the serial thread immediately on reception,
        # independently of GUI plots, dialogs and CSV processing.

    def poll_serial(self) -> None:
        if self.serial_worker is None: return
        self.rx_buffer.extend(self.serial_worker.take_received())
        if self.discard_until_newline:
            newline = self.rx_buffer.find(b"\n")
            if newline < 0:
                return
            del self.rx_buffer[:newline + 1]
            self.discard_until_newline = False
        while True:
            newline = self.rx_buffer.find(b"\n")
            if newline < 0: break
            raw = bytes(self.rx_buffer[:newline]).strip(); del self.rx_buffer[:newline + 1]
            if raw: self.process_line(raw)
        if len(self.rx_buffer) > 1024 * 1024:
            self.rx_buffer.clear(); self.invalid_lines += 1

    def process_line(self, raw: bytes) -> None:
        try: line = raw.decode("ascii")
        except UnicodeDecodeError:
            self.invalid_lines += 1; return
        if line.startswith("#RTCM,"):
            self._process_rtcm_status(line); return
        if line.startswith("#"):
            if line.startswith("# booting"): self.log_event("设备", "收到 STM32 启动消息。")
            if line.startswith("# booting") and self.ntrip_client is not None:
                self.stop_ntrip("基站: STM32 重启，请等待就绪后重新连接")
            return
        parts = line.split(",")
        try:
            if parts[0] == "IMU" and len(parts) == 13:
                self._process_imu([int(v) for v in parts[1:]])
            elif parts[0] == "GNSS" and len(parts) == 21:
                self._process_gnss([int(v) for v in parts[1:]])
            elif parts[0] == "SAT" and len(parts) == 10:
                self._process_sat([int(v) for v in parts[1:]])
            elif parts[0] == "SAT_END" and len(parts) == 5:
                self._process_sat_end([int(v) for v in parts[1:]])
            elif parts[0] == "RAWX" and len(parts) == 8:
                self._process_rawx_header(parts)
            elif parts[0] == "RAWX_MEAS" and len(parts) == 14:
                self._process_rawx_measurement(parts)
            elif parts[0] == "RAWX_END" and len(parts) == 2:
                self.rawx_epoch = None
            else:
                self.invalid_lines += 1
        except (ValueError, IndexError):
            self.invalid_lines += 1

    def _process_imu(self, values: list[int]) -> None:
        sample, week, tow_us, valid, timer_us, ax, ay, az, temp, gx, gy, gz = values
        if self.last_sample is not None:
            delta = sample - self.last_sample
            if delta > 1: self.lost_imu += delta - 1
            elif delta <= 0: self.invalid_lines += 1
        self.last_sample = sample
        if self.first_timer_us is None: self.first_timer_us = timer_us
        elapsed = (timer_us - self.first_timer_us) / 1e6
        self.last_dt_ms = 0.0 if self.last_timer_us is None else (timer_us - self.last_timer_us) / 1000.0
        self.last_timer_us = timer_us
        physical = (ax * ACCEL_SCALE, ay * ACCEL_SCALE, az * ACCEL_SCALE,
                    temp / 340.0 + 36.53, gx * GYRO_DEG_H_SCALE,
                    gy * GYRO_DEG_H_SCALE, gz * GYRO_DEG_H_SCALE)
        for name, value in zip(("ax", "ay", "az", "temp", "gx", "gy", "gz"), physical):
            self.imu[name].append(float(value))
        self.imu["time"].append(elapsed); self.total_imu += 1
        arrival = time.monotonic(); self.arrivals.append(arrival)
        while self.arrivals and arrival - self.arrivals[0] > 2.0: self.arrivals.popleft()
        if self.imu_writer is not None:
            self.imu_writer.writerow([sample, week, tow_us, valid, timer_us, elapsed,
                                      self.last_dt_ms / 1000.0, ax, ay, az, temp, gx, gy, gz, *physical])
            self._periodic_flush()
        self._trim_buffers()

    def _process_gnss(self, values: list[int]) -> None:
        (week, tow_ms, valid, rx_timer_us, fix, num_sv, flags, flags2, carr_soln,
         lat_e7, lon_e7, hmsl_mm, h_acc_mm, v_acc_mm, vn, ve, vd, ground,
         s_acc_mms, pdop) = values
        gnss_fix_ok = flags & 0x01
        diff_soln = (flags >> 1) & 0x01
        self.gnss_diagnostics = {"tow_ms": tow_ms, "flags": flags, "carr_soln": carr_soln,
                                 "diff_soln": diff_soln, "hacc_mm": h_acc_mm}
        h_acc_m = h_acc_mm / 1000.0
        v_acc_m = v_acc_mm / 1000.0
        s_acc_m_s = s_acc_mms / 1000.0
        lat, lon, height = lat_e7 / 1e7, lon_e7 / 1e7, hmsl_mm / 1000.0
        velocities = (vn / 1000.0, ve / 1000.0, vd / 1000.0, ground / 1000.0)
        absolute = week * GPS_WEEK_SECONDS + tow_ms / 1000.0
        if valid:
            if self.first_gnss_time is None: self.first_gnss_time = absolute
            elapsed = absolute - self.first_gnss_time
        else:
            elapsed = self.speed["time"][-1] + 1.0 if self.speed["time"] else 0.0
        self.speed["time"].append(elapsed)
        for name, value in zip(("vn", "ve", "vd", "ground"), velocities): self.speed[name].append(value)
        self.total_gnss += 1
        self.time_label.setText(f"GPS时间: W{week} {tow_ms / 1000.0:.3f}s" if valid else "GPS时间: 无效")
        fix_names = {0: "无", 1: "航位推算", 2: "2D", 3: "3D", 4: "GNSS+DR", 5: "仅时间"}
        carrier_names = {1: "RTK浮点", 2: "RTK固定"}
        carrier = carrier_names.get(carr_soln, "")
        quality = f" / {carrier}" if carrier else ""
        if gnss_fix_ok == 0: quality += " / 解无效"
        self.fix_label.setText(f"定位: {fix_names.get(fix, str(fix))}{quality}")
        state = (fix, carr_soln, gnss_fix_ok, diff_soln)
        if state != self._logged_fix:
            self.log_event("定位", f"W{week} {tow_ms / 1000:.3f}s · "
                           f"{self.fix_label.text()} · 差分 {diff_soln} · hAcc {h_acc_m:.3f} m")
            self._logged_fix = state
        self.sv_label.setText(f"卫星数: {num_sv}"); self.pdop_label.setText(f"PDOP: {pdop / 100.0:.2f}")
        self.lat_value.setText(f"{lat:.9f}°"); self.lon_value.setText(f"{lon:.9f}°")
        self.height_value.setText(f"{height:.3f} m"); self.speed_value.setText(f"{velocities[3]:.3f} m/s")
        self.velocity_value.setText(f"N {velocities[0]:.3f}  E {velocities[1]:.3f}  D {velocities[2]:.3f} m/s")
        if (valid and fix >= 2 and gnss_fix_ok != 0 and
                abs(lat) <= 90 and abs(lon) <= 180):
            self.map_widget.set_position(lat, lon)
        if self.gnss_writer is not None:
            self.gnss_writer.writerow([week, tow_ms, valid,
                                       rx_timer_us, fix, num_sv, flags, flags2,
                                       carr_soln, gnss_fix_ok, diff_soln,
                                       lat, lon, height, h_acc_m, v_acc_m,
                                       velocities[0], velocities[1], velocities[2], velocities[3],
                                       s_acc_m_s, pdop / 100.0])
            self._periodic_flush()
        self._trim_buffers()

    def _process_sat(self, values: list[int]) -> None:
        week, tow_ms, _valid, gnss, sv, cno, elev, azim, used = values
        epoch = week, tow_ms
        if self.satellite_epoch != epoch:
            self.satellite_epoch = epoch; self.pending_satellites = []
        self.pending_satellites.append({"gnss": gnss, "sv": sv, "cno": cno,
                                        "elev": elev, "azim": azim, "used": used})

    def _process_sat_end(self, values: list[int]) -> None:
        week, tow_ms, _valid, _count = values
        if self.satellite_epoch == (week, tow_ms):
            self.sky_plot.set_satellites(list(self.pending_satellites))

    def _process_rawx_header(self, parts: list[str]) -> None:
        self.rawx_epoch = {
            "gps_week": int(parts[1]), "rcv_tow_s": float_from_hex(parts[2], 8),
            "leap_s": int(parts[3]), "rec_stat": int(parts[4]),
            "num_meas": int(parts[5]), "total_meas": int(parts[6]),
            "rx_timer_us": int(parts[7]),
        }
        self.rawx_seen_header = True

    def _process_rawx_measurement(self, parts: list[str]) -> None:
        if self.rawx_epoch is None:
            # Opening an already-running serial stream often starts in the
            # middle of a RAWX epoch. Ignore that startup fragment until the
            # next header instead of reporting every measurement as invalid.
            if not self.rawx_seen_header:
                return
            raise ValueError("RAWX_MEAS without RAWX header")
        gnss_id, sv_id, sig_id, freq_id = (int(v) for v in parts[1:5])
        pr = float_from_hex(parts[5], 8); cp = float_from_hex(parts[6], 8)
        doppler = float_from_hex(parts[7], 4)
        lock_ms, cno, pr_std, cp_std, do_std, trk = (int(v) for v in parts[8:])
        signal, frequency = signal_name_frequency(gnss_id, sig_id, freq_id)
        if self.rawx_writer is not None:
            epoch = self.rawx_epoch
            cp_sigma: float | str = "" if cp_std == 15 else cp_std * 0.004
            self.rawx_writer.writerow([
                epoch["gps_week"], epoch["rcv_tow_s"], epoch["rx_timer_us"],
                epoch["leap_s"], epoch["rec_stat"], epoch["num_meas"],
                epoch["total_meas"], gnss_id, sv_id, sig_id, freq_id, signal,
                frequency, pr, cp, doppler, lock_ms, cno, 0.01 * (2 ** pr_std),
                cp_sigma, 0.002 * (2 ** do_std), trk & 1, (trk >> 1) & 1,
                (trk >> 2) & 1, (trk >> 3) & 1,
            ])
            self._periodic_flush()
        self.total_rawx += 1

    def _periodic_flush(self) -> None:
        self.rows_since_flush += 1
        if self.rows_since_flush >= 100:
            if self.imu_stream is not None: self.imu_stream.flush()
            if self.gnss_stream is not None: self.gnss_stream.flush()
            if self.rawx_stream is not None: self.rawx_stream.flush()
            self.rows_since_flush = 0

    def _trim_buffers(self) -> None:
        imu_max = int(self.window_spin.value() * 130); gnss_max = int(self.window_spin.value() * 2 + 10)
        for channel in self.imu.values():
            while len(channel) > imu_max: channel.popleft()
        for channel in self.speed.values():
            while len(channel) > gnss_max: channel.popleft()

    def update_plots(self) -> None:
        if self.plot_paused: return
        if len(self.imu["time"]) >= 2:
            x = np.fromiter(self.imu["time"], dtype=np.float64); x -= x[-1]
            for name, curve in self.accel_curves.items():
                curve.setData(x, np.fromiter(self.imu[name], dtype=np.float64))
            for name, curve in self.gyro_curves.items():
                curve.setData(x, np.fromiter(self.imu[name], dtype=np.float64))
            self.temp_curve.setData(x, np.fromiter(self.imu["temp"], dtype=np.float64))
            self.accel_plot.setXRange(-float(self.window_spin.value()), 0.0, padding=0.0)
        if len(self.speed["time"]) >= 2:
            x = np.fromiter(self.speed["time"], dtype=np.float64); x -= x[-1]
            for name, curve in self.speed_curves.items():
                curve.setData(x, np.fromiter(self.speed[name], dtype=np.float64))
            self.speed_plot.setXRange(-float(self.window_spin.value()), 0.0, padding=0.0)

    def update_stats(self) -> None:
        errors = (self.lost_imu, self.invalid_lines)
        if errors != self._logged_errors:
            if any(a > b for a, b in zip(errors, self._logged_errors)):
                self.log_event("采集警告", f"累计 IMU 丢帧 {errors[0]}；无效行 {errors[1]}（本次统计窗口合并报告）")
            self._logged_errors = errors
        rate = 0.0
        if len(self.arrivals) >= 2:
            span = self.arrivals[-1] - self.arrivals[0]
            if span > 0: rate = (len(self.arrivals) - 1) / span
        self.rate_label.setText(f"IMU: {rate:.2f} Hz · dt {self.last_dt_ms:.3f} ms" if rate else "IMU: -- Hz")
        self.frames_label.setText(f"IMU/GNSS/RAWX: {self.total_imu:,} / {self.total_gnss:,} / {self.total_rawx:,}")
        self.loss_label.setText(f"丢帧: {self.lost_imu:,} · 无效行: {self.invalid_lines:,}")
        if self.bridge_report is not None:
            r = self.bridge_report
            ready = "就绪" if r[1] else "初始化中"
            if time.monotonic() - self.bridge_report_at > 2: ready = "状态超时"
            network = ""
            if self.ntrip_client is not None:
                c = self.ntrip_client
                queued, peak, age = c.frames.snapshot()
                network = (f"网络 {c.network_bytes:,} B/{c.frame_count} 帧 CRC错 {c.crc_errors} "
                           f"重连 {c.reconnects} 排队 {queued} B/{age:.2f}s · ")
                self.rtcm_label.setToolTip(
                    f"STM32/F9P 计数自启动累计；关注本次增量。队列峰值 {peak} B。\n"
                    f"F9P MSM兼容：移除 NavIC {c.msm_adapter.filtered} 帧，"
                    f"修正历元结束标志 {c.msm_adapter.rewritten} 次；观测数值不改。\n"
                    "距接收是状态消息到达间隔，不是观测历元差分龄期。")
            age = "--" if r[12] == 0xFFFFFFFF else f"{r[12] / 1000:.1f}s"
            self.rtcm_label.setText(
                f"RTCM {ready} · {network}STM32 收/转发 {r[2]:,}/{r[3]:,} B · "
                f"丢字节/串口错 {r[4]}/{r[5]} · F9P 收/使用 {r[7]}/{r[8]} 帧 "
                f"CRC错 {r[9]} · 站号 {r[10]} 消息 {r[11]} · 距接收 {age} · GNSS溢出 {r[6]}")

    def toggle_pause(self) -> None:
        self.plot_paused = not self.plot_paused
        self.pause_button.setText("继续绘图" if self.plot_paused else "暂停绘图")
        self.statusBar().showMessage("绘图暂停，串口接收和文件保存仍继续。" if self.plot_paused else "绘图已继续。")

    def clear_data(self) -> None:
        self._logged_fix = None
        self._logged_errors = (0, 0)
        for channel in (*self.imu.values(), *self.speed.values()): channel.clear()
        self.last_sample = self.first_timer_us = self.last_timer_us = None
        self.first_gnss_time = None
        self.total_imu = self.total_gnss = self.total_rawx = self.lost_imu = self.invalid_lines = 0
        self.last_dt_ms = 0.0; self.arrivals.clear(); self.pending_satellites = []; self.satellite_epoch = None
        self.rawx_epoch = None; self.rawx_seen_header = False
        self.map_widget.clear_track(); self.sky_plot.set_satellites([])
        for curve in (*self.accel_curves.values(), *self.gyro_curves.values(), self.temp_curve,
                      *self.speed_curves.values()): curve.clear()
        self.update_stats()

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:  # noqa: N802
        self.disconnect_serial("应用已关闭。")
        if self.serial_worker is not None or any(worker.isRunning() for worker in self.ntrip_workers):
            for worker in self.ntrip_workers: worker.stop()
            event.ignore()
            QtCore.QTimer.singleShot(100, self.close)
        else: event.accept()


def main() -> None:
    QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling, True)
    QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_UseHighDpiPixmaps, True)
    QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_ShareOpenGLContexts, True)
    app = QtWidgets.QApplication(sys.argv); app.setStyle("Fusion")
    window = NavigationMonitor(); window.show()
    raise SystemExit(app.exec_())


if __name__ == "__main__":
    main()
