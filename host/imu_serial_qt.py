"""Real-time synchronized MPU6050/ZED-F9P navigation data monitor."""

from __future__ import annotations

import csv
import json
import math
import os
import pathlib
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
        self.map_status = QtWidgets.QLabel("当前：本地轨迹")
        self.load_button.clicked.connect(self.load_amap)
        self.local_button.clicked.connect(self.show_local_map)
        bar.addWidget(self.key_edit, 2); bar.addWidget(self.security_edit, 2)
        bar.addWidget(self.load_button); bar.addWidget(self.local_button)
        bar.addWidget(self.map_status); layout.addLayout(bar)
        self.stack = QtWidgets.QStackedWidget()
        self.local_plot = pg.PlotWidget(); self.local_plot.setBackground("#101418")
        self.local_plot.showGrid(x=True, y=True, alpha=0.3); self.local_plot.setAspectLocked(True)
        self.local_plot.setLabel("left", "北向", units="m"); self.local_plot.setLabel("bottom", "东向", units="m")
        self.local_plot.setTitle("WGS-84 本地轨迹（高德 Key 未加载时使用）")
        self.track_curve = self.local_plot.plot(pen=pg.mkPen("#45a3ff", width=2))
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
        self.rx_buffer = bytearray(); self.discard_until_newline = False
        self.plot_paused = False
        self.imu_stream = self.gnss_stream = None
        self.imu_writer: csv.writer | None = None
        self.gnss_writer: csv.writer | None = None
        self.rows_since_flush = 0
        self.imu = {name: deque() for name in ("time", "ax", "ay", "az", "gx", "gy", "gz", "temp")}
        self.speed = {name: deque() for name in ("time", "vn", "ve", "vd", "ground")}
        self.last_sample: int | None = None
        self.first_timer_us: int | None = None
        self.last_timer_us: int | None = None
        self.first_gnss_time: float | None = None
        self.total_imu = self.total_gnss = self.lost_imu = self.invalid_lines = 0
        self.last_dt_ms = 0.0; self.arrivals: deque[float] = deque()
        self.satellite_epoch: tuple[int, int] | None = None
        self.pending_satellites: list[dict[str, int]] = []
        self._build_ui(); self._build_timers(); self.refresh_ports()

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
        self.save_checkbox = QtWidgets.QCheckBox("分别保存 IMU/GNSS CSV"); self.save_checkbox.setChecked(True)
        self.connection_label = QtWidgets.QLabel("● 未连接"); self.connection_label.setStyleSheet("color:#ff6174;font-weight:bold")
        for text, widget in (("串口", self.port_combo), ("波特率", self.baud_combo), ("窗口", self.window_spin)):
            controls.addWidget(QtWidgets.QLabel(text)); controls.addWidget(widget)
        for widget in (self.refresh_button, self.connect_button, self.pause_button, self.clear_button, self.save_checkbox):
            controls.addWidget(widget)
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
        self.tabs = QtWidgets.QTabWidget(); self.tabs.addTab(self._build_navigation_tab(), "导航")
        self.tabs.addTab(self._build_imu_tab(), "IMU"); outer.addWidget(self.tabs, 1)
        self.setCentralWidget(central)
        self.statusBar().showMessage("选择 PA9 对应的 USB-TTL 串口，默认 460800 bit/s。")

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

    def _open_logs(self) -> bool:
        if not self.save_checkbox.isChecked(): return True
        DEFAULT_DATA_DIR.mkdir(parents=True, exist_ok=True)
        parent = QtWidgets.QFileDialog.getExistingDirectory(self, "选择数据保存目录", str(DEFAULT_DATA_DIR))
        if not parent: return False
        stamp = time.strftime("%Y%m%d_%H%M%S")
        try:
            imu_path = pathlib.Path(parent) / f"imu_gnss_time_{stamp}.csv"
            gnss_path = pathlib.Path(parent) / f"gnss_nav_{stamp}.csv"
            self.imu_stream = imu_path.open("w", newline="", encoding="utf-8")
            self.gnss_stream = gnss_path.open("w", newline="", encoding="utf-8")
            self.imu_writer = csv.writer(self.imu_stream); self.gnss_writer = csv.writer(self.gnss_stream)
            self.imu_writer.writerow(["sample", "gps_week", "gps_tow_us", "time_valid", "timer_us", "time_s", "dt_s",
                "ax_raw", "ay_raw", "az_raw", "temp_raw", "gx_raw", "gy_raw", "gz_raw",
                "ax_m_s2", "ay_m_s2", "az_m_s2", "temp_deg_c", "gx_deg_h", "gy_deg_h", "gz_deg_h"])
            self.gnss_writer.writerow(["gps_week", "gps_tow_ms", "time_valid", "rx_timer_us", "fix", "num_sv",
                "flags", "flags2", "carr_soln", "gnss_fix_ok", "diff_soln", "lat_deg", "lon_deg",
                "hmsl_m", "h_acc_m", "v_acc_m", "vel_n_m_s", "vel_e_m_s", "vel_d_m_s",
                "ground_speed_m_s", "s_acc_m_s", "pdop"])
            self.statusBar().showMessage(f"保存到 {imu_path.name} 和 {gnss_path.name}"); return True
        except OSError as error:
            self._close_logs(); QtWidgets.QMessageBox.critical(self, "文件错误", f"无法创建数据文件：\n{error}"); return False

    def _close_logs(self) -> None:
        for stream in (self.imu_stream, self.gnss_stream):
            if stream is not None: stream.flush(); stream.close()
        self.imu_stream = self.gnss_stream = None; self.imu_writer = self.gnss_writer = None; self.rows_since_flush = 0

    def connect_serial(self) -> None:
        device = self.port_combo.currentData()
        if not device:
            QtWidgets.QMessageBox.warning(self, "没有串口", "没有发现可用串口。"); return
        if not self._open_logs(): return
        try:
            port = serial.Serial(device, int(self.baud_combo.currentText()), timeout=0, write_timeout=0)
            port.dtr = False; port.rts = False; port.reset_input_buffer()
        except (serial.SerialException, OSError) as error:
            self._close_logs(); QtWidgets.QMessageBox.critical(self, "串口连接失败", f"无法打开 {device}：\n{error}"); return
        self.serial_port = port; self.rx_buffer.clear(); self.discard_until_newline = True
        self.port_combo.setEnabled(False); self.baud_combo.setEnabled(False)
        self.save_checkbox.setEnabled(False); self.pause_button.setEnabled(True); self.connect_button.setText("断开")
        self.connection_label.setText(f"● 已连接 {device}"); self.connection_label.setStyleSheet("color:#5bd18b;font-weight:bold")
        self.statusBar().showMessage(f"正在接收 {device}；协议 IMU/GNSS/SAT，460800 bit/s。")

    def disconnect_serial(self, reason: str) -> None:
        if self.serial_port is not None:
            try: self.serial_port.close()
            except serial.SerialException: pass
        self.serial_port = None; self._close_logs(); self.port_combo.setEnabled(True); self.baud_combo.setEnabled(True)
        self.save_checkbox.setEnabled(True); self.pause_button.setEnabled(False); self.connect_button.setText("连接")
        self.connection_label.setText("● 未连接"); self.connection_label.setStyleSheet("color:#ff6174;font-weight:bold")
        self.statusBar().showMessage(reason)

    def poll_serial(self) -> None:
        if self.serial_port is None: return
        try:
            waiting = self.serial_port.in_waiting
            if waiting: self.rx_buffer.extend(self.serial_port.read(waiting))
        except (serial.SerialException, OSError) as error:
            self.disconnect_serial(f"串口异常断开：{error}"); return
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
        if line.startswith("#"): return
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

    def _periodic_flush(self) -> None:
        self.rows_since_flush += 1
        if self.rows_since_flush >= 100:
            if self.imu_stream is not None: self.imu_stream.flush()
            if self.gnss_stream is not None: self.gnss_stream.flush()
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
        rate = 0.0
        if len(self.arrivals) >= 2:
            span = self.arrivals[-1] - self.arrivals[0]
            if span > 0: rate = (len(self.arrivals) - 1) / span
        self.rate_label.setText(f"IMU: {rate:.2f} Hz · dt {self.last_dt_ms:.3f} ms" if rate else "IMU: -- Hz")
        self.frames_label.setText(f"IMU/GNSS: {self.total_imu:,} / {self.total_gnss:,}")
        self.loss_label.setText(f"丢帧: {self.lost_imu:,} · 无效行: {self.invalid_lines:,}")

    def toggle_pause(self) -> None:
        self.plot_paused = not self.plot_paused
        self.pause_button.setText("继续绘图" if self.plot_paused else "暂停绘图")
        self.statusBar().showMessage("绘图暂停，串口接收和文件保存仍继续。" if self.plot_paused else "绘图已继续。")

    def clear_data(self) -> None:
        for channel in (*self.imu.values(), *self.speed.values()): channel.clear()
        self.last_sample = self.first_timer_us = self.last_timer_us = None
        self.first_gnss_time = None; self.total_imu = self.total_gnss = self.lost_imu = self.invalid_lines = 0
        self.last_dt_ms = 0.0; self.arrivals.clear(); self.pending_satellites = []; self.satellite_epoch = None
        self.map_widget.clear_track(); self.sky_plot.set_satellites([])
        for curve in (*self.accel_curves.values(), *self.gyro_curves.values(), self.temp_curve,
                      *self.speed_curves.values()): curve.clear()
        self.update_stats()

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:  # noqa: N802
        self.disconnect_serial("应用已关闭。"); event.accept()


def main() -> None:
    QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling, True)
    QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_UseHighDpiPixmaps, True)
    QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_ShareOpenGLContexts, True)
    app = QtWidgets.QApplication(sys.argv); app.setStyle("Fusion")
    window = NavigationMonitor(); window.show()
    raise SystemExit(app.exec_())


if __name__ == "__main__":
    main()
