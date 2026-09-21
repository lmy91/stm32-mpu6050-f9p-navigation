"""Real-time synchronized MPU6050/ZED-F9P navigation data monitor."""

from __future__ import annotations

import csv
import json
import math
import os
import pathlib
import queue
import struct
import sys
import time
import urllib.parse
from collections import deque
from dataclasses import asdict

SOURCE_ROOT = pathlib.Path(__file__).resolve().parent.parent
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))

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
from fusion import (GnssObservation, ImuSample, NavigationInitialState,
                    RealtimeLooseNavigation, RealtimeSelfAim, SelfAimConfig)

try:
    from PyQt5 import QtWebEngineWidgets
except ImportError:
    QtWebEngineWidgets = None

G0 = 9.80665
ACCEL_SCALE = G0 / 16384.0
GYRO_DEG_H_SCALE = 3600.0 / 131.0
GPS_WEEK_SECONDS = 604800.0
PROJECT_ROOT = pathlib.Path(sys.executable).resolve().parent if getattr(sys, "frozen", False) else SOURCE_ROOT
RESOURCE_ROOT = pathlib.Path(getattr(sys, "_MEIPASS", SOURCE_ROOT))
DEFAULT_DATA_DIR = PROJECT_ROOT / "data" / "decoded"
DEFAULT_AIM_CONFIG = RESOURCE_ROOT / "fusion" / "self_aim_config.json"
LOG_LEVEL_VALUES = {"DEBUG": 10, "INFO": 20, "WARN": 30, "ERROR": 40}
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
# Observability counters emitted once per PPS in the "# sync" line. The four
# "d_" columns are unsigned-32 deltas vs. the previous # sync (first row = 0);
# backlog = interrupt_count - sample_count.
SYNC_COLUMNS = [
    "unix_ms", "pps",
    "sample_count", "interrupt_count", "interrupt_overruns",
    "cc2_overcapture", "dt_gap_count", "i2c_errors",
    "d_interrupt_overruns", "d_cc2_overcapture", "d_dt_gap_count", "d_i2c_errors",
    "backlog",
]
SYNC_COUNTERS = ("sample_count", "interrupt_count", "interrupt_overruns",
                 "cc2_overcapture", "dt_gap_count", "i2c_errors")


def parse_sync_line(line: str) -> dict[str, int] | None:
    """Parse one "# sync,key=value,..." diagnostic line into its counters."""
    if not line.startswith("# sync,"):
        return None
    fields: dict[str, str] = {}
    for item in line[len("# sync,"):].split(","):
        if "=" in item:
            key, value = item.split("=", 1)
            fields[key] = value
    try:
        return {name: int(fields[name]) for name in SYNC_COUNTERS} | {
            "pps": int(fields.get("pps", "0"))}
    except (KeyError, ValueError):
        return None


def u32_delta(current: int, previous: int) -> int:
    """Unsigned 32-bit difference to survive firmware counter wrap."""
    return (current - previous) & 0xFFFFFFFF


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
        self.auto_fit_track = True
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
        self.fit_button.setToolTip(
            "显示全部轨迹并恢复自动适应；鼠标滚轮缩放或左键拖动后保持人工查看范围")
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
        self.local_plot.getViewBox().sigRangeChangedManually.connect(
            self._manual_track_view)
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
        self._update_local_status()

    def _update_local_status(self) -> None:
        suffix = "（自动适应）" if self.auto_fit_track else "（局部查看）"
        self.map_status.setText("当前：本地轨迹" + suffix)

    def _manual_track_view(self, _axes=None) -> None:
        """Keep a mouse-selected local range until the user requests a fit."""
        self.auto_fit_track = False
        if self.stack.currentWidget() is self.local_plot:
            self._update_local_status()

    def fit_track(self) -> None:
        """Restore an equal-scale view fitted to all collected positions."""
        self.auto_fit_track = True
        self.show_local_map()
        self._fit_local_track()
        self._update_local_status()

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
        self.auto_fit_track = True
        self._fit_local_track()
        if self.stack.currentWidget() is self.local_plot:
            self._update_local_status()
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
        if self.auto_fit_track and self._track_needs_fit():
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


class SelfAimConfigDialog(QtWidgets.QDialog):
    """Edit every SelfAimConfig field without hand-editing JSON."""

    VECTOR_FIELDS = (
        "lever_arm_body_m", "initial_attitude_std_deg", "initial_velocity_std_m_s",
        "initial_position_std_m", "initial_gyro_bias_std_deg_h",
        "initial_accel_bias_std_mg", "default_velocity_measurement_std_m_s",
        "default_position_measurement_std_m",
        "gyro_arw_deg_sqrt_h", "accel_vrw_m_s_sqrt_h",
        "gyro_bias_sigma_deg_h", "accel_bias_sigma_mg",
        "gyro_bias_correlation_time_s", "accel_bias_correlation_time_s")

    def __init__(self, config: SelfAimConfig, path: pathlib.Path,
                 parent: QtWidgets.QWidget | None = None) -> None:
        super().__init__(parent)
        self.setWindowTitle("自瞄算法配置"); self.setModal(True); self.resize(760, 650)
        self.config_path = pathlib.Path(path); self.result_config = config
        outer = QtWidgets.QVBoxLayout(self)
        file_row = QtWidgets.QHBoxLayout()
        self.path_label = QtWidgets.QLabel(str(self.config_path)); self.path_label.setTextInteractionFlags(
            QtCore.Qt.TextSelectableByMouse); self.path_label.setToolTip(str(self.config_path))
        for title, callback in (("从文件导入…", self.import_config),
                                ("保存到文件…", self.save_config),
                                ("恢复默认值", self.restore_defaults)):
            button = QtWidgets.QPushButton(title); button.clicked.connect(callback); file_row.addWidget(button)
        file_row.addWidget(self.path_label, 1); outer.addLayout(file_row)
        self.tabs = QtWidgets.QTabWidget(); outer.addWidget(self.tabs, 1)
        self.scalar_widgets = {}; self.vector_widgets = {}; self.bool_widgets = {}
        self._build_timing_tab(); self._build_initial_p_tab(); self._build_process_q_tab()
        self._build_measurement_r_tab(); self._build_quality_tab(); self._build_installation_tab()
        note = QtWidgets.QLabel("点击“应用”只更新本次运行参数；需要长期保留时请先使用“保存到文件…”。")
        note.setStyleSheet("color:#666"); outer.addWidget(note)
        buttons = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)
        buttons.button(QtWidgets.QDialogButtonBox.Ok).setText("应用")
        buttons.accepted.connect(self.validate_and_accept); buttons.rejected.connect(self.reject)
        outer.addWidget(buttons); self.set_config(config)

    @staticmethod
    def _form_page() -> tuple[QtWidgets.QWidget, QtWidgets.QFormLayout]:
        page = QtWidgets.QWidget(); form = QtWidgets.QFormLayout(page)
        form.setFieldGrowthPolicy(QtWidgets.QFormLayout.AllNonFixedFieldsGrow)
        return page, form

    def _double(self, form, key, label, unit="", maximum=1e9, decimals=4, tip=""):
        widget = QtWidgets.QDoubleSpinBox(); widget.setRange(0.0, maximum)
        widget.setDecimals(decimals); widget.setSuffix(f" {unit}" if unit else "")
        widget.setKeyboardTracking(False); widget.setToolTip(tip)
        form.addRow(label, widget); self.scalar_widgets[key] = widget

    def _vector(self, form, key, label, unit, tip):
        widget = QtWidgets.QLineEdit(); widget.setPlaceholderText("x, y, z")
        widget.setToolTip(tip); form.addRow(f"{label} [{unit}]", widget)
        self.vector_widgets[key] = widget

    def _build_timing_tab(self):
        page, form = self._form_page(); self.tabs.addTab(page, "对准设置")
        self._double(form, "coarse_alignment_seconds", "粗对准时长", "s", 86400, 1)
        samples = QtWidgets.QSpinBox(); samples.setRange(2, 100000000)
        form.addRow("粗对准最少IMU样本", samples); self.scalar_widgets["minimum_imu_samples"] = samples
        self.initial_heading = QtWidgets.QDoubleSpinBox(); self.initial_heading.setRange(0.0, 359.9999)
        self.initial_heading.setDecimals(4); self.initial_heading.setSuffix(" °")
        self.initial_heading.setToolTip("必须手动装订；真北为0°，顺时针为正，不使用GNSS航迹自动替换")
        form.addRow("手动装订初始航向", self.initial_heading)
        self._double(form, "navigation_buffer_seconds", "组合导航历史缓存", "s", 60, 1,
                     "必须不小于最大GNSS时间差；100Hz下5秒约500帧")
        self._double(form, "output_rate_hz", "对准/导航记录输出频率", "Hz", 1000, 2)

    def _build_quality_tab(self):
        page, form = self._form_page(); self.tabs.addTab(page, "GNSS质量门限")
        for key, label, unit, maximum in (
                ("maximum_gnss_age_s", "最大GNSS时间差", "s", 60),
                ("maximum_hacc_m", "最大水平精度hAcc", "m", 10000),
                ("maximum_sacc_m_s", "最大速度精度sAcc", "m/s", 1000),
                ("maximum_pdop", "最大PDOP", "", 100)):
            self._double(form, key, label, unit, maximum, 3)

    def _build_installation_tab(self):
        page, form = self._form_page(); self.tabs.addTab(page, "安装与杆臂")
        self.matrix_rows = []
        matrix_box = QtWidgets.QWidget(); matrix_layout = QtWidgets.QGridLayout(matrix_box)
        matrix_layout.setContentsMargins(0, 0, 0, 0)
        for row in range(3):
            edits = []
            for column in range(3):
                edit = QtWidgets.QDoubleSpinBox(); edit.setRange(-1.0, 1.0); edit.setDecimals(8)
                edit.setSingleStep(0.01); matrix_layout.addWidget(edit, row, column); edits.append(edit)
            self.matrix_rows.append(edits)
        matrix_box.setToolTip("传感器坐标到载体前-右-下(FRD)坐标的右手正交旋转矩阵")
        form.addRow("传感器→载体旋转矩阵", matrix_box)
        self._vector(form, "lever_arm_body_m", "IMU→天线杆臂 x,y,z", "m",
                     "载体系前、右、下三个分量")

    def _build_initial_p_tab(self):
        page, form = self._form_page(); self.tabs.addTab(page, "初始P阵")
        definitions = (
            ("initial_attitude_std_deg", "姿态初始1σ R,P,H", "deg"),
            ("initial_velocity_std_m_s", "速度初始1σ N,E,D", "m/s"),
            ("initial_position_std_m", "位置初始1σ N,E,D", "m"),
            ("initial_gyro_bias_std_deg_h", "陀螺零偏初始1σ x,y,z", "deg/h"),
            ("initial_accel_bias_std_mg", "加计零偏初始1σ x,y,z", "mg"))
        for key, label, unit in definitions:
            self._vector(form, key, label, unit, "用于15维误差状态初始协方差P0的对角项")

    def _build_process_q_tab(self):
        page, form = self._form_page(); self.tabs.addTab(page, "初始Q阵")
        definitions = (
            ("gyro_arw_deg_sqrt_h", "陀螺ARW x,y,z", "deg/√h",
             "Allan -1/2斜率拟合得到的角度随机游走"),
            ("accel_vrw_m_s_sqrt_h", "加计VRW x,y,z", "m/s/√h",
             "Allan -1/2斜率拟合得到的速度随机游走"),
            ("gyro_bias_sigma_deg_h", "陀螺零偏GM稳态1σ x,y,z", "deg/h",
             "Allan零偏不稳定性作为一阶高斯-马尔可夫稳态标准差"),
            ("accel_bias_sigma_mg", "加计零偏GM稳态1σ x,y,z", "mg",
             "Allan零偏不稳定性作为一阶高斯-马尔可夫稳态标准差"),
            ("gyro_bias_correlation_time_s", "陀螺零偏GM相关时间 x,y,z", "s",
             "当前取各轴Allan零偏平台拟合区间的几何中心，必须大于0"),
            ("accel_bias_correlation_time_s", "加计零偏GM相关时间 x,y,z", "s",
             "当前取各轴Allan零偏平台拟合区间的几何中心，必须大于0"))
        for key, label, unit, tip in definitions:
            self._vector(form, key, label, unit, tip)

    def _build_measurement_r_tab(self):
        page, form = self._form_page(); self.tabs.addTab(page, "初始R阵")
        mode = QtWidgets.QComboBox()
        mode.addItem("实时值（F9P hAcc/vAcc/sAcc）", True)
        mode.addItem("固定默认值", False)
        mode.setToolTip("实时模式：位置1σ=[hAcc,hAcc,vAcc]，速度1σ=[sAcc,sAcc,sAcc]；无效分量回退默认值")
        form.addRow("GNSS噪声来源", mode)
        self.bool_widgets["use_realtime_gnss_accuracy"] = mode
        self._double(form, "heading_measurement_std_deg", "装订航向量测1σ", "deg", 180, 3,
                     "精对准期间航向约束对应的R阵标准差")
        self._vector(form, "default_position_measurement_std_m",
                     "默认位置1σ N,E,D", "m", "实时精度关闭或无效时使用")
        self._vector(form, "default_velocity_measurement_std_m_s",
                     "默认速度1σ N,E,D", "m/s", "实时精度关闭或无效时使用")

    @staticmethod
    def _format_vector(value) -> str:
        return ", ".join(f"{float(item):.10g}" for item in value)

    @staticmethod
    def _parse_vector(text: str, name: str) -> tuple[float, float, float]:
        parts = [part.strip() for part in text.replace("，", ",").split(",")]
        if len(parts) != 3: raise ValueError(f"{name} 必须填写三个逗号分隔的数值")
        try: values = tuple(float(part) for part in parts)
        except ValueError as error: raise ValueError(f"{name} 包含非数值内容") from error
        if not all(math.isfinite(value) for value in values): raise ValueError(f"{name} 必须为有限数值")
        return values

    def set_config(self, config: SelfAimConfig) -> None:
        for key, widget in self.scalar_widgets.items(): widget.setValue(getattr(config, key))
        self.initial_heading.setValue(config.initial_heading_deg % 360)
        for row in range(3):
            for column in range(3): self.matrix_rows[row][column].setValue(config.body_from_sensor[row][column])
        for key, widget in self.vector_widgets.items(): widget.setText(self._format_vector(getattr(config, key)))
        for key, widget in self.bool_widgets.items():
            index = widget.findData(bool(getattr(config, key)))
            widget.setCurrentIndex(max(index, 0))

    def current_config(self) -> SelfAimConfig:
        values = {key: widget.value() for key, widget in self.scalar_widgets.items()}
        values["minimum_imu_samples"] = int(values["minimum_imu_samples"])
        values["initial_heading_deg"] = float(self.initial_heading.value())
        values["body_from_sensor"] = tuple(tuple(widget.value() for widget in row)
                                                   for row in self.matrix_rows)
        for key, widget in self.vector_widgets.items():
            values[key] = self._parse_vector(widget.text(), key)
        for key, widget in self.bool_widgets.items(): values[key] = bool(widget.currentData())
        config = SelfAimConfig(**values); config.validate(); return config

    def import_config(self) -> None:
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "导入自瞄配置", str(self.config_path), "JSON 配置 (*.json)")
        if not path: return
        try: config = SelfAimConfig.load(path)
        except (OSError, ValueError, json.JSONDecodeError) as error:
            QtWidgets.QMessageBox.warning(self, "配置无效", str(error)); return
        self.config_path = pathlib.Path(path); self.path_label.setText(path); self.path_label.setToolTip(path)
        self.set_config(config)

    def save_config(self) -> None:
        try: config = self.current_config()
        except ValueError as error:
            QtWidgets.QMessageBox.warning(self, "参数无效", str(error)); return
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "保存自瞄配置", str(self.config_path), "JSON 配置 (*.json)")
        if not path: return
        try: config.save(path, self.config_path)
        except OSError as error:
            QtWidgets.QMessageBox.warning(self, "保存失败", str(error)); return
        self.config_path = pathlib.Path(path); self.path_label.setText(path); self.path_label.setToolTip(path)

    def restore_defaults(self) -> None:
        self.set_config(SelfAimConfig())

    def validate_and_accept(self) -> None:
        try: self.result_config = self.current_config()
        except ValueError as error:
            QtWidgets.QMessageBox.warning(self, "参数无效", str(error)); return
        self.accept()


class LooseNavigationThread(QtCore.QThread):
    """Single owner of the mutable INS/KF state; producers only enqueue data."""

    solution_ready = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.commands: queue.Queue = queue.Queue(maxsize=4096)
        self._shutdown_requested = False

    def _submit(self, command, payload=None) -> bool:
        if self._shutdown_requested:
            return False
        try:
            self.commands.put_nowait((command, payload))
            return True
        except queue.Full:
            return False

    def start_navigation(self, config, initial) -> bool:
        return self._submit("start", (config, initial))

    def submit_imu(self, sample) -> bool:
        return self._submit("imu", sample)

    def submit_gnss(self, observation) -> bool:
        return self._submit("gnss", observation)

    def stop_navigation(self) -> bool:
        if self._shutdown_requested:
            return False
        if self._submit("stop"):
            return True
        # Stopping has priority over stale sensor work.  A full queue already
        # means continuity is lost, so discard it and terminate the estimator.
        while True:
            try: self.commands.get_nowait()
            except queue.Empty: break
        return self._submit("stop")

    def shutdown(self) -> None:
        self._shutdown_requested = True
        while True:
            try: self.commands.get_nowait()
            except queue.Empty: break
        try: self.commands.put_nowait(("shutdown", None))
        except queue.Full: pass

    def run(self) -> None:
        engine = None
        last_output_time = -math.inf
        while True:
            try:
                command, payload = self.commands.get(timeout=0.1)
                if command == "shutdown":
                    break
                if command == "start":
                    config, initial = payload
                    engine = RealtimeLooseNavigation(config)
                    solution = engine.start(initial)
                    last_output_time = -math.inf
                    self.solution_ready.emit(solution)
                elif command == "stop":
                    if engine is not None and engine.running:
                        self.solution_ready.emit(engine.stop())
                    engine = None
                elif command == "imu" and engine is not None and engine.running:
                    solution = engine.update_imu(payload)
                    if not solution.running:
                        self.solution_ready.emit(solution)
                        engine = None
                        self.failed.emit(solution.status)
                        continue
                    interval = 1.0/max(engine.config.output_rate_hz, 1e-9)
                    if (solution.gps_time_s is not None and
                            solution.gps_time_s-last_output_time+1e-9 >= interval):
                        last_output_time = solution.gps_time_s
                        self.solution_ready.emit(solution)
                elif command == "gnss" and engine is not None and engine.running:
                    engine.update_gnss(payload)
                    solution = engine.solution()
                    if solution.gps_time_s is not None:
                        last_output_time = solution.gps_time_s
                    self.solution_ready.emit(solution)
            except queue.Empty:
                continue
            except Exception as error:  # keep acquisition alive; stop only fusion
                engine = None
                self.failed.emit(f"{type(error).__name__}: {error}")


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
        self.imu_stream = self.gnss_stream = self.rawx_stream = self.aim_stream = None
        self.nav_stream = None
        self.event_stream = None
        self.imu_writer: csv.writer | None = None
        self.gnss_writer: csv.writer | None = None
        self.rawx_writer: csv.writer | None = None
        self.aim_writer: csv.writer | None = None
        self.nav_writer: csv.writer | None = None
        self.sync_writer: csv.writer | None = None
        self.sync_stream = None
        self._prev_sync: dict[str, int] | None = None
        self._sync_diag: dict[str, int] | None = None
        self.pending_nav_row: list | None = None
        self.pending_nav_time: float | None = None
        self.last_navigation_history_time: float | None = None
        self.last_navigation_seen_updates = 0
        self.last_navigation_seen_rejections = 0
        self.session_metadata_path: pathlib.Path | None = None
        self.session_metadata: dict = {}
        self.rows_since_flush = 0
        self.imu = {name: deque() for name in ("time", "ax", "ay", "az", "gx", "gy", "gz", "temp")}
        self.speed = {name: deque() for name in ("time", "vn", "ve", "vd", "ground")}
        self.aim_history = {name: deque() for name in (
            "time", "pn", "pe", "pd", "vn", "ve", "vd", "roll", "pitch", "heading",
            "gbx", "gby", "gbz", "abx", "aby", "abz",
            "std_roll", "std_pitch", "std_heading", "std_vn", "std_ve", "std_vd",
            "std_pn", "std_pe", "std_pd", "std_gbx", "std_gby", "std_gbz",
            "std_abx", "std_aby", "std_abz")}
        self.nav_history = {name: deque() for name in (
            "time", "vn", "ve", "vd", "roll", "pitch", "heading")}
        configured_path = pathlib.Path(str(self.settings.value("self_aim_config", DEFAULT_AIM_CONFIG)))
        try:
            self.self_aim_config = SelfAimConfig.load(configured_path)
            self.self_aim_config_path = configured_path
            self.self_aim_config_error = ""
        except (OSError, ValueError, json.JSONDecodeError) as error:
            self.self_aim_config = SelfAimConfig()
            self.self_aim_config_path = DEFAULT_AIM_CONFIG
            self.self_aim_config_error = str(error)
        self.self_aim = RealtimeSelfAim(self.self_aim_config)
        self.last_aim_solution = self.self_aim.solution()
        self.last_aim_output_time = -math.inf
        self.navigation_active = False
        self.last_navigation_solution = None
        self.last_navigation_map_time = -math.inf
        self.navigation_worker = LooseNavigationThread(self)
        self.navigation_worker.solution_ready.connect(self._consume_navigation_solution)
        self.navigation_worker.failed.connect(self._navigation_failed)
        self.navigation_worker.start()
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
        self._pending_ntrip_error_detail = None
        self._logged_fix = None
        self._logged_errors = (0, 0)
        self._build_ui(); self._build_timers(); self.refresh_ports()
        self.log_event("系统", "程序已启动；界面保留最近 5000 条，勾选 LOG 可随采集保存，也可手动导出。")
        if self.self_aim_config_error:
            self.log_event("自瞄", f"配置读取失败，已采用内置默认值：{self.self_aim_config_error}", "WARN")

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
        for name in ("IMU", "GNSS", "RAWX", "AIM", "NAV", "LOG"):
            checkbox = QtWidgets.QCheckBox(name); checkbox.setChecked(name not in ("AIM", "NAV", "LOG"))
            self.save_checkboxes[name] = checkbox
        self.save_checkboxes["AIM"].setToolTip("可选：保存 PC 实时自瞄解算结果 aim.csv")
        self.save_checkboxes["NAV"].setToolTip("可选：保存 PC 实时松组合结果 nav.csv")
        self.save_checkboxes["LOG"].setToolTip("连接前勾选：本次日志保存到采集文件夹中的 event.log")
        self.select_all_button = QtWidgets.QPushButton("一键存储")
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
        self.capture_label = QtWidgets.QLabel("Capture: --")
        for label in (self.rate_label, self.frames_label, self.loss_label, self.time_label,
                      self.fix_label, self.sv_label, self.pdop_label, self.capture_label):
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
        self.tabs.addTab(self._build_self_aim_tab(), "自瞄")
        self.tabs.addTab(self._build_loose_navigation_tab(), "组合导航")
        self.tabs.addTab(self._build_event_log_tab(), "日志")
        outer.addWidget(self.tabs, 1)
        self.setCentralWidget(central)
        self.statusBar().showMessage("选择 PA9 对应的 USB-TTL 串口，默认 460800 bit/s。")

    def _build_event_log_tab(self) -> QtWidgets.QWidget:
        page = QtWidgets.QWidget(); layout = QtWidgets.QVBoxLayout(page)
        controls = QtWidgets.QHBoxLayout()
        self.log_follow = QtWidgets.QCheckBox("自动滚动"); self.log_follow.setChecked(True)
        controls.addWidget(self.log_follow)
        controls.addWidget(QtWidgets.QLabel("输出等级"))
        self.log_level_combo = QtWidgets.QComboBox()
        for title, level in (("INFO 关键及以上", "INFO"), ("WARN 警告及以上", "WARN"),
                             ("ERROR 仅错误", "ERROR"), ("DEBUG 全部调试", "DEBUG")):
            self.log_level_combo.addItem(title, level)
        self.log_level_combo.setToolTip("同时控制日志页和 event.log 后续写入；DEBUG 每 10 秒输出一条精简链路状态")
        controls.addWidget(self.log_level_combo)
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

    def log_event(self, category: str, message: str, level: str = "INFO") -> None:
        # UI-thread events only: never append every IMU/RTCM packet.
        level = level.upper()
        if level not in LOG_LEVEL_VALUES: raise ValueError(f"未知日志等级：{level}")
        selected = self.log_level_combo.currentData() or "INFO"
        if LOG_LEVEL_VALUES[level] < LOG_LEVEL_VALUES[selected]: return
        message = " ".join(str(message).splitlines())[:2000]
        bar = self.event_log.verticalScrollBar(); previous = bar.value()
        stamp = QtCore.QDateTime.currentDateTime().toString("yyyy-MM-dd HH:mm:ss.zzz")
        line = f"{stamp} [{level}] [{category}] {message}"
        cursor = self.event_log.textCursor()
        cursor.movePosition(QtGui.QTextCursor.End)
        if not self.event_log.document().isEmpty(): cursor.insertBlock()
        text_format = QtGui.QTextCharFormat()
        if level in ("WARN", "ERROR"):
            text_format.setForeground(QtGui.QColor("#d32f2f"))
        elif level == "DEBUG":
            text_format.setForeground(QtGui.QColor("#777777"))
        if level == "ERROR":
            text_format.setFontWeight(QtGui.QFont.Bold)
        cursor.insertText(line, text_format)
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
        self.log_event("文件", f"LOG 保存失败：{error}；后续仅显示，CSV 采集继续。", "ERROR")

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
            self.log_event("文件", f"日志导出失败：{error}", "ERROR")
            QtWidgets.QMessageBox.warning(self, "导出失败", str(error))

    def _set_ntrip_status(self, message: str) -> None:
        self.ntrip_label.setText(message)
        now = time.monotonic()
        if "后重连" in message:
            if self._ntrip_interrupted_at is None: self._ntrip_interrupted_at = now
            detail = (f"；{self._pending_ntrip_error_detail}"
                      if self._pending_ntrip_error_detail else "")
            self._pending_ntrip_error_detail = None
            self.log_event("基站重连", message + detail, "ERROR")
        elif "收到有效 RTCM" in message:
            extra = ""
            if self._ntrip_interrupted_at is not None:
                extra = f"；距首次报错 {now - self._ntrip_interrupted_at:.1f} 秒（非差分龄期）"
            self._ntrip_interrupted_at = None
            self.log_event("基站", message + extra)
        else:
            self.log_event("基站", message)

    def _brief_link_state(self) -> str:
        worker = self.serial_worker
        if worker is None:
            return "串口线程不可用"
        sent, pending, max_age = worker.snapshot
        detail = (f"串口已写/未确认={sent}/{pending}B，最大发送等待={max_age:.2f}s，"
                  f"采集丢帧/无效行={self.lost_imu}/{self.invalid_lines}，{self.fix_label.text()}")
        report = worker.report  # Replaced, never mutated by the serial owner.
        if report is not None:
            detail += (f"，STM32收/转发={report[2]}/{report[3]}B，"
                       f"丢字节/串口错={report[4]}/{report[5]}，"
                       f"{self._format_f9p_counters(report)}")
        return detail

    @staticmethod
    def _format_f9p_counters(report) -> str:
        """Do not present unsupported firmware counters as measured zeros."""
        if report[3] > 0 and report[7] == report[8] == report[9] == 0:
            return "F9P收/使用/CRC错=--/--/--（固件未提供计数）"
        return f"F9P收/使用/CRC错={report[7]}/{report[8]}/{report[9]}"

    def _brief_network_state(self) -> str:
        client = self.ntrip_client
        if client is None: return "NTRIP线程不可用"
        queued, _peak, age = client.frames.snapshot()
        return (f"网络={client.network_bytes}B/{client.frame_count}帧，CRC错={client.crc_errors}，"
                f"重连={client.reconnects}，待发={queued}B/{age:.2f}s")

    def _log_link_debug(self, message: str) -> None:
        reason = message.split(" | ", 1)[0]
        if reason.startswith("异常现场"):
            self._pending_ntrip_error_detail = (f"{self._brief_network_state()}；"
                                                f"{self._brief_link_state()}")
            return
        if reason.startswith("有效 RTCM 恢复"):
            self.log_event("基站", "RTCM 数据恢复，当前 NTRIP 连接未重建。")
            return
        if reason == "周期诊断":
            self.log_event("链路", f"{self._brief_network_state()}；{self._brief_link_state()}",
                           "DEBUG")
            return
        if reason.startswith("停止前即时现场："):
            reason = reason.split("；", 1)[0]
        if reason.startswith(("预警：", "MSM兼容：")):
            self.log_event("链路", f"{reason}；{self._brief_link_state()}", "WARN")
            return
        if reason.startswith("停止前即时现场：") or "异常" in reason:
            self.log_event("链路", f"{reason}；{self._brief_link_state()}", "ERROR")
        # 开始连接和首次有效 RTCM 已由 INFO 状态事件覆盖，不重复输出。

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

    def _build_self_aim_tab(self) -> QtWidgets.QWidget:
        page = QtWidgets.QWidget(); layout = QtWidgets.QVBoxLayout(page)
        layout.setContentsMargins(6, 6, 6, 6); layout.setSpacing(5)
        controls = QtWidgets.QHBoxLayout(); controls.setSpacing(6)
        self.aim_load_button = QtWidgets.QPushButton("算法配置…")
        self.aim_load_button.clicked.connect(self.configure_self_aim_config)
        self.aim_start_button = QtWidgets.QPushButton("开始粗对准")
        self.aim_start_button.clicked.connect(self.start_self_aim)
        self.aim_manual_fine_button = QtWidgets.QPushButton("立即进入精对准")
        self.aim_manual_fine_button.clicked.connect(self.start_fine_alignment)
        self.aim_manual_fine_button.setEnabled(False)
        self.aim_stop_button = QtWidgets.QPushButton("停止对准")
        self.aim_stop_button.clicked.connect(self.stop_self_aim); self.aim_stop_button.setEnabled(False)
        self.aim_config_label = QtWidgets.QLabel(self.self_aim_config_path.name)
        self.aim_config_label.setToolTip(str(self.self_aim_config_path))
        for widget in (self.aim_load_button, self.aim_start_button,
                       self.aim_manual_fine_button, self.aim_stop_button):
            controls.addWidget(widget)
        controls.addWidget(QtWidgets.QLabel("配置：")); controls.addWidget(self.aim_config_label)
        controls.addStretch(1); layout.addLayout(controls)

        status = QtWidgets.QHBoxLayout(); status.setSpacing(10)
        self.aim_stage_label = QtWidgets.QLabel("未启动"); self.aim_stage_label.setStyleSheet("font-weight:bold")
        self.aim_progress = QtWidgets.QProgressBar(); self.aim_progress.setRange(0, 1000)
        self.aim_progress.setMaximumWidth(260); self.aim_progress.setMaximumHeight(20)
        self.aim_time_label = QtWidgets.QLabel("用时 -- · 匹配 --")
        self.aim_update_label = QtWidgets.QLabel("GNSS 0/0 · 尚无数据")
        status.addWidget(QtWidgets.QLabel("阶段：")); status.addWidget(self.aim_stage_label)
        status.addWidget(self.aim_progress); status.addWidget(self.aim_time_label)
        status.addWidget(self.aim_update_label, 1); layout.addLayout(status)

        live_values = QtWidgets.QGridLayout(); live_values.setHorizontalSpacing(14)
        self.aim_value_labels = {}
        for index, (key, title) in enumerate((
                ("position", "位置 N/E/D"), ("velocity", "速度 N/E/D"),
                ("attitude", "姿态 R/P/H"), ("gyro_bias", "陀螺零偏 X/Y/Z"),
                ("accel_bias", "加表零偏 X/Y/Z"))):
            label = QtWidgets.QLabel(f"{title}: --")
            label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
            self.aim_value_labels[key] = label
            live_values.addWidget(label, index // 3, index % 3)
        layout.addLayout(live_values)

        self.aim_initial_avp_label = QtWidgets.QLabel("阶段初值：等待粗对准期间的有效数据")
        self.aim_initial_avp_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        layout.addWidget(self.aim_initial_avp_label)

        splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal); layout.addWidget(splitter, 1)
        left_graphics = pg.GraphicsLayoutWidget(); right_graphics = pg.GraphicsLayoutWidget()
        splitter.addWidget(left_graphics); splitter.addWidget(right_graphics)
        self.aim_series_plots: list[pg.PlotItem] = []
        self.aim_series_curves: dict[str, pg.PlotDataItem] = {}
        self.aim_series_groups: list[tuple[str, str, str]] = []
        colors = ("#45a3ff", "#ffad42", "#5bd18b")

        def add_series(graphics, row, title, y_name, unit, channels, labels):
            plot = graphics.addPlot(row=row, col=0)
            self._configure_plot(plot, title, y_name, unit)
            plot.addLegend(offset=(6, 4))
            if row == 4: plot.setLabel("bottom", "相对时间", units="s")
            if self.aim_series_plots: plot.setXLink(self.aim_series_plots[0])
            self.aim_series_plots.append(plot)
            self.aim_series_groups.append(tuple(channels))
            for channel, label, color in zip(channels, labels, colors):
                self.aim_series_curves[channel] = plot.plot(
                    pen=pg.mkPen(color, width=1.3), name=label)

        add_series(left_graphics, 0, "NED位置", "位置", "m",
                   ("pn", "pe", "pd"), ("N", "E", "D"))
        add_series(left_graphics, 1, "NED速度", "速度", "m/s",
                   ("vn", "ve", "vd"), ("Vn", "Ve", "Vd"))
        add_series(left_graphics, 2, "姿态", "角度", "deg",
                   ("roll", "pitch", "heading"), ("Roll", "Pitch", "Heading"))
        add_series(left_graphics, 3, "陀螺仪零偏", "零偏", "deg/s",
                   ("gbx", "gby", "gbz"), ("Bx", "By", "Bz"))
        add_series(left_graphics, 4, "加速度计零偏", "零偏", "m/s²",
                   ("abx", "aby", "abz"), ("Bx", "By", "Bz"))
        add_series(right_graphics, 0, "位置标准差（1σ）", "标准差", "m",
                   ("std_pn", "std_pe", "std_pd"), ("N", "E", "D"))
        add_series(right_graphics, 1, "速度标准差（1σ）", "标准差", "m/s",
                   ("std_vn", "std_ve", "std_vd"), ("N", "E", "D"))
        add_series(right_graphics, 2, "姿态标准差（1σ）", "标准差", "deg",
                   ("std_roll", "std_pitch", "std_heading"), ("Roll", "Pitch", "Heading"))
        add_series(right_graphics, 3, "陀螺零偏标准差（1σ）", "标准差", "deg/s",
                   ("std_gbx", "std_gby", "std_gbz"), ("X", "Y", "Z"))
        add_series(right_graphics, 4, "加表零偏标准差（1σ）", "标准差", "m/s²",
                   ("std_abx", "std_aby", "std_abz"), ("X", "Y", "Z"))
        splitter.setSizes((700, 700))
        return page

    def _build_loose_navigation_tab(self) -> QtWidgets.QWidget:
        page = QtWidgets.QWidget(); layout = QtWidgets.QVBoxLayout(page)
        controls = QtWidgets.QHBoxLayout()
        self.nav_start_button = QtWidgets.QPushButton("结束精对准并启动组合导航")
        self.nav_start_button.clicked.connect(self.start_loose_navigation)
        self.nav_start_button.setEnabled(False)
        self.nav_stop_button = QtWidgets.QPushButton("停止组合导航")
        self.nav_stop_button.clicked.connect(self.stop_loose_navigation)
        self.nav_stop_button.setEnabled(False)
        self.nav_status_label = QtWidgets.QLabel("未启动")
        self.nav_status_label.setStyleSheet("font-weight:bold")
        controls.addWidget(self.nav_start_button); controls.addWidget(self.nav_stop_button)
        controls.addWidget(QtWidgets.QLabel("状态：")); controls.addWidget(self.nav_status_label)
        controls.addStretch(1)
        layout.addLayout(controls)

        metrics = QtWidgets.QGridLayout()
        self.nav_gnss_label = QtWidgets.QLabel("GNSS更新：--")
        self.nav_timing_label = QtWidgets.QLabel("延迟/重放：--")
        metrics.addWidget(self.nav_gnss_label, 0, 0)
        metrics.addWidget(self.nav_timing_label, 0, 1)
        self.nav_value_labels = {}
        for index, (key, title) in enumerate((
                ("time", "导航时刻"), ("position", "组合位置 B/L/H"),
                ("velocity", "NED速度"), ("attitude", "FRD姿态 R/P/H"),
                ("gyro_bias", "陀螺零偏 X/Y/Z"),
                ("accel_bias", "加表零偏 X/Y/Z"))):
            label = QtWidgets.QLabel(f"{title}: --")
            label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
            self.nav_value_labels[key] = label
            metrics.addWidget(label, 1 + index // 3, index % 3)
        layout.addLayout(metrics)

        splitter = QtWidgets.QSplitter(QtCore.Qt.Vertical); layout.addWidget(splitter, 1)
        self.fusion_map_widget = NavigationMapWidget(self.settings)
        splitter.addWidget(self.fusion_map_widget)
        plots = pg.GraphicsLayoutWidget(); splitter.addWidget(plots)
        self.nav_velocity_plot = plots.addPlot(row=0, col=0)
        self._configure_plot(self.nav_velocity_plot, "组合导航NED速度", "速度", "m/s")
        self.nav_velocity_plot.addLegend(offset=(6, 4))
        self.nav_velocity_plot.setLabel("bottom", "相对时间", units="s")
        self.nav_attitude_plot = plots.addPlot(row=0, col=1)
        self._configure_plot(self.nav_attitude_plot, "载体姿态（FRD相对NED）", "角度", "deg")
        self.nav_attitude_plot.addLegend(offset=(6, 4))
        self.nav_attitude_plot.setLabel("bottom", "相对时间", units="s")
        colors = ("#45a3ff", "#ffad42", "#5bd18b")
        self.nav_curves = {}
        for key, label, color in zip(("vn", "ve", "vd"), ("Vn", "Ve", "Vd"), colors):
            self.nav_curves[key] = self.nav_velocity_plot.plot(
                pen=pg.mkPen(color, width=1.4), name=label)
        for key, label, color in zip(
                ("roll", "pitch", "heading"), ("Roll", "Pitch", "Heading"), colors):
            self.nav_curves[key] = self.nav_attitude_plot.plot(
                pen=pg.mkPen(color, width=1.4), name=label)
        splitter.setSizes((520, 360))
        return page

    def configure_self_aim_config(self) -> None:
        dialog = SelfAimConfigDialog(self.self_aim_config, self.self_aim_config_path, self)
        if dialog.exec_() != QtWidgets.QDialog.Accepted: return
        config = dialog.result_config
        self.self_aim_config = config; self.self_aim_config_path = dialog.config_path
        self.self_aim = RealtimeSelfAim(config); self.last_aim_solution = self.self_aim.solution()
        if self.self_aim_config_path.is_file():
            self.settings.setValue("self_aim_config", str(self.self_aim_config_path))
        self.aim_config_label.setText(self.self_aim_config_path.name)
        self.aim_config_label.setToolTip(str(self.self_aim_config_path))
        self.aim_start_button.setEnabled(True); self.aim_stop_button.setEnabled(False)
        self.aim_manual_fine_button.setEnabled(False)
        self._update_aim_labels(self.last_aim_solution)
        self._update_session_metadata(
            effective_self_aim_config=asdict(self.self_aim_config),
            self_aim_config_source=str(self.self_aim_config_path.resolve()))
        self.log_event(
            "自瞄", f"算法参数已从Qt界面应用；输出目标 "
                    f"{self.self_aim_config.output_rate_hz:.2f} Hz。")

    def start_self_aim(self) -> None:
        if self.navigation_active:
            self.log_event("组合导航", "组合导航运行中，不能重新开始对准。", "WARN")
            return
        self.self_aim.start(); self.last_aim_solution = self.self_aim.solution()
        self.last_aim_output_time = -math.inf
        for channel in self.aim_history.values(): channel.clear()
        for curve in self.aim_series_curves.values(): curve.clear()
        self.aim_start_button.setEnabled(False); self.aim_stop_button.setEnabled(True)
        self.aim_load_button.setEnabled(False); self.aim_manual_fine_button.setEnabled(True)
        self._update_session_metadata(
            alignment_started_local=QtCore.QDateTime.currentDateTime().toString(
                QtCore.Qt.ISODateWithMs),
            effective_self_aim_config=asdict(self.self_aim_config))
        self.log_event(
            "自瞄", "已开始：等待同步IMU/GNSS数据并进行粗对准；"
                    f"记录输出目标 {self.self_aim_config.output_rate_hz:.2f} Hz。")
        self._update_aim_labels(self.last_aim_solution)

    def start_fine_alignment(self) -> None:
        accepted = self.self_aim.start_fine_alignment()
        solution = self.self_aim.solution(); self._consume_aim_solution(solution)
        if accepted:
            self.aim_manual_fine_button.setEnabled(False)
            self.log_event("自瞄", "已按用户操作立即进入精对准。")
        else:
            self.log_event("自瞄", f"不能进入精对准：{solution.last_gnss_reason}", "WARN")
        self._update_aim_labels(solution)

    def stop_self_aim(self) -> None:
        if self.self_aim.running:
            self.self_aim.stop(); self.last_aim_solution = self.self_aim.solution()
            self._update_session_metadata(
                alignment_stopped_local=QtCore.QDateTime.currentDateTime().toString(
                    QtCore.Qt.ISODateWithMs))
            self.log_event("自瞄", "已停止。")
        self.aim_start_button.setEnabled(True); self.aim_stop_button.setEnabled(False)
        self.aim_load_button.setEnabled(True); self.aim_manual_fine_button.setEnabled(False)
        self._update_aim_labels(self.last_aim_solution)

    def start_loose_navigation(self) -> None:
        if self.serial_port is None:
            QtWidgets.QMessageBox.warning(self, "未连接串口", "请先连接采集串口。")
            return
        if self.navigation_active:
            return
        try:
            initial = NavigationInitialState.from_alignment(self.self_aim)
        except ValueError as error:
            self.log_event("组合导航", f"不能启动：{error}", "WARN")
            QtWidgets.QMessageBox.warning(self, "不能启动组合导航", str(error))
            return
        if not self.navigation_worker.start_navigation(self.self_aim_config, initial):
            self._navigation_failed("组合导航线程队列已满或已退出")
            return
        # The initial state is copied before alignment is stopped, so the next
        # synchronized IMU sample continues the same timestamp without a gap.
        self.self_aim.stop(); self.last_aim_solution = self.self_aim.solution()
        self.navigation_active = True; self.last_navigation_solution = None
        self.last_navigation_map_time = -math.inf
        self.last_navigation_history_time = None
        self.last_navigation_seen_updates = 0
        self.last_navigation_seen_rejections = 0
        self._flush_pending_nav_row()
        for channel in self.nav_history.values(): channel.clear()
        for curve in self.nav_curves.values(): curve.clear()
        self.fusion_map_widget.clear_track()
        self.nav_start_button.setEnabled(False); self.nav_stop_button.setEnabled(True)
        self.aim_start_button.setEnabled(False); self.aim_stop_button.setEnabled(False)
        self.aim_load_button.setEnabled(False); self.aim_manual_fine_button.setEnabled(False)
        self.nav_status_label.setText(
            f"正在启动… · 输出目标 {self.self_aim_config.output_rate_hz:.2f} Hz")
        self._update_aim_labels(self.last_aim_solution)
        self._update_session_metadata(
            alignment_finished_local=QtCore.QDateTime.currentDateTime().toString(
                QtCore.Qt.ISODateWithMs),
            navigation_started_local=QtCore.QDateTime.currentDateTime().toString(
                QtCore.Qt.ISODateWithMs),
            effective_self_aim_config=asdict(self.self_aim_config))
        self.log_event(
            "组合导航", "已保存精对准末状态，进入NED/FRD实时松组合；"
                        f"记录输出目标 {self.self_aim_config.output_rate_hz:.2f} Hz。")

    def stop_loose_navigation(self, reason: str | bool = "用户停止") -> None:
        # QPushButton.clicked supplies a boolean; programmatic callers supply
        # a diagnostic string used in the event log.
        if not isinstance(reason, str):
            reason = "用户停止"
        if not self.navigation_active:
            return
        self.navigation_active = False
        self._flush_pending_nav_row()
        self._update_session_metadata(
            navigation_stopped_local=QtCore.QDateTime.currentDateTime().toString(
                QtCore.Qt.ISODateWithMs),
            navigation_stop_reason=reason)
        self.navigation_worker.stop_navigation()
        self.nav_start_button.setEnabled(False); self.nav_stop_button.setEnabled(False)
        self.aim_start_button.setEnabled(True); self.aim_load_button.setEnabled(True)
        self.nav_status_label.setText("已停止")
        self.log_event("组合导航", reason)

    def _navigation_failed(self, message: str) -> None:
        self.navigation_active = False
        self._flush_pending_nav_row()
        self._update_session_metadata(
            navigation_stopped_local=QtCore.QDateTime.currentDateTime().toString(
                QtCore.Qt.ISODateWithMs),
            navigation_stop_reason=f"算法异常停止：{message}")
        self.navigation_worker.stop_navigation()
        if hasattr(self, "nav_start_button"):
            self.nav_start_button.setEnabled(False); self.nav_stop_button.setEnabled(False)
            self.aim_start_button.setEnabled(True); self.aim_load_button.setEnabled(True)
            self.nav_status_label.setText("异常停止")
        self.log_event("组合导航", f"算法异常停止：{message}", "ERROR")

    def _consume_navigation_solution(self, solution) -> None:
        self.last_navigation_solution = solution
        if solution.gnss_updates > self.last_navigation_seen_updates:
            output_source = "GNSS_REPLAY_CORRECTED"
        elif solution.gnss_rejections > self.last_navigation_seen_rejections:
            output_source = "GNSS_REJECTED_INS_CONTINUED"
        elif solution.elapsed_s <= 1e-9:
            output_source = "INITIAL"
        else:
            output_source = "INS_PROPAGATION"
        self.last_navigation_seen_updates = solution.gnss_updates
        self.last_navigation_seen_rejections = solution.gnss_rejections
        self.nav_status_label.setText(
            f"{solution.status} · 输出目标 {self.self_aim_config.output_rate_hz:.2f} Hz")
        delay = "--" if not math.isfinite(solution.gnss_delay_s) else f"{solution.gnss_delay_s:.3f}s"
        self.nav_gnss_label.setText(
            f"GNSS更新/拒绝：{solution.gnss_updates}/{solution.gnss_rejections} · "
            f"{solution.last_gnss_reason}")
        self.nav_timing_label.setText(
            f"延迟 {delay} · 重放 {solution.replay_samples}点/{solution.replay_time_ms:.2f}ms · "
            f"缓存 {solution.buffer_span_s:.2f}s · 输入队列 {self.navigation_worker.commands.qsize()}")

        def triplet(values, precision):
            return "/".join("--" if not math.isfinite(v) else f"{v:.{precision}f}" for v in values)

        if solution.gps_time_s is None:
            self.nav_value_labels["time"].setText("导航时刻: --")
        else:
            week = int(solution.gps_time_s // GPS_WEEK_SECONDS)
            tow = solution.gps_time_s - week * GPS_WEEK_SECONDS
            self.nav_value_labels["time"].setText(f"导航时刻: W{week} {tow:.3f}s")
        self.nav_value_labels["position"].setText(
            f"组合位置 B/L/H: {solution.lat_deg:.9f}° / {solution.lon_deg:.9f}° / "
            f"{solution.height_m:.3f}m")
        self.nav_value_labels["velocity"].setText(
            f"NED速度: {triplet(solution.velocity_ned_m_s, 4)} m/s")
        self.nav_value_labels["attitude"].setText(
            f"FRD姿态 R/P/H: {triplet((solution.roll_deg, solution.pitch_deg, solution.heading_deg), 4)} °")
        self.nav_value_labels["gyro_bias"].setText(
            f"陀螺零偏 X/Y/Z: {triplet(solution.gyro_bias_deg_s, 6)} deg/s")
        self.nav_value_labels["accel_bias"].setText(
            f"加表零偏 X/Y/Z: {triplet(solution.accel_bias_m_s2, 6)} m/s²")
        if solution.gps_time_s is None or not math.isfinite(solution.gps_time_s):
            return
        # A GNSS correction can follow the IMU output for exactly the same
        # navigation epoch. Replace that point instead of drawing and saving
        # two different states with an identical timestamp.
        values = {
            "time": solution.elapsed_s,
            **dict(zip(("vn", "ve", "vd"), solution.velocity_ned_m_s)),
            **dict(zip(("roll", "pitch", "heading"),
                       (solution.roll_deg, solution.pitch_deg, solution.heading_deg))),
        }
        same_epoch = (self.last_navigation_history_time is not None and
                      abs(solution.gps_time_s-self.last_navigation_history_time) <= 1e-6 and
                      bool(self.nav_history["time"]))
        for key, value in values.items():
            if same_epoch:
                self.nav_history[key][-1] = value
            else:
                self.nav_history[key].append(value)
        self.last_navigation_history_time = solution.gps_time_s
        if (math.isfinite(solution.lat_deg) and math.isfinite(solution.lon_deg) and
                solution.gps_time_s-self.last_navigation_map_time >= 0.2):
            self.fusion_map_widget.set_position(solution.lat_deg, solution.lon_deg)
            self.last_navigation_map_time = solution.gps_time_s
        if solution.running:
            self._queue_navigation_row(solution, output_source)

    @staticmethod
    def _navigation_row(solution, output_source: str) -> list:
        week = int(solution.gps_time_s // GPS_WEEK_SECONDS)
        tow = solution.gps_time_s-week*GPS_WEEK_SECONDS
        return [
            solution.gps_time_s, week, tow, solution.elapsed_s, solution.status,
            output_source,
            solution.lat_deg, solution.lon_deg, solution.height_m,
            *solution.position_ned_m, *solution.velocity_ned_m_s,
            solution.roll_deg, solution.pitch_deg, solution.heading_deg,
            *solution.gyro_bias_deg_s, *solution.accel_bias_m_s2,
            solution.gnss_updates, solution.gnss_rejections,
            solution.last_gnss_reason, solution.gnss_delay_s,
            solution.replay_samples, solution.replay_time_ms,
            solution.buffer_span_s, *solution.covariance_std]

    def _queue_navigation_row(self, solution, output_source: str) -> None:
        if self.nav_writer is None:
            return
        row = self._navigation_row(solution, output_source)
        if (self.pending_nav_time is not None and
                abs(solution.gps_time_s-self.pending_nav_time) <= 1e-6):
            self.pending_nav_row = row
            return
        self._flush_pending_nav_row()
        self.pending_nav_time = solution.gps_time_s
        self.pending_nav_row = row

    def _flush_pending_nav_row(self) -> None:
        if self.pending_nav_row is not None and self.nav_writer is not None:
            self.nav_writer.writerow(self.pending_nav_row)
            self._periodic_flush()
        self.pending_nav_row = None
        self.pending_nav_time = None

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
        select = not all(checkbox.isChecked() for checkbox in self.save_checkboxes.values())
        for checkbox in self.save_checkboxes.values(): checkbox.setChecked(select)
        self.select_all_button.setText("一键取消" if select else "一键存储")

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

    def _update_session_metadata(self, **updates) -> None:
        """Persist the effective runtime setup without NTRIP/API secrets."""
        if self.session_metadata_path is None:
            return
        self.session_metadata.update(updates)
        try:
            self.session_metadata_path.write_text(
                json.dumps(self.session_metadata, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8")
        except OSError as error:
            self.log_event("文件", f"会话配置保存失败：{error}", "WARN")

    def _open_logs(self) -> bool:
        self.session_metadata_path = None
        self.session_metadata = {}
        self.pending_nav_row = None
        self.pending_nav_time = None
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
            if selected.intersection({"IMU", "GNSS", "RAWX"}):
                path = directory / "sync.csv"
                self.sync_stream = path.open("w", newline="", encoding="utf-8")
                self.sync_writer = csv.writer(self.sync_stream)
                self.sync_writer.writerow(SYNC_COLUMNS)
                created.append(path.name)
            if "AIM" in selected:
                path = directory / "aim.csv"
                self.aim_stream = path.open("w", newline="", encoding="utf-8")
                self.aim_writer = csv.writer(self.aim_stream)
                self.aim_writer.writerow([
                    "gps_time_s", "elapsed_s", "stage", "progress", "roll_deg", "pitch_deg",
                    "heading_deg", "heading_valid", "fine_initial_roll_deg",
                    "fine_initial_pitch_deg", "fine_initial_heading_deg",
                    "fine_initial_vel_n_m_s", "fine_initial_vel_e_m_s", "fine_initial_vel_d_m_s",
                    "fine_initial_lat_deg", "fine_initial_lon_deg", "fine_initial_height_m",
                    "fine_initial_position_samples",
                    "lat_deg", "lon_deg", "height_m", "pos_n_m", "pos_e_m", "pos_d_m",
                    "vel_n_m_s", "vel_e_m_s", "vel_d_m_s", "gyro_bias_x_deg_s",
                    "gyro_bias_y_deg_s", "gyro_bias_z_deg_s", "accel_bias_x_m_s2",
                    "accel_bias_y_m_s2", "accel_bias_z_m_s2", "gnss_updates",
                    "gnss_rejections", "last_gnss_reason", "time_match_s",
                    "std_att_roll_deg", "std_att_pitch_deg", "std_att_heading_deg",
                    "std_vel_n_m_s", "std_vel_e_m_s", "std_vel_d_m_s",
                    "std_pos_n_m", "std_pos_e_m", "std_pos_d_m",
                    "std_gyro_bias_x_deg_s", "std_gyro_bias_y_deg_s", "std_gyro_bias_z_deg_s",
                    "std_accel_bias_x_m_s2", "std_accel_bias_y_m_s2", "std_accel_bias_z_m_s2"])
                created.append(path.name)
            if "NAV" in selected:
                path = directory / "nav.csv"
                self.nav_stream = path.open("w", newline="", encoding="utf-8")
                self.nav_writer = csv.writer(self.nav_stream)
                self.nav_writer.writerow([
                    "gps_time_s", "gps_week", "gps_tow_s", "elapsed_s", "status",
                    "output_source",
                    "lat_deg", "lon_deg", "height_m", "pos_n_m", "pos_e_m", "pos_d_m",
                    "vel_n_m_s", "vel_e_m_s", "vel_d_m_s", "roll_deg", "pitch_deg",
                    "heading_deg", "gyro_bias_x_deg_s", "gyro_bias_y_deg_s",
                    "gyro_bias_z_deg_s", "accel_bias_x_m_s2", "accel_bias_y_m_s2",
                    "accel_bias_z_m_s2", "gnss_updates", "gnss_rejections",
                    "last_gnss_reason", "gnss_delay_s", "replay_samples",
                    "replay_time_ms", "buffer_span_s", "std_att_roll_deg",
                    "std_att_pitch_deg", "std_att_heading_deg", "std_vel_n_m_s",
                    "std_vel_e_m_s", "std_vel_d_m_s", "std_pos_n_m", "std_pos_e_m",
                    "std_pos_d_m", "std_gyro_bias_x_deg_s", "std_gyro_bias_y_deg_s",
                    "std_gyro_bias_z_deg_s", "std_accel_bias_x_m_s2",
                    "std_accel_bias_y_m_s2", "std_accel_bias_z_m_s2"])
                created.append(path.name)
            if selected.intersection({"AIM", "NAV"}):
                self.session_metadata_path = directory / "session.json"
                self.session_metadata = {
                    "schema": "mpu6050-f9p-session-v1",
                    "created_local": QtCore.QDateTime.currentDateTime().toString(
                        QtCore.Qt.ISODateWithMs),
                    "serial_protocol": "v3",
                    "navigation_frame": "NED",
                    "body_frame": "FRD",
                    "selected_outputs": sorted(selected),
                    "serial_port": self.port_combo.currentData() or "",
                    "serial_baud": int(self.baud_combo.currentText()),
                    "self_aim_config_source": str(self.self_aim_config_path.resolve()),
                    "effective_self_aim_config": asdict(self.self_aim_config),
                }
                self._update_session_metadata()
                if self.session_metadata_path.is_file():
                    created.append(self.session_metadata_path.name)
            self.log_event("记录", f"数据目录：{directory}；文件：" + "、".join(created))
            if "LOG" in selected and self.event_stream is None:
                raise OSError("LOG 初始写入失败，未开始采集")
            self.statusBar().showMessage(
                f"保存到 {directory.name}\\" + "、".join(created)); return True
        except OSError as error:
            self.log_event("文件", f"创建数据文件失败：{error}", "ERROR")
            self._close_logs(); QtWidgets.QMessageBox.critical(self, "文件错误", f"无法创建数据文件：\n{error}"); return False

    def _close_logs(self) -> None:
        self._flush_pending_nav_row()
        self._update_session_metadata(
            closed_local=QtCore.QDateTime.currentDateTime().toString(QtCore.Qt.ISODateWithMs))
        for stream in (self.imu_stream, self.gnss_stream, self.rawx_stream,
                       self.aim_stream, self.nav_stream, self.sync_stream):
            if stream is not None: stream.flush(); stream.close()
        self.imu_stream = self.gnss_stream = self.rawx_stream = self.aim_stream = self.nav_stream = None
        self.sync_stream = None
        self.imu_writer = self.gnss_writer = self.rawx_writer = self.aim_writer = self.nav_writer = None
        self.sync_writer = None
        self.rows_since_flush = 0
        self.session_metadata_path = None
        self.session_metadata = {}
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
            self.log_event("串口", f"无法打开 {device}：{error}", "ERROR")
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
        if was_connected:
            self.log_event("串口", reason + "；正在关闭数据文件。",
                           "ERROR" if "异常" in reason else "INFO")
        if self.self_aim.running:
            self.stop_self_aim()
        if self.navigation_active:
            self.stop_loose_navigation("串口断开，组合导航停止")
        self.serial_port = None; self._close_logs(); self.port_combo.setEnabled(True); self.baud_combo.setEnabled(True)
        self._set_log_controls_enabled(True); self.pause_button.setEnabled(False); self.connect_button.setText("连接")
        self.connection_label.setText("● 未连接"); self.connection_label.setStyleSheet("color:#ff6174;font-weight:bold")
        self.capture_label.setText("Capture: --"); self.capture_label.setStyleSheet("")
        self._prev_sync = None; self._sync_diag = None
        self.statusBar().showMessage(reason)

    def _start_serial_worker(self, port):
        self.serial_worker = SerialWorker(port, self)
        worker = self.serial_worker
        worker.diagnostic.connect(lambda message, source=worker:
                                  self._log_link_debug(message)
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
            self.log_event("基站", f"无法启动基站：{error}", "ERROR")
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
        self._pending_ntrip_error_detail = None
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
        if was_active:
            normal_stop = reason in ("基站: 已停止", "基站: 已停止，IMU/GNSS 采集继续",
                                     "基站: 网络线程已退出")
            self.log_event("基站", reason, "INFO" if normal_stop else "ERROR")
        self._ntrip_interrupted_at = None
        self._pending_ntrip_error_detail = None

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
        if line.startswith("# sync,"):
            self._process_sync(line); return
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

    def _process_sync(self, line: str) -> None:
        sync = parse_sync_line(line)
        if sync is None:
            self.invalid_lines += 1
            return
        if self._prev_sync is not None:
            deltas = {
                "d_interrupt_overruns": u32_delta(
                    sync["interrupt_overruns"], self._prev_sync["interrupt_overruns"]),
                "d_cc2_overcapture": u32_delta(
                    sync["cc2_overcapture"], self._prev_sync["cc2_overcapture"]),
                "d_dt_gap_count": u32_delta(
                    sync["dt_gap_count"], self._prev_sync["dt_gap_count"]),
                "d_i2c_errors": u32_delta(
                    sync["i2c_errors"], self._prev_sync["i2c_errors"]),
            }
        else:
            deltas = {"d_interrupt_overruns": 0, "d_cc2_overcapture": 0,
                      "d_dt_gap_count": 0, "d_i2c_errors": 0}
        backlog = (sync["interrupt_count"] - sync["sample_count"]) & 0xFFFFFFFF
        if self.sync_writer is not None:
            self.sync_writer.writerow([
                round(time.time() * 1000), sync["pps"],
                sync["sample_count"], sync["interrupt_count"],
                sync["interrupt_overruns"], sync["cc2_overcapture"],
                sync["dt_gap_count"], sync["i2c_errors"],
                deltas["d_interrupt_overruns"], deltas["d_cc2_overcapture"],
                deltas["d_dt_gap_count"], deltas["d_i2c_errors"], backlog])
            # sync is one row per second; flush it immediately so the file stays
            # current even before the bulk _periodic_flush() threshold is hit.
            if self.sync_stream is not None:
                self.sync_stream.flush()
        self._sync_diag = {**sync, **deltas, "backlog": backlog}
        self._prev_sync = sync
        self._update_capture_status(deltas, backlog)

    def _update_capture_status(self, deltas: dict[str, int], backlog: int) -> None:
        # Warning reflects only the just-elapsed #sync cycle, never the
        # cumulative counters (those live in sync.csv for post-hoc analysis).
        warnings = []
        if deltas["d_interrupt_overruns"] > 0:
            warnings.append(f"SW overrun +{deltas['d_interrupt_overruns']}")
        if deltas["d_cc2_overcapture"] > 0:
            warnings.append(f"HW overcapture +{deltas['d_cc2_overcapture']}")
        if deltas["d_dt_gap_count"] > 0:
            warnings.append(f"Timestamp gap +{deltas['d_dt_gap_count']}")
        if deltas["d_i2c_errors"] > 0:
            warnings.append(f"I2C error +{deltas['d_i2c_errors']}")
        if warnings:
            self.capture_label.setText(
                "Capture warning: " + " · ".join(warnings) + f" · Backlog {backlog}")
            self.capture_label.setStyleSheet("color:#ff6174;font-weight:bold")
            self.capture_label.setToolTip(
                "最近一个 #sync 周期新增的异常（非累计值、非 lost samples）。"
                "backlog 只显示，不设阈值。")
        else:
            self.capture_label.setText(f"Capture: OK · Backlog {backlog}")
            self.capture_label.setStyleSheet("color:#4ade80")
            self.capture_label.setToolTip("")

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
        gps_time = week * GPS_WEEK_SECONDS + tow_us / 1e6 if valid else None
        gyro_rad_s = np.radians(np.asarray(physical[4:7], dtype=float) / 3600.0)
        solution = self.self_aim.update_imu(gps_time, physical[:3], gyro_rad_s,
                                            self.last_dt_ms / 1000.0)
        self._consume_aim_solution(solution)
        if self.navigation_active and gps_time is not None and self.last_dt_ms > 0.0:
            sample_data = ImuSample(
                sample, gps_time, tuple(float(v) for v in physical[:3]),
                tuple(float(v) for v in gyro_rad_s), self.last_dt_ms / 1000.0)
            if not self.navigation_worker.submit_imu(sample_data):
                self._navigation_failed("IMU输入队列已满，未继续使用可能不连续的数据")
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
            previous = self._logged_fix
            degraded = bool(previous and (
                carr_soln < previous[1] or gnss_fix_ok < previous[2]
                or diff_soln < previous[3]))
            self.log_event("定位", f"W{week} {tow_ms / 1000:.3f}s · "
                           f"{self.fix_label.text()} · 差分 {diff_soln} · hAcc {h_acc_m:.3f} m",
                           "WARN" if degraded else "INFO")
            self._logged_fix = state
        self.sv_label.setText(f"卫星数: {num_sv}"); self.pdop_label.setText(f"PDOP: {pdop / 100.0:.2f}")
        self.lat_value.setText(f"{lat:.9f}°"); self.lon_value.setText(f"{lon:.9f}°")
        self.height_value.setText(f"{height:.3f} m"); self.speed_value.setText(f"{velocities[3]:.3f} m/s")
        self.velocity_value.setText(f"N {velocities[0]:.3f}  E {velocities[1]:.3f}  D {velocities[2]:.3f} m/s")
        if (valid and fix >= 2 and gnss_fix_ok != 0 and
                abs(lat) <= 90 and abs(lon) <= 180):
            self.map_widget.set_position(lat, lon)
        observation = GnssObservation(
            gps_time_s=absolute, lat_deg=lat, lon_deg=lon, height_m=height,
            velocity_ned_m_s=np.asarray(velocities[:3], dtype=float),
            valid=bool(valid and fix >= 3 and gnss_fix_ok), hacc_m=h_acc_m,
            sacc_m_s=s_acc_m_s, pdop=pdop / 100.0, vacc_m=v_acc_m)
        self.self_aim.update_gnss(observation)
        self._consume_aim_solution(self.self_aim.solution())
        if self.navigation_active:
            nav_observation = GnssObservation(
                gps_time_s=absolute, lat_deg=lat, lon_deg=lon, height_m=height,
                velocity_ned_m_s=np.asarray(velocities[:3], dtype=float).copy(),
                valid=bool(valid and fix >= 3 and gnss_fix_ok), hacc_m=h_acc_m,
                sacc_m_s=s_acc_m_s, pdop=pdop / 100.0, vacc_m=v_acc_m)
            if not self.navigation_worker.submit_gnss(nav_observation):
                self._navigation_failed("GNSS输入队列已满，组合导航停止")
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
            if self.aim_stream is not None: self.aim_stream.flush()
            if self.nav_stream is not None: self.nav_stream.flush()
            self.rows_since_flush = 0

    def _consume_aim_solution(self, solution) -> None:
        previous_stage = self.last_aim_solution.stage
        self.last_aim_solution = solution
        stage_changed = solution.stage != previous_stage
        if stage_changed:
            level = "WARN" if "等待" in solution.stage else "INFO"
            self.log_event("自瞄", f"阶段：{previous_stage} → {solution.stage}；{solution.last_gnss_reason}", level)
            initial = solution.fine_initial_attitude_deg
            if solution.stage == "精对准" and all(math.isfinite(v) for v in initial):
                position = solution.fine_initial_position_geodetic
                self.log_event("自瞄", "粗对准完成，精对准初始AVP："
                               f"横滚 {initial[0]:.4f}°、俯仰 {initial[1]:.4f}°、"
                               f"航向 {initial[2]:.4f}°；速度[0,0,0]m/s；"
                               f"位置[{position[0]:.9f},{position[1]:.9f},{position[2]:.3f}]，"
                               f"有效GNSS均值={solution.fine_initial_position_samples}点")
        if solution.gps_time_s is None or not math.isfinite(solution.gps_time_s): return
        interval = 1.0 / self.self_aim_config.output_rate_hz
        if (not stage_changed and
                solution.gps_time_s - self.last_aim_output_time + 1e-9 < interval):
            return
        self.last_aim_output_time = solution.gps_time_s
        values = (
            *solution.position_ned_m, *solution.velocity_ned_m_s,
            solution.roll_deg, solution.pitch_deg, solution.heading_deg,
            *solution.gyro_bias_deg_s, *solution.accel_bias_m_s2,
            *solution.covariance_std)
        channels = tuple(name for name in self.aim_history if name != "time")
        self.aim_history["time"].append(solution.elapsed_s)
        for name, value in zip(channels, values): self.aim_history[name].append(value)
        if self.aim_writer is not None:
            self.aim_writer.writerow([
                solution.gps_time_s, solution.elapsed_s, solution.stage, solution.progress,
                solution.roll_deg, solution.pitch_deg, solution.heading_deg,
                int(solution.heading_valid), *solution.fine_initial_attitude_deg,
                *solution.fine_initial_velocity_ned_m_s,
                *solution.fine_initial_position_geodetic,
                solution.fine_initial_position_samples,
                solution.lat_deg, solution.lon_deg,
                solution.height_m, *solution.position_ned_m,
                *solution.velocity_ned_m_s, *solution.gyro_bias_deg_s,
                *solution.accel_bias_m_s2, solution.gnss_updates, solution.gnss_rejections,
                solution.last_gnss_reason, solution.time_match_s, *solution.covariance_std])
            self._periodic_flush()

    def _update_aim_labels(self, solution) -> None:
        if not hasattr(self, "aim_stage_label"): return
        self.aim_stage_label.setText(solution.stage)
        if self.self_aim.running and solution.stage == "精对准":
            self.aim_progress.setRange(0, 0)
            self.aim_progress.setFormat("持续精对准")
        else:
            self.aim_progress.setRange(0, 1000)
            self.aim_progress.setFormat("%p%")
            self.aim_progress.setValue(round(1000 * solution.progress))
        match = "--" if not math.isfinite(solution.time_match_s) else f"{solution.time_match_s:+.3f}s"
        self.aim_time_label.setText(f"{solution.elapsed_s:.1f}s · 匹配 {match}")
        self.aim_update_label.setText(
            f"GNSS {solution.gnss_updates}/{solution.gnss_rejections} · {solution.last_gnss_reason}")
        def triplet(values, precision=4):
            return "/".join("--" if not math.isfinite(value) else f"{value:.{precision}f}"
                            for value in values)

        self.aim_value_labels["position"].setText(
            f"位置 N/E/D: {triplet(solution.position_ned_m, 3)} m")
        self.aim_value_labels["velocity"].setText(
            f"速度 N/E/D: {triplet(solution.velocity_ned_m_s, 4)} m/s")
        self.aim_value_labels["attitude"].setText(
            f"姿态 R/P/H: {triplet((solution.roll_deg, solution.pitch_deg, solution.heading_deg), 4)} °")
        self.aim_value_labels["gyro_bias"].setText(
            f"陀螺零偏 X/Y/Z: {triplet(solution.gyro_bias_deg_s, 6)} deg/s")
        self.aim_value_labels["accel_bias"].setText(
            f"加表零偏 X/Y/Z: {triplet(solution.accel_bias_m_s2, 6)} m/s²")
        initial = solution.fine_initial_attitude_deg
        if all(math.isfinite(v) for v in initial):
            velocity = solution.fine_initial_velocity_ned_m_s
            position = solution.fine_initial_position_geodetic
            self.aim_initial_avp_label.setText(
                f"精对准初始AVP：R {initial[0]:.4f}°  P {initial[1]:.4f}°  H {initial[2]:.4f}°  |  "
                "Vn {:.3f}  Ve {:.3f}  Vd {:.3f} m/s  |  ".format(*velocity) +
                f"B {position[0]:.9f}°  L {position[1]:.9f}°  H {position[2]:.3f} m "
                f"（{solution.fine_initial_position_samples}点均值）")
        else:
            self.aim_initial_avp_label.setText("阶段初值：等待粗对准期间的有效姿态和GNSS位置均值")
        if self.self_aim.running:
            self.aim_manual_fine_button.setEnabled(solution.stage == "粗对准")
        if hasattr(self, "nav_start_button") and not self.navigation_active:
            self.nav_start_button.setEnabled(
                self.self_aim.running and solution.stage == "精对准")

    def _trim_buffers(self) -> None:
        imu_max = int(self.window_spin.value() * 130); gnss_max = int(self.window_spin.value() * 2 + 10)
        for channel in self.imu.values():
            while len(channel) > imu_max: channel.popleft()
        for channel in self.speed.values():
            while len(channel) > gnss_max: channel.popleft()
        aim_max = int(self.window_spin.value() * max(self.self_aim_config.output_rate_hz, 1.0) * 1.2 + 20)
        for channel in self.aim_history.values():
            while len(channel) > aim_max: channel.popleft()
        nav_max = int(self.window_spin.value() * max(self.self_aim_config.output_rate_hz, 1.0) * 1.2 + 20)
        for channel in self.nav_history.values():
            while len(channel) > nav_max: channel.popleft()

    def update_plots(self) -> None:
        self._update_aim_labels(self.last_aim_solution)
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
        if len(self.aim_history["time"]) >= 2:
            x = np.fromiter(self.aim_history["time"], dtype=np.float64); x -= x[-1]
            for name, curve in self.aim_series_curves.items():
                curve.setData(x, np.fromiter(self.aim_history[name], dtype=np.float64))
            self.aim_series_plots[0].setXRange(
                -float(self.window_spin.value()), 0.0, padding=0.0)
        if len(self.nav_history["time"]) >= 2:
            x = np.fromiter(self.nav_history["time"], dtype=np.float64); x -= x[-1]
            for name, curve in self.nav_curves.items():
                curve.setData(x, np.fromiter(self.nav_history[name], dtype=np.float64))
            window = float(self.window_spin.value())
            self.nav_velocity_plot.setXRange(-window, 0.0, padding=0.0)
            self.nav_attitude_plot.setXRange(-window, 0.0, padding=0.0)

    def update_stats(self) -> None:
        errors = (self.lost_imu, self.invalid_lines)
        if errors != self._logged_errors:
            if any(a > b for a, b in zip(errors, self._logged_errors)):
                self.log_event("采集", f"累计 IMU 丢帧 {errors[0]}；无效行 {errors[1]}（本次统计窗口合并报告）",
                               "WARN")
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
                    f"修正历元结束标志 {c.msm_adapter.rewritten} 次，"
                    f"历元边界恢复 {c.msm_adapter.recovered_groups} 组，"
                    f"丢弃不完整组 {c.msm_adapter.dropped_groups} 组/"
                    f"{c.msm_adapter.dropped_frames} 帧；观测数值不改。\n"
                    "距接收是状态消息到达间隔，不是观测历元差分龄期。")
            age = "--" if r[12] == 0xFFFFFFFF else f"{r[12] / 1000:.1f}s"
            self.rtcm_label.setText(
                f"RTCM {ready} · {network}STM32 收/转发 {r[2]:,}/{r[3]:,} B · "
                f"丢字节/串口错 {r[4]}/{r[5]} · "
                f"{self._format_f9p_counters(r)} · 站号 {r[10]} 消息 {r[11]} · "
                f"距接收 {age} · GNSS溢出 {r[6]}")

    def toggle_pause(self) -> None:
        self.plot_paused = not self.plot_paused
        self.pause_button.setText("继续绘图" if self.plot_paused else "暂停绘图")
        self.statusBar().showMessage("绘图暂停，串口接收和文件保存仍继续。" if self.plot_paused else "绘图已继续。")

    def clear_data(self) -> None:
        self._logged_fix = None
        self._logged_errors = (0, 0)
        for channel in (*self.imu.values(), *self.speed.values(), *self.aim_history.values(),
                        *self.nav_history.values()): channel.clear()
        self.last_navigation_history_time = None
        self.last_navigation_seen_updates = 0
        self.last_navigation_seen_rejections = 0
        self.last_sample = self.first_timer_us = self.last_timer_us = None
        self.first_gnss_time = None
        self.total_imu = self.total_gnss = self.total_rawx = self.lost_imu = self.invalid_lines = 0
        self.last_dt_ms = 0.0; self.arrivals.clear(); self.pending_satellites = []; self.satellite_epoch = None
        self.rawx_epoch = None; self.rawx_seen_header = False
        self.map_widget.clear_track(); self.fusion_map_widget.clear_track(); self.sky_plot.set_satellites([])
        for curve in (*self.accel_curves.values(), *self.gyro_curves.values(), self.temp_curve,
                      *self.speed_curves.values(), *self.aim_series_curves.values(),
                      *self.nav_curves.values()): curve.clear()
        self.update_stats()

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:  # noqa: N802
        self.disconnect_serial("应用已关闭。")
        if self.serial_worker is not None or any(worker.isRunning() for worker in self.ntrip_workers):
            for worker in self.ntrip_workers: worker.stop()
            event.ignore()
            QtCore.QTimer.singleShot(100, self.close)
        elif self.navigation_worker.isRunning():
            self.navigation_worker.shutdown()
            if self.navigation_worker.wait(1000):
                event.accept()
            else:
                event.ignore(); QtCore.QTimer.singleShot(100, self.close)
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
