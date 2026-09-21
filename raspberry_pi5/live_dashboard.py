"""LAN dashboard and recording control for the Raspberry Pi GNSS/IMU service.

The server deliberately never opens the UART. It reads an atomic live-state
file published by tools/capture_serial.py, follows recorded GNSS history, and
toggles recording through a volatile control file.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import mimetypes
import os
import pathlib
import re
import shlex
import shutil
import signal
import subprocess
import threading
import time
from collections import deque
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

try:
    from .ntrip_client import write_ntrip_control
except ImportError:  # Direct execution on the Raspberry Pi.
    from ntrip_client import write_ntrip_control


HERE = pathlib.Path(__file__).resolve().parent
DEFAULT_DATA_DIR = HERE.parent / "data" / "decoded"
STATIC_DIR = HERE / "live_dashboard"
SESSION_RE = re.compile(r"^\d{14}(?:_\d{2})?$")
MAX_TRACK_POINTS = 5000
MAX_COMMAND_OUTPUT = 32 * 1024
COMMAND_HELP = """可用命令：
  help                         显示本帮助
  status                       定位、采集和基站摘要
  service status logger        采集服务状态
  service status dashboard     网页服务状态
  service restart logger       安全重启采集服务（保存中拒绝执行）
  service restart dashboard    1秒后重启网页服务
  record start|stop            开始或停止保存采集文件
  base status|reconnect|disconnect
                               查询、重连或断开NTRIP基站
  logs logger|dashboard [行数] 显示最近日志，默认40行、最多200行
  network                      网卡、地址与默认路由
  disk                         数据盘容量
  clear                        仅清空手机上的结果窗口

安全限制：此窗口不执行任意Shell命令，不支持sudo、关机、删除文件或读取密码。"""
SERVICE_UNITS = {
    "logger": "gnss-imu-logger.service",
    "dashboard": "gnss-imu-dashboard.service",
}


def _number(row: dict[str, str], name: str, integer: bool = False):
    value = row.get(name, "")
    try:
        number = int(value) if integer else float(value)
    except (TypeError, ValueError):
        return None
    if not integer and not math.isfinite(number):
        return None
    return number


def normalize_gnss(row: dict[str, str]) -> dict[str, int | float | str | None]:
    fix = _number(row, "fix", True)
    carrier = _number(row, "carr_soln", True)
    fix_names = {0: "无定位", 1: "航迹推算", 2: "2D", 3: "3D", 4: "GNSS+DR", 5: "仅时间"}
    carrier_names = {0: "单点", 1: "RTK浮点", 2: "RTK固定"}
    fix_text = fix_names.get(fix, f"状态{fix}" if fix is not None else "未知")
    if carrier in (1, 2):
        fix_text += " / " + carrier_names[carrier]
    return {
        "gps_week": _number(row, "gps_week", True),
        "gps_tow_ms": _number(row, "gps_tow_ms", True),
        "time_valid": _number(row, "time_valid", True),
        "fix": fix,
        "fix_text": fix_text,
        "num_sv": _number(row, "num_sv", True),
        "carr_soln": carrier,
        "gnss_fix_ok": _number(row, "gnss_fix_ok", True),
        "diff_soln": _number(row, "diff_soln", True),
        "lat_deg": _number(row, "lat_deg"),
        "lon_deg": _number(row, "lon_deg"),
        "height_m": _number(row, "hmsl_m"),
        "h_acc_m": _number(row, "h_acc_m"),
        "v_acc_m": _number(row, "v_acc_m"),
        "vel_n_m_s": _number(row, "vel_n_m_s"),
        "vel_e_m_s": _number(row, "vel_e_m_s"),
        "vel_d_m_s": _number(row, "vel_d_m_s"),
        "ground_speed_m_s": _number(row, "ground_speed_m_s"),
        "s_acc_m_s": _number(row, "s_acc_m_s"),
        "pdop": _number(row, "pdop"),
    }


def normalize_imu(row: dict[str, object]) -> dict[str, int | float] | None:
    """Validate the compact live IMU sample produced by capture_serial.py."""
    integer_names = ("sample", "gps_week", "gps_tow_us", "time_valid", "timer_us")
    float_names = ("ax_m_s2", "ay_m_s2", "az_m_s2", "temp_deg_c",
                   "gx_deg_s", "gy_deg_s", "gz_deg_s")
    result: dict[str, int | float] = {}
    for name in integer_names:
        value = _number(row, name, True)
        if value is None:
            return None
        result[name] = value
    for name in float_names:
        value = _number(row, name)
        if value is None:
            return None
        result[name] = value
    if result["time_valid"] not in (0, 1):
        return None
    return result


def normalize_satellites(value: object) -> list[dict[str, int]]:
    """Validate the bounded live NAV-SAT epoch before sending it to browsers."""
    result: list[dict[str, int]] = []
    if not isinstance(value, list):
        return result
    for item in value[:255]:
        if not isinstance(item, dict):
            continue
        try:
            satellite = {name: int(item[name]) for name in (
                "gnss_id", "sv_id", "cno_dbhz", "elev_deg", "azim_deg", "used")}
        except (KeyError, TypeError, ValueError):
            continue
        if (0 <= satellite["gnss_id"] <= 15 and 0 <= satellite["sv_id"] <= 255 and
                0 <= satellite["cno_dbhz"] <= 255 and
                -90 <= satellite["elev_deg"] <= 90 and
                0 <= satellite["azim_deg"] < 360 and satellite["used"] in (0, 1)):
            result.append(satellite)
    return result


def newest_gnss_file(data_dir: pathlib.Path) -> pathlib.Path | None:
    try:
        sessions = [entry for entry in data_dir.iterdir()
                    if entry.is_dir() and SESSION_RE.fullmatch(entry.name)]
    except OSError:
        return None
    for session in sorted(sessions, key=lambda item: item.name, reverse=True):
        candidate = session / "gnss.csv"
        if candidate.is_file():
            return candidate
    return None


class GnssStore:
    """Thread-safe live state, recorded history, and recording control."""

    def __init__(self, data_dir: pathlib.Path, state_file: pathlib.Path | None = None,
                 control_file: pathlib.Path | None = None,
                 ntrip_file: pathlib.Path | None = None,
                 map_config_file: pathlib.Path | None = None):
        self.data_dir = data_dir
        self.state_file = state_file
        self.control_file = control_file
        self.ntrip_file = ntrip_file
        self.map_config_file = map_config_file
        self._lock = threading.Lock()
        self._path: pathlib.Path | None = None
        self._signature: tuple[int, int] | None = None
        self._latest: dict[str, int | float | str | None] | None = None

    def map_config(self) -> dict[str, object]:
        """Read browser-required AMap values from a Pi-local file."""
        if self.map_config_file is None:
            return {"ok": True, "configured": False}
        try:
            value = json.loads(self.map_config_file.read_text(encoding="utf-8"))
            key = str(value.get("key", "")).strip()
            security = str(value.get("securityJsCode", "")).strip()
        except (OSError, ValueError, TypeError, json.JSONDecodeError):
            return {"ok": True, "configured": False}
        if not re.fullmatch(r"[0-9A-Za-z_-]{16,128}", key):
            return {"ok": True, "configured": False}
        if security and not re.fullmatch(r"[0-9A-Za-z_-]{16,128}", security):
            return {"ok": True, "configured": False}
        return {"ok": True, "configured": True, "key": key,
                "securityJsCode": security}

    def latest(self) -> dict[str, object]:
        with self._lock:
            live = self._read_live_state()
            if live is not None:
                return live
            return self._latest_from_csv()

    def _read_live_state(self) -> dict[str, object] | None:
        if self.state_file is None:
            return None
        try:
            state = json.loads(self.state_file.read_text(encoding="utf-8"))
            stat = self.state_file.stat()
        except (OSError, ValueError, TypeError):
            return None
        now_ms = round(time.time() * 1000)
        heartbeat_ms = state.get("updated_unix_ms")
        if not isinstance(heartbeat_ms, (int, float)):
            heartbeat_ms = round(stat.st_mtime * 1000)
        heartbeat_age = max(0.0, (now_ms - heartbeat_ms) / 1000.0)
        service_running = bool(state.get("service_active")) and heartbeat_age <= 3.5
        imu_updated_ms = state.get("imu_updated_unix_ms")
        imu_age = (max(0.0, (now_ms - imu_updated_ms) / 1000.0)
                   if isinstance(imu_updated_ms, (int, float)) else None)
        raw_imu = state.get("imu")
        imu = normalize_imu(raw_imu) if isinstance(raw_imu, dict) else None
        gnss_updated_ms = state.get("gnss_updated_unix_ms")
        gnss_age = (max(0.0, (now_ms - gnss_updated_ms) / 1000.0)
                    if isinstance(gnss_updated_ms, (int, float)) else None)
        raw_gnss = state.get("gnss")
        data = normalize_gnss(raw_gnss) if isinstance(raw_gnss, dict) else None
        satellites = normalize_satellites(state.get("satellites"))
        satellites_updated_ms = state.get("satellites_updated_unix_ms")
        satellites_age = (max(0.0, (now_ms - satellites_updated_ms) / 1000.0)
                          if isinstance(satellites_updated_ms, (int, float)) else None)
        if not service_running:
            message = "定位服务没有运行或状态已失联"
        elif data is None:
            message = "定位服务运行中，等待第一条GNSS结果"
        elif gnss_age is not None and gnss_age > 3.5:
            message = "定位服务运行中，但GNSS结果超过3.5秒未更新"
        elif state.get("recording_error"):
            message = f"保存启动失败：{state['recording_error']}"
        elif state.get("recording"):
            message = "实时定位正常，正在保存采集文件"
        else:
            message = "实时定位正常，当前不保存采集文件"
        return {
            "ok": data is not None,
            "service_running": service_running,
            "recording": bool(state.get("recording")) and service_running,
            "online": service_running and data is not None and
                      gnss_age is not None and gnss_age <= 3.5,
            "age_s": round(gnss_age, 2) if gnss_age is not None else None,
            "session": state.get("session"),
            "updated_unix_ms": gnss_updated_ms,
            "imu_updated_unix_ms": imu_updated_ms,
            "imu_age_s": round(imu_age, 2) if imu_age is not None else None,
            "imu_online": service_running and imu is not None and
                          imu_age is not None and imu_age <= 2.5,
            "imu": imu,
            "counts": state.get("counts", {}),
            "ubx": state.get("ubx", {}),
            "satellites": satellites,
            "satellites_age_s": (round(satellites_age, 2)
                                 if satellites_age is not None else None),
            "ntrip": state.get("ntrip", {
                "requested": False, "connected": False, "phase": "未连接",
                "error": "", "forwarded_bytes": 0, "bridge_ready": False,
            }),
            "data": data,
            "message": message,
        }

    def _latest_from_csv(self) -> dict[str, object]:
        path = newest_gnss_file(self.data_dir)
        if path is None:
            return {"ok": False, "service_running": False,
                    "recording": False, "online": False,
                    "message": "尚未找到GNSS采集文件"}
        try:
            stat = path.stat()
        except OSError as error:
            return {"ok": False, "service_running": False,
                    "recording": False, "online": False, "message": str(error)}
        signature = (stat.st_mtime_ns, stat.st_size)
        if path != self._path or signature != self._signature:
            self._latest = self._read_last(path)
            self._path, self._signature = path, signature
        age = max(0.0, time.time() - stat.st_mtime)
        if self._latest is None:
            return {"ok": False, "service_running": False,
                    "recording": False, "online": False, "session": path.parent.name,
                    "message": "等待第一条完整GNSS记录"}
        return {
            "ok": True,
            "service_running": age <= 3.5,
            "recording": False,
            "online": age <= 3.5,
            "age_s": round(age, 2),
            "session": path.parent.name,
            "updated_unix_ms": round(stat.st_mtime * 1000),
            "data": self._latest,
        }

    def set_recording(self, enabled: bool) -> dict[str, object]:
        if self.control_file is None:
            return {"ok": False, "message": "本服务未配置采集控制文件"}
        try:
            self.control_file.parent.mkdir(parents=True, exist_ok=True)
            if enabled:
                self.control_file.touch(exist_ok=True)
                message = "已请求开始采集，正在创建新的时间戳目录"
            else:
                self.control_file.unlink(missing_ok=True)
                message = "已请求停止采集，正在刷新并关闭当前文件"
        except OSError as error:
            return {"ok": False, "message": f"采集控制失败：{error}"}
        return {"ok": True, "requested_recording": enabled, "message": message}

    def set_ntrip(self, value: dict[str, object] | None) -> dict[str, object]:
        if self.ntrip_file is None:
            return {"ok": False, "message": "本服务未配置基站控制文件"}
        if value is None:
            try:
                self.ntrip_file.unlink(missing_ok=True)
            except OSError as error:
                return {"ok": False, "message": f"断开基站失败：{error}"}
            return {"ok": True, "requested": False,
                    "message": "已请求断开基站，实时定位继续运行"}
        try:
            write_ntrip_control(self.ntrip_file, value)
        except (OSError, ValueError, TypeError) as error:
            return {"ok": False, "message": f"基站配置失败：{error}"}
        return {"ok": True, "requested": True,
                "message": "已请求连接基站，正在等待有效RTCM"}

    @staticmethod
    def _process(command: list[str], timeout: float = 5.0) -> tuple[bool, str]:
        try:
            completed = subprocess.run(
                command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, encoding="utf-8", errors="replace", timeout=timeout,
                check=False)
        except (OSError, subprocess.TimeoutExpired) as error:
            return False, f"命令执行失败：{error}"
        output = completed.stdout.strip() or "（命令没有输出）"
        if len(output) > MAX_COMMAND_OUTPUT:
            output = "…输出过长，仅保留末尾…\n" + output[-MAX_COMMAND_OUTPUT:]
        return completed.returncode == 0, output

    @staticmethod
    def _ntrip_text(ntrip: object) -> str:
        value = ntrip if isinstance(ntrip, dict) else {}
        requested = bool(value.get("requested"))
        if not requested:
            return "基站：未连接"
        endpoint = f"{value.get('host', '--')}:{value.get('port', '--')}/{value.get('mountpoint', '--')}"
        lines = [f"基站：{value.get('phase') or '连接中'}  {endpoint}",
                 "RTCM：网络 {frames} 帧 / 已转发 {forwarded} B / STM32桥接 {bridge}".format(
                     frames=int(value.get("frames") or 0),
                     forwarded=int(value.get("forwarded_bytes") or 0),
                     bridge="就绪" if value.get("bridge_ready") else "等待")]
        error = value.get("delivery_error") or value.get("error")
        if error:
            lines.append(f"错误：{error}")
        return "\n".join(lines)

    def run_console_command(self, command: object) -> dict[str, object]:
        if not isinstance(command, str) or not command.strip():
            return {"ok": False, "output": "请输入命令；输入 help 查看可用命令。"}
        if len(command) > 160:
            return {"ok": False, "output": "命令过长。"}
        try:
            words = shlex.split(command.strip())
        except ValueError as error:
            return {"ok": False, "output": f"命令格式错误：{error}"}
        words = [word.lower() for word in words]
        if words == ["help"]:
            return {"ok": True, "output": COMMAND_HELP}
        if words == ["status"]:
            state = self.latest()
            data = state.get("data") if isinstance(state.get("data"), dict) else {}
            output = (
                f"定位服务：{'运行中' if state.get('service_running') else '已停止'}\n"
                f"数据保存：{'进行中' if state.get('recording') else '未开启'}"
                f"  会话：{state.get('session') or '--'}\n"
                f"GNSS：{data.get('fix_text', '--')}  卫星：{data.get('num_sv', '--')}"
                f"  PDOP：{data.get('pdop', '--')}  数据龄期：{state.get('age_s', '--')} s\n"
                f"{self._ntrip_text(state.get('ntrip'))}")
            return {"ok": True, "output": output}
        if len(words) == 3 and words[:2] == ["service", "status"]:
            unit = SERVICE_UNITS.get(words[2])
            if unit is None:
                return {"ok": False, "output": "服务名称只能是 logger 或 dashboard。"}
            ok, output = self._process([
                "/usr/bin/systemctl", "--no-pager", "--full", "status", unit])
            return {"ok": ok, "output": output}
        if len(words) == 3 and words[:2] == ["service", "restart"]:
            unit = SERVICE_UNITS.get(words[2])
            if unit is None:
                return {"ok": False, "output": "服务名称只能是 logger 或 dashboard。"}
            if words[2] == "dashboard":
                timer = threading.Timer(1.0, lambda: os.kill(os.getpid(), signal.SIGINT))
                timer.daemon = True
                timer.start()
                return {"ok": True, "output": "已请求重启网页服务；页面约4秒后恢复。"}
            state = self.latest()
            if state.get("recording"):
                return {"ok": False,
                        "output": "当前正在保存数据。请先执行 record stop，再重启采集服务。"}
            ok, output = self._process([
                "/usr/bin/systemctl", "show", "--property=MainPID", "--value", unit])
            try:
                pid = int(output.strip()) if ok else 0
            except ValueError:
                pid = 0
            if pid <= 1:
                return {"ok": False, "output": f"采集服务没有可重启的运行进程：{output}"}
            try:
                os.kill(pid, signal.SIGINT)
            except OSError as error:
                return {"ok": False, "output": f"采集服务重启请求失败：{error}"}
            return {"ok": True, "output": "已安全结束采集进程；systemd将在3秒后自动启动。"}
        if len(words) == 2 and words[0] == "record" and words[1] in ("start", "stop"):
            result = self.set_recording(words[1] == "start")
            return {"ok": bool(result["ok"]), "output": str(result["message"])}
        if words == ["base", "status"]:
            return {"ok": True, "output": self._ntrip_text(self.latest().get("ntrip"))}
        if words == ["base", "disconnect"]:
            result = self.set_ntrip(None)
            return {"ok": bool(result["ok"]), "output": str(result["message"])}
        if words == ["base", "reconnect"]:
            if self.ntrip_file is None:
                return {"ok": False, "output": "本服务未配置基站控制文件。"}
            try:
                value = json.loads(self.ntrip_file.read_text(encoding="utf-8"))
                if not isinstance(value, dict):
                    raise ValueError("配置不是对象")
                write_ntrip_control(self.ntrip_file, value)
            except (OSError, ValueError, TypeError) as error:
                return {"ok": False, "output": f"没有可用的本次开机配置：{error}"}
            return {"ok": True, "output": "已请求重新连接基站。"}
        if 2 <= len(words) <= 3 and words[0] == "logs":
            unit = SERVICE_UNITS.get(words[1])
            if unit is None:
                return {"ok": False, "output": "日志名称只能是 logger 或 dashboard。"}
            try:
                line_count = int(words[2]) if len(words) == 3 else 40
            except ValueError:
                return {"ok": False, "output": "日志行数必须是整数。"}
            if not 1 <= line_count <= 200:
                return {"ok": False, "output": "日志行数范围是1到200。"}
            ok, output = self._process([
                "/usr/bin/journalctl", "--no-pager", "-o", "short-iso",
                "-n", str(line_count), "-u", unit])
            return {"ok": ok, "output": output}
        if words == ["network"]:
            ok1, devices = self._process([
                "/usr/bin/nmcli", "-t", "-f", "DEVICE,TYPE,STATE,CONNECTION",
                "device", "status"])
            ok2, addresses = self._process(["/usr/sbin/ip", "-brief", "address"])
            ok3, routes = self._process(["/usr/sbin/ip", "route", "show", "default"])
            return {"ok": ok1 and ok2 and ok3,
                    "output": f"网卡：\n{devices}\n\n地址：\n{addresses}\n\n默认路由：\n{routes}"}
        if words == ["disk"]:
            try:
                usage = shutil.disk_usage(self.data_dir)
            except OSError as error:
                return {"ok": False, "output": f"读取磁盘容量失败：{error}"}
            gib = 1024 ** 3
            return {"ok": True, "output": (
                f"数据目录：{self.data_dir}\n"
                f"总容量：{usage.total / gib:.2f} GiB\n"
                f"已使用：{usage.used / gib:.2f} GiB\n"
                f"可用：{usage.free / gib:.2f} GiB")}
        return {"ok": False, "output": "不支持该命令；输入 help 查看白名单。"}

    @staticmethod
    def _read_last(path: pathlib.Path):
        try:
            with path.open("rb") as stream:
                header_line = stream.readline()
                header_end = stream.tell()
                header = next(csv.reader([header_line.decode("utf-8")]))
                stream.seek(0, 2)
                end = stream.tell()
                start = max(header_end, end - 16384)
                stream.seek(start)
                if start > header_end:
                    stream.readline()  # discard a possible partial first row
                lines = stream.read().splitlines()
        except (OSError, csv.Error, UnicodeError):
            return None
        for line in reversed(lines):
            try:
                values = next(csv.reader([line.decode("utf-8")]))
            except (csv.Error, UnicodeError):
                continue
            if len(values) == len(header):
                row = dict(zip(header, values))
                if row.get("gps_week"):
                    return normalize_gnss(row)
        return None

    def track(self, limit: int, session: str | None = None) -> dict[str, object]:
        path = None
        if session and SESSION_RE.fullmatch(session):
            candidate = self.data_dir / session / "gnss.csv"
            if candidate.is_file():
                path = candidate
        if path is None:
            path = newest_gnss_file(self.data_dir)
        if path is None:
            return {"ok": False, "points": [], "message": "尚未找到GNSS采集文件"}
        rows: deque[dict[str, object]] = deque(maxlen=limit)
        try:
            with path.open("r", encoding="utf-8", newline="") as stream:
                for row in csv.DictReader(stream):
                    item = normalize_gnss(row)
                    lat, lon = item["lat_deg"], item["lon_deg"]
                    if (isinstance(lat, float) and isinstance(lon, float)
                            and -90 <= lat <= 90 and -180 <= lon <= 180):
                        rows.append({
                            "gps_week": item["gps_week"],
                            "gps_tow_ms": item["gps_tow_ms"],
                            "lat_deg": lat,
                            "lon_deg": lon,
                            "carr_soln": item["carr_soln"],
                        })
        except (OSError, csv.Error, UnicodeError) as error:
            return {"ok": False, "points": [], "message": str(error)}
        return {"ok": True, "session": path.parent.name, "points": list(rows)}


class DashboardHandler(BaseHTTPRequestHandler):
    server_version = "GNSSDashboard/1.0"

    def do_GET(self):  # noqa: N802 - BaseHTTPRequestHandler API
        parsed = urlparse(self.path)
        if parsed.path == "/api/status":
            self._json(self.server.store.latest())
            return
        if parsed.path == "/api/track":
            query = parse_qs(parsed.query)
            try:
                limit = int(query.get("limit", ["600"])[0])
            except ValueError:
                limit = 600
            session = query.get("session", [None])[0]
            self._json(self.server.store.track(
                max(10, min(MAX_TRACK_POINTS, limit)), session=session))
            return
        if parsed.path == "/api/map-config":
            self._json(self.server.store.map_config())
            return
        relative = "index.html" if parsed.path in ("", "/") else parsed.path.lstrip("/")
        target = (STATIC_DIR / relative).resolve()
        try:
            target.relative_to(STATIC_DIR.resolve())
        except ValueError:
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        if not target.is_file():
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        content = target.read_bytes()
        content_type = mimetypes.guess_type(target.name)[0] or "application/octet-stream"
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type + ("; charset=utf-8" if content_type.startswith("text/") else ""))
        self.send_header("Content-Length", str(len(content)))
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        self.wfile.write(content)

    def do_POST(self):  # noqa: N802 - BaseHTTPRequestHandler API
        parsed = urlparse(self.path)
        actions = {"/api/recording/start": True, "/api/recording/stop": False}
        ntrip_actions = {"/api/ntrip/connect", "/api/ntrip/disconnect"}
        if (parsed.path not in actions and parsed.path not in ntrip_actions and
                parsed.path != "/api/command"):
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        # A custom header makes cross-site form submission insufficient to
        # control recording; the UI is served from this same origin.
        if self.headers.get("X-GNSS-Dashboard") != "1":
            self._json({"ok": False, "message": "缺少同源控制标记"},
                       HTTPStatus.FORBIDDEN)
            return
        if parsed.path == "/api/command":
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if not 1 <= length <= 1024:
                    raise ValueError("请求大小无效")
                value = json.loads(self.rfile.read(length).decode("utf-8"))
                if not isinstance(value, dict):
                    raise ValueError("命令请求必须是对象")
            except (ValueError, UnicodeError, json.JSONDecodeError) as error:
                self._json({"ok": False, "output": f"命令请求无效：{error}"},
                           HTTPStatus.BAD_REQUEST)
                return
            result = self.server.store.run_console_command(value.get("command"))
            self._json(result, HTTPStatus.OK if result["ok"] else
                       HTTPStatus.BAD_REQUEST)
            return
        if parsed.path in actions:
            result = self.server.store.set_recording(actions[parsed.path])
        elif parsed.path == "/api/ntrip/disconnect":
            result = self.server.store.set_ntrip(None)
        else:
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if not 1 <= length <= 4096:
                    raise ValueError("请求大小无效")
                value = json.loads(self.rfile.read(length).decode("utf-8"))
                if not isinstance(value, dict):
                    raise ValueError("基站配置必须是对象")
            except (ValueError, UnicodeError, json.JSONDecodeError) as error:
                self._json({"ok": False, "message": f"基站配置无效：{error}"},
                           HTTPStatus.BAD_REQUEST)
                return
            result = self.server.store.set_ntrip(value)
        self._json(result, HTTPStatus.OK if result["ok"] else
                   HTTPStatus.SERVICE_UNAVAILABLE)

    def _json(self, value: object, status: HTTPStatus = HTTPStatus.OK):
        content = json.dumps(value, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(content)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(content)

    def log_message(self, format_string: str, *args):
        # One status request per client per second would otherwise flood the
        # system journal. API/HTTP failures are returned to the browser UI.
        return


class DashboardServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address, store: GnssStore):
        self.store = store
        super().__init__(address, DashboardHandler)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--data-dir", type=pathlib.Path, default=DEFAULT_DATA_DIR)
    parser.add_argument("--state-file", type=pathlib.Path)
    parser.add_argument("--record-control", type=pathlib.Path)
    parser.add_argument("--ntrip-control", type=pathlib.Path)
    parser.add_argument("--map-config", type=pathlib.Path)
    args = parser.parse_args()
    if not STATIC_DIR.is_dir():
        raise SystemExit(f"dashboard assets not found: {STATIC_DIR}")
    server = DashboardServer((args.host, args.port), GnssStore(
        args.data_dir, state_file=args.state_file, control_file=args.record_control,
        ntrip_file=args.ntrip_control, map_config_file=args.map_config))
    print(f"GNSS dashboard: http://{args.host}:{args.port}", flush=True)
    print(f"Data directory: {args.data_dir.resolve()}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
