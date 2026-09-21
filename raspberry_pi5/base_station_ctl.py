#!/usr/bin/env python3
"""Terminal control for the live NTRIP base-station connection."""

from __future__ import annotations

import argparse
import getpass
import json
import pathlib
import sys
import time

from ntrip_client import write_ntrip_control


DEFAULT_CONTROL = pathlib.Path("/run/gnss-imu/ntrip.json")
DEFAULT_STATE = pathlib.Path("/run/gnss-imu/live.json")


def prompt(label: str, default: str = "") -> str:
    suffix = f" [{default}]" if default else ""
    value = input(f"{label}{suffix}: ").strip()
    return value or default


def read_json(path: pathlib.Path) -> dict[str, object]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError, TypeError):
        return {}
    return value if isinstance(value, dict) else {}


def ntrip_state(path: pathlib.Path) -> dict[str, object]:
    value = read_json(path).get("ntrip", {})
    return value if isinstance(value, dict) else {}


def show_status(path: pathlib.Path) -> bool:
    state = ntrip_state(path)
    if not state.get("requested"):
        print("基站：未连接")
        return False
    phase = str(state.get("phase") or "连接中")
    endpoint = ":".join((str(state.get("host") or "--"),
                         str(state.get("port") or "--")))
    mountpoint = str(state.get("mountpoint") or "--")
    print(f"基站：{phase}  {endpoint}/{mountpoint}")
    print("RTCM：网络 {frames} 帧 / 已转发 {forwarded} B / STM32桥接 {bridge}".format(
        frames=int(state.get("frames") or 0),
        forwarded=int(state.get("forwarded_bytes") or 0),
        bridge="就绪" if state.get("bridge_ready") else "等待"))
    error = str(state.get("delivery_error") or state.get("error") or "")
    if error:
        print(f"错误：{error}")
    return bool(state.get("connected")) and not error


def connect(args: argparse.Namespace) -> int:
    existing = read_json(args.control_file)
    host = args.host or prompt("服务器", str(existing.get("host") or "ntrip.gnsswhu.cn"))
    port_text = str(args.port) if args.port else prompt("端口", str(existing.get("port") or 2101))
    mountpoint = args.mountpoint or prompt(
        "挂载点", str(existing.get("mountpoint") or "WUH200CHN0"))
    username = args.username
    if username is None:
        username = prompt("用户名", str(existing.get("username") or ""))
    if args.password_stdin:
        password = sys.stdin.readline().rstrip("\r\n")
    else:
        password = getpass.getpass("密码（不会显示）: ")
    try:
        write_ntrip_control(args.control_file, {
            "host": host, "port": int(port_text), "mountpoint": mountpoint,
            "username": username, "password": password,
        })
    except (OSError, ValueError, TypeError) as error:
        print(f"连接请求失败：{error}", file=sys.stderr)
        return 2
    print("已提交连接请求，等待有效 RTCM…")
    deadline = time.monotonic() + max(0.0, args.wait)
    while time.monotonic() < deadline:
        state = ntrip_state(args.state_file)
        if state.get("connected") or state.get("error") or state.get("delivery_error"):
            return 0 if show_status(args.state_file) else 1
        time.sleep(0.25)
    show_status(args.state_file)
    return 0


def disconnect(args: argparse.Namespace) -> int:
    try:
        args.control_file.unlink(missing_ok=True)
    except OSError as error:
        print(f"断开请求失败：{error}", file=sys.stderr)
        return 2
    print("已请求断开基站；实时定位和串口采集继续运行")
    return 0


def reconnect(args: argparse.Namespace) -> int:
    value = read_json(args.control_file)
    if not value:
        print("没有可重连的易失配置，请先运行 gnss-imu-base connect", file=sys.stderr)
        return 2
    try:
        write_ntrip_control(args.control_file, value)
    except (OSError, ValueError, TypeError) as error:
        print(f"重连请求失败：{error}", file=sys.stderr)
        return 2
    print("已请求重新连接基站")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="GNSS/IMU NTRIP 基站控制")
    parser.add_argument("--control-file", type=pathlib.Path, default=DEFAULT_CONTROL,
                        help=argparse.SUPPRESS)
    parser.add_argument("--state-file", type=pathlib.Path, default=DEFAULT_STATE,
                        help=argparse.SUPPRESS)
    commands = parser.add_subparsers(dest="command", required=True)
    connect_parser = commands.add_parser("connect", help="交互输入账号并连接")
    connect_parser.add_argument("--host")
    connect_parser.add_argument("--port", type=int)
    connect_parser.add_argument("--mountpoint")
    connect_parser.add_argument("--username")
    connect_parser.add_argument("--password-stdin", action="store_true",
                                help="从标准输入读取密码，避免写入命令历史")
    connect_parser.add_argument("--wait", type=float, default=12.0,
                                help="等待连接结果的秒数，默认12")
    commands.add_parser("disconnect", help="断开并清除易失账号")
    commands.add_parser("reconnect", help="使用本次开机的易失配置重连")
    commands.add_parser("status", help="显示基站和RTCM转发状态")
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    if args.command == "connect":
        return connect(args)
    if args.command == "disconnect":
        return disconnect(args)
    if args.command == "reconnect":
        return reconnect(args)
    return 0 if show_status(args.state_file) else 1


if __name__ == "__main__":
    raise SystemExit(main())
