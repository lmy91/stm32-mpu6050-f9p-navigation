"""Local HTTP forwarder that lets BNC reach the WHU NTRIP caster directly.

The proxy intentionally accepts only ntrip.gnsswhu.cn:2101 and binds to
localhost. It does not inspect or store NTRIP credentials or RTCM payloads.
"""

from __future__ import annotations

import argparse
import select
import socket
import threading
from urllib.parse import urlsplit


ALLOWED_HOST = "ntrip.gnsswhu.cn"
ALLOWED_PORT = 2101


def _read_header(client: socket.socket) -> bytes:
    data = bytearray()
    while b"\r\n\r\n" not in data:
        chunk = client.recv(4096)
        if not chunk:
            break
        data.extend(chunk)
        if len(data) > 65536:
            raise ValueError("HTTP header is too large")
    return bytes(data)


def _rewrite_request(data: bytes) -> bytes:
    header, separator, tail = data.partition(b"\r\n\r\n")
    lines = header.split(b"\r\n")
    method, target, version = lines[0].decode("latin1").split(" ", 2)
    parsed = urlsplit(target)
    if parsed.scheme:
        host = (parsed.hostname or "").lower()
        port = parsed.port or 80
        path = parsed.path or "/"
        if parsed.query:
            path += "?" + parsed.query
    else:
        host_line = next(
            (line for line in lines[1:] if line.lower().startswith(b"host:")), b""
        )
        authority = host_line.partition(b":")[2].strip().decode("ascii")
        host, _, port_text = authority.partition(":")
        host = host.lower()
        port = int(port_text) if port_text else 80
        path = target
    if host != ALLOWED_HOST or port != ALLOWED_PORT:
        raise PermissionError("destination is not the configured WHU caster")
    output = [f"{method} {path} {version}".encode("latin1")]
    output.extend(line for line in lines[1:] if not line.lower().startswith(b"proxy-connection:"))
    return b"\r\n".join(output) + separator + tail


def _relay(left: socket.socket, right: socket.socket) -> None:
    sockets = [left, right]
    while True:
        readable, _, _ = select.select(sockets, [], [], 30.0)
        if not readable:
            continue
        for source in readable:
            data = source.recv(65536)
            if not data:
                return
            destination = right if source is left else left
            destination.sendall(data)


def _handle(client: socket.socket) -> None:
    upstream = None
    try:
        client.settimeout(15.0)
        request = _rewrite_request(_read_header(client))
        upstream = socket.create_connection((ALLOWED_HOST, ALLOWED_PORT), timeout=15.0)
        upstream.sendall(request)
        client.settimeout(None)
        upstream.settimeout(None)
        _relay(client, upstream)
    except Exception:
        try:
            client.sendall(b"HTTP/1.1 502 Bad Gateway\r\nConnection: close\r\n\r\n")
        except OSError:
            pass
    finally:
        if upstream is not None:
            upstream.close()
        client.close()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=32101)
    args = parser.parse_args()
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(("127.0.0.1", args.port))
        server.listen(8)
        while True:
            client, _ = server.accept()
            threading.Thread(target=_handle, args=(client,), daemon=True).start()


if __name__ == "__main__":
    main()
