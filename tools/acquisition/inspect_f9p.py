"""Read-only UBX polls on the F9P native USB port (not the STM32 COM port).

Print receiver mode, navigation validity and signal quality. No VALSET, reset,
RTCM injection, persistent settings or files. Close u-center before running.
"""
import argparse
import collections
import contextlib
import json
import struct
import time

import serial


def packet(cls, mid, payload=b""):
    data = bytes((cls, mid)) + struct.pack("<H", len(payload)) + payload
    a = b = 0
    for value in data:
        a = (a + value) & 255; b = (b + a) & 255
    return b"\xb5\x62" + data + bytes((a, b))


def frames(buffer):
    while len(buffer) >= 8:
        start = buffer.find(b"\xb5\x62")
        if start < 0:
            del buffer[:-1]; return
        del buffer[:start]
        if len(buffer) < 8: return
        length = int.from_bytes(buffer[4:6], "little")
        if length > 16384:
            del buffer[:1]; continue
        if len(buffer) < length + 8: return
        data = bytes(buffer[:length + 8])
        if packet(data[2], data[3], data[6:-2]) != data:
            del buffer[:1]; continue
        del buffer[:length + 8]
        yield data[2], data[3], data[6:-2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port")
    parser.add_argument("--seconds", type=float, default=5)
    parser.add_argument("--details", action="store_true")
    parser.add_argument("--via-stm32", help="send bounded polls via idle STM32 COM7; read F9P UART1 mirror on COM4")
    args = parser.parse_args()
    keys = {0x20030001: "TMODE", 0x20140011: "DGNSSMODE", 0x10730004: "UART1_RTCM_IN"}
    polls = [(10, 4, b""), (1, 7, b""), (1, 53, b""), (1, 67, b""),
             (6, 139, bytes(4) + b"".join(struct.pack("<I", k) for k in keys))]
    result = {}; counts = collections.Counter(); data = bytearray()
    rtcm_status = collections.Counter()
    port = serial.Serial(port=None, baudrate=115200, timeout=0, write_timeout=1)
    port.dtr = False; port.rts = False; port.port = args.port
    with port, contextlib.ExitStack() as stack:
        sender = port
        if args.via_stm32:
            sender = serial.Serial(port=None, baudrate=460800, timeout=0, write_timeout=1)
            sender.dtr = False; sender.rts = False; sender.port = args.via_stm32
            stack.enter_context(sender)
        request = b"".join(packet(cls, mid, payload) for cls, mid, payload in polls)
        assert len(request) < 1024  # fits the MCU credit window; no concurrent correction injector
        sender.write(request)
        end = time.monotonic() + args.seconds
        while time.monotonic() < end:
            data.extend(port.read(min(port.in_waiting, 8192)))
            for cls, mid, p in frames(data):
                counts[f"{cls:02x}-{mid:02x}"] += 1
                if (cls, mid) == (2, 50) and len(p) == 8:
                    message = int.from_bytes(p[6:8], "little")
                    rtcm_status[f"type={message},used={(p[1] >> 1) & 3},crc_failed={p[1] & 1}"] += 1
                if (cls, mid) == (10, 4):
                    result["MON_VER"] = [part.decode("ascii", "replace").rstrip("\0")
                                         for part in [p[:30], p[30:40]] +
                                         [p[i:i+30] for i in range(40, len(p), 30)]]
                elif (cls, mid) == (6, 139) and len(p) >= 4:
                    pos = 4
                    while pos + 4 < len(p):
                        key = struct.unpack_from("<I", p, pos)[0]; pos += 4
                        size = {1: 1, 2: 1, 3: 2, 4: 4, 5: 8}.get((key >> 28) & 7)
                        if size is None or pos + size > len(p): break
                        result[keys.get(key, hex(key))] = int.from_bytes(p[pos:pos+size], "little")
                        pos += size
                elif (cls, mid) == (1, 7) and len(p) >= 92:
                    result["PVT"] = {"tow_ms": struct.unpack_from("<I", p)[0], "fix": p[20],
                                     "flags": p[21], "carr_soln": (p[21] >> 6) & 3,
                                     "diff_soln": (p[21] >> 1) & 1, "num_sv": p[23],
                                     "hacc_mm": struct.unpack_from("<I", p, 40)[0]}
                elif (cls, mid) == (1, 53) and len(p) >= 8:
                    result["SAT"] = [{"gnss": p[i], "sv": p[i+1], "cno": p[i+2],
                                      "flags": struct.unpack_from("<I", p, i+8)[0]}
                                     for i in range(8, len(p)-11, 12)]
                elif (cls, mid) == (1, 67) and len(p) >= 8:
                    result["SIG"] = [{"gnss": p[i], "sv": p[i+1], "sig": p[i+2],
                                      "cno": p[i+6], "quality": p[i+7], "corr_source": p[i+8],
                                      "flags": struct.unpack_from("<H", p, i+10)[0]}
                                     for i in range(8, len(p)-15, 16)]
            time.sleep(0.005)
    result["message_counts"] = counts
    result["rtcm_status"] = rtcm_status
    result["config_response_received"] = "TMODE" in result
    if not args.details:
        sats = result.pop("SAT", [])
        signals = result.pop("SIG", [])
        result["satellites_tracked"] = sum(s["cno"] > 0 for s in sats)
        result["signal_count"] = len(signals)
        result["signal_correction_sources"] = dict(collections.Counter(s["corr_source"] for s in signals))
        result["signal_quality_levels"] = dict(collections.Counter(s["quality"] for s in signals))
    print(json.dumps(result, indent=2, ensure_ascii=True))


if __name__ == "__main__": main()
