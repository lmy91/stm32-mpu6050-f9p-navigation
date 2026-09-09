# MPU6050/F9P Navigation Qt Monitor

[Project home](../README_EN.md) | [中文](README.md) | English

This monitor displays IMU, navigation status, and sky-view data. It saves separate GPS-timestamped IMU, navigation, and raw-observation CSV files. Signal and frequency details are kept out of the live UI but remain in the RAWX file for later tightly coupled processing.

## Install dependencies

Run from the repository root:

    D:\anaconda\envs\allan-toolkit\python.exe -m pip install -r host\requirements.txt

Any Python 3.10+ interpreter may be used. PyQtWebEngine provides the optional online AMap view; the local WGS-84 metric track works without an API key.

## Start

Run from the repository root:

    D:\anaconda\envs\allan-toolkit\python.exe host\imu_serial_qt.py

Alternatively, double-click host/run_imu_serial_qt.bat.

## Operation

1. Verify PA9→USB-TTL RX, USB-TTL TX→PA10 (3.3 V TTL), and common ground.
2. Plug USB-TTL into the computer and close any serial terminal using the port.
3. Click Refresh and select the corresponding COM port.
4. Select 460800 baud and click Connect.
5. Use the Navigation tab for position, speed, DOP, satellites and sky view; use the IMU tab for sensor curves.
6. Select any combination of IMU, GNSS, and RAWX logging, or click Select All. After choosing a parent directory, the monitor creates a session folder such as `20260908180500`; only selected files are created. Selecting none keeps live display without logging.
7. Click Disconnect before unplugging USB-TTL.

Pause plots stops UI refresh only; reception and enabled recording continue. Clear plots clears the display buffer without deleting saved CSV files.

Each connection starts a fresh local track. Independent point rendering avoids horizontal path artifacts seen with continuous curves on some Windows/PyQtGraph combinations. The map applies no position-jump filter and displays every valid navigation point saved in the session's `gnss.csv`.

The local east/north axes use the same metric scale. When a new point approaches the frame edge, the view automatically zooms out to keep the complete track visible. After panning or zooming, Best View restores the same fitted equal-scale view manually.

## Input and output

### Event log

Debug summaries are enabled every 10 seconds, with a warning after 5 seconds without a valid RTCM frame and a snapshot before automatic reconnection. They include session/phase, TCP/body/frame deltas and arrival intervals, HTTP/chunk buffering, RTCM residual/resynchronization bytes and CRC errors, pending MSM groups and the outgoing queue. An approximate serial snapshot adds written/unacknowledged bytes, maximum send delay, GUI backlog, device counters and fix state. No request headers, authentication or raw network payloads are logged.

TCP includes HTTP/chunk framing; body bytes exclude it. These counts reset per network connection, unlike the top-level cumulative network count. A 0.5-second read timeout is a polling event, not a reconnect. Stale TCP arrivals indicate no received bytes; fresh TCP with no body points to HTTP decoding; growing body with no valid frames points to RTCM parsing. These distinguish pipeline stages, not a proven VPN/server root cause. Cross-thread snapshots are approximate; arrival intervals are not correction age.

The Log (日志) tab keeps local millisecond timestamps for connection/retry reasons, RTCM recovery, serial errors, recording paths, GNSS fix transitions and aggregated acquisition warnings. Recovery duration starts at the first reported error, not the beginning of the outage; it is not correction age. Credentials and individual observation packets are not logged.

Auto-scroll, copy, manual UTF-8 `.log` export and clear are available. The display retains only the latest 5000 entries. Select `LOG` before connecting to write new session events to `event.log` beside the selected CSV files in the timestamped directory. The file has no 5000-entry cap and excludes earlier display history. `LOG` is unchecked by default; Select All includes it, and recording selections are locked while connected. LOG alone can be selected without CSV files.

Each low-rate event is flushed to the OS buffer; disconnect/exit closes the file (not a power-loss durability guarantee). The log tab shows saving status; a LOG write failure is reported and disables LOG writing without deliberately stopping CSV acquisition. The three CSV formats are unchanged. Clear affects only the display, not the file or acquisition. Manual export cannot overwrite the active log file. Without LOG enabled, export before closing if you need to keep displayed events.

### Direct NTRIP and RTCM forwarding

#### Separate stage deadlines

MSM assembly time is no longer counted as serial queue wait. Each outgoing frame retains its received time and group-release time: assembly is bounded at 2 seconds, release-to-serial-write wait (including queue/credit/partial writes) at 2 seconds, and total host residence at 4 seconds. Dequeue and partial writes never refresh those timestamps. Host residence is not measurement correction age and excludes transmission after the serial write.

The 1024-byte outstanding cap, ACK/progress deadlines, CRC and incomplete-MSM protections remain. Genuine stalls still stop without replaying a possibly written prefix. The serial owner captures counters, pending bytes and timing immediately before stopping, rather than relying on a stale GUI snapshot. Simulated regression covers 1.8-second assembly plus 0.3-second partial sending across the credit window, and confirms that actual send stalls and missing ACKs still stop. Firmware, F9P configuration and CSV formats are unchanged.

#### WUH2 / HPG 1.13 MSM termination compatibility

The observed WUH2 stream marks supported-constellation MSMs as continued, then terminates with NavIC 1137. This F9P HPG 1.13 reports 1137 as unused. `F9pMsmAdapter` waits for a complete group (64 KiB/two-second bounds), omits unsupported NavIC 1131–1137 and clears the multiple-message bit in the last retained MSM, recalculating CRC24Q. All other observation bits are preserved. Normally terminated groups are unchanged; missing terminators, station changes and same-constellation epoch changes fail explicitly. Non-MSM data passes through; reconnects cannot join old groups. Hardware verification subsequently reported `carr_soln=2, diffSoln=1`.

This is an F9P-specific compatibility layer, not transparent RTCM recording. For a future NavIC-capable receiver use `NtripClient(..., f9p_compat=False)`. Network counts include original NavIC; forwarded byte counts will be lower. The status tooltip reports filtered/rewritten counts. Qt never synthesizes RTK fix status; it still comes from the receiver.

Connect the STM32 port at 460800 and wait for bridge-ready status. Base settings default to `ntrip.gnsswhu.cn:2101/WUH200CHN0`; enter credentials or import a user-selected private BNC file. Passwords remain in memory unless explicit local plaintext storage is selected. Never upload private BNC files.

A network thread requests NTRIP v2 over direct TCP, decodes HTTP chunks and checks RTCM CRC24Q. A 64 KiB byte-bounded queue absorbs bursts instead of rejecting the 17th frame; a full queue applies bounded producer backpressure. An independent serial thread owns the same COM port, batches writes and processes acknowledgements without GUI scheduling. Outstanding bytes remain limited to 1024. Two-second backlog/acknowledgement timeouts or UART errors stop injection while capture continues. Network failures retry after five seconds without replaying old queued frames. Disconnecting serial or closing Qt stops both workers. BNC is not required; stop other correction injectors.

The UI shows reconnection count and queued bytes/age. The serial-to-GUI queue is capped at 2 MiB; a prolonged GUI/logging stall stops acquisition with an explicit error rather than silently losing rows. CSV parsing/writing still runs in the GUI thread. This is a buffered, monitored realtime acquisition application, not a hard-realtime deadline guarantee.

System HTTP proxies are bypassed, but VPN TUN/global routes still require a direct-routing exception. This client targets real base streams that do not require periodic GGA (such as the configured WUH2 stream); VRS GGA upload and TLS are not implemented. Diagnostics distinguish network frames, MCU bytes and F9P receipt/use. Arrival interval is not GNSS measurement correction age. RTK float/fixed status is decoded from NAV-PVT `carr_soln` and saved in the existing `gnss.csv`; no additional capture file is created.

The input consists of typed records:

    IMU,sample,gps_week,gps_tow_us,time_valid,timer_us,...
    GNSS,gps_week,gps_tow_ms,time_valid,rx_timer_us,fix,num_sv,flags,flags2,carr_soln,position,accuracy,velocity,speed_accuracy,pdop
    SAT,gps_week,gps_tow_ms,time_valid,gnss_id,sv_id,cno,elevation,azimuth,used
    RAWX,gps_week,rcv_tow_f64hex,leap_s,rec_stat,num_meas,total_meas,rx_timer_us
    RAWX_MEAS,gnss_id,sv_id,sig_id,freq_id,pr_f64hex,cp_f64hex,do_f32hex,lock_ms,cno,pr_std,cp_std,do_std,trk_stat
    RAWX_END,num_meas

Three short-name files are available inside each session folder; only those selected before connecting are created:

- `imu.csv`: GPS time, local capture time, raw IMU and physical units.
- `gnss.csv`: GPS time and receive time, WGS-84 position/velocity and accuracy, PDOP, fix/RTK quality, and satellite count.
- `rawx.csv`: per-signal pseudorange, carrier phase, Doppler, frequency, C/N0, lock time, and quality flags.

If acquisition starts twice within the same second, the next folder is suffixed with `_01` so existing data is never overwritten.

The monitor accepts complete protocol-v3 records only.

## Package a Windows EXE

The project uses PyInstaller onedir mode. It loads files directly from the output directory instead of extracting a large onefile archive at every launch.

Create an isolated packaging environment from the repository root:

    D:\anaconda\envs\allan-toolkit\python.exe -m venv .venv-package
    .\.venv-package\Scripts\python.exe -m pip install --upgrade pip
    .\.venv-package\Scripts\python.exe -m pip install -r host\requirements.txt pyinstaller

Build:

    .\.venv-package\Scripts\python.exe -m PyInstaller --noconfirm --clean host\MPU6050_F9P_Navigation.spec

Output:

    dist\MPU6050_F9P_Navigation\MPU6050_F9P_Navigation.exe

Zip and distribute the complete MPU6050_F9P_Navigation directory, not the EXE alone. build/, dist/, and .venv-package/ are reproducible and excluded from Git.

## Troubleshooting

- Connected but no data: verify the COM port, 460800 baud, PA9→RX, and common ground.
- Port cannot be opened: close other serial programs, reconnect USB-TTL, and refresh the list.
- Garbled text or increasing invalid lines: confirm the firmware format and baud rate.
- A connection can begin in the middle of a RAWX epoch. Measurements before the first epoch header are discarded without increasing the invalid-line count; malformed fields and headerless measurements after synchronization are still counted.
- Batch file closes immediately: run the Python command in PowerShell to see the error.
- Slow EXE startup: use the current onedir spec and launch from the complete output directory.

For command-line capture, decoding, and Allan analysis, see the [tools guide](../tools/README_EN.md).
