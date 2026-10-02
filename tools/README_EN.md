# MPU6050/F9P Tool Index

On Windows, use `python -X utf8` for analysis tools that print Chinese or superscript units. Use the discover command below for legacy tests with directory-local imports.

[Project home](../README_EN.md) | [中文](README.md) | English

These tools match STM32 serial protocol v3 and separately save GPS-timestamped IMU, 1 Hz navigation, and 1 Hz RXM-RAWX observations.

## Tools

Tools were grouped by purpose on 2026-10-02. Run commands from the repository root; dependency lists and the downloader source ZIP remain in `tools/`.

| Folder | Purpose |
| --- | --- |
| [acquisition/](acquisition/README.md) | Serial capture, RAWX decoding, F9P polling and Python tests |
| [quality_control/](quality_control/README.md) | Session quality, synchronization and RTCM checks |
| [calibration/](calibration/README.md) | Temperature fitting, 24-position calibration and SOP |
| [noise_analysis/](noise_analysis/README.md) | Allan analysis and GM identification/validation |
| [repeatability/](repeatability/README.md) | Analysis of manually recorded power cycles |
| [visualization/](visualization/README.md) | KML export and MATLAB tests |
| [station_observation_downloader/](station_observation_downloader/README.md) | Qt station observations and broadcast ephemeris downloader |
| [legacy/](legacy/README.md) | Historical BNC, timing, innovation and plotting helpers |

Initialize MATLAB functions with `addpath('tools'); setup_tools;`. Python commands use the new category paths. The historical Raspberry Pi service in this repository now uses `tools/acquisition/capture_serial.py`; installed copies need the updated service configuration. The separate production repository retains its own update procedure.

The [WUH2 observation downloader](station_observation_downloader/README.md) has a separate Qt interface: double-click `station_observation_downloader/run_gui.bat`. It reads rover RINEX 3 times and downloads the minimum required 15-minute segments by default. Full-day observations and daily mixed broadcast ephemeris have independent optional checkboxes. Results default to the rover folder. Closing the window stops the task and its processing children; valid files are reused on restart. A source ZIP is included at `tools/station_observation_downloader.zip`; Python dependencies and external CRX2RNX/GFZRNX tools are documented in the package README.

| File | Purpose | Default output |
| --- | --- | --- |
| `acquisition/capture_serial.py` | Headless capture of the complete stream | IMU, navigation, and RAWX CSV files in a session folder |
| `acquisition/decode_rawx.py` | Validate and summarize RAWX observations | Epoch completeness, signal quality, lock resets, and phase-Doppler outlier candidates |
| `quality_control/check_rtcm_bridge.py` | Bounded hardware smoke test using the same Qt forwarding code | Console counters only |
| `acquisition/inspect_f9p.py` | Read-only UBX polls over the receiver native USB port | Console summary; `--details` for per-signal data |
| `noise_analysis/allan_noise_identification.py` | Read the canonical IMU CSV and identify Allan noise terms | `data/allan_results/` |
| `repeatability/analyze_turn_on_bias.py` | Analyze manually recorded power-cycle sessions: raw-domain TC, fixed calibration, QC and covariance of run means | Experiment `analysis/` directory; [usage](repeatability/TURN_ON_BIAS.md) |
| `calibration/fit_temp_order_selection.m` | Compare per-axis polynomial orders 1–5; recommendations only | Order-selection MAT/CSV/report/plot |
| `calibration/fit_temp_bias_raw.m` | Raw-domain TC fit; explicit `axisOrder` is authoritative | Coefficient MAT/CSV, full-rate corrected CSV, comparison plots |
| `calibration/calib24_static_numbered_tempcomp.m` | Apply TC before 24-position calibration | Matched calibration MAT/CSV/plots |
| `calibration/calib24_static_numbered.m` | Uncompensated reference calibration, not current replay default | Raw-domain calibration outputs |
| `calibration/run_tempcal_sop.m` | Run order selection, TC fit and calibration | `data/calib24/`; Allan remains a separate Python step |
| `noise_analysis/allan_compare_tc_configs.py` | Raw / TC / TC+calibration Allan comparison | Session result directory; `--help` documents stage/layout options |
| `quality_control/check_sync.py` | Read-only sync-counter QC | Console; `python tools/quality_control/check_sync.py <session>/sync.csv` |
| `noise_analysis/gm_autocorrelation_analysis.m` | Calibrated six-axis ACF; correct 10 s averaging attenuation | Provisional GM parameters, blocks, MAT and plots |
| `noise_analysis/gm_validate_parameters.m` | Segments, detrending, independent records and 30–600 s audit | Separate `gm_validation/`; original GM CSV is untouched |
| `visualization/position_to_kml.m` | Uniformly sampled WGS84 KML, no Mapping Toolbox | KML plus exported positions/time |
| `visualization/tests/test_position_to_kml.m` | XML, units, resampling and edge-case regression | Pass/fail output, isolated temporary files |

## Offline workflow and prerequisites (2026-09-30)

Run commands from the repository root. Install analysis dependencies with
`python -m pip install -r tools/requirements_temp_analysis.txt`.
The [e-bike replay](../data/decoded/20260923104556_电动车2/README.md) needs external PSINS initialization, but all replay data/parameters are published.
TC, GM and KML tools do not require PSINS. Re-fitting TC or GM needs the excluded long raw CSV; full independent validation also needs the two excluded independent records.
The included pilot is continuous-power data, not a verified power-cycle covariance.

```matlab
addpath('tools'); setup_tools;
run_tempcal_sop('skipStep1',true); % Frozen TC + published 24-position inputs; overwrites calibration
% With the long raw recordings restored:
% run('tools/noise_analysis/gm_autocorrelation_analysis.m');
% results = gm_validate_parameters;
gm_validate_parameters(struct('replotOnly',true)); % Included validation MAT only
addpath('tools/visualization/tests'); test_position_to_kml;
[llh1,t1] = position_to_kml(llh_rad,time_s,'track.kml');
```

KML input is `[latitude_rad,longitude_rad,height_m]` by default. Output positions use degrees.
The default grid is integer seconds, with linear interpolation and no extrapolation.
Use `'AngleUnit','deg'` for degree inputs. Default `'AltitudeMode','clampToGround'` retains
height values but displays a terrain track; `'absolute'` requires sea-level height, not raw ellipsoid height.
The current replay exports `kml/combined_navigation_1hz.kml` from continuous antenna-position propagation logs.

Auxiliary tools: `quality_control/analyze_latest_data.py <session>` summarizes a capture; `legacy/bnc_direct_proxy.py`
is only for legacy BNC connections, not normal Qt NTRIP.
`legacy/deep_dive_timing.py`, `legacy/innovation_dive.py`, `legacy/extract_viz_data.py` and `legacy/gen_viz_html.py`
have historical hardcoded paths: inspect/configure them and restore their data before use.
`python -X utf8 -m unittest discover -s tools/acquisition -p 'test_*.py'` runs capture/decoder regression.
See the [TC SOP](../docs/温补标定SOP.md), [GM audit guide](noise_analysis/GM_VALIDATION.md) and
[snapshot guide](../docs/版本整理与使用指南_20260930.md). Re-running fitting can overwrite frozen outputs;
never mix TC coefficients with calibration from a different configuration.

## Install

`check_rtcm_bridge.py --seconds 180 --stall-gui` deliberately pauses GUI handling for half a second every ten seconds while the serial thread keeps running. Statistics include reconnects, byte-queue peak, maximum send-queue age, incomplete-MSM drop counters and RTCM MSM headers. MSM epochs retain their constellation time scales: add 14 seconds to BDS for GPST; GLONASS combines day of week and time of day and cannot be directly subtracted from GPS TOW. The printed ECEF position belongs to the reference station, not the rover.

`python tools/acquisition/inspect_f9p.py COM3` sends only UBX polls, not VALSET, reset or RTCM. Verify the native USB port first; do not substitute the STM32 COM7. A false `config_response_received` means no configuration response was received, not that the settings equal zero.

`acquisition/capture_serial.py` remains capture-only in normal PC CLI use. In Raspberry Pi
service mode, explicit `--ntrip-control` enables NTRIP and consumes `#RTCM`
credit reports inside the sole UART owner. Never open one port in two programs.

In Raspberry Pi service mode, the script continuously publishes live position
but creates the three CSV files only while a volatile control file exists. With
`--ubx-port`, it also records the original F9P stream as `f9p.ubx` in that session.
Stopping recording leaves the UART and live position running. Normal PC CLI use
still starts recording immediately, and the web process never opens the UART.

After closing Qt and stopping other correction injectors, run `python tools/quality_control/check_rtcm_bridge.py COM7 --seconds 40 --bnc <private-config-path>` to test the real Qt path. Without `--bnc`, it only reads acquisition statistics. The test writes no files, credentials or coordinates. Exit code 2 means the base-enabled test did not meet error-free, loss-free capture and positive receiver-feedback checks; it is not by itself proof of a UART defect.

Run from the repository root:

    D:\anaconda\envs\allan-toolkit\python.exe -m pip install -r tools\requirements.txt

Any Python 3.10+ interpreter may be used. The Qt monitor, command-line capture, and serial terminals cannot own the same COM port simultaneously.

## 1. Command-line capture

The tested USB-TTL port on the current computer is COM7:

    D:\anaconda\envs\allan-toolkit\python.exe tools\acquisition\capture_serial.py COM7 --hours 12

The default baud rate is 460800. `--hours 0` runs until Ctrl+C and closes the files safely. Like the Qt monitor, each run creates a session folder containing:

    data\decoded\YYYYMMDDHHMMSS\imu.csv
    data\decoded\YYYYMMDDHHMMSS\gnss.csv
    data\decoded\YYYYMMDDHHMMSS\rawx.csv

Save only selected types:

    D:\anaconda\envs\allan-toolkit\python.exe tools\acquisition\capture_serial.py COM7 --save imu gnss

Any combination of `imu`, `gnss`, and `rawx` is accepted; all three are enabled by default. Runs started within the same second receive an `_01` suffix and never overwrite existing data.

For the Raspberry Pi GPIO5/RXD2 tap of F9P UART1, add the second serial port:

    python3 tools/acquisition/capture_serial.py /dev/ttyAMA0 --baud 460800 --ubx-port /dev/ttyAMA2 --ubx-baud 115200

This creates `f9p.ubx` in the same session. The second reader continuously drains
the UART but writes bytes only while recording is active; bytes are not decoded or modified.

The capture tool prints one flushed status line every five seconds with the recent IMU rate,
cumulative GNSS/RAWX/SAT counts, lost frames, invalid lines, and elapsed time. Use
`--status-interval 2` for a two-second interval or `--status-interval 0` to disable periodic
status output. Direct terminal runs refresh the same line in place; on the Raspberry Pi systemd
deployment, use `gnss-imu-status` for the same single-line view.

The capture tool reports lost IMU frames, invalid lines, and satellite records. `SAT`/`SAT_END` records are counted but not duplicated into the GNSS navigation table.

## 2. Allan noise identification

The capture tool and Qt monitor produce canonical physical-unit IMU CSV files directly:

    D:\anaconda\envs\allan-toolkit\python.exe tools\noise_analysis\allan_noise_identification.py data\decoded\20260908180500\imu.csv --rate 100 --skip-minutes 30 --points 90

`--rate` is the nominal sampling rate, currently 100 Hz; `--skip-minutes` discards warm-up; `--points` must be at least 30. Results include Allan and stability plots, parameter CSV files, and a Chinese interpretation report.

## Current serial protocol

    IMU,sample,gps_week,gps_tow_us,time_valid,timer_us,ax_raw,ay_raw,az_raw,temp_raw,gx_raw,gy_raw,gz_raw
    GNSS,gps_week,gps_tow_ms,time_valid,rx_timer_us,fix,num_sv,flags,flags2,carr_soln,lat_e7,lon_e7,hmsl_mm,h_acc_mm,v_acc_mm,vel_n_mms,vel_e_mms,vel_d_mms,g_speed_mms,s_acc_mms,pdop_x100
    SAT,gps_week,gps_tow_ms,time_valid,gnss_id,sv_id,cno_dbhz,elev_deg,azim_deg,used
    SAT_END,gps_week,gps_tow_ms,time_valid,num_svs
    RAWX,gps_week,rcv_tow_f64hex,leap_s,rec_stat,num_meas,total_meas,rx_timer_us
    RAWX_MEAS,gnss_id,sv_id,sig_id,freq_id,pr_f64hex,cp_f64hex,do_f32hex,lock_ms,cno,pr_std,cp_std,do_std,trk_stat
    RAWX_END,num_meas

Canonical IMU CSV:

    sample,gps_week,gps_tow_us,time_valid,timer_us,time_s,dt_s,ax_raw,ay_raw,az_raw,temp_raw,gx_raw,gy_raw,gz_raw,ax_m_s2,ay_m_s2,az_m_s2,temp_deg_c,gx_deg_h,gy_deg_h,gz_deg_h

Canonical GNSS CSV:

    gps_week,gps_tow_ms,time_valid,rx_timer_us,fix,num_sv,flags,flags2,carr_soln,gnss_fix_ok,diff_soln,lat_deg,lon_deg,hmsl_m,h_acc_m,v_acc_m,vel_n_m_s,vel_e_m_s,vel_d_m_s,ground_speed_m_s,s_acc_m_s,pdop

`time_valid=1` means GPS time is valid. Coordinates are WGS-84. The capture tool accepts complete protocol-v3 records only. Angular rates are stored in deg/h and converted to rad/s internally by the Allan tool.

The RAWX CSV restores the receiver's IEEE-754 values and adds `signal` and `frequency_mhz`. GLONASS frequencies include the `freq_id` channel offset; unknown future signal IDs are retained.

## Recommendations

- After power-up, wait for an F9P fix and verify `time_valid=1`.
- For a static Allan test, rigidly mount and warm up the IMU for about 30 minutes; avoid temperature changes and cable motion.
- Run a 5–10 minute pilot first and confirm `lost=0`, `invalid=0`, and one GNSS row per second.
- Analyze a copied snapshot of a file that is still being recorded; never use two writers on one file.

## Troubleshooting

- Port busy: close the Qt monitor, u-center, and every other serial program.
- Garbled or invalid lines: select the STM32 USB-TTL port at 460800, not the C099 USB port.
- No GNSS rows: check crossed PA2/PA3 wiring, common ground, and the C099 J4 ARD route.
- Invalid GPS time: move the antenna to an open-sky location and wait for valid F9P time.
- Too few Allan samples: reduce `--skip-minutes` or record a longer static data set.
