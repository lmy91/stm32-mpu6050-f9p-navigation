# PC Real-Time Alignment and GNSS/INS Loose Coupling

## File map and parameter boundary (2026-09-30)

| File | Purpose/use |
| --- | --- |
| `realtime_self_aim.py` | Live coarse/fine alignment and configuration validation, started through Qt |
| `realtime_loose_navigation.py` | Live loose coupling and delayed GNSS replay, started through Qt |
| `self_aim_config.json` | Runtime configuration, also editable through Qt |
| `test_realtime_self_aim.py`, `test_realtime_loose_navigation.py` | `python -m unittest fusion.test_realtime_self_aim fusion.test_realtime_loose_navigation` |
| `__init__.py` | Package entry |

The live filter is unchanged. JSON uses the frozen 2026-09-10 `imu_noise` Allan reference, not the offline replay's 0930 Allan, ACF-derived GM or TC/calibration MAT files. BI/plateau-derived GM is an engineering starting point, not independent validation. See [data inventory](../data/README.md).


[中文](README.md) | English

`realtime_self_aim.py` performs coarse/fine alignment. `realtime_loose_navigation.py` then copies the final AVP, biases, covariance and GPS epoch and runs a PC-side 15-state closed-loop GNSS/INS filter. The Qt Integrated Navigation tab displays the fused WGS-84 track, NED velocity, FRD attitude, delay/replay diagnostics, and can record `nav.csv`; alignment remains available as `aim.csv`. A GPS epoch is written only once using its final corrected solution, while `session.json` captures the exact effective runtime configuration for reproducibility.

The workflow is configuration binding, coarse alignment, fine alignment, then the explicit **Finish fine alignment and start integrated navigation** action. Fine alignment otherwise runs continuously. The next timestamped IMU sample continues from the copied fine-alignment state without a reset.

The IMU configuration uses the frozen 2026-09-10 Allan reference in `data/allan_results/imu_noise/allan_parameters.csv`. Gyroscope ARW and accelerometer VRW drive white-noise terms. Each bias axis uses first-order GM with `phi=exp(-dt/tau)` and `Q=sigma^2[1-exp(-2dt/tau)]`. BI and a plateau's geometric midpoint supply provisional sigma and correlation time; initial covariance is configured separately. The nominal process is centred on the coarse-alignment turn-on estimate, not absolute zero. RRW/rate-ramp terms are excluded due to temperature contamination. This is not independent GM validation.

The R page selects real-time or fixed GNSS noise. Real-time mode builds `position sigma=[hAcc,hAcc,vAcc]` and `velocity sigma=[sAcc,sAcc,sAcc]` for every F9P epoch; an invalid or zero reported value falls back to the configured default. Fixed mode always uses the configured three-axis defaults.

A short GNSS outage never stops propagation. The navigation worker retains `navigation_buffer_seconds` of IMU samples and state snapshots. A delayed GNSS position/velocity observation restores the state at its actual epoch, performs the update, and replays every later buffered IMU sample to the newest processed epoch. Measurements older than `maximum_gnss_age_s` or the retained history are rejected. Innovation gates remain disabled; validity, age, hAcc, sAcc and PDOP checks remain active.

Fine alignment uses closed-loop error-state bias feedback. Bias errors are defined as estimated minus true bias. After every accepted GNSS update, the nominal estimates are corrected with `b_g <- b_g-delta_b_g` and `b_a <- b_a-delta_b_a`, then the 15-state error vector is reset. Every subsequent IMU sample is mechanized with `omega_m-b_g` and `f_m-b_a`; raw recorded IMU values are not modified.

An MPU6050 cannot reliably gyrocompass while static, so heading is accepted only from the manual value in Algorithm Configuration and is never replaced by GNSS course. During fine alignment it is applied as a heading constraint with configurable standard deviation, so yaw error is estimated and fed back. At the coarse-to-fine transition, the current roll/pitch/manual heading are used, velocity is initialized to zero, and position is initialized from the mean of all valid coarse-stage GNSS positions. The live 15 physical states and all 15 standard deviations are displayed and optionally recorded throughout every stage.

Navigation axes are North-East-Down (NED), and body axes are Front-Right-Down (FRD). `body_from_sensor` remains the explicit 3x3 IMU-sensor-to-FRD rotation used by both alignment and navigation; raw IMU logging is unchanged. Serial reception, Qt parsing/display and the single-owner navigation worker are separated, with a bounded typed-data queue between them.

This is a research/integration prototype, not flight-certified software. Local-origin reset for long trajectories, higher-order coning/sculling, complete calibration-state estimation, redundancy/FDE and raw-observation tight coupling remain future work. Fusion input reuses [serial protocol v3](../README_EN.md#protocol-v3).
