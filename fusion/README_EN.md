# Navigation Fusion Layer (Planned)

[中文](README.md) | English

The repository currently provides time synchronization, the serial protocol, logging, and visualization. This directory is reserved for navigation estimation; it does not yet contain an EKF that publishes a fused position solution.

Recommended implementation order:

1. Define common states and frames (NED navigation frame, quaternion attitude, and WGS-84 input are recommended).
2. Drive 100 Hz INS mechanization with each `IMU` record's `gps_week + gps_tow_us`.
3. Update position and velocity from `GNSS` records on the same GPS time axis.
4. Apply Allan parameters, installation-angle calibration, and antenna lever-arm compensation in a loosely coupled error-state EKF.
5. Later add F9P raw pseudorange, Doppler, carrier phase, and RTK status interfaces for tight coupling.

Fusion input should reuse [serial protocol v3](../README_EN.md#protocol-v3) and the canonical CSV outputs instead of introducing a second, inconsistent time representation.
