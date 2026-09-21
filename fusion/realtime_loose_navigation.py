"""Real-time delayed-measurement GNSS/INS loose integration.

The nominal INS uses local NED coordinates.  Body axes are front-right-down
(FRD); ``body_from_sensor`` remains the explicit IMU-sensor to body rotation.
Only one owner thread may call this mutable engine.  Serial reception may run
elsewhere and enqueue immutable :class:`ImuSample`/:class:`GnssObservation`
objects for that owner.
"""

from __future__ import annotations

import math
import time
from collections import deque
from dataclasses import dataclass

import numpy as np

from .realtime_self_aim import (
    EARTH_RATE,
    G0,
    GnssObservation,
    SelfAimConfig,
    _dcm_from_euler,
    _euler_from_dcm,
    _matrix,
    _orthogonalize,
    _radii,
    _skew,
    _vec,
)


@dataclass(frozen=True)
class ImuSample:
    """One synchronized IMU sample in the physical sensor coordinate system."""

    sequence: int
    gps_time_s: float
    accel_sensor_m_s2: tuple[float, float, float]
    gyro_sensor_rad_s: tuple[float, float, float]
    dt_s: float


@dataclass
class NavigationInitialState:
    """Complete closed-loop state handed over by the fine alignment."""

    gps_time_s: float
    origin_geodetic_rad_m: tuple[float, float, float]
    c_nb: np.ndarray
    position_ned_m: np.ndarray
    velocity_ned_m_s: np.ndarray
    gyro_bias_rad_s: np.ndarray
    accel_bias_m_s2: np.ndarray
    gyro_bias_reference_rad_s: np.ndarray
    accel_bias_reference_m_s2: np.ndarray
    covariance: np.ndarray

    @classmethod
    def from_alignment(cls, alignment) -> "NavigationInitialState":
        if alignment.origin is None or alignment.current_time is None:
            raise ValueError("精对准尚未形成有效位置和时间")
        if alignment.stage != "精对准":
            raise ValueError("只能使用正在运行的精对准末状态启动组合导航")
        return cls(
            float(alignment.current_time), tuple(float(v) for v in alignment.origin),
            np.asarray(alignment.c_nb, dtype=float).copy(),
            np.asarray(alignment.position_ned, dtype=float).copy(),
            np.asarray(alignment.velocity_ned, dtype=float).copy(),
            np.asarray(alignment.gyro_bias, dtype=float).copy(),
            np.asarray(alignment.accel_bias, dtype=float).copy(),
            np.asarray(alignment.gyro_bias_reference, dtype=float).copy(),
            np.asarray(alignment.accel_bias_reference, dtype=float).copy(),
            np.asarray(alignment.p, dtype=float).copy(),
        )


@dataclass
class NavigationSolution:
    gps_time_s: float | None
    elapsed_s: float
    running: bool
    status: str
    lat_deg: float
    lon_deg: float
    height_m: float
    position_ned_m: tuple[float, float, float]
    velocity_ned_m_s: tuple[float, float, float]
    roll_deg: float
    pitch_deg: float
    heading_deg: float
    gyro_bias_deg_s: tuple[float, float, float]
    accel_bias_m_s2: tuple[float, float, float]
    covariance_std: tuple[float, ...]
    gnss_updates: int
    gnss_rejections: int
    last_gnss_reason: str
    gnss_delay_s: float
    replay_samples: int
    replay_time_ms: float
    buffer_span_s: float


@dataclass
class _Snapshot:
    time_s: float
    c_nb: np.ndarray
    position_ned: np.ndarray
    velocity_ned: np.ndarray
    gyro_bias: np.ndarray
    accel_bias: np.ndarray
    p: np.ndarray
    x: np.ndarray
    last_gyro_body: np.ndarray

    def clone(self) -> "_Snapshot":
        return _Snapshot(
            self.time_s, self.c_nb.copy(), self.position_ned.copy(),
            self.velocity_ned.copy(), self.gyro_bias.copy(),
            self.accel_bias.copy(), self.p.copy(), self.x.copy(),
            self.last_gyro_body.copy())


@dataclass
class _HistoryEntry:
    sample: ImuSample
    after: _Snapshot


class RealtimeLooseNavigation:
    """15-state closed-loop ESKF with delayed GNSS update and IMU replay."""

    TIME_EPSILON_S = 1e-6

    def __init__(self, config: SelfAimConfig):
        config.validate()
        self.config = config
        self.sensor_to_body = _matrix(config.body_from_sensor, "body_from_sensor")
        self.running = False
        self.status = "未启动"
        self.origin = None
        self.start_time = self.current_time = None
        self.c_nb = np.eye(3)
        self.position_ned = np.zeros(3)
        self.velocity_ned = np.zeros(3)
        self.gyro_bias = np.zeros(3)
        self.accel_bias = np.zeros(3)
        self.gyro_bias_reference = np.zeros(3)
        self.accel_bias_reference = np.zeros(3)
        self.p = np.eye(15)
        self.x = np.zeros(15)
        self.last_gyro_body = np.zeros(3)
        self._base_snapshot = None
        self._history: deque[_HistoryEntry] = deque()
        self.gnss_updates = 0
        self.gnss_rejections = 0
        self.last_gnss_reason = "尚无数据"
        self.last_gnss_time = -math.inf
        self.last_gnss_delay = math.nan
        self.last_replay_samples = 0
        self.last_replay_ms = 0.0

    def start(self, initial: NavigationInitialState) -> NavigationSolution:
        if not math.isfinite(initial.gps_time_s):
            raise ValueError("组合导航初始GPS时间无效")
        if np.asarray(initial.covariance).shape != (15, 15):
            raise ValueError("组合导航初始P阵必须为15x15")
        self.origin = tuple(float(v) for v in initial.origin_geodetic_rad_m)
        self.current_time = self.start_time = float(initial.gps_time_s)
        self.c_nb = _orthogonalize(np.asarray(initial.c_nb, dtype=float).copy())
        self.position_ned = _vec(initial.position_ned_m, "初始NED位置").copy()
        self.velocity_ned = _vec(initial.velocity_ned_m_s, "初始NED速度").copy()
        self.gyro_bias = _vec(initial.gyro_bias_rad_s, "初始陀螺零偏").copy()
        self.accel_bias = _vec(initial.accel_bias_m_s2, "初始加计零偏").copy()
        self.gyro_bias_reference = _vec(
            initial.gyro_bias_reference_rad_s, "陀螺零偏GM参考值").copy()
        self.accel_bias_reference = _vec(
            initial.accel_bias_reference_m_s2, "加计零偏GM参考值").copy()
        covariance = np.asarray(initial.covariance, dtype=float)
        if not np.all(np.isfinite(covariance)):
            raise ValueError("组合导航初始P阵含非有限值")
        self.p = 0.5 * (covariance + covariance.T)
        self.x = np.zeros(15)
        self.last_gyro_body = np.zeros(3)
        self._history.clear()
        self._base_snapshot = self._snapshot()
        self.gnss_updates = self.gnss_rejections = 0
        self.last_gnss_reason = "等待GNSS位置/速度量测"
        self.last_gnss_time = -math.inf
        self.last_gnss_delay = math.nan
        self.last_replay_samples = 0
        self.last_replay_ms = 0.0
        self.running = True
        self.status = "组合导航运行"
        return self.solution()

    def stop(self) -> NavigationSolution:
        self.running = False
        self.status = "已停止"
        return self.solution()

    def update_imu(self, sample: ImuSample) -> NavigationSolution:
        if not self.running:
            return self.solution()
        if not math.isfinite(sample.gps_time_s):
            self.status = "IMU时间无效"
            return self.solution()
        if self.current_time is None:
            raise RuntimeError("组合导航未初始化")
        dt = sample.gps_time_s - self.current_time
        if dt <= self.TIME_EPSILON_S:
            self.status = f"IMU时间未递增：{dt:+.6f}s"
            return self.solution()
        if not 0.001 <= dt <= 0.1:
            # Continuing after a long unobserved interval would make the
            # realtime solution look valid although mechanization is broken.
            self.running = False
            self.status = f"已停止：IMU时间间隔异常 {dt:.6f}s"
            return self.solution()
        accel_body, gyro_body = self._body_measurement(sample)
        self._propagate(accel_body, gyro_body, dt)
        # Store the timestamp-derived interval.  Replay must reproduce the
        # exact state timeline and must not depend on a caller's stale dt.
        normalized = ImuSample(
            sample.sequence, float(sample.gps_time_s),
            tuple(float(v) for v in sample.accel_sensor_m_s2),
            tuple(float(v) for v in sample.gyro_sensor_rad_s), float(dt))
        self._history.append(_HistoryEntry(normalized, self._snapshot()))
        self._trim_history()
        self.status = "组合导航运行"
        return self.solution()

    def update_gnss(self, observation: GnssObservation) -> bool:
        observation.velocity_ned_m_s = _vec(
            observation.velocity_ned_m_s, "GNSS速度").copy()
        good, reason = self._gnss_good(observation)
        if not self.running or self.current_time is None:
            self.last_gnss_reason = "组合导航未运行"
            return False
        if not good:
            return self._reject_gnss(reason)
        delay = self.current_time - observation.gps_time_s
        self.last_gnss_delay = delay
        if delay < -self.TIME_EPSILON_S:
            return self._reject_gnss(f"GNSS历元领先当前IMU {abs(delay):.3f}s")
        if delay > self.config.maximum_gnss_age_s:
            return self._reject_gnss(f"GNSS延迟 {delay:.3f}s超限")
        if observation.gps_time_s <= self.last_gnss_time + self.TIME_EPSILON_S:
            return self._reject_gnss("GNSS历元重复或乱序")
        if self._base_snapshot is None or (
                observation.gps_time_s < self._base_snapshot.time_s-self.TIME_EPSILON_S):
            return self._reject_gnss("GNSS历元早于历史缓存")

        started = time.perf_counter()
        entries = list(self._history)
        target = float(observation.gps_time_s)
        prefix = [entry for entry in entries
                  if entry.sample.gps_time_s < target-self.TIME_EPSILON_S]
        if prefix:
            self._restore(prefix[-1].after)
        else:
            self._restore(self._base_snapshot)

        first_after = next((entry for entry in entries
                            if entry.sample.gps_time_s > target+self.TIME_EPSILON_S), None)
        exact = next((entry for entry in entries
                      if abs(entry.sample.gps_time_s-target) <= self.TIME_EPSILON_S), None)
        if exact is not None:
            self._restore(exact.after)
        elif self.current_time < target-self.TIME_EPSILON_S:
            if first_after is None:
                # This can only occur for tiny floating point disagreement at
                # the current endpoint; there is no measurement to propagate.
                self._restore(self._snapshot_at_current_endpoint(entries))
                if abs(self.current_time-target) > self.TIME_EPSILON_S:
                    return self._reject_gnss("GNSS历元没有对应IMU区间")
            else:
                accel_body, gyro_body = self._body_measurement(first_after.sample)
                self._propagate(accel_body, gyro_body, target-self.current_time)

        self._measurement_update(observation)
        corrected_target = self._snapshot()

        rebuilt: deque[_HistoryEntry] = deque(prefix)
        if exact is not None:
            rebuilt.append(_HistoryEntry(exact.sample, corrected_target.clone()))
        elif abs(target-self._base_snapshot.time_s) <= self.TIME_EPSILON_S:
            self._base_snapshot = corrected_target.clone()

        replayed = 0
        for entry in entries:
            if entry.sample.gps_time_s <= target+self.TIME_EPSILON_S:
                continue
            dt = entry.sample.gps_time_s-self.current_time
            if dt <= self.TIME_EPSILON_S:
                continue
            accel_body, gyro_body = self._body_measurement(entry.sample)
            self._propagate(accel_body, gyro_body, dt)
            rebuilt.append(_HistoryEntry(entry.sample, self._snapshot()))
            replayed += 1
        self._history = rebuilt
        self._trim_history()
        self.gnss_updates += 1
        self.last_gnss_time = target
        self.last_gnss_reason = "GNSS位置/速度更新并完成IMU重放"
        self.last_replay_samples = replayed
        self.last_replay_ms = (time.perf_counter()-started)*1000.0
        self.status = "组合导航运行"
        return True

    def _snapshot_at_current_endpoint(self, entries):
        return entries[-1].after if entries else self._base_snapshot

    def _reject_gnss(self, reason: str) -> bool:
        self.gnss_rejections += 1
        self.last_gnss_reason = reason
        self.last_replay_samples = 0
        self.last_replay_ms = 0.0
        return False

    def _gnss_good(self, obs):
        if not obs.valid:
            return False, "GNSS解无效"
        values = (obs.gps_time_s, obs.lat_deg, obs.lon_deg, obs.height_m,
                  obs.hacc_m, obs.sacc_m_s, obs.pdop, *obs.velocity_ned_m_s)
        if not all(math.isfinite(float(v)) for v in values):
            return False, "GNSS含非有限值"
        if abs(obs.lat_deg) > 90 or abs(obs.lon_deg) > 180:
            return False, "GNSS坐标越界"
        if obs.hacc_m > self.config.maximum_hacc_m:
            return False, f"hAcc {obs.hacc_m:.2f}m超限"
        if obs.sacc_m_s > self.config.maximum_sacc_m_s:
            return False, f"sAcc {obs.sacc_m_s:.2f}m/s超限"
        if obs.pdop > self.config.maximum_pdop:
            return False, f"PDOP {obs.pdop:.2f}超限"
        return True, "有效"

    def _body_measurement(self, sample: ImuSample):
        return (self.sensor_to_body @ _vec(sample.accel_sensor_m_s2, "IMU加速度"),
                self.sensor_to_body @ _vec(sample.gyro_sensor_rad_s, "IMU角速度"))

    def _propagate(self, accel, gyro, dt):
        tau_g = _vec(self.config.gyro_bias_correlation_time_s, "gyro_bias_correlation_time_s")
        tau_a = _vec(self.config.accel_bias_correlation_time_s, "accel_bias_correlation_time_s")
        decay_g = np.exp(-dt/tau_g)
        decay_a = np.exp(-dt/tau_a)
        lat, _, height = self._geodetic_from_position()
        rm, rn = _radii(lat)
        vn, ve, _ = self.velocity_ned
        omega_ie = np.array((EARTH_RATE*math.cos(lat), 0.0,
                             -EARTH_RATE*math.sin(lat)))
        omega_en = np.array((ve/(rn+height), -vn/(rm+height),
                             -ve*math.tan(lat)/(rn+height)))
        omega_nb_b = gyro-self.gyro_bias-self.c_nb.T@(omega_ie+omega_en)
        self.c_nb = _orthogonalize(self.c_nb@(np.eye(3)+_skew(omega_nb_b*dt)))
        specific_n = self.c_nb@(accel-self.accel_bias)
        self.velocity_ned += (specific_n+np.array((0.0, 0.0, G0))
                              - np.cross(2*omega_ie+omega_en, self.velocity_ned))*dt
        self.position_ned += self.velocity_ned*dt
        self.gyro_bias = (self.gyro_bias_reference
                          + decay_g*(self.gyro_bias-self.gyro_bias_reference))
        self.accel_bias = (self.accel_bias_reference
                           + decay_a*(self.accel_bias-self.accel_bias_reference))
        self.last_gyro_body = np.asarray(gyro, dtype=float).copy()

        f = np.zeros((15, 15))
        f[:3, :3] = -_skew(omega_ie+omega_en)
        f[:3, 9:12] = -self.c_nb
        f[3:6, :3] = -_skew(specific_n)
        f[3:6, 12:15] = -self.c_nb
        f[6:9, 3:6] = np.eye(3)
        f[9:12, 9:12] = -np.diag(1.0/tau_g)
        f[12:15, 12:15] = -np.diag(1.0/tau_a)
        phi = np.eye(15)+f*dt
        phi[9:12, 9:12] = np.diag(decay_g)
        phi[12:15, 12:15] = np.diag(decay_a)
        gyro_n = np.radians(_vec(self.config.gyro_arw_deg_sqrt_h, "gyro_arw"))/60.0
        accel_n = _vec(self.config.accel_vrw_m_s_sqrt_h, "accel_vrw")/60.0
        sigma_g = np.radians(
            _vec(self.config.gyro_bias_sigma_deg_h, "gyro_bias_sigma_deg_h")/3600.0)
        sigma_a = _vec(self.config.accel_bias_sigma_mg, "accel_bias_sigma_mg")*G0/1000.0
        q = np.diag(np.r_[gyro_n**2*dt, accel_n**2*dt, np.zeros(3),
                          sigma_g**2*(-np.expm1(-2.0*dt/tau_g)),
                          sigma_a**2*(-np.expm1(-2.0*dt/tau_a))])
        self.p = phi@self.p@phi.T+q
        self.p = 0.5*(self.p+self.p.T)
        self.x = phi@self.x
        self.current_time += dt

    def _measurement_update(self, observation):
        gnss_pos = self._position_from_geodetic(
            observation.lat_deg, observation.lon_deg, observation.height_m)
        lever_b = _vec(self.config.lever_arm_body_m, "lever_arm_body_m")
        lever_n = self.c_nb@lever_b
        omega_corrected = self.last_gyro_body-self.gyro_bias
        predicted_velocity = self.velocity_ned+self.c_nb@np.cross(omega_corrected, lever_b)
        predicted_position = self.position_ned+lever_n
        residual = np.r_[predicted_velocity-observation.velocity_ned_m_s,
                          predicted_position-gnss_pos]
        h = np.zeros((6, 15))
        h[:3, 3:6] = np.eye(3)
        h[3:6, :3] = _skew(lever_n)
        h[3:6, 6:9] = np.eye(3)
        rv, rp = self._measurement_standard_deviation(observation)
        r = np.diag(np.r_[rv*rv, rp*rp])
        s = h@self.p@h.T+r
        k = np.linalg.solve(s, h@self.p).T
        self.x += k@(residual-h@self.x)
        i_kh = np.eye(15)-k@h
        self.p = i_kh@self.p@i_kh.T+k@r@k.T
        self._apply_error_state()

    def _measurement_standard_deviation(self, observation):
        rv = _vec(self.config.default_velocity_measurement_std_m_s,
                  "default_velocity_measurement_std_m_s").copy()
        rp = _vec(self.config.default_position_measurement_std_m,
                  "default_position_measurement_std_m").copy()
        if self.config.use_realtime_gnss_accuracy:
            if math.isfinite(observation.sacc_m_s) and observation.sacc_m_s > 0:
                rv[:] = observation.sacc_m_s
            if math.isfinite(observation.hacc_m) and observation.hacc_m > 0:
                rp[:2] = observation.hacc_m
            if math.isfinite(observation.vacc_m) and observation.vacc_m > 0:
                rp[2] = observation.vacc_m
        return rv, rp

    def _apply_error_state(self):
        attitude_error = self.x[:3].copy()
        self.c_nb = _orthogonalize((np.eye(3)-_skew(attitude_error))@self.c_nb)
        self.velocity_ned -= self.x[3:6]
        self.position_ned -= self.x[6:9]
        self.gyro_bias -= self.x[9:12]
        self.accel_bias -= self.x[12:15]
        # First-order reset Jacobian keeps the covariance consistent with the
        # injected nominal attitude.  Other error components are additive.
        reset = np.eye(15)
        reset[:3, :3] -= 0.5*_skew(attitude_error)
        self.p = reset@self.p@reset.T
        self.p = 0.5*(self.p+self.p.T)
        self.x[:] = 0.0

    def _snapshot(self):
        return _Snapshot(
            float(self.current_time), self.c_nb.copy(), self.position_ned.copy(),
            self.velocity_ned.copy(), self.gyro_bias.copy(), self.accel_bias.copy(),
            self.p.copy(), self.x.copy(), self.last_gyro_body.copy())

    def _restore(self, snapshot):
        snapshot = snapshot.clone()
        self.current_time = snapshot.time_s
        self.c_nb = snapshot.c_nb
        self.position_ned = snapshot.position_ned
        self.velocity_ned = snapshot.velocity_ned
        self.gyro_bias = snapshot.gyro_bias
        self.accel_bias = snapshot.accel_bias
        self.p = snapshot.p
        self.x = snapshot.x
        self.last_gyro_body = snapshot.last_gyro_body

    def _trim_history(self):
        if self.current_time is None:
            return
        oldest_allowed = self.current_time-self.config.navigation_buffer_seconds
        while self._history and self._history[0].sample.gps_time_s < oldest_allowed:
            removed = self._history.popleft()
            self._base_snapshot = removed.after.clone()

    def _position_from_geodetic(self, lat_deg, lon_deg, height):
        lat0, lon0, h0 = self.origin
        rm, rn = _radii(lat0)
        return np.array(((math.radians(lat_deg)-lat0)*(rm+h0),
                         (math.radians(lon_deg)-lon0)*(rn+h0)*math.cos(lat0),
                         h0-height))

    def _geodetic_from_position(self):
        lat0, lon0, h0 = self.origin
        rm, rn = _radii(lat0)
        return (lat0+self.position_ned[0]/(rm+h0),
                lon0+self.position_ned[1]/((rn+h0)*math.cos(lat0)),
                h0-self.position_ned[2])

    def solution(self):
        if self.origin is None or self.current_time is None:
            lat = lon = height = math.nan
            roll = pitch = heading = math.nan
        else:
            lat, lon, height = self._geodetic_from_position()
            roll, pitch, heading = _euler_from_dcm(self.c_nb)
            lat, lon = math.degrees(lat), math.degrees(lon)
            heading = math.degrees(heading) % 360.0
            roll, pitch = math.degrees(roll), math.degrees(pitch)
        std = np.sqrt(np.maximum(np.diag(self.p), 0.0))
        display_std = np.r_[np.degrees(std[:3]), std[3:9],
                            np.degrees(std[9:12]), std[12:15]]
        span = 0.0
        if self._base_snapshot is not None and self.current_time is not None:
            span = max(0.0, self.current_time-self._base_snapshot.time_s)
        elapsed = 0.0 if self.start_time is None or self.current_time is None else max(
            0.0, self.current_time-self.start_time)
        return NavigationSolution(
            self.current_time, elapsed, self.running, self.status, lat, lon, height,
            tuple(self.position_ned), tuple(self.velocity_ned), roll, pitch, heading,
            tuple(np.degrees(self.gyro_bias)), tuple(self.accel_bias),
            tuple(display_std), self.gnss_updates, self.gnss_rejections,
            self.last_gnss_reason, self.last_gnss_delay, self.last_replay_samples,
            self.last_replay_ms, span)
