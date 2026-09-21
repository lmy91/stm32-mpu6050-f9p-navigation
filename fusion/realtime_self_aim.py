"""Real-time GNSS/INS self-alignment and local-level loose coupling.

This independent PC implementation consumes synchronized v3 IMU and F9P
position/velocity records. Accelerometers make roll/pitch observable. The
initial heading is manually bound because an MPU6050 cannot gyrocompass
reliably; fine alignment feeds that binding back as a heading constraint.
"""

from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass, fields
from pathlib import Path

import numpy as np

WGS84_A = 6378137.0
WGS84_E2 = 6.6943799901413165e-3
EARTH_RATE = 7.292115e-5
G0 = 9.80665


def _vec(value, name):
    array = np.asarray(value, dtype=float)
    if array.shape != (3,) or not np.all(np.isfinite(array)):
        raise ValueError(f"{name} 必须是 3 个有限数值")
    return array


def _matrix(value, name):
    array = np.asarray(value, dtype=float)
    if array.shape != (3, 3) or not np.all(np.isfinite(array)):
        raise ValueError(f"{name} 必须是 3x3 有限数值矩阵")
    if not np.allclose(array @ array.T, np.eye(3), atol=1e-6) or np.linalg.det(array) < 0.999:
        raise ValueError(f"{name} 必须是右手正交旋转矩阵")
    return array


@dataclass(frozen=True)
class SelfAimConfig:
    coarse_alignment_seconds: float = 300.0
    minimum_imu_samples: int = 1000
    initial_heading_deg: float = 0.0
    heading_measurement_std_deg: float = 1.0
    maximum_gnss_age_s: float = 2.0
    maximum_hacc_m: float = 20.0
    maximum_sacc_m_s: float = 2.0
    maximum_pdop: float = 8.0
    body_from_sensor: tuple = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    lever_arm_body_m: tuple = (0.0, 0.0, 0.0)
    initial_attitude_std_deg: tuple = (2.0, 2.0, 20.0)
    initial_velocity_std_m_s: tuple = (1.0, 1.0, 1.0)
    initial_position_std_m: tuple = (5.0, 5.0, 8.0)
    initial_gyro_bias_std_deg_h: tuple = (
        10.280829835183116, 9.240629263511165, 3.42212508894922)
    initial_accel_bias_std_mg: tuple = (
        0.04008832679137311, 0.029884809743429882, 0.06729772892931844)
    gyro_arw_deg_sqrt_h: tuple = (
        0.22851654946443578, 0.24634596124624988, 0.2670258750420165)
    accel_vrw_m_s_sqrt_h: tuple = (
        0.0860242233880005, 0.08755342115936089, 0.1253146352362672)
    gyro_bias_sigma_deg_h: tuple = (
        10.280829835183116, 9.240629263511165, 3.42212508894922)
    accel_bias_sigma_mg: tuple = (
        0.04008832679137311, 0.029884809743429882, 0.06729772892931844)
    gyro_bias_correlation_time_s: tuple = (
        28.9698895407, 70.2489629817, 265.2435876699)
    accel_bias_correlation_time_s: tuple = (
        109.3812506785, 1160.8002125689, 70.2471835734)
    use_realtime_gnss_accuracy: bool = True
    default_velocity_measurement_std_m_s: tuple = (0.2, 0.2, 0.3)
    default_position_measurement_std_m: tuple = (3.0, 3.0, 5.0)
    navigation_buffer_seconds: float = 5.0
    output_rate_hz: float = 100.0

    @classmethod
    def load(cls, path: str | Path) -> "SelfAimConfig":
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        if not isinstance(data, dict):
            raise ValueError("自瞄配置根节点必须是 JSON 对象")
        # JSON has no standard comment syntax. Human-readable configuration
        # files use _comment_* string fields, which are deliberately ignored.
        data = {key: value for key, value in data.items()
                if not key.startswith("_comment")}
        unknown = set(data) - {f.name for f in fields(cls)}
        if unknown:
            raise ValueError("未知配置项：" + ", ".join(sorted(unknown)))
        config = cls(**data)
        config.validate()
        return config

    def save(self, path: str | Path, comment_template: str | Path | None = None) -> None:
        """Save standard JSON while retaining optional _comment_* fields."""
        self.validate()
        output = {}
        values = asdict(self)
        if comment_template is not None:
            try:
                template = json.loads(Path(comment_template).read_text(encoding="utf-8"))
                if isinstance(template, dict):
                    for key, value in template.items():
                        if key.startswith("_comment"):
                            output[key] = value
                        elif key in values:
                            output[key] = values.pop(key)
            except (OSError, ValueError, json.JSONDecodeError):
                pass
        output.update(values)
        Path(path).write_text(json.dumps(output, ensure_ascii=False, indent=2) + "\n",
                              encoding="utf-8")

    def validate(self):
        positive = ("coarse_alignment_seconds",
                    "heading_measurement_std_deg", "maximum_gnss_age_s", "maximum_hacc_m",
                    "maximum_sacc_m_s", "maximum_pdop",
                    "navigation_buffer_seconds", "output_rate_hz")
        for name in positive:
            if not math.isfinite(float(getattr(self, name))) or float(getattr(self, name)) < 0:
                raise ValueError(f"{name} 必须是非负有限数值")
        if self.minimum_imu_samples < 2 or self.output_rate_hz <= 0:
            raise ValueError("minimum_imu_samples 至少为 2，output_rate_hz 必须大于 0")
        if not math.isfinite(float(self.initial_heading_deg)):
            raise ValueError("initial_heading_deg 必须是手动装订的有限数值")
        if self.heading_measurement_std_deg <= 0:
            raise ValueError("heading_measurement_std_deg 必须大于 0")
        if self.navigation_buffer_seconds < self.maximum_gnss_age_s:
            raise ValueError("navigation_buffer_seconds 不能小于 maximum_gnss_age_s")
        _matrix(self.body_from_sensor, "body_from_sensor")
        if not isinstance(self.use_realtime_gnss_accuracy, bool):
            raise ValueError("use_realtime_gnss_accuracy 必须是布尔值")
        for name in ("lever_arm_body_m", "initial_attitude_std_deg",
                     "initial_velocity_std_m_s", "initial_position_std_m",
                     "initial_gyro_bias_std_deg_h", "initial_accel_bias_std_mg",
                     "default_velocity_measurement_std_m_s",
                     "default_position_measurement_std_m",
                     "gyro_arw_deg_sqrt_h", "accel_vrw_m_s_sqrt_h",
                     "gyro_bias_sigma_deg_h", "accel_bias_sigma_mg"):
            if np.any(_vec(getattr(self, name), name) < 0):
                raise ValueError(f"{name} 不能为负数")
        for name in ("gyro_bias_correlation_time_s", "accel_bias_correlation_time_s"):
            if np.any(_vec(getattr(self, name), name) <= 0):
                raise ValueError(f"{name} 必须全部大于 0")
        for name in ("default_velocity_measurement_std_m_s",
                     "default_position_measurement_std_m"):
            if np.any(_vec(getattr(self, name), name) <= 0):
                raise ValueError(f"{name} 必须全部大于 0")


@dataclass
class GnssObservation:
    gps_time_s: float
    lat_deg: float
    lon_deg: float
    height_m: float
    velocity_ned_m_s: np.ndarray
    valid: bool
    hacc_m: float
    sacc_m_s: float
    pdop: float
    vacc_m: float = math.nan


@dataclass
class AimSolution:
    gps_time_s: float | None
    elapsed_s: float
    stage: str
    progress: float
    roll_deg: float
    pitch_deg: float
    heading_deg: float
    heading_valid: bool
    fine_initial_attitude_deg: tuple
    fine_initial_velocity_ned_m_s: tuple
    fine_initial_position_geodetic: tuple
    fine_initial_position_samples: int
    lat_deg: float
    lon_deg: float
    height_m: float
    position_ned_m: tuple
    velocity_ned_m_s: tuple
    gyro_bias_deg_s: tuple
    accel_bias_m_s2: tuple
    covariance_std: tuple
    gnss_updates: int
    gnss_rejections: int
    last_gnss_reason: str
    time_match_s: float


def _skew(v):
    x, y, z = v
    return np.array(((0.0, -z, y), (z, 0.0, -x), (-y, x, 0.0)))


def _radii(lat):
    s = math.sin(lat)
    root = math.sqrt(1.0 - WGS84_E2 * s * s)
    return WGS84_A * (1.0 - WGS84_E2) / root**3, WGS84_A / root


def _dcm_from_euler(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                               math.sin(pitch), math.cos(yaw), math.sin(yaw))
    return np.array(((cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr),
                     (sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr),
                     (-sp, cp*sr, cp*cr)))


def _euler_from_dcm(c):
    pitch = math.asin(float(np.clip(-c[2, 0], -1.0, 1.0)))
    roll = math.atan2(c[2, 1], c[2, 2])
    yaw = math.atan2(c[1, 0], c[0, 0])
    return roll, pitch, yaw


def _orthogonalize(c):
    u, _, vh = np.linalg.svd(c)
    result = u @ vh
    if np.linalg.det(result) < 0:
        u[:, -1] *= -1
        result = u @ vh
    return result


class RealtimeSelfAim:
    """Stateful 100 Hz mechanization with 1 Hz position/velocity aiding."""

    def __init__(self, config: SelfAimConfig):
        config.validate()
        self.config = config
        self.sensor_to_body = _matrix(config.body_from_sensor, "body_from_sensor")
        self.reset()

    def reset(self):
        self.running = False
        self.stage = "未启动"
        self.start_time = self.current_time = None
        self.fine_start_time = None
        self.coarse_accel = np.zeros(3)
        self.coarse_gyro = np.zeros(3)
        self.coarse_count = 0
        self.heading = math.radians(self.config.initial_heading_deg % 360.0)
        self.fine_initial_attitude_deg = (math.nan, math.nan, math.nan)
        self.fine_initial_velocity_ned_m_s = (math.nan, math.nan, math.nan)
        self.fine_initial_position_geodetic = (math.nan, math.nan, math.nan)
        self.fine_initial_position_samples = 0
        self.coarse_gnss_position_sum = np.zeros(3)
        self.coarse_gnss_position_count = 0
        self.latest_gnss = None
        self.origin = None
        self.c_nb = np.eye(3)
        self.position_ned = np.zeros(3)
        self.velocity_ned = np.zeros(3)
        self.gyro_bias = np.zeros(3)
        self.accel_bias = np.zeros(3)
        self.gyro_bias_reference = np.zeros(3)
        self.accel_bias_reference = np.zeros(3)
        attitude_std = np.radians(_vec(
            self.config.initial_attitude_std_deg, "initial_attitude_std_deg"))
        velocity_std = _vec(self.config.initial_velocity_std_m_s,
                            "initial_velocity_std_m_s")
        position_std = _vec(self.config.initial_position_std_m,
                            "initial_position_std_m")
        gyro_bias_std = np.radians(
            _vec(self.config.initial_gyro_bias_std_deg_h,
                 "initial_gyro_bias_std_deg_h") / 3600.0)
        accel_bias_std = (_vec(self.config.initial_accel_bias_std_mg,
                               "initial_accel_bias_std_mg") * G0 / 1000.0)
        self.p = np.diag(np.r_[attitude_std**2, velocity_std**2,
                               position_std**2,
                               gyro_bias_std**2, accel_bias_std**2])
        self.x = np.zeros(15)
        self.gnss_updates = self.gnss_rejections = 0
        self.last_gnss_reason = "尚无数据"
        self.last_time_match = math.nan

    def start(self):
        self.reset()
        self.running = True
        self.stage = "等待同步数据"

    def stop(self):
        self.running = False
        self.stage = "已停止"

    def start_fine_alignment(self):
        """Manually finish coarse alignment with the latest valid input."""
        if not self.running:
            self.last_gnss_reason = "自瞄尚未开始"
            return False
        if self.stage != "粗对准":
            self.last_gnss_reason = "只有粗对准阶段可进入精对准"
            return False
        return self._try_initialize(force=True)

    def _gnss_good(self, obs):
        if not obs.valid:
            return False, "GNSS解无效"
        if not all(math.isfinite(v) for v in (obs.gps_time_s, obs.lat_deg, obs.lon_deg,
                                               obs.height_m, obs.hacc_m, obs.sacc_m_s, obs.pdop)):
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

    def _measurement_standard_deviation(self, observation):
        """Return velocity and position 1-sigma vectors used to build R."""
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

    def update_gnss(self, observation: GnssObservation):
        observation.velocity_ned_m_s = _vec(observation.velocity_ned_m_s, "GNSS速度")
        good, reason = self._gnss_good(observation)
        self.latest_gnss = observation
        if self.running and self.origin is None and good:
            self.coarse_gnss_position_sum += np.array(
                (observation.lat_deg, observation.lon_deg, observation.height_m))
            self.coarse_gnss_position_count += 1
        if not self.running or self.origin is None:
            self.last_gnss_reason = reason
            return False
        if not good:
            self.gnss_rejections += 1
            self.last_gnss_reason = reason
            return False
        age = 0.0 if self.current_time is None else self.current_time - observation.gps_time_s
        self.last_time_match = age
        if age < -0.25 or age > self.config.maximum_gnss_age_s:
            self.gnss_rejections += 1
            self.last_gnss_reason = f"时间差 {age:+.3f}s超限"
            return False
        gnss_pos = self._position_from_geodetic(observation.lat_deg, observation.lon_deg,
                                                observation.height_m)
        gnss_pos = gnss_pos + observation.velocity_ned_m_s * max(age, 0.0)
        lever_n = self.c_nb @ _vec(self.config.lever_arm_body_m, "lever_arm_body_m")
        predicted_pos = self.position_ned + lever_n
        residual = np.r_[self.velocity_ned - observation.velocity_ned_m_s,
                         predicted_pos - gnss_pos]
        fine = self.stage == "精对准"
        measurement_count = 7 if fine else 6
        h = np.zeros((measurement_count, 15))
        h[:3, 3:6] = np.eye(3); h[3:6, 6:9] = np.eye(3)
        rv, rp = self._measurement_standard_deviation(observation)
        measurement = residual
        variances = np.r_[rv*rv, rp*rp]
        if fine:
            # x[:3] is the navigation-frame attitude error and feedback uses
            # C <- (I-skew(x))C. Therefore predicted-minus-bound heading is
            # the correct residual sign: a positive yaw error produces a
            # positive x_z and is subtracted by _apply_error_state().
            yaw = _euler_from_dcm(self.c_nb)[2]
            bound = math.radians(self.config.initial_heading_deg % 360.0)
            heading_error = math.atan2(math.sin(yaw-bound), math.cos(yaw-bound))
            h[6, 2] = 1.0
            measurement = np.r_[residual, heading_error]
            variances = np.r_[variances,
                              math.radians(self.config.heading_measurement_std_deg)**2]
        r = np.diag(variances)
        s = h @ self.p @ h.T + r
        k = np.linalg.solve(s, h @ self.p).T
        self.x += k @ (measurement - h @ self.x)
        i_kh = np.eye(15) - k @ h
        self.p = i_kh @ self.p @ i_kh.T + k @ r @ k.T
        self._apply_error_state()
        self.gnss_updates += 1
        self.last_gnss_reason = "位置/速度/航向更新" if fine else "位置/速度更新"
        return True

    def update_imu(self, gps_time_s, accel_sensor_m_s2, gyro_sensor_rad_s, dt_s):
        if not self.running:
            return self.solution()
        if not (math.isfinite(dt_s) and 0.001 <= dt_s <= 0.1):
            self.last_gnss_reason = f"IMU dt {dt_s!r}无效"
            return self.solution()
        if gps_time_s is not None and math.isfinite(gps_time_s):
            self.current_time = float(gps_time_s)
        elif self.current_time is not None:
            self.current_time += dt_s
        else:
            return self.solution()
        if self.start_time is None:
            self.start_time = self.current_time
        accel = self.sensor_to_body @ _vec(accel_sensor_m_s2, "IMU加速度")
        gyro = self.sensor_to_body @ _vec(gyro_sensor_rad_s, "IMU角速度")
        if self.origin is None:
            self.stage = "粗对准"
            self.coarse_accel += accel
            self.coarse_gyro += gyro
            self.coarse_count += 1
            self._update_coarse_estimate()
            self._try_initialize(force=False)
        else:
            self._mechanize(accel, gyro, dt_s)
        return self.solution()

    def _update_coarse_estimate(self):
        """Expose the running 15 physical states throughout coarse alignment."""
        mean_a = self.coarse_accel / max(self.coarse_count, 1)
        if np.linalg.norm(mean_a) < 1.0:
            return
        roll = math.atan2(-mean_a[1], -mean_a[2])
        pitch = math.atan2(mean_a[0], math.hypot(mean_a[1], mean_a[2]))
        self.c_nb = _dcm_from_euler(roll, pitch, self.heading)
        mean_g = self.coarse_gyro / max(self.coarse_count, 1)
        lat = math.radians(self.latest_gnss.lat_deg) if self.latest_gnss else 0.0
        earth_n = np.array((EARTH_RATE*math.cos(lat), 0.0, -EARTH_RATE*math.sin(lat)))
        self.gyro_bias = mean_g - self.c_nb.T @ earth_n
        gravity_b = self.c_nb.T @ np.array((0.0, 0.0, -G0))
        self.accel_bias = mean_a - gravity_b
        # Treat the observed turn-on bias as the non-zero mean of the GM
        # process. Filter corrections then describe correlated variations
        # around this calibrated value instead of unrealistically forcing the
        # complete MPU6050 turn-on bias toward zero.
        self.gyro_bias_reference = self.gyro_bias.copy()
        self.accel_bias_reference = self.accel_bias.copy()

    def _try_initialize(self, force=False):
        elapsed = self._elapsed()
        if self.latest_gnss is None:
            self.last_gnss_reason = "等待有效GNSS位置"
            return False
        good, reason = self._gnss_good(self.latest_gnss)
        if not good:
            self.last_gnss_reason = reason
            return False
        age = self.current_time - self.latest_gnss.gps_time_s
        if age < -0.25 or age > self.config.maximum_gnss_age_s:
            self.last_gnss_reason = f"等待当前GNSS：最近位置时间差 {age:+.3f}s"
            return False
        if force:
            if self.coarse_count < 2:
                self.last_gnss_reason = "手动精对准至少需要2个有效IMU样本"
                return False
        elif elapsed < self.config.coarse_alignment_seconds or self.coarse_count < self.config.minimum_imu_samples:
            return False
        mean_a = self.coarse_accel / self.coarse_count
        norm_a = np.linalg.norm(mean_a)
        if norm_a < 1.0:
            self.last_gnss_reason = "重力矢量不足"
            return False
        roll = math.atan2(-mean_a[1], -mean_a[2])
        pitch = math.atan2(mean_a[0], math.hypot(mean_a[1], mean_a[2]))
        self.c_nb = _dcm_from_euler(roll, pitch, self.heading)
        # Freeze all three coarse-alignment angles as the initial attitude of
        # the fine-alignment mechanization. Subsequent updates evolve c_nb;
        # this tuple remains an auditable record of the hand-over state.
        self.fine_initial_attitude_deg = (
            math.degrees(roll), math.degrees(pitch), math.degrees(self.heading))
        self.fine_initial_velocity_ned_m_s = (0.0, 0.0, 0.0)
        if self.coarse_gnss_position_count <= 0:
            self.last_gnss_reason = "粗对准期间没有有效GNSS位置"
            return False
        mean_position = self.coarse_gnss_position_sum / self.coarse_gnss_position_count
        self.fine_initial_position_geodetic = tuple(mean_position)
        self.fine_initial_position_samples = self.coarse_gnss_position_count
        self._update_coarse_estimate()
        self.origin = (math.radians(mean_position[0]),
                       math.radians(mean_position[1]), mean_position[2])
        self.position_ned[:] = 0.0
        self.velocity_ned[:] = 0.0
        self.fine_start_time = self.current_time
        self.stage = "精对准"
        mode = "手动切换" if force else "自动切换"
        self.last_gnss_reason = f"粗对准完成（{mode}），航向=手动装订"
        return True

    def _mechanize(self, accel, gyro, dt):
        tau_g = _vec(self.config.gyro_bias_correlation_time_s,
                     "gyro_bias_correlation_time_s")
        tau_a = _vec(self.config.accel_bias_correlation_time_s,
                     "accel_bias_correlation_time_s")
        decay_g = np.exp(-dt / tau_g); decay_a = np.exp(-dt / tau_a)
        lat, _, height = self._geodetic_from_position()
        rm, rn = _radii(lat)
        vn, ve, _ = self.velocity_ned
        omega_ie = np.array((EARTH_RATE*math.cos(lat), 0.0, -EARTH_RATE*math.sin(lat)))
        omega_en = np.array((ve/(rn+height), -vn/(rm+height),
                             -ve*math.tan(lat)/(rn+height)))
        omega_nb_b = gyro - self.gyro_bias - self.c_nb.T @ (omega_ie + omega_en)
        self.c_nb = _orthogonalize(self.c_nb @ (np.eye(3) + _skew(omega_nb_b*dt)))
        specific_n = self.c_nb @ (accel - self.accel_bias)
        self.velocity_ned += (specific_n + np.array((0.0, 0.0, G0))
                              - np.cross(2*omega_ie + omega_en, self.velocity_ned)) * dt
        self.position_ned += self.velocity_ned * dt
        self.gyro_bias = (self.gyro_bias_reference +
                          decay_g * (self.gyro_bias-self.gyro_bias_reference))
        self.accel_bias = (self.accel_bias_reference +
                           decay_a * (self.accel_bias-self.accel_bias_reference))
        f = np.zeros((15, 15))
        f[:3, :3] = -_skew(omega_ie + omega_en)
        f[:3, 9:12] = -self.c_nb
        f[3:6, :3] = -_skew(specific_n)
        # Bias error states use estimated-minus-true convention, matching
        # velocity/position error states and the closed-loop subtraction in
        # _apply_error_state(). A positive estimated accelerometer-bias error
        # therefore produces a negative velocity-error derivative.
        f[3:6, 12:15] = -self.c_nb
        f[6:9, 3:6] = np.eye(3)
        f[9:12, 9:12] = -np.diag(1.0 / tau_g)
        f[12:15, 12:15] = -np.diag(1.0 / tau_a)
        phi = np.eye(15) + f*dt
        phi[9:12, 9:12] = np.diag(decay_g)
        phi[12:15, 12:15] = np.diag(decay_a)
        gyro_n = np.radians(_vec(self.config.gyro_arw_deg_sqrt_h, "gyro_arw")) / 60.0
        accel_n = _vec(self.config.accel_vrw_m_s_sqrt_h, "accel_vrw") / 60.0
        sigma_g = np.radians(
            _vec(self.config.gyro_bias_sigma_deg_h, "gyro_bias_sigma_deg_h") / 3600.0)
        sigma_a = (_vec(self.config.accel_bias_sigma_mg,
                        "accel_bias_sigma_mg") * G0 / 1000.0)
        # Exact discrete driving covariance for a stationary first-order GM
        # process.  expm1 keeps 1-exp(-2dt/tau) accurate when dt << tau.
        gm_fraction_g = -np.expm1(-2.0*dt/tau_g)
        gm_fraction_a = -np.expm1(-2.0*dt/tau_a)
        q = np.diag(np.r_[gyro_n**2*dt, accel_n**2*dt, np.zeros(3),
                          sigma_g**2*gm_fraction_g,
                          sigma_a**2*gm_fraction_a])
        self.p = phi @ self.p @ phi.T + q
        self.x = phi @ self.x

    def _apply_error_state(self):
        """Feed KF errors into the nominal INS state, then reset the errors."""
        self.c_nb = _orthogonalize((np.eye(3) - _skew(self.x[:3])) @ self.c_nb)
        self.velocity_ned -= self.x[3:6]
        self.position_ned -= self.x[6:9]
        # x_bias = estimated bias - true bias. Subtracting it closes the
        # loop; _mechanize() uses the corrected estimates for every following
        # gyro and accelerometer sample.
        self.gyro_bias -= self.x[9:12]
        self.accel_bias -= self.x[12:15]
        self.x[:] = 0.0

    def _position_from_geodetic(self, lat_deg, lon_deg, height):
        lat0, lon0, h0 = self.origin
        rm, rn = _radii(lat0)
        return np.array(((math.radians(lat_deg)-lat0)*(rm+h0),
                         (math.radians(lon_deg)-lon0)*(rn+h0)*math.cos(lat0),
                         h0-height))

    def _geodetic_from_position(self):
        if self.origin is None:
            return math.nan, math.nan, math.nan
        lat0, lon0, h0 = self.origin
        rm, rn = _radii(lat0)
        return (lat0+self.position_ned[0]/(rm+h0),
                lon0+self.position_ned[1]/((rn+h0)*math.cos(lat0)),
                h0-self.position_ned[2])

    def _elapsed(self):
        if self.start_time is None or self.current_time is None:
            return 0.0
        return max(0.0, self.current_time-self.start_time)

    def _fine_elapsed(self):
        if self.fine_start_time is None or self.current_time is None:
            return 0.0
        return max(0.0, self.current_time-self.fine_start_time)

    def solution(self):
        elapsed = self._elapsed()
        if self.origin is None:
            mean = self.coarse_accel/max(self.coarse_count, 1)
            roll = math.atan2(-mean[1], -mean[2]) if np.linalg.norm(mean) else math.nan
            pitch = math.atan2(mean[0], math.hypot(mean[1], mean[2])) if np.linalg.norm(mean) else math.nan
            lat = math.radians(self.latest_gnss.lat_deg) if self.latest_gnss else math.nan
            lon = math.radians(self.latest_gnss.lon_deg) if self.latest_gnss else math.nan
            height = self.latest_gnss.height_m if self.latest_gnss else math.nan
        else:
            roll, pitch, yaw = _euler_from_dcm(self.c_nb)
            self.heading = yaw % (2*math.pi)
            lat, lon, height = self._geodetic_from_position()
        if self.stage in ("等待同步数据", "粗对准"):
            progress = min(1.0, elapsed/max(self.config.coarse_alignment_seconds, 1e-9))
        elif self.stage == "精对准":
            progress = 1.0
        else:
            progress = 1.0
        std = np.sqrt(np.maximum(np.diag(self.p), 0.0))
        display_std = np.r_[np.degrees(std[:3]), std[3:9],
                            np.degrees(std[9:12]), std[12:15]]
        return AimSolution(
            self.current_time, elapsed, self.stage, progress, math.degrees(roll),
            math.degrees(pitch), math.nan if self.heading is None else math.degrees(self.heading),
            self.heading is not None, self.fine_initial_attitude_deg,
            self.fine_initial_velocity_ned_m_s, self.fine_initial_position_geodetic,
            self.fine_initial_position_samples,
            math.degrees(lat), math.degrees(lon), height,
            tuple(self.position_ned),
            tuple(self.velocity_ned), tuple(np.degrees(self.gyro_bias)),
            tuple(self.accel_bias), tuple(display_std), self.gnss_updates,
            self.gnss_rejections, self.last_gnss_reason, self.last_time_match)
