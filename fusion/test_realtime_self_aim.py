import math
import json
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path

import numpy as np

from fusion import GnssObservation, RealtimeSelfAim, SelfAimConfig


class RealtimeSelfAimTests(unittest.TestCase):
    def test_latest_allan_defaults_and_initial_bias_covariance(self):
        config = SelfAimConfig()
        self.assertTrue(np.allclose(config.gyro_arw_deg_sqrt_h,
                                    (0.22851655, 0.24634596, 0.26702588)))
        self.assertTrue(np.allclose(config.accel_vrw_m_s_sqrt_h,
                                    (0.08602422, 0.08755342, 0.12531464)))
        self.assertEqual(config.output_rate_hz, 100.0)
        engine = RealtimeSelfAim(config)
        gyro_sigma = np.radians(np.asarray(config.initial_gyro_bias_std_deg_h) / 3600.0)
        accel_sigma = np.asarray(config.initial_accel_bias_std_mg) * 9.80665 / 1000.0
        self.assertTrue(np.allclose(np.diag(engine.p)[9:12], gyro_sigma**2))
        self.assertTrue(np.allclose(np.diag(engine.p)[12:15], accel_sigma**2))

    def test_gauss_markov_covariance_is_stationary_and_bias_decays_to_reference(self):
        config = SelfAimConfig()
        engine = RealtimeSelfAim(config)
        engine.origin = (math.radians(30.0), math.radians(114.0), 20.0)
        initial_bias_variance = np.diag(engine.p)[9:15].copy()
        engine.gyro_bias_reference = np.array((0.001, -0.002, 0.003))
        engine.accel_bias_reference = np.array((0.01, -0.02, 0.03))
        gyro_offset = np.array((0.004, -0.005, 0.006))
        accel_offset = np.array((0.04, -0.05, 0.06))
        engine.gyro_bias = engine.gyro_bias_reference + gyro_offset
        engine.accel_bias = engine.accel_bias_reference + accel_offset
        dt = 0.01
        engine._mechanize((0.0, 0.0, -9.80665), (0.0, 0.0, 0.0), dt)
        decay_g = np.exp(-dt / np.asarray(config.gyro_bias_correlation_time_s))
        decay_a = np.exp(-dt / np.asarray(config.accel_bias_correlation_time_s))
        self.assertTrue(np.allclose(engine.gyro_bias-engine.gyro_bias_reference,
                                    decay_g*gyro_offset))
        self.assertTrue(np.allclose(engine.accel_bias-engine.accel_bias_reference,
                                    decay_a*accel_offset))
        self.assertTrue(np.allclose(np.diag(engine.p)[9:15], initial_bias_variance))

    def test_sensor_bias_error_is_closed_loop_subtracted(self):
        engine = RealtimeSelfAim(SelfAimConfig())
        engine.gyro_bias = np.array((0.01, -0.02, 0.03))
        engine.accel_bias = np.array((0.1, -0.2, 0.3))
        gyro_error = np.array((0.001, -0.002, 0.003))
        accel_error = np.array((0.01, -0.02, 0.03))
        engine.x[9:12] = gyro_error
        engine.x[12:15] = accel_error
        engine._apply_error_state()
        self.assertTrue(np.allclose(engine.gyro_bias,
                                    np.array((0.01, -0.02, 0.03))-gyro_error))
        self.assertTrue(np.allclose(engine.accel_bias,
                                    np.array((0.1, -0.2, 0.3))-accel_error))
        self.assertTrue(np.array_equal(engine.x, np.zeros(15)))

    def test_mechanization_uses_feedback_bias_compensation(self):
        engine = RealtimeSelfAim(SelfAimConfig())
        latitude = math.radians(30.0)
        engine.origin = (latitude, math.radians(114.0), 20.0)
        engine.c_nb = np.eye(3)
        engine.gyro_bias = engine.gyro_bias_reference = np.array((0.01, -0.02, 0.03))
        engine.accel_bias = engine.accel_bias_reference = np.array((0.1, -0.2, 0.3))
        earth_rate_n = np.array((7.292115e-5*math.cos(latitude), 0.0,
                                 -7.292115e-5*math.sin(latitude)))
        gyro_measurement = engine.gyro_bias + earth_rate_n
        accel_measurement = engine.accel_bias + np.array((0.0, 0.0, -9.80665))
        engine._mechanize(accel_measurement, gyro_measurement, 0.01)
        self.assertTrue(np.allclose(engine.c_nb, np.eye(3), atol=1e-12))
        self.assertTrue(np.allclose(engine.velocity_ned, np.zeros(3), atol=1e-12))

    def test_zero_gauss_markov_correlation_time_is_rejected(self):
        config = replace(SelfAimConfig(), gyro_bias_correlation_time_s=(1.0, 0.0, 1.0))
        with self.assertRaises(ValueError):
            RealtimeSelfAim(config)

    def make_engine(self):
        config = replace(
            SelfAimConfig(), coarse_alignment_seconds=0.02,
            minimum_imu_samples=3, initial_heading_deg=90.0)
        engine = RealtimeSelfAim(config); engine.start()
        engine.update_gnss(GnssObservation(
            100.0, 30.0, 114.0, 20.0, np.zeros(3), True, 0.02, 0.03, 1.2))
        for index in range(10):
            engine.update_imu(100.0 + index * 0.01, (0.0, 0.0, -9.80665),
                              (0.0, 0.0, 0.0), 0.01)
        return engine

    def test_coarse_to_fine_state(self):
        solution = self.make_engine().solution()
        self.assertEqual(solution.stage, "精对准")
        self.assertAlmostEqual(solution.roll_deg, 0.0, places=2)
        self.assertAlmostEqual(solution.pitch_deg, 0.0, places=2)
        self.assertAlmostEqual(solution.heading_deg, 90.0, places=2)
        self.assertTrue(solution.heading_valid)
        self.assertAlmostEqual(solution.fine_initial_attitude_deg[0], 0.0, places=2)
        self.assertAlmostEqual(solution.fine_initial_attitude_deg[1], 0.0, places=2)
        self.assertAlmostEqual(solution.fine_initial_attitude_deg[2], 90.0, places=2)
        self.assertEqual(solution.fine_initial_velocity_ned_m_s, (0.0, 0.0, 0.0))
        self.assertEqual(solution.fine_initial_position_samples, 1)
        self.assertTrue(np.allclose(solution.position_ned_m, (0.0, 0.0, 0.0)))
        self.assertEqual(len(solution.covariance_std), 15)

    def test_manual_fine_alignment_uses_mean_gnss_position_and_zero_velocity(self):
        config = replace(SelfAimConfig(), coarse_alignment_seconds=300.0,
                         minimum_imu_samples=1000, initial_heading_deg=45.0)
        engine = RealtimeSelfAim(config); engine.start()
        for tow, lat, lon, height in ((100.0, 30.0, 114.0, 20.0),
                                      (100.01, 30.2, 114.4, 24.0)):
            engine.update_gnss(GnssObservation(
                tow, lat, lon, height, np.array((1.0, 2.0, 3.0)), True, 0.1, 0.1, 1.0))
            engine.update_imu(tow, (0.0, 0.0, -9.80665), (0.0, 0.0, 0.0), 0.01)
        self.assertTrue(engine.start_fine_alignment())
        solution = engine.solution()
        self.assertEqual(solution.stage, "精对准")
        self.assertEqual(solution.fine_initial_velocity_ned_m_s, (0.0, 0.0, 0.0))
        self.assertEqual(solution.fine_initial_position_samples, 2)
        self.assertTrue(np.allclose(solution.fine_initial_position_geodetic,
                                    (30.1, 114.2, 22.0)))

    def test_manual_heading_is_not_overwritten_by_gnss_course(self):
        config = replace(SelfAimConfig(), coarse_alignment_seconds=300.0,
                         initial_heading_deg=90.0)
        engine = RealtimeSelfAim(config); engine.start()
        engine.update_gnss(GnssObservation(
            100.08, 30.0, 114.0, 20.0, np.array((0.0, 2.0, 0.0)),
            True, 0.02, 0.03, 1.2))
        self.assertAlmostEqual(engine.config.initial_heading_deg, 90.0)
        self.assertAlmostEqual(math.degrees(engine.heading), 90.0)

    def test_fine_alignment_compensates_heading_error(self):
        config = replace(SelfAimConfig(), coarse_alignment_seconds=300.0,
                         initial_heading_deg=90.0,
                         heading_measurement_std_deg=0.1)
        engine = RealtimeSelfAim(config); engine.start()
        observation = GnssObservation(
            100.0, 30.0, 114.0, 20.0, np.zeros(3), True, 0.02, 0.03, 1.2)
        engine.update_gnss(observation)
        engine.update_imu(100.0, (0, 0, -9.80665), (0, 0, 0), 0.01)
        engine.update_imu(100.01, (0, 0, -9.80665), (0, 0, 0), 0.01)
        self.assertTrue(engine.start_fine_alignment())
        error = math.radians(10.0)
        engine.c_nb = engine.c_nb @ np.array(
            ((math.cos(error), -math.sin(error), 0.0),
             (math.sin(error), math.cos(error), 0.0), (0.0, 0.0, 1.0)))
        before = engine.solution().heading_deg
        observation.gps_time_s = 100.01
        engine.update_gnss(observation)
        after = engine.solution().heading_deg
        self.assertLess(abs(after - 90.0), abs(before - 90.0))

    def test_fine_alignment_has_no_automatic_navigation_transition(self):
        engine = self.make_engine()
        for index in range(10000):
            engine.update_imu(101.0 + index*0.01, (0, 0, -9.80665), (0, 0, 0), 0.01)
        self.assertTrue(engine.running)
        self.assertEqual(engine.solution().stage, "精对准")
        engine.stop()
        self.assertFalse(engine.running)
        self.assertEqual(engine.solution().stage, "已停止")

    def test_realtime_gnss_accuracy_builds_r_standard_deviations(self):
        engine = RealtimeSelfAim(SelfAimConfig())
        observation = GnssObservation(
            100.0, 30.0, 114.0, 20.0, np.zeros(3), True,
            hacc_m=0.03, sacc_m_s=0.04, pdop=1.0, vacc_m=0.07)
        velocity, position = engine._measurement_standard_deviation(observation)
        self.assertTrue(np.allclose(velocity, (0.04, 0.04, 0.04)))
        self.assertTrue(np.allclose(position, (0.03, 0.03, 0.07)))

    def test_fixed_and_invalid_realtime_accuracy_use_defaults(self):
        defaults = replace(
            SelfAimConfig(), use_realtime_gnss_accuracy=False,
            default_velocity_measurement_std_m_s=(1.0, 2.0, 3.0),
            default_position_measurement_std_m=(4.0, 5.0, 6.0))
        observation = GnssObservation(
            100.0, 30.0, 114.0, 20.0, np.zeros(3), True,
            hacc_m=0.03, sacc_m_s=0.04, pdop=1.0, vacc_m=0.07)
        velocity, position = RealtimeSelfAim(defaults)._measurement_standard_deviation(observation)
        self.assertTrue(np.allclose(velocity, (1.0, 2.0, 3.0)))
        self.assertTrue(np.allclose(position, (4.0, 5.0, 6.0)))
        realtime = replace(defaults, use_realtime_gnss_accuracy=True)
        invalid = replace(observation, hacc_m=0.0, sacc_m_s=math.nan, vacc_m=-1.0)
        velocity, position = RealtimeSelfAim(realtime)._measurement_standard_deviation(invalid)
        self.assertTrue(np.allclose(velocity, (1.0, 2.0, 3.0)))
        self.assertTrue(np.allclose(position, (4.0, 5.0, 6.0)))

    def test_ned_down_has_negative_height_sign(self):
        engine = self.make_engine()
        engine.position_ned[2] = 5.0
        self.assertAlmostEqual(engine.solution().height_m, 15.0, places=6)

    def test_old_gnss_observation_is_rejected(self):
        engine = self.make_engine()
        accepted = engine.update_gnss(GnssObservation(
            90.0, 30.0, 114.0, 20.0, np.zeros(3), True, 0.02, 0.03, 1.2))
        self.assertFalse(accepted)
        self.assertIn("时间差", engine.solution().last_gnss_reason)

    def test_fresh_gnss_resumes_after_outage_without_innovation_gate(self):
        engine = self.make_engine()
        for index in range(500):
            engine.update_imu(101.0 + index*0.01, (0, 0, -9.80665),
                              (0, 0, 0), 0.01)
        self.assertEqual(engine.solution().stage, "精对准")
        updates_before = engine.gnss_updates
        # Deliberately large position/velocity residuals verify that only the
        # remaining GNSS validity/age/accuracy checks control acceptance.
        accepted = engine.update_gnss(GnssObservation(
            engine.current_time, 30.01, 114.02, 25.0,
            np.array((50.0, -40.0, 10.0)), True,
            hacc_m=0.02, sacc_m_s=0.03, pdop=1.2, vacc_m=0.04))
        self.assertTrue(accepted)
        self.assertEqual(engine.gnss_updates, updates_before + 1)
        self.assertEqual(engine.solution().stage, "精对准")
        self.assertIn("更新", engine.solution().last_gnss_reason)

    def test_invalid_rotation_matrix_is_rejected(self):
        config = replace(SelfAimConfig(), body_from_sensor=((1, 0, 0), (0, 1, 0), (0, 0, -1)))
        with self.assertRaises(ValueError):
            RealtimeSelfAim(config)

    def test_comment_fields_are_ignored(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.json"
            path.write_text(json.dumps({"_comment_heading": "说明",
                                        "initial_heading_deg": 12.5}), encoding="utf-8")
            config = SelfAimConfig.load(path)
            self.assertEqual(config.initial_heading_deg, 12.5)

    def test_save_retains_comment_fields(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "source.json"
            target = Path(directory) / "saved.json"
            source.write_text('{"_comment_timing":"说明","output_rate_hz":1}', encoding="utf-8")
            config = replace(SelfAimConfig(), output_rate_hz=20.0)
            config.save(target, source)
            saved = json.loads(target.read_text(encoding="utf-8"))
            self.assertEqual(saved["_comment_timing"], "说明")
            self.assertEqual(saved["output_rate_hz"], 20.0)


if __name__ == "__main__":
    unittest.main()
