import math
import unittest
from dataclasses import replace

import numpy as np

from fusion import (
    GnssObservation,
    ImuSample,
    NavigationInitialState,
    RealtimeLooseNavigation,
    SelfAimConfig,
)


class RealtimeLooseNavigationTests(unittest.TestCase):
    @staticmethod
    def initial_state(time_s=100.0):
        return NavigationInitialState(
            time_s, (math.radians(30.0), math.radians(114.0), 20.0),
            np.eye(3), np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3),
            np.zeros(3), np.zeros(3), np.diag(np.linspace(0.01, 0.15, 15)))

    @staticmethod
    def stationary_sample(sequence, time_s):
        latitude = math.radians(30.0)
        earth_rate_n = (7.292115e-5*math.cos(latitude), 0.0,
                        -7.292115e-5*math.sin(latitude))
        return ImuSample(sequence, time_s, (0.0, 0.0, -9.80665),
                         earth_rate_n, 0.01)

    @staticmethod
    def observation(time_s):
        return GnssObservation(
            time_s, 30.0, 114.0, 20.0, np.zeros(3), True,
            hacc_m=0.1, sacc_m_s=0.05, pdop=1.2, vacc_m=0.2)

    def test_delayed_update_replay_matches_on_time_update(self):
        config = replace(SelfAimConfig(), navigation_buffer_seconds=5.0)
        on_time = RealtimeLooseNavigation(config)
        delayed = RealtimeLooseNavigation(config)
        on_time.start(self.initial_state())
        delayed.start(self.initial_state())

        for sequence in range(1, 21):
            sample = self.stationary_sample(sequence, 100.0+sequence*0.01)
            on_time.update_imu(sample)
            delayed.update_imu(sample)
            if sequence == 10:
                self.assertTrue(on_time.update_gnss(self.observation(100.1)))

        self.assertTrue(delayed.update_gnss(self.observation(100.1)))
        self.assertEqual(delayed.solution().replay_samples, 10)
        self.assertTrue(np.allclose(delayed.position_ned, on_time.position_ned, atol=1e-10))
        self.assertTrue(np.allclose(delayed.velocity_ned, on_time.velocity_ned, atol=1e-10))
        self.assertTrue(np.allclose(delayed.c_nb, on_time.c_nb, atol=1e-10))
        self.assertTrue(np.allclose(delayed.p, on_time.p, atol=1e-10))

    def test_sensor_to_frd_rotation_is_preserved(self):
        rotation = ((0.0, 1.0, 0.0), (-1.0, 0.0, 0.0), (0.0, 0.0, 1.0))
        engine = RealtimeLooseNavigation(replace(
            SelfAimConfig(), body_from_sensor=rotation))
        accel, gyro = engine._body_measurement(ImuSample(
            1, 100.01, (1.0, 2.0, 3.0), (4.0, 5.0, 6.0), 0.01))
        self.assertTrue(np.array_equal(accel, (2.0, -1.0, 3.0)))
        self.assertTrue(np.array_equal(gyro, (5.0, -4.0, 6.0)))

    def test_old_and_duplicate_gnss_are_rejected(self):
        engine = RealtimeLooseNavigation(SelfAimConfig())
        engine.start(self.initial_state())
        for sequence in range(1, 21):
            engine.update_imu(self.stationary_sample(sequence, 100.0+sequence*0.01))
        self.assertFalse(engine.update_gnss(self.observation(90.0)))
        self.assertTrue(engine.update_gnss(self.observation(100.1)))
        self.assertFalse(engine.update_gnss(self.observation(100.1)))
        self.assertIn("重复", engine.solution().last_gnss_reason)

    def test_ned_down_is_negative_height(self):
        engine = RealtimeLooseNavigation(SelfAimConfig())
        engine.start(self.initial_state())
        engine.position_ned[2] = 5.0
        self.assertAlmostEqual(engine.solution().height_m, 15.0)

    def test_long_imu_gap_stops_instead_of_freezing_silently(self):
        engine = RealtimeLooseNavigation(SelfAimConfig())
        engine.start(self.initial_state())
        solution = engine.update_imu(self.stationary_sample(1, 100.2))
        self.assertFalse(solution.running)
        self.assertIn("IMU时间间隔异常", solution.status)


if __name__ == "__main__":
    unittest.main()
