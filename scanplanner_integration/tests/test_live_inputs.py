from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from live_inputs import InputHealth


class StartupHealth(unittest.TestCase):
    def test_real_stream_required_instead_of_discovered_or_latched_topic(self):
        h = InputHealth()
        self.assertFalse(h.fresh('odom', 10.))
        for _ in range(20):
            h.receive('odom', 10., 10.1)
        self.assertFalse(h.fresh('odom', 10.1))
        h.receive('odom', 10.1, 10.2)
        self.assertTrue(h.fresh('odom', 10.2))
        self.assertFalse(h.fresh('odom', 10.7))

    def test_stale_future_invalid_and_wrong_frame_do_not_unlock_startup(self):
        for stamp, valid in [(5., True), (11., True), (float('nan'), True), (9.9, False)]:
            h = InputHealth()
            h.receive('odom', stamp, 10., valid)
            h.receive('odom', stamp+.01, 10., valid)
            self.assertFalse(h.fresh('odom', 10.))

    def test_cloud_and_imu_do_not_replace_body_odometry(self):
        h = InputHealth()
        for name in ('cloud', 'imu'):
            for stamp in (9.9, 10.):
                h.receive(name, stamp, 10.1)
        state = h.summary(10.1)
        self.assertTrue(state['cloud']['fresh'])
        self.assertTrue(state['imu']['fresh'])
        self.assertFalse(state['odom']['fresh'])

    def test_registered_cloud_requires_matching_lidar_update_not_imu_odom(self):
        h=InputHealth()
        h.receive('registered',10.,10.1);h.receive('odom',10.,10.1)
        h.receive('lidar_odom',10.01,10.1)
        self.assertFalse(h.registered_pair(10.1))
        h.receive('lidar_odom',10.,10.1)  # Out of order is ignored.
        self.assertFalse(h.registered_pair(10.1))
        h.receive('registered',10.1,10.2);h.receive('lidar_odom',10.1,10.2)
        self.assertTrue(h.registered_pair(10.2));self.assertFalse(h.registered_pair(10.7))

if __name__ == '__main__':
    unittest.main()
