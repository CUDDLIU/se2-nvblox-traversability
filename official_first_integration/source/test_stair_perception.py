"""Input synchronization and semantic obstacle execution, without ROS/hardware."""
import unittest
from types import SimpleNamespace as N
import numpy as np
from stair_perception import StairPerception
from core import Core

def header(t,frame):return N(stamp=N(sec=int(t),nanosec=round((t-int(t))*1e9)),frame_id=frame)
def pose(t):return N(header=header(t,'nvblox_odom'),child_frame_id='base_link_dog',pose=N(pose=N(position=N(x=0.,y=0.,z=.55),orientation=N(x=0.,y=0.,z=0.,w=1.))))
def imu(t):return N(header=header(t,'imu'),orientation=N(x=0.,y=0.,z=0.,w=1.))
def scan(t):return N(header=header(t,'base_link_dog'))
def ground():return np.array([[x,y,-.55] for x in np.arange(-2,3,.06) for y in np.arange(-1,1,.06)],np.float32)

class StairInputs(unittest.TestCase):
 def setUp(self):self.p=StairPerception();self.p.enable(True,after=1_000_000_000)
 def tearDown(self):self.p.close()
 def feed(self,t):
  self.p.odometry(pose(t));self.p.cloud(scan(t));self.p.imu(imu(t-.01));self.p.imu(imu(t+.01))
 def test_exact_pose_and_imu_bracket_required(self):
  self.p.odometry(pose(1.1));self.p.cloud(scan(1.1001));self.p.imu(imu(1.09));self.p.imu(imu(1.09+.005))
  self.assertIsNone(self.p.update(lambda _:ground(),now=1.12))
  self.p.cloud(scan(1.1));self.assertIsNone(self.p.update(lambda _:ground(),now=1.12))
  self.p.imu(imu(1.11));self.assertIsNotNone(self.p.update(lambda _:ground(),now=1.12))
  self.assertTrue(self.p.diagnostics['gravity_matched'])
 def test_reset_discards_previous_generation_empty_scan_never_ready(self):
  self.feed(1.1);self.assertIsNotNone(self.p.update(lambda _:ground(),now=1.12))
  self.p.enable(True,after=1_200_000_000)
  self.assertIsNone(self.p.update(lambda _:ground(),now=1.22))
  self.feed(1.3);self.assertIsNone(self.p.update(lambda _:np.empty((0,3)),now=1.32))
  self.assertEqual(self.p.diagnostics['stair_perception'],'empty_registered_scan')
 def test_no_silent_flat_fallback_on_old_or_unavailable_imu(self):
  self.feed(1.1)
  self.assertIsNone(self.p.update(lambda _:ground(),now=1.6))
  self.p.odometry(pose(1.7));self.p.cloud(scan(1.7))
  self.assertIsNone(self.p.update(lambda _:ground(),now=1.72))
  invalid=imu(1.71);invalid.orientation_covariance=[-1];self.p.imu(invalid)
  self.assertIsNone(self.p.update(lambda _:ground(),now=1.72))

class SemanticObstacles(unittest.TestCase):
 @classmethod
 def setUpClass(cls):cls.core=Core()
 @classmethod
 def tearDownClass(cls):cls.core.close()
 def test_one_cost_cell_z_zero_is_not_dropped_by_raw_return_grid(self):
  c=self.core;c.obstacles([[.8,0,0]]);c.goal()
  blocked=c.plan((.8,0,0),(.8,0,0))
  self.assertEqual((blocked.x,blocked.y,blocked.w,blocked.status),(0,0,0,1102))
  c.obstacles([]);c.goal();self.assertGreater(c.plan((1.5,0,0),(5,0,0)).x,0.)
 def test_height_obstacle_blocks_planner_and_clear_map_recovers(self):
  from height_frontend import HeightFrontend
  # Tall close wall enclosing the front, not a 15 cm stair tread.
  wall=np.array([[x,y,z] for x in np.arange(.50,.85,.025) for y in np.arange(-1,1,.025) for z in np.arange(-.55,.45,.025)],np.float32)
  c=self.core
  with HeightFrontend(accumulate=False) as f:
   f.policy.set_terrain(2);f.process(np.vstack([ground(),wall]),1.)
   nav=f.height_map()['cloud_nav'];self.assertGreater(len(nav),0)
   c.obstacles(nav);c.goal();v=c.plan((1.5,0,0),(5,0,0));self.assertEqual((v.x,v.y,v.w,v.status),(0,0,0,1101))
   c.goal();v=c.line((3,0,0));self.assertEqual((v.x,v.y,v.w,v.status),(0,0,0,2005))
   f.process(ground(),2.);c.obstacles(f.height_map()['cloud_nav']);c.goal()
   self.assertGreater(c.line((3,0,0)).x,0.)

if __name__=='__main__':unittest.main()
