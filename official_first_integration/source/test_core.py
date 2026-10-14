"""Behavioral sequences. No ROS, DDS or robot controller required."""
import math,unittest
import numpy as np
from core import Core
def block(x0,x1,y0,y1):
    return np.array([[x,y,z] for x in np.arange(x0,x1,.01) for y in np.arange(y0,y1,.01) for z in (.06,.08,.1,.12)],np.float32)
class SourceBehavior(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.core=Core();cls.clear=np.vstack((block(-2.,-1.9,1.5,1.7),block(2.,2.1,1.5,1.7)))
    @classmethod
    def tearDownClass(cls):cls.core.close()
    def setUp(self):self.core.cloud(self.clear);self.core.goal()
    def obstacle(self,points):self.core.cloud(np.vstack((self.clear,points)))
    def test_cancel_invalidates_moving_goal(self):
        self.assertGreater(self.core.plan((1.5,0,0),(8,0,0)).x,0)
        self.core.cancel();r=self.core.plan((1.5,0,0),(8,0,0));self.assertEqual((r.x,r.y,r.w,r.status),(0,0,0,999))
        self.core.goal();self.assertGreater(self.core.plan((1.5,0,0),(8,0,0)).x,0)
    def test_terminal_does_not_complete_route_loop_early(self):
        r=self.core.plan((1.5,0,0),(.01,0,0),terminal=False)
        self.assertEqual(r.status,1001);self.assertGreater(r.x,0)
        r=self.core.plan((.01,0,0),(.01,0,0));self.assertEqual(r.status,999)
    def test_obstacle_clear_waits_ten_cycles(self):
        self.obstacle(block(.6,.8,-.3,.3));r=self.core.line((3,0,0))
        self.assertEqual((r.x,r.y,r.w,r.status),(0,0,0,2005))
        self.core.cloud(self.clear)
        for _ in range(10):
            r=self.core.line((3,0,0));self.assertEqual((r.x,r.y,r.w,r.status),(0,0,0,2005))
        self.assertGreater(self.core.line((3,0,0)).x,0)
    def test_new_goal_resets_obstacle_wait(self):
        self.obstacle(block(.6,.8,-.3,.3));self.core.line((3,0,0))
        self.core.cancel();self.core.cloud(self.clear);self.core.goal()
        self.assertGreater(self.core.line((3,0,0)).x,0)
    def test_rotation_clear_waits_ten_cycles(self):
        self.obstacle(block(.46,.49,.05,.09));r=self.core.line((3,1,0))
        self.assertEqual((r.x,r.y,r.w,r.status),(0,0,0,2004))
        self.core.cloud(self.clear)
        for _ in range(10):self.assertEqual(self.core.line((3,1,0)).w,0)
        self.assertGreater(self.core.line((3,1,0)).w,0)
    def test_front_obstacle_allows_small_yaw_only(self):
        self.core.line((3,0,0)) # Enter and latch approach stage.
        self.obstacle(block(.6,.8,-.3,.3));r=self.core.line((3,.6,0))
        self.assertEqual((r.x,r.y,r.status),(0,0,2015));self.assertAlmostEqual(r.w,.35,places=6)
    def test_turn_approach_final_alignment(self):
        self.assertEqual(self.core.line((3,1,0)).status,2001)
        self.assertEqual(self.core.line((3,0,0)).status,2002)
        r=self.core.line((.01,0,.5));self.assertEqual(r.status,2003);self.assertGreater(r.w,0)
        self.assertEqual(self.core.line((.01,0,.01)).status,999)
    def test_nonfinite_goal_and_negative_limit_rejected(self):
        for p in ((math.nan,0,0),(0,math.inf,0),(0,0,math.nan)):
            with self.assertRaises(ValueError):self.core.plan(p,(1,0,0))
            with self.assertRaises(ValueError):self.core.line(p)
        with self.assertRaises(ValueError):self.core.plan((1,0,0),(1,0,0),limits=(-1,.6,1))
    def test_enclosed_goal_is_blocked_then_recovers(self):
        self.obstacle(np.vstack((block(.46,.6,-.65,.65),block(-.6,-.46,-.65,.65),block(-.45,.45,.27,.4),block(-.45,.45,-.4,-.27))))
        r=self.core.plan((1.5,0,0),(8,0,0));self.assertEqual((r.x,r.y,r.w,r.status),(0,0,0,1101))
        self.core.cloud(self.clear);r=self.core.plan((1.5,0,0),(8,0,0));self.assertEqual(r.status,1001);self.assertGreater(r.x,0)
    def test_final_goal_in_obstacle(self):
        self.obstacle(block(.75,.95,-.15,.15));r=self.core.plan((.8,0,0),(.8,0,0))
        self.assertEqual((r.x,r.y,r.w,r.status),(0,0,0,1102))
if __name__=='__main__':unittest.main()
