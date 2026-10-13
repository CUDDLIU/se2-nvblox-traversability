"""State-machine regressions; run in the ROS environment on the Jetson."""
import sys,time,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
try:
    from route_node import RouteNode
    from scan_planner_msgs.msg import Bspline
    from geometry_msgs.msg import Point
except ImportError:
    RouteNode=None

@unittest.skipIf(RouteNode is None,'requires installed ROS messages')
class RouteStateTests(unittest.TestCase):
    def candidate(self):
        m=Bspline();m.order=3;m.knots=[float(x)*.5 for x in range(-3,7)]
        m.pos_pts=[Point(x=x,y=0.,z=.57) for x in [0.,0.,.2,.4,.6,.6]]
        ref=np.array([[0.,0.,.57,1.2],[.6,0.,.57,1.2]])
        return m,(m,1,ref,.57,time.monotonic(),np.array([0.,0.,.57,0.]),.57)

    def test_valid_native_spline_is_not_replaced_for_heading_difference(self):
        m,item=self.candidate()
        owner=SimpleNamespace(max_step=.2,guard=SimpleNamespace(surface_poses=lambda poses:poses,motions=lambda pairs:np.ones(len(pairs),dtype=bool)))
        _,poses,reason=RouteNode.verify(owner,item)
        self.assertIsNotNone(poses);self.assertEqual(reason,'verified native SCAN spline')

    def test_new_body_height_does_not_shift_an_existing_reference_floor(self):
        _,item=self.candidate()
        item=(*item[:6],.60)
        pairs_seen=[]
        def check(pairs):
            pairs_seen.extend(pairs);return np.ones(len(pairs),dtype=bool)
        owner=SimpleNamespace(max_step=.2,guard=SimpleNamespace(surface_poses=lambda poses:poses,motions=check))
        _,poses,_=RouteNode.verify(owner,item)
        self.assertIsNotNone(poses)
        np.testing.assert_allclose(poses[:,2],0.,atol=1e-9)
        self.assertAlmostEqual(pairs_seen[0][0][2],0.)

    def test_body_z_smoothing_does_not_move_the_saved_ground_layer(self):
        m,item=self.candidate()
        for p in m.pos_pts:p.z+=.025
        owner=SimpleNamespace(max_step=.2,guard=SimpleNamespace(surface_poses=lambda poses:poses,motions=lambda pairs:np.ones(len(pairs),dtype=bool)))
        _,poses,_=RouteNode.verify(owner,item)
        self.assertIsNotNone(poses);np.testing.assert_allclose(poses[:,2],0.,atol=1e-9)
        for p in m.pos_pts:p.z+=.3
        _,poses,reason=RouteNode.verify(owner,item)
        self.assertIsNone(poses);self.assertIn('surface layer',reason)

    def test_inconsistent_measured_height_cannot_snap_to_another_floor(self):
        _,item=self.candidate();item=(*item[:6],.9)
        owner=SimpleNamespace(max_step=.2,guard=Mock(surface_poses=lambda poses:poses))
        _,poses,reason=RouteNode.verify(owner,item)
        self.assertIsNone(poses);self.assertIn('surface layer',reason);owner.guard.motions.assert_not_called()

    def test_native_detour_is_decided_by_geometry_not_a_narrow_reference_band(self):
        m,item=self.candidate()
        m.pos_pts=[Point(x=float(x),y=float(y),z=.57) for x,y in [(0,0),(0,0),(.1,1.3),(.5,1.3),(.6,0),(.6,0)]]
        owner=SimpleNamespace(max_step=.2,guard=SimpleNamespace(surface_poses=lambda poses:poses,motions=lambda pairs:np.ones(len(pairs),dtype=bool)))
        _,poses,reason=RouteNode.verify(owner,item)
        self.assertIsNotNone(poses);self.assertEqual(reason,'verified native SCAN spline')
        owner.guard.motions=lambda pairs:np.zeros(len(pairs),dtype=bool)
        _,poses,reason=RouteNode.verify(owner,item)
        self.assertIsNone(poses);self.assertIn('body/support sweep',reason)

    def test_unsafe_native_spline_is_rejected_without_inventing_motion(self):
        _,item=self.candidate()
        owner=SimpleNamespace(max_step=.2,guard=SimpleNamespace(surface_poses=lambda poses:poses,motions=lambda pairs:np.zeros(len(pairs),dtype=bool)))
        _,poses,reason=RouteNode.verify(owner,item)
        self.assertIsNone(poses);self.assertIn('body/support sweep',reason)

    def test_stationary_stop_is_not_a_motion_certificate(self):
        m,item=self.candidate();m.pos_pts=[Point(x=0.,y=0.,z=.57) for _ in range(6)]
        owner=SimpleNamespace(max_step=.2,guard=Mock(surface_poses=lambda poses:poses))
        _,poses,reason=RouteNode.verify(owner,item)
        self.assertIsNone(poses);self.assertIn('stop trajectory',reason);owner.guard.motions.assert_not_called()
if __name__=='__main__':unittest.main()
