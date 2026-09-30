import math
from pathlib import Path
import struct
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from core import Route,sample_bspline,route_yaws,velocity_packet,decode_packet,HEADER

class IntegrationCore(unittest.TestCase):
    def test_same_xy_other_floor_cannot_advance(self):
        r=Route([[0,0,-1,0],[1,0,-1,0],[1,0,0,0],[0,0,0,0],[0,0,3,0]])
        r.window([0,0,-.44,0],.56)
        self.assertEqual(r.index,0)
        self.assertGreater(r.distance([0,0,3]),3)
    def test_rotation_retained_until_heading_reached(self):
        r=Route([[0,0,-1,0],[0,0,-1,math.pi/2],[1,0,-1,math.pi/2]])
        w,turn=r.window([0,0,-.44,0],.56)
        self.assertTrue(turn);self.assertEqual(r.index,0);self.assertEqual(len(w),2)
        self.assertAlmostEqual(w[-1,3],math.pi/2)
        w,turn=r.window([0,0,-.44,math.pi/2],.56)
        self.assertFalse(turn);self.assertEqual(r.index,1)
    def test_stairs_preserve_z_and_heading(self):
        r=Route([[0,0,-2,1.1],[1,0,-1.8,1.2],[2,0,-1.6,1.3]])
        w,_=r.window([0,0,-1.43,1.1],.57,horizon=1.5)
        self.assertGreater(w[-1,2],w[0,2]);self.assertLess(w[-1,2],0)
        self.assertAlmostEqual(w[0,3],1.1)
    def test_spline_agrees_with_scipy(self):
        from scipy.interpolate import BSpline
        p=np.array([[0,0,0],[0,0,0],[.2,0,.1],[.8,.1,.3],[1.,0,.5],[1.,0,.5]])
        k=np.arange(-3,7,dtype=float)*.2
        got=sample_bspline(p,k,3)
        expected=BSpline(k,p,3)(np.linspace(k[3],k[-4],len(got)))
        np.testing.assert_allclose(got,expected,atol=1e-12)
    def test_bad_spline_rejected(self):
        with self.assertRaises(ValueError):sample_bspline([[0,0,0]]*4,[0]*8,3)
        with self.assertRaises(ValueError):sample_bspline([[np.nan,0,0]]*4,list(range(8)),3)
    def test_orientation_wrap(self):
        a=route_yaws([[.5,0,0]],np.array([[0,0,0,3.1],[1,0,0,-3.1]]))
        self.assertAlmostEqual(abs(a[0]),math.pi)
    def test_velocity_exact_units_header_and_sequence(self):
        raw=velocity_packet(.12,-.03,.2,65537)
        self.assertEqual(raw[:4],bytes.fromhex('eb91eb90'));self.assertEqual(HEADER.size,16)
        sequence,doc=decode_packet(raw)
        self.assertEqual(sequence,1);self.assertEqual((doc['Type'],doc['Command']),(2,25))
        self.assertEqual(doc['Items']['X'],.12);self.assertEqual(doc['Items']['Yaw'],.2)
        with self.assertRaises(ValueError):decode_packet(raw[:-1])
        with self.assertRaises(ValueError):velocity_packet(float('nan'),0,0)
        _,normal=decode_packet(velocity_packet(.5,-.25,.6))
        self.assertEqual(normal['Items']['X'],.5)
        self.assertEqual(normal['Items']['Y'],-.25)
        for v in [(.51,0,0),(0,.26,0),(0,0,.61)]:
            with self.assertRaises(ValueError):velocity_packet(*v)
class Watchdog(unittest.TestCase):
    def test_ready_then_stale_estop_and_reverse_direction(self):
        from m20_udp import Gate
        g=Gate();g.command=(.1,.02,.1);g.command_at=10
        b=dict(ControlUsageMode=1,HES=0,Direction=0,MotionState=17,Sleep=0)
        g.receive(dict(Type=1002,Command=6,Items=dict(BasicStatus=b)),10)
        g.receive(dict(Type=1002,Command=4,Items={}),10)
        self.assertEqual(g.output(10.05),g.command)
        self.assertEqual(g.output(10.13),(0,0,0))
        b['HES']=1;self.assertEqual(g.output(10.05),(0,0,0))
        b['HES']=0;b['Direction']=1;self.assertEqual(g.output(10.05),(0,0,0))
        b['Direction']=0;g.receive(dict(Type=2,Command=25,ErrorCode=5),10)
        self.assertEqual(g.output(10.05),(0,0,0))
if __name__=='__main__':unittest.main()
