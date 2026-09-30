import math
import unittest
from kinematics import model_angles, OFFSETS_DEG, DIRECTIONS, JOINT_NAMES, WheelIntegrator


class KinematicsTests(unittest.TestCase):
    def test_sdk_zero_and_complete_namespace(self):
        values=[-math.radians(o)/s for o,s in zip(OFFSETS_DEG,DIRECTIONS)]
        for i,a in enumerate(model_angles(values)):
            if i%4==3:self.assertIsNone(a)
            else:self.assertAlmostEqual(a,0.)
        self.assertEqual(len(set(JOINT_NAMES)),16)
        self.assertTrue(all(n.startswith('nvblox_robot/') for n in JOINT_NAMES))

    def test_real_standing_sample_and_wrap(self):
        values=[.52339685,2.0933256,2.3620794,.0059,-.487569,-2.0831356,-2.3251224,-.0279,
                -.5290882,-2.0424206,-2.3202713,-.0249,.5654038,2.0132504,2.1633732,-.0121]
        q=model_angles(values)
        self.assertAlmostEqual(math.degrees(q[0]),4.99,places=2)
        self.assertAlmostEqual(math.degrees(q[2]),24.66,places=2)
        values[1]+=2*math.pi
        self.assertAlmostEqual(model_angles(values)[1],q[1])
        values[0]=99
        with self.assertRaises(ValueError):model_angles(values)

    def test_wheels_are_relative_and_do_not_extrapolate_gaps(self):
        w=WheelIntegrator();values=[0.]*16
        for i in (3,7,11,15):values[i]=2.
        p,s=w.update(values,1_000_000_000)
        self.assertEqual(p,[0.]*4);self.assertEqual(s,[2.,-2.,2.,-2.])
        p,_=w.update(values,1_100_000_000)
        for a,b in zip(p,[.2,-.2,.2,-.2]):self.assertAlmostEqual(a,b)
        p2,_=w.update(values,2_000_000_000);self.assertEqual(p2,p)
        with self.assertRaises(ValueError):w.update(values,1_000_000_000)


if __name__=='__main__':unittest.main()
