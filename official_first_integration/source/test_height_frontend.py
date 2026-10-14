import math,unittest
import numpy as np
from height_frontend import HeightFrontend,library,ptr

class FrontendTests(unittest.TestCase):
    def test_accumulated_wall_stays_in_world_during_translation_and_turn(self):
        wall=np.array([[2.,y,z] for y in np.arange(-1,1,.09) for z in np.arange(-.8,.3,.09)],np.float32)
        with HeightFrontend(casting=False) as f:
            first=f.process(wall,1.)
            q=[0,0,math.sin(.3),math.cos(.3)]
            second=f.process([],2.,[.25,-.1,0,*q])
            c,s=math.cos(.6),math.sin(.6)
            expected=(first-[.25,-.1,0])@np.array([[c,-s,0],[s,c,0],[0,0,1]])
            # Keep pairs unordered because PCL voxel ordering changes with frame.
            from scipy.spatial import cKDTree
            distances=cKDTree(expected).query(second)[0]
            self.assertLess(float(max(distances)),3e-6)
            self.assertEqual(len(first),len(second))
    def test_imu_roll_pitch_removed_heading_preserved(self):
        from scipy.spatial.transform import Rotation
        rotation=Rotation.from_euler('ZYX',[.7,.25,-.1])
        points=np.array([[1.2,.3,-.3],[2.,-.4,-.2]],np.float32)
        with HeightFrontend() as f:
            f.add_imu(.9,rotation.as_quat());f.add_imu(1.1,rotation.as_quat())
            actual=f.process(points,1.)
            g=Rotation.from_euler('Z',-.7)*rotation
            expected=g.apply(f.cloud(body=True))
            np.testing.assert_allclose(actual,expected,atol=4e-7)
            self.assertTrue(f.gravity_matched)
    def test_no_bracketing_imu_uses_native_uncorrected_branch(self):
        with HeightFrontend() as f:
            f.add_imu(.8,[0,.2,0,1]);f.add_imu(.9,[0,.2,0,1])
            out=f.process([[1.2,.3,-.3]],1.)
            np.testing.assert_array_equal(out,f.cloud(body=True))
            self.assertFalse(f.gravity_matched)
    def test_old_obstacle_is_cleared_by_farther_return(self):
        # Off voxel boundaries; removal also checked against original AArch64.
        lib=library();old=np.array([[.91,.113,-.127],[-2,-1,-1],[2,1,.5]],np.float32)
        now=np.array([[1.93,.339,-.381]],np.float32);out=np.empty_like(old)
        n=lib.aos_clear(ptr(now),1,ptr(old),3,1,ptr(out))
        np.testing.assert_array_equal(out[:n],old[1:])
        # An axis-aligned zero component prevents strict XYZ removal in native.
        old[0,1]=0.;now[0,1]=0.
        self.assertEqual(lib.aos_clear(ptr(now),1,ptr(old),3,1,ptr(out)),3)
    def test_native_serial_keeps_near_body_point_parallel_clears(self):
        lib=library();old=np.array([[.83,.53,-.13],[-2,-1,-1],[2,1,.5]],np.float32)
        now=np.array([[1.69,1.59,-.39]],np.float32);out=np.empty_like(old)
        self.assertEqual(lib.aos_clear(ptr(now),1,ptr(old),3,0,ptr(out)),3)
        n=lib.aos_clear(ptr(now),1,ptr(old),3,1,ptr(out))
        np.testing.assert_array_equal(out[:n],old[1:])
    def test_filters_empty_reset_and_sanitization(self):
        with HeightFrontend(accumulate=False) as f:
            out=f.process([[0,0,0],[4.1,0,0],[1,0,.6],[1,0,-1.6],[1,0,float('nan')],[1,.4,-.5]],1.)
            np.testing.assert_array_equal(out,np.array([[1,.4,-.5]],np.float32))
            f.process([],2.)
            self.assertEqual(len(f.cloud()),0)
            m=f.height_map();self.assertEqual(len(m['cloud_nav']),0)
            self.assertTrue(np.all(m['height']==np.float32(-.55)))
            with self.assertRaises(ValueError):f.process([],2.)
    def test_stair_treads_are_not_walls_but_tall_wall_is_retained(self):
        points=[]
        for x in np.arange(.5,2.8,.022):
            for y in np.arange(-.7,.7,.022):
                z=-.55+min(4,max(0,int((x-1.)/.3)+1))*.15
                points.append((x,y,z))
        # 0.48 m wide wall patch, rising above a complete tread.
        for x in np.arange(1.8,2.05,.022):
            for y in np.arange(.35,.65,.022):points.append((x,y,.5))
        with HeightFrontend() as f:
            f.process(points,1.)
            f.policy.set_terrain(2);m=f.height_map()
            nav=m['cloud_nav'];self.assertGreater(len(nav),0)
            self.assertTrue(np.all(nav[:,2]==0.))
            self.assertTrue(np.any((nav[:,0]>1.7)&(nav[:,0]<2.1)&(nav[:,1]>.3)))
            # Centerline stair risers remain traversable with captured 25 cm threshold.
            self.assertFalse(np.any((nav[:,0]<1.7)&(np.abs(nav[:,1])<.2)))

if __name__=='__main__':unittest.main()
