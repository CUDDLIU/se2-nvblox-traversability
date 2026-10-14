"""Continuous full-flight regression for the localization-cloud wiring error.

Old /body_cloud excluded range <=2 m and used 0.5 m centroids. It must not be
used as a height-map observation. All obstacle kernels/thresholds stay equal.
"""
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from height_frontend import HeightFrontend
from core import Core


def flight_cloud():
    x,y=np.meshgrid(np.arange(-2,7,.035),np.arange(-1.1,1.1,.035))
    z=np.clip(np.floor((x-1.)/.3)+1,0,12)*.15
    return np.c_[x.ravel(),y.ravel(),z.ravel()]


def localization_cloud(points):
    points=points[np.linalg.norm(points,axis=1)>2.]
    _,index=np.unique(np.floor(points/.5).astype(int),axis=0,return_inverse=True)
    count=np.bincount(index)
    return np.array([np.bincount(index,weights=points[:,i])/count for i in range(3)]).T.astype(np.float32)


class ContinuousFlight(unittest.TestCase):
    def test_full_flight_without_goal_reset_and_with_real_obstacle(self):
        world=flight_cloud();results={}
        for sparse in (True,False):
            c=Core()
            try:
                with HeightFrontend() as f:
                    f.policy.set_terrain(2);c.goal();blocked=0
                    for j,x in enumerate(np.arange(.1,4.3,.06)):
                        t=1.+j*.1;z=np.clip((x-1.)/.3+.5,0,12)*.15+.55
                        r=Rotation.from_euler('ZYX',[0,-.42 if x>1 else 0,0]);q=r.as_quat()
                        points=r.inv().apply(world-[x,0,z])
                        points=points[np.linalg.norm(points,axis=1)<5.]
                        if sparse:points=localization_cloud(points)
                        f.add_imu(t-.01,q);f.add_imu(t+.01,q);f.process(points,t,[x,0,z,*q])
                        c.obstacles(f.height_map()['cloud_nav'])
                        v=c.plan((1.5,0,0),(6,0,0),terminal=False)
                        blocked+=v.status==1101
                    results[sparse]=blocked
                    if not sparse:
                        # A real tall obstacle must still stop the same active
                        # goal; retaining dense input is not an obstacle bypass.
                        wall=np.array([[x,y,z] for x in np.arange(.5,.85,.025)
                                       for y in np.arange(-1,1,.025) for z in np.arange(-.55,.45,.025)])
                        f.add_imu(t+.09,q);f.add_imu(t+.11,q)
                        f.process(np.vstack([points,r.inv().apply(wall)]),t+.1,[x,0,z,*q])
                        c.obstacles(f.height_map()['cloud_nav'])
                        self.assertEqual(c.plan((1.5,0,0),(6,0,0),terminal=False).status,1101)
            finally:c.close()
        self.assertGreaterEqual(results[True],10)
        self.assertEqual(results[False],0)


if __name__=='__main__':unittest.main()
