import sys,unittest
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from alignment import MapAligner,rotation,deskew

class AlignmentTests(unittest.TestCase):
    def test_deskew_measured_motion(self):
        poses=np.array([[10.,0,0,0,0,0,0,1],[11.,1,0,0,0,0,0,1]])
        ts=np.linspace(10,11,600);raw=np.column_stack([3-(ts-10),np.linspace(-2,2,600),np.zeros(600),ts])
        actual=deskew(raw,poses)
        np.testing.assert_allclose(actual[:,0],3,atol=.0003)
        with self.assertRaises(ValueError):deskew(raw,poses+np.array([20,0,0,0,0,0,0,0]))
    def test_match_known_transform_and_reject_unrelated(self):
        rng=np.random.default_rng(17)
        target=rng.uniform(-3,3,(6500,3));target[:2200,2]=-.57;target[2200:4400,0]=2.8;target[4400:,1]=-2.7
        a=MapAligner.__new__(MapAligner);a.target=target;a.tree=cKDTree(target)
        a.normals=np.zeros_like(target);a.normals[:2200,2]=1;a.normals[2200:4400,0]=1;a.normals[4400:,1]=1
        truth_t=np.array([.13,-.19,.015]);truth_y=.08
        source=(target-truth_t)@rotation(truth_y);body=np.zeros(3)
        t,y,q=a.fit(source,body,(truth_t+np.array([.02,-.015,.01]),truth_y+.007))
        self.assertTrue(a.acceptable(q));self.assertLess(np.linalg.norm(t-truth_t),.025);self.assertLess(abs(y-truth_y),.007)
        self.assertFalse(a.acceptable(a.quality(source+30,body,t,y)))
        # Initialization must already agree with the plane-based tracker;
        # no first-frame jump is authorized to bridge incompatible objectives.
        a.anchor=truth_t.copy()
        t,y,q=a.fit(source,body)
        self.assertLess(np.linalg.norm(t-truth_t),.01)
        self.assertLess(abs(y-truth_y),.01)
        t2,y2,_=a.fit(source,body,(t,y))
        self.assertLess(np.linalg.norm(t2-t),.005)
        self.assertLess(abs(y2-y),.002)
    def test_floor_alone_cannot_establish_yaw_and_xy(self):
        a=MapAligner.__new__(MapAligner);xx,yy=np.meshgrid(np.linspace(-2,2,60),np.linspace(-2,2,60))
        a.target=np.column_stack([xx.ravel(),yy.ravel(),np.full(xx.size,-.57)]);a.tree=cKDTree(a.target)
        q=a.quality(a.target,np.zeros(3),np.zeros(3),0.)
        self.assertFalse(a.acceptable(q));self.assertEqual(q['structure_points'],0)
        a.normals=np.zeros_like(a.target);a.normals[:,2]=1
        with self.assertRaises(ValueError):a.refine_planes(a.target,np.zeros(3),0.)
        with self.assertRaises(ValueError):a.refine_rigid_planes(a.target,np.zeros(3),np.eye(3))

    def test_rigid_initialization_corrects_tilt_and_tracks_without_jump(self):
        rng=np.random.default_rng(19)
        target=rng.uniform(-3,3,(6500,3))
        target[:2200,2]=-.57;target[2200:4400,0]=2.8;target[4400:,1]=-2.7
        a=MapAligner.__new__(MapAligner);a.target=target;a.tree=cKDTree(target)
        a.normals=np.zeros_like(target);a.normals[:2200,2]=1;a.normals[2200:4400,0]=1;a.normals[4400:,1]=1
        truth_t=np.array([.13,-.19,.015]);truth_r=Rotation.from_euler('xyz',[-2.1,4.6,-7.4],degrees=True).as_matrix()
        a.anchor=truth_t.copy();source=(target-truth_t)@truth_r;body=np.zeros(3)
        t,r,q=a.fit_rigid(source,body)
        self.assertGreater(q['inliers_15cm'],.85)
        self.assertLess(np.linalg.norm(t-truth_t),.01)
        self.assertLess(Rotation.from_matrix(r@truth_r.T).magnitude(),.005)
        t2,r2,_=a.fit_rigid(source,body,(t,r))
        self.assertLess(np.linalg.norm(t2-t),.005)
        self.assertLess(Rotation.from_matrix(r2@r.T).magnitude(),.002)
        with self.assertRaises(ValueError):a.fit_rigid(source+30,body,(t,r))
        with self.assertRaises(ValueError):a.fit_rigid(source,body,(t+np.array([.3,0,0]),r))
if __name__=='__main__':unittest.main()
