import sys,unittest
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
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
    def test_floor_alone_cannot_establish_yaw_and_xy(self):
        a=MapAligner.__new__(MapAligner);xx,yy=np.meshgrid(np.linspace(-2,2,60),np.linspace(-2,2,60))
        a.target=np.column_stack([xx.ravel(),yy.ravel(),np.full(xx.size,-.57)]);a.tree=cKDTree(a.target)
        q=a.quality(a.target,np.zeros(3),np.zeros(3),0.)
        self.assertFalse(a.acceptable(q));self.assertEqual(q['structure_points'],0)
        a.normals=np.zeros_like(a.target);a.normals[:,2]=1
        with self.assertRaises(ValueError):a.refine_planes(a.target,np.zeros(3),0.)
if __name__=='__main__':unittest.main()
