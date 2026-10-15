"""Initial search must cover poses between the former metre-spaced seeds."""
from pathlib import Path
import sys
import unittest
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'scanplanner_integration'),str(ROOT/'official_first_integration')]
from reference_alignment import ReferenceAligner
from alignment import MapAligner


class InitialSearchTests(unittest.TestCase):
    def test_sparse_surface_interpolation_is_bounded_and_normal_supported(self):
        a=ReferenceAligner.__new__(ReferenceAligner)
        x,y=np.meshgrid(np.arange(-1.,1.01,.2),np.arange(-1.,1.01,.2))
        a.target=np.column_stack([x.ravel(),y.ravel(),np.zeros(x.size)])
        a.normals=np.tile([0.,0.,1.],(len(a.target),1));a.tree=cKDTree(a.target)
        points=np.array([[.1,.1,.03],[1.2,.1,.03],[.1,.1,.6]])
        d,supported=a.surface_distances(points)
        self.assertAlmostEqual(d[0],.03,places=8)
        self.assertTrue(supported[0]);self.assertFalse(supported[1]);self.assertFalse(supported[2])
        np.testing.assert_allclose(d[1:],a.tree.query(points[1:])[0])
        # A normal incompatible with the sampled geometry cannot manufacture a surface.
        a.normals[:]=[1.,0.,0.]
        d,supported=a.surface_distances(points)
        np.testing.assert_allclose(d,a.tree.query(points)[0]);self.assertFalse(supported.any())

    def test_continuous_recovery_uses_bounded_prior_but_does_not_weaken_cold_start(self):
        rng=np.random.default_rng(516)
        target=rng.uniform(-2.5,2.5,(6000,3))
        target[:2000,2]=-.57;target[2000:4000,0]=2.4;target[4000:,1]=-2.3
        a=ReferenceAligner.__new__(ReferenceAligner);a.target=target;a.tree=cKDTree(target)
        a.normals=np.zeros_like(target);a.normals[:2000,2]=1;a.normals[2000:4000,0]=1;a.normals[4000:,1]=1
        a.reference_kind='synthetic'
        source=np.vstack([target,rng.uniform([-.9,-.9,-1.4],[.9,.9,-1.0],(1700,3))])
        held=np.arange(len(source))%5==0;train=source[~held];test=source[held]
        seed=(np.array([.03,-.02,.01]),Rotation.from_euler('xyz',[.002,-.003,.01]).as_matrix())
        result=a.recover_continuous(train,test,np.zeros(3),seed)
        self.assertIsNotNone(result)
        t,r,q=result
        self.assertTrue(a.acceptable(q));self.assertFalse(a.initial_acceptable(q))
        self.assertEqual(q['alignment_scope'],'continuous_odometry_recovery')
        self.assertLess(np.linalg.norm(t),.01);self.assertLess(Rotation.from_matrix(r).magnitude(),.005)
        self.assertIsNone(a.recover_continuous(train,test,np.zeros(3),(np.array([.35,0,0]),np.eye(3))))
        self.assertIsNone(a.recover_continuous(train+20,test+20,np.zeros(3),seed))
        # The same prior/correction must work far from the odometry origin.
        offset=np.array([10.,-20.,3.]);shifted_seed=(seed[0]-seed[1]@offset,seed[1])
        far=a.recover_continuous(train+offset,test+offset,offset,shifted_seed)
        self.assertIsNotNone(far)
        np.testing.assert_allclose(far[0]+far[1]@offset,t,atol=1e-8)
        np.testing.assert_allclose(far[1],r,atol=1e-8)

    def test_floor_only_cannot_recover_even_with_a_previous_pose(self):
        a=ReferenceAligner.__new__(ReferenceAligner)
        x,y=np.meshgrid(np.linspace(-2,2,60),np.linspace(-2,2,60))
        a.target=np.column_stack([x.ravel(),y.ravel(),np.full(x.size,-.57)])
        a.normals=np.tile([0.,0.,1.],(len(a.target),1));a.tree=cKDTree(a.target)
        a.reference_kind='synthetic'
        self.assertIsNone(a.recover_continuous(a.target,a.target,np.zeros(3),(np.zeros(3),np.eye(3))))

    def test_two_supported_nearby_poses_do_not_silently_resume(self):
        rng=np.random.default_rng(516)
        source=rng.uniform(-2.5,2.5,(6000,3))
        source[:2000,2]=-.57;source[2000:4000,0]=2.4;source[4000:,1]=-2.3
        normals=np.zeros_like(source);normals[:2000,2]=1;normals[2000:4000,0]=1;normals[4000:,1]=1
        a=ReferenceAligner.__new__(ReferenceAligner)
        # Two plausible wall locations, 12 cm apart, with the same floor and side wall.
        a.target=np.vstack([source,source[2000:4000]+[.12,0,0]])
        a.normals=np.vstack([normals,normals[2000:4000]])
        a.tree=cKDTree(a.target);a.reference_kind='synthetic'
        held=np.arange(len(source))%5==0
        self.assertIsNone(a.recover_continuous(source[~held],source[held],np.zeros(3),
                                             (np.array([.06,0,0]),np.eye(3))))

    def test_half_metre_pose_reaches_a_narrow_refinement_basin(self):
        rng=np.random.default_rng(73)
        target=rng.uniform(-2.5,2.5,(4500,3))
        target[:1500,2]=-.57;target[1500:3000,0]=2.4;target[3000:,1]=-2.3
        aligner=ReferenceAligner.__new__(ReferenceAligner)
        aligner.target=target;aligner.tree=cKDTree(target);aligner.anchor=np.zeros(3)
        aligner.reference_kind='synthetic'
        aligner.normals=np.zeros_like(target)
        aligner.normals[:1500,2]=1;aligner.normals[1500:3000,0]=1;aligner.normals[3000:,1]=1
        truth=np.array([-.5,-.5,0.]);source=target-truth
        # Represent a repeated-structure scene whose local optimizer only
        # converges inside a 35 cm basin. The old integer lattice misses it.
        def bounded_refine(points,t,r):
            if np.linalg.norm(t-truth)>.35 or Rotation.from_matrix(r).magnitude()>.2:
                raise ValueError('outside local refinement basin')
            return MapAligner.refine_rigid(aligner,points,t,r)
        aligner.refine_rigid=bounded_refine
        t,r,q=aligner.fit_rigid(source,np.zeros(3))
        np.testing.assert_allclose(t,truth,atol=.005)
        self.assertLess(Rotation.from_matrix(r).magnitude(),.002)
        self.assertTrue(aligner.initial_acceptable(q))
        # Increased seed density must not bridge a different floor/search area.
        with self.assertRaises(ValueError):aligner.fit_rigid(source+np.array([0,0,3]),np.zeros(3))


if __name__=='__main__':unittest.main()
