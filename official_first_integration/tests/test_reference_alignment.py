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
