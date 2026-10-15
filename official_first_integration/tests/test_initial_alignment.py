"""Cold start must find other mapped locations and reject look-alike floors."""
from pathlib import Path
import sys
import unittest
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'scanplanner_integration'),str(ROOT/'official_first_integration')]
from initial_alignment import GlobalInitialSearch
from reference_alignment import ReferenceAligner


def room():
    rng=np.random.default_rng(615)
    points=rng.uniform([-3.,-2.,-.5],[3.,4.,2.],(6900,3))
    normals=np.zeros_like(points)
    points[:2000,2]=-.57;normals[:2000,2]=1.
    points[2000:4000,0]=2.6;normals[2000:4000,0]=1.
    points[4000:6000,1]=-1.7;normals[4000:6000,1]=1.
    points[6000:,1]=rng.uniform(.8,2.,900);points[6000:,0]=.4+.4*points[6000:,1]
    normals[6000:]=np.array([1.,-.4,0.])/np.sqrt(1.16)
    return points,normals


def aligner(points,normals,seeds):
    a=ReferenceAligner.__new__(ReferenceAligner)
    a.target=points;a.normals=normals;a.tree=cKDTree(points);a.reference_kind='synthetic'
    a.anchor=np.zeros(3)
    a.global_search=GlobalInitialSearch(points,seeds)
    return a


class WholeMapInitialization(unittest.TestCase):
    def test_other_floor_and_heading_without_a_recording_start_hint(self):
        points,normals=room();position=np.array([9.,-8.,3.2])
        r=Rotation.from_euler('xyz',[.01,-.015,1.1]).as_matrix()
        a=aligner(points+position,normals,np.array([[0.,0.,0.],position+[.2,-.2,0.]]))
        t,actual,q=a.fit_rigid(points@r,np.zeros(3))
        np.testing.assert_allclose(t,position,atol=.015)
        self.assertLess(Rotation.from_matrix(actual@r.T).magnitude(),.006)
        self.assertEqual(q['initial_search_scope'],'whole_map')
        self.assertGreater(q['initial_observability_ratio'],.01)

    def test_identical_rooms_on_different_floors_are_ambiguous(self):
        points,normals=room();first=np.array([9.,-8.,3.2]);second=first+[0,0,4.]
        a=aligner(np.vstack([points+first,points+second]),np.vstack([normals,normals]),
                  np.array([first,second]))
        with self.assertRaisesRegex(ValueError,'ambiguous whole-map'):
            a.fit_rigid(points,np.zeros(3))

    def test_floor_alone_and_unrelated_scene_cannot_initialize(self):
        x,y=np.meshgrid(np.linspace(-3,3,65),np.linspace(-3,3,65))
        points=np.column_stack([x.ravel(),y.ravel(),np.full(x.size,-.57)])
        normals=np.tile([0.,0.,1.],(len(points),1))
        a=aligner(points,normals,np.array([[0.,0.,0.]]))
        with self.assertRaises(ValueError):a.fit_rigid(points,np.zeros(3))
        with self.assertRaises(ValueError):a.fit_rigid(points+30,np.zeros(3))

    def test_reference_outliers_do_not_allocate_an_unbounded_distance_grid(self):
        points,normals=room();points=np.vstack([points,[10000.,10000.,10000.]])
        search=GlobalInitialSearch(points,np.array([[0.,0.,0.]]))
        self.assertLessEqual(search.field.size,8_000_000)
        self.assertLess(search.field.nbytes,32_000_001)


if __name__=='__main__':unittest.main()
