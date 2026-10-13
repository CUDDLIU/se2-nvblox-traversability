import sys,unittest
from pathlib import Path
from types import SimpleNamespace
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from observed_mesh import ObservedVoxels

class Observations(unittest.TestCase):
    def records(self, ids, state=0):
        ids=np.asarray(ids).reshape(-1,3)
        return np.column_stack(((ids+.5)*.05,np.full(len(ids),.05),np.full(len(ids),state)))
    def test_unknown_and_occupied_do_not_clear_saved_faces(self):
        ids=np.array([[x,y,z] for x in range(3) for y in range(3) for z in range(3)])
        a=self.records(ids)
        lo=np.array([[.051,.051,.051]]);hi=np.array([[.099,.099,.099]])
        self.assertTrue(ObservedVoxels(a).free_boxes(lo,hi)[0])
        center=np.all(ids==[1,1,1],axis=1)
        self.assertFalse(ObservedVoxels(a[~center]).free_boxes(lo,hi)[0])
        a[center,4]=1
        self.assertFalse(ObservedVoxels(a).free_boxes(lo,hi)[0])
        self.assertFalse(ObservedVoxels(a).free_boxes([[-.1,0,0]],[[0,0,0]])[0])
    def test_full_triangle_extent_not_centroid_and_boundary_both_sides(self):
        a=self.records([[0,0,0],[1,0,0]])
        obs=ObservedVoxels(a)
        self.assertTrue(obs.free_boxes([[.05,.01,.01]],[[.05,.02,.02]])[0])
        self.assertFalse(ObservedVoxels(a[:1]).free_boxes([[.05,.01,.01]],[[.05,.02,.02]])[0])
        self.assertFalse(obs.free_boxes([[.01,.01,.01]],[[.12,.01,.01]])[0])
    def test_conflicting_occupied_wins(self):
        a=np.concatenate((self.records([[0,0,0]]),self.records([[0,0,0]],1)))
        self.assertFalse(ObservedVoxels(a).free_boxes([[.01]*3],[[.04]*3])[0])
    def test_support_requires_four_neighbours_same_layer_and_slope(self):
        a=self.records([[0,0,0],[1,0,0],[1,1,0],[0,1,0]],1)
        self.assertEqual(len(ObservedVoxels(a).support_triangles(.025,.2,30)),2)
        self.assertEqual(len(ObservedVoxels(a[:3]).support_triangles(.025,.2,30)),0)
        self.assertEqual(len(ObservedVoxels(a).support_triangles(3.,.2,30)),0)
        a[2,2]+=.1
        self.assertEqual(len(ObservedVoxels(a).support_triangles(.025,.2,30)),0)
    def test_fusion_retains_unobserved_wall_and_adds_no_unseen_ground(self):
        a=self.records([[0,0,0]])
        triangles=np.array([[[.01,.01,.01],[.04,.01,.01],[.01,.04,.01]],
                            [[.11,.01,.01],[.14,.01,.01],[.11,.04,.01]]])
        fused,report=ObservedVoxels(a).fuse(triangles,0,SimpleNamespace(max_step=.2,max_slope_deg=30))
        np.testing.assert_allclose(fused,triangles[1:]);self.assertEqual(report['removed_triangles'],1)
        self.assertEqual(report['support_triangles'],0)
if __name__=='__main__':unittest.main()
