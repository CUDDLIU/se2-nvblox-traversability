import math
import unittest
from dataclasses import replace
import numpy as np
from paper_pipeline import Config,Engine
from test_paper_pipeline import plane,combine,grid

def result(engine):
    return {(s.ix,s.iy,s.iz):int(mask) for r in engine.results.values() for s,mask in zip(r.spans,r.masks)}

class WindowTests(unittest.TestCase):
    def test_distant_disconnected_maps_preserve_masks_without_dense_extent_allocation(self):
        from native_nav import Classifier
        from terrain import footprint_masks
        c=Config(surface_normal_filter=True,stair_riser_filter=True)
        spans=grid();classifier=Classifier(c,footprint_masks(c));reference=classifier(spans)
        self.assertTrue(any(reference[0]))
        translated=[replace(s,ix=s.ix+10000000,iy=s.iy-10000000) for s in spans]
        actual=classifier(spans+translated)
        for first,last in ((0,len(spans)),(len(spans),2*len(spans))):
            np.testing.assert_array_equal(actual[0][first:last],reference[0])
            np.testing.assert_allclose(actual[1][first:last],reference[1],atol=1e-8)
            self.assertEqual(actual[2][first:last],reference[2])

    def engine(self,blocks,config=None,pose=(0.,0.,.45),clip=True):
        e=Engine(config or Config(tile_cells=8),native_mesh=True)
        if clip:e.current_position=pose
        e.update(blocks);e.process(budget_ms=35,defer_polygons=True,priority_position=pose)
        return e

    def test_native_dirty_clip_preserves_all_current_masks(self):
        b={(0,0,0):combine(plane(-5,-3,5,3),plane(-5,-3,5,3,3.))}
        for p in [(0.,0.,.45),(2.,0.,.45),(.2,0.,3.45)]:
            a=self.engine(b,pose=p);reference=self.engine(b,pose=p,clip=False)
            self.assertEqual(result(a),result(reference));self.assertFalse(a.pending)
            self.assertLess(a.last_metrics['processed_slabs'],reference.last_metrics['processed_slabs'])

    def test_pose_window_change_reclassifies_retained_mesh_without_new_blocks(self):
        e=self.engine({(0,0,0):plane(-5,-3,5,3)})
        e.current_position=(3.,0.,.45);e.update({})
        self.assertTrue(e.dirty_keys)
        e.process(budget_ms=35,defer_polygons=True,priority_position=e.current_position)
        self.assertTrue(any(x>30 and mask for (x,y,z),mask in result(e).items()))

    def test_patch_normals_keep_holes_ceilings_and_steep_slopes_rejected(self):
        c=Config(tile_cells=8,max_step=.2,max_slope_deg=35.,surface_normal_filter=True,stair_riser_filter=True)
        e=self.engine({(0,0,0):plane(-3,-3,3,3,slope=math.tan(math.radians(48)))},c)
        self.assertFalse(any(result(e).values()))
        e=self.engine({(0,0,0):combine(plane(-3,-3,3,3),plane(-3,-3,3,3,.6,downward=True))},c)
        self.assertFalse(any(result(e).values()))
        e=self.engine({(0,0,0):plane(-3,-3,-.2,3),(1,0,0):plane(.2,-3,3,3)},c)
        self.assertFalse(any(-2<=x<2 and mask for (x,y,z),mask in result(e).items()))
        self.assertTrue(any(result(e).values()))

    def test_observed_risers_connect_but_missing_tread_does_not(self):
        c=Config(tile_cells=8,max_step=.20,max_slope_deg=35.,stair_riser_filter=True)
        b={}
        for i in range(14):
            b[i,0,0]=plane(i*.3,-1.2,(i+1)*.3,1.2,i*.16)
            if i:
                x=i*.3
                b[i,0,1]=(np.array([[x,-1.2,(i-1)*.16],[x,1.2,(i-1)*.16],[x,1.2,i*.16],[x,-1.2,i*.16]]),np.array([[0,1,2],[0,2,3]]))
        e=self.engine(b,c,pose=(2.,0.,1.2))
        permitted={x for (x,y,z),mask in result(e).items() if y==0 and mask}
        self.assertTrue(set(range(8,33))<=permitted)
        del b[7,0,0];del b[7,0,1];del b[8,0,1]
        e=self.engine(b,c,pose=(2.,0.,1.2))
        self.assertFalse(any(x==22 and mask for (x,y,z),mask in result(e).items()))

if __name__=='__main__':unittest.main()
