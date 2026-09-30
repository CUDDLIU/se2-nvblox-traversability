"""An upstairs boundary must not erase unchanged support downstairs."""
import math
import unittest
from dataclasses import replace
import numpy as np
from paper_pipeline import Config,Span,classify_reference
from native_nav import Classifier
from terrain import footprint_masks


def surface(x,y,z,roof=math.inf):
    return Span(x,y,round(z*100),z,roof,True,True)


def folded_floors(hole=None):
    lower=[surface(x,y,0.,3.) for x in range(-15,16) for y in range(-15,16)]
    upper=[surface(x,y,3.) for x in range(-15,16) for y in range(-15,16) if (x,y)!=hole]
    # The distant switchback joins the floors without joining their overlapping
    # columns. Its narrow walkway is not certified for a full robot footprint.
    stairs=([surface(x,0,(x-15)*.1) for x in range(16,31)]+
            [surface(30,y,1.5+y*.1) for y in range(1,16)]+
            [surface(x,15,3.) for x in range(16,30)])
    return lower+upper+stairs


class FoldedFloorTests(unittest.TestCase):
    def setUp(self):
        self.c=Config(tile_cells=8,max_step=.2,max_slope_deg=40.)
        self.classifier=Classifier(self.c,footprint_masks(self.c))

    def test_upstairs_hole_preserves_downstairs_masks_and_distance(self):
        plain=folded_floors();changed=folded_floors((1,0))
        roots=[i for i,s in enumerate(plain[:961]) if abs(s.ix)<6 and abs(s.iy)<6]
        before=self.classifier(plain,roots);after=self.classifier(changed,roots)
        np.testing.assert_array_equal(after[0][roots],before[0][roots])
        np.testing.assert_allclose(after[1][roots],before[1][roots])
        center=next(i for i,s in enumerate(changed) if (s.ix,s.iy,s.z)==(0,0,0.))
        self.assertEqual(int(after[0][center]),(1<<self.c.yaw_bins)-1)
        self.assertAlmostEqual(after[1][center],1.5)

    def test_same_floor_hole_still_rejects(self):
        spans=[s for s in folded_floors() if (s.ix,s.iy,s.z)!=(1,0,0.)]
        root=next(i for i,s in enumerate(spans) if (s.ix,s.iy,s.z)==(0,0,0.))
        masks,distance,_=self.classifier(spans,[root])
        self.assertEqual(int(masks[root]),0);self.assertEqual(distance[root],0.)

    def test_low_roof_still_rejects(self):
        spans=folded_floors((1,0))
        spans=[replace(s,ceiling=.7,walkable=False) if s.z==0 else s for s in spans]
        root=next(i for i,s in enumerate(spans) if (s.ix,s.iy,s.z)==(0,0,0.))
        self.assertEqual(int(self.classifier(spans,[root])[0][root]),0)

    def test_flat_distance_matches_euclidean_and_reference_masks(self):
        spans=[surface(x,y,0.) for x in range(-9,10) for y in range(-9,10)]
        got=self.classifier(spans)
        expected=np.array([min(9-abs(s.ix),9-abs(s.iy))*.1 for s in spans])
        np.testing.assert_allclose(got[1],expected,atol=1e-12)
        reference=classify_reference(spans,self.c,footprint_masks(self.c))
        np.testing.assert_array_equal(got[0],reference[0])
        np.testing.assert_allclose(got[1],reference[1],atol=1e-12)


if __name__=='__main__':unittest.main()
