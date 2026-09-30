import sys,unittest
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from shapely.geometry import box
from shapely.strtree import STRtree
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from static_guard import StaticGuard
class Proofs(unittest.TestCase):
    def guard(self,mask):
        shapes=[box(0,0,1,1)]
        p=SimpleNamespace(config=SimpleNamespace(vertical_resolution=.01),bins=40,
          polygons=[dict(floor=-1.,proven_yaw_mask=mask,yaw_mask=(1<<40)-1)],
          shapes=shapes,tree=STRtree(shapes),checker=SimpleNamespace(motions=lambda pairs:np.zeros(len(pairs),bool)))
        return StaticGuard(p)
    def test_refined_bits_never_become_polygon_proof(self):
        a=np.array([.1,.1,-1,0]);b=np.array([.9,.9,-1,0])
        self.assertFalse(self.guard(0).motions([(a,b)])[0]);self.assertTrue(self.guard(1).motions([(a,b)])[0])
    def test_rotation_and_other_floor_require_their_own_proofs(self):
        a=np.array([.1,.1,-1,0]);b=np.array([.9,.9,-1,np.pi/2])
        self.assertFalse(self.guard(1).motions([(a,b)])[0])
        self.assertTrue(self.guard((1<<40)-1).motions([(a,b)])[0])
        b[2]=2;self.assertFalse(self.guard((1<<40)-1).motions([(a,b)])[0])
if __name__=='__main__':unittest.main()
