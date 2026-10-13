import sys,unittest
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from shapely.geometry import box
from shapely.strtree import STRtree
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from corridor import encode_corridor

class CorridorEncoding(unittest.TestCase):
    def test_partial_cells_export_only_pose_witnesses(self):
        shapes=[box(0,0,1,1),box(1,0,2,1),box(0,0,1,1)]
        p=SimpleNamespace(bins=40,config=SimpleNamespace(max_step=.2,vertical_resolution=.01),shapes=shapes,tree=STRtree(shapes),
          polygons=[dict(floor=0.,proven_yaw_mask=1,vertices=[[0,0,0],[1,0,0],[1,1,0],[0,1,0]]),
                    dict(floor=0.,proven_yaw_mask=0,yaw_mask=(1<<40)-1),
                    dict(floor=3.,proven_yaw_mask=1)],
          region_positions={0:[],1:[0],2:[]},witness_cells={0:(10,0,0)},
          positions=[np.array([1.04,.05,0.])],angles=[{0:0.}])
        data=encode_corridor(p,[[0,0,.57,0],[1.5,0,.57,0]],.57,123.)
        self.assertEqual(data[:5],[1.,123.,40.,.2+1e-9,2.])
        self.assertEqual(data[5:9],[.57,1.,0.,4.])
        self.assertEqual(data[17:21],[.57,1.,0.,1.])
        np.testing.assert_allclose(data[21:],[1.04,.05])

if __name__=='__main__':unittest.main()
