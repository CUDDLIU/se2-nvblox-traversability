import sys,unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from live_occupancy import LiveOccupancy
class LiveOccupancyTests(unittest.TestCase):
    def test_mixed_cell_sizes_preserve_unresolved_coarse_obstacles(self):
        p=[[.425,0.,.025]]
        self.assertFalse(LiveOccupancy(p,.05).clear(np.zeros(4)))
        self.assertTrue(LiveOccupancy(p,[.01],.05).clear(np.zeros(4)))
        self.assertFalse(LiveOccupancy(p+[[0,.275,.025]],[.01,.05],.05).clear(np.zeros(4)))
    def test_front_obstacle_blocks_entire_forward_sweep(self):
        g=LiveOccupancy([[.8,0.,.15]])
        self.assertTrue(g.clear([0,0,0,0]))
        self.assertFalse(g.motions([(np.zeros(4),np.array([.5,0,0,0]))])[0])
    def test_rear_object_does_not_block_forward_separation(self):
        g=LiveOccupancy([[-.8,0,.15]])
        self.assertTrue(g.motions([(np.zeros(4),np.array([.5,0,0,0]))])[0])
    def test_rotation_checks_intermediate_corners(self):
        g=LiveOccupancy([[.33,.33,.15]])
        self.assertTrue(g.clear([0,0,0,0]));self.assertTrue(g.clear([0,0,0,np.pi/2]))
        self.assertFalse(g.motions([(np.zeros(4),np.array([0,0,0,np.pi/2]))])[0])
    def test_floor_is_not_a_torso_obstacle(self):
        self.assertTrue(LiveOccupancy([[0,0,-.55]]).clear([0,0,0,0]))
if __name__=='__main__':unittest.main()
