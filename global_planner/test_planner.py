import json
import math
from pathlib import Path
import tempfile
import unittest
import numpy as np

from build import build_map
from planner import Planner, NoPath
from geometry import MotionChecker
from paper_pipeline import Config


def plane(x0, y0, x1, y1, z=0.):
    return (np.array([[x0,y0,z],[x1,y0,z],[x1,y1,z],[x0,y1,z]], dtype=float),
            np.array([[0,1,2],[0,2,3]], dtype=np.int64))


def make(directory, blocks):
    arrays = {'keys': np.asarray(list(blocks), dtype=np.int32)}
    for i, (v, t) in enumerate(blocks.values()):
        arrays['v'+str(i)], arrays['t'+str(i)] = v, t
    np.savez_compressed(directory/'mesh.npz', **arrays)
    config = Config(tile_cells=8, max_step=.20, max_slope_deg=40.,
                    surface_normal_filter=True, stair_riser_filter=True)
    build_map(directory/'mesh.npz', config, directory)
    return Planner(directory)


class GlobalPlannerTests(unittest.TestCase):
    def test_free_floor_asa_exact_endpoints_and_goal_yaw(self):
        with tempfile.TemporaryDirectory() as path:
            planner = make(Path(path), {(0,0,0):plane(-3,-2,3,2)})
            result = planner.query([-1.5,0,.01,0.], [1.5,0,.01], goal_yaw=math.pi/2)
            self.assertTrue(result['success'])
            self.assertAlmostEqual(result['poses'][-1][3], math.pi/2)
            self.assertLess(result['length_m'], 3.2)
            self.assertEqual(result['motion_segments_verified'],len(result['poses'])-1)
            self.assertIsInstance(result['yaw_refinement'],dict)
            # Query vertices and their validation cache may not leak to a later query.
            self.assertEqual(len(planner.positions), planner.base_positions)

    def test_overlapping_floors_do_not_connect_without_stairs(self):
        with tempfile.TemporaryDirectory() as path:
            planner = make(Path(path), {(0,0,0):plane(-2,-2,2,2), (0,0,5):plane(-2,-2,2,2,3.)})
            with self.assertRaises(NoPath):
                planner.query([0,0,.01,0.],[0,0,3.01],timeout=15.)
            with self.assertRaises(NoPath):
                planner.project([0,0,1.5])

    def test_body_width_corridor_keeps_actual_heading_without_full_bin_proof(self):
        from test_physical_clearance import corridor
        with tempfile.TemporaryDirectory() as path:
            planner=make(Path(path),corridor())
            self.assertFalse(any(p['proven_yaw_mask'] for p in planner.polygons))
            route=planner.query([-1.,.05,.01,0.],[1.,.05,.01],goal_yaw=0.)
            self.assertTrue(route['success'])
            self.assertGreater(route['metric_certified_segments'],0)
            self.assertEqual(route['voxel_certified_segments'],0)
            for p in route['poses']:
                self.assertAlmostEqual(p[1],.05,places=7)
                self.assertAlmostEqual(math.sin(p[3]),0.,places=7)
            with self.assertRaises(NoPath):
                planner.query([-1.,.05,.01,math.pi/2],[1.,.05,.01])
            self.assertEqual(len(planner.positions),planner.base_positions)
            self.assertTrue(planner.query([-1.,.05,.01,0.],[1.,.05,.01])['success'])

    def test_changed_mesh_invalidates_saved_graph(self):
        with tempfile.TemporaryDirectory() as path:
            directory=Path(path)
            make(directory,{(0,0,0):plane(-2,-2,2,2)})
            with (directory/'mesh.npz').open('ab') as f:f.write(b'changed')
            with self.assertRaisesRegex(ValueError,'Mesh'):
                Planner(directory)

    def test_free_goal_heading_in_rotated_body_width_corridor(self):
        from test_physical_clearance import corridor
        angle=math.radians(17.3)
        with tempfile.TemporaryDirectory() as path:
            planner=make(Path(path),corridor(angle=angle,offset=.023))
            a=[-math.cos(angle),-math.sin(angle)+.023,.01,angle]
            b=[math.cos(angle),math.sin(angle)+.023,.01]
            result=planner.query(a,b)
            self.assertTrue(result['success'])
            for p in result['poses']:
                self.assertAlmostEqual(math.sin(p[3]-angle),0.,places=6)

    def test_missing_floor_cannot_be_shortened_across(self):
        blocks={(0,0,0):plane(-3,-2,-.2,2), (1,0,0):plane(.2,-2,3,2)}
        c=Config(max_step=.2,max_slope_deg=40.)
        checker=MotionChecker(blocks,c)
        self.assertFalse(checker.motion([-1,0,.01,0.],[1,0,.01,0.]))

    def test_multifloor_stairs_and_return_above_start(self):
        blocks={(0,0,0):plane(-2,-3,0,3)}
        for i in range(20):
            blocks[(i+1,0,0)]=plane(i*.35,-1.2,(i+1)*.35,1.2,(i+1)*.15)
        blocks[(30,0,0)]=plane(7,-3,9,3,3.)
        blocks[(31,0,0)]=plane(-2,1.3,7,3,3.)
        with tempfile.TemporaryDirectory() as path:
            planner=make(Path(path),blocks)
            result=planner.query([-1,2,.01,0.],[-1,2,3.01],timeout=90.)
            self.assertTrue(result['success'])
            self.assertGreater(result['height_range_m'][1]-result['height_range_m'][0],2.9)
            self.assertGreater(max(p[0] for p in result['poses']),6.9)
            self.assertGreater(result['length_m'],15.)


if __name__=='__main__':unittest.main()
