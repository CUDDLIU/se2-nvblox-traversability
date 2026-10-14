import copy
import json
import unittest
from pathlib import Path
import numpy as np
from navigation_modes import TerrainAnnotations, arc_lengths, envelope, mode, validate_route


def plane(a,b,z,z1=None):
    return (np.array([[a,-1,z],[b,-1,z if z1 is None else z1],
                      [b,1,z if z1 is None else z1],[a,1,z]],float),
            np.array([[0,1,2],[0,2,3]]))


class TerrainModes(unittest.TestCase):
    def test_treads_up_down_and_preparation(self):
        blocks={0:plane(-2,0,0),1:plane(0,.3,.15),2:plane(.3,.6,.3),3:plane(.6,3,.45)}
        poses=[[-1.5,0,0,0],[0,0,.15,0],[.3,0,.3,0],[.6,0,.45,0],[2,0,.45,0]]
        for path,label in ((poses,'stair_up'),(poses[::-1],'stair_down')):
            result=TerrainAnnotations(blocks).annotate(path)
            self.assertEqual([m['terrain'] for m in result],['flat',label,'flat'])
            self.assertEqual(result[1]['Gait'],0x3003)
            self.assertLess(result[1]['start_m'],1.4)
            validate_route(envelope(path,result,(1,0),'se2_scan_map'))

    def test_flat_z_noise_single_bump_and_ramp_do_not_trigger(self):
        scenes=[({0:plane(-2,3,0)},[[-1,0,0,0],[0,0,.13,0],[2,0,0,0]]),
                ({0:plane(-2,0,0),1:plane(0,.3,.13),2:plane(.3,3,0)},
                 [[-1,0,0,0],[0,0,.13,0],[.3,0,0,0],[2,0,0,0]]),
                ({0:plane(-2,3,0,1)},[[-1,0,.2,0],[2,0,.8,0]])]
        for blocks,poses in scenes:
            self.assertEqual([m['terrain'] for m in TerrainAnnotations(blocks).annotate(poses)],['flat'])

    def test_overlapping_floor_uses_path_level(self):
        blocks={0:plane(-2,0,0),1:plane(0,.3,.15),2:plane(.3,.6,.3),
                3:plane(.6,3,.45),4:plane(-2,3,3)}
        path=[[-1,0,3,0],[2,0,3,0]]
        self.assertEqual(TerrainAnnotations(blocks).annotate(path),[mode(0,3)])

    def test_real_tsdf_failed_routes_and_floor_routes(self):
        directory=Path(__file__).with_name('fixtures')/'stair_scene_20261014'
        with np.load(directory/'mesh_corridors.npz') as mesh:
            classifier=TerrainAnnotations({0:(mesh['vertices'],mesh['triangles'])})
        fixture=json.loads((directory/'routes.json').read_text())
        for query in fixture['queries']:
            for reverse in (False,True):
                with self.subTest(query=query['query'],reverse=reverse):
                    path=query['poses'][::-1] if reverse else query['poses']
                    result=classifier.annotate(path)
                    label=('stair_down' if reverse else 'stair_up') if query['expected']!='flat' else 'flat'
                    self.assertEqual([m['terrain'] for m in result],
                                     ['flat',label,'flat'] if label!='flat' else ['flat'])
                    self.assertGreater(classifier.diagnostics['mesh_coverage'],.97)
                    validate_route(envelope(path,result,(1,0),'se2_scan_map'))
                    if query['query']=='20261014_152435_229875.json' and not reverse:
                        self.assertLess(result[1]['start_m'],fixture['failed_route_progress_m']-.3)
                        # Both flights and the intervening landing keep one gait.
                        self.assertGreater(result[1]['end_m'],10.5)
                        self.assertEqual(classifier.diagnostics['profile_flights'],2)

    def test_smoothed_climb_is_mesh_supported_and_not_a_tall_discontinuity(self):
        blocks={0:plane(-2,0,0),1:plane(0,2,0,1.2),2:plane(2,4,1.2)}
        path=[[-1,0,0,0],[0,0,0,0],[2,0,1.2,0],[3,0,1.2,0]]
        result=TerrainAnnotations(blocks).annotate(path)
        self.assertEqual([m['terrain'] for m in result],['flat','stair_up','flat'])
        # Route heights alone cannot create a stair label on a flat mesh.
        self.assertEqual(TerrainAnnotations({0:plane(-2,4,0)}).annotate(path),[mode(0,4)])
        wall={0:plane(-2,0,0),1:plane(0,4,.6)}
        self.assertEqual(TerrainAnnotations(wall).annotate([[-1,0,0,0],[0,0,.6,0],[3,0,.6,0]]),[mode(0,4)])

    def test_mesh_gap_cannot_join_insufficient_climb_evidence(self):
        blocks={0:plane(-1,0,0),1:plane(0,.4,0,.24),2:plane(.8,1.2,.48,.72),3:plane(1.2,2,.72)}
        path=[[-1,0,0,0],[0,0,0,0],[1.2,0,.72,0],[2,0,.72,0]]
        self.assertEqual(TerrainAnnotations(blocks).annotate(path),[mode(0,3)])

    def test_atomic_envelope_rejects_gaps_disabled_avoidance_and_nonfinite(self):
        doc=envelope([[0,0,0,0],[2,0,0,0]],[mode(0,2)],(1,2),'se2_scan_map')
        validate_route(doc)
        for change in ('gap','avoidance','nan'):
            bad=copy.deepcopy(doc)
            if change=='gap':bad['modes'][0]['end_m']=1
            elif change=='avoidance':bad['modes'][0]['ObsMode']=1
            else:bad['poses'][0][0]=float('nan')
            with self.assertRaises(ValueError):validate_route(bad)
        validate_route(envelope([],[],(1,3),'se2_scan_map'))


if __name__=='__main__':unittest.main()
