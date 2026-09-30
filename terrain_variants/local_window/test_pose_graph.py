import math
import unittest
from test_physical_clearance import corridor,run


def reachable(graph,source):
    neighbors={}
    for e in graph['edges']:
        neighbors.setdefault(e['source'],[]).append(e['target']);neighbors.setdefault(e['target'],[]).append(e['source'])
    visited={source};todo=[source]
    while todo:
        for n in neighbors.get(todo.pop(),[]):
            if n not in visited:visited.add(n);todo.append(n)
    return visited

class PoseGraphTests(unittest.TestCase):
    def test_narrow_corridor_joins_actual_pose_witnesses(self):
        for angle,offset in [(0.,.05),(math.radians(17.3),.023)]:
            e=run(corridor(angle=angle,offset=offset));g=e.snapshot()['graph']
            nodes=[n for n in g['nodes'] if abs(math.remainder(n['yaw_rad']-angle,2*math.pi))<1e-6]
            self.assertGreater(len(nodes),10)
            a=min(nodes,key=lambda n:n['position'][0]);b=max(nodes,key=lambda n:n['position'][0])
            self.assertIn(b['id'],reachable(g,a['id']))
            self.assertFalse(any(q['kind']=='rotation' for q in g['edges']))
            self.assertEqual(g['translation_model'],'metric_pose_swept_body')

    def test_no_connection_across_real_gap_or_obstacle(self):
        for blocks in [corridor(gap=True),corridor(obstacle=True)]:
            g=run(blocks).snapshot()['graph'];nodes=[n for n in g['nodes'] if abs(n['yaw_rad'])<1e-6]
            self.assertTrue(nodes)
            a=min(nodes,key=lambda n:n['position'][0]);b=max(nodes,key=lambda n:n['position'][0])
            self.assertNotIn(b['id'],reachable(g,a['id']))

if __name__=='__main__':unittest.main()
