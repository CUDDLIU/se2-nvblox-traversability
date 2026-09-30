import math
import unittest
from dataclasses import dataclass

import numpy as np
from shapely.geometry import Point, Polygon, box
from shapely.ops import unary_union

from paper_polygons import build_graph, build_polygons


@dataclass
class Span:
    ix: int
    iy: int
    iz: int = 0
    z: float = 0.0
    ceiling: float = math.inf
    walkable: bool = True


def geometry(polygons):
    return unary_union([Polygon([(v[0], v[1]) for v in p['vertices']]) for p in polygons])


def reachable(graph, start):
    edges = {}
    for e in graph['edges']:
        edges.setdefault(e['source'], []).append(e['target'])
        edges.setdefault(e['target'], []).append(e['source'])
    seen, pending = {start}, [start]
    while pending:
        for p in edges.get(pending.pop(), []):
            if p not in seen:
                seen.add(p)
                pending.append(p)
    return seen


class PolygonTest(unittest.TestCase):
    def build(self, spans, masks=None, distances=None, tile=(0, 0, 0)):
        return build_polygons(spans, [15] * len(spans) if masks is None else masks,
                              [1] * len(spans) if distances is None else distances, 1.0, tile)

    def test_flat_floor_merges_exactly_and_ids_stable(self):
        spans = [Span(x, y) for x in range(4) for y in range(3)]
        polygons = self.build(spans)
        self.assertEqual(len(polygons), 1)
        self.assertEqual(len(polygons[0]['vertices']), 4)
        self.assertAlmostEqual(geometry(polygons).area, 12)
        self.assertEqual(polygons, self.build(list(reversed(spans))))
        graph = build_graph(polygons, 4, .03, 1)
        self.assertEqual(len(graph['nodes']), 4)
        self.assertEqual(len(reachable(graph, graph['nodes'][0]['id'])), 4)

    def test_unbounded_distance_plateau_is_one_region(self):
        spans = [Span(x, y) for x in range(4) for y in range(3)]
        polygons = self.build(spans, distances=[math.inf] * len(spans))
        self.assertEqual(len(polygons), 1)
        self.assertAlmostEqual(geometry(polygons).area, 12)

    def test_yaw_angles_match_centered_footprint_channels(self):
        polygons = self.build([Span(0, 0)])
        graph = build_graph(polygons, 4, .03, 1)
        angles = {node['yaw']: node['yaw_rad'] for node in graph['nodes']}
        for yaw in range(4):
            self.assertAlmostEqual(angles[yaw], yaw * math.pi / 2)

    def test_hole_is_never_filled_by_merge_or_graph(self):
        spans = [Span(x, y) for x in range(5) for y in range(5) if (x, y) != (2, 2)]
        polygons = self.build(spans)
        geom = geometry(polygons)
        self.assertAlmostEqual(geom.area, 24)
        self.assertFalse(geom.covers(Point(2.5, 2.5)))
        for p in polygons:
            shape = Polygon(p['vertices'])
            self.assertAlmostEqual(shape.area, shape.convex_hull.area)
        graph = build_graph(polygons, 4, .03, 1)
        self.assertEqual(len(reachable(graph, graph['nodes'][0]['id'])), len(graph['nodes']))
        by_id = {n['id']: n for n in graph['nodes']}
        from shapely.geometry import LineString
        for e in graph['edges']:
            if e['kind'] == 'translation':
                a, b = by_id[e['source']]['position'], by_id[e['target']]['position']
                self.assertTrue(geom.buffer(1e-10).covers(LineString([a[:2], b[:2]])))

    def test_diagonal_touch_does_not_connect(self):
        polygons = self.build([Span(0, 0), Span(1, 1)])
        self.assertEqual(len(polygons), 2)
        graph = build_graph(polygons, 4, .03, 1)
        self.assertEqual(len(reachable(graph, graph['nodes'][0]['id'])), 4)
        self.assertFalse(any(e['kind'] == 'translation' for e in graph['edges']))

    def test_mixed_yaw_only_shared_channel_connects(self):
        polygons = self.build([Span(0, 0), Span(1, 0)], [3, 6])
        self.assertEqual(len(polygons), 2)
        graph = build_graph(polygons, 4, .03, 1)
        portals = [n for n in graph['nodes'] if n['kind'] == 'portal']
        self.assertEqual([n['yaw'] for n in portals], [1])
        self.assertEqual(len(reachable(graph, graph['nodes'][0]['id'])), len(graph['nodes']))

    def test_no_rotation_across_missing_channel(self):
        polygons = self.build([Span(0, 0)], [5])
        graph = build_graph(polygons, 4, .03, 1)
        self.assertEqual(len(graph['nodes']), 2)
        self.assertFalse(graph['edges'])

    def test_multilevel_floors_remain_separate(self):
        polygons = self.build([Span(0, 0), Span(0, 0, 20, 2.0)])
        self.assertEqual(len(polygons), 2)
        graph = build_graph(polygons, 4, .03, 1)
        self.assertEqual(len(reachable(graph, graph['nodes'][0]['id'])), 4)

    def test_tile_seam_and_vertical_step(self):
        left = self.build([Span(0, 0)], tile=(0, 0, 0))
        right = self.build([Span(1, 0, 1, .02)], tile=(1, 0, 0))
        graph = build_graph(left + right, 4, .03, 1)
        self.assertEqual(len(reachable(graph, graph['nodes'][0]['id'])), len(graph['nodes']))
        rejected = build_graph(left + right, 4, .01, 1)
        self.assertFalse(any(n['kind'] == 'portal' for n in rejected['nodes']))

    def test_shared_clearance_must_fit_at_higher_floor(self):
        left = self.build([Span(0, 0, 0, 0, 1.01)], tile=(0, 0, 0))
        right = self.build([Span(1, 0, 1, .02, 2)], tile=(1, 0, 0))
        graph = build_graph(left + right, 4, .03, 1)
        self.assertFalse(any(n['kind'] == 'portal' for n in graph['nodes']))

    def test_distance_maxima_create_connected_basins(self):
        spans = [Span(x, 0) for x in range(7)]
        polygons = self.build(spans, distances=[1, 3, 2, 1, 2, 3, 1])
        self.assertEqual(len({p['region'] for p in polygons}), 2)
        self.assertAlmostEqual(geometry(polygons).area, 7)
        graph = build_graph(polygons, 4, .03, 1)
        self.assertEqual(len(reachable(graph, graph['nodes'][0]['id'])), len(graph['nodes']))

    def test_nonwalkable_and_zero_mask_omitted(self):
        polygons = self.build([Span(0, 0), Span(1, 0), Span(2, 0, walkable=False)], [15, 0, 15])
        self.assertAlmostEqual(geometry(polygons).area, 1)

    def test_metric_grid_negative_coordinates_and_seams(self):
        left = build_polygons([Span(-1, -2, -3, -.03)], [15], [1], .1, (-1, -1, -1))
        right = build_polygons([Span(0, -2, -1, -.01)], [15], [1], .1, (0, -1, -1))
        self.assertAlmostEqual(geometry(left + right).area, .02)
        graph = build_graph(left + right, 4, .03, 1)
        self.assertEqual(len(reachable(graph, graph['nodes'][0]['id'])), len(graph['nodes']))

    def test_empty_input_has_empty_graph(self):
        polygons = self.build([])
        self.assertEqual(polygons, [])
        graph = build_graph(polygons, 4, .03, 1)
        self.assertFalse(graph['nodes'])
        self.assertFalse(graph['edges'])

    def test_random_masks_exactly_preserve_cells(self):
        rng = np.random.default_rng(734)
        for _ in range(6):
            cells = [(x, y) for x in range(7) for y in range(7) if rng.random() > .25]
            spans = [Span(x, y) for x, y in cells]
            polygons = self.build(spans, distances=rng.random(len(spans)))
            expected = unary_union([box(x, y, x + 1, y + 1) for x, y in cells])
            self.assertAlmostEqual(geometry(polygons).symmetric_difference(expected).area, 0)
            for p in polygons:
                shape = Polygon(p['vertices'])
                self.assertAlmostEqual(shape.area, shape.convex_hull.area)


if __name__ == '__main__':
    unittest.main()
