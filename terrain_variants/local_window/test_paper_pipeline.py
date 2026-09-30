import math
import unittest
from dataclasses import replace

import numpy as np

from paper_pipeline import Config, Engine, Span, classify, voxelize
from terrain import footprint_masks


def plane(x0, y0, x1, y1, z=0., slope=0., downward=False):
    v = np.array([[x0, y0, z], [x1, y0, z+slope*(x1-x0)],
                  [x1, y1, z+slope*(x1-x0)], [x0, y1, z]])
    t = np.array([[0, 1, 2], [0, 2, 3]])
    return v, t[:, ::-1] if downward else t


def combine(*meshes):
    vertices, triangles, offset = [], [], 0
    for v, t in meshes:
        vertices.append(v)
        triangles.append(t+offset)
        offset += len(v)
    return np.concatenate(vertices), np.concatenate(triangles)


def grid(x0=-14, x1=15, y0=-14, y1=15, z=0., ceiling=math.inf):
    return [Span(x, y, round(z/.01), z, ceiling, True, True)
            for x in range(x0, x1) for y in range(y0, y1)]


def signature(engine):
    return {(s.ix, s.iy, s.iz): (s.ceiling, int(mask), reason)
            for result in engine.results.values()
            for s, mask, reason in zip(result.spans, result.masks, result.reasons)}


class VoxelTests(unittest.TestCase):
    def test_two_floors_and_ceiling_intervals(self):
        c = Config()
        spans, solids = voxelize(*combine(plane(0, 0, 1, 1), plane(0, 0, 1, 1, 1.2)), c)
        column = [s for s in spans if (s.ix, s.iy) == (3, 3)]
        self.assertEqual(len(column), 2)
        self.assertEqual([s.iz for s in column], [1, 121])
        self.assertAlmostEqual(column[0].ceiling, 1.2)
        self.assertTrue(all(s.walkable for s in column))
        self.assertTrue(math.isinf(column[1].ceiling))
        self.assertEqual([a[2:] for a in solids if a[:2] == (3, 3)], [(0, 1), (120, 121)])

    def test_low_ceiling_rejects_lower_floor(self):
        spans, _ = voxelize(*combine(plane(0, 0, 1, 1),
                                    plane(0, 0, 1, 1, .99, downward=True)), Config())
        column = [s for s in spans if (s.ix, s.iy) == (3, 3)]
        self.assertFalse(column[0].walkable)
        self.assertTrue(column[0].slope_ok)
        self.assertFalse(column[1].slope_ok)

    def test_slope_comes_from_triangle_normal_without_plane_fit(self):
        for degrees, expected in ((10, True), (20, False)):
            spans, _ = voxelize(*plane(0, 0, 1, 1, slope=math.tan(math.radians(degrees))), Config())
            s = next(s for s in spans if (s.ix, s.iy) == (3, 3))
            self.assertEqual(s.slope_ok, expected)

    def test_tiny_surface_follows_voxel_abstraction_without_area_gate(self):
        spans, _ = voxelize(*plane(.021, .021, .025, .025), Config())
        self.assertEqual(len(spans), 1)
        self.assertTrue(spans[0].walkable)

    def test_merged_solid_top_surface_owns_slope(self):
        spans, _ = voxelize(*combine(plane(0, 0, 1, 1),
                                    plane(0, 0, 1, 1, .01, downward=True)), Config())
        column = [s for s in spans if (s.ix, s.iy) == (3, 3)]
        self.assertEqual(len(column), 1)
        self.assertFalse(column[0].walkable)

    def test_invalid_mesh_and_configuration(self):
        with self.assertRaises(ValueError):
            voxelize(np.array([[0, 0, 0]]), [[0, 1, 2]], Config())
        with self.assertRaises(ValueError):
            Config(vertical_resolution=.1)


class ClassificationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = Config()
        cls.masks = footprint_masks(cls.config)

    def classify(self, spans):
        g, d, r = classify(spans, self.config, self.masks)
        return {(s.ix, s.iy, s.iz): (int(mask), distance, reason)
                for s, mask, distance, reason in zip(spans, g, d, r)}

    def test_flat_interior_all_yaw_boundary_blocked(self):
        result = self.classify(grid())
        self.assertEqual(result[0, 0, 0][0], (1 << 40)-1)
        self.assertEqual(result[-14, 0, 0][0], 0)
        self.assertAlmostEqual(result[0, 0, 0][1], 1.4)

    def test_hole_blocks_full_footprint_and_distance(self):
        spans = [s for s in grid() if (s.ix, s.iy) != (2, 0)]
        result = self.classify(spans)
        self.assertEqual(result[0, 0, 0][0], 0)
        self.assertLess(result[0, 0, 0][1], .333)

    def test_narrow_corridor_preserves_only_longitudinal_yaws(self):
        # 1.0 m corridor: the 0.98 m long envelope fits only near its
        # longitudinal channels; the 0.666 m transverse envelope still fits.
        result = self.classify(grid(y0=-5, y1=5))
        mask = result[0, 0, 0][0]
        self.assertTrue(mask & 1)
        self.assertFalse(mask & (1 << 10))
        self.assertGreater(mask, 0)
        self.assertLess(mask, (1 << 40)-1)

    def test_step_bound_and_distinct_layers(self):
        for step, permitted in ((.03, True), (.04, False)):
            spans = grid()
            for s in spans:
                if s.ix >= 0:
                    s.z = step
                    s.iz = round(step/.01)
            result = self.classify(spans)
            self.assertEqual(bool(result[-1, 0, 0][0]), permitted)
        result = self.classify(grid(ceiling=1.2)+grid(z=1.2))
        self.assertEqual(result[0, 0, 0][0], (1 << 40)-1)
        self.assertEqual(result[0, 0, 120][0], (1 << 40)-1)

    def test_common_neighbor_clearance_rejects_crossing(self):
        spans = grid(ceiling=1.01)
        for s in spans:
            if s.ix >= 0:
                s.z, s.iz, s.ceiling = .02, 2, 2.
        result = self.classify(spans)
        self.assertEqual(result[-1, 0, 0][0], 0)

    def test_nonadjacent_low_roof_blocks_full_footprint(self):
        spans = grid()
        # Every individual span and adjacent pair fits the envelope, but a
        # low roof on the left and higher floor on the right do not share
        # a complete robot-height free interval across the whole footprint.
        for s in spans:
            if s.ix <= -3:
                s.ceiling = 1.02
            if s.ix >= 3:
                s.z, s.iz = .03, 3
        result = self.classify(spans)
        self.assertEqual(result[0, 0, 0][0], 0)


class IncrementalTests(unittest.TestCase):
    config = Config(length=.20, width=.16, side_margin=0., height=.2,
                    top_margin=0., yaw_bins=8, sweep_sample_deg=3.)

    def test_partial_update_matches_full_rebuild_and_preserves_distant_tile(self):
        blocks = {(0, 0, 0): plane(-1.5, -1.5, 1.5, 1.5),
                  (20, 0, 0): plane(10, -1.5, 13, 1.5)}
        e = Engine(self.config)
        e.update(blocks)
        e.process(10000)
        before = signature(e)
        far = {k: r for k, r in e.results.items() if k[0] >= 5}
        obstacle = plane(-.15, -.15, .15, .15, .1, downward=True)
        e.update({(0, 0, 1): obstacle})
        e.process(10000)
        fresh = Engine(self.config)
        fresh.update({**blocks, (0, 0, 1): obstacle})
        fresh.process(10000)
        self.assertEqual(signature(e), signature(fresh))
        self.assertNotEqual(before, signature(e))
        for k, r in far.items():
            self.assertIs(e.results[k], r)

    def test_unchanged_delete_clear(self):
        mesh = plane(-1, -1, 1, 1)
        e = Engine(self.config)
        e.update({(0, 0, 0): mesh})
        e.process(10000)
        self.assertTrue(e.snapshot()['polygons'])
        e.update({(0, 0, 0): mesh})
        self.assertFalse(e.pending)
        e.update({(0, 0, 0): ([], [])})
        e.process(10000)
        self.assertFalse(e.results)
        self.assertFalse(e.snapshot()['graph']['nodes'])
        e.update({(0, 0, 0): mesh})
        e.process(10000)
        e.update({}, clear=True)
        self.assertFalse(e.results)
        self.assertFalse(e.index.blocks)
        self.assertEqual(e.generation, 1)

    def test_tile_seams_match_global_classification(self):
        # Ground is close to a slab top; obstacle is across the slab boundary.
        mesh = combine(plane(-1.2, -1.2, 2.8, 1.2, .79),
                       plane(1.48, -.2, 1.75, .2, .94, downward=True))
        spans, _ = voxelize(*mesh, self.config)
        masks, _, _ = classify(spans, self.config, footprint_masks(self.config))
        expected = {(s.ix, s.iy, s.iz): int(m) for s, m in zip(spans, masks)}
        e = Engine(self.config)
        e.update({(0, 0, 0): mesh})
        e.process(10000)
        self.assertEqual({k: v[1] for k, v in signature(e).items()}, expected)

    def test_vertical_siblings_grouped_match_unbounded_processing(self):
        # Two separated layers in one XY tile force multiple dirty Z slabs.
        mesh = combine(plane(-1.2, -1.2, 2.8, 1.2, 0.),
                       plane(-1.2, -1.2, 2.8, 1.2, 1.35))
        grouped = Engine(self.config)
        grouped.update({(0, 0, 0): mesh})
        grouped.process(1)  # one XY group may complete several Z siblings
        while grouped.pending:
            grouped.process(1)
        unbounded = Engine(self.config)
        unbounded.update({(0, 0, 0): mesh})
        unbounded.process(10000)
        self.assertEqual(signature(grouped), signature(unbounded))
        self.assertEqual(grouped.snapshot()['polygons'], unbounded.snapshot()['polygons'])
        self.assertGreaterEqual(grouped.last_metrics['processed_xy_groups'], 1)

    def test_grouped_layers_walls_and_overhang_match_global_geometry(self):
        wall = (np.array([[.7, -.5, -.1], [.7, .5, -.1],
                          [.7, .5, 2.2], [.7, -.5, 2.2]]),
                np.array([[0, 1, 2], [0, 2, 3]]))
        mesh = combine(plane(-1.2, -1.2, 2.8, 1.2, .79),
                       plane(-1.2, -1.2, 2.8, 1.2, 1.79),
                       plane(1.48, -.2, 1.75, .2, .94, downward=True), wall)
        spans, _ = voxelize(*mesh, self.config)
        masks, _, _ = classify(spans, self.config, footprint_masks(self.config))
        expected = {(s.ix, s.iy, s.iz): int(mask) for s, mask in zip(spans, masks)}
        engine = Engine(self.config)
        engine.update({(0, 0, 0): mesh})
        pending = set(engine.pending)
        self.assertEqual(engine.process(0)['processed_slabs'], 0)
        self.assertEqual(engine.pending, pending)
        while engine.pending:
            engine.process(1)
        self.assertEqual({key: value[1] for key, value in signature(engine).items()}, expected)


if __name__ == '__main__':
    unittest.main()
