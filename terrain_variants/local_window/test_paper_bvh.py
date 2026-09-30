import unittest

import numpy as np

from paper_bvh import MeshIndex


class MeshIndexTests(unittest.TestCase):
    def test_native_cached_spans_match_uncached_query_after_edits(self):
        from paper_native import voxel_spans
        rng=np.random.default_rng(411)
        index=MeshIndex()
        for i in range(12):
            vertices=(rng.uniform(-.16,.16,(12,3,3))+rng.uniform(-1,1,(1,1,3))).reshape(-1,3)
            index.replace((i,),vertices,np.arange(len(vertices)).reshape(-1,3))
        queries=[(np.full(3,-3.),np.full(3,3.))]
        for _ in range(25):
            low=rng.uniform(-1.2,.3,3);queries.append((low,low+rng.uniform(.1,1.,3)))
        for slope,height in ((.96,1.),(.8,.7),(.96,1.)):
            for low,high in queries:
                v,t=index.query(low,high)
                expected=voxel_spans(v,t,.1,.01,slope,height)
                got=index.query_spans(low,high,.1,.01,slope,height)
                np.testing.assert_array_equal(got,expected)
        index.replace((0,),[],[])
        index.replace((1,),[[0,0,.1],[1,0,.1],[0,1,.1]],[[0,1,2]])
        low,high=queries[0]
        np.testing.assert_array_equal(index.query_spans(low,high,.1,.01,.96,1.),
            voxel_spans(*index.query(low,high),.1,.01,.96,1.))

    def test_queries_match_brute_force_and_are_deterministic(self):
        rng = np.random.default_rng(732)
        index = MeshIndex()
        all_points = []
        for i in range(12):
            points = rng.normal(size=(80, 3, 3)) * 0.15 + rng.uniform(-5, 5, size=3)
            index.replace((i, 0, 0), points.reshape((-1, 3)),
                          np.arange(points.size // 3).reshape((-1, 3)))
            all_points.extend(points)
        all_points = np.array(all_points)
        for _ in range(45):
            low = rng.uniform(-6, 4, size=3)
            high = low + rng.uniform(0.1, 4, size=3)
            expected = all_points[np.all(all_points.max(axis=1) >= low, axis=1)
                                  & np.all(all_points.min(axis=1) <= high, axis=1)]
            vertices, triangles = index.query(low, high)
            actual = vertices[triangles]
            as_set = lambda x: sorted(tuple(t.ravel()) for t in x)
            self.assertEqual(as_set(expected), as_set(actual))
            again_v, again_t = index.query(low, high)
            np.testing.assert_array_equal(vertices, again_v)
            np.testing.assert_array_equal(triangles, again_t)
        self.assertEqual(index.stats["top_level_rebuilds"], 1)

    def test_delete_replace_clear_and_lazy_rebuild(self):
        index = MeshIndex()
        vertices = np.array([[0., 0, 0], [1, 0, 0], [0, 1, 0]])
        triangles = [[0, 1, 2]]
        old, new = index.replace((0, 0, 0), vertices, triangles)
        self.assertIsNone(old)
        np.testing.assert_equal(new, ([0, 0, 0], [1, 1, 0]))
        self.assertEqual(index.stats["top_level_rebuilds"], 0)
        vertices[:] = 99  # callers cannot mutate indexed geometry
        self.assertEqual(len(index.query([0, 0, 0], [1, 1, 0])[1]), 1)
        replacement = np.array([[0., 0, 2], [1, 0, 2], [0, 1, 2]])
        old, new = index.replace((0, 0, 0), replacement, triangles)
        np.testing.assert_equal(old, ([0, 0, 0], [1, 1, 0]))
        np.testing.assert_equal(new, ([0, 0, 2], [1, 1, 2]))
        self.assertEqual(index.stats["top_level_rebuilds"], 1)
        self.assertEqual(len(index.query([0, 0, 0], [1, 1, 0])[1]), 0)
        index.replace((0, 0, 0), replacement, triangles)
        index.query([0, 0, 0], [1, 1, 3])
        self.assertEqual(index.stats["top_level_rebuilds"], 2)
        old, new = index.replace((0, 0, 0), [], [])
        self.assertIsNotNone(old)
        self.assertIsNone(new)
        self.assertEqual(len(index.blocks), 0)
        self.assertEqual(len(index.query([-1] * 3, [10] * 3)[1]), 0)
        index.replace((0, 0, 0), replacement, triangles)
        index.clear()
        self.assertEqual(len(index.blocks), 0)
        self.assertEqual(index.stats["triangle_count"], 0)
        self.assertEqual(index.query([-1] * 3, [10] * 3)[0].shape, (0, 3))

    def test_both_bvh_levels_prune_and_distinguish_height(self):
        index = MeshIndex(triangle_leaf_size=2, block_leaf_size=2)
        points = np.array([[[x, 0., 0.], [x + .2, 0., 0.], [x, .2, 0.]]
                           for x in range(128)])
        for height in range(64):
            block = points + [0, 0, height * 2]
            index.replace((0, 0, height), block.reshape((-1, 3)),
                          np.arange(128 * 3).reshape((-1, 3)))
        vertices, triangles = index.query([31.9, -.1, 3.9], [32.3, .3, 4.1])
        self.assertEqual(len(triangles), 1)
        self.assertTrue(np.all(vertices[:, 2] == 4))
        self.assertEqual(index.stats["candidate_blocks"], 1)
        self.assertLess(index.stats["top_nodes_visited"], 25)
        self.assertLess(index.stats["triangle_nodes_visited"], 25)
        index.query([-100] * 3, [-99] * 3)
        self.assertEqual(index.stats["visited_nodes"], 1)

    def test_invalid_input_does_not_replace_existing_block(self):
        index = MeshIndex()
        vertices = [[0, 0, 0], [1, 0, 0], [0, 1, 0]]
        index.replace((1,), vertices, [[0, 1, 2]])
        for triangles in ([[0, 1, 3]], [[-1, 1, 2]], [[0, 1.5, 2]],
                          [[0, float("nan"), 2]], [["0", "1", "2"]]):
            with self.subTest(triangles=triangles), self.assertRaises(ValueError):
                index.replace((1,), vertices, triangles)
        with self.assertRaises(ValueError):
            index.replace((1,), [[np.inf, 0, 0]], [[0, 0, 0]])
        with self.assertRaises(ValueError):
            index.replace((1,), vertices, [[0, 1]])
        for low, high in (([2] * 3, [1] * 3), ([0, 0], [1] * 3),
                          ([np.nan] * 3, [1] * 3)):
            with self.assertRaises(ValueError):
                index.query(low, high)
        self.assertEqual(len(index.query([-1] * 3, [2] * 3)[1]), 1)

    def test_degenerate_boundary_and_orphan_vertices(self):
        index = MeshIndex()
        _, bounds = index.replace((0,), [[1, 2, 3], [999, 999, 999]], [[0, 0, 0]])
        np.testing.assert_equal(bounds, ([1, 2, 3], [1, 2, 3]))
        self.assertEqual(len(index.query([1, 2, 3], [1, 2, 3])[1]), 1)
        self.assertEqual(len(index.query([2, 2, 3], [2, 2, 3])[1]), 0)

    def test_has_mesh_matches_candidates_without_materializing_geometry(self):
        rng = np.random.default_rng(945)
        index = MeshIndex(triangle_leaf_size=2, block_leaf_size=2)
        for i in range(24):
            points = rng.uniform(-1, 1, (25, 3, 3)) + rng.uniform(-6, 6, 3)
            index.replace((i,), points.reshape((-1, 3)), np.arange(75).reshape((-1, 3)))
        for _ in range(150):
            low = rng.uniform(-8, 6, 3)
            high = low + rng.uniform(0, 4, 3)
            expected = bool(len(index.query(low, high)[1]))
            self.assertEqual(index.has_mesh(low, high), expected)
        # Existence must stop early even when every block/triangle intersects.
        self.assertTrue(index.has_mesh([-10] * 3, [10] * 3))
        self.assertEqual(index.stats['candidate_blocks'], 1)
        self.assertEqual(index.stats['candidate_triangles'], 1)
        self.assertLess(index.stats['visited_nodes'], 20)
        for key in list(index.blocks):
            index.replace(key, [], [])
        self.assertFalse(index.has_mesh([-10] * 3, [10] * 3))
        index.replace((99,), [[0, 0, 0]], [[0, 0, 0]])
        self.assertTrue(index.has_mesh([0, 0, 0], [0, 0, 0]))
        index.clear()
        self.assertFalse(index.has_mesh([0, 0, 0], [0, 0, 0]))
        with self.assertRaises(ValueError):
            index.has_mesh([0, 0], [1, 1, 1])


if __name__ == "__main__":
    unittest.main()
