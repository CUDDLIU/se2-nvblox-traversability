import unittest
from dataclasses import replace
from unittest.mock import patch

import numpy as np

from local_updates import LocalUpdates
from paper_pipeline import Config, Engine, classify, classify_reference, voxelize
from test_paper_pipeline import plane, combine, signature, grid
from terrain import footprint_masks


class RealtimeGeometryTests(unittest.TestCase):
    def test_cached_raster_matches_original_after_replace_delete_and_reset(self):
        c = Config(length=.2, width=.16, side_margin=0., yaw_bins=8, sweep_sample_deg=3.)
        cached, reference = Engine(c), Engine(c, cache_raster=False)
        batches = [({(0,): plane(-2, -2, 2, 2), (1,): plane(-1, -1, 1, 1, 1.1)}, False),
                   ({(2,): plane(-.2, -.2, .2, .2, .6, downward=True)}, False),
                   ({(1,): plane(-1, -1, 1, 1, .9)}, False),
                   ({(2,): ([], [])}, False),
                   ({(0,): plane(-1, -1, 1, 1, .79)}, True)]
        for batch, clear in batches:
            for engine in (cached, reference):
                engine.update(batch, clear=clear)
                engine.process(100000)
            self.assertEqual(signature(cached), signature(reference))
            self.assertEqual(cached.snapshot(), reference.snapshot())

    def test_native_classification_matches_reference_on_noisy_layered_holes(self):
        c = Config()
        masks = footprint_masks(c)
        rng = np.random.default_rng(991)
        for _ in range(15):
            spans = grid(-8, 9, -8, 9)+grid(-8, 9, -8, 9, z=1.2)
            spans = [s for s in spans if rng.random() > .035]
            for s in spans:
                s.z += rng.choice([-.03, 0., 0., 0., .01, .04])
                s.ceiling = s.z+rng.choice([.9, 1., 1.01, 1.3, float('inf')])
                s.slope_ok = bool(rng.random() > .02)
                s.walkable = s.slope_ok and s.ceiling-s.z >= c.required_height-1e-9
            actual = classify(spans, c, masks)
            expected = classify_reference(spans, c, masks)
            np.testing.assert_array_equal(actual[0], expected[0])
            np.testing.assert_array_equal(actual[1], expected[1])
            self.assertEqual(actual[2], expected[2])

    def test_empty_slabs_do_not_consume_completion_budget(self):
        e = Engine()
        e.pending.update((x, 0, 0) for x in range(20))
        e.process(1)
        self.assertFalse(e.pending)
        self.assertEqual(len(e.completed_keys), 20)

    def test_zero_deadline_and_deferred_polygons(self):
        e = Engine()
        e.update({(0,): plane(-1.5, -1.5, 1.5, 1.5)})
        pending = e.pending.copy()
        e.process(budget_ms=0)
        self.assertEqual(e.pending, pending)
        e.process(100000, defer_polygons=True)
        self.assertTrue(e.snapshot(rebuild_polygons=True)['polygons'])
        self.assertFalse(e.snapshot()['polygons'])

    def test_fair_queue_does_not_starve_old_tile_under_new_updates(self):
        e = Engine()
        e.update({(0,): plane(10, 10, 11, 11)})
        oldest = min(e._group_order, key=e._group_order.get)
        e.update({(1,): plane(-10, -10, -9, -9)})
        self.assertEqual(min(e._group_order, key=e._group_order.get), oldest)


class ProvenanceTests(unittest.TestCase):
    def test_superseded_updates_preserve_oldest_wait_not_only_newest(self):
        e = Engine()
        e.update({(0,): plane(-1, -1, 1, 1)})
        tracker = LocalUpdates()
        keys = sorted(e.pending)
        tracker.invalidate(keys, 10., 1, 100.)
        tracker.invalidate(keys, 10.090, 2, 100.090)
        e.process(100000, defer_polygons=True)
        result = tracker.finish(e, e.completed_keys, now=10.1)
        self.assertAlmostEqual(result['latency_oldest_ms'], 100.)
        self.assertAlmostEqual(result['latency_newest_ms'], 10.)
        self.assertTrue(result['complete_input'])
        self.assertEqual({s['input_sequence'] for s in result['updated_slabs']}, {2})
        self.assertIsNone(tracker.finish(e, [], now=11.))
        self.assertEqual(tracker.sequence, 1)

    def test_deleted_tiles_are_explicit_empty_replacements(self):
        e = Engine()
        e.update({(0,): plane(-1, -1, 1, 1)})
        e.process(100000)
        e.update({(0,): ([], [])})
        tracker = LocalUpdates()
        tracker.invalidate(e.dirty_keys, 1., 2, 20.)
        e.process(100000)
        output = tracker.finish(e, e.completed_keys, now=1.02)
        self.assertTrue(output['updated_slabs'])
        self.assertTrue(all(not s['cells'] for s in output['updated_slabs']))


if __name__ == '__main__':
    unittest.main()
