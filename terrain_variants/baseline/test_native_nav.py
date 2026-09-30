import unittest
from dataclasses import replace

import numpy as np

from native_nav import Classifier
from paper_pipeline import Config, Engine, classify_reference
from terrain import footprint_masks
from test_paper_pipeline import grid, plane, combine, signature


class NativeNavigationTests(unittest.TestCase):
    def test_nearby_group_precedes_old_distant_group_but_fifo_turn_prevents_starvation(self):
        blocks={(0,0,0):plane(.1,.1,1.5,1.5),
                (2,0,0):plane(3.3,.1,4.7,1.5),
                (-4,0,0):plane(-6.3,.1,-4.9,1.5)}
        e=Engine(native_mesh=True);e.update(blocks)
        e.pending={(0,0,0),(2,0,0),(-4,0,0)}
        e._group_order={(-4,0):0,(2,0):1,(0,0):2}
        e.process(budget_ms=1000,defer_polygons=True,priority_position=(.4,.4,0))
        self.assertEqual(e.completed_keys,[(0,0,0)])
        self.assertIn((0,0),e._completed_groups)
        e._process_turn=3
        e.process(budget_ms=1000,defer_polygons=True,priority_position=(.4,.4,0))
        self.assertEqual(e.completed_keys,[(-4,0,0)])
        e.update({},clear=True)
        self.assertFalse(e._completed_groups)

    def test_exact_interval_cache_revalidates_changed_mesh_and_invalidates_on_config(self):
        e=Engine(native_mesh=True);v,t=plane(-2,-2,2,2,.011)
        e.update({(0,0,0):(v,t)});e.process(100000,defer_polygons=True)
        initial=signature(e)
        moved=v.copy();moved[:,2]+=.001
        e.update({(0,0,0):(moved,t)});e.process(100000,defer_polygons=True)
        self.assertGreater(e.last_metrics['unchanged_interval_groups'],0)
        self.assertEqual(signature(e),initial)
        e.reconfigure(replace(e.config,width=.3))
        self.assertFalse(e._span_signatures)
        e.process(100000,defer_polygons=True)
        fresh=Engine(native_mesh=True);fresh.reconfigure(e.config)
        fresh.update({(0,0,0):(moved,t)});fresh.process(100000,defer_polygons=True)
        self.assertEqual(signature(e),signature(fresh))
        e.update({},clear=True);self.assertFalse(e._span_signatures)

    def test_global_export_runs_in_spawned_process_without_native_pointer_copy(self):
        import json
        import multiprocessing
        from concurrent.futures import ProcessPoolExecutor
        from global_export import export_document
        e=Engine(native_mesh=True)
        e.update({(0,0,0):plane(-1,-1,1,1)})
        e.process(100000,defer_polygons=True)
        with ProcessPoolExecutor(1,mp_context=multiprocessing.get_context('spawn')) as pool:
            text=pool.submit(export_document,e.config,e.results,e.generation,e.revision,
                             {'frame':'test','valid':False}).result(timeout=20)
        data=json.loads(text)
        self.assertEqual(data['revision'],e.revision)
        self.assertEqual(data['frame'],'test')
        self.assertFalse(data['valid'])
        self.assertTrue(data['polygons'])
        self.assertEqual(len(data['polygons']),len(e.snapshot(rebuild_polygons=True)['polygons']))

    def test_native_mesh_batches_match_reference_and_reconfigure(self):
        blocks={(0,0,0):combine(plane(-2,-2,2,2),plane(-1,-1,1,1,.8,downward=True)),
                (1,0,0):plane(-2,-2,2,2,1.4)}
        native=Engine(native_mesh=True);reference=Engine(native_classify=False)
        for update in (blocks,blocks,{(1,0,0):(np.empty((0,3)),np.empty((0,3),dtype=int))}):
            a=native.update(update);b=reference.update(update)
            self.assertEqual(a,b)
            self.assertEqual(native.pending,reference.pending)
            for e in (native,reference):e.process(100000,defer_polygons=True)
            self.assertEqual(signature(native),signature(reference))
        for e in (native,reference):
            e.reconfigure(replace(e.config,height=.5,max_slope_deg=20.))
        self.assertEqual(native.pending,reference.pending)
        for e in (native,reference):e.process(100000,defer_polygons=True)
        self.assertEqual(signature(native),signature(reference))
        for e in (native,reference):e.update({},clear=True)
        self.assertEqual(signature(native),signature(reference))
        self.assertEqual(len(native.index.blocks),0)

    def compare(self, spans, config=Config(), roots=None):
        masks=footprint_masks(config)
        expected=classify_reference(spans,config,masks)
        got=Classifier(config,masks)(spans,roots)
        ids=np.arange(len(spans)) if roots is None else np.asarray(roots,dtype=int)
        np.testing.assert_array_equal(got[0][ids],expected[0][ids])
        np.testing.assert_allclose(got[1][ids],expected[1][ids],rtol=0.,atol=1e-12)
        self.assertEqual([got[2][i] for i in ids],[expected[2][i] for i in ids])

    def test_flat_layers_and_limited_roots(self):
        spans=grid()+grid(z=1.3)
        self.compare(spans)
        self.compare(spans,roots=list(range(0,len(spans),7)))

    def test_random_holes_bad_slope_low_roof_and_steps(self):
        rng=np.random.default_rng(933)
        for _ in range(4):
            spans=[]
            for s in grid():
                if rng.random()<.08:continue
                z=s.z+rng.choice([0.,.01,.03,.20])
                roof=rng.choice([.6,1.01,1.2, np.inf])
                slope=rng.random()>.08
                spans.append(replace(s,z=z,ceiling=roof,slope_ok=slope,
                                     walkable=slope and roof-z>=1.))
            self.compare(spans)

    def test_full_engine_matches_all_roots_for_multilayer_and_deletion(self):
        c=Config()
        blocks={(0,):combine(plane(-2,-2,2,2),plane(-1,-1,1,1,.8,downward=True)),
                (1,):plane(-2,-2,2,2,1.4)}
        native=Engine(c);reference=Engine(c,native_classify=False)
        for update in (blocks,{(1,):(np.empty((0,3)),np.empty((0,3),dtype=int))}):
            for e in (native,reference):e.update(update);e.process(100000,defer_polygons=True)
            self.assertEqual(signature(native),signature(reference))
            for key,result in native.results.items():
                np.testing.assert_allclose(result.distances,reference.results[key].distances,atol=1e-12)
                self.assertEqual(result.reasons,reference.results[key].reasons)

    def test_reconfigured_classifier_uses_new_envelope(self):
        e=Engine();e.update({(0,):plane(-1,-1,1,1)});e.process(100000)
        c=replace(e.config,width=.2,height=.5)
        e.reconfigure(c);e.process(100000)
        fresh=Engine(c,native_classify=False);fresh.update({(0,):plane(-1,-1,1,1)});fresh.process(100000)
        self.assertEqual(signature(e),signature(fresh))


if __name__=='__main__':unittest.main()
