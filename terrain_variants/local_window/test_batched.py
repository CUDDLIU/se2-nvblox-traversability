"""Batch-wide invalidation and physical geometry regressions."""
import importlib.util
from pathlib import Path
import sys
import unittest
import numpy as np
from paper_pipeline import Config, Engine
from test_paper_pipeline import plane, combine
import test_native_nav

spec=importlib.util.spec_from_file_location('unbatched_reference',Path(__file__).parents[1]/'baseline/paper_pipeline.py')
reference=importlib.util.module_from_spec(spec);sys.modules[spec.name]=reference;spec.loader.exec_module(reference)

def masks(engine):
    return {(s.ix,s.iy,s.iz):int(m) for r in engine.results.values() for s,m in zip(r.spans,r.masks)}

class BatchTests(unittest.TestCase):
    def compare(self,config,updates):
        batched=Engine(config,native_mesh=True);old=reference.Engine(config,native_mesh=True)
        for update in updates:
            batched.update(update);old.update(update)
            batched.process(budget_ms=35,defer_polygons=True)
            old.process(100000,defer_polygons=True)
            self.assertFalse(batched.pending)
            self.assertEqual(masks(batched),masks(old))
        return batched

    def test_multilayer_delete_and_replacement(self):
        c=Config()
        self.compare(c,[{(0,0,0):combine(plane(-3,-2,3,2),plane(-2,-2,2,2,.7,downward=True)),
                         (0,0,1):plane(-3,-2,3,2,1.5)},
                        {(0,0,1):(np.empty((0,3)),np.empty((0,3),dtype=np.int64))},
                        {(0,0,0):plane(-2,-2,2,2)}])

    def test_stairs_pass_but_missing_tread_does_not_get_filled(self):
        c=Config(max_step=.20,max_slope_deg=35.)
        stairs={(i,0,0):plane(i*.3,-1.2,(i+1)*.3,1.2,i*.16) for i in range(18)}
        stairs[(-1,0,0)]=plane(-2,-1.2,0,1.2)
        stairs[(18,0,0)]=plane(5.4,-1.2,7,1.2,17*.16)
        e=self.compare(c,[stairs])
        permitted={ix for (ix,iy,iz),m in masks(e).items() if iy==0 and m}
        self.assertTrue(set(range(3,48))<=permitted)
        e.update({(9,0,0):(np.empty((0,3)),np.empty((0,3),dtype=np.int64))})
        e.process(budget_ms=35,defer_polygons=True)
        self.assertFalse(any(m for (ix,iy,iz),m in masks(e).items() if ix==28 and iy==0))

    def test_native_parallel_matches_reference_on_random_hazards(self):
        test_native_nav.NativeNavigationTests().test_random_holes_bad_slope_low_roof_and_steps()

if __name__=='__main__':unittest.main()
