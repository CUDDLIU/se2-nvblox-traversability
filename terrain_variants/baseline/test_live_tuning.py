import unittest
from dataclasses import replace
from unittest.mock import patch
from types import SimpleNamespace

from local_display import LocalDisplay
from paper_pipeline import Config, Engine
from test_paper_pipeline import plane, combine, signature
from test_realtime_ros import node, delta


class LiveTuningTests(unittest.TestCase):
    def test_height_and_slope_change_matches_fresh_rebuild(self):
        c=Config(length=.2,width=.16,side_margin=0.,yaw_bins=8,sweep_sample_deg=3.)
        blocks={(0,):combine(plane(-1,-1,1,1),plane(-1,-1,1,1,.8,downward=True))}
        e=Engine(c);e.update(blocks);e.process(100000)
        before=signature(e)
        new=replace(c,height=.5,max_slope_deg=20.)
        e.reconfigure(new);e.process(100000)
        fresh=Engine(new);fresh.update(blocks);fresh.process(100000)
        self.assertEqual(signature(e),signature(fresh))
        self.assertNotEqual(before,signature(e))
        self.assertTrue(any(v[1] for v in signature(e).values()))

    def test_archive_persists_but_populated_rejection_overrides_history(self):
        d=LocalDisplay()
        d.replace([dict(key=[0,0,0],cells=[[1,1,1,15]])])
        d.archive()
        self.assertEqual(len(d.snapshot(4)['archived']),1)
        doc=d.to_document('world','session-a',{})
        resumed=LocalDisplay()
        self.assertFalse(resumed.load_document(doc,'world','session-b'))
        self.assertFalse(resumed.archive_cells)
        self.assertTrue(resumed.load_document(doc,'world','session-a'))
        self.assertEqual(len(resumed.snapshot(4)['archived']),1)
        resumed.replace([dict(key=[0,0,0],cells=[[1,1,1,0]])])
        self.assertFalse(resumed.snapshot(4)['archived'])

    def test_reconfigure_marks_old_parameters_blue_until_replaced(self):
        d=LocalDisplay();d.replace([dict(key=[0,0,0],cells=[[0,0,1,15]])])
        d.parameters_changed()
        self.assertFalse(d.snapshot(4)['green'])
        self.assertEqual(len(d.snapshot(4)['archived']),1)
        d.replace([dict(key=[0,0,0],cells=[[0,0,1,1]])])
        self.assertEqual(len(d.snapshot(4)['amber']),1)
        self.assertFalse(d.snapshot(4)['archived'])

    def test_bounded_admission_still_processes_and_keeps_inbox(self):
        n=node();n.options['max_update_blocks']=1
        n.receive_mesh(delta(0));n.receive_mesh(delta(1))
        n.tick()
        self.assertEqual(len(n.inbox),1)
        self.assertGreater(n.local_events,0)
        self.assertFalse(n.last_local['complete_input'])
        n.tick()
        self.assertFalse(n.inbox)

    def test_atomic_parameter_validation_and_recompute_without_new_mesh(self):
        n=node();n.receive_mesh(delta(0));n.tick()
        params=[SimpleNamespace(name='height',value=.7),SimpleNamespace(name='top_margin',value=.05)]
        self.assertTrue(n.parameters_changed(params).successful)
        self.assertFalse(n.parameters_changed([SimpleNamespace(name='height',value=-1.)]).successful)
        self.assertFalse(n.parameters_changed([SimpleNamespace(name='resolution',value=.2)]).successful)
        values={'height':.7,'top_margin':.05}
        n.get_parameter=lambda name:SimpleNamespace(value=values.get(name,getattr(n.config,name)))
        n.tick()
        self.assertEqual(n.config.height,.7)
        self.assertEqual(n.config_version,1)
        self.assertFalse(n.compute_error)
        self.assertEqual(n.engine.config,n.config)


if __name__=='__main__':unittest.main()
