import json,unittest
from pathlib import Path
import numpy as np
from handler import Handler,Selection

class HandlerTests(unittest.TestCase):
    def test_original_instruction_selection_fixtures(self):
        data=json.loads(Path(__file__).with_name('handler_reference_cases.json').read_text())
        for case in data['cases']:
            with self.subTest(valid=case['valid'],points=len(case['points'])):
                h=Handler();h.cfg=np.array(case['cfg']);h.bounds=np.array(case['bounds']);h.desired=np.array(case['desired'])
                result=h.select(case['points'],case['measured'])
                self.assertEqual(result.valid,case['valid']);self.assertEqual(result.accepted,case['accepted'])
                np.testing.assert_allclose(result.velocity,case['velocity'],atol=1e-12,rtol=1e-12)
                self.assertAlmostEqual(result.clearance,case['clearance'],places=6)
    def test_unready_or_new_steer_loss_clears_previous_command(self):
        h=Handler();h.enabled=True;h.process_steer([.5,0,.5])
        np.testing.assert_array_equal(h.tick([],np.zeros(3)),[0.,0.,0.])
        np.testing.assert_array_equal(h.desired,[0.,0.,0.])
        h.process_steer([.5,0,.5]);h.steer_watchdog(0)
        np.testing.assert_array_equal(h.tick([],np.zeros(3),ready=True),[0.,0.,0.])
        np.testing.assert_array_equal(h.desired,[0.,0.,0.])
    def test_no_candidate_stops_and_applies_pending_stair_oa(self):
        h=Handler();h.enabled=True;h.process_steer([.5,0,.5]);h.stair_oa=True
        h.select=lambda *args:Selection(False,np.zeros(3),1.,1000.,0)
        calls=[];out=h.tick([],np.zeros(3),ready=True,stair_oa_call=lambda value:calls.append(value))
        np.testing.assert_array_equal(out,[0.,0.,0.]);self.assertEqual(calls,[True]);self.assertTrue(h.previous_stair_oa)
        self.assertEqual(h.state,2) # Native status is clearance-based, not a new failure state.
    def test_native_filter_removes_body_and_preserves_single_obstacle_return(self):
        h=Handler();out=h.filter_cloud([[0,0,0],[3.1,0,0],[1,0,.4],[1,0,-.7],[.8,.3,0],[float('nan'),0,0]])
        np.testing.assert_array_equal(out,np.array([[.8,.3,0]],np.float32))
    def test_service_changes_hard_limit_without_recomputing_native_slopes(self):
        h=Handler();h.set_parameters([.9,.7,.8],False);h.update_limits(.5,0.)
        np.testing.assert_allclose(h.bounds[::2],[1.0666666666666667,.6,1.0666666666666667],atol=1e-14)
        h.update_limits(1.,0.);np.testing.assert_array_equal(h.bounds[::2],[.9,.7,.8])
    def test_invalid_sampling_input_is_rejected_before_native_loop(self):
        h=Handler();h.cfg[10]=0.
        with self.assertRaises(ValueError):h.select([],np.zeros(3))

if __name__=='__main__':unittest.main()
