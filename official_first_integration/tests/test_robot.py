import importlib.util,sys,unittest
from pathlib import Path
HERE=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('first_robot',HERE/'robot.py');robot=importlib.util.module_from_spec(spec);spec.loader.exec_module(robot)
class RobotGateTests(unittest.TestCase):
    def test_cancel_waits_for_sent_gait_and_never_overlaps_requests(self):
        sys.path.insert(0,str(HERE))
        from source.gait import GaitSwitch
        g=self.fresh();tr=GaitSwitch()
        state=dict(wanted=0x3003,wanted_at=10.,pending=0,requested_at=0.,failed=False)
        self.assertEqual(robot.source_gait_step(tr,state,g,10.,True),0x3003)
        state['wanted']=0
        self.assertIsNone(robot.source_gait_step(tr,state,g,10.1,True))
        self.assertEqual(state['pending'],0x3003)
        state['wanted']=0x3002;state['wanted_at']=10.2
        self.assertIsNone(robot.source_gait_step(tr,state,g,10.2,True))
        self.assertEqual(tr.target,0x3003)
        g.basic['Gait']=0x3003;g.basic_at=g.motion_at=10.3
        self.assertEqual(robot.source_gait_step(tr,state,g,10.3,True),0x3002)
        self.assertEqual(state['pending'],0x3002)
    def test_source_gait_retry_cancel_and_fault_latch(self):
        sys.path.insert(0,str(HERE))
        from source.gait import GaitSwitch
        g=self.fresh();tr=GaitSwitch()
        state=dict(wanted=0x3003,wanted_at=10.,pending=0,requested_at=0.,failed=False)
        for i,t in enumerate((10.,13.,16.,19.)):
            state['wanted_at']=t;g.basic_at=g.motion_at=t
            request=robot.source_gait_step(tr,state,g,t,True)
            self.assertEqual(request,0x3003 if i<3 else None)
        self.assertTrue(g.failed);self.assertEqual(state['attempts'],3)
        g.basic['Gait']=0x3003
        robot.source_gait_step(tr,state,g,19.01,True)
        self.assertTrue(g.failed)
        state['wanted']=0
        robot.source_gait_step(tr,state,g,19.02,True)
        self.assertEqual(state['pending'],0);self.assertTrue(g.failed)
    def test_source_gait_never_retries_on_stale_or_operator_status(self):
        sys.path.insert(0,str(HERE))
        from source.gait import GaitSwitch
        for field,value in (('HES',1),('ControlUsageMode',0),('MotionState',1),('Direction',1)):
            g=self.fresh();g.basic[field]=value;tr=GaitSwitch()
            state=dict(wanted=0x3003,wanted_at=10.,pending=0,requested_at=0.,failed=False)
            self.assertIsNone(robot.source_gait_step(tr,state,g,10.,True))
            self.assertEqual(state['pending'],0x3003)
        g=self.fresh();tr=GaitSwitch();state['wanted_at']=12.
        self.assertIsNone(robot.source_gait_step(tr,state,g,12.,True))
    def test_source_envelope_does_not_clip_task_output(self):
        items=robot.decode_packet(robot.source_velocity_packet(2.,1.,1.))[1]['Items']
        self.assertEqual((items['X'],items['Y'],items['Yaw']),(2.,1.,1.))
        with self.assertRaises(ValueError):robot.source_velocity_packet(2.01,0,0)
    def fresh(self):
        g=robot.Gate();g.command=(1.2,.3,.4);g.command_at=10.
        g.receive({'Type':1002,'Command':6,'Items':{'BasicStatus':{'ControlUsageMode':1,'HES':0,'Direction':0,'MotionState':17,'Sleep':0,'Gait':0x3002}}},10.)
        g.receive({'Type':1002,'Command':4,'Items':{'MotionStatus':{'Height':.57}}},10.)
        return g
    def test_native_command_is_unchanged(self):
        g=self.fresh();self.assertEqual(g.output(10.01),(1.2,.3,.4))
    def test_official_lateral_limit_preserves_native_float32(self):
        native_y=0.6000000238418579
        value=robot.decode_packet(robot.official_velocity_packet(1.5,native_y,1.))
        self.assertEqual(value[1]['Items']['Y'],native_y)
        for v in ((0.,.61,0.),(1.51,0.,0.),(0.,0.,1.01),(float('nan'),0.,0.)):
            with self.assertRaises(ValueError):robot.official_velocity_packet(*v)
        with self.assertRaises(ValueError):robot.velocity_packet(0.,native_y,0.)
    def test_stale_command(self):
        self.assertEqual(self.fresh().output(10.13),(0.,0.,0.))
    def test_status_watchdogs(self):
        g=self.fresh();g.command_at=10.36;self.assertEqual(g.stop_reason(10.36),'stale_motion_status')
        g.motion_at=11.21;g.command_at=11.21;self.assertEqual(g.stop_reason(11.21),'stale_basic_status')
    def test_error_and_operator_stop(self):
        g=self.fresh();g.basic['HES']=1;self.assertEqual(g.output(10.01),(0.,0.,0.))
        g=self.fresh();g.basic['ControlUsageMode']=0;self.assertEqual(g.output(10.01),(0.,0.,0.))
        g=self.fresh();g.receive({'Type':2,'Command':23,'ErrorCode':57347},10.01);self.assertEqual(g.output(10.01),(0.,0.,0.))
    def test_delayed_gait_confirmation_does_not_latch(self):
        g=self.fresh();g.basic['Gait']=0x3003
        state=dict(pending=0x3002,requested_at=10.,failed=False)
        g.basic_at=13.1
        robot.update_gait_confirmation(state,g,13.1)
        self.assertFalse(g.failed)
        g.basic['Gait']=0x3002;g.basic_at=14.
        robot.update_gait_confirmation(state,g,14.)
        self.assertEqual(state['pending'],0);self.assertFalse(g.failed)
        self.assertEqual(state['confirmation_seconds'],4.)
    def test_gait_timeout_and_late_confirmation_stay_stopped(self):
        g=self.fresh();g.basic['Gait']=0x3003
        state=dict(pending=0x3002,requested_at=10.,failed=False)
        g.basic_at=16.1
        robot.update_gait_confirmation(state,g,16.1)
        self.assertTrue(state['failed']);self.assertEqual(g.last_error['source'],'gait_confirmation_timeout')
        g.basic['Gait']=0x3002;g.basic_at=g.motion_at=g.command_at=17.
        robot.update_gait_confirmation(state,g,17.)
        self.assertEqual(state['pending'],0);self.assertEqual(g.output(17.),(0.,0.,0.))
    def test_gait_confirmation_preserves_robot_fault(self):
        g=self.fresh();g.receive({'Type':2,'Command':23,'ErrorCode':57347},10.01)
        state=dict(pending=0x3002,requested_at=10.,failed=False)
        robot.update_gait_confirmation(state,g,10.02)
        self.assertEqual(state['pending'],0);self.assertTrue(g.failed)
        self.assertEqual(g.last_error['code'],57347)
    def test_stale_basic_status_cannot_confirm_gait(self):
        g=self.fresh();state=dict(pending=0x3002,requested_at=10.,failed=False)
        robot.update_gait_confirmation(state,g,11.3)
        self.assertEqual(state['pending'],0x3002)
if __name__=='__main__':unittest.main()
