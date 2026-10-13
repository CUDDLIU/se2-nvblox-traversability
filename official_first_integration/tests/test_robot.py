import importlib.util,sys,unittest
from pathlib import Path
HERE=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('first_robot',HERE/'robot.py');robot=importlib.util.module_from_spec(spec);spec.loader.exec_module(robot)
class RobotGateTests(unittest.TestCase):
    def fresh(self):
        g=robot.Gate();g.command=(1.2,.3,.4);g.command_at=10.
        g.receive({'Type':1002,'Command':6,'Items':{'BasicStatus':{'ControlUsageMode':1,'HES':0,'Direction':0,'MotionState':17,'Sleep':0,'Gait':0x3002}}},10.)
        g.receive({'Type':1002,'Command':4,'Items':{'MotionStatus':{'Height':.57}}},10.)
        return g
    def test_native_command_is_unchanged(self):
        g=self.fresh();self.assertEqual(g.output(10.01),(1.2,.3,.4))
    def test_stale_command(self):
        self.assertEqual(self.fresh().output(10.13),(0.,0.,0.))
    def test_status_watchdogs(self):
        g=self.fresh();g.command_at=10.36;self.assertEqual(g.stop_reason(10.36),'stale_motion_status')
        g.motion_at=11.21;g.command_at=11.21;self.assertEqual(g.stop_reason(11.21),'stale_basic_status')
    def test_error_and_operator_stop(self):
        g=self.fresh();g.basic['HES']=1;self.assertEqual(g.output(10.01),(0.,0.,0.))
        g=self.fresh();g.basic['ControlUsageMode']=0;self.assertEqual(g.output(10.01),(0.,0.,0.))
        g=self.fresh();g.receive({'Type':2,'Command':23,'ErrorCode':57347},10.01);self.assertEqual(g.output(10.01),(0.,0.,0.))
if __name__=='__main__':unittest.main()
