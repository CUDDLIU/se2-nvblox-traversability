import unittest
import numpy as np
from core import Core
from task_profile import DEFAULT,task_parameters


class TaskParameters(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.core=Core()
    @classmethod
    def tearDownClass(cls):cls.core.close()
    def task(self,**kw):
        return dict(Gait=0x3003,NavMode=0,Speed=0,Manner=0,ObsMode=0,**kw)
    def test_speed_enum_is_not_numeric_order(self):
        t=self.task();p=task_parameters(t);self.assertEqual(p['maxSpeedX'],1.)
        t['Speed']=1;self.assertEqual(task_parameters(t)['maxSpeedX'],.8)
        t['Speed']=2;self.assertEqual(task_parameters(t)['maxSpeedX'],1.2)
    def test_profile_change_invalidates_existing_goal(self):
        c=self.core;c.cloud(np.empty((0,3)));c.configure(DEFAULT);c.goal()
        self.assertGreater(c.line((3,0,0)).x,0)
        c.configure(task_parameters(self.task()))
        self.assertEqual(c.line((3,0,0)).status,999)
        c.goal();self.assertAlmostEqual(c.line((.4,0,0)).x,.8,places=6)
    def test_oa_and_stair_gait_are_independent(self):
        t=self.task();self.assertFalse(task_parameters(t)['judge_close'])
        t['ObsMode']=1;self.assertTrue(task_parameters(t)['judge_close'])
    def test_invalid_profiles_rejected(self):
        for name,value in [('Gait',0),('Speed',-1),('NavMode',2),('ObsMode',True)]:
            t=self.task();t[name]=value
            with self.assertRaises(ValueError):task_parameters(t)
        with self.assertRaises(ValueError):self.core.configure(dict(DEFAULT,maxSpeedX=float('nan')))
    def test_line_task_cap_and_backward_obstacle_direction(self):
        c=self.core;t=self.task();t['Speed']=1;c.configure(task_parameters(t));c.cloud(np.empty((0,3)));c.goal()
        self.assertAlmostEqual(c.line((4,0,0)).x,.8,places=6)
        c.goal();self.assertLess(c.line((-4,0,0),backward=True).x,0)


if __name__=='__main__':unittest.main()
