import unittest
from supervision import WarningTimer

class Supervision(unittest.TestCase):
    def test_warning_changes_do_not_reset_shared_timer(self):
        s=WarningTimer(1000)
        self.assertEqual(s.update(11000,1101),0)
        self.assertEqual(s.update(31000,1102),0)
        self.assertEqual(s.update(31001,1102),0xa34c)
        self.assertEqual(s.update(32000,0),0xa34c)
    def test_obstacle_flag_only_clears_with_zero_warning(self):
        s=WarningTimer(0);s.update(100,2004);s.update(200,1101)
        self.assertTrue(s.obstacle_wait)
        self.assertEqual(s.update(30001,1101),0xa34b)
        s=WarningTimer(0);s.update(100,2004);s.update(200,0)
        self.assertFalse(s.obstacle_wait);self.assertEqual(s.last_good_ms,200)
    def test_localization_and_command_timeout_and_new_goal(self):
        self.assertEqual(WarningTimer(0).update(30001,0,localized=False),0xa313)
        self.assertEqual(WarningTimer(0).update(30001,0,command_fresh=False),0xa34d)
        self.assertEqual(WarningTimer(30001).update(30002,0),0)

if __name__=='__main__':unittest.main()
