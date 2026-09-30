import math
import unittest
from angular_display import orange_color,result_rows
from local_display import LocalDisplay
from test_physical_clearance import run,corridor

class AngularDisplayTests(unittest.TestCase):
    def test_color_follows_certified_range_not_number_of_bits(self):
        deep=orange_color(0.);mid=orange_color(.01);light=orange_color(.5)
        self.assertTrue(deep[0]<mid[0]<light[0]);self.assertTrue(deep[1]<mid[1]<light[1])
        self.assertEqual(orange_color(-1),deep)
        d=LocalDisplay();d.replace([dict(key=[0,0,0],cells=[[0,0,0,3,0,0.],[1,0,0,3,0,.4]])])
        result=d.snapshot(40)
        self.assertEqual(len(result['amber']),2)
        self.assertLess(result['amber'][0][3],result['amber'][1][3])
        self.assertFalse(result['green'])

    def test_full_physical_mask_is_not_comfortable_green(self):
        full=(1<<40)-1;d=LocalDisplay()
        d.replace([dict(key=[0,0,0],cells=[[0,0,0,full,0,2*math.pi],[1,0,0,full,full,2*math.pi]])])
        s=d.snapshot(40);self.assertEqual(len(s['green']),1);self.assertEqual(len(s['amber']),1)

    def test_history_and_rejected_cells_do_not_turn_orange(self):
        d=LocalDisplay();d.replace([dict(key=[0,0,0],cells=[[0,0,0,3,0,0],[1,0,0,0,0,0]])])
        d.parameters_changed();s=d.snapshot(40)
        self.assertFalse(s['amber']);self.assertEqual(len(s['archived']),1)

    def test_just_fitting_corridor_exports_zero_range(self):
        e=run(corridor());rows=[row for r in e.results.values() for row in result_rows(r,40) if row[3]]
        self.assertTrue(rows);self.assertTrue(all(row[4]==0 and row[5]<1e-8 for row in rows))

if __name__=='__main__':unittest.main()
