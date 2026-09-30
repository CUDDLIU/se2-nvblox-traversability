import unittest
from analyze_route import components,component_cover


class ConnectivityTests(unittest.TestCase):
    def test_stairs_connect_within_step_limit(self):
        _,counts=components([(0,0,0,3),(1,0,16,3),(2,0,32,3)],4,20)
        self.assertEqual(list(counts.values()),[6])

    def test_missing_tread_and_excessive_step_remain_disconnected(self):
        for cells in ([(0,0,0,1),(2,0,16,1)],[(0,0,0,1),(1,0,21,1)]):
            _,counts=components(cells,4,20);self.assertEqual(len(counts),2)

    def test_yaw_and_stacked_floors_are_not_merged(self):
        _,counts=components([(0,0,0,1),(1,0,0,4),(0,0,120,1)],4,20)
        self.assertEqual(len(counts),3)

    def test_alternate_valid_component_prevents_nearest_fragment_bias(self):
        best,cover=component_cover([{1,9},{2,9},{3,9},set(),{4,9}])
        self.assertEqual(best,4);self.assertEqual(cover,[dict(component=9,newly_covered_samples=4)])
        best,cover=component_cover([{1},{1},{2},set()])
        self.assertEqual(best,2);self.assertEqual(len(cover),2)
        self.assertEqual(component_cover([set()]),(0,[]))

if __name__=='__main__':unittest.main()
