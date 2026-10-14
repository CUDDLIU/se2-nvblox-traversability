"""Scene-level tests for recovered kernels, with no robot interface."""
import unittest
import numpy as np
from heightmap import HeightPolicy, obstacle_mask, build_height_map
from gait import GaitSwitch


class HeightScenes(unittest.TestCase):
    def test_builder_ring_keeps_last_thirty_samples(self):
        p=np.array([[.02,.02,z] for z in range(35)],np.float32)
        h,s,n,raw=build_height_map(p,shape=(7,7),origin=(0,0),filtered=False)
        self.assertEqual(n[0,0],30)
        self.assertEqual(sorted(s[:,0,0]),list(range(5,35)))
        self.assertEqual(h[0,0],34)
        self.assertEqual(raw[1,1],-10)
        self.assertTrue(np.isnan(s[:,1,1]).all())

    def test_builder_missing_cells_use_lower_median_or_low_side(self):
        p=np.array([[.12,.32,-.55],[.52,.32,-.4],[.32,.12,-.45],[.32,.52,-.42]],np.float32)
        h,s,n,raw=build_height_map(p,shape=(7,7),origin=(0,0),filtered=False)
        self.assertAlmostEqual(h[3,3],-.45,places=6)
        self.assertEqual(n[3,3],0)
        p[1,2]=.2
        h,*_=build_height_map(p,shape=(7,7),origin=(0,0),filtered=False)
        self.assertAlmostEqual(h[3,3],-.55,places=6)

    def test_builder_stair_and_wall_clouds(self):
        p=np.array([[i*.1+.02,j*.1+.02,-.55+(.13 if i>=7 else 0)+k*.001]
                    for i in range(15) for j in range(15) for k in range(4)],np.float32)
        h,s,n,_=build_height_map(p,shape=(15,15),origin=(0,0))
        policy=HeightPolicy();self.assertFalse(obstacle_mask(h,s,n,policy).any())
        policy.set_stair_oa(1);self.assertTrue(obstacle_mask(h,s,n,policy)[7,7])
        p[p[:,0]>=.7,2]+=.5
        h,s,n,_=build_height_map(p,shape=(15,15),origin=(0,0))
        policy.set_stair_oa(0);self.assertTrue(obstacle_mask(h,s,n,policy)[7,7])

    def scene(self, step):
        h = np.full((15,15), -.55, np.float32)
        h[7:,:] += np.float32(step)
        return h, np.repeat(h[None],4,axis=0), np.full(h.shape,4,np.int32)

    def test_flat_is_clear_in_both_oa_states(self):
        for enabled in (0,1):
            p=HeightPolicy();p.set_stair_oa(enabled)
            self.assertFalse(obstacle_mask(*self.scene(0),p).any())

    def test_stair_oa_means_avoiding_steps_not_enabling_climbing(self):
        # 13 cm rises are below .25 but above .08: switching OA ON makes
        # these an obstacle. It must never be used as an 'upstairs enable'.
        p=HeightPolicy()
        self.assertFalse(obstacle_mask(*self.scene(.13),p).any())
        p.set_stair_oa(1)
        mask=obstacle_mask(*self.scene(.13),p)
        self.assertTrue(mask[7,7]);self.assertFalse(mask[8,7])

    def test_wall_remains_an_obstacle(self):
        for enabled in (0,1):
            p=HeightPolicy();p.set_stair_oa(enabled)
            self.assertTrue(obstacle_mask(*self.scene(.8),p)[7,7])

    def test_strict_point_count_and_deep_neighbor(self):
        p=HeightPolicy()
        h,s,n=self.scene(.3);n.fill(2)
        self.assertFalse(obstacle_mask(h,s,n,p).any())
        n.fill(3)
        self.assertTrue(obstacle_mask(h,s,n,p)[7,7])
        h,s,n=self.scene(.3);n.fill(0);h[6,7]=-1.
        self.assertTrue(obstacle_mask(h,s,n,p)[7,7])

    def test_unknown_terrain_preserves_threshold_when_oa_off(self):
        p=HeightPolicy();p.set_stair_oa(1);p.set_terrain(99)
        old=p.height_range;p.set_stair_oa(0)
        self.assertEqual(p.height_range,old)
        p.set_terrain(2);self.assertEqual(p.height_range,.25)

    def test_invalid_input_is_not_treated_as_free_space(self):
        h,s,n=self.scene(0);h[5,5]=np.nan
        with self.assertRaises(ValueError):obstacle_mask(h,s,n,HeightPolicy())
        h[5,5]=-.55;n[5,5]=5
        with self.assertRaises(ValueError):obstacle_mask(h,s,n,HeightPolicy())


class GaitSequence(unittest.TestCase):
    def test_three_attempts_then_timeout(self):
        g=GaitSwitch();self.assertEqual(g.begin(0x3003,0x3002,0),0x3003)
        self.assertIsNone(g.update(0x3002,2.99))
        self.assertEqual(g.update(0x3002,3),0x3003)
        self.assertEqual(g.update(0x3002,6),0x3003)
        self.assertIsNone(g.update(0x3002,9));self.assertEqual(g.state,'timeout')
        g.update(0x3003,10);self.assertEqual(g.state,'timeout')

    def test_confirmation_at_deadline_prevents_retry(self):
        g=GaitSwitch();g.begin(0x3003,0x3002,0)
        self.assertIsNone(g.update(0x3003,3))
        self.assertEqual((g.state,g.attempts),('confirmed',1))

    def test_same_gait_lying_and_cancel(self):
        g=GaitSwitch();self.assertIsNone(g.begin(0x3002,0x3002,0))
        self.assertEqual(g.state,'confirmed')
        g.begin(0x3003,0x3002,0,lying=True);self.assertEqual(g.state,'rejected_lying')
        g.begin(0x3003,0x3002,0);g.cancel()
        self.assertIsNone(g.update(0x3002,5));self.assertEqual(g.state,'cancelled')


if __name__=='__main__':unittest.main()
