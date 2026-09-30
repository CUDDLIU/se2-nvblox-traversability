import math
import unittest
from dataclasses import replace
import numpy as np
from terrain import (Config, Terrain, Surface, rasterize, footprint_masks, evaluate,
                     certify_free, required_free_keys, observed_clearance_tops,
                     invalidate_occupied_evidence, geometry_display_state)


def plane(x0, x1, y0, y1, z, upward=True):
    v = np.array([[x0,y0,z], [x1,y0,z], [x1,y1,z], [x0,y1,z]])
    ids = [[0,1,2], [0,2,3]] if upward else [[2,1,0], [3,2,0]]
    return v, np.array(ids)


class TerrainTests(unittest.TestCase):
    def setUp(self):
        self.cfg = Config(side_margin=0, yaw_bins=8)
        self.masks = footprint_masks(self.cfg)

    def test_two_floors_and_overhang_clearance(self):
        terrain = Terrain(self.cfg)
        for key, height, up in ((0,0,True),(1,.8,False),(2,1.8,True)):
            terrain.replace_block(key, rasterize(*plane(0,.3,0,.3,height,up), self.cfg))
        terrain.rebuild()
        spans = terrain.columns[(1,1)]
        self.assertEqual(len(spans), 2)
        self.assertTrue(all(s.covered for s in spans))
        self.assertAlmostEqual(spans[0].z, 0)
        self.assertLess(spans[0].ceiling, .8)
        self.assertAlmostEqual(spans[1].z, 1.8)
        self.assertTrue(math.isinf(spans[1].ceiling))

    def test_duplicate_faces_do_not_fill_hole(self):
        terrain = Terrain(self.cfg)
        records = rasterize(*plane(.0,.045,.0,.1,0), self.cfg)
        for i in range(4):
            terrain.replace_block(i, records)
        terrain.rebuild()
        self.assertFalse(terrain.columns[(0,0)][0].covered)

    def test_remove_block_removes_support(self):
        terrain = Terrain(self.cfg)
        terrain.replace_block(0, rasterize(*plane(0,.2,0,.2,0), self.cfg))
        terrain.rebuild()
        self.assertTrue(terrain.columns)
        terrain.replace_block(0, {})
        terrain.rebuild()
        self.assertFalse(terrain.columns)

    def test_steep_or_downward_faces_are_not_support(self):
        terrain = Terrain(self.cfg)
        terrain.replace_block(0, rasterize(*plane(0,.2,0,.2,0,False), self.cfg))
        terrain.rebuild()
        self.assertFalse(terrain.columns)

    def floor(self, hole=None, low_ceiling=None):
        return [Surface(x,y,0, .7 if (x,y)==low_ceiling else math.inf, True)
                for x in range(-9,10) for y in range(-9,10) if (x,y)!=hole]

    def test_full_mask_detects_hole_missed_by_nine_points(self):
        surfaces = self.floor(hole=(1,1))
        geometric, _ = evaluate(surfaces, self.cfg, self.masks, np.full(len(surfaces), 2.))
        center = next(i for i,s in enumerate(surfaces) if (s.ix,s.iy)==(0,0))
        self.assertEqual(int(geometric[center]), 0)

    def test_low_ceiling_anywhere_under_footprint_blocks(self):
        surfaces = self.floor(low_ceiling=(1,1))
        geometric, verified = evaluate(surfaces, self.cfg, self.masks, np.full(len(surfaces), 2.))
        center = next(i for i,s in enumerate(surfaces) if (s.ix,s.iy)==(0,0))
        self.assertEqual(int(geometric[center]), 0)
        self.assertEqual(int(verified[center]), 0)

    def test_unseen_ceiling_is_unknown_not_free(self):
        surfaces = self.floor()
        geometry, verified = evaluate(surfaces, self.cfg, self.masks, np.zeros(len(surfaces)))
        self.assertTrue(np.any(geometry))
        self.assertFalse(np.any(verified))

    def test_mesh_safe_display_does_not_require_depth_clearance(self):
        surfaces=self.floor()
        geometry,verified=evaluate(surfaces,self.cfg,self.masks,np.zeros(len(surfaces)))
        center=next(i for i,s in enumerate(surfaces) if (s.ix,s.iy)==(0,0))
        self.assertEqual(int(verified[center]),0)
        self.assertEqual(geometry_display_state(geometry[center],0,self.cfg.yaw_bins),0)

    def test_mesh_display_distinguishes_yaw_unknown_and_stale(self):
        full=(1<<self.cfg.yaw_bins)-1
        self.assertEqual(geometry_display_state(full,0,self.cfg.yaw_bins),0)
        self.assertEqual(geometry_display_state(1,full-1,self.cfg.yaw_bins),1)
        self.assertEqual(geometry_display_state(0,1,self.cfg.yaw_bins),3)
        self.assertEqual(geometry_display_state(0,0,self.cfg.yaw_bins),2)
        self.assertEqual(geometry_display_state(full,0,self.cfg.yaw_bins,False),4)

    def test_native_yaw_checks_match_reference_for_layers_holes_and_ceilings(self):
        rng=np.random.default_rng(272)
        for seed in range(4):
            cfg=replace(self.cfg,yaw_bins=40)
            surfaces=[]
            for z in (0.,1.8):
                for x in range(-9,10):
                    for y in range(-9,10):
                        if seed and rng.random()<.01:
                            continue
                        surfaces.append(Surface(x,y,z+float(rng.uniform(-.008,.008)),
                            z+.8 if seed and rng.random()<.01 else math.inf,
                            not seed or rng.random()>.01))
            tops=np.array([s.z+rng.choice([.3,1.2,2.]) for s in surfaces])
            diagnostics_a,diagnostics_b={},{}
            masks=footprint_masks(cfg)
            a=evaluate(surfaces,cfg,masks,tops,diagnostics_a)
            b=evaluate(surfaces,cfg,masks,tops,diagnostics_b,native=False)
            np.testing.assert_array_equal(a,b)
            np.testing.assert_array_equal(diagnostics_a['unknown_support'],diagnostics_b['unknown_support'])

    def test_map_history_retains_distant_cells_until_delete_or_reset(self):
        terrain=Terrain(self.cfg)
        terrain.replace_block(0,rasterize(*plane(20,20.2,0,.2,0),self.cfg))
        terrain.rebuild()
        self.assertFalse(terrain.local_surfaces(0,0,4))
        self.assertTrue(terrain.local_surfaces(0,0,math.inf))
        terrain.replace_block(0,{})
        terrain.rebuild()
        self.assertFalse(terrain.local_surfaces(0,0,math.inf))

    def test_flat_complete_observed_floor_passes(self):
        surfaces = self.floor()
        _, verified = evaluate(surfaces, self.cfg, self.masks, np.full(len(surfaces), 2.))
        center = next(i for i,s in enumerate(surfaces) if (s.ix,s.iy)==(0,0))
        self.assertEqual(int(verified[center]), (1 << self.cfg.yaw_bins)-1)

    def test_stair_and_overlapping_floor_do_not_bridge(self):
        surfaces = [Surface(x,y,0 if x<=0 else .2, math.inf, True)
                    for x in range(-9,10) for y in range(-9,10)]
        surfaces += [Surface(x,y,2,math.inf,True) for x in range(-9,10) for y in range(-9,10)]
        geometry, _ = evaluate(surfaces, self.cfg, self.masks, np.full(len(surfaces),4.))
        center = next(i for i,s in enumerate(surfaces) if (s.ix,s.iy,s.z)==(0,0,0))
        upper = next(i for i,s in enumerate(surfaces) if (s.ix,s.iy,s.z)==(0,0,2))
        self.assertEqual(int(geometry[center]),0)
        self.assertNotEqual(int(geometry[upper]),0)

    def test_narrow_corridor_is_yaw_dependent(self):
        cfg = replace(self.cfg, yaw_bins=40)
        surfaces = [Surface(x,y,0,math.inf,True) for x in range(-10,11) for y in range(-3,4)]
        geom, _ = evaluate(surfaces, cfg, footprint_masks(cfg), np.full(len(surfaces),2.))
        i = next(i for i,s in enumerate(surfaces) if (s.ix,s.iy)==(0,0))
        self.assertTrue(int(geom[i]) & 1)
        self.assertFalse(int(geom[i]) & (1<<10))

    def test_continuous_sweep_covers_intermediate_orientations(self):
        cfg = replace(self.cfg, yaw_bins=40)
        mask = set(footprint_masks(cfg)[0][0])
        # Densely sampled perimeter points at angles not used to build the mask.
        for angle in np.linspace(-math.pi/40, math.pi/40, 117):
            rot = np.array([[math.cos(angle),-math.sin(angle)], [math.sin(angle),math.cos(angle)]])
            for x in np.linspace(-cfg.length/2,cfg.length/2,31):
                for y in (-cfg.width/2,cfg.width/2):
                    q = rot@np.array([x,y])
                    self.assertIn(tuple(np.floor(q/cfg.resolution+.5).astype(int)),mask)

    def test_depth_requires_whole_voxel_and_valid_pixels(self):
        keys = np.array([[0,0,20],[0,0,50]])
        depth = np.full((100,100),2.,dtype=np.float32)
        intrinsic = (50,50,50,50)
        free = certify_free(keys,depth,intrinsic,np.eye(4),self.cfg)
        self.assertEqual(free.tolist(),[True,False])
        depth[51,51] = 0
        self.assertFalse(certify_free(keys[:1],depth,intrinsic,np.eye(4),self.cfg)[0])

    def test_depth_window_ignores_pixels_outside_exact_projection(self):
        keys=np.array([[0,0,20]])
        depth=np.full((100,100),2.,dtype=np.float32)
        depth[49,49]=0  # Outside [50..55] x [50..55], inside old square bucket.
        self.assertTrue(certify_free(keys,depth,(50,50,50,50),np.eye(4),self.cfg)[0])
        depth[50,55]=0
        self.assertFalse(certify_free(keys,depth,(50,50,50,50),np.eye(4),self.cfg)[0])

    def test_depth_exact_window_at_image_edge(self):
        keys=np.array([[0,0,20]])
        depth=np.full((100,100),2.,dtype=np.float32)
        self.assertTrue(certify_free(keys,depth,(50,50,0,0),np.eye(4),self.cfg)[0])

    def test_compiled_rectangles_match_direct_pixel_checks(self):
        from fast_raster import rectangles_clear
        rng=np.random.default_rng(63)
        depth=rng.uniform(0,4,(24,31)).astype(np.float32)
        depth[1,2]=np.nan
        depth[5,6]=np.inf
        depth[10,8]=0
        lo=rng.integers(-2,20,(300,2))
        hi=lo+rng.integers(0,16,(300,2))
        thresholds=rng.uniform(0,3,300)
        thresholds[0]=np.nan
        expected=[]
        for (x0,y0),(x1,y1),threshold in zip(lo,hi,thresholds):
            valid=np.isfinite(threshold) and x0>=0 and y0>=0 and x1<31 and y1<24
            patch=depth[y0:y1+1,x0:x1+1] if valid else np.array([0])
            expected.append(valid and bool(np.all(np.isfinite(patch)&(patch>0)&(patch>threshold))))
        self.assertEqual(rectangles_clear(depth,lo,hi,thresholds).tolist(),expected)

    def test_batched_rebuild_matches_scalar_on_mixed_terrain_and_updates(self):
        from fast_raster import rasterize as fast
        rng=np.random.default_rng(147)
        batch,scalar=Terrain(self.cfg),Terrain(self.cfg)
        for step in range(3):
            for i in range(35):
                x=(i%7)*.1-.35
                y=(i//7)*.1-.25
                v,tri=plane(x,x+.1-rng.choice([0,.002,.02]),y,y+.1,float(i%3)*.8)
                v[:,2] += (v[:,0]-x)*rng.uniform(-.5,.5)+rng.normal(0,.003,4)
                records=fast(v,tri,self.cfg) if not (step==2 and i%5==0) else {}
                batch.replace_block(i,records)
                scalar.replace_block(i,records)
            batch.rebuild()
            scalar.rebuild_reference()
            self.assertEqual(batch.columns,scalar.columns)

    def test_free_evidence_expires_and_is_contiguous(self):
        s = [Surface(0,0,0,math.inf,True)]
        keys = required_free_keys(s,self.cfg)
        evidence = {tuple(k):10. for k in keys}
        self.assertGreater(observed_clearance_tops(s,evidence,self.cfg,11,5)[0],1.)
        self.assertEqual(observed_clearance_tops(s,evidence,self.cfg,16,5)[0],0.)
        evidence.pop(tuple(keys[2]))
        self.assertLess(observed_clearance_tops(s,evidence,self.cfg,11,5)[0],.3)

    def test_compiled_raster_matches_reference(self):
        from fast_raster import rasterize as fast
        rng = np.random.default_rng(41)
        cases = [plane(-.2,.2,-.2,.2,0), plane(0,.1,0,.1,.5,False)]
        for _ in range(50):
            cases.append((rng.uniform(-.2,.2,(3,3)),np.array([[0,1,2]])))
        for vertices,ids in cases:
            slow,quick = rasterize(vertices,ids,self.cfg),fast(vertices,ids,self.cfg)
            self.assertEqual(set(slow),set(quick))
            for key in slow:
                self.assertEqual(len(slow[key]),len(quick[key]))
                for a,b in zip(slow[key],quick[key]):
                    np.testing.assert_allclose(a[:2],b[:2],atol=1e-12)
                    self.assertEqual(a[2] is None,b[2] is None)
                    if a[2] is not None:
                        self.assertAlmostEqual(a[2].symmetric_difference(b[2]).area,0,places=10)

    def test_compiled_raster_rejects_invalid_indices(self):
        from fast_raster import rasterize as fast
        with self.assertRaises(ValueError):
            fast([[0,0,0]],[[0,0,5]],self.cfg)

    def test_batched_raster_preserves_block_ownership_and_deletion(self):
        from fast_raster import rasterize as fast, rasterize_blocks
        blocks={0:plane(-.2,.2,-.2,.2,0.),1:plane(-.1,.1,-.1,.1,1.),
                2:(np.empty((0,3)),np.empty((0,3),dtype=int))}
        records=rasterize_blocks(blocks,self.cfg)
        a,b=Terrain(self.cfg),Terrain(self.cfg)
        for key,args in blocks.items():
            a.replace_block(key,records[key])
            b.replace_block(key,fast(*args,self.cfg))
        a.rebuild();b.rebuild()
        self.assertEqual(a.columns,b.columns)
        deletion=rasterize_blocks({1:blocks[2]},self.cfg)
        a.replace_block(1,deletion[1]);a.rebuild()
        self.assertTrue(all(s.z<.1 for spans in a.columns.values() for s in spans))

    def test_batched_raster_rejects_cross_block_indices(self):
        from fast_raster import rasterize_blocks
        with self.assertRaises(ValueError):
            rasterize_blocks({0:([[0,0,0]],[[0,1,2]]),1:plane(0,1,0,1,0)},self.cfg)

    def test_only_bounded_small_support_cracks_are_tolerated(self):
        for gap, expected in ((.004, True), (.012, False)):
            terrain = Terrain(self.cfg)
            terrain.replace_block(0, rasterize(*plane(0,.05-gap/2,0,.1,0),self.cfg))
            terrain.replace_block(1, rasterize(*plane(.05+gap/2,.1,0,.1,0),self.cfg))
            terrain.rebuild()
            self.assertEqual(terrain.columns[(0,0)][0].covered, expected)

    def corrugated_cell(self, amplitude):
        vertices, ids = [], []
        for i in range(10):
            x0,x1 = i*.01,(i+1)*.01
            z0,z1 = amplitude*(i%2),amplitude*((i+1)%2)
            j=len(vertices)
            vertices.extend([[x0,0,z0],[x1,0,z1],[x1,.1,z1],[x0,.1,z0]])
            ids.extend([[j,j+1,j+2],[j,j+2,j+3]])
        return vertices, ids

    def test_noisy_facets_fit_flat_but_rough_surface_does_not(self):
        for amplitude, expected in ((.005,True),(.03,False)):
            terrain = Terrain(self.cfg)
            terrain.replace_block(0,rasterize(*self.corrugated_cell(amplitude),self.cfg))
            terrain.rebuild()
            self.assertEqual(any(s.covered for s in terrain.columns.get((0,0),[])),expected)

    def test_cell_plane_fit_does_not_allow_steep_ramp(self):
        for angle,expected in ((10,True),(25,False)):
            v,ids=plane(0,.1,0,.1,0.)
            v[:,2]=v[:,0]*math.tan(math.radians(angle))
            terrain=Terrain(self.cfg)
            terrain.replace_block(0,rasterize(v,ids,self.cfg))
            terrain.rebuild()
            self.assertEqual(terrain.columns[(0,0)][0].covered,expected)

    def test_step_inside_one_cell_is_not_smoothed_away(self):
        terrain=Terrain(self.cfg)
        terrain.replace_block(0,rasterize(*plane(0,.05,0,.1,0),self.cfg))
        terrain.replace_block(1,rasterize(*plane(.05,.1,0,.1,.04),self.cfg))
        terrain.rebuild()
        self.assertFalse(any(s.covered for s in terrain.columns[(0,0)]))

    def test_mesh_refresh_keeps_free_evidence_and_obstacle_invalidates(self):
        terrain=Terrain(self.cfg)
        terrain.replace_block(0,rasterize(*plane(0,.1,0,.1,0),self.cfg))
        terrain.rebuild()
        evidence={(0,0,2):10., (0,0,10):10., (0,0,18):10., (4,4,10):10.}
        self.assertEqual(invalidate_occupied_evidence(evidence,terrain,{(0,0)}),evidence)
        terrain.replace_block(1,rasterize(*plane(0,.1,0,.1,.51,False),self.cfg))
        retained=invalidate_occupied_evidence(evidence,terrain,{(0,0)})
        self.assertNotIn((0,0,10),retained)
        self.assertIn((0,0,18),retained)
        self.assertIn((4,4,10),retained)

    def test_unknown_support_diagnostic_differs_from_low_ceiling(self):
        for surfaces,unknown in ((self.floor(hole=(1,1)),True),
                                 (self.floor(low_ceiling=(1,1)),False)):
            diagnostics={}
            geometry,_=evaluate(surfaces,self.cfg,self.masks,np.full(len(surfaces),2.),diagnostics)
            i=next(i for i,s in enumerate(surfaces) if (s.ix,s.iy)==(0,0))
            self.assertEqual(int(geometry[i]),0)
            self.assertEqual(bool(diagnostics['unknown_support'][i]),unknown)


if __name__ == '__main__':
    unittest.main()
