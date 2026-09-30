"""Exercise the production cell classifier with metric Mesh geometry."""
import math
import unittest
from dataclasses import replace
import numpy as np
from paper_pipeline import Config,Engine


def corridor(width=.506,angle=0.,offset=.05,gap=False,obstacle=False,ceiling=None):
    vertices=[];triangles=[]
    def quad(p):
        i=len(vertices);vertices.extend(p);triangles.extend([(i,i+1,i+2),(i,i+2,i+3)])
    for lo,hi in ([(-2,-.15),(.15,2)] if gap else [(-2,2)]):
        quad([(lo,-width/2,0),(hi,-width/2,0),(hi,width/2,0),(lo,width/2,0)])
    for y in (-width/2,width/2):
        quad([(-2,y,0),(2,y,0),(2,y,1.2),(-2,y,1.2)])
    if obstacle:quad([(-.02,-width/2,.4),(.02,-width/2,.4),(.02,width/2,.4),(-.02,width/2,.4)])
    if ceiling is not None:quad([(-2,-width/2,ceiling),(-2,width/2,ceiling),(2,width/2,ceiling),(2,-width/2,ceiling)])
    v=np.array(vertices,dtype=float);c,s=math.cos(angle),math.sin(angle)
    v[:,:2]=v[:,:2]@np.array([[c,s],[-s,c]]);v[:,1]+=offset
    return {(0,0,0):(v,np.array(triangles))}


def run(blocks):
    e=Engine(Config(tile_cells=8,max_step=.2,max_slope_deg=40.,surface_normal_filter=True,stair_riser_filter=True),native_mesh=True)
    e.current_position=(0.,.05,.565);e.update(blocks)
    e.process(10000,defer_polygons=True,priority_position=e.current_position)
    return e


def cells(e):
    return [(s,int(r.masks[i]),r.pose_states[i]) for r in e.results.values() for i,s in enumerate(r.spans)
            if s.z<.2 and abs((s.ix+.5)*.1)<.9]


class PhysicalClearanceTests(unittest.TestCase):
    def test_just_body_width_yields_original_orange_cells(self):
        e=run(corridor());valid=[(s,m,p) for s,m,p in cells(e) if m]
        self.assertEqual({s.iy for s,m,p in valid},{0})
        self.assertEqual({s.ix for s,m,p in valid},set(range(-9,9)))
        for s,m,p in valid:
            self.assertEqual(m,(1<<0)|(1<<20))
            self.assertAlmostEqual(float(np.nansum(p[:,5]-p[:,4])),0.,places=6)

    def test_off_grid_and_non_bin_orientation(self):
        angle=math.radians(17.3)
        e=run(corridor(angle=angle,offset=.023))
        valid=[p for s,m,p in cells(e) if m]
        self.assertGreater(len(valid),12)
        for p in valid:
            p=p[np.isfinite(p[:,0])]
            self.assertTrue(np.all(np.abs(np.sin(p[:,3]-angle))<1e-6))
            self.assertTrue(np.all(np.abs(-math.sin(angle)*p[:,0]+math.cos(angle)*(p[:,1]-.023))<1e-6))

    def test_too_narrow_and_low_roof_remain_blocked(self):
        for blocks in (corridor(width=.500),corridor(ceiling=.95)):
            self.assertFalse(any(m for s,m,p in cells(run(blocks))))

    def test_real_obstacle_and_missing_floor_remain_blocked(self):
        for blocks in (corridor(obstacle=True),corridor(gap=True)):
            self.assertFalse(any(m for s,m,p in cells(run(blocks)) if abs((s.ix+.5)*.1)<.25))

    def test_new_observation_and_parameter_changes_revoke_permission(self):
        e=run(corridor());self.assertTrue(any(m for s,m,p in cells(e)))
        e.update(corridor(obstacle=True));e.process(10000,defer_polygons=True,priority_position=e.current_position)
        self.assertFalse(any(m for s,m,p in cells(e) if abs((s.ix+.5)*.1)<.25))
        e.update(corridor());e.process(10000,defer_polygons=True,priority_position=e.current_position)
        self.assertTrue(any(m for s,m,p in cells(e)))
        e.reconfigure(replace(e.config,width=.52));e.process(10000,defer_polygons=True,priority_position=e.current_position)
        self.assertFalse(any(m for s,m,p in cells(e)))

    def test_angular_measure_increases_with_available_width(self):
        measures=[]
        for width in (.506,.54,.60,.75):
            valid=[p for s,m,p in cells(run(corridor(width=width))) if s.ix==0 and s.iy==0 and m]
            self.assertEqual(len(valid),1)
            measures.append(float(np.nansum(valid[0][:,5]-valid[0][:,4])))
        self.assertTrue(all(a<b for a,b in zip(measures,measures[1:])),measures)

if __name__=='__main__':unittest.main()
