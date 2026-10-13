import math,sys,unittest
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from shapely.geometry import box
from shapely.strtree import STRtree
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from static_guard import StaticGuard
class Proofs(unittest.TestCase):
    def guard(self,mask):
        shapes=[box(0,0,1,1)]
        p=SimpleNamespace(config=SimpleNamespace(vertical_resolution=.01,max_step=.2),bins=40,
          polygons=[dict(floor=-1.,proven_yaw_mask=mask,yaw_mask=(1<<40)-1)],
          shapes=shapes,tree=STRtree(shapes),checker=SimpleNamespace(motions=lambda pairs:np.zeros(len(pairs),bool)))
        return StaticGuard(p)
    def test_refined_bits_never_become_polygon_proof(self):
        a=np.array([.1,.1,-1,0]);b=np.array([.9,.9,-1,0])
        self.assertFalse(self.guard(0).motions([(a,b)])[0]);self.assertTrue(self.guard(1).motions([(a,b)])[0])
    def test_surface_height_uses_only_containing_certified_nearby_floor(self):
        g=self.guard(1);a=np.array([[.2,.2,-1.006,0],[.8,.8,-1.006,0]])
        self.assertFalse(g.motions([(a[0],a[1])])[0])
        b=g.surface_poses(a)
        np.testing.assert_allclose(b[:,2],-1.)
        np.testing.assert_array_equal(a[:,[0,1,3]],b[:,[0,1,3]])
        self.assertTrue(g.motions([(b[0],b[1])])[0])
        for bad in ([[.2,.2,-1.21,0]],[[1.01,.2,-1.006,0]],[[.2,.2,-1.006,math.pi/2]]):
            np.testing.assert_array_equal(g.surface_poses(bad),bad)
        np.testing.assert_array_equal(self.guard(0).surface_poses(a),a)
        # A two-metre sparse guide can differ from local uneven ground by
        # more than one centimetre without changing the actual support layer.
        np.testing.assert_allclose(g.surface_poses([[.2,.2,-1.025,0]])[:,2],-1.)

    def test_surface_selection_does_not_create_a_portal(self):
        g=self.guard(1);p=g.planner
        p.shapes=[box(0,0,.5,1),box(.5,0,1,1)];p.tree=STRtree(p.shapes)
        p.polygons=[dict(floor=-1.,proven_yaw_mask=1),dict(floor=-.99,proven_yaw_mask=1)]
        a,b=g.surface_poses([[.1,.5,-.996,0],[.9,.5,-.996,0]])
        self.assertFalse(g.motions([(a,b)])[0])
        p.portal_lookup={(0,1):[dict(segment=[[.5,0,-.99],[.5,1,-.99]],yaw_mask=1)]}
        self.assertTrue(g.motions([(a,b)])[0])
    def test_adjacent_certificates_cover_a_continuous_motion(self):
        a=np.array([.1,.1,-1,0]);b=np.array([.9,.9,-1,0])
        g=self.guard(1);g.planner.shapes=[box(0,0,.5,1),box(.5,0,1,1)]
        g.planner.polygons=[dict(floor=-1.,proven_yaw_mask=1)]*2
        g.planner.tree=STRtree(g.planner.shapes)
        self.assertTrue(g.motions([(a,b)])[0])
        g.planner.shapes[1]=box(.6,0,1,1);g.planner.tree=STRtree(g.planner.shapes)
        self.assertFalse(g.motions([(a,b)])[0])
        g.planner.shapes[1]=box(.5,0,1,1);g.planner.tree=STRtree(g.planner.shapes)
        g.planner.polygons[1]=dict(floor=-1.,proven_yaw_mask=0)
        self.assertFalse(g.motions([(a,b)])[0])

    def test_subdivision_reuses_complete_certificates_not_only_mesh_fallback(self):
        g=self.guard(1);p=g.planner
        p.shapes=[box(i,0,i+1,1)for i in range(3)];p.tree=STRtree(p.shapes)
        p.polygons=[dict(floor=-1.+i*.01,proven_yaw_mask=1)for i in range(3)]
        p.portal_lookup={(i,i+1):[dict(segment=[[i+1,0,-1.+(i+1)*.01],[i+1,1,-1.+(i+1)*.01]],yaw_mask=1)]for i in range(2)}
        a=np.array([.1,.5,-1.,0.]);b=np.array([2.9,.5,-.98,0.])
        self.assertFalse(g.proof(a,b))
        self.assertTrue(g.proof(a,a));self.assertTrue(g.proof(b,b))
        # Each half has a complete portal proof despite the conservative mesh
        # fallback rejecting even the endpoints. Every accepted leaf is proved.
        self.assertTrue(g.motions([(a,b)])[0])
        p.portal_lookup={};self.assertFalse(g.motions([(a,b)])[0])

    def test_rotation_and_other_floor_require_their_own_proofs(self):
        a=np.array([.1,.1,-1,0]);b=np.array([.9,.9,-1,np.pi/2])
        self.assertFalse(self.guard(1).motions([(a,b)])[0])
        self.assertTrue(self.guard((1<<40)-1).motions([(a,b)])[0])
        b[2]=2;self.assertFalse(self.guard((1<<40)-1).motions([(a,b)])[0])

    def test_quantized_floor_transition_requires_crossed_certified_portal(self):
        a=np.array([.1,.1,-1.,0]);b=np.array([.9,.9,-.999,0])
        g=self.guard(1);p=g.planner
        p.shapes=[box(0,0,.5,1),box(.5,0,1,1)];p.tree=STRtree(p.shapes)
        p.polygons=[dict(floor=-1.,proven_yaw_mask=1),dict(floor=-.99,proven_yaw_mask=1)]
        portal=dict(segment=[[.5,0,-.99],[.5,1,-.99]],yaw_mask=1)
        self.assertFalse(g.proof(a,b))
        p.portal_lookup={(0,1):[portal]};self.assertTrue(g.proof(a,b))
        portal['yaw_mask']=0;self.assertFalse(g.proof(a,b));portal['yaw_mask']=1
        portal['segment']=[[.5,.7,-.99],[.5,1,-.99]];self.assertFalse(g.proof(a,b))
        portal['segment']=[[.5,0,-.99],[.5,1,-.99]]
        p.polygons[1]['floor']=-.98;self.assertFalse(g.proof(a,b))
        p.polygons[1]['floor']=-.99;p.shapes[1]=box(.6,0,1,1);p.tree=STRtree(p.shapes)
        self.assertFalse(g.proof(a,b))

sys.path[:0]=[str(Path(__file__).resolve().parents[2]/"global_planner"),str(Path(__file__).resolve().parents[2]/"terrain_variants/local_window")]
from geometry import MotionChecker
from paper_pipeline import Config

class StaticGuardTests(unittest.TestCase):
    def guard(self,wall=False,gap=False):
        vertices=[];triangles=[]
        def quad(v):
            k=len(vertices);vertices.extend(v);triangles.extend([(k,k+1,k+2),(k,k+2,k+3)])
        intervals=[(-2.,-.1),(.1,2.)] if gap else [(-2.,2.)]
        for a,b in intervals:quad([[a,-2,0],[b,-2,0],[b,2,0],[a,2,0]])
        if wall:quad([[0,-1,0],[0,1,0],[0,1,.5],[0,-1,.5]])
        c=Config();checker=MotionChecker({(0,0,0):(np.array(vertices),np.array(triangles))},c)
        return StaticGuard(SimpleNamespace(config=c,bins=c.yaw_bins,tree=STRtree([]),checker=checker))
    def test_open_ground_translation_and_rotation(self):
        g=self.guard();a=np.array([-.8,0,0,-.4]);b=np.array([.8,0,0,.4])
        self.assertTrue(g.motions([(a,b)])[0])
    def test_wall_between_clear_endpoints_is_not_skipped(self):
        g=self.guard(wall=True);a=np.array([-.8,0,0,0]);b=np.array([.8,0,0,0])
        self.assertTrue(g.motions([(a,a),(b,b)]).all())
        self.assertFalse(g.motions([(a,b)])[0])
    def test_support_gap_between_clear_endpoints_is_not_skipped(self):
        g=self.guard(gap=True);a=np.array([-.8,0,0,0]);b=np.array([.8,0,0,0])
        self.assertTrue(g.motions([(a,a),(b,b)]).all())
        self.assertFalse(g.motions([(a,b)])[0])
if __name__=='__main__':unittest.main()
