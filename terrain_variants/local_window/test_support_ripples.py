"""Ground vertex extrema between support samples must not become obstacles."""
import unittest
import numpy as np
from native_mesh import NativeMesh
from paper_pipeline import Config
from pose_graph import validate


def scene(peak=.003, wall=False, roof=False, gap=False):
    axis=np.linspace(-.6,.6,121)
    v=np.array([(x,y,peak*max(0.,1-abs(x-.03)/.02)*max(0.,1-abs(y-.02)/.02)) for x in axis for y in axis])
    triangles=[]
    for i in range(120):
        for j in range(120):
            if gap and abs(axis[i])<.04:continue
            k=i*121+j
            triangles.extend([(k,k+121,k+122),(k,k+122,k+1)])
    if wall or roof:
        extra=([[-.02,-.2,0],[-.02,.2,0],[-.02,.2,.4],[-.02,-.2,.4]] if wall else
               [[-.6,-.6,.95],[-.6,.6,.95],[.6,.6,.95],[.6,-.6,.95]])
        k=len(v);v=np.vstack([v,extra]);triangles.extend([(k,k+1,k+2),(k,k+2,k+3)])
    mesh=NativeMesh();c=Config();mesh.update_blocks({(0,0,0):(v,np.array(triangles))},c)
    a=np.zeros(6);b=a.copy();b[0]=.01
    return validate(mesh,[np.r_[a,a],np.r_[a,b]],[0,0],c)


class SupportRipples(unittest.TestCase):
    def test_sub_quantum_ground_vertex_between_samples(self):
        self.assertTrue(scene().all())

    def test_higher_bump_remains_blocked(self):
        self.assertFalse(scene(peak=.03).any())

    def test_wall_and_low_roof_remain_blocked(self):
        self.assertFalse(scene(wall=True).any())
        self.assertFalse(scene(roof=True).any())

    def test_support_gap_remains_blocked(self):
        self.assertFalse(scene(gap=True).any())


if __name__=='__main__':unittest.main()
