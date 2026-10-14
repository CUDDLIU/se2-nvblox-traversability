import unittest
import tempfile
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from clearance import ClearancePreference


def field(cells):
    return ClearancePreference([dict(cells=cells)], SimpleNamespace(resolution=.1, vertical_resolution=.01, max_step=.2))


class ClearanceTests(unittest.TestCase):
    def test_green_preference_uses_display_masks_and_preserves_height(self):
        cells=np.array([[x,y,z] for z in (0,300) for x in range(20) for y in range(10)])
        states=np.zeros((len(cells),4,6));states[:,:,5]=np.pi/8
        # Lower floor: yellow/red bottom half; upper floor entirely green.
        comfortable=np.where((cells[:,1]>=5)|(cells[:,2]==300),15,1)
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'field.npz'
            np.savez(path,cells=cells,states=states,comfortable=comfortable)
            c=ClearancePreference([dict(cells=cells)],
                SimpleNamespace(resolution=.1,vertical_resolution=.01,max_step=.2,yaw_bins=4),field_path=path)
            value=c.region_penalty(np.array([[1.,.2,0.],[1.,.8,0.],[1.,.2,3.]]))
            self.assertGreater(value[0],.8);self.assertEqual(value[1],0.);self.assertEqual(value[2],0.)
            # A colored passage stays finite/reachable, never becomes a wall.
            q=np.array([[1.,.2,0.]])
            self.assertTrue(np.isfinite(c.edge_factors(q,q)).all())
            points=np.array([[.93,.481,.012]])
            _,gradient=c.region_penalty(points,gradient=True)
            for axis in range(3):
                a,b=points.copy(),points.copy();a[:,axis]+=1e-6;b[:,axis]-=1e-6
                numeric=(c.region_penalty(a)-c.region_penalty(b))/(2e-6)
                self.assertAlmostEqual(gradient[0,axis],numeric[0],places=5)

    def test_tread_and_polygon_seams_do_not_repulse_path(self):
        cells = [[x, y, 0 if x < 10 else 15] for x in range(20) for y in range(10)]
        c = field(cells)
        self.assertGreater(c.distances([[1., .5, .15]])[0], .49)
        self.assertLess(c.distances([[1., .02, .15]])[0], .03)

    def test_stacked_floor_cannot_fill_a_lower_floor_hole(self):
        cells = [[x, y, 0] for x in range(20) for y in range(10) if x != 10]
        cells += [[x, y, 300] for x in range(20) for y in range(10)]
        c = field(cells)
        self.assertLess(c.distances([[1., .5, 0.]])[0], 1e-8)
        self.assertGreater(c.distances([[1., .5, 3.]])[0], .49)

    def test_segment_interior_and_turn_location_are_penalized(self):
        cells = [[x, y, 0] for x in range(40) for y in range(20)
                 if not (18 <= x <= 21 and y < 10)]
        c = field(cells)
        # Endpoints are clear; the segment passes near an interior corner.
        f = c.edge_factors(np.array([[1.,1.02,0.],[1.,1.6,0.]]), np.array([[3.,1.02,0.],[3.,1.6,0.]]))
        self.assertGreater(f[0], f[1]+.2)
        turns = np.array([[1.8,1.02,0.],[1.,1.6,0.]])
        self.assertGreater(c.edge_factors(turns, turns)[0], c.edge_factors(turns, turns)[1])

    def test_string_objective_gradient(self):
        c = field([[x,y,0] for x in range(30) for y in range(10)])
        p = np.array([[.351,.131,.012],[1.337,.193,.019],[2.513,.122,.008]])
        _, grad = c.objective(p)
        numeric = np.zeros_like(p)
        eps = 1e-6
        for i in range(len(p)):
            for j in range(3):
                a,b = p.copy(),p.copy();a[i,j]+=eps;b[i,j]-=eps
                numeric[i,j]=(c.objective(a)[0]-c.objective(b)[0])/(2*eps)
        np.testing.assert_allclose(grad,numeric,atol=2e-5)


if __name__ == '__main__':
    unittest.main()
