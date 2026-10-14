"""Accelerated filtering must preserve PCL points and planner decisions."""
import unittest
import numpy as np
from core import Core
class CloudEquivalence(unittest.TestCase):
 @classmethod
 def setUpClass(cls):cls.core=Core()
 @classmethod
 def tearDownClass(cls):cls.core.close()
 def compare(self,points,passability=False):
  c=self.core;reference=c.cloud(points,passability,reference=True);c.goal();old=c.plan((1.5,0,0),(5,0,0))
  actual=c.cloud(points,passability);c.goal();new=c.plan((1.5,0,0),(5,0,0))
  np.testing.assert_array_equal(actual,reference)
  self.assertEqual((old.x,old.y,old.w,old.id,old.status),(new.x,new.y,new.w,new.id,new.status))
 def test_empty_nonfinite_boundaries_and_sparse_points(self):
  rng=np.random.default_rng(16)
  points=rng.uniform([-4.1,-4.1,-.4],[4.1,4.1,.7],(12000,3)).astype('f4')
  edges=np.array([[x,y,z] for x in [-4.,-.45,.45,4.] for y in [-4.,-.25,.25,4.] for z in [-.3,0,.6]],'f4')
  for a in (np.empty((0,3),'f4'),np.vstack([points,edges,[[np.nan,0,0],[0,np.inf,0]]])):
   for p in (False,True):self.compare(a,p)
 def test_dense_surfaces_and_radius_boundary(self):
  rng=np.random.default_rng(92)
  for p in (False,True):
   wall=rng.uniform([.5,-2,-.2],[.6,2,.6],(25000,3)).astype('f4')
   self.compare(wall,p)
  # Exactly four peers including samples on / adjacent to the 10 cm sphere.
  for dx in (-1e-6,0,1e-6):
   cluster=np.array([[1,0,0],[1.1+dx,0,0],[1,.02,0],[1,0,.02],[1,-.02,0]],'f4')
   self.compare(cluster)
if __name__=='__main__':unittest.main()
