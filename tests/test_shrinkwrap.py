"""Validate closed proxy topology and scale on an analytic observed sphere."""
import unittest
import numpy as np
from processing.shrinkwrap import build_wrap

class WrapGeometryTests(unittest.TestCase):
    def test_sphere_is_closed_and_has_expected_offset_scale(self):
        rng=np.random.default_rng(42);p=rng.normal(size=(2000,3));p=.1*p/np.linalg.norm(p,axis=1)[:,None]
        mesh,_=build_wrap(p,voxel=.0125,offset=.025,closing=.025,min_volume=0)
        self.assertTrue(mesh.is_watertight())
        self.assertTrue(mesh.is_vertex_manifold())
        radius=np.linalg.norm(np.asarray(mesh.vertices),axis=1)
        self.assertAlmostEqual(float(np.median(radius)),.125,delta=.02)
        self.assertGreater(mesh.get_volume(),4/3*np.pi*.1**3)

    def test_grid_budget_rejects_unbounded_input(self):
        p=np.array([[0,0,0],[100,0,0],[0,100,0],[0,0,100]])
        with self.assertRaises(ValueError):build_wrap(p,voxel=.01)

if __name__=='__main__':unittest.main()
