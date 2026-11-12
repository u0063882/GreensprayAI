import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from captures.terrain import fit_plane,plane_pair_residuals

class TerrainTests(unittest.TestCase):
    def test_different_xy_on_sloping_ground_is_not_a_height_error(self):
        p=np.array([.01,.01,.12*.01-.08*.01+.03]);q=np.array([.14,.13,.12*.14-.08*.13+.03])
        n=np.array([-.12,.08,1.]);n/=np.linalg.norm(n)
        self.assertGreater(abs(p[2]-q[2]),.005)
        np.testing.assert_allclose(plane_pair_residuals(p,q,n,n),0,atol=1e-15)

    def test_real_tilt_and_height_error_still_have_nonzero_residual(self):
        p=np.array([1.,.5,.12]);n=np.array([-.08,-.08,1.]);n/=np.linalg.norm(n)
        R=Rotation.from_euler('x',2,degrees=True).as_matrix()
        d=plane_pair_residuals(p,p,n,n,Rj=R,tj=[0,0,.02])
        self.assertGreater(np.linalg.norm(d),.03)

    def test_local_plane_retains_real_slope_despite_sparse_vegetation(self):
        rng=np.random.default_rng(220);xy=rng.uniform(-.16,.16,(400,2));z=.12*xy[:,0]-.08*xy[:,1]+.03+rng.normal(0,.001,400);z[::9]+=.06
        fit=fit_plane(np.c_[xy,z]);self.assertIsNotNone(fit)
        np.testing.assert_allclose(fit['slope'],[.12,-.08],atol=.006)
        self.assertAlmostEqual(fit['centroid'][2]-(fit['centroid'][:2]@[.12,-.08]),.03,delta=.001)

if __name__=='__main__':unittest.main()
