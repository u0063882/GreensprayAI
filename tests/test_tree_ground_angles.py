import unittest
import numpy as np
from tools.check_tree_ground_angles import fit_section


class TreeGroundAngleTests(unittest.TestCase):
    def test_across_section_angle_preserves_slope_and_input_geometry(self):
        rng=np.random.default_rng(66)
        x=rng.uniform(-3,3,9000);z=rng.uniform(.5,3.5,9000)
        p=np.c_[x,.04*x+.18*z+rng.normal(0,.002,len(x)),z];before=p.copy()
        n=np.array([-.08,-.035,1.]);n/=np.linalg.norm(n)
        g=dict(coef=np.array([.08,.035,0.]),normal=n,offset=0.,rmse_m=.001)
        r=fit_section(p,g)
        expected=90-abs(np.degrees(np.arctan(.18))+np.degrees(np.arctan(.035)))
        self.assertAlmostEqual(r['angle_deg'],expected,delta=.05)
        np.testing.assert_array_equal(p,before)
        self.assertFalse(r['structural_centre_verified'])

    def test_two_v_arms_remain_in_mean_plane(self):
        x,z=np.meshgrid(np.linspace(-3,3,70),np.linspace(.5,3.5,70));x=x.ravel();z=z.ravel()
        p=np.concatenate([np.c_[x,sign*.4*z,z] for sign in [-1,1]])
        g=dict(coef=np.zeros(3),normal=np.array([0.,0.,1.]),offset=0.,rmse_m=0.)
        r=fit_section(p,g)
        self.assertAlmostEqual(r['angle_deg'],90,delta=.1)
        self.assertGreater(r['tree_across_rms_m'],.6)

if __name__=='__main__':unittest.main()
