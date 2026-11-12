import unittest
import numpy as np
from tools.check_under_tree_ground import narrow_ground,compare_fixed_tree
from tools.check_tree_ground_angles import fit_section


class UnderTreeGroundTests(unittest.TestCase):
    def test_under_trunk_strip_excludes_raised_shoulders_preserves_real_slope(self):
        rng=np.random.default_rng(91);x=rng.uniform(-3,3,18000);y=rng.uniform(-1.5,1.5,len(x))
        z=.015*x+.03*y+.22*np.maximum(abs(y)-.32,0)+rng.normal(0,.001,len(x));p=np.c_[x,y,z];before=p.copy()
        n=np.array([-.015,-.10,1.]);n/=np.linalg.norm(n)
        g=dict(coef=np.array([.015,.10,0.]),normal=n,offset=0.,rmse_m=.02)
        row=dict(centre=np.zeros(2),along=np.array([1.,0.]),across=np.array([0.,1.]),extent=np.array([-2.9,2.9]))
        allowed=x<2.
        fit,ix,detail=narrow_ground(p,g,row,allowed=allowed)
        self.assertTrue(np.all(allowed[ix]))
        np.testing.assert_allclose(fit['coef'][:2],[.015,.03],atol=.001)
        self.assertLess(abs(p[ix,1]).max(),.25);self.assertGreater(detail['supported_returns'],100)
        np.testing.assert_array_equal(p,before)

    def test_points_in_line_do_not_establish_across_row_slope(self):
        rng=np.random.default_rng(92);x=rng.uniform(-3,3,3000);y=rng.uniform(-.015,.015,len(x));p=np.c_[x,y,rng.normal(0,.001,len(x))]
        g=dict(coef=np.zeros(3),normal=np.array([0.,0.,1.]),offset=0.,rmse_m=.001)
        row=dict(centre=np.zeros(2),along=np.array([1.,0.]),across=np.array([0.,1.]),extent=np.array([-3.,3.]))
        with self.assertRaisesRegex(ValueError,'coverage|line'):narrow_ground(p,g,row)

    def test_ground_reference_change_never_refits_or_reselects_tree(self):
        rng=np.random.default_rng(93);x=rng.uniform(-3,3,5000);z=rng.uniform(.35,3.8,len(x));p=np.c_[x,.05*x+.13*z+rng.normal(0,.02,len(x)),z]
        g=dict(coef=np.zeros(3),normal=np.array([0.,0.,1.]),offset=0.,rmse_m=.001);base=fit_section(p,g)
        changed=dict(coef=np.array([.02,.07,-.10]),normal=np.array([-.02,-.07,1.])/np.linalg.norm([-.02,-.07,1.]),offset=.10,rmse_m=.002)
        new=compare_fixed_tree(base,changed)
        for key in ['tree_plane_Y_aX_bZ_c','tree_line_Y_bZ_c','tree_returns','section_X_m','tree_across_rms_m']:self.assertEqual(base[key],new[key])
        self.assertGreater(abs(new['angle_deg']-base['angle_deg']),3.)

if __name__=='__main__':unittest.main()
