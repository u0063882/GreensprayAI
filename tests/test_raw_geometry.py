import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from captures.raw_geometry import wall_fit,plane,fit_stem
from tools.audit_raw_capture_geometry import geographic_sides


def row():return dict(centre=np.zeros(3),along=np.array([0.,1.,0.]),outward=np.array([1.,0.,0.]),extent=np.array([-1.2,1.2]))


def wall(angle,seed=30):
    rng=np.random.default_rng(seed);h,z=np.meshgrid(np.linspace(-1.1,1.1,45),np.linspace(.66,2.34,55));h=h.ravel();z=z.ravel();u=.12+np.tan(np.deg2rad(angle))*z+.025*h+rng.normal(0,.004,len(h));return np.c_[u,h,z]


class RawGeometryTests(unittest.TestCase):
    def test_asymmetry_is_measured_without_a_symmetry_target(self):
        r=row();e=wall_fit(wall(25),np.ones(2475),r);w=wall_fit(wall(8,31),np.ones(2475),r)
        self.assertAlmostEqual(e['outward_tilt_from_vertical_deg'],25,delta=.3)
        self.assertAlmostEqual(w['outward_tilt_from_vertical_deg'],8,delta=.3)
        self.assertGreater(e['outward_tilt_from_vertical_deg']-w['outward_tilt_from_vertical_deg'],16)

    def test_yaw_and_origin_choice_do_not_change_the_inclination(self):
        p=wall(25);r=row();before=wall_fit(p,np.ones(len(p)),r)
        R=Rotation.from_euler('z',37,degrees=True).as_matrix();offset=np.array([4.,-3.,1.7]);q=p@R.T+offset
        r2=dict(centre=offset,along=R@r['along'],outward=R@r['outward'],extent=r['extent'])
        after=wall_fit(q,np.ones(len(q)),r2)
        self.assertAlmostEqual(before['outward_tilt_from_vertical_deg'],after['outward_tilt_from_vertical_deg'],places=7)

    def test_ground_slope_is_retained_and_observations_are_not_modified(self):
        rng=np.random.default_rng(35);xy=rng.uniform(-2,2,(6000,2));p=np.c_[xy,.035*xy[:,0]-.018*xy[:,1]-1.6+rng.normal(0,.001,6000)];copy=p.copy();fit=plane(p)
        np.testing.assert_allclose(fit['coef'][:2],[.035,-.018],atol=.001)
        np.testing.assert_array_equal(p,copy)
        self.assertGreater(np.rad2deg(np.arccos(fit['normal'][2])),2)

    def test_a_genuinely_leaning_raw_stem_is_not_made_upright(self):
        rng=np.random.default_rng(36);axis=np.array([.12,-.06,1.]);axis/=np.linalg.norm(axis);a=np.array([1.,0.,0.]);a-=axis*np.dot(a,axis);a/=np.linalg.norm(a);b=np.cross(axis,a)
        theta,t=np.meshgrid(np.linspace(.55*np.pi,1.45*np.pi,25),np.linspace(-.15,.15,24));theta=theta.ravel();t=t.ravel();p=np.array([2.,0.,-1.3])+t[:,None]*axis+.075*(np.cos(theta)[:,None]*a+np.sin(theta)[:,None]*b)+rng.normal(0,.0005,(len(t),3));copy=p.copy()
        r=row();r['outward']=-r['outward'];f=fit_stem(p,None,r,np.random.default_rng(38));self.assertIsNotNone(f)
        self.assertAlmostEqual(f['inclination_from_gravity_deg'],np.rad2deg(np.arccos(axis[2])),delta=.8)
        self.assertAlmostEqual(f['radius_m'],.075,delta=.003);np.testing.assert_array_equal(p,copy)

    def test_geographic_east_west_is_not_assumed_from_folder_letters(self):
        reports=[dict(side='sideA',latitude=51.,longitude=5.),dict(side='sideB',latitude=51.,longitude=5.0001)]
        mapping,_=geographic_sides(reports);self.assertEqual(mapping,{'sideA':'west','sideB':'east'})

if __name__=='__main__':unittest.main()
