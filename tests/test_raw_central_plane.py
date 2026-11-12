import unittest
import numpy as np
from captures.raw_central_plane import central_tree_profile,central_plane


def make(side,shape,slope=.08,offset=0.,seed=50):
    rng=np.random.default_rng(seed);h,z=np.meshgrid(np.linspace(-1.1,1.1,45),np.linspace(.31,2.99,85));h=h.ravel();z=z.ravel();sign=1 if side=='east' else -1
    centre=.04+slope*z;u=shape(z)+sign*centre+offset+rng.normal(0,.0005,len(z))
    row=dict(centre=np.array([2.,0.,-1.6]),along=np.array([0.,1.,0.]),outward=np.array([-1.,0.,0.]),extent=np.array([-1.2,1.2]))
    p=row['centre']+u[:,None]*row['outward']+h[:,None]*row['along']+z[:,None]*[0.,0.,1.]
    normal=-sign*.015*row['outward']+np.array([0.,0.,1.]);normal/=np.linalg.norm(normal)
    report=dict(row=row,ground=dict(normal=normal));data=dict(xyz=p)
    return dict(scan=side+str(seed),geographic_side=side,central_profile=central_tree_profile(data,report))


class RawCentralPlaneTests(unittest.TestCase):
    def test_different_growth_shapes_recover_the_same_freely_leaning_central_plane(self):
        for shape in [lambda z:.25+.19*z,lambda z:.35+.12*np.sin(z),lambda z:.2+.45*(1-np.exp(-2*z))]:
            r=central_plane([make('east',shape),make('west',shape,seed=51)])
            self.assertAlmostEqual(r['central_plane_tilt_from_imu_vertical_deg'],np.rad2deg(np.arctan(.08)),delta=.35)
            self.assertLess(r['central_plane_to_ground_deg'],86)
            self.assertGreater(r['central_plane_to_ground_deg'],83)

    def test_track_balance_is_not_changed_by_more_recordings_from_one_side(self):
        shape=lambda z:.3+.16*z;a=make('east',shape);b=make('west',shape,seed=52)
        first=central_plane([a,b]);duplicate=dict(a,scan='east_duplicate');second=central_plane([a,duplicate,b])
        self.assertAlmostEqual(first['central_plane_to_ground_deg'],second['central_plane_to_ground_deg'],places=7)

    def test_independent_horizontal_origins_do_not_define_the_inclination(self):
        shape=lambda z:.3+.16*z;a=make('east',shape);b=make('west',shape,seed=52);old=central_plane([a,b])
        for r,delta in [(a,.22),(b,-.18)]:
            r['central_profile']['profile']=[[z,u+delta] for z,u in r['central_profile']['profile']]
        new=central_plane([a,b]);self.assertAlmostEqual(old['central_plane_to_ground_deg'],new['central_plane_to_ground_deg'],places=6)

    def test_missing_height_coverage_is_not_extrapolated(self):
        shape=lambda z:.3+.16*z;a=make('east',shape);b=make('west',shape,seed=52)
        b['central_profile']['profile']=[p for p in b['central_profile']['profile'] if 1.0<p[0]<2.6]
        r=central_plane([a,b]);lo,hi=r['common_height_range_m'];self.assertGreaterEqual(lo,b['central_profile']['profile'][0][0]);self.assertLessEqual(hi,b['central_profile']['profile'][-1][0])

if __name__=='__main__':unittest.main()
