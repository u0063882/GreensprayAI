"""Analytic geometry checks: distinguish real diameter from centre misalignment."""
import unittest
import numpy as np
from processing.check_cross_row import circle_fit

class TrunkGeometryTests(unittest.TestCase):
    def test_opposing_surfaces_do_not_need_to_coincide(self):
        rng=np.random.default_rng(5)
        angle_a=np.linspace(np.pi,2*np.pi,500)
        angle_b=np.linspace(0,np.pi,500)
        ca=np.array([3.,1.95]);cb=ca+[0,.04];radius=.08
        a=ca+radius*np.c_[np.cos(angle_a),np.sin(angle_a)]+rng.normal(0,.0015,(500,2))
        b=cb+radius*np.c_[np.cos(angle_b),np.sin(angle_b)]+rng.normal(0,.0015,(500,2))
        fa,*_=circle_fit(a,[3.,1.97,radius]);fb,*_=circle_fit(b,[3.,1.97,radius])
        self.assertAlmostEqual(fa[2],radius,delta=.004)
        self.assertAlmostEqual(fb[2],radius,delta=.004)
        self.assertAlmostEqual(fb[1]-fa[1],.04,delta=.005)
        # Opposing-face separation is larger than the centre error.
        self.assertGreater(np.median(b[:,1])-np.median(a[:,1]),.1)

    def test_short_arc_is_flagged_as_ill_conditioned(self):
        t=np.linspace(-.25,.25,120);p=np.c_[.08*np.cos(t),.08*np.sin(t)]
        _,_,coverage,condition=circle_fit(p,[0,0,.08])
        self.assertLess(coverage,40)
        self.assertGreater(condition,150)

if __name__=='__main__':unittest.main()
