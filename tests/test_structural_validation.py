import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from captures.structural_validation import evaluate_pairs,structural_pairs


class StructuralValidationTests(unittest.TestCase):
    def test_multiheight_checks_detect_tilt_hidden_at_ground(self):
        rng=np.random.default_rng(60);p=np.c_[rng.uniform(-1,1,600),
            rng.uniform(-.5,.5,600),rng.uniform(.6,3.5,600)]
        n=np.tile([0.,1.,0.],(len(p),1))
        pairs=[dict(i=0,j=1,p=p,q=p.copy(),n=n,cross_side=True)]
        C=np.tile(np.eye(4),(2,1,1));good=evaluate_pairs(pairs,C)
        self.assertTrue(good['observable']);self.assertTrue(good['validation_passed'])
        C[1,:3,:3]=Rotation.from_euler('x',2,degrees=True).as_matrix()
        bad=evaluate_pairs(pairs,C)
        self.assertTrue(bad['observable']);self.assertFalse(bad['validation_passed'])
        self.assertGreater(bad['checks']['cross_side']['after']['rmse_m'],.05)

    def test_asymmetric_geometry_needs_no_symmetry_prior(self):
        rng=np.random.default_rng(61);p=rng.normal(size=(700,3));p[:,2]=rng.uniform(.6,3.5,700)
        p[:,1]+=.2*p[:,2]**2
        shift=np.array([.01,.02,-.005]);q=p+shift
        n=rng.normal(size=p.shape);n/=np.linalg.norm(n,axis=1)[:,None]
        C=np.tile(np.eye(4),(2,1,1));C[1,:3,3]=-shift
        result=evaluate_pairs([dict(i=0,j=1,p=p,q=q,n=n,cross_side=True)],C)
        self.assertTrue(result['validation_passed'])
        self.assertLess(result['checks']['cross_side']['after']['euclidean_rmse_m'],1e-12)

    def test_opposite_bark_faces_cannot_be_squashed_together(self):
        x,z=np.meshgrid(np.linspace(-.3,.3,100),np.linspace(.6,3.4,150))
        p=np.c_[x.ravel(),np.zeros(x.size),z.ravel()];q=p+[0,.025,0]
        # Parallel measured surfaces, viewed from opposite sides: their outward
        # normals oppose, even though they are close enough for naive ICP.
        pairs=structural_pairs([p,q],[np.ones(len(p)),np.ones(len(q))],
            [[0,-2,1.5],[0,2,1.5]],['A','B'])
        self.assertFalse(any(pair['cross_side'] for pair in pairs))
        result=evaluate_pairs(pairs,np.tile(np.eye(4),(2,1,1)))
        self.assertFalse(result['observable']);self.assertFalse(result['validation_passed'])

    def test_scale_is_rejected(self):
        C=np.tile(np.eye(4),(2,1,1));C[1,0,0]=.98
        with self.assertRaises(ValueError):evaluate_pairs([],C)

if __name__=='__main__':unittest.main()
