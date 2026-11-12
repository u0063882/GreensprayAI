import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from captures.structural_frame import estimate_structural_frame

class StructuralFrameTests(unittest.TestCase):
    def scene(self,angle):
        rng=np.random.default_rng(13);clouds=[];scores=[];labels=[];R=Rotation.from_euler('x',angle,degrees=True).as_matrix()
        for side in [-1,1]:
            parts=[]
            for x in [0.,.8,1.6,2.4]:
                z=rng.uniform(.45,2.95,5000);y=side*(.07+.25*z)+rng.normal(0,.015,len(z));p=np.c_[x+rng.normal(0,.14,len(z)),y,z]
                parts.append(p@R.T)
            p=np.concatenate(parts);clouds.append(p);scores.append(np.ones(len(p)));labels.append(str(side))
        return clouds,scores,labels
    def test_common_axis_recovery_preserves_asymmetric_cloud_and_branch_angles(self):
        p,s,l=self.scene(8);T,r=estimate_structural_frame(p,s,l)
        self.assertTrue(r['supported']);self.assertAlmostEqual(r['applied_rotation_deg'],-8,delta=1.)
        np.testing.assert_allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-12)
        q=p[0][[0,10,50]];m=q@T[:3,:3].T+T[:3,3]
        self.assertAlmostEqual(np.linalg.norm(q[0]-q[1]),np.linalg.norm(m[0]-m[1]),places=10)
        self.assertEqual(r['physical_gravity_reference_in_output'],T[:3,2].tolist())
    def test_upright_input_does_not_get_a_forced_rotation(self):
        p,s,l=self.scene(0);T,r=estimate_structural_frame(p,s,l);self.assertFalse(r['supported']);np.testing.assert_array_equal(T,np.eye(4))
    def test_single_side_cannot_straighten_a_v_arm_independently(self):
        p,s,l=self.scene(8)
        with self.assertRaises(ValueError):estimate_structural_frame(p[:1],s[:1],l[:1])

if __name__=='__main__':unittest.main()
