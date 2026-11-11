"""Trunk figures must report diameters in cm without changing measured points."""
from pathlib import Path
import tempfile,unittest
import numpy as np
from captures.trunk_diagnostics import plot_dimensioned_slices


class DimensionedTrunkTests(unittest.TestCase):
    def test_known_opposite_bark_arcs_keep_fourteen_cm_diameter(self):
        clouds=[]
        for offset in [0,np.pi]:
            theta=np.linspace(.1,np.pi-.1,150)+offset
            clouds.append(np.c_[.07*np.cos(theta),.07*np.sin(theta),np.full(len(theta),.18)])
        original=[p.copy() for p in clouds]
        with tempfile.TemporaryDirectory() as temporary:
            path=Path(temporary)/'slices.png'
            report=plot_dimensioned_slices(clouds,['sideA','sideB'],[(0,np.array([0.,0.]))],path,'synthetic',heights=(.18,))
            self.assertTrue(path.is_file());self.assertTrue(path.with_suffix('.json').is_file())
        for row in report['slices']:
            self.assertAlmostEqual(row['fit']['diameter_cm'],14.,places=4)
            self.assertEqual(row['thickness_m'],.02)
        for p,q in zip(clouds,original):np.testing.assert_array_equal(p,q)


if __name__=='__main__':unittest.main()
