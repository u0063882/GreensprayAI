"""Matched-height diagnostics must preserve trunk lean and side misalignment."""
import unittest
import numpy as np
from processing.plot_trunk_slices import fit_one_slice


class ThinSliceTests(unittest.TestCase):
    def test_lean_does_not_become_a_cross_side_offset(self):
        rng = np.random.default_rng(71)
        heights = [.2, .6, 1.]
        points = []; acquisition = []
        for height in heights:
            for scan in [0, 1, 2, 9, 10, 11]:
                angle = np.linspace(-np.pi, 0, 160) if scan < 9 else np.linspace(0, np.pi, 160)
                z = height+rng.uniform(-.025, .025, len(angle))
                centres = np.c_[2.+.10*z, 2.-.08*z]
                if scan >= 9: centres += [.012, .035]
                xy = centres+.07*np.c_[np.cos(angle), np.sin(angle)]+rng.normal(0, .0008, (len(angle), 2))
                points.append(np.c_[xy, z]); acquisition.append(np.full(len(z), scan))
        points = np.concatenate(points); acquisition = np.concatenate(acquisition)
        fitted=[]
        for height in heights:
            record, _ = fit_one_slice(points, acquisition, [2., 2.], height, bootstrap=25)
            self.assertTrue(record['usable_comparison'], record['issues'])
            self.assertAlmostEqual(record['centre_delta_across_m'], .035, delta=.004)
            self.assertAlmostEqual(record['centre_delta_along_m'], .012, delta=.004)
            np.testing.assert_allclose(record['side_A']['centre_m'], [2.+.10*height, 2.-.08*height], atol=.003)
            fitted.append(record['side_A']['centre_m'])
        self.assertGreater(fitted[-1][0]-fitted[0][0], .07)

    def test_absent_opposing_view_has_no_estimated_offset(self):
        angle=np.linspace(-np.pi, 0, 100)
        points=np.c_[2.+.07*np.cos(angle), 2.+.07*np.sin(angle), np.full(len(angle), .35)]
        record, _=fit_one_slice(points, np.zeros(len(angle), dtype=int), [2., 2.], .35, bootstrap=25)
        self.assertFalse(record['usable_comparison'])
        self.assertIsNone(record['side_B'])
        self.assertIsNone(record['centre_delta_across_m'])
        self.assertIsNone(record['delta_ci95_m'])

if __name__=='__main__': unittest.main()
