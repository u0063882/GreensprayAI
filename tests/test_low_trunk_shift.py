import unittest
from processing.assess_low_trunk_shift import assess_shift
from processing.plot_trunk_slices import validate_alignment_slices


def sample_report(deltas):
    candidates=[dict(candidate=i+1,along_m=float(i)) for i in range(len(deltas))]
    rows=[]
    for candidate,delta in zip(candidates,deltas):
        for height in [.18,.3,.42]:
            rows.append(dict(candidate=candidate['candidate'],z_high_m=height+.03,issues=[],side_A={},side_B={},centre_delta_across_m=delta,usable_comparison=True))
    return dict(candidate_reference=candidates,sections=rows)


class LowTrunkTests(unittest.TestCase):
    def test_upper_edge_of_slice_must_remain_below_ceiling(self):
        validate_alignment_slices([.14,.18,.46],.06)
        with self.assertRaises(ValueError):validate_alignment_slices([.2,.49],.06)

    def test_constant_offset_transfers_to_held_out_trunks(self):
        result=assess_shift(sample_report([.06]*5))
        self.assertTrue(result['passes_centre_diagnostics'])
        self.assertAlmostEqual(result['proposed_side_B_dy_m'],-.06)
        self.assertAlmostEqual(result['validation']['leave_one_candidate_out_rms_m'],0.)
        self.assertFalse(result['automatic_correction_applied'])

    def test_common_shift_cannot_hide_a_trunk_that_worsens(self):
        report=sample_report([.01,.06,.06,.06,0.])
        result=assess_shift(report)
        self.assertFalse(result['passes_centre_diagnostics'])
        self.assertGreater(result['validation']['worst_held_out_absolute_worsening_m'],.01)
        # Many repeated slices at one trunk must not change its voting weight.
        report['sections'] += [report['sections'][0].copy() for _ in range(100)]
        self.assertEqual(assess_shift(report)['proposed_side_B_dy_m'],result['proposed_side_B_dy_m'])

if __name__=='__main__':unittest.main()
