"""Test a cross-row translation using only low trunk slices, without applying it.

One median per candidate prevents overlapping slices from counting as independent
trunks. Leave-one-candidate-out evaluation checks whether a common shift transfers.
This diagnostic never changes registered points, poses, or meshes.
"""
import os
import tempfile
os.environ.setdefault('MPLCONFIGDIR',os.path.join(tempfile.gettempdir(),'greenspray-mpl'))
from pathlib import Path
import argparse
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def assess_shift(report):
    candidates=[]
    for section in report['sections']:
        if section['z_high_m'] >= .5:
            raise ValueError('Shift assessment requires every full slice below Z=0.5 m.')
    for candidate in report['candidate_reference']:
        rows=[r for r in report['sections'] if r['candidate']==candidate['candidate']]
        # Wide bootstrap intervals may inform a proposal, but cannot make it accepted.
        geometry=[]
        for r in rows:
            excluded=[issue for issue in r['issues'] if 'bootstrap' not in issue and 'across-centre interval' not in issue]
            if not excluded and r['side_A'] is not None and r['side_B'] is not None:
                geometry.append(r)
        if len(geometry)<2:continue
        deltas=np.array([r['centre_delta_across_m'] for r in geometry])
        candidates.append(dict(candidate=candidate['candidate'],along_m=candidate['along_m'],
                               median_delta_across_m=float(np.median(deltas)),
                               height_range_delta_across_m=[float(deltas.min()),float(deltas.max())],
                               proposal_sections=len(geometry),strict_usable_sections=sum(r['usable_comparison'] for r in rows)))
    result=dict(method='Exploratory pure across-row translation of Side B; along/Z/rotations held fixed. One median per candidate, equal candidate weights. Leave-one-candidate-out validation.',
                proposal_policy='Geometry gates retained. Broad acquisition-bootstrap intervals may inform the proposal but remain uncertain; acceptance also needs at least three strictly usable candidates.',
                alignment_ceiling_z_m=.5,candidates=candidates,proposed_side_B_dy_m=None,
                strict_usable_candidates=sum(c['strict_usable_sections']>0 for c in candidates),
                validation=None,passes_centre_diagnostics=False,automatic_correction_applied=False,
                limitations=['Candidate IDs are not validated tree identities. Non-circular stems and partial arcs can bias centres.',
                'Overlapping slices and shared scans are correlated. This is transfer across candidate stems, not independent survey validation.',
                'This diagnostic does not validate ground overlap or apply any transform. It cannot establish accuracy of a new registration.'])
    reasons=[]
    if len(candidates)<3:
        result['decision']='insufficient candidate stems for transfer validation'
        return result
    delta=np.array([c['median_delta_across_m'] for c in candidates]);dy=-float(np.median(delta))
    held_out=delta+np.array([-np.median(np.delete(delta,i)) for i in range(len(delta))])
    before=float(np.sqrt(np.mean(delta**2)));after=float(np.sqrt(np.mean(held_out**2)))
    for c,residual in zip(candidates,held_out):
        c['after_common_proposal_delta_m']=c['median_delta_across_m']+dy
        c['held_out_delta_m']=float(residual)
    result['proposed_side_B_dy_m']=dy
    result['validation']=dict(before_candidate_rms_m=before,leave_one_candidate_out_rms_m=after,
                              in_sample_rms_m=float(np.sqrt(np.mean((delta+dy)**2))),
                              relative_held_out_improvement=float(1-after/max(before,1e-12)),
                              worst_held_out_absolute_worsening_m=float(np.max(abs(held_out)-abs(delta))))
    if result['strict_usable_candidates']<3:reasons.append('fewer than three distinct candidates pass the strict slice gates')
    if after>=.8*before:reasons.append('held-out RMS improvement below 20%')
    if np.max(abs(held_out)-abs(delta))>.01:reasons.append('at least one held-out candidate worsens by more than 1 cm')
    if abs(dy)>.15:reasons.append('proposed translation exceeds 15 cm')
    result['rejection_reasons']=reasons
    result['passes_centre_diagnostics']=not reasons
    result['decision']='reject common shift; retain existing registration' if reasons else 'common shift passes centre checks; ground/point-cloud validation still required before use'
    return result


def write_shift_report(source,out):
    report=assess_shift(source)
    out=Path(out);out.mkdir(parents=True,exist_ok=True)
    (out/'low_trunk_shift_quality.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    fig,axes=plt.subplots(1,2,figsize=(13,5),layout='constrained')
    candidates=report['candidates']
    if report['validation'] is not None:
        x=np.array([c['along_m'] for c in candidates]);before=np.array([c['median_delta_across_m'] for c in candidates])*100
        dy=report['proposed_side_B_dy_m']*100
        axes[0].axhline(0,color='gray',lw=.8)
        axes[0].scatter(x,before,label='Original B−A centres',color='#315f99')
        axes[0].scatter(x,before+dy,label='With proposed common shift',marker='x',color='#853f9b')
        for c,y in zip(candidates,before):axes[0].annotate(str(c['candidate']),(c['along_m'],y),xytext=(4,5),textcoords='offset points')
        axes[0].set(xlabel='Along-row candidate position (m)',ylabel='Across-centre difference (cm)',title=f'Exploratory Side B shift: {dy:+.1f} cm');axes[0].legend(fontsize=9)
        positions=np.arange(len(candidates));held=np.array([c['held_out_delta_m'] for c in candidates])*100
        axes[1].bar(positions-.18,abs(before),width=.36,label='Original',color='#315f99')
        axes[1].bar(positions+.18,abs(held),width=.36,label='Leave-this-candidate-out shift',color='#853f9b')
        axes[1].set(xticks=positions,xticklabels=[str(c['candidate']) for c in candidates],xlabel='Candidate ID',ylabel='Absolute across-centre residual (cm)',title='Transfer check: one vote per candidate');axes[1].legend(fontsize=9)
    else:
        for axis in axes:axis.text(.5,.5,report['decision'],ha='center',va='center',transform=axis.transAxes)
    for axis in axes:axis.grid(alpha=.2)
    fig.suptitle('LOW TRUNKS ONLY · '+report['decision'],fontsize=12)
    fig.savefig(out/'low_trunk_shift_check.png',dpi=170);plt.close(fig)
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',type=Path,default=Path('output/trunk_slices/trunk_slice_quality.json'))
    parser.add_argument('--output-dir',type=Path,default=Path('output/trunk_slices'))
    args=parser.parse_args()
    print(json.dumps(write_shift_report(json.loads(args.input.read_text()),args.output_dir),indent=2))

if __name__=='__main__':main()
