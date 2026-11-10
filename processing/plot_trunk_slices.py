"""Plot independent opposing-view circle fits in thin, matched-height trunk slices.

Consumes registered PLYs only; does not modify points, poses, or meshes.
Candidate IDs come from the low-stem QA report and are not verified tree labels.
"""
import os
import tempfile
os.environ.setdefault('MPLCONFIGDIR', os.path.join(tempfile.gettempdir(), 'greenspray-mpl'))
os.environ.setdefault('OMP_NUM_THREADS', '2')
from pathlib import Path
import argparse
import csv
import hashlib
import json
import zipfile
import numpy as np
import open3d as o3d
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.lines import Line2D
try:
    from .check_cross_row import circle_fit
except ImportError:
    from check_cross_row import circle_fit

COLOURS = {'A': '#1673b1', 'B': '#e57b1f'}
MIN_POINTS = 30


def fit_side(points, acquisition, initial):
    """Expose conditioning, clutter and bounds, including failed fits."""
    if len(points) < MIN_POINTS:
        return None
    try:
        parameters, rmse, arc, condition = circle_fit(points, initial)
    except (ValueError, RuntimeError, np.linalg.LinAlgError):
        return None
    if not np.isfinite([*parameters, rmse, arc, condition]).all():
        return None
    residual = np.linalg.norm(points-parameters[:2], axis=1)-parameters[2]
    inlier = abs(residual) < .022
    boundary = bool(parameters[2] <= .013 or parameters[2] >= .179 or np.any(abs(parameters[:2]-initial[:2]) >= .149))
    reasons = []
    if arc <= 100: reasons.append('short observed arc')
    if condition >= 150: reasons.append('ill-conditioned circle')
    if rmse >= .018: reasons.append('high radial residual')
    if np.mean(inlier) < .55: reasons.append('mixed/cluttered section')
    if boundary: reasons.append('fit reaches search bound')
    if len(np.unique(acquisition)) < 2: reasons.append('fewer than two acquisitions')
    return dict(centre_m=parameters[:2].tolist(), radius_m=float(parameters[2]), radial_rmse_m=rmse,
                arc_deg=arc, condition=condition, input_points=len(points), inlier_points=int(inlier.sum()),
                inlier_fraction=float(np.mean(inlier)), acquisitions=int(len(np.unique(acquisition))),
                touches_bound=boundary, issues=reasons)


def fit_one_slice(points, acquisition, seed, height, thickness=.06, halfwidth=(.22, .26), bootstrap=45, rng=None):
    """Both sides use the same Z band and spatial window; centres are never shared."""
    rng = rng if rng is not None else np.random.default_rng(321)
    seed = np.asarray(seed, dtype=float)
    selection = ((abs(points[:, 2]-height) < thickness/2) &
                 (abs(points[:, 0]-seed[0]) < halfwidth[0]) &
                 (abs(points[:, 1]-seed[1]) < halfwidth[1]))
    xy = points[selection, :2]; ids = acquisition[selection]
    sides = {name: (xy[ids < 9], ids[ids < 9]) if name == 'A' else (xy[ids >= 9], ids[ids >= 9]) for name in ['A', 'B']}
    initial = np.array([*seed, .075])
    if len(xy) >= MIN_POINTS:
        try:
            pooled = circle_fit(xy, initial)[0]
            # Initialization only. The two fits have independent centres and radii.
            initial = pooled
        except (ValueError, RuntimeError, np.linalg.LinAlgError):
            pass
    fits = {name: fit_side(q, sid, initial) for name, (q, sid) in sides.items()}
    record = dict(height_z_m=float(height), z_low_m=float(height-thickness/2), z_high_m=float(height+thickness/2),
                  roi_centre_m=seed.tolist(), roi_halfwidth_m=list(halfwidth), side_A=fits['A'], side_B=fits['B'],
                  points_A=len(sides['A'][0]), points_B=len(sides['B'][0]),
                  centre_delta_along_m=None, centre_delta_across_m=None, delta_ci95_m=None,
                  bootstrap_successes=0, usable_comparison=False, issues=[])
    if any(fit is None for fit in fits.values()):
        record['issues'] = ['insufficient points or failed fit on at least one side']
        return record, sides
    delta = np.array(fits['B']['centre_m'])-fits['A']['centre_m']
    record['centre_delta_along_m'] = float(delta[0]); record['centre_delta_across_m'] = float(delta[1])
    estimates = []
    # Do not bootstrap a single acquisition into a falsely zero-width interval.
    if min(fit['acquisitions'] for fit in fits.values()) >= 2:
        for _ in range(bootstrap):
            centres = []
            for name in ['A', 'B']:
                q, sid = sides[name]; unique = np.unique(sid)
                picked = rng.choice(unique, len(unique), replace=True)
                sample = np.concatenate([q[sid == i] for i in picked])
                fit = fit_side(sample, np.concatenate([np.full(np.sum(sid == i), i) for i in picked]), initial)
                if fit is None: break
                centres.append(np.array(fit['centre_m']))
            if len(centres) == 2: estimates.append(centres[1]-centres[0])
    record['bootstrap_successes'] = len(estimates)
    if len(estimates) >= max(20, int(.8*bootstrap)):
        record['delta_ci95_m'] = np.percentile(estimates, [2.5, 97.5], axis=0).tolist()
    issues = [f'{name}: {reason}' for name in ['A', 'B'] for reason in fits[name]['issues']]
    if abs(fits['A']['radius_m']-fits['B']['radius_m']) >= .035: issues.append('side radii disagree by at least 3.5 cm')
    if record['delta_ci95_m'] is None: issues.append('insufficient acquisition-bootstrap support')
    elif record['delta_ci95_m'][1][1]-record['delta_ci95_m'][0][1] >= .04: issues.append('across-centre interval at least 4 cm wide')
    record['issues'] = issues; record['usable_comparison'] = not issues
    return record, sides


def section_panel(ax, record, sides, seed):
    seed = np.asarray(seed)
    for name in ['A', 'B']:
        p = (sides[name][0]-seed)*100
        ax.scatter(p[:, 0], p[:, 1], s=5, color=COLOURS[name], alpha=.42, linewidths=0, rasterized=True)
        fit = record['side_'+name]
        if fit is not None:
            theta = np.linspace(0, 2*np.pi, 200); centre = (np.array(fit['centre_m'])-seed)*100
            radius = fit['radius_m']*100
            ax.plot(centre[0]+radius*np.cos(theta), centre[1]+radius*np.sin(theta), color=COLOURS[name], lw=1.6,
                    ls='-' if record['usable_comparison'] else '--')
            ax.plot(*centre, marker='+', color=COLOURS[name], ms=11, mew=2)
    if record['side_A'] is not None and record['side_B'] is not None:
        centres = (np.array([record['side_A']['centre_m'], record['side_B']['centre_m']])-seed)*100
        ax.plot(centres[:, 0], centres[:, 1], color='#343b43', lw=1, zorder=0)
        text = f"B−A across {record['centre_delta_across_m']*100:+.1f} cm"
    else: text = 'Insufficient support for two fits'
    status = 'passes gates' if record['usable_comparison'] else 'UNCERTAIN'
    ax.set_title(f"Z={record['height_z_m']:.2f} m · {status}\n{text}", fontsize=9,
                 color='#19723e' if record['usable_comparison'] else '#994518')
    halfwidth = np.array(record['roi_halfwidth_m'])*100
    ax.set(xlim=(-halfwidth[0], halfwidth[0]), ylim=(-halfwidth[1], halfwidth[1]))
    ax.set_aspect('equal'); ax.grid(alpha=.18); ax.tick_params(labelsize=8)


def plot_centres(panels, records_by_candidate, out):
    fig, axes = plt.subplots(len(panels), 3, figsize=(14, 2.9*len(panels)), squeeze=False)
    for row, (candidate, _) in enumerate(panels):
        records = records_by_candidate[candidate['candidate']]; seed = np.array([candidate['along_m'], candidate['across_m']])
        for axis, dimension in zip(axes[row, :2], [0, 1]):
            for name in ['A', 'B']:
                x = [100*(r['side_'+name]['centre_m'][dimension]-seed[dimension]) if r['side_'+name] is not None else np.nan for r in records]
                heights = [r['height_z_m'] for r in records]
                axis.plot(x, heights, '--', lw=.8, color=COLOURS[name], alpha=.6)
                for value, r in zip(x, records):
                    if np.isfinite(value): axis.plot(value, r['height_z_m'], 'o', color=COLOURS[name], mfc=COLOURS[name] if r['usable_comparison'] else 'white', ms=5)
            axis.set_xlabel(('Along' if dimension == 0 else 'Across')+' centre from base reference (cm)', fontsize=9)
            axis.grid(alpha=.2)
        axis = axes[row, 2]; axis.axvline(0, color='gray', lw=.8)
        for r in records:
            if r['centre_delta_across_m'] is None: continue
            value = r['centre_delta_across_m']*100; height = r['height_z_m']
            if r['delta_ci95_m'] is not None:
                low, high = np.array(r['delta_ci95_m'])[:, 1]*100
                axis.plot([low, high], [height, height], color='#66727d', lw=1)
            axis.plot(value, height, 'o', color='#19723e' if r['usable_comparison'] else '#994518',
                      mfc='#19723e' if r['usable_comparison'] else 'white', ms=5)
        axis.set_xlabel('B−A across centre (cm), bootstrap 95% interval', fontsize=9); axis.grid(alpha=.2)
        axes[row, 0].set_ylabel(f"Candidate {candidate['candidate']}\nHeight Z (m)", fontsize=10)
        for axis in axes[row]: axis.set_ylim(records[0]['z_low_m']-.04, records[-1]['z_high_m']+.04)
    handles = [Line2D([0],[0],color=COLOURS[n],marker='o',label='Side '+n) for n in ['A','B']]
    handles.append(Line2D([0],[0],color='gray',marker='o',mfc='white',label='Uncertain comparison'))
    fig.legend(handles=handles, loc='upper center', ncol=3, bbox_to_anchor=(.5,.982))
    fig.suptitle('Centre position versus matched height · lean is visible as horizontal drift', fontsize=15, y=.999)
    fig.tight_layout(rect=(0,0,1,.967)); fig.savefig(out/'trunk_slice_centres.png', dpi=160); plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scan-dir', type=Path, required=True)
    parser.add_argument('--candidate-report', type=Path, default=Path('results/cross_row_quality.json'))
    parser.add_argument('--output-dir', type=Path, default=Path('output/trunk_slices'))
    parser.add_argument('--heights', type=float, nargs='+', default=[.18,.35,.55,.75,1.,1.25])
    parser.add_argument('--thickness', type=float, default=.06)
    parser.add_argument('--bootstrap', type=int, default=45)
    args = parser.parse_args()
    heights = np.array(sorted(set(args.heights)))
    if not np.isfinite(heights).all() or len(heights) < 2 or not np.isfinite(args.thickness) or args.thickness <= 0 or args.bootstrap < 25:
        parser.error('Need at least two finite heights, positive slice thickness, and at least 25 bootstrap replicates.')
    files = sorted(args.scan_dir.glob('scan_*_registered.ply'))
    if len(files) != 18 or not all(('sideA' if i < 9 else 'sideB') in p.name for i,p in enumerate(files)):
        parser.error('Expected 18 registered scan PLYs, nine Side A followed by nine Side B.')
    candidates = json.loads(args.candidate_report.read_text())['candidates']
    if not candidates: parser.error('Candidate report contains no candidate sections.')
    out = args.output_dir; out.mkdir(parents=True, exist_ok=True)
    parts=[]; sources=[]; manifest=[]
    for i, path in enumerate(files):
        p = np.asarray(o3d.io.read_point_cloud(str(path)).points)
        if len(p)==0 or not np.isfinite(p).all(): parser.error('Input scan empty or nonfinite: '+path.name)
        selection=(p[:,2] > heights[0]-args.thickness/2)&(p[:,2] < heights[-1]+args.thickness/2)
        parts.append(p[selection]);sources.append(np.full(selection.sum(),i))
        manifest.append(dict(file=path.name, sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
    points=np.concatenate(parts); acquisition=np.concatenate(sources)
    records=[];panels=[];by_candidate={};rng=np.random.default_rng(321)
    for candidate in candidates:
        seed=np.array([candidate['along_m'],candidate['across_m']]); cells=[];by_candidate[candidate['candidate']]=[]
        for height in heights:
            record, sides=fit_one_slice(points,acquisition,seed,height,args.thickness,bootstrap=args.bootstrap,rng=rng)
            record['candidate']=candidate['candidate'];records.append(record);by_candidate[candidate['candidate']].append(record);cells.append((record,sides))
        panels.append((candidate,cells))
        print('Fitted candidate',candidate['candidate'],'usable slices',sum(r['usable_comparison'] for r in by_candidate[candidate['candidate']]),flush=True)
    report=dict(coordinates='Input registered frame: X along row, Y across row, Z up; metres',
                height_definition='Absolute Z in the shared frame, approximately above ground; not separately normalized per side or to local terrain.',
                heights_z_m=heights.tolist(),slice_thickness_m=args.thickness,bootstrap_replicates=args.bootstrap,
                bootstrap_seed=321,input_scans=manifest,candidate_report_sha256=hashlib.sha256(args.candidate_report.read_bytes()).hexdigest(),
                candidate_reference=[{k:c[k] for k in ['candidate','along_m','across_m']} for c in candidates],
                usable_comparisons=sum(r['usable_comparison'] for r in records),total_sections=len(records),
                usable_candidates=sorted(set(r['candidate'] for r in records if r['usable_comparison'])),
                method='Separate robust XY circle fits in identical thin Z slices; acquisition-block bootstrap of B-minus-A centres. Independent centres/radii at every height; no vertical-cylinder or common-centre constraint.',
                gates=dict(min_points_per_side=MIN_POINTS,min_acquisitions_per_side=2,min_arc_deg=100,max_condition=150,max_radial_rmse_m=.018,min_inlier_fraction=.55,inlier_distance_m=.022,max_radius_disagreement_m=.035,max_across_bootstrap_width_m=.04),
                limitations=['Candidate IDs are low-stem clusters, not confirmed tree labels; weeds, stakes, branches and mixed stems can be included.',
                'Horizontal sections of leaning/non-circular trunks can be elliptical; a circular model and different visible arcs can bias the centres.',
                'Fixed candidate windows are not a validated trunk tracker. Strong lean leaving the window or bifurcation can invalidate a fit.',
                'Finite slice thickness still mixes a small amount of lean/taper. Bootstrap intervals quantify acquisition sensitivity, not survey accuracy.',
                'A filled marker passes the documented diagnostic gates; it does not prove the circular model represents a single trunk.'],
                automatic_correction_applied=False,sections=records)
    (out/'trunk_slice_quality.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    flat=[]
    for r in records:
        row={k:r[k] for k in ['candidate','height_z_m','z_low_m','z_high_m','points_A','points_B','centre_delta_along_m','centre_delta_across_m','usable_comparison']}
        row['issues']='; '.join(r['issues'])
        for name in ['A','B']:
            f=r['side_'+name]
            for key in ['radius_m','radial_rmse_m','arc_deg','condition','inlier_fraction','acquisitions']:
                row[name+'_'+key]=f[key] if f else None
            row[name+'_centre_along_m']=f['centre_m'][0] if f else None;row[name+'_centre_across_m']=f['centre_m'][1] if f else None
        for dim,name in enumerate(['along','across']):
            for i,end in enumerate(['low','high']):row[name+'_ci95_'+end+'_m']=r['delta_ci95_m'][i][dim] if r['delta_ci95_m'] is not None else None
        flat.append(row)
    with (out/'trunk_slice_centres.csv').open('w') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(flat[0]));writer.writeheader();writer.writerows(flat)
    fig,axes=plt.subplots(len(panels),len(heights),figsize=(3.3*len(heights),3.4*len(panels)),squeeze=False)
    for row,(candidate,cells) in enumerate(panels):
        seed=[candidate['along_m'],candidate['across_m']]
        for col,(record,sides) in enumerate(cells):
            section_panel(axes[row,col],record,sides,seed)
            if col==0:axes[row,col].set_ylabel(f"Candidate {candidate['candidate']} · base X={seed[0]:.2f} m\nAcross from base reference (cm)",fontsize=9)
            if row==len(panels)-1:axes[row,col].set_xlabel('Along from base reference (cm)',fontsize=9)
    fig.suptitle(f'Opposing trunk slices · {args.thickness*100:g} cm thick · blue A / orange B · + fitted centres\nDashed circles: uncertain; black segments connect centres, not bark surfaces. Fixed reference across heights.',fontsize=15)
    fig.tight_layout(rect=(0,0,1,.965));fig.savefig(out/'trunk_slice_fits.png',dpi=170);plt.close(fig)
    with PdfPages(out/'trunk_slice_fits.pdf') as pdf:
        for candidate,cells in panels:
            ncols=min(3,len(heights));nrows=int(np.ceil(len(heights)/ncols))
            fig,axes=plt.subplots(nrows,ncols,figsize=(12,4*nrows),squeeze=False)
            for ax,(record,sides) in zip(axes.flat,cells):
                section_panel(ax,record,sides,[candidate['along_m'],candidate['across_m']]);ax.set(xlabel='Along from base reference (cm)',ylabel='Across from base reference (cm)')
            for ax in list(axes.flat)[len(cells):]:ax.axis('off')
            fig.suptitle(f"Candidate {candidate['candidate']} · base X={candidate['along_m']:.2f}, Y={candidate['across_m']:.2f} m\nBlue: Side A; orange: Side B; +: estimated centre; dashed: uncertain",fontsize=13)
            fig.tight_layout(rect=(0,0,1,.94));fig.savefig(out/f"candidate_{candidate['candidate']:02d}_slices.png",dpi=170);pdf.savefig(fig);plt.close(fig)
    plot_centres(panels,by_candidate,out)
    (out/'README.txt').write_text('TRUNK CROSS-SECTIONS AT MATCHED HEIGHTS\n\nBlue=Side A, orange=Side B; crosses=circle centres; connecting lines show B-minus-A displacement.\nCircles are fitted independently at every height. Dashed circles/hollow centre markers are uncertain.\nHeights are absolute Z in the common registered frame; 6 cm slices by default.\nXY plots are centimetres from each candidate\'s FIXED low-stem reference, so lean remains visible.\nSee the per-candidate PNGs or seven-page PDF for detail, and centre-versus-height PNG for trends.\nJSON/CSV record absolute coordinates, radius, quality gates and acquisition-bootstrap intervals.\nNo registration or point-cloud coordinates were changed. Candidates are not confirmed tree identities.\n')
    names=['trunk_slice_fits.png','trunk_slice_fits.pdf','trunk_slice_centres.png','trunk_slice_quality.json','trunk_slice_centres.csv','README.txt']+[f"candidate_{c['candidate']:02d}_slices.png" for c in candidates]
    archive=out.parent/'trunk_slice_diagnostics.zip'
    with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED) as z:
        for name in names:z.write(out/name,name)
    print(json.dumps(dict(sections=len(records),usable=report['usable_comparisons'],archive=str(archive))),flush=True)

if __name__=='__main__':main()
