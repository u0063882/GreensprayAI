"""Bounded per-recording west registration against a fixed assembled east side.

Uses low stems and ground only. Starts from registered per-recording clouds; no
mesh reconstruction is imported or required. One rigid pose per acquisition.
"""
import os
import tempfile
os.environ.setdefault('OMP_NUM_THREADS','2');os.environ.setdefault('OPENBLAS_NUM_THREADS','2')
os.environ.setdefault('MPLCONFIGDIR',os.path.join(tempfile.gettempdir(),'greenspray-mpl'))
from pathlib import Path
import argparse
import hashlib
import json
import numpy as np
import open3d as o3d
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
try:
    from .check_cross_row import circle_fit
except ImportError:
    from check_cross_row import circle_fit

BOUNDS=np.r_[np.deg2rad([.5,.5,2.]),.04,.15,.025]
PRIOR_SIGMA=np.r_[np.deg2rad([.2,.2,.6]),.015,.08,.01]
HELD_OUT_CANDIDATE=5


def correction_matrix(parameters,pivot):
    """A rigid correction in the common frame, rotating about a local row pivot."""
    R=Rotation.from_rotvec(np.asarray(parameters)[:3]).as_matrix()
    T=np.eye(4);T[:3,:3]=R;T[:3,3]=np.asarray(pivot)-R@pivot+np.asarray(parameters)[3:]
    return T


def move(points,parameters,pivot):
    T=correction_matrix(parameters,pivot)
    return points@T[:3,:3].T+T[:3,3]


def relative_correction_residual(a,b,pivot_a,pivot_b,separation=1):
    """Soft relative-pose prior; a common world transform has zero residual."""
    A=correction_matrix(a,pivot_a);B=correction_matrix(b,pivot_b)
    midpoint=(np.asarray(pivot_a)+pivot_b)/2
    rotation=Rotation.from_matrix(B[:3,:3].T@A[:3,:3]).as_rotvec()
    translation=(A[:3,:3]@midpoint+A[:3,3])-(B[:3,:3]@midpoint+B[:3,3])
    sigma=np.r_[np.deg2rad([.3,.3,.5]),.02,.03,.015]*np.sqrt(separation)
    return np.r_[rotation,translation]/sigma


def stem_residual(points,model):
    """Distance to a fixed, leaning, linearly tapered circular-axis stem model."""
    v=np.asarray(model);base=np.r_[v[:2],.3];axis=np.r_[v[3:5],1.];axis/=np.linalg.norm(axis)
    d=np.asarray(points)-base;t=d@axis
    return np.linalg.norm(d-t[:,None]*axis,axis=1)-(v[2]+v[5]*t)


def visibility_geometry(points,origin,model):
    v=np.asarray(model);base=np.r_[v[:2],.3];axis=np.r_[v[3:5],1.];axis/=np.linalg.norm(axis)
    d=np.asarray(points)-base;t=d@axis;radial=d-t[:,None]*axis
    rho=np.maximum(np.linalg.norm(radial,axis=1),1e-9);radius=np.maximum(v[2]+v[5]*t,.01)
    normal=radial/rho[:,None]-v[5]*axis
    normal/=np.maximum(np.linalg.norm(normal,axis=1)[:,None],1e-9)
    view=np.asarray(origin)-points;view/=np.maximum(np.linalg.norm(view,axis=1)[:,None],1e-9)
    cosine=np.sum(normal*view,axis=1)
    # Distant clutter is not reliable evidence for an opaque stem's visibility.
    support=np.exp(-((rho-radius)/.05)**2)
    return cosine,radius,support


def visibility_penalty(points,origin,model):
    """An opaque bark surface must face its supplying scanner, not the far side."""
    cosine,radius,support=visibility_geometry(points,origin,model)
    return np.maximum(-cosine,0)*radius*support


def sample(points,voxel,maximum=None):
    if len(points)==0:return points
    _,ix=np.unique(np.floor(points/voxel).astype(np.int64),axis=0,return_index=True)
    if maximum and len(ix)>maximum:ix=ix[np.linspace(0,len(ix)-1,maximum,dtype=int)]
    return points[ix]


def fit_reference(points,seed):
    points=sample(points,.008)
    if len(points)<150:return None
    initial=circle_fit(points[:,:2],[*seed,.075])[0];v0=np.r_[initial,0.,0.,0.]
    lower=np.r_[np.asarray(seed)-.2,.02,-.5,-.5,-.25]
    upper=np.r_[np.asarray(seed)+.2,.18,.5,.5,.25]
    fit=least_squares(lambda v:stem_residual(points,v),v0,bounds=(lower,upper),loss='soft_l1',f_scale=.005,max_nfev=250)
    inlier=abs(stem_residual(points,fit.x))<.018
    if inlier.sum()<100:return None
    fit=least_squares(lambda v:stem_residual(points[inlier],v),fit.x,bounds=(lower,upper),loss='soft_l1',f_scale=.004,max_nfev=150)
    e=stem_residual(points,fit.x);inlier=abs(e)<.018
    at_bound=bool(np.any((fit.x-lower)/(upper-lower)<.01) or np.any((upper-fit.x)/(upper-lower)<.01))
    good=bool(inlier.mean()>=.65 and np.sqrt(np.mean(e[inlier]**2))<.012 and not at_bound)
    return dict(parameters=fit.x.tolist(),points=len(points),inlier_fraction=float(inlier.mean()),
                radial_rmse_m=float(np.sqrt(np.mean(e[inlier]**2))),at_bound=at_bound,usable=good)


def ground_reference(east):
    p=east[(east[:,2]>-.12)&(east[:,2]<.12)]
    # Robust per-cell heights retain local terrain instead of flattening it.
    ij=np.floor(p[:,:2]/.04).astype(int);_,inv=np.unique(ij,axis=0,return_inverse=True)
    order=np.argsort(inv);groups=np.split(order,np.where(np.diff(inv[order]))[0]+1)
    centres=[]
    for indices in groups:
        if len(indices)<5:continue
        q=p[indices];z=np.quantile(q[:,2],.3);q=q[abs(q[:,2]-z)<.02]
        if len(q)>=3:centres.append(np.median(q,axis=0))
    return np.asarray(centres)


def ground_matches(points,reference):
    """Local terrain planes fitted only from fixed east ground."""
    distance,indices=cKDTree(reference[:,:2]).query(points[:,:2],k=18,workers=2)
    valid=distance[:,0]<.10;points=points[valid];indices=indices[valid]
    centres=[];normals=[];kept=[]
    for i,ix in enumerate(indices):
        q=reference[ix];centre=np.median(q,axis=0)
        coefficients=np.linalg.lstsq(np.c_[q[:,:2]-centre[:2],np.ones(len(q))],q[:,2]-centre[2],rcond=None)[0]
        if np.linalg.norm(coefficients[:2])>.25:continue
        normal=np.r_[-coefficients[:2],1.];normal/=np.linalg.norm(normal)
        centre[2]+=coefficients[2]
        centres.append(centre);normals.append(normal);kept.append(points[i])
    return np.asarray(kept),np.asarray(centres),np.asarray(normals)


def training_heights(points):
    # Unseen, disjoint low-height intervals remain available for validation.
    z=points[:,2]
    return ((z>=.12)&(z<.20))|((z>=.26)&(z<.34))|((z>=.40)&(z<.46))


def metrics(values):
    a=np.asarray(values)
    if len(a)==0:return None
    return dict(points=len(a),median_absolute_m=float(np.median(abs(a))),rmse_m=float(np.sqrt(np.mean(a*a))),
                capped_rmse_5cm_m=float(np.sqrt(np.mean(np.minimum(abs(a),.05)**2))))


def west_overlap_pairs(stems,parameters,pivots,max_distance=.035,subset='all'):
    """Same-face west overlap constraints; omit held-out stem and height bands."""
    clouds=[];normals=[]
    for groups in stems:
        arrays=[q[training_heights(q)] for k,q in groups.items() if k!=HELD_OUT_CANDIDATE]
        q=np.concatenate(arrays) if arrays else np.empty((0,3))
        pc=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(q))
        pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=.08,max_nn=30))
        clouds.append(q);normals.append(np.asarray(pc.normals))
    pairs=[]
    for i in range(9):
        for j in range(i+1,min(i+3,9)):
            if len(clouds[i])<30 or len(clouds[j])<30:continue
            a=move(clouds[i],parameters[i],pivots[i]);b=move(clouds[j],parameters[j],pivots[j])
            distance,ix=cKDTree(b).query(a,workers=2)
            Rj=correction_matrix(parameters[j],pivots[j])[:3,:3]
            n=normals[j][ix]@Rj.T
            keep=(distance<max_distance)&(abs(n[:,2])<.65)
            selected=np.flatnonzero(keep)
            if len(selected)<30:continue
            selected=selected[np.linspace(0,len(selected)-1,min(180,len(selected)),dtype=int)]
            q=clouds[i][selected];target=clouds[j][ix[selected]];n=normals[j][ix[selected]]
            cells=np.floor(q[:,:2]/.08).astype(int);hold=(cells[:,0]+2*cells[:,1])%4==0
            if subset!='all':
                keep=hold if subset=='validation' else ~hold
                q,target,n=q[keep],target[keep],n[keep]
            if len(q)<8:continue
            pairs.append((i,j,q,target,n))
    return pairs


def solve_west(stems,grounds,pivots,models,origins,verbose=True):
    """Greedy insertion followed by bounded joint refinement; east is immutable."""
    parameters=np.zeros((9,6));seen=[]
    trusted_pairs=west_overlap_pairs(stems,parameters,pivots,max_distance=.02,subset='training')
    ground_train=[]
    for q,c,n in grounds:
        cells=np.floor(q[:,:2]/.12).astype(int);hold=(cells[:,0]+2*cells[:,1])%4==0
        ground_train.append((q[~hold],c[~hold],n[~hold]))
    def data_residual(i,v):
        pieces=[]
        for candidate,q in stems[i].items():
            if candidate==HELD_OUT_CANDIDATE:continue
            q=q[training_heights(q)]
            if len(q)<25:continue
            transformed=move(q,v,pivots[i])
            d=stem_residual(transformed,models[candidate]['parameters'])
            pieces.append(d/(.010*np.sqrt(len(q))))
            origin=move(np.asarray(origins[i])[None,:],v,pivots[i])[0]
            pieces.append(visibility_penalty(transformed,origin,models[candidate]['parameters'])/(.006*np.sqrt(len(q))))
        q,c,n=ground_train[i]
        if len(q):pieces.append(np.sum((move(q,v,pivots[i])-c)*n,axis=1)/(.006*np.sqrt(len(q))))
        pieces.append(v/PRIOR_SIGMA)
        return np.concatenate(pieces)
    # Information-rich recordings enter first; neighbour priors are soft, not locks.
    order=sorted(range(9),key=lambda i:sum(len(q) for k,q in stems[i].items() if k!=HELD_OUT_CANDIDATE),reverse=True)
    for i in order:
        neighbours=[j for j in seen if abs(i-j)<=2]
        def residual(v):
            parts=[data_residual(i,v)]
            for j in neighbours:parts.append(relative_correction_residual(v,parameters[j],pivots[i],pivots[j],abs(i-j)))
            return np.concatenate(parts)
        fit=least_squares(residual,parameters[i],bounds=(-BOUNDS,BOUNDS),loss='soft_l1',f_scale=1.,max_nfev=100,x_scale=BOUNDS)
        parameters[i]=fit.x;seen.append(i)
        if verbose:print('Inserted west scan',i+9,'dy cm',round(fit.x[4]*100,2),'cost',round(fit.cost,3),flush=True)
    for _ in range(3):
        pairs=west_overlap_pairs(stems,parameters,pivots,subset='training')
        def joint(v):
            v=v.reshape(9,6);parts=[data_residual(i,v[i]) for i in range(9)]
            for i in range(9):
                for j in range(i+1,min(i+3,9)):
                    parts.append(relative_correction_residual(v[i],v[j],pivots[i],pivots[j],j-i))
            for i,j,a,b,n in pairs:
                Rj=correction_matrix(v[j],pivots[j])[:3,:3]
                d=move(a,v[i],pivots[i])-move(b,v[j],pivots[j])
                parts.append(np.sum(d*(n@Rj.T),axis=1)/(.012*np.sqrt(len(a))))
            # Keep already close same-face measurements coherent as other poses move.
            for i,j,a,b,n in trusted_pairs:
                Rj=correction_matrix(v[j],pivots[j])[:3,:3]
                d=move(a,v[i],pivots[i])-move(b,v[j],pivots[j])
                parts.append(np.sum(d*(n@Rj.T),axis=1)/(.006*np.sqrt(len(a))))
            return np.concatenate(parts)
        fit=least_squares(joint,parameters.ravel(),bounds=(-np.tile(BOUNDS,9),np.tile(BOUNDS,9)),loss='soft_l1',f_scale=1.,max_nfev=100,x_scale=np.tile(BOUNDS,9),ftol=1e-7)
        parameters=fit.x.reshape(9,6)
    return parameters,order,bool(fit.success)


def check_west_overlap(stems,parameters,pivots):
    pairs=west_overlap_pairs(stems,np.zeros((9,6)),pivots,subset='validation')
    before=[];after=[];details=[]
    for i,j,a,b,n in pairs:
        first=np.sum((a-b)*n,axis=1)
        Rj=correction_matrix(parameters[j],pivots[j])[:3,:3]
        last=np.sum((move(a,parameters[i],pivots[i])-move(b,parameters[j],pivots[j]))*(n@Rj.T),axis=1)
        before.extend(first);after.extend(last)
        details.append(dict(scan_ids=[i+9,j+9],before=metrics(first),after=metrics(last)))
    return dict(before=metrics(before),after=metrics(after),pairs=details,
                method='Baseline same-face west point/plane matches, spatial checkerboard matches excluded from pose residuals. Conditional on local normal estimates and correspondence assumptions; not independent survey accuracy.')


def acceptance(report):
    reasons=[]
    if not report['solver_success']:reasons.append('solver did not converge')
    if max(r['bound_fraction_max'] for r in report['per_recording'])>=.95:reasons.append('a recording approaches its correction bound')
    if report['maximum_transformed_fitting_z_m']>=.5:reasons.append('a fitted stem sample exceeds the low-height ceiling')
    for name in ['held_out_height_validation','held_out_stem_validation']:
        section=report[name]
        if section['before'] is None or section['after'] is None or section['after']['points']<100:reasons.append(name+': insufficient samples')
        elif section['after']['median_absolute_m']>.8*section['before']['median_absolute_m']:reasons.append(name+': median improvement below 20%')
    ground=report['held_out_ground_validation']
    if ground['after'] is None or ground['after']['median_absolute_m']>ground['before']['median_absolute_m']+.002 or ground['after']['capped_rmse_5cm_m']>ground['before']['capped_rmse_5cm_m']+.002:reasons.append('held-out ground agreement worsens beyond 2 mm tolerance')
    overlap=report['held_out_west_overlap_validation']
    if overlap['after'] is None or overlap['after']['points']<100:reasons.append('insufficient same-side validation matches')
    elif overlap['after']['rmse_m']>overlap['before']['rmse_m']+.002:reasons.append('held-out same-side point/plane RMS worsens by more than 2 mm')
    for pair in overlap['pairs']:
        if pair['after']['rmse_m']>pair['before']['rmse_m']+.01:reasons.append('same-side pair '+str(pair['scan_ids'])+' worsens by over 1 cm RMS')
    for model in report['per_stem']:
        if model['after'] is not None and model['after']['median_absolute_m']>model['before']['median_absolute_m']+.005:reasons.append('stem '+str(model['candidate'])+' median residual worsens by over 5 mm')
        if model['scanner_back_facing_fraction_after']>model['scanner_back_facing_fraction_before']+.10:reasons.append('stem '+str(model['candidate'])+' scanner-facing consistency worsens by over 10 percentage points')
    return not reasons,reasons


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scan-dir',type=Path,required=True)
    parser.add_argument('--candidate-report',type=Path,default=Path('presets/orchard_stem_candidates.json'))
    parser.add_argument('--output-dir',type=Path,default=Path('output/anchored_registration'))
    parser.add_argument('--transforms',type=Path,default=Path('results/scan_transforms.json'))
    parser.add_argument('--export',action='store_true',help='Export candidate cloud ZIPs only when the internal acceptance checks pass.')
    args=parser.parse_args();out=args.output_dir;out.mkdir(parents=True,exist_ok=True)
    if args.scan_dir.resolve()==(out/'registered_scans').resolve():parser.error('Choose an output directory distinct from the input scans.')
    files=sorted(args.scan_dir.glob('scan_*_registered.ply'))
    if len(files)!=18 or not all(('sideA' if i<9 else 'sideB') in p.name for i,p in enumerate(files)):parser.error('Expected 18 registered scans, east/A then west/B.')
    clouds=[np.asarray(o3d.io.read_point_cloud(str(path)).points) for path in files]
    if any(len(p)==0 or not np.isfinite(p).all() for p in clouds):parser.error('All scan clouds must be nonempty and finite.')
    transform_metadata=json.loads(args.transforms.read_text())
    coordinates=transform_metadata['scans']
    input_manifest=json.loads((args.scan_dir/'scan_manifest.json').read_text())
    if len(coordinates)!=18 or len(input_manifest['scans'])!=18 or any(not np.allclose(r['sensor_to_merged'],m['sensor_to_merged'],atol=1e-7) for r,m in zip(coordinates,input_manifest['scans'])):parser.error('Supply --transforms matching the input scan manifest.')
    if np.median([r['gps_longitude'] for r in coordinates[:9]])<=np.median([r['gps_longitude'] for r in coordinates[9:]]):parser.error('RTK metadata does not identify Side A as east for this capture.')
    candidates=json.loads(args.candidate_report.read_text())['candidates']
    east=np.concatenate(clouds[:9]);low=east[(east[:,2]>.12)&(east[:,2]<.48)];models={};reference_models=[]
    for c in candidates:
        seed=np.array([c['along_m'],c['across_m']]);q=low[(abs(low[:,0]-seed[0])<.23)&(abs(low[:,1]-seed[1])<.26)]
        m=fit_reference(q,seed)
        if m is None:continue
        m.update(candidate=c['candidate'],seed=seed.tolist());reference_models.append(m)
        if m['usable']:models[c['candidate']]=m
    print('Usable fixed east stem models',list(models),flush=True)
    if len(models)<3:raise RuntimeError('Not enough usable east reference stems for bounded registration.')
    reference=ground_reference(east);stems=[];grounds=[];pivots=[]
    for scan in clouds[9:]:
        groups={}
        for candidate,model in models.items():
            seed=np.array(model['seed']);q=scan[(scan[:,2]>.12)&(scan[:,2]<.46)&(abs(scan[:,0]-seed[0])<.23)&(abs(scan[:,1]-seed[1])<.30)]
            q=sample(q,.01,350)
            if len(q)>=35:groups[candidate]=q
        stems.append(groups)
        ground=sample(scan[(scan[:,2]>-.12)&(scan[:,2]<.12)],.10,500)
        grounds.append(ground_matches(ground,reference))
        pivots.append(np.array([np.median(scan[:,0]),1.9875,.3]))
    origins=[np.asarray(row['sensor_to_merged'])[:3,3] for row in coordinates[9:]]
    parameters,order,success=solve_west(stems,grounds,pivots,models,origins=origins)
    corrections=np.tile(np.eye(4),(18,1,1))
    for i in range(9):corrections[i+9]=correction_matrix(parameters[i],pivots[i])
    np.save(out/'corrections.npy',corrections)
    by_model=[];height_before=[];height_after=[];held_before=[];held_after=[];ground_before=[];ground_after=[]
    for candidate,model in models.items():
        before=[];after=[];visibility_before=[];visibility_after=[]
        for i,groups in enumerate(stems):
            if candidate not in groups:continue
            q=groups[candidate];a=stem_residual(q,model['parameters']);b=stem_residual(move(q,parameters[i],pivots[i]),model['parameters'])
            visibility_before.extend(visibility_geometry(q,origins[i],model['parameters'])[0])
            visibility_after.extend(visibility_geometry(move(q,parameters[i],pivots[i]),move(origins[i][None,:],parameters[i],pivots[i])[0],model['parameters'])[0])
            if candidate==HELD_OUT_CANDIDATE:held_before.extend(a);held_after.extend(b)
            else:
                m=~training_heights(q);height_before.extend(a[m]);height_after.extend(b[m])
            before.extend(a);after.extend(b)
        by_model.append(dict(candidate=candidate,held_out_from_alignment=candidate==HELD_OUT_CANDIDATE,before=metrics(before),after=metrics(after),
                             scanner_back_facing_fraction_before=float(np.mean(np.array(visibility_before)<-.05)),
                             scanner_back_facing_fraction_after=float(np.mean(np.array(visibility_after)<-.05))))
    for i,(q,c,n) in enumerate(grounds):
        cells=np.floor(q[:,:2]/.12).astype(int);hold=(cells[:,0]+2*cells[:,1])%4==0
        ground_before.extend(np.sum((q[hold]-c[hold])*n[hold],axis=1));ground_after.extend(np.sum((move(q[hold],parameters[i],pivots[i])-c[hold])*n[hold],axis=1))
    max_z=max(float(move(q[training_heights(q)],parameters[i],pivots[i])[:,2].max()) for i,groups in enumerate(stems) for k,q in groups.items() if k!=HELD_OUT_CANDIDATE and len(q[training_heights(q)]))
    report=dict(method=__doc__,anchor='East = Side A, confirmed by RTK longitude; all nine east corrections are exact identity',
                reference_assumption='Uses the delivered, within-side-registered east cloud as a fixed anchor. This does not establish independent survey accuracy of the anchor.',
                visibility_constraint='Stem surface normals must face the actual supplying scanner; normal/view cosine penalties prevent matching the wrong bark hemisphere. Not a full ray-tracing or occlusion model.',
                maximum_alignment_input_z_m=.48,held_out_candidate=HELD_OUT_CANDIDATE,held_out_height_bands_m=[[.20,.26],[.34,.40]],
                maximum_transformed_fitting_z_m=max_z,
                bounds=dict(roll_pitch_deg=.5,yaw_deg=2.,along_m=.04,across_m=.15,height_m=.025,rotation_pivot='Per-recording local row pivot, not coordinate origin'),
                solver_success=success,insertion_order_scan_ids=[i+9 for i in order],reference_models=reference_models,
                per_recording=[dict(scan_id=i+9,stem_models=sorted(stems[i]),parameters=parameters[i].tolist(),pivot_m=pivots[i].tolist(),correction=corrections[i+9].tolist(),bound_fraction_max=float(np.max(abs(parameters[i])/BOUNDS))) for i in range(9)],
                held_out_height_validation=dict(before=metrics(height_before),after=metrics(height_after)),
                held_out_stem_validation=dict(before=metrics(held_before),after=metrics(held_after)),
                held_out_ground_validation=dict(before=metrics(ground_before),after=metrics(ground_after)),per_stem=by_model,
                held_out_west_overlap_validation=check_west_overlap(stems,parameters,pivots),
                validation_caveat='Held-out features/heights share acquisitions and may be correlated. East geometry and circular tapered-stem assumptions are not independent survey truth.',
                input_transforms_sha256=hashlib.sha256(args.transforms.read_bytes()).hexdigest(),
                input_candidate_windows_sha256=hashlib.sha256(args.candidate_report.read_bytes()).hexdigest(),
                input_scans=[dict(file=p.name,sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in files],candidate_exported=False)
    report['validation_passed'],report['rejection_reasons']=acceptance(report)
    (out/'anchored_registration_quality.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:report[k] for k in ['solver_success','validation_passed','rejection_reasons','held_out_height_validation','held_out_stem_validation','held_out_ground_validation','held_out_west_overlap_validation','per_stem']},indent=2),flush=True)
    fig,axes=plt.subplots(1,2,figsize=(13,5),layout='constrained')
    x=np.arange(9);axes[0].plot(x+9,parameters[:,4]*100,'o-',label='Across correction');axes[0].plot(x+9,parameters[:,3]*100,'o-',label='Along correction');axes[0].set(xlabel='West recording ID',ylabel='Local pivot translation (cm)',title='Each west recording retains its own pose');axes[0].legend()
    names=[m['candidate'] for m in by_model];before=[m['before']['median_absolute_m']*100 for m in by_model];after=[m['after']['median_absolute_m']*100 for m in by_model]
    x=np.arange(len(names));axes[1].bar(x-.18,before,.36,label='Before');axes[1].bar(x+.18,after,.36,label='Per-recording fit');axes[1].set(xticks=x,xticklabels=[str(n)+(' (held out)' if n==HELD_OUT_CANDIDATE else '') for n in names],xlabel='Fixed east stem model',ylabel='Median absolute radial residual (cm)',title='Low stems; opposite faces retain the stem radius');axes[1].legend()
    for axis in axes:axis.grid(alpha=.2)
    fig.savefig(out/'anchored_registration_comparison.png',dpi=170);plt.close(fig)
    # Independent circle display uses the same fixed windows before and after.
    try:
        from .plot_trunk_slices import fit_one_slice,section_panel
    except ImportError:
        from plot_trunk_slices import fit_one_slice,section_panel
    seed_record=next(c for c in candidates if c['candidate']==4)
    seed=np.array([seed_record['along_m'],seed_record['across_m']]);display=[];display_ids=[]
    for i,p in enumerate(clouds):
        mask=(abs(p[:,0]-seed[0])<.3)&(abs(p[:,1]-seed[1])<.4)&(p[:,2]>.1)&(p[:,2]<.46)
        display.append(p[mask]);display_ids.append(np.full(mask.sum(),i))
    original=np.concatenate(display);source=np.concatenate(display_ids)
    moved=np.concatenate([q@C[:3,:3].T+C[:3,3] for q,C in zip(display,corrections)])
    fig,axes=plt.subplots(2,3,figsize=(12,8))
    for row,p in enumerate([original,moved]):
        for col,height in enumerate([.18,.26,.34]):
            record,sides=fit_one_slice(p,source,seed,height,bootstrap=45)
            section_panel(axes[row,col],record,sides,seed)
            if col==0:axes[row,col].set_ylabel(('Before' if row==0 else 'East anchored')+'\nAcross from fixed reference (cm)')
            if row==1:axes[row,col].set_xlabel('Along from fixed reference (cm)')
    fig.suptitle('Candidate near X=3.01 m · independent circle checks\nBlue=east, orange=west; crosses=centres; dashed=uncertain',fontsize=13)
    fig.tight_layout(rect=(0,0,1,.94));fig.savefig(out/'anchored_trunk_sections.png',dpi=170);plt.close(fig)
    if args.export:
        if not report['validation_passed']:raise RuntimeError('Candidate failed internal acceptance checks; see the comparison report. No cloud exports written.')
        try:
            from .export_anchored import export_candidate
        except ImportError:
            from export_anchored import export_candidate
        report['candidate_exported']=True
        (out/'anchored_registration_quality.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
        counts=export_candidate(args.scan_dir,out,corrections,args.transforms,out/'anchored_registration_quality.json')
        print('Exported candidate',counts,flush=True)

if __name__=='__main__':main()
