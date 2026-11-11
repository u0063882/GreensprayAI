"""Independent, optional leaf/wood-guided rigid refinement of a delivered baseline.

No meshing imports. The assembled first side stays exactly fixed. Opposite
hemispheres are constrained to fixed stem surfaces, never nearest-neighbour ICP.
Only same-side, same-face overlaps use semantic point-to-plane correspondences.
All fitting tree points are below 0.5 m. Every recording has one rigid SE(3).
"""
import numpy as np
import open3d as o3d
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from captures.orchard import fit_stem_model,transform
from processing.register_anchored import (correction_matrix,move,stem_residual,
    training_heights,visibility_penalty,ground_reference,ground_matches,metrics)

DEFAULTS=dict(rotation_deg=[.25,.25,.6],translation_m=[.015,.025,.008],
              wood_boost=3.,class_mismatch_penalty_m=.015,maximum_pair_distance_m=.03,
              maximum_points_per_stem=220,maximum_nfev=70,use_semantics=True)


def sample_indices(points,voxel=.012,maximum=220):
    if not len(points):return np.empty(0,dtype=int)
    _,ix=np.unique(np.floor(points/voxel).astype(np.int64),axis=0,return_index=True)
    if len(ix)>maximum:ix=ix[np.linspace(0,len(ix)-1,maximum,dtype=int)]
    return ix


def spatial_holdout(points):
    cells=np.floor(points[:,:2]/.08).astype(np.int64)
    return (cells[:,0]+2*cells[:,1])%4==0


def semantic_weights(a,b,boost=3.):
    """Soft agreement, wood priority, no hard rejection of uncertain labels."""
    a=np.asarray(a);b=np.asarray(b)
    agreement=a*b+(1-a)*(1-b)
    return (.25+.75*agreement)*(1+boost*np.minimum(a,b))


def _normals(points,origin):
    pc=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
    pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=.06,max_nn=30))
    n=np.asarray(pc.normals).copy();flip=np.sum(n*(np.asarray(origin)-points),axis=1)<0;n[flip]*=-1
    return n


def make_pairs(points,scores,origins,moving_ids,settings,validation=False):
    """Fixed identities selected once; no validation returns enter training.

    Candidate neighbours are restricted by distance and oriented normal angle.
    Source spatial holdouts are excluded from BOTH ends of training pairs.
    Validation uses disjoint height bands (at both ends), geometry-only matching.
    """
    arrays={};values={};normals={};pairs=[]
    for i in moving_ids:
        p=points[i];train=training_heights(p)&~spatial_holdout(p)
        keep=(~training_heights(p) if validation else train)&(p[:,2]>.12)&(p[:,2]<.46)
        ix=np.flatnonzero(keep);ix=ix[sample_indices(p[ix],.01,1200)]
        arrays[i]=p[ix];values[i]=scores[i][ix]
        normals[i]=_normals(p[ix],origins[i]) if len(ix)>30 else np.empty((len(ix),3))
    for local,i in enumerate(moving_ids):
        for j in moving_ids[local+1:local+3]:
            p,q=arrays[i],arrays[j]
            if min(len(p),len(q))<40:continue
            distance,ix=cKDTree(q).query(p,k=min(5,len(q)),workers=2)
            if distance.ndim==1:distance=distance[:,None];ix=ix[:,None]
            dot=np.sum(normals[i][:,None,:]*normals[j][ix],axis=2)
            cost=distance**2
            if not validation and settings['use_semantics']:
                cost=cost+settings['class_mismatch_penalty_m']**2*(values[i][:,None]-values[j][ix])**2
            cost[(distance>settings['maximum_pair_distance_m'])|(dot<.6)]=np.inf
            best=np.argmin(cost,axis=1);target=ix[np.arange(len(p)),best]
            keep=np.isfinite(cost[np.arange(len(p)),best])
            if keep.sum()<20:continue
            source=np.flatnonzero(keep);source=source[np.linspace(0,len(source)-1,min(350,len(source)),dtype=int)];target=target[source]
            w=semantic_weights(values[i][source],values[j][target],settings['wood_boost']) if settings['use_semantics'] else np.ones(len(source))
            pairs.append((int(i),int(j),p[source],q[target],normals[j][target],w))
    return pairs


def register_leafwood(clouds,wood_scores,scanner_origins,side_labels,stem_seeds,settings=None,held_out_stem=-1):
    """Return per-recording corrections and an honest baseline/candidate report.

    clouds are ALREADY registered baseline coordinates in metres. Inputs are
    never mutated. Stem seeds use the same frame. This is an incremental
    alternative, not an independent SLAM/initialisation or a survey benchmark.
    """
    options={**DEFAULTS,**(settings or {})}
    labels=np.asarray(side_labels);anchor_label=labels[0];anchor=np.flatnonzero(labels==anchor_label);moving=np.flatnonzero(labels!=anchor_label)
    if len(np.unique(labels))!=2 or not len(anchor) or not len(moving):raise ValueError('Two sides are required')
    points=[np.asarray(p,dtype=float) for p in clouds];scores=[np.asarray(s) for s in wood_scores]
    if any(len(p)!=len(s) for p,s in zip(points,scores)):raise ValueError('Point/attribute count mismatch')
    fixed=np.concatenate([points[i] for i in anchor]);rng=np.random.default_rng(41);models={}
    # These fresh diagnostic/reference models never fit withheld height bands.
    for k,seed in stem_seeds:
        q=fixed[(fixed[:,2]>.12)&(fixed[:,2]<.46)&training_heights(fixed)&(abs(fixed[:,0]-seed[0])<.23)&(abs(fixed[:,1]-seed[1])<.25)]
        model=fit_stem_model(q,seed,rng)
        if model and model['usable']:models[int(k)]=model
    if len(models)<3:raise ValueError('Insufficient fixed-side low-stem support for semantic comparison')
    stem_points=[];stem_scores=[]
    for p,s in zip(points,scores):
        mask=np.zeros(len(p),dtype=bool)
        for k,model in models.items():
            if k==held_out_stem:continue
            v=model['parameters'];mask|=(abs(p[:,0]-v[0])<.26)&(abs(p[:,1]-v[1])<.30)&(abs(stem_residual(p,v))<.045)
        mask&=(p[:,2]>.12)&(p[:,2]<.46)
        stem_points.append(p[mask]);stem_scores.append(s[mask])
    pairs=make_pairs(stem_points,stem_scores,scanner_origins,moving,options)
    validation_pairs=make_pairs(stem_points,stem_scores,scanner_origins,moving,options,validation=True)
    reference=ground_reference(fixed);grounds={};stems={};pivots={};origins=np.asarray(scanner_origins)
    for i in moving:
        p=points[i];pivots[i]=np.r_[np.median(p[:,:2],axis=0),.3];groups=[]
        for k,model in models.items():
            v=model['parameters'];keep=(p[:,2]>.12)&(p[:,2]<.46)&(abs(p[:,0]-v[0])<.25)&(abs(p[:,1]-v[1])<.3)&(abs(stem_residual(p,v))<.045)
            ix=np.flatnonzero(keep);ix=ix[sample_indices(p[ix],maximum=options['maximum_points_per_stem'])]
            if len(ix)>=25:groups.append((k,p[ix],scores[i][ix]))
        stems[i]=groups
        q=p[(p[:,2]>-.08)&(p[:,2]<.10)];q=q[sample_indices(q,.07,500)]
        if reference.ndim!=2 or len(reference)<18:raise ValueError('Insufficient fixed-side ground support')
        grounds[i]=ground_matches(q,reference)
    bounds=np.r_[np.deg2rad(options['rotation_deg']),options['translation_m']]
    prior=np.r_[np.deg2rad([.12,.12,.3]),.008,.012,.004]
    parameters=np.zeros((len(points),6))
    def residual(v):
        parameters=np.zeros((len(points),6));parameters[moving]=v.reshape(-1,6);parts=[]
        for i in moving:
            value=parameters[i]
            for k,q,s in stems[i]:
                if k==held_out_stem:continue
                keep=training_heights(q)&~spatial_holdout(q);q=q[keep];s=s[keep]
                if len(q)<15:continue
                m=move(q,value,pivots[i]);w=np.sqrt(1+options['wood_boost']*s) if options['use_semantics'] else np.ones(len(q))
                parts.append(w*stem_residual(m,models[k]['parameters'])/(.010*np.sqrt(len(q))))
                origin=move(origins[i][None,:],value,pivots[i])[0]
                parts.append(visibility_penalty(m,origin,models[k]['parameters'])/(.006*np.sqrt(len(q))))
            q,c,n=grounds[i];keep=~spatial_holdout(q);q,c,n=q[keep],c[keep],n[keep]
            if len(q):parts.append(np.sum((move(q,value,pivots[i])-c)*n,axis=1)/(.006*np.sqrt(len(q))))
            parts.append(value/prior)
        for i,j,p,q,n,w in pairs:
            R=correction_matrix(parameters[j],pivots[j])[:3,:3]
            d=move(p,parameters[i],pivots[i])-move(q,parameters[j],pivots[j])
            parts.append(np.sqrt(w)*np.sum(d*(n@R.T),axis=1)/(.008*np.sqrt(len(p))))
        return np.concatenate(parts)
    fit=least_squares(residual,np.zeros(len(moving)*6),bounds=(-np.tile(bounds,len(moving)),np.tile(bounds,len(moving))),
                      loss='soft_l1',f_scale=1.,max_nfev=options['maximum_nfev'],x_scale=np.tile(bounds,len(moving)),ftol=1e-7)
    parameters[moving]=fit.x.reshape(-1,6);corrections=np.tile(np.eye(4),(len(points),1,1))
    for i in moving:corrections[i]=correction_matrix(parameters[i],pivots[i])
    np.testing.assert_array_equal(corrections[anchor],np.tile(np.eye(4),(len(anchor),1,1)))
    updated=[transform(p,T) for p,T in zip(points,corrections)]
    before_radial=[];after_radial=[];before_ground=[];after_ground=[];before_overlap=[];after_overlap=[];fitting_z=[]
    for i in moving:
        for k,q,s in stems[i]:
            hold=~training_heights(q)|spatial_holdout(q)
            if k==held_out_stem:hold[:]=True
            before_radial.extend(stem_residual(q[hold],models[k]['parameters']));after_radial.extend(stem_residual(transform(q[hold],corrections[i]),models[k]['parameters']))
            if k!=held_out_stem:fitting_z.extend(transform(q[~hold],corrections[i])[:,2])
        q,c,n=grounds[i];keep=spatial_holdout(q)
        before_ground.extend(np.sum((q[keep]-c[keep])*n[keep],axis=1));after_ground.extend(np.sum((transform(q[keep],corrections[i])-c[keep])*n[keep],axis=1))
    for i,j,p,q,n,w in validation_pairs:
        before_overlap.extend(np.sum((p-q)*n,axis=1))
        after_overlap.extend(np.sum((transform(p,corrections[i])-transform(q,corrections[j]))*(n@corrections[j,:3,:3].T),axis=1))
    checks={name:dict(baseline=metrics(before),leafwood=metrics(after)) for name,before,after in [
        ('withheld_stem_surface',before_radial,after_radial),('withheld_ground',before_ground,after_ground),('withheld_same_face_overlap',before_overlap,after_overlap)]}
    # Same low-height measured support for width checks, not a recropped cloud.
    shape=[]
    for k,model in models.items():
        widths=[]
        for clouds_ in [points,updated]:
            parts=[]
            for i,p in enumerate(points):
                keep=(p[:,2]>.20)&(p[:,2]<.40)&~training_heights(p)&(abs(stem_residual(p,model['parameters']))<.02)&(abs(p[:,0]-model['parameters'][0])<.22)&(abs(p[:,1]-model['parameters'][1])<.24)
                parts.append(clouds_[i][keep])
            q=np.concatenate(parts)
            widths.append(float(np.quantile(q[:,1],.98)-np.quantile(q[:,1],.02)) if len(q)>40 else None)
        shape.append(dict(stem=k,baseline_width_m=widths[0],leafwood_width_m=widths[1],change_m=widths[1]-widths[0] if all(x is not None for x in widths) else None,fixed_model_diameter_m=2*model['parameters'][2]))
    reasons=[]
    if not fit.success:reasons.append('Semantic solver did not converge')
    if len(pairs)<2:reasons.append('Insufficient same-face semantic overlap pairs')
    if np.max(abs(parameters[moving])/bounds)>=.98:reasons.append('A rigid correction reaches its bound')
    if not fitting_z or max(fitting_z)>=.5:reasons.append('Insufficient or out-of-range fitting heights')
    for name,check in checks.items():
        a,b=check['baseline'],check['leafwood']
        if a is None or a['points']<100:reasons.append(name+': insufficient validation support')
        elif b['median_absolute_m']>a['median_absolute_m']+.001 or b['capped_rmse_5cm_m']>a['capped_rmse_5cm_m']+.001:reasons.append(name+': worsens by more than 1 mm')
    for row in shape:
        if row['change_m'] is not None and row['change_m']<-.003:reasons.append('Low-stem measured across width shrinks by over 3 mm: '+str(row['stem']))
    return corrections,dict(method=__doc__.strip(),baseline_dependency='Incremental refinement of the delivered geometry-only baseline, not independent initialisation',
        settings=options,anchor_scan_ids=anchor.tolist(),moving_scan_ids=moving.tolist(),held_out_stem=held_out_stem,
        models={str(k):v for k,v in models.items()},checks=checks,shape_checks=shape,
        training_pair_groups=len(pairs),training_pair_points=sum(len(p) for _,_,p,_,_,_ in pairs),
        validation_pair_points=sum(len(p) for _,_,p,_,_,_ in validation_pairs),solver_success=bool(fit.success),
        maximum_transformed_fitting_z_m=float(max(fitting_z)) if fitting_z else None,
        per_recording=[dict(scan_id=int(i),parameters=parameters[i].tolist(),correction=corrections[i].tolist(),bound_fraction=float(max(abs(parameters[i])/bounds))) for i in moving],
        validation_passed=not reasons,rejection_reasons=reasons,
        accuracy_limit='Internal registration consistency on withheld samples, conditional on the baseline fixed side and stem model. Shared raw acquisitions and baseline fitting prevent an independent survey-accuracy claim. Wood classification is unlabelled.',
        selection_policy='The experiment never replaces the baseline automatically, including when these checks pass.')
