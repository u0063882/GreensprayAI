"""Independent joint rigid refinement of both assembled orchard sides.

Measured terrain may slope. Shared low-trunk axes may lean. Every original
recording has one rigid pose; no tree-form symmetry, flat-soil constraint,
per-point deformation or mesh reconstruction is used.
"""
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from captures.terrain import ground_patches,patch_pairs,terrain_validation
from captures.joint_landmarks import collect_landmarks
from captures.recover_views import recover_views
from captures.structural_validation import _holdout,_sample,_normals
from captures.overlap import validation_features,rematched_overlap

DEFAULTS=dict(gravity_sigma_deg=.75,rotation_bound_deg=[3.,3.,4.],
    translation_bound_m=[.08,.20,.08],maximum_evaluations=150,
    ground_median_tolerance_m=.001,ground_rms_tolerance_m=.001,
    upper_rms_tolerance_m=.001,maximum_anchor_difference_m=.010)


def apply(points,T):return np.asarray(points)@T[:3,:3].T+T[:3,3]


def _pose_matrices(parameters,pivots):
    R=Rotation.from_rotvec(parameters[:,:3]).as_matrix()
    t=pivots-np.einsum('nij,nj->ni',R,pivots)+parameters[:,3:]
    return R,t


def _cylinder(points,model):
    axis=np.r_[model[2:4],1.];axis/=np.linalg.norm(axis)
    d=points-np.r_[model[:2],.3];along=d@axis;radial=d-along[:,None]*axis
    rho=np.linalg.norm(radial,axis=1)
    return rho-(model[4]+model[5]*along),radial,rho


def _upper_training(clouds,scores,origins,labels,originals):
    points=[];normals=[];trees=[]
    for p,s,o,original in zip(clouds,scores,origins,originals):
        keep=(original[:,2]>.55)&(original[:,2]<3.6)&(s>.65)&~_holdout(original,.12)
        q=p[keep];q=q[_sample(q,.025)]
        n=_normals(q,o,.085) if len(q)>=30 else np.zeros_like(q)
        points.append(q);normals.append(n);trees.append(cKDTree(q))
    pairs=[]
    for i,p in enumerate(points):
        if len(p)<30:continue
        for j in range(i+1,len(points)):
            q=points[j]
            if len(q)<30:continue
            cross=labels[i]!=labels[j];distance,k=trees[j].query(p)
            back=trees[i].query(q[k])[1]
            keep=(distance<(.06 if cross else .04))&(np.sum(normals[i]*normals[j][k],axis=1)>.85)&(back==np.arange(len(p)))
            ix=np.flatnonzero(keep)
            if len(ix)<(8 if cross else 30):continue
            ix=ix[np.linspace(0,len(ix)-1,min(400,len(ix)),dtype=int)]
            pairs.append((i,j,p[ix],q[k[ix]],normals[j][k[ix]],.006 if cross else .004))
    return pairs


def _terrain_training(clouds,originals):
    masks=[(np.floor(p[:,0]/.25).astype(int)+2*np.floor(p[:,1]/.25).astype(int))%4==0 for p in originals]
    patches=ground_patches([p[~mask] for p,mask in zip(clouds,masks)])
    a,b=patch_pairs(patches)
    return a+b


def _model_initial(keys,observations,radius):
    models=[]
    for key in keys:
        obs=[o for o in observations if o['training'] and (o['stem'],o['role'])==key]
        models.append(np.r_[np.median([o['centre'] for o in obs],axis=0),
            np.median([o['lean'] for o in obs],axis=0),radius,
            np.median([o['taper'] for o in obs])])
    return np.asarray(models)


def _fit_bundle(clouds,origins,ups,observations,keys,initial,reference,ground,pairs,gauge,options):
    N=len(clouds);K=len(keys);key_index={key:i for i,key in enumerate(keys)}
    pivots=np.array([np.r_[np.median(p[:,:2],axis=0),.3] for p in clouds])
    groups=[]
    for obs in observations:
        key=obs['stem'],obs['role']
        if obs['training'] and key in key_index:groups.append((obs['scan'],key_index[key],clouds[obs['scan']][obs['indices']]))
    gi=np.array([g['i'] for g in ground]);gj=np.array([g['j'] for g in ground]);gp=np.array([g['p'] for g in ground]);gq=np.array([g['q'] for g in ground]);gn=np.array([g['nj'] for g in ground]);gm=np.array([g['ni'] for g in ground]);gw=1/np.sqrt([g['multiplicity'] for g in ground]);gs=np.array([max(.006,g['sigma']) for g in ground])
    radius_prior=initial[:,4].copy();radius_sigma=np.full(K,reference['radius_sigma_m'])
    lower_model=np.c_[initial[:,:2]-.22,np.full((K,2),-1.),np.maximum(.012,radius_prior-2.5*radius_sigma),np.full(K,-.25)]
    upper_model=np.c_[initial[:,:2]+.22,np.full((K,2),1.),radius_prior+2.5*radius_sigma,np.full(K,.25)]
    bound=np.r_[np.deg2rad(options['rotation_bound_deg']),options['translation_bound_m']]
    lower=np.r_[np.tile(-bound,N),lower_model.ravel()];upper=np.r_[np.tile(bound,N),upper_model.ravel()]
    def residual(v):
        parameter=v[:N*6].reshape(N,6);models=v[N*6:].reshape(K,6);R,t=_pose_matrices(parameter,pivots);parts=[]
        for i,k,q in groups:
            p=q@R[i].T+t[i];m=models[k];d,radial,rho=_cylinder(p,m)
            view=R[i]@origins[i]+t[i]-p;view/=np.maximum(np.linalg.norm(view,axis=1)[:,None],1e-9)
            facing=np.sum(radial/np.maximum(rho[:,None],1e-9)*view,axis=1)
            parts.extend([d/(.006*np.sqrt(len(q))),np.maximum(-facing,0)*m[4]/(.006*np.sqrt(len(q)))])
        if len(ground):
            d=np.einsum('nij,nj->ni',R[gi],gp)+t[gi]-np.einsum('nij,nj->ni',R[gj],gq)-t[gj]
            ni=np.einsum('nij,nj->ni',R[gi],gm);nj=np.einsum('nij,nj->ni',R[gj],gn)
            parts.extend([gw*np.sum(d*ni,axis=1)/gs,gw*np.sum(d*nj,axis=1)/gs,((ni[:,:2]-nj[:,:2])*gw[:,None]/.04).ravel()])
        for i,j,p,q,n,sigma in pairs:
            d=p@R[i].T+t[i]-q@R[j].T-t[j]
            parts.append(np.sum(d*(n@R[j].T),axis=1)/(sigma*np.sqrt(len(p))))
        world_up=np.einsum('nij,nj->ni',R,ups)
        parts.append((world_up[:,:2]/np.deg2rad(options['gravity_sigma_deg'])).ravel())
        parts.append((parameter/np.r_[np.deg2rad([5.,5.,5.]),.10,.20,.08]).ravel())
        # Coordinate gauge only: release roll/pitch, and every other recording.
        parts.append(parameter[gauge,[2,3,4,5]]/1e-5)
        parts.extend([(models[:,4]-radius_prior)/radius_sigma,models[:,5]/.15])
        return np.concatenate(parts)
    start=np.r_[np.zeros(N*6),initial.ravel()];start=np.clip(start,lower+1e-8,upper-1e-8)
    fit=least_squares(residual,start,bounds=(lower,upper),loss='soft_l1',max_nfev=options['maximum_evaluations'],ftol=2e-7,
        x_scale=np.r_[np.tile(bound,N),np.tile([.05,.05,.2,.2,.01,.05],K)])
    parameters=fit.x[:N*6].reshape(N,6);models=fit.x[N*6:].reshape(K,6);R,t=_pose_matrices(parameters,pivots)
    C=np.tile(np.eye(4),(N,1,1));C[:,:3,:3]=R;C[:,:3,3]=t
    position=(models-lower_model)/(upper_model-lower_model)
    ambiguous=[keys[k] for k in range(K) if min(position[k,4],1-position[k,4],position[k,5],1-position[k,5])<.01]
    return C,models,parameters,dict(solver_success=bool(fit.success),evaluations=fit.nfev,cost=float(fit.cost),
        ambiguous_training_landmarks=ambiguous,maximum_motion_bound_fraction=float(np.max(abs(parameters)/bound)))


def _baseline_models(originals,origins,observations,keys,initial,reference):
    mapping={key:i for i,key in enumerate(keys)};groups=[(o['scan'],mapping[(o['stem'],o['role'])],originals[o['scan']][o['indices']]) for o in observations if o['training'] and (o['stem'],o['role']) in mapping]
    K=len(keys);lo=np.c_[initial[:,:2]-.3,np.full((K,2),-1.),np.maximum(.012,initial[:,4]-2.5*reference['radius_sigma_m']),np.full(K,-.25)]
    hi=np.c_[initial[:,:2]+.3,np.full((K,2),1.),initial[:,4]+2.5*reference['radius_sigma_m'],np.full(K,.25)]
    def objective(v):
        models=v.reshape(K,6);parts=[]
        for i,k,q in groups:
            m=models[k];d,r,rho=_cylinder(q,m);view=origins[i]-q;view/=np.maximum(np.linalg.norm(view,axis=1)[:,None],1e-9)
            parts.extend([d/(.006*np.sqrt(len(q))),np.maximum(-np.sum(r/np.maximum(rho[:,None],1e-9)*view,axis=1),0)*m[4]/(.006*np.sqrt(len(q)))])
        parts.extend([(models[:,4]-initial[:,4])/reference['radius_sigma_m'],models[:,5]/.15]);return np.concatenate(parts)
    fit=least_squares(objective,np.clip(initial.ravel(),lo.ravel()+1e-8,hi.ravel()-1e-8),bounds=(lo.ravel(),hi.ravel()),loss='soft_l1',max_nfev=120)
    return fit.x.reshape(K,6)


def _stem_validation(originals,observations,keys,before_models,after_models,corrections):
    mapping={key:i for i,key in enumerate(keys)};values={'before':[],'after':[]};objects={}
    for o in observations:
        key=o['stem'],o['role']
        if o['training'] or key not in mapping:continue
        i=o['scan'];k=mapping[key];q=originals[i][o['indices']]
        before=_cylinder(q,before_models[k])[0];after=_cylinder(apply(q,corrections[i]),after_models[k])[0]
        for state,v in [('before',before),('after',after)]:values[state].extend(v.tolist());objects.setdefault(str(key[0]),{'before':[],'after':[]})[state].extend(v.tolist())
    def stats(v):
        v=np.asarray(v);return dict(points=len(v),median_absolute_m=float(np.median(abs(v))),rmse_m=float(np.sqrt(np.mean(v*v)))) if len(v) else None
    return dict(**{s:stats(v) for s,v in values.items()},per_object={k:{s:stats(v) for s,v in a.items()} for k,a in objects.items()},
        method='Independent 26–34 cm returns, with a separately trained baseline model and candidate model; identical source-return identities. Radius priors are representative sizes, not survey truth.')


def anchor_difference(clouds,first,second):
    a=np.concatenate([apply(p[::100],C) for p,C in zip(clouds,first)]);b=np.concatenate([apply(p[::100],C) for p,C in zip(clouds,second)])
    x=b-b.mean(0);y=a-a.mean(0);u,_,vt=np.linalg.svd(x.T@y);R=vt.T@u.T
    if np.linalg.det(R)<0:vt[-1]*=-1;R=vt.T@u.T
    t=a.mean(0)-R@b.mean(0);distance=np.linalg.norm(b@R.T+t-a,axis=1)
    return dict(rmse_m=float(np.sqrt(np.mean(distance**2))),p95_m=float(np.quantile(distance,.95)),
        method='Matching measured returns after removing only the best common rigid frame difference')


def refine_joint(clouds,wood_scores,origins,ups,labels,seeds,reference,initial_edges=None,settings=None,gauge_scan=0,check_anchors=True):
    options={**DEFAULTS,**(settings or {})};originals=[np.asarray(p,dtype=float) for p in clouds];scores=[np.asarray(s) for s in wood_scores]
    origins=np.asarray(origins,dtype=float);ups=np.asarray(ups,dtype=float);labels=list(labels);N=len(originals)
    if not (len(scores)==len(origins)==len(ups)==len(labels)==N) or len(set(labels))!=2:raise ValueError('Two labelled sides and one origin, up vector and score array per recording required')
    seed,repairs=recover_views(originals,scores,origins,labels,initial_edges)
    points=[apply(p,C) for p,C in zip(originals,seed)];sensor_origins=np.array([apply(o[None,:],C)[0] for o,C in zip(origins,seed)]);sensor_ups=np.einsum('nij,nj->ni',seed[:,:3,:3],ups)
    observations,keys,initial,prior=collect_landmarks(points,sensor_origins,labels,seeds,reference,originals)
    if len(keys)<3:raise ValueError('Insufficient shared low-trunk landmarks for joint refinement')
    terrain=_terrain_training(points,originals);pairs=_upper_training(points,scores,sensor_origins,labels,originals)
    if len(terrain)<100:raise ValueError('Insufficient measured terrain overlap')
    C,models,parameters,fit=_fit_bundle(points,sensor_origins,sensor_ups,observations,keys,initial,prior,terrain,pairs,gauge_scan,options)
    excluded=fit['ambiguous_training_landmarks']
    if excluded:
        keep=[k for k in keys if k not in excluded]
        if len(keep)<3:raise ValueError('Too few reliable landmarks after training-only ambiguity rejection')
        keys=keep;initial=_model_initial(keys,observations,prior['radius_prior_m'])
        C,models,parameters,fit=_fit_bundle(points,sensor_origins,sensor_ups,observations,keys,initial,prior,terrain,pairs,gauge_scan,options)
    total=C@seed;identity=np.tile(np.eye(4),(N,1,1))
    baseline=_baseline_models(originals,origins,observations,keys,initial,prior)
    stem=_stem_validation(originals,observations,keys,baseline,models,total)
    ground=dict(before=terrain_validation(originals,identity),after=terrain_validation(originals,total))
    features=validation_features(originals,scores,origins);overlap=dict(before=rematched_overlap(features,labels,identity),after=rematched_overlap(features,labels,total))
    anchors=[]
    if check_anchors:
        alternatives=sorted(set([N-1,next(i for i,label in enumerate(labels) if label!=labels[gauge_scan])]))
        for other in alternatives:
            if other==gauge_scan:continue
            cc,_,_,ff=_fit_bundle(points,sensor_origins,sensor_ups,observations,keys,initial,prior,terrain,pairs,other,options)
            anchors.append(dict(gauge_scan_id=other,solver_success=ff['solver_success'],**anchor_difference(originals,total,cc@seed)))
    support=[]
    for i in range(N):
        landmarks=set((o['stem'],o['role']) for o in observations if o['training'] and o['scan']==i and (o['stem'],o['role']) in keys)
        overlap_points=sum(len(pair[2]) for pair in pairs if i in pair[:2])
        support.append(dict(scan_id=i,training_landmarks=len(landmarks),upper_overlap_points=overlap_points,horizontal_pose_supported=bool(landmarks or overlap_points>=30)))
    reasons=[]
    unsupported=[x['scan_id'] for x in support if not x['horizontal_pose_supported']]
    if unsupported:reasons.append('No horizontal pose support for recordings: '+str(unsupported))
    if not fit['solver_success']:reasons.append('Joint solver did not converge')
    if fit['ambiguous_training_landmarks']:reasons.append('A retained landmark remains at its radius or taper search bound')
    if fit['maximum_motion_bound_fraction']>=.985:reasons.append('A joint pose reaches its motion bound')
    if any(not r['accepted'] for r in repairs):reasons.append('A disconnected recording could not be recovered')
    a,b=ground['before'],ground['after']
    if b['coverage']<a['coverage']-.02:reasons.append('Ground validation coverage drops by over two percentage points')
    if b['median_absolute_m']>a['median_absolute_m']+options['ground_median_tolerance_m'] or b['capped_rmse_m']>a['capped_rmse_m']+options['ground_rms_tolerance_m']:reasons.append('Excluded terrain consistency worsens')
    for kind in ['same_side','cross_side']:
        a=overlap['before']['aggregate'][kind];b=overlap['after']['aggregate'][kind]
        if a is None or b is None:reasons.append(kind+': insufficient full-height validation returns')
        elif b['coverage']<a['coverage']-.01 or b['capped_distance_rmse_m']>a['capped_distance_rmse_m']+options['upper_rms_tolerance_m']:reasons.append(kind+': excluded full-height overlap worsens')
    if stem['before'] is None or stem['after'] is None or stem['before']['points']<100:reasons.append('Insufficient excluded low-stem returns')
    elif stem['after']['rmse_m']>stem['before']['rmse_m']*.90:reasons.append('Excluded low-stem RMS improvement below 10%')
    for item in anchors:
        if not item['solver_success'] or item['rmse_m']>options['maximum_anchor_difference_m']:reasons.append('Relative geometry is sensitive to the coordinate gauge')
    report=dict(method=__doc__.strip(),settings=options,recoveries=repairs,solver=fit,
        gauge_scan_id=gauge_scan,gauge_policy='Only coordinate origin and yaw are constrained at one node; both sides and all roll/pitch states remain adjustable',
        anchor_checks=anchors,per_recording_support=support,excluded_ambiguous_training_landmarks=excluded,
        retained_landmarks=[dict(stem=k[0],role=k[1],parameters=m.tolist()) for k,m in zip(keys,models)],
        baseline_landmarks=[dict(stem=k[0],role=k[1],parameters=m.tolist()) for k,m in zip(keys,baseline)],
        stem_validation=stem,ground_validation=ground,overlap_validation=overlap,
        validation_passed=not reasons,rejection_reasons=reasons,object_observations=observations,
        seed_corrections=seed.tolist(),joint_corrections=C.tolist(),total_corrections=total.tolist(),
        per_recording=[dict(scan_id=i,parameters=parameters[i].tolist()) for i in range(N)],
        accuracy_limit='Conditional internal checks on shared source acquisitions; no independent surveyed geometry. Unobserved upper geometry and biological symmetry are not verified. No raw returns are moved independently, rescaled, removed or recoloured.')
    return total,report
