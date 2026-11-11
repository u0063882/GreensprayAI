"""Independent final trunk/post refinement: across translation and bounded yaw.

Starts from registered measured point arrays, without importing decoding/meshing.
Only training heights 15–24 / 36–45 cm fit poses. 26–34 cm is withheld.
"""
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial import cKDTree

def detect_object(p,origin,seed,radius,zlo=.15,zhi=.45,training=True,role='trunk',x_span=None,rng=None,exclude=None,learn_radius=False,radius_sigma=.006):
 if rng is None:rng=np.random.default_rng(4)
 x_bound=x_span if x_span is not None else .14 if role=='trunk' else .28
 z=p[:,2];mask=(z>zlo)&(z<zhi)&(abs(p[:,0]-seed[0])<(.15 if role=='trunk' else .28))&(abs(p[:,1]-seed[1])<.30)
 if training:mask&=((z<.24)|(z>.36))
 else:mask&=((z>.26)&(z<.34))
 
 if exclude is not None:
  ec=np.asarray(exclude['centre'])+(p[:,2,None]-.3)*np.asarray(exclude['lean']);er=np.linalg.norm(p[:,:2]-ec,axis=1)
  mask &= er>exclude['radius']+.025
 ix=np.flatnonzero(mask);q=p[ix]
 if len(q)<20:return None
 if len(q)>350:pick=np.linspace(0,len(q)-1,350,dtype=int);q=q[pick];ix=ix[pick]
 xy=q[:,:2];centres=[]
 for _ in range(500):
  pair=xy[rng.choice(len(q),2,replace=False)];v=pair[1]-pair[0];length=np.linalg.norm(v)
  if length<.015 or length>2*radius:continue
  mid=pair.mean(axis=0);n=np.array([-v[1],v[0]])/length;h=np.sqrt(max(0,radius**2-length**2/4))
  for c in [mid+h*n,mid-h*n]:
   if abs(c[0]-seed[0])>(x_span if x_span is not None else .12 if role=='trunk' else .27) or abs(c[1]-seed[1])>.28:continue
   radial=xy-c;d=abs(np.linalg.norm(radial,axis=1)-radius);visible=np.sum(radial*(origin[:2]-xy),axis=1)>0;keep=(d<.015)&visible
   if keep.sum()<15:continue
   score=float(np.mean(np.minimum(d/.018,1)**2)+.12*np.mean(~visible))
   centres.append((score,c))
 if not centres:return None
 candidates=[]
 for _,centre in sorted(centres,key=lambda a:a[0])[:8]:
  v=np.r_[centre,0.,0.,radius,0.]
  for _ in range(3):
   c=v[:2]+(q[:,2,None]-.3)*v[2:4];radial=q[:,:2]-c;d=abs(np.linalg.norm(radial,axis=1)-(v[4]+v[5]*(q[:,2]-.3)));visible=np.sum(radial*(origin[:2]-q[:,:2]),axis=1)>0;keep=(d<.020)&visible
   if keep.sum()<15:break
   qq=q[keep]
   def res(v):
    c=v[:2]+(qq[:,2,None]-.3)*v[2:4];rr=np.linalg.norm(qq[:,:2]-c,axis=1)-(v[4]+v[5]*(qq[:,2]-.3))
    return np.r_[rr/.006, v[2:4]/.25, (v[0]-seed[0])/.012 if x_span is not None else 0., (v[4]-radius)/(radius_sigma if learn_radius else 1e-6),v[5]/(.10 if role=='trunk' else 1e-6)]
   fit=least_squares(res,v,bounds=(np.r_[seed-[x_bound,.31],-.5,-.5,max(.015,radius-2.5*radius_sigma),-.25],np.r_[seed+[x_bound,.31],.5,.5,radius+2.5*radius_sigma,.25]),loss='soft_l1',f_scale=1.,max_nfev=50);v=fit.x
  c=v[:2]+(q[:,2,None]-.3)*v[2:4];radial=q[:,:2]-c;d=abs(np.linalg.norm(radial,axis=1)-(v[4]+v[5]*(q[:,2]-.3)));visible=np.sum(radial*(origin[:2]-q[:,:2]),axis=1)>0;keep=(d<.015)&visible
  if keep.sum()<15 or keep.mean()<.42:continue
  angles=np.sort(np.mod(np.arctan2(radial[keep,1],radial[keep,0]),2*np.pi));span=2*np.pi-max(np.diff(np.r_[angles,angles[0]+2*np.pi]));rmse=np.sqrt(np.mean(d[keep]**2))
  if np.rad2deg(span)<65 or rmse>.010 or np.ptp(q[keep,2])<(.10 if training else .03):continue
  score=float(np.mean(np.minimum(d/.018,1)**2)+.12*np.mean(~visible))
  candidates.append((score,dict(centre=v[:2].tolist(),lean=v[2:4].tolist(),radius=float(v[4]),taper=float(v[5]),inliers=int(keep.sum()),points=len(q),rmse=float(rmse),arc=float(np.rad2deg(span)),indices=ix[keep].tolist())))
 return min(candidates,key=lambda x:x[0])[1] if candidates else None


def _weighted_median(values,weights):
    values=np.asarray(values);order=np.argsort(values);weights=np.asarray(weights)[order]
    return float(values[order[np.searchsorted(np.cumsum(weights),weights.sum()/2)]])


def _apply(p,T):return np.asarray(p)@T[:3,:3].T+T[:3,3]


def _correction(value,pivot):
    dy,yaw=value;T=np.eye(4);c,s=np.cos(yaw),np.sin(yaw)
    T[:3,:3]=np.array([[c,-s,0.],[s,c,0.],[0.,0.,1.]])
    pivot=np.r_[pivot,0.];T[:3,3]=pivot-T[:3,:3]@pivot+[0.,dy,0.]
    return T


def _across(centre,value,pivot):
    dy,yaw=value;cx,cy=centre;px,py=pivot
    return float(np.sin(yaw)*(cx-px)+np.cos(yaw)*(cy-py)+py+dy)


def _stats(values):
    a=np.asarray(values,dtype=float)
    if not len(a):return None
    return dict(count=len(a),median_absolute_m=float(np.median(abs(a))),rmse_m=float(np.sqrt(np.mean(a*a))),p90_absolute_m=float(np.quantile(abs(a),.9)))


def _fit_graph(observations,anchor_ids,moving_ids,pivots,settings):
    """Refine the reference side first; then freeze its learned landmarks."""
    training=[o for o in observations if o['training']];n=len(pivots)
    reference=[o for o in training if o['scan'] in anchor_ids]
    keys=sorted(set((o['stem'],o['role']) for o in reference))
    key_index={k:i for i,k in enumerate(keys)}
    anchor=max(anchor_ids,key=lambda i:sum(o['inliers'] for o in reference if o['scan']==i and o['role']=='trunk'))
    observed={int(i):[o for o in training if o['scan']==i and (o['stem'],o['role']) in key_index] for i in range(n)}
    active_anchor=[int(i) for i in anchor_ids if i!=anchor and observed[int(i)]]
    parameters=np.zeros((n,2))
    cy0=[_weighted_median([o['centre'][1] for o in reference if (o['stem'],o['role'])==key],
                         [o['inliers'] for o in reference if (o['stem'],o['role'])==key]) for key in keys]
    yaw_bound=np.deg2rad(settings['maximum_yaw_deg'])
    def limits(ids,across_bound):
        limits=[]
        for i in ids:
            x=[o['centre'][0] for o in observed[i] if o['role']=='trunk']
            # A single small object does not establish yaw: retain its baseline
            # rotation and solve only the observable across translation.
            observable=len(x)>=2 and np.ptp(x)>.65
            limits.extend([across_bound,yaw_bound if observable else 1e-10])
        return np.asarray(limits)
    def row_residual(o,value,target):
        sigma=max(.004,o['rmse']*1.5) if o['role']=='trunk' else max(.008,o['rmse']*2)
        return (_across(o['centre'],value,pivots[o['scan']])-target)/sigma
    def residual(v):
        local=np.zeros((n,2));local[active_anchor]=v[:2*len(active_anchor)].reshape(-1,2);cy=v[2*len(active_anchor):]
        data=[row_residual(o,local[o['scan']],cy[key_index[(o['stem'],o['role'])]]) for o in reference]
        return np.r_[data,(local[active_anchor]/[.035,np.deg2rad(.6)]).ravel()]
    initial=np.r_[np.zeros(2*len(active_anchor)),cy0];bound=limits(active_anchor,settings['maximum_anchor_across_m'])
    fit=least_squares(residual,initial,bounds=(np.r_[-bound,np.full(len(keys),-np.inf)],np.r_[bound,np.full(len(keys),np.inf)]),loss='soft_l1',max_nfev=120,ftol=1e-9)
    parameters[active_anchor]=fit.x[:2*len(active_anchor)].reshape(-1,2);centres=dict(zip(keys,fit.x[2*len(active_anchor):]));success=bool(fit.success)
    active_moving=[int(i) for i in moving_ids if observed[int(i)]]
    for i in active_moving:
        bound=limits([i],settings['maximum_moving_across_m'])
        def residual_moving(v):
            return np.r_[[row_residual(o,v,centres[(o['stem'],o['role'])]) for o in observed[i]],v/[.10,np.deg2rad(1.5)]]
        fit=least_squares(residual_moving,[0.,0.],bounds=(-bound,bound),loss='soft_l1',max_nfev=100,ftol=1e-9)
        parameters[i]=fit.x;success&=bool(fit.success)
    unsupported=[int(i) for i in range(n) if i!=anchor and not observed[i]]
    return parameters,centres,int(anchor),unsupported,success


def _validate(observations,parameters,pivots,anchor_ids,moving_ids):
    """Fixed held-out observations, detected before any correction is applied."""
    validation=[o for o in observations if not o['training']];keys=sorted(set((o['stem'],o['role']) for o in validation));rows=[];before=[];after=[];along_before=[];along_after=[]
    def centre(o,corrected):
        p=np.r_[o['centre'],.3][None,:]
        return _apply(p,_correction(parameters[o['scan']],pivots[o['scan']]))[0] if corrected else p[0]
    for key in keys:
        a=[o for o in validation if (o['stem'],o['role'])==key and o['scan'] in anchor_ids]
        b=[o for o in validation if (o['stem'],o['role'])==key and o['scan'] in moving_ids]
        if not a or not b:continue
        ac=np.array([centre(o,False) for o in a]);bc=np.array([centre(o,False) for o in b]);aa=np.array([centre(o,True) for o in a]);ba=np.array([centre(o,True) for o in b])
        # Include all same-object A/B pairs, preserving identities and sample
        # counts. Single-view checks remain visible but are marked weak.
        first=(bc[:,1,None]-ac[None,:,1]).ravel();last=(ba[:,1,None]-aa[None,:,1]).ravel()
        before.extend(first);after.extend(last)
        along_before.extend((bc[:,0,None]-ac[None,:,0]).ravel());along_after.extend((ba[:,0,None]-aa[None,:,0]).ravel())
        rows.append(dict(stem=key[0],role=key[1],anchor_views=len(a),moving_views=len(b),well_supported=bool(len(a)>=2 and len(b)>=2),
            before_gap_m=float(np.median(bc[:,1])-np.median(ac[:,1])),after_gap_m=float(np.median(ba[:,1])-np.median(aa[:,1])),before=_stats(first),after=_stats(last)))
    return dict(height_bands_m=[[.26,.34]],method='Independent per-recording held-out fits with training-derived object IDs and radii; exactly the same detections and cross-side pair identities before/after',before=_stats(before),after=_stats(after),along_before=_stats(along_before),along_after=_stats(along_after),objects=rows)


def _ground_check(clouds,corrections,anchor_ids,moving_ids):
    """Same measured ground samples against local reference-side terrain planes."""
    references=[]
    for corrected in [False,True]:
        parts=[]
        for i in anchor_ids:
            p=clouds[i];p=p[(p[:,2]>-.10)&(p[:,2]<.10)]
            if corrected:p=_apply(p,corrections[i])
            _,ix=np.unique(np.floor(p[:,:2]/.035).astype(int),axis=0,return_index=True);parts.append(p[ix])
        references.append(np.concatenate(parts))
    samples=[]
    for i in moving_ids:
        p=clouds[i];p=p[(p[:,2]>-.08)&(p[:,2]<.10)]
        _,ix=np.unique(np.floor(p[:,:2]/.12).astype(int),axis=0,return_index=True);ix=ix[np.linspace(0,len(ix)-1,min(len(ix),250),dtype=int)]
        samples.append((p[ix],_apply(p[ix],corrections[i])))
    raw=np.concatenate([q for q,_ in samples]);new=np.concatenate([q for _,q in samples]);values=[];valids=[]
    for ref,q in zip(references,[raw,new]):
        if len(ref)<18:return dict(before=None,after=None)
        distance,indices=cKDTree(ref[:,:2]).query(q[:,:2],k=18,workers=2);residual=np.zeros(len(q));valid=distance[:,0]<.12
        for i,neighbours in enumerate(indices):
            if not valid[i]:continue
            points=ref[neighbours];mid=np.median(points,axis=0)
            coef=np.linalg.lstsq(np.c_[points[:,:2]-mid[:2],np.ones(len(points))],points[:,2]-mid[2],rcond=None)[0]
            if np.linalg.norm(coef[:2])>.3:valid[i]=False;continue
            residual[i]=(q[i,2]-mid[2]-np.r_[q[i,:2]-mid[:2],1.]@coef)/np.sqrt(1+np.sum(coef[:2]**2))
        values.append(residual);valids.append(valid)
    common=valids[0]&valids[1]
    return dict(before=_stats(values[0][common]),after=_stats(values[1][common]),points_considered=len(raw),same_measured_points=int(common.sum()),z_coordinates_changed=False)


def refine_trunks(clouds,scanner_origins,side_labels,stem_seeds,reference,settings=None):
    """Optional final refinement, starting from registered baseline coordinates.

    One bounded rigid correction per original recording; no scaling/warping.
    References are population sizes, not exact tree identities or survey truth.
    """
    defaults=dict(maximum_anchor_across_m=.08,maximum_moving_across_m=.15,maximum_yaw_deg=3.)
    options={**defaults,**(settings or {})};labels=np.asarray(side_labels);a=np.flatnonzero(labels==labels[0]);b=np.flatnonzero(labels!=labels[0]);clouds=[np.asarray(p,dtype=float) for p in clouds];origins=np.asarray(scanner_origins)
    if len(np.unique(labels))!=2 or min(len(a),len(b))<2:raise ValueError('Need two assembled sides with at least two original recordings each')
    radii=np.asarray(reference['trunk']['circumferences_cm'])/(200*np.pi);r0=float(np.median(radii));radius_sigma=max(.004,float(np.std(radii))+.002)
    post_r=float(np.median(reference['support']['circumferences_cm']))/(200*np.pi)
    rng=np.random.default_rng(2026);first=[];models={};notes=[]
    for i in a:
        for k,seed in stem_seeds:
            fit=detect_object(clouds[i],origins[i],np.asarray(seed),r0,rng=rng,learn_radius=True,radius_sigma=radius_sigma)
            if fit:first.append(dict(scan=int(i),stem=int(k),role='trunk',training=True,**fit))
    for k,seed in stem_seeds:
        obs=[o for o in first if o['stem']==k]
        if len(obs)<2:notes.append(f'Stem window {k}: insufficient reference-side training views');continue
        # Along-row centre consensus disambiguates the nearby support without
        # changing the already registered along-row coordinates.
        x=np.array([o['centre'][0] for o in obs]);support=np.array([o['inliers'] for o in obs]);scores=[support[abs(x-v)<.035].sum() for v in x];keep=abs(x-x[np.argmax(scores)])<.035
        obs=[o for o,v in zip(obs,keep) if v]
        if len(obs)<2:continue
        weights=[o['inliers'] for o in obs]
        models[int(k)]=dict(seed=np.asarray(seed).tolist(),x=_weighted_median([o['centre'][0] for o in obs],weights),radius_m=_weighted_median([o['radius'] for o in obs],weights),radius_prior_m=r0,radius_prior_sigma_m=radius_sigma,diameter_cm=200*_weighted_median([o['radius'] for o in obs],weights))
    if len(models)<3:raise ValueError('Insufficient independently supported trunk identities for a safe refinement')
    observations=[];trunk_training={}
    for i,p in enumerate(clouds):
        for k,model in models.items():
            seed=np.array([model['x'],model['seed'][1]])
            for training in [True,False]:
                fit=detect_object(p,origins[i],seed,model['radius_m'],training=training,x_span=.04,rng=rng)
                if fit:
                    record=dict(scan=i,stem=k,role='trunk',training=training,**fit);observations.append(record)
                    if training:trunk_training[(i,k)]=record
    # Optional posts are identified in the residual points and associated by
    # their trunk-relative vector, which is invariant to across translation.
    post_candidates={};post_models={}
    for (i,k),trunk in trunk_training.items():
        fit=detect_object(clouds[i],origins[i],np.array(trunk['centre']),post_r,role='post',rng=rng,exclude=trunk)
        if fit and np.linalg.norm(np.array(fit['centre'])-trunk['centre'])>models[k]['radius_m']+post_r-.01:
            post_candidates[(i,k)]=dict(scan=i,stem=k,role='post',training=True,relative=(np.array(fit['centre'])-trunk['centre']).tolist(),**fit)
    for k in models:
        candidates=[o for (i,j),o in post_candidates.items() if j==k and i in a]
        if len(candidates)<2:continue
        rel=np.array([o['relative'] for o in candidates]);distance=np.linalg.norm(rel[:,None,:]-rel[None,:,:],axis=2);best=np.argmax((distance<.035).sum(axis=1));keep=distance[best]<.035
        if keep.sum()<2:continue
        mean=np.median(rel[keep],axis=0);posts=[o for (i,j),o in post_candidates.items() if j==k and np.linalg.norm(np.array(o['relative'])-mean)<.035]
        if not any(o['scan'] in b for o in posts):continue
        post_models[k]=dict(relative_to_trunk_m=mean.tolist(),radius_m=post_r,note='Optional recurrent support proxy; circular-equivalent population size, not material identification')
        observations.extend(posts)
        for post in posts:
            i=post['scan'];seed=np.array(post['centre'])
            fit=detect_object(clouds[i],origins[i],seed,post_r,training=False,role='post',x_span=.04,rng=rng,exclude=trunk_training[(i,k)])
            if fit:observations.append(dict(scan=i,stem=k,role='post',training=False,**fit))
    pivots=np.array([[float(np.median(p[:,0])),float(np.median([s[1][1] for s in stem_seeds]))] for p in clouds])
    parameters,centres,anchor,unsupported,success=_fit_graph(observations,a,b,pivots,options)
    corrections=np.array([_correction(v,p) for v,p in zip(parameters,pivots)])
    for k,model in models.items():
        model['centre_m']=[model['x'],float(centres[(k,'trunk')])]
        obs=[o for o in observations if o['training'] and o['role']=='trunk' and o['stem']==k and o['scan'] in a]
        model['lean_xy_per_m']=np.median([np.asarray(o['lean'])@corrections[o['scan'],:2,:2].T for o in obs],axis=0).tolist()
        model['taper_m_per_m']=float(np.median([o['taper'] for o in obs]))
    validation=_validate(observations,parameters,pivots,a,b);ground=_ground_check(clouds,corrections,a,b)
    before,after=validation['before'],validation['after'];reasons=[]
    if not success:reasons.append('Pose graph did not converge')
    if before is None or before['count']<20:reasons.append('Insufficient independent height-band correspondences')
    elif after['median_absolute_m']>min(before['median_absolute_m']*.8,before['median_absolute_m']-.002):reasons.append('Across-row held-out median does not improve by both 20% and 2 mm')
    for row in validation['objects']:
        if row['well_supported'] and abs(row['after_gap_m'])>abs(row['before_gap_m'])+.005:reasons.append(f"Held-out {row['role']} {row['stem']} worsens by more than 5 mm")
    if validation['along_before'] and validation['along_after'] and validation['along_after']['median_absolute_m']>validation['along_before']['median_absolute_m']+.005:
        reasons.append('Held-out along-row centre discrepancy worsens by more than 5 mm')
    for i,v in enumerate(parameters):
        bound=options['maximum_anchor_across_m'] if i in a else options['maximum_moving_across_m']
        if abs(v[0])>=.985*bound or abs(v[1])>=np.deg2rad(options['maximum_yaw_deg'])*.985:reasons.append('A recording reaches its correction limit: '+str(i))
    if ground['before'] and ground['after']:
        if ground['after']['median_absolute_m']>ground['before']['median_absolute_m']+.005:reasons.append('Ground median worsens by more than 5 mm')
        if ground['after']['rmse_m']>ground['before']['rmse_m']+.005:reasons.append('Ground RMS worsens by more than 5 mm')
    else:reasons.append('Insufficient held-out ground support')
    along=[]
    for p,C in zip(clouds,corrections):along.extend((_apply(p[::8],C)[:,0]-p[::8,0]).tolist())
    along=np.asarray(along)
    if np.sqrt(np.mean(along**2))>.020:reasons.append('Along-row displacement RMS exceeds 2 cm')
    np.testing.assert_allclose(corrections[:,:3,:3].transpose(0,2,1)@corrections[:,:3,:3],np.tile(np.eye(3),(len(corrections),1,1)),atol=1e-12)
    np.testing.assert_array_equal(corrections[anchor],np.eye(4))
    report=dict(method=__doc__.strip(),variant='Independent final across translation / yaw refinement; reference side assembled first, then independently corrected opposite recordings',
        reference_usage='Population size priors only; no exact workbook-tree mapping; loose netting and post shape uncertainty retained',settings=options,reference=reference,
        trunk_models={str(k):v for k,v in models.items()},support_models={str(k):v for k,v in post_models.items()},anchor_scan_id=anchor,anchor_side_scan_ids=a.tolist(),moving_side_scan_ids=b.tolist(),
        unchanged_unsupported_scan_ids=unsupported,solver_success=success,validation=validation,ground_validation=ground,
        along_row_displacement=dict(rms_m=float(np.sqrt(np.mean(along**2))),maximum_absolute_m=float(max(abs(along),default=0))),
        per_recording=[dict(scan_id=i,across_translation_m=float(v[0]),yaw_deg=float(np.rad2deg(v[1])),pivot_m=pivots[i].tolist(),correction=corrections[i].tolist(),training_trunks=sorted(set(o['stem'] for o in observations if o['scan']==i and o['training'] and o['role']=='trunk'))) for i,v in enumerate(parameters)],
        object_observations=observations,notes=notes,validation_passed=not reasons,rejection_reasons=reasons,
        accuracy_limit='Conditional agreement on withheld 26–34cm observations from the same acquisitions, under size-based object association. Representative manual references are priors, not independent surveyed tree centres. No scaling, point warping or deletion.')
    return corrections,report

