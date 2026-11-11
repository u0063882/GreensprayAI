"""Registration for isolated, named orchard captures; meshing is independent."""
import os
os.environ.setdefault('OMP_NUM_THREADS','2')
os.environ.setdefault('OPENBLAS_NUM_THREADS','2')
os.environ.setdefault('MPLCONFIGDIR','/tmp/greenspray-capture-mpl')
from pathlib import Path
import hashlib,json,subprocess,sys,zipfile
import numpy as np
import open3d as o3d
from scipy.optimize import least_squares,lsq_linear
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from scipy.sparse import coo_matrix,csr_matrix,vstack
from processing.register_anchored import (correction_matrix,move,stem_residual,sample,
    ground_reference,ground_matches,visibility_penalty,visibility_geometry,
    training_heights,metrics,relative_correction_residual)


def transform(p,T):return p@T[:3,:3].T+T[:3,3]

def sha256(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()

def save_json(path,value):Path(path).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')

def gps_frame(scans):
    """A/B are arbitrary labels. Infer inward headings from their RTK tracks."""
    first=scans[0];radius=6378137.
    en=np.array([[(s['lon']-first['lon'])*np.pi/180*radius*np.cos(np.deg2rad(first['lat'])),
                  (s['lat']-first['lat'])*np.pi/180*radius] for s in scans])
    labels=np.array([s['tag'].split('_')[0] for s in scans]);sides=list(dict.fromkeys(labels))
    if len(sides)!=2:raise ValueError('Exactly two labelled sides are required')
    a=np.flatnonzero(labels==sides[0]);b=np.flatnonzero(labels==sides[1])
    row=en[a[-1]]-en[a[0]]
    if np.linalg.norm(row)<1:raise ValueError('First-side RTK track is too short to establish row direction')
    row/=np.linalg.norm(row);left=np.array([-row[1],row[0]]);basis=np.array([row,left]);gps=en@basis.T
    separation=np.median(gps[b,1])-np.median(gps[a,1])
    if abs(separation)<1:raise ValueError('RTK does not separate the two sides by at least 1 metre')
    sign=np.sign(separation);yaw=np.where(labels==sides[0],sign*np.pi/2,-sign*np.pi/2)
    return gps,yaw,a,b,dict(origin_latitude=first['lat'],origin_longitude=first['lon'],
        horizontal_EN_to_row=basis.tolist(),label_A=sides[0],label_B=sides[1],signed_track_separation_m=float(separation),
        heading_method='Sensor forward faces toward the other RTK track; side names do not imply east or west.')


def initial_registration(raw,scans,config):
    gps,yaws,a,b,frame=gps_frame(scans);poses=[];clouds=[];reg=o3d.pipelines.registration
    for i,p in enumerate(raw):
        g=p[(p[:,2]<-1.25)&(p[:,0]>.7)&(p[:,0]<2.7)]
        if len(g)<300:raise RuntimeError('Insufficient ground to initialize sensor height')
        T=np.eye(4);T[:3,:3]=Rotation.from_euler('z',yaws[i]).as_matrix();T[:2,3]=gps[i];T[2,3]=-np.median(g[:,2]);poses.append(T)
        q=p[(p[:,0]>.7)&(p[:,0]<2.8)&(p[:,2]>-1.35)]
        pc=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(q)).voxel_down_sample(.035)
        pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=.14,max_nn=35));clouds.append(pc)
    poses=np.array(poses);edges=[];records=[]
    for ids in [a,b]:
        for local,i in enumerate(ids):
            for j in ids[local+1:local+4]:
                T0=np.linalg.inv(poses[j])@poses[i];T=T0.copy()
                for threshold in [.25,.13,.07]:
                    result=reg.registration_icp(clouds[i],clouds[j],threshold,T,
                        reg.TransformationEstimationPointToPlane(reg.TukeyLoss(k=threshold*.7)),
                        reg.ICPConvergenceCriteria(max_iteration=35));T=result.transformation
                delta=np.linalg.inv(T0)@T;dr=np.linalg.norm(Rotation.from_matrix(delta[:3,:3]).as_rotvec());dt=np.linalg.norm(delta[:3,3])
                accepted=bool(result.fitness>.18 and dt<.45 and dr<.12)
                records.append(dict(i=int(i),j=int(j),fitness=result.fitness,rmse_m=result.inlier_rmse,accepted=accepted))
                if accepted:edges.append((i,j,T))
                print('Within-side edge',i,j,'accepted',accepted,flush=True)
    n=len(raw)
    def matrix(v):
        T=np.eye(4);T[:3,:3]=Rotation.from_rotvec(v[:3]).as_matrix();T[:3,3]=v[3:];return T
    v0=np.array([np.r_[Rotation.from_matrix(T[:3,:3]).as_rotvec(),T[:3,3]] for T in poses])
    def residual(v):
        Ts=[matrix(row) for row in v.reshape(n,6)];pieces=[]
        for i,j,T in edges:
            E=np.linalg.inv(T)@np.linalg.inv(Ts[j])@Ts[i]
            pieces.extend(Rotation.from_matrix(E[:3,:3]).as_rotvec()/.014);pieces.extend(E[:3,3]/.035)
        for i in range(n):
            E=np.linalg.inv(poses[i])@Ts[i]
            pieces.extend(Rotation.from_matrix(E[:3,:3]).as_rotvec()/np.array([.025,.025,.09]));pieces.extend(E[:3,3]/np.array([.2,.2,.08]))
        return pieces
    fit=least_squares(residual,v0.ravel(),loss='soft_l1',max_nfev=60,ftol=1e-7)
    if not fit.success:raise RuntimeError('Initial within-side graph did not converge')
    return np.array([matrix(v) for v in fit.x.reshape(n,6)]),dict(frame=frame,edges=records,solver_success=bool(fit.success))


def row_mask(p,config,ground=False):
    c=config['crop'];centre=c['centre_intercept_m']+c['centre_slope']*p[:,0]
    zlo,zhi=(-.23,.23) if ground else c['height_m']
    return (p[:,0]>c['along_m'][0])&(p[:,0]<c['along_m'][1])&(abs(p[:,1]-centre)<c['half_width_m'])&(p[:,2]>zlo)&(p[:,2]<zhi)


def coarse_stems(raw,poses,a,b,config):
    """SE(2) seed only; final moving recordings retain independent SE(3) poses."""
    sides=[]
    for ids in [a,b]:
        parts=[]
        for i in ids:
            q=transform(raw[i],poses[i]);parts.append(q[row_mask(q,config)&(q[:,2]>.13)&(q[:,2]<.38)])
        sides.append(sample(np.concatenate(parts),.025))
    xa,xb=sides;tree=cKDTree(xa[:,:2]);candidates=[]
    for dx in np.arange(-.8,1.21,.2):
        for dy in [-.5,-.25,0.]:
            T=np.eye(4);T[:2,3]=[dx,dy]
            for threshold in [.25,.13,.07]:
                for _ in range(35):
                    q=transform(xb,T);distance,ix=tree.query(q[:,:2],workers=2);keep=distance<threshold
                    if keep.sum()<50:break
                    source=q[keep,:2];target=xa[ix[keep],:2];w=1/(1+(distance[keep]/.045)**2);w/=w.sum()
                    cs=np.sum(source*w[:,None],axis=0);ct=np.sum(target*w[:,None],axis=0)
                    u,_,vt=np.linalg.svd((source-cs).T@((target-ct)*w[:,None]));R=vt.T@u.T
                    if np.linalg.det(R)<0:vt[-1]*=-1;R=vt.T@u.T
                    E=np.eye(4);E[:2,:2]=R;E[:2,3]=ct-R@cs;T=E@T
                    if np.linalg.norm(E-np.eye(4))<1e-5:break
            q=transform(xb,T);d=tree.query(q[:,:2],workers=2)[0];back=cKDTree(q[:,:2]).query(xa[:,:2],workers=2)[0]
            score=float((np.mean(np.minimum(d,.12)**2)+np.mean(np.minimum(back,.12)**2))/2)
            candidates.append(dict(score=score,transform=T.tolist()))
    candidates.sort(key=lambda r:r['score']);T=np.array(candidates[0]['transform']);poses=poses.copy();poses[b]=T@poses[b]
    return poses,dict(method='Multistart low-return SE(2) initializer, not the final registration; physical trunk surfaces are used in the bounded refinement.',candidates=candidates)


def refine_ground(raw,poses,config):
    """Relative rigid roll/pitch/height, preserving local terrain; hold out cells."""
    o3d.utility.random.seed(7);n=len(raw);reference=np.array([3.,config['crop']['centre_intercept_m'],0.]);patches={};ground=[]
    for i,(p,T) in enumerate(zip(raw,poses)):
        q=transform(p,T);g=q[row_mask(q,config,ground=True)]
        pc=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(g)).voxel_down_sample(.025)
        plane,_=pc.segment_plane(distance_threshold=.023,ransac_n=3,num_iterations=1000)
        if plane[2]<0:plane=-plane
        if plane[2]<.9:raise RuntimeError('Ground plane is not compatible with IMU up')
        g=g[abs(g@plane[:3]+plane[3])<.027];ground.append(g)
        cells=np.floor(g[:,:2]/.15).astype(int);_,inv=np.unique(cells,axis=0,return_inverse=True)
        order=np.argsort(inv);groups=np.split(order,np.where(np.diff(inv[order]))[0]+1)
        for ids in groups:
            if len(ids)<8:continue
            mid=np.median(g[ids],axis=0);cell=tuple(np.floor(mid[:2]/.15).astype(int));patches.setdefault(cell,[]).append((i,mid))
    rows=[];cols=[];values=[];targets=[];weights=[];train=[];held=[]
    for cell,observations in patches.items():
        for ia,(i,p) in enumerate(observations):
            for j,q in observations[ia+1:]:
                if (cell[0]+2*cell[1])%4==0:held.append((i,j,p,q));continue
                u=p-reference;v=q-reference;row=len(targets)
                for column,value in [(3*i,u[1]),(3*i+1,-u[0]),(3*i+2,1),(3*j,-v[1]),(3*j+1,v[0]),(3*j+2,-1)]:rows.append(row);cols.append(column);values.append(value)
                targets.append(q[2]-p[2]);weights.append(1/np.sqrt(max(1,len(observations)-1)));train.append((i,j,p,q))
    if len(held)<100:raise RuntimeError('Insufficient overlapping held-out ground patches')
    A=coo_matrix((values,(rows,cols)),shape=(len(targets),3*n)).tocsr();b=np.array(targets);w=np.array(weights)
    P=np.diag(np.tile([.04,.04,.008],n));G=np.zeros((3,3*n))
    for i in range(n):G[:,3*i:3*i+3]=np.diag([300.,300.,100.])/n
    prior=csr_matrix(np.vstack([P,G]));AA=vstack([A.multiply(w[:,None]),prior]).tocsr();bb=np.r_[b*w,np.zeros(prior.shape[0])];x=np.zeros(3*n)
    lower=np.tile([-.035,-.035,-.18],n);upper=-lower;success=False
    for iteration in range(120):
        residual=AA@x-bb;ww=1/np.sqrt(1+(residual/.0075)**2)
        H=(AA.T@AA.multiply(ww[:,None])).toarray()+np.eye(3*n)*1e-10;rhs=AA.T@(ww*bb);L=np.linalg.cholesky(H)
        fit=lsq_linear(L.T,np.linalg.solve(L,rhs),bounds=(lower,upper),tol=1e-12,max_iter=200)
        change=np.max(abs(fit.x-x));x=fit.x
        if change<1e-8:success=True;break
    corrections=[]
    for v in x.reshape(n,3):corrections.append(correction_matrix([v[0],v[1],0,0,0,v[2]],reference))
    def check(records):
        before=[];after=[]
        for i,j,p,q in records:
            before.append(p[2]-q[2]);after.append(transform(p[None,:],corrections[i])[0,2]-transform(q[None,:],corrections[j])[0,2])
        return dict(before=metrics(before),after=metrics(after))
    report=dict(method='Relative rigid ground patches, not ground flattening; every fourth 15cm cell excluded from fitting.',training=check(train),held_out_ground_patches=check(held),solver_success=success,corrections=np.array(corrections).tolist())
    if not success or report['held_out_ground_patches']['after']['rmse_m']>=report['held_out_ground_patches']['before']['rmse_m']:raise RuntimeError('Ground refinement failed held-out validation')
    return np.array(corrections)@poses,report


def fit_stem_model(points,seed,rng):
    """Robust partial-bark cylinder, with explicit boundary and support checks."""
    q=sample(points,.009);seed=np.array(seed);starts=[]
    if len(q)<150:return None
    for _ in range(1200):
        p=q[rng.choice(len(q),3,replace=False)];A=2*(p[1:,:2]-p[0,:2]);b=np.sum(p[1:,:2]**2,axis=1)-np.sum(p[0,:2]**2)
        try:centre=np.linalg.solve(A,b)
        except np.linalg.LinAlgError:continue
        radius=np.linalg.norm(p[0,:2]-centre)
        if not(.025<radius<.16) or np.linalg.norm(centre-seed)>.18:continue
        v=np.r_[centre,radius,0.,0.,0.];d=abs(stem_residual(q,v))
        if (d<.015).sum()>=50:starts.append((np.mean(np.minimum(d/.015,1)**2),v))
    if not starts:return None
    lower=np.r_[seed-.2,.02,-.6,-.6,-.2];upper=np.r_[seed+.2,.18,.6,.6,.2];results=[]
    for _,v in sorted(starts,key=lambda a:a[0])[:8]:
        for _ in range(3):
            keep=abs(stem_residual(q,v))<.022
            fit=least_squares(lambda a:stem_residual(q[keep],a),v,bounds=(lower,upper),loss='soft_l1',f_scale=.006,max_nfev=150);v=fit.x
        d=abs(stem_residual(q,v));keep=d<.018;results.append((np.mean(np.minimum(d/.018,1)**2),v,keep,d))
    _,v,keep,d=min(results,key=lambda a:a[0]);bound=bool(np.any((v-lower)/(upper-lower)<.01) or np.any((upper-v)/(upper-lower)<.01))
    good=bool(not bound and keep.mean()>=.35 and np.ptp(q[keep,2])>.25 and keep.sum()>=100)
    return dict(parameters=v.tolist(),seed=seed.tolist(),points=len(q),inlier_fraction=float(keep.mean()),radial_inlier_rmse_m=float(np.sqrt(np.mean(d[keep]**2))),at_bound=bound,usable=good)


def refine_moving(raw,poses,a,b,config):
    from .bounded import solve_moving,west_overlap_pairs
    clouds=[transform(p,T) for p,T in zip(raw,poses)];fixed=np.concatenate([clouds[i] for i in a]);low=fixed[(fixed[:,2]>.12)&(fixed[:,2]<.48)]
    rng=np.random.default_rng(42);models={};all_models=[]
    for record in config['stem_candidates']:
        k=record['id'];seed=np.array(record['seed_m']);q=low[(abs(low[:,0]-seed[0])<.22)&(abs(low[:,1]-seed[1])<.23)];model=fit_stem_model(q,seed,rng)
        if model is None:continue
        model['candidate']=k;all_models.append(model)
        if model['usable']:models[k]=model
    held=config.get('held_out_stem',-1)
    if len([k for k in models if k!=held])<3 or (held>=0 and held not in models):raise RuntimeError('Insufficient usable stem support: '+str(all_models))
    print('Usable fixed-side models',list(models),'held-out',held,flush=True)
    reference=ground_reference(fixed[row_mask(fixed,config)]);stems=[];grounds=[];pivots=[];origins=[]
    for index in b:
        p=clouds[index];groups={}
        for k,model in models.items():
            centre=np.array(model['parameters'][:2]);q=p[(p[:,2]>.12)&(p[:,2]<.46)&(abs(p[:,0]-centre[0])<.26)&(abs(p[:,1]-centre[1])<.34)]
            if k!=held:q=q[abs(stem_residual(q,model['parameters']))<config.get('stem_capture_radius_m',.15)]
            q=sample(q,.01,350)
            if len(q)>=35:groups[k]=q
        stems.append(groups);ground=sample(p[row_mask(p,config)&(p[:,2]>-.12)&(p[:,2]<.12)],.10,500);grounds.append(ground_matches(ground,reference))
        x=float(np.median(p[row_mask(p,config),0]));pivots.append(np.array([x,config['crop']['centre_intercept_m']+config['crop']['centre_slope']*x,.3]));origins.append(poses[index,:3,3])
    bounds=np.r_[np.deg2rad(config['bounds']['rotation_deg']),config['bounds']['translation_m']];prior=np.r_[np.deg2rad([.2,.2,.8]),.04,.08,.01]
    parameters,order,success=solve_moving(stems,grounds,pivots,models,origins,bounds,prior,held)
    corrections=np.tile(np.eye(4),(len(raw),1,1))
    for i,index in enumerate(b):corrections[index]=correction_matrix(parameters[i],pivots[i])
    hb=[];ha=[];sb=[];sa=[];gb=[];ga=[];ob=[];oa=[];fitting_z=[];per_model=[]
    for k,model in models.items():
        before=[];after=[]
        for i,groups in enumerate(stems):
            if k not in groups:continue
            q=groups[k];moved=move(q,parameters[i],pivots[i]);first=stem_residual(q,model['parameters']);last=stem_residual(moved,model['parameters']);before.extend(first);after.extend(last)
            if k==held:sb.extend(first);sa.extend(last)
            else:
                mask=~training_heights(q);hb.extend(first[mask]);ha.extend(last[mask]);fitting_z.extend(moved[~mask,2])
        per_model.append(dict(candidate=k,held_out=k==held,before=metrics(before),after=metrics(after)))
    for i,(q,c,normal) in enumerate(grounds):
        cells=np.floor(q[:,:2]/.12).astype(int);keep=(cells[:,0]+2*cells[:,1])%4==0
        gb.extend(np.sum((q[keep]-c[keep])*normal[keep],axis=1));ga.extend(np.sum((move(q[keep],parameters[i],pivots[i])-c[keep])*normal[keep],axis=1))
    for i,j,p,q,normal in west_overlap_pairs(stems,np.zeros((len(b),6)),pivots,subset='validation',held_out_candidate=held):
        ob.extend(np.sum((p-q)*normal,axis=1));R=correction_matrix(parameters[j],pivots[j])[:3,:3];oa.extend(np.sum((move(p,parameters[i],pivots[i])-move(q,parameters[j],pivots[j]))*(normal@R.T),axis=1))
    report=dict(method='Fixed assembled reference side; sequential insertion and bounded joint SE(3) per moving recording. Low stem radial, scanner-facing, ground and same-face overlap constraints.',anchor_scan_ids=a.tolist(),moving_scan_ids=b.tolist(),reference_models=all_models,per_stem=per_model,held_out_stem=held if held>=0 else None,stem_holdout_note='No entire-stem holdout when it leaves end recordings unsupported; disjoint height bands and spatial patches remain withheld.' if held<0 else 'Entire stem excluded from bounded pose residuals',
        held_out_height_validation=dict(before=metrics(hb),after=metrics(ha)),held_out_stem_validation=dict(before=metrics(sb),after=metrics(sa)),held_out_ground_validation=dict(before=metrics(gb),after=metrics(ga)),held_out_overlap_validation=dict(before=metrics(ob),after=metrics(oa)),
        maximum_transformed_fitting_z_m=float(max(fitting_z)),bounds=config['bounds'],solver_success=bool(success),
        per_recording=[dict(scan_id=int(index),parameters=parameters[i].tolist(),pivot_m=pivots[i].tolist(),bound_fraction=float(max(abs(parameters[i])/bounds)),correction=corrections[index].tolist(),stem_models=sorted(stems[i])) for i,index in enumerate(b)],
        validation_caveat='Conditional checks on the fixed-side geometry, coarse initialization, circular stem model and shared acquisitions; not independent survey accuracy.')
    reasons=[]
    if not success:reasons.append('Bounded solver did not converge')
    if max(row['bound_fraction'] for row in report['per_recording'])>=.98:reasons.append('A moving recording reaches its correction bound')
    if report['maximum_transformed_fitting_z_m']>=.5:reasons.append('A fitting point exceeds 0.5 m')
    checks=['held_out_height_validation','held_out_ground_validation','held_out_overlap_validation']
    if held>=0:checks.append('held_out_stem_validation')
    for key in checks:
        r=report[key]
        if r['after'] is None or r['after']['points']<100:reasons.append(key+': insufficient held-out points')
        elif r['after']['median_absolute_m']>r['before']['median_absolute_m']+.002:reasons.append(key+': median worsens by over 2 mm')
        elif r['after']['capped_rmse_5cm_m']>r['before']['capped_rmse_5cm_m']+.002:reasons.append(key+': capped RMS worsens by over 2 mm')
    report.update(validation_passed=not reasons,rejection_reasons=reasons)
    return corrections@poses,report
