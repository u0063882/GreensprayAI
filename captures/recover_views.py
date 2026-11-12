"""Recover recordings disconnected from their own side's initial pose graph.

Only same-side measured surfaces are matched. No biological shape targets,
meshing or correspondence to opposite bark hemispheres are used. Source and
target spatial validation returns are excluded before training normal/ICP work.
"""
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from captures.structural_validation import _holdout


def disconnected_views(labels,edges):
    if edges is None:return [],{}
    labels=np.asarray(labels);todo=[];trusted={}
    for label in dict.fromkeys(labels):
        ids=np.flatnonzero(labels==label).tolist();adj={i:set() for i in ids}
        for e in edges:
            i,j=int(e['i']),int(e['j'])
            if e.get('accepted') and i in adj and j in adj:adj[i].add(j);adj[j].add(i)
        unseen=set(ids);components=[]
        while unseen:
            component=set();stack=[min(unseen)]
            while stack:
                i=stack.pop()
                if i in component:continue
                component.add(i);unseen.discard(i);stack.extend(adj[i]-component)
            components.append(sorted(component))
        core=max(components,key=len)
        if len(core)<3:continue
        trusted[label]=core;todo.extend(i for i in ids if i not in core)
    return todo,trusted


def recover_views(clouds,scores,origins,labels,edges):
    import open3d as o3d
    todo,trusted=disconnected_views(labels,edges);N=len(clouds)
    corrections=np.tile(np.eye(4),(N,1,1));reports=[];reg=o3d.pipelines.registration
    def subset(i,validation=False):
        q=np.asarray(clouds[i]);h=_holdout(q,.12)
        keep=(np.asarray(scores[i])>.6)&(q[:,2]>.55)&(q[:,2]<3.6)
        return q[keep&(h if validation else ~h)]
    for i in todo:
        core=trusted[labels[i]];source=subset(i);target=np.concatenate([subset(j) for j in core])
        if min(len(source),len(target))<300:
            reports.append(dict(scan_id=int(i),accepted=False,reason='Insufficient same-side training geometry'));continue
        pcs=[]
        for q in [source,target]:
            pc=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(q)).voxel_down_sample(.04)
            pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=.12,max_nn=35));pcs.append(pc)
        pivot=np.r_[np.median(source[:,:2],axis=0),.3];trials=[]
        for yaw in [-20,-10,0,10,20]:
            for dx in [-.4,0,.4]:
                for dy in [-.25,0,.25]:
                    T=np.eye(4);T[:3,:3]=Rotation.from_euler('z',yaw,degrees=True).as_matrix();T[:3,3]=pivot-T[:3,:3]@pivot+[dx,dy,0]
                    for threshold in [.25,.13,.07]:
                        fit=reg.registration_icp(pcs[0],pcs[1],threshold,T,
                            reg.TransformationEstimationPointToPlane(reg.TukeyLoss(k=threshold*.7)),
                            reg.ICPConvergenceCriteria(max_iteration=35));T=fit.transformation
                    angles=Rotation.from_matrix(T[:3,:3]).as_euler('xyz',degrees=True)
                    delta=T[:3,:3]@pivot+T[:3,3]-pivot
                    if np.max(abs(angles[:2]))>3 or abs(angles[2])>25 or np.linalg.norm(delta)>.65:continue
                    trials.append(dict(initial=[yaw,dx,dy],fitness=fit.fitness,rmse_m=fit.inlier_rmse,
                        angles_deg=angles.tolist(),pivot_shift_m=delta.tolist(),correction=T.tolist()))
        if not trials:
            reports.append(dict(scan_id=int(i),accepted=False,reason='No plausible rigid recovery'));continue
        trials.sort(key=lambda t:t['rmse_m']+.08*(1-t['fitness']));best=trials[0];C=np.array(best['correction'])
        modes=[]
        for trial in trials:
            S=np.array(trial['correction']);angle=np.linalg.norm(Rotation.from_matrix(S[:3,:3]@C[:3,:3].T).as_rotvec())
            if angle<np.deg2rad(1) and np.linalg.norm(S[:3,3]-C[:3,3])<.06:modes.append(trial)
        vs=subset(i,True);vt=np.concatenate([subset(j,True) for j in core])
        pc=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(vt)).voxel_down_sample(.025)
        pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=.085,max_nn=30));vt=np.asarray(pc.points);normals=np.asarray(pc.normals);tree=cKDTree(vt);check={}
        for state,T in [('before',np.eye(4)),('after',C)]:
            q=vs@T[:3,:3].T+T[:3,3];distance,k=tree.query(q);matched=distance<.08
            residual=np.sum((q-vt[k])*normals[k],axis=1)
            check[state]=dict(points=len(q),coverage_8cm=float(matched.mean()),
                capped_distance_rmse_m=float(np.sqrt(np.mean(np.minimum(distance,.08)**2))),
                median_distance_m=float(np.median(distance)),
                matched_plane_rmse_m=float(np.sqrt(np.mean(residual[matched]**2))) if matched.any() else None)
        before,after=check['before'],check['after'];reasons=[]
        if len(vs)<200:reasons.append('Too few excluded validation returns')
        if after['coverage_8cm']<before['coverage_8cm']+.15:reasons.append('Coverage gain below 15 percentage points')
        if after['capped_distance_rmse_m']>before['capped_distance_rmse_m']*.9:reasons.append('Capped distance RMS improvement below 10%')
        if after['median_distance_m']>before['median_distance_m']*.8:reasons.append('Median distance improvement below 20%')
        if len(modes)<3 or len(set(t['initial'][0] for t in modes))<2:reasons.append('No stable multistart mode')
        accepted=not reasons
        if accepted:corrections[i]=C
        reports.append(dict(scan_id=int(i),trusted_target_scan_ids=core,accepted=accepted,
            rejection_reasons=reasons,best=best,converged_starts=len(modes),plausible_starts=len(trials),
            validation=check,trials=trials,
            caution='Internal same-acquisition checks, not independent survey accuracy. Validation-return sets stay fixed and are re-matched after pose changes; every unmatched query receives the distance cap.'))
    return corrections,reports
