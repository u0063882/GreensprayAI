"""Local measured terrain patches: slope-aware constraints, no flattening.

A patch has a centroid and measured plane normal in its recording's current
frame. Different views need not sample the same XY locations. Pair residuals
are signed distances to each plane, rather than differences in median Z.
"""
import numpy as np
from scipy.spatial import cKDTree


def fit_plane(points, centre=None, iterations=6):
    p=np.asarray(points,dtype=float)
    if len(p)<15 or not np.isfinite(p).all():return None
    xy=p[:,:2];origin=np.median(xy,axis=0) if centre is None else np.asarray(centre)
    A=np.c_[xy-origin,np.ones(len(p))]
    if np.linalg.cond(A)>100:return None
    v=np.linalg.lstsq(A,p[:,2],rcond=None)[0]
    for _ in range(iterations):
        d=A@v-p[:,2];scale=max(.004,1.4826*np.median(abs(d-np.median(d))))
        w=1/np.sqrt(1+(d/(1.5*scale))**2)
        v=np.linalg.lstsq(A*w[:,None],p[:,2]*w,rcond=None)[0]
    d=A@v-p[:,2];keep=abs(d)<.02
    if keep.sum()<15 or keep.mean()<.6:return None
    normal=np.r_[-v[:2],1.];normal/=np.linalg.norm(normal)
    if normal[2]<.9:return None
    rmse=float(np.sqrt(np.mean((d[keep]*normal[2])**2)))
    if rmse>.012:return None
    centroid=np.median(p[keep,:2],axis=0)
    centroid=np.r_[centroid,(centroid-origin)@v[:2]+v[2]]
    return dict(centroid=centroid,normal=normal,rmse=rmse,points=int(keep.sum()),
                slope=v[:2])


def ground_patches(clouds,cell_m=.25,radius_m=.12,z_limits=(-.13,.13),maximum_per_scan=220):
    if not (0<radius_m<=cell_m/2):raise ValueError("Patch footprints must fit inside disjoint cells")
    patches={}
    for i,p in enumerate(clouds):
        p=np.asarray(p);g=p[(p[:,2]>z_limits[0])&(p[:,2]<z_limits[1])]
        if len(g)<100:continue
        _,ix=np.unique(np.floor(g/.022).astype(np.int64),axis=0,return_index=True);g=g[ix]
        tree=cKDTree(g[:,:2]);cells=np.unique(np.floor(g[:,:2]/cell_m).astype(np.int64),axis=0)
        if len(cells)>maximum_per_scan:cells=cells[np.linspace(0,len(cells)-1,maximum_per_scan,dtype=int)]
        for cell in cells:
            centre=(cell+.5)*cell_m;ix=tree.query_ball_point(centre,radius_m)
            if len(ix)<25:continue
            q=g[ix];delta=q[:,:2]-centre
            # Two-dimensional support prevents a grazing strip establishing a
            # spurious steep plane from a nearly one-dimensional sample.
            if np.linalg.eigvalsh(np.cov(delta.T)).min()<.001:continue
            fit=fit_plane(q,centre)
            if fit is not None:patches.setdefault(tuple(cell),[]).append(dict(scan=i,**fit))
    return patches


def patch_pairs(patches,holdout_modulus=4):
    train=[];held=[]
    for cell,obs in sorted(patches.items()):
        target=held if (cell[0]+2*cell[1])%holdout_modulus==0 else train
        for a in range(len(obs)):
            for b in range(a+1,len(obs)):
                p,q=obs[a],obs[b]
                target.append(dict(i=p['scan'],j=q['scan'],p=p['centroid'],q=q['centroid'],
                    ni=p['normal'],nj=q['normal'],cell=cell,
                    sigma=max(.004,np.hypot(p['rmse'],q['rmse'])),
                    multiplicity=len(obs)-1))
    return train,held


def plane_pair_residuals(p,q,ni,nj,Ri=None,Rj=None,ti=None,tj=None):
    """Symmetric point-to-plane residuals at actual measured locations."""
    Ri=np.eye(3) if Ri is None else np.asarray(Ri);Rj=np.eye(3) if Rj is None else np.asarray(Rj)
    ti=np.zeros(3) if ti is None else np.asarray(ti);tj=np.zeros(3) if tj is None else np.asarray(tj)
    p=np.asarray(p)@Ri.T+ti;q=np.asarray(q)@Rj.T+tj
    ni=np.asarray(ni)@Ri.T;nj=np.asarray(nj)@Rj.T;d=p-q
    return np.stack([np.sum(d*ni,axis=-1),np.sum(d*nj,axis=-1)],axis=-1)


def terrain_validation(clouds,corrections,original_clouds=None,cap_m=.05):
    """Re-match excluded ground returns to terrain planes at their actual XY.

    Selection is frozen in original coordinates. Targets use excluded returns
    only. A local plane may slope; an unmatched query receives the cost cap.
    """
    original_clouds=clouds if original_clouds is None else original_clouds
    ground=[];queries=[]
    for p,original,T in zip(clouds,original_clouds,corrections):
        p=np.asarray(p);original=np.asarray(original)
        cells=np.floor(original[:,:2]/.25).astype(np.int64)
        keep=((cells[:,0]+2*cells[:,1])%4==0)&(original[:,2]>-.10)&(original[:,2]<.10)
        q=p[keep]@T[:3,:3].T+T[:3,3]
        if len(q):
            _,ix=np.unique(np.floor(original[keep]/.025).astype(np.int64),axis=0,return_index=True);g=q[ix]
            # Keep the query identities in the original frame, not pose-dependent voxels.
            before=original[keep];_,ix=np.unique(np.floor(before/.10).astype(np.int64),axis=0,return_index=True)
            if len(ix)>220:ix=ix[np.linspace(0,len(ix)-1,220,dtype=int)]
            queries.append(q[ix]);ground.append(g)
        else:queries.append(np.empty((0,3)));ground.append(np.empty((0,3)))
    per_scan=[];all_cost=[];all_valid=[]
    for i,q in enumerate(queries):
        target=np.concatenate([p for j,p in enumerate(ground) if j!=i]);tree=cKDTree(target[:,:2]);cost=[];valid=[]
        for p in q:
            ids=tree.query_ball_point(p[:2],.30);fit=fit_plane(target[ids],p[:2])
            if fit is None:cost.append(cap_m);valid.append(False)
            else:cost.append(min(cap_m,abs(np.dot(p-fit['centroid'],fit['normal']))));valid.append(True)
        values=np.asarray(cost);per_scan.append(dict(scan_id=i,points=len(q),coverage=float(np.mean(valid)) if valid else 0.,
            median_absolute_m=float(np.median(values)) if len(values) else None,
            capped_rmse_m=float(np.sqrt(np.mean(values**2))) if len(values) else None))
        all_cost.extend(cost);all_valid.extend(valid)
    values=np.asarray(all_cost)
    return dict(points=len(values),coverage=float(np.mean(all_valid)) if all_valid else 0.,
        median_absolute_m=float(np.median(values)) if len(values) else None,
        capped_rmse_m=float(np.sqrt(np.mean(values**2))) if len(values) else None,
        per_scan=per_scan,method='Fixed excluded ground queries; re-matched robust local terrain planes evaluated at actual XY; no horizontal-plane target')
