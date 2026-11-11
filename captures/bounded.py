"""Variable-count bounded side insertion using the validated orchard residuals."""
import numpy as np
import open3d as o3d
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from processing.register_anchored import (move,correction_matrix,training_heights,stem_residual,
    visibility_penalty,relative_correction_residual)

def west_overlap_pairs(stems,parameters,pivots,max_distance=.035,subset='all',held_out_candidate=5):
    """Same-face moving-side overlap constraints; omit held-out stem and height bands."""
    clouds=[];normals=[]
    for groups in stems:
        arrays=[q[training_heights(q)] for k,q in groups.items() if k!=held_out_candidate]
        q=np.concatenate(arrays) if arrays else np.empty((0,3))
        pc=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(q))
        if len(q):pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=.08,max_nn=30))
        clouds.append(q);normals.append(np.asarray(pc.normals))
    pairs=[]
    for i in range(len(stems)):
        for j in range(i+1,min(i+3,len(stems))):
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


def solve_moving(stems,grounds,pivots,models,origins,bounds,prior_sigma,held_out_candidate,verbose=True):
    """Greedy insertion followed by bounded joint refinement; reference side is immutable."""
    parameters=np.zeros((len(stems),6));seen=[]
    trusted_pairs=west_overlap_pairs(stems,parameters,pivots,max_distance=.02,subset='training',held_out_candidate=held_out_candidate)
    ground_train=[]
    for q,c,n in grounds:
        cells=np.floor(q[:,:2]/.12).astype(int);hold=(cells[:,0]+2*cells[:,1])%4==0
        ground_train.append((q[~hold],c[~hold],n[~hold]))
    def data_residual(i,v):
        pieces=[]
        for candidate,q in stems[i].items():
            if candidate==held_out_candidate:continue
            q=q[training_heights(q)]
            if len(q)<25:continue
            transformed=move(q,v,pivots[i])
            d=stem_residual(transformed,models[candidate]['parameters'])
            pieces.append(d/(.010*np.sqrt(len(q))))
            origin=move(np.asarray(origins[i])[None,:],v,pivots[i])[0]
            pieces.append(visibility_penalty(transformed,origin,models[candidate]['parameters'])/(.006*np.sqrt(len(q))))
        q,c,n=ground_train[i]
        if len(q):pieces.append(np.sum((move(q,v,pivots[i])-c)*n,axis=1)/(.006*np.sqrt(len(q))))
        pieces.append(v/prior_sigma)
        return np.concatenate(pieces)
    # Information-rich recordings enter first; neighbour priors are soft, not locks.
    order=sorted(range(len(stems)),key=lambda i:sum(len(q) for k,q in stems[i].items() if k!=held_out_candidate),reverse=True)
    for i in order:
        neighbours=[j for j in seen if abs(i-j)<=2]
        def residual(v):
            parts=[data_residual(i,v)]
            for j in neighbours:parts.append(relative_correction_residual(v,parameters[j],pivots[i],pivots[j],abs(i-j)))
            return np.concatenate(parts)
        fit=least_squares(residual,parameters[i],bounds=(-bounds,bounds),loss='soft_l1',f_scale=1.,max_nfev=100,x_scale=bounds)
        parameters[i]=fit.x;seen.append(i)
        if verbose:print('Inserted moving scan',i,'dy cm',round(fit.x[4]*100,2),'cost',round(fit.cost,3),flush=True)
    for _ in range(3):
        pairs=west_overlap_pairs(stems,parameters,pivots,subset='training',held_out_candidate=held_out_candidate)
        def joint(v):
            v=v.reshape(len(stems),6);parts=[data_residual(i,v[i]) for i in range(len(stems))]
            for i in range(len(stems)):
                for j in range(i+1,min(i+3,len(stems))):
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
        fit=least_squares(joint,parameters.ravel(),bounds=(-np.tile(bounds,len(stems)),np.tile(bounds,len(stems))),loss='soft_l1',f_scale=1.,max_nfev=100,x_scale=np.tile(bounds,len(stems)),ftol=1e-7)
        parameters=fit.x.reshape(len(stems),6)
    return parameters,order,bool(fit.success)
