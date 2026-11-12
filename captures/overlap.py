"""Rigid overlap checks with fixed validation returns and renewed correspondences.

Large pose repairs invalidate old nearest-neighbour identities. Validation
keeps the original query/target return sets, re-matches them in each state,
and charges the distance cap for every unmatched return. Coverage is reported.
"""
import numpy as np
from scipy.spatial import cKDTree
from captures.structural_validation import _holdout,_sample,_normals


def validation_features(clouds,scores,origins,voxel=.025):
    features=[]
    for p,s,o in zip(clouds,scores,origins):
        p=np.asarray(p);keep=(p[:,2]>.55)&(p[:,2]<3.6)&(np.asarray(s)>.65)&_holdout(p,.12)
        ix=np.flatnonzero(keep);ix=ix[_sample(p[ix],voxel)];q=p[ix]
        n=_normals(q,np.asarray(o),.085) if len(q)>=30 else np.zeros_like(q)
        features.append((q,n))
    return features


def _stats(distance,plane,matched):
    if not len(distance):return None
    return dict(points=len(distance),coverage=float(np.mean(matched)),
        capped_distance_rmse_m=float(np.sqrt(np.mean(np.asarray(distance)**2))),
        capped_plane_rmse_m=float(np.sqrt(np.mean(np.asarray(plane)**2))))


def rematched_overlap(features,labels,corrections,cap=.08,normal_dot=.85):
    labels=np.asarray(labels);C=np.asarray(corrections);all_p=[];all_n=[]
    for (p,n),T in zip(features,C):all_p.append(p@T[:3,:3].T+T[:3,3]);all_n.append(n@T[:3,:3].T)
    result={};aggregate={kind:[[],[],[]] for kind in ['same_side','cross_side']}
    for i,(p,n) in enumerate(zip(all_p,all_n)):
        per={}
        for kind in aggregate:
            ids=[j for j in range(len(labels)) if j!=i and (labels[j]==labels[i])==(kind=='same_side')]
            if not ids:continue
            q=np.concatenate([all_p[j] for j in ids]);nn=np.concatenate([all_n[j] for j in ids])
            if not len(p) or not len(q):continue
            distance,k=cKDTree(q).query(p,k=min(12,len(q)),workers=2)
            if distance.ndim==1:distance=distance[:,None];k=k[:,None]
            agreement=np.sum(n[:,None,:]*nn[k],axis=2)
            cost=np.where(agreement>=normal_dot,distance,np.inf);pick=np.argmin(cost,axis=1);rows=np.arange(len(p));target=k[rows,pick];d=cost[rows,pick];matched=d<cap
            dd=np.minimum(d,cap);rr=np.full(len(p),cap);rr[matched]=np.minimum(abs(np.sum((p[matched]-q[target[matched]])*nn[target[matched]],axis=1)),cap)
            per[kind]=_stats(dd,rr,matched)
            aggregate[kind][0].extend(dd);aggregate[kind][1].extend(rr);aggregate[kind][2].extend(matched)
        result[str(i)]=per
    return dict(per_scan=result,aggregate={k:_stats(*v) for k,v in aggregate.items()},
        caution='Conditional internal overlap. Visibility, wood classification and density affect coverage. Every fixed validation return is charged a capped cost if unmatched; correspondence identities may change. No symmetry or horizontal-ground target is used.')
