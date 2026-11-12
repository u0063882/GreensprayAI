"""A shared structural-up frame from paired-view wood-centre PCA.

This chooses a coordinate convention, not surveyed Earth vertical. It never
rotates individual views toward their own PCA axes or changes relative shape.
"""
import numpy as np
from scipy.spatial.transform import Rotation

DEFAULTS=dict(voxel_m=.04,window_m=.8,height_bin_m=.15,height_range_m=[.45,2.85],
    minimum_points_per_view_bin=8,minimum_bins=8,minimum_windows=3,
    minimum_rotation_deg=1.,bootstrap_samples=256)


def estimate_structural_frame(clouds,wood_scores,side_labels,pivot_y=0.,settings=None):
    opt={**DEFAULTS,**(settings or {})};labels=list(dict.fromkeys(side_labels))
    if len(labels)!=2 or not (len(clouds)==len(wood_scores)==len(side_labels)):
        raise ValueError('Two labelled viewing sides and one score array per cloud required')
    sides={}
    for label in labels:
        parts=[]
        for p,s,side in zip(clouds,wood_scores,side_labels):
            if side!=label:continue
            p=np.asarray(p);s=np.asarray(s)
            if p.ndim!=2 or p.shape[1]!=3 or len(p)!=len(s):raise ValueError('Expected Nx3 points and N wood scores')
            keep=np.isfinite(p).all(axis=1)&(s>.65)&(p[:,2]>.4)&(p[:,2]<3.2);parts.append(p[keep])
        q=np.concatenate(parts)
        if len(q)<100:raise ValueError('Insufficient provisional wood returns')
        _,ix=np.unique(np.floor(q/opt['voxel_m']).astype(np.int64),axis=0,return_index=True);sides[label]=q[ix]
    all_points=np.concatenate(list(sides.values()));xmin,xmax=np.quantile(all_points[:,0],[.03,.97]);edges=np.arange(xmin,xmax+opt['window_m'],opt['window_m']);windows=[];covariances=[]
    for lo,hi in zip(edges[:-1],edges[1:]):
        path=[]
        for zlo in np.arange(opt['height_range_m'][0],opt['height_range_m'][1],opt['height_bin_m']):
            centres=[]
            for label in labels:
                q=sides[label];part=q[(q[:,0]>lo)&(q[:,0]<hi)&(q[:,2]>zlo)&(q[:,2]<zlo+opt['height_bin_m'])]
                if len(part)<opt['minimum_points_per_view_bin']:break
                centres.append(np.median(part[:,[1,2]],axis=0))
            if len(centres)==2:path.append(np.mean(centres,axis=0))
        if len(path)<opt['minimum_bins']:continue
        path=np.asarray(path);cov=np.cov(path.T);eig,V=np.linalg.eigh(cov);axis=V[:,-1]*np.sign(V[-1,-1]);angle=np.rad2deg(np.arctan2(axis[0],axis[1]));windows.append(dict(along_m=[float(lo),float(hi)],angle_deg=float(angle),centres_yz_m=path.tolist(),eigenvalues=eig.tolist()));covariances.append(cov)
    if len(windows)<opt['minimum_windows']:raise ValueError('Too few paired-view height spines')
    angles=np.array([w['angle_deg'] for w in windows]);mid=np.median(angles);mad=max(1.,1.4826*np.median(abs(angles-mid)));keep=abs(angles-mid)<=max(3.,3*mad)
    covariances=np.array(covariances)[keep]
    if len(covariances)<opt['minimum_windows']:raise ValueError('Insufficient consistent structural directions')
    def angle(cov):
        eig,V=np.linalg.eigh(cov);v=V[:,-1]*np.sign(V[-1,-1]);return float(np.rad2deg(np.arctan2(v[0],v[1])))
    correction=angle(np.mean(covariances,axis=0));rng=np.random.default_rng(29)
    boot=[angle(covariances[rng.integers(len(covariances),size=len(covariances))].mean(axis=0)) for _ in range(opt['bootstrap_samples'])];interval=np.quantile(boot,[.025,.975])
    supported=bool(interval[0]*interval[1]>0 and abs(correction)>=opt['minimum_rotation_deg'])
    T=np.eye(4);pivot=np.array([0.,pivot_y,0.]);R=Rotation.from_euler('x',correction if supported else 0.,degrees=True).as_matrix();T[:3,:3]=R;T[:3,3]=pivot-R@pivot
    return T,dict(method=__doc__.strip(),settings=opt,estimated_rotation_deg=correction,applied_rotation_deg=correction if supported else 0.,supported=supported,bootstrap_interval_deg=interval.tolist(),windows=windows,retained_windows=keep.tolist(),pivot_m=pivot.tolist(),registration_to_structural=T.tolist(),physical_gravity_reference_in_output=(R@np.array([0.,0.,1.])).tolist(),reference_policy='Z denotes paired-view woody growth direction, not measured gravity or a horizontal soil plane. One common rigid transform preserves all relative poses, branch angles and distances.',accuracy_limit='Bootstrap describes window-to-window structural direction variation, not uncertainty of surveyed vertical. Provisional wood classification and occlusion can bias structural-up estimates.')
