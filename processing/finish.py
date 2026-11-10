import os
os.environ['OMP_NUM_THREADS']='2';os.environ['OPENBLAS_NUM_THREADS']='2';os.environ.setdefault('MPLCONFIGDIR',os.path.join(os.getcwd(),'work','mpl'))
from pathlib import Path
import numpy as np,json,open3d as o3d
from scipy.spatial import cKDTree
from sklearn.cluster import DBSCAN
from scipy.signal import find_peaks
from scipy.ndimage import gaussian_filter1d
out=Path('output');out.mkdir(exist_ok=True)
poses=np.load('work/registered/final_poses.npy');files=sorted(Path('work/decoded').glob('*.npz'))
pts=[];refs=[];sources=[]
for i,(p,T) in enumerate(zip(files,poses)):
 d=np.load(p);q=d['xyz']@T[:3,:3].T+T[:3,3];pts.append(q);refs.append(d['reflectivity']);sources.append(np.full(len(q),i,dtype='u1'))
pts=np.concatenate(pts);refs=np.concatenate(refs);sources=np.concatenate(sources)
# Estimate tree-row centre from stems, independent of canopy surface seen from either side.
base=pts[(pts[:,2]>.12)&(pts[:,2]<.45)&(pts[:,0]>-.5)&(pts[:,0]<6.6)]
yhist,bins=np.histogram(base[:,1],bins=np.arange(.9,2.8,.025));yi=np.argmax(gaussian_filter1d(yhist.astype(float),2));cy=bins[yi]+.0125
cy=1.9875  # Preserve the previously delivered crop centre.
print('row centre',cy,flush=True)
# Keep a strip broad enough to preserve canopy and only direct ground beneath it.
mask=(pts[:,0]>-.5)&(pts[:,0]<6.6)&(abs(pts[:,1]-cy)<.70)&(pts[:,2]>-.12)&(pts[:,2]<4.5)
p=pts[mask];r=refs[mask];s=sources[mask]
_,idx=np.unique(np.floor(p/.012).astype(np.int32),axis=0,return_index=True);p=p[idx];r=r[idx];s=s[idx]
pc=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(p));_,keep=pc.remove_radius_outlier(nb_points=5,radius=.06)
p=p[keep];r=r[keep];s=s[keep]
# Density clusters in the trunk band. Candidates are preserved as provisional labels, not asserted biological identities.
trunk=p[(p[:,2]>.13)&(p[:,2]<.48)&(abs(p[:,1]-cy)<.30)]
tpc=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(trunk)).voxel_down_sample(.025)
tp=np.asarray(tpc.points);lab=DBSCAN(eps=.09,min_samples=6,n_jobs=2).fit_predict(tp[:,:2])
centres=[]
for k in np.unique(lab):
 if k<0:continue
 q=tp[lab==k]
 if len(q)>=14: centres.append(np.median(q,axis=0))
centres=np.array(sorted(centres,key=lambda a:a[0]));print('stem candidates',centres.tolist(),flush=True)
np.savez_compressed('work/registered/cropped.npz',xyz=p,reflectivity=r,source=s,centres=centres)
print('points',len(p),flush=True)
