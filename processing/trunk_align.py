import os
os.environ['OMP_NUM_THREADS']='2';os.environ['OPENBLAS_NUM_THREADS']='2';os.environ.setdefault('MPLCONFIGDIR',os.path.join(os.getcwd(),'work','mpl'))
from pathlib import Path
import numpy as np,open3d as o3d,json
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
reg=o3d.pipelines.registration;poses=np.load('work/registered/poses.npy');files=sorted(Path('work/decoded').glob('*.npz'));sides=[]
for ids in [range(9),range(9,18)]:
 ps=[]
 for i in ids:
  q=np.load(files[i])['xyz'];T=poses[i];q=q@T[:3,:3].T+T[:3,3];ps.append(q[(q[:,2]>.13)&(q[:,2]<.38)&(q[:,0]>-1)&(q[:,0]<8)&(q[:,1]>1.2)&(q[:,1]<2.8)])
 pc=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.concatenate(ps)));sides.append(pc.voxel_down_sample(.025))
a,b=sides;xa=np.asarray(a.points);xb=np.asarray(b.points);tree=cKDTree(xa);print(len(xa),len(xb),flush=True)
candidates=[]
for dx in np.arange(-.8,1.21,.2):
 for dy in [-.5,-.25,0]:
  T=np.eye(4);T[:2,3]=[dx,dy]
  for th in [.25,.13,.07]:
   for iteration in range(35):
    q=xb@T[:3,:3].T+T[:3,3]
    dist,ix=cKDTree(xa[:,:2]).query(q[:,:2],workers=2)
    m=dist<th
    if m.sum()<50: break
    src=q[m,:2];dst=xa[ix[m],:2]
    w=1/(1+(dist[m]/.045)**2);w/=w.sum()
    cs=np.sum(src*w[:,None],axis=0);ct=np.sum(dst*w[:,None],axis=0)
    H=(src-cs).T@((dst-ct)*w[:,None]);u,sv,vt=np.linalg.svd(H)
    rot=vt.T@u.T
    if np.linalg.det(rot)<0: vt[-1]*=-1;rot=vt.T@u.T
    E=np.eye(4);E[:2,:2]=rot;E[:2,3]=ct-rot@cs;T=E@T
    if np.linalg.norm(E-np.eye(4))<1e-5:break
  q=xb@T[:3,:3].T+T[:3,3];dist=cKDTree(xa[:,:2]).query(q[:,:2],workers=2)[0];da=cKDTree(q[:,:2]).query(xa[:,:2],workers=2)[0]
  score=float((np.mean(np.minimum(dist,.12)**2)+np.mean(np.minimum(da,.12)**2))/2)
  fitness=float(np.mean(dist<.07));rmse=float(np.sqrt(np.mean(dist[dist<.07]**2)))
  candidates.append((score,T,fitness,rmse))
print('top',[(round(a,6),t[:3,3].round(3).tolist(),round(f,3),round(r,3)) for a,t,f,r in sorted(candidates,key=lambda a:a[0])[:10]],flush=True)
score,T,fitness,rmse=min(candidates,key=lambda a:a[0]);print('BEST',T,flush=True)
np.save('work/registered/trunk_cross_transform.npy',T);poses[9:]=T@poses[9:];np.save('work/registered/final_poses.npy',poses)
Path('work/registered/trunk_quality.json').write_text(json.dumps(dict(transform=T.tolist(),fitness=fitness,rmse=rmse,score=score),indent=2))
