import os
os.environ['OMP_NUM_THREADS']='2';os.environ['OPENBLAS_NUM_THREADS']='2'
from pathlib import Path
import numpy as np,json,open3d as o3d
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares
D=Path('work/decoded'); scans=json.loads((D/'scans.json').read_text());out=Path('work/registered');out.mkdir(exist_ok=True)
lat0=scans[0]['lat'];lon0=scans[0]['lon'];R_earth=6378137
p=np.array([[(a['lon']-lon0)*np.pi/180*R_earth*np.cos(np.deg2rad(lat0)),(a['lat']-lat0)*np.pi/180*R_earth] for a in scans])
row=p[8]-p[0];row/=np.linalg.norm(row);left=np.array([-row[1],row[0]]);gps=p@np.array([row,left]).T
initial=[];clouds=[];grounds=[]
for i,a in enumerate(scans):
 xyz=np.load(D/(a['tag']+'.npz'))['xyz']; yaw=np.pi/2 if i<9 else -np.pi/2
 # Ground elevation is measured independently because H5 contains no altitude.
 g=xyz[(xyz[:,2]<-1.25)&(xyz[:,0]>.7)&(xyz[:,0]<2.7)]
 z0=-np.median(g[:,2]); grounds.append(z0)
 T=np.eye(4);T[:3,:3]=Rotation.from_euler('z',yaw).as_matrix();T[:2,3]=gps[i];T[2,3]=z0;initial.append(T)
 xyz=xyz[(xyz[:,0]>.7)&(xyz[:,0]<2.8)&(xyz[:,2]>-1.35)]
 pc=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz));pc=pc.voxel_down_sample(.035);pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=.14,max_nn=35));clouds.append(pc)
np.savez(out/'initial.npz',poses=np.array(initial),gps=gps)
reg=o3d.pipelines.registration
edges=[]
for side in [range(9),range(9,18)]:
 for i in side:
  for j in side:
   if j<=i or abs(j-i)>3:continue
   T0=np.linalg.inv(initial[j])@initial[i];T=T0.copy()
   for th in [.25,.13,.07]:
    r=reg.registration_icp(clouds[i],clouds[j],th,T,reg.TransformationEstimationPointToPlane(reg.TukeyLoss(k=th*.7)),reg.ICPConvergenceCriteria(max_iteration=35));T=r.transformation
   delta=np.linalg.inv(T0)@T;dr=np.linalg.norm(Rotation.from_matrix(delta[:3,:3]).as_rotvec());dt=np.linalg.norm(delta[:3,3])
   accept=r.fitness>.18 and dt<.45 and dr<.12
   print('edge',i,j,round(r.fitness,3),round(r.inlier_rmse,3),'shift',round(dt,3),accept,flush=True)
   if accept:edges.append((i,j,T,r.fitness,r.inlier_rmse))
# Global graph, retain heading and position priors so repetitive orchard geometry cannot shift tree indices.
def params(T):return np.r_[Rotation.from_matrix(T[:3,:3]).as_rotvec(),T[:3,3]]
def matrix(v):
 T=np.eye(4);T[:3,:3]=Rotation.from_rotvec(v[:3]).as_matrix();T[:3,3]=v[3:];return T
v0=np.array([params(T) for T in initial])
def fun(v):
 v=v.reshape(18,6);Ts=[matrix(a) for a in v];rs=[]
 for i,j,T,f,rm in edges:
  E=np.linalg.inv(T)@np.linalg.inv(Ts[j])@Ts[i]
  rs.extend(Rotation.from_matrix(E[:3,:3]).as_rotvec()/.014);rs.extend(E[:3,3]/.035)
 for i in range(18):
  E=np.linalg.inv(initial[i])@Ts[i]
  rs.extend(Rotation.from_matrix(E[:3,:3]).as_rotvec()/np.array([.025,.025,.09]));rs.extend(E[:3,3]/np.array([.2,.2,.08]))
 return rs
sol=least_squares(fun,v0.ravel(),loss='soft_l1',max_nfev=60,ftol=1e-7)
poses=np.array([matrix(v) for v in sol.x.reshape(18,6)])
np.savez(out/'within_sides.npz',poses=poses,gps=gps)
(out/'within_edges.json').write_text(json.dumps([dict(i=i,j=j,T=T.tolist(),fitness=f,rmse=rm) for i,j,T,f,rm in edges],indent=2))
# Aggregate each side for cross-side registration. Restrict to trunk/branch points above ground.
sides=[]
for ids in [range(9),range(9,18)]:
 pc=o3d.geometry.PointCloud()
 for i in ids: pc+=o3d.geometry.PointCloud(clouds[i]).transform(poses[i])
 pc=pc.voxel_down_sample(.035);pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=.14,max_nn=40));sides.append(pc)
for label,pc in zip(['A','B'],sides):o3d.io.write_point_cloud(str(out/f'side{label}.ply'),pc)
np.save(out/'poses.npy',poses)
print('WITHIN SIDE COMPLETE',flush=True)
