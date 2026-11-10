import os
os.environ['OMP_NUM_THREADS']='2'
import numpy as np,json,open3d as o3d
from pathlib import Path
from scipy.spatial import cKDTree
poses=np.load('work/registered/final_poses.npy');files=sorted(Path('work/decoded').glob('*.npz'));sides=[]
for ids in [range(9),range(9,18)]:
 ps=[]
 for i in ids:
  q=np.load(files[i])['xyz'];T=poses[i];q=q@T[:3,:3].T+T[:3,3];ps.append(q[(q[:,2]>.13)&(q[:,2]<.38)&(q[:,0]>-.5)&(q[:,0]<6.6)&(abs(q[:,1]-1.9875)<.5)])
 pc=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.concatenate(ps))).voxel_down_sample(.025);sides.append(np.asarray(pc.points))
metrics={}
for label,a,b in [('A_to_B',sides[0],sides[1]),('B_to_A',sides[1],sides[0])]:
 d=cKDTree(b).query(a,workers=2)[0];m=d<.07
 metrics[label]=dict(point_count=len(a),fraction_within_7cm=float(np.mean(m)),matched_rmse_m=float(np.sqrt(np.mean(d[m]**2))),median_distance_m=float(np.median(d)))
pc=o3d.io.read_point_cloud('output/orchard_row_registered.ply');p=np.asarray(pc.points)
assert len(p)==len(np.load('work/registered/cropped.npz')['xyz']) and np.isfinite(p).all()
metrics['export']=dict(points=len(p),bounds_min=p.min(0).tolist(),bounds_max=p.max(0).tolist(),source_scans=18)
metrics['caveat']='Nearest-neighbour residuals on the fitted trunk band quantify agreement, not independent survey accuracy. Opposite sides observe different stem surfaces.'
Path('output/registration_quality.json').write_text(json.dumps(metrics,indent=2));print(json.dumps(metrics,indent=2))
