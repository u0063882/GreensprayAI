import os
os.environ['OMP_NUM_THREADS']='2';os.environ['OPENBLAS_NUM_THREADS']='2'
from pathlib import Path
import numpy as np,json,open3d as o3d
from scipy.spatial import cKDTree
root=Path.cwd();files=sorted((root/'work/decoded').glob('*.npz'));raw=[np.load(f)['xyz'] for f in files];results={}
for version,poses in [('before',np.load(root/'work/registration_v1/final_poses.npy')),('after',np.load(root/'work/registered/ground_refined_poses.npy'))]:
 globalps=[p@T[:3,:3].T+T[:3,3] for p,T in zip(raw,poses)];bands={}
 for label,(lo,hi) in [('lower_trunks',(.15,.4)),('branches',(.5,1.5)),('upper_canopy',(2.,3.8))]:
  sides=[]
  for ids in [range(9),range(9,18)]:
   ps=np.concatenate([globalps[i] for i in ids]);m=(ps[:,0]>-.5)&(ps[:,0]<6.6)&(ps[:,1]>1.2875)&(ps[:,1]<2.6875)&(ps[:,2]>lo)&(ps[:,2]<hi)
   pc=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(ps[m])).voxel_down_sample(.035);sides.append(np.asarray(pc.points))
  ds=[]
  for a,b in [(sides[0],sides[1]),(sides[1],sides[0])]:
   d=cKDTree(b).query(a,workers=2)[0];m=d<.07;ds.append(dict(median_distance_m=float(np.median(d)),fraction_within_7cm=float(np.mean(m)),matched_rmse_m=float(np.sqrt(np.mean(d[m]**2))),capped_rmse_10cm_m=float(np.sqrt(np.mean(np.minimum(d,.1)**2)))))
  bands[label]=ds
 results[version]=bands
(root/'output/ground_refinement_tree_checks.json').write_text(json.dumps(results,indent=2));print(json.dumps(results,indent=2))
