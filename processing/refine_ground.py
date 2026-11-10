import os
os.environ['OMP_NUM_THREADS']='2';os.environ['OPENBLAS_NUM_THREADS']='2';os.environ.setdefault('MPLCONFIGDIR',os.path.join(os.getcwd(),'work','mpl'))
from pathlib import Path
import numpy as np,open3d as o3d,json
from scipy.optimize import least_squares, lsq_linear, OptimizeResult
from scipy.spatial.transform import Rotation
from scipy.sparse import coo_matrix,csr_matrix,vstack
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path.cwd();D=root/'work/decoded';files=sorted(D.glob('*.npz'));baseline=root/'work/registration_v1/final_poses.npy';baseline.parent.mkdir(exist_ok=True)
if not baseline.exists():np.save(baseline,np.load(root/'work/registered/final_poses.npy'))
poses=np.load(baseline);ref=np.array([3.,1.9875,0.])
o3d.utility.random.seed(7)
patches={};allground=[];allcloud=[];planes=[]
for i,(file,T) in enumerate(zip(files,poses)):
 p=np.load(file)['xyz'];q=p@T[:3,:3].T+T[:3,3];allcloud.append(q)
 m=(q[:,0]>-.7)&(q[:,0]<6.8)&(q[:,1]>1.2875)&(q[:,1]<2.6875)&(q[:,2]>-.23)&(q[:,2]<.23)
 g=q[m];pc=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(g)).voxel_down_sample(.025)
 plane,inliers=pc.segment_plane(distance_threshold=.023,ransac_n=3,num_iterations=1000)
 if plane[2]<0:plane=-plane
 distances=abs(g@plane[:3]+plane[3]);g=g[distances<.027]
 allground.append(g);planes.append(plane)
 # Hold out a checkerboard of spatial cells for verification.
 ij=np.floor(g[:,:2]/.15).astype('i4');_,inv=np.unique(ij,axis=0,return_inverse=True)
 cells=[]
 for k in range(inv.max()+1):
  pcell=g[inv==k]
  if len(pcell)<8:continue
  mid=np.median(pcell,axis=0);c=tuple(np.floor(mid[:2]/.15).astype(int));patches.setdefault(c,[]).append((i,mid,len(pcell)))
 print(i,file.stem,'ground',len(g),'plane slopes',(-plane[:2]/plane[2]).round(5),'zmid',round(float(-(plane[0]*3+plane[1]*1.9875+plane[3])/plane[2]),4),flush=True)
rows=[];cols=[];vals=[];targets=[];weights=[];edgeinfo=[];heldout=[];n=0
for cell,obs in patches.items():
 if len(obs)<2:continue
 for ia in range(len(obs)):
  for ib in range(ia+1,len(obs)):
   i,a,ca=obs[ia];j,b,cb=obs[ib]
   record=(i,j,a,b)
   # Every fourth ground patch is excluded from fitting.
   if (cell[0]+2*cell[1])%4==0:heldout.append(record);continue
   u=a-ref;v=b-ref
   for c,x in [(3*i,u[1]),(3*i+1,-u[0]),(3*i+2,1),(3*j,-v[1]),(3*j+1,v[0]),(3*j+2,-1)]:
    rows.append(n);cols.append(c);vals.append(x)
   targets.append(b[2]-a[2]);weights.append(1/np.sqrt(max(1,len(obs)-1)));edgeinfo.append(record);n+=1
A=coo_matrix((vals,(rows,cols)),shape=(n,54)).tocsr();b=np.array(targets);w=np.array(weights)
# Low-weight priors preserve the gauge and discourage unsupported tilt changes.
P=np.diag(np.tile([.04,.04,.008],18))
# Anchor the mean correction, preserving original row frame rather than declaring the soil flat.
G=np.zeros((3,54))
for i in range(18):G[:,3*i:3*i+3]=np.diag([300.,300.,100.])/18
prior=csr_matrix(np.vstack([P,G]));AA=vstack([A.multiply(w[:,None]),prior]).tocsr();bb=np.r_[b*w,np.zeros(prior.shape[0])]
x=np.zeros(54);converged=False
limits=(np.tile([-.035,-.035,-.18],18),np.tile([.035,.035,.18],18))
# IRLS solves each bounded 54-variable quadratic exactly; robust cell residuals
# keep weeds, stones and grazing-angle samples from driving the poses.
for iteration in range(120):
 residual=AA@x-bb;weights=1/np.sqrt(1+(residual/.0075)**2)
 H=(AA.T@AA.multiply(weights[:,None])).toarray()+np.eye(54)*1e-10
 rhs=AA.T@(weights*bb);L=np.linalg.cholesky(H)
 step=lsq_linear(L.T,np.linalg.solve(L,rhs),bounds=limits,tol=1e-12,max_iter=200)
 change=np.max(abs(step.x-x));x=step.x
 if change<1e-8:converged=True;break
sol=OptimizeResult(x=x,success=converged,message=f'IRLS iterations {iteration+1}, final parameter change {change:.3g}')

corr=sol.x.reshape(18,3);new=[];corrected=[]
for i,(T,v) in enumerate(zip(poses,corr)):
 R=Rotation.from_rotvec([v[0],v[1],0]).as_matrix();E=np.eye(4);E[:3,:3]=R;E[:3,3]=ref-R@ref+[0,0,v[2]];new.append(E@T);corrected.append(allground[i]@R.T+E[:3,3])
 print('correction',i,'roll pitch deg',np.rad2deg(v[:2]).round(3),'dz',round(v[2],4),flush=True)
# Evaluate held-out cell height differences using exact rigid transforms.
def metrics(records):
 before=[];after=[]
 for i,j,a,b in records:
  before.append(a[2]-b[2]);zz=[]
  for k,p in [(i,a),(j,b)]:
   v=corr[k];R=Rotation.from_rotvec([v[0],v[1],0]).as_matrix();zz.append(((p-ref)@R.T+ref+[0,0,v[2]])[2])
  after.append(zz[0]-zz[1])
 def stat(a):
  a=np.asarray(a);return dict(rmse_m=float(np.sqrt(np.mean(a*a))),median_absolute_m=float(np.median(abs(a))),p95_absolute_m=float(np.percentile(abs(a),95)),pairs=len(a))
 return dict(before=stat(before),after=stat(after))
quality=dict(training=metrics(edgeinfo),held_out_ground_patches=metrics(heldout),corrections=[dict(scan_id=i,roll_deg=float(np.rad2deg(v[0])),pitch_deg=float(np.rad2deg(v[1])),dz_m=float(v[2])) for i,v in enumerate(corr)],method='Relative rigid roll/pitch/height corrections using overlapping 15cm ground patches. Every fourth patch held out. Mean correction anchored; no ground flattening or per-point deformation.',solver_success=bool(sol.success),solver_message=sol.message)
np.save(root/'work/registered/ground_refined_poses.npy',np.array(new));(root/'work/registered/ground_refinement_quality.json').write_text(json.dumps(quality,indent=2))
print(json.dumps({k:v for k,v in quality.items() if k!='corrections'},indent=2),flush=True)
# Ground-only side profile highlights doubled sheets without canopy hiding them.
fig,axs=plt.subplots(2,1,figsize=(14,6),sharex=True,facecolor='#101823');palette=plt.get_cmap('tab20')(np.arange(18))
for ax,ground,title in zip(axs,[allground,corrected],['Before ground refinement','After ground refinement']):
 for i,g in enumerate(ground):ax.scatter(g[::3,0],g[::3,2],c=[palette[i]],s=.5,alpha=.45,linewidths=0)
 ax.set_facecolor('#101823');ax.set_title(title,color='white');ax.set_ylabel('Ground height (m)',color='#d0dce8');ax.tick_params(colors='#d0dce8');ax.grid(alpha=.15);ax.set_ylim(-.14,.14)
axs[-1].set_xlabel('Along row (m)',color='#d0dce8');fig.tight_layout();fig.savefig(root/'output/ground_refinement_before_after.png',dpi=160,facecolor=fig.get_facecolor())

assert sol.success and quality["held_out_ground_patches"]["after"]["rmse_m"]<quality["held_out_ground_patches"]["before"]["rmse_m"]
np.save(root/"work/registered/final_poses.npy",np.array(new))
