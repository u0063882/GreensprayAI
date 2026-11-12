"""Measure raw gravity-levelled scan geometry without changing its orientation.

No registered cloud, fitted scan pose, mesh, PCA-up frame or symmetry target is
used. A row direction is fitted horizontally from low-stem/support candidates;
that direction only defines the diagnostic cross-section. Wall inclinations
summarise visible vegetation, not verified individual branch axes.
"""
import itertools
import numpy as np
import open3d as o3d
from scipy.optimize import least_squares

def fit_stem(points,ground,row,rng):
 q=np.asarray(points,dtype=float);zref=float(np.median(q[:,2]));seed=np.median(q[:,:2],axis=0);candidates=[]
 height_bins=np.floor((q[:,2]-q[:,2].min())/.035).astype(int)
 bins=[np.flatnonzero(height_bins==k) for k in np.unique(height_bins) if (height_bins==k).sum()>=4]
 for _ in range(250):
  if not bins:break
  ids=bins[rng.integers(len(bins))];pair=q[rng.choice(ids,3,replace=False)];A=2*(pair[1:,:2]-pair[0,:2]);b=np.sum(pair[1:,:2]**2,axis=1)-np.sum(pair[0,:2]**2)
  try:c=np.linalg.solve(A,b)
  except np.linalg.LinAlgError:continue
  radius=np.linalg.norm(c-pair[0,:2])
  if .02<radius<.13 and np.linalg.norm(c-seed)<.2:
   v=np.r_[c,0.,0.,radius,0.];d=np.linalg.norm(q[:,:2]-c,axis=1)-radius;score=np.mean(np.minimum(abs(d)/.012,1)**2);candidates.append((score,v))
 if not candidates:return None
 lower=np.r_[seed-.2,-1.,-1.,.02,-.15];upper=np.r_[seed+.2,1.,1.,.13,.15]
 def residual(v):
  axis=np.r_[v[2:4],1.];axis/=np.linalg.norm(axis);delta=q-np.r_[v[:2],zref];along=delta@axis;return np.linalg.norm(delta-along[:,None]*axis,axis=1)-(v[4]+v[5]*along)
 results=[]
 for _,v in sorted(candidates,key=lambda x:x[0])[:4]:
  fit=least_squares(residual,np.clip(v,lower+1e-8,upper-1e-8),bounds=(lower,upper),loss='soft_l1',f_scale=.004,max_nfev=80);v=fit.x;d=residual(v);keep=abs(d)<.010
  if keep.sum()<20 or keep.mean()<.65 or np.ptp(q[keep,2])<.15:continue
  if np.any((v-lower)/(upper-lower)<.015) or np.any((upper-v)/(upper-lower)<.015):continue
  axis=np.r_[v[2:4],1.];axis/=np.linalg.norm(axis);delta=q-np.r_[v[:2],zref];along=delta@axis;radial=delta-along[:,None]*axis;origin=-q;origin/=np.maximum(np.linalg.norm(origin,axis=1)[:,None],1e-9);visible=np.sum(radial/np.maximum(np.linalg.norm(radial,axis=1)[:,None],1e-9)*origin,axis=1)>.0
  if np.mean(visible[keep])<.8:continue
  # The fitted inclination is measured, never pulled toward vertical.
  outward=float(np.rad2deg(np.arctan(np.dot(np.r_[v[2:4],0.],row['outward']))));along_angle=float(np.rad2deg(np.arctan(np.dot(np.r_[v[2:4],0.],row['along']))));total=float(np.rad2deg(np.arctan(np.linalg.norm(v[2:4]))))
  result=dict(centre_at_reference_z=np.r_[v[:2],zref].tolist(),axis=axis.tolist(),radius_m=float(v[4]),diameter_cm=float(200*v[4]),taper=float(v[5]),inclination_from_gravity_deg=total,outward_inclination_deg=outward,along_row_inclination_deg=along_angle,points=len(q),inliers=int(keep.sum()),rmse_m=float(np.sqrt(np.mean(d[keep]**2))),role='plausible trunk' if .055<=v[4]<=.11 else 'small stem / support candidate',uncertainty='Partial low-height tube, free lean/radius/taper; inferred role is not verified tree identity')
  results.append((np.mean(np.minimum(abs(d)/.012,1)**2),result))
 return min(results,key=lambda x:x[0])[1] if results else None


def plane(points,threshold=.025):
 points=np.asarray(points);_,sample=np.unique(np.floor(points/.025).astype(np.int64),axis=0,return_index=True);pc=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points[sample]));o3d.utility.random.seed(123)
 v,ix=pc.segment_plane(distance_threshold=threshold,ransac_n=3,num_iterations=2000)
 if v[2]<0:v=-v
 if v[2]<.85:raise ValueError('Dominant detected plane is not compatible with a ground surface')
 p=np.asarray(points);keep=abs(p@v[:3]+v[3])<threshold*1.5;q=p[keep]
 A=np.c_[q[:,:2],np.ones(len(q))];b=q[:,2];f=least_squares(lambda c:A@c-b,np.r_[-v[:2]/v[2],-v[3]/v[2]],loss='soft_l1',f_scale=.008,max_nfev=60);coef=f.x;n=np.r_[-coef[:2],1.];n/=np.linalg.norm(n);d=-(coef[2])*n[2];err=q@n+d
 return dict(normal=n,offset=d,coef=coef,inliers=len(q),rmse_m=float(np.sqrt(np.mean(err**2))))

def row_geometry(p,g):
 height=p[:,2]-(p[:,:2]@g['coef'][:2]+g['coef'][2]);mask=(height>.13)&(height<.47)&(p[:,0]>.6)&(p[:,0]<5.)&(abs(p[:,1])<3.8);q=p[mask]
 pc=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.c_[q[:,:2],np.zeros(len(q))]));labels=np.asarray(pc.cluster_dbscan(eps=.045,min_points=5,print_progress=False));clusters=[]
 for k in np.unique(labels):
  if k<0:continue
  c=q[labels==k]
  if len(c)<18 or np.ptp(c[:,2])<.13 or np.max(np.ptp(c[:,:2],axis=0))>.38:continue
  clusters.append(dict(points=c,centre=np.median(c[:,:2],axis=0),count=len(c)))
 if len(clusters)<3:raise ValueError('Insufficient low-stem/support candidates to estimate a local row axis')
 xy=np.array([c['centre'] for c in clusters]);choices=[]
 for i,j in itertools.combinations(range(len(xy)),2):
  h=xy[j]-xy[i];length=np.linalg.norm(h)
  if length<.7:continue
  h/=length
  if abs(h[1])<.65:continue
  h*=np.sign(h[1]);a=np.array([h[1],-h[0]]);distance=abs((xy-xy[i])@a);keep=distance<.17
  if keep.sum()<3:continue
  span=np.ptp(xy[keep]@h)
  if span<1.3:continue
  score=keep.sum()+span/.5-float(np.mean(distance[keep]))/.17;choices.append((score,keep))
 if not choices:raise ValueError('No coherent low-stem row with sufficient horizontal span')
 keep=max(choices,key=lambda x:x[0])[1]
 for _ in range(3):
  centre=np.mean(xy[keep],axis=0);_,_,vt=np.linalg.svd(xy[keep]-centre);h=vt[0];h*=np.sign(h[1]);a=np.array([h[1],-h[0]]);keep=abs((xy-centre)@a)<.17
 centre=np.mean(xy[keep],axis=0);h=np.r_[h,0.];outward=np.r_[-a,0.];ground_z=float(centre@g['coef'][:2]+g['coef'][2]);c=np.r_[centre,ground_z];along=(p-c)@h;out=(p-c)@outward;lim=np.quantile((xy[keep]-centre)@h[:2],[0,1])+[-.25,.25]
 return dict(centre=c,along=h,outward=outward,extent=lim,clusters=[v for v,k in zip(clusters,keep) if k],row_residual_m=float(np.sqrt(np.mean(((xy[keep]-centre)@a)**2))))

def wall_fit(p,wood,row,band=(.65,2.35),quantile=.5,wood_only=False):
 c=row['centre'];h=(p-c)@row['along'];u=(p-c)@row['outward'];z=p[:,2]-c[2]
 keep=(h>row['extent'][0])&(h<row['extent'][1])&(u>.03)&(u<1.7)&(z>band[0])&(z<band[1])
 if wood_only:keep&=wood>.65
 ix=np.flatnonzero(keep);q=np.c_[h[ix],u[ix],z[ix]];_,sub=np.unique(np.floor(q/.025+1e-9).astype(np.int64),axis=0,return_index=True);q=q[sub];cells=np.c_[np.floor(q[:,0]/.45+1e-9),np.floor(q[:,2]/.12+1e-9)].astype(int);keys,inv=np.unique(cells,axis=0,return_inverse=True);samples=[]
 for k,key in enumerate(keys):
  a=q[inv==k]
  if len(a)<8:continue
  samples.append([float(np.median(a[:,0])),float(np.quantile(a[:,1],quantile)),float(np.median(a[:,2])),len(a)])
 samples=np.array(samples)
 if len(samples)<15 or np.ptp(samples[:,2])<.9:raise ValueError('Insufficient tree-wall support across height')
 A=np.c_[samples[:,0],samples[:,2],np.ones(len(samples))];b=samples[:,1];v=np.linalg.lstsq(A,b,rcond=None)[0];fit=least_squares(lambda v:A@v-b,v,loss='soft_l1',f_scale=.07,max_nfev=80);v=fit.x
 normal=row['outward']-v[0]*row['along']-v[1]*np.array([0.,0.,1.]);normal/=np.linalg.norm(normal)
 return dict(outward_tilt_from_vertical_deg=float(np.rad2deg(np.arctan(v[1]))),plane_tilt_from_vertical_deg=float(np.rad2deg(np.arctan(v[1]/np.sqrt(1+v[0]**2)))),normal=normal.tolist(),coef=v.tolist(),samples=samples.tolist(),height_band_m=list(band),quantile=quantile,wood_only=wood_only,cells=len(samples),fit_rmse_m=float(np.sqrt(np.mean(fit.fun**2))))

def analyse_raw_cloud(data,metadata=None):
 metadata={} if metadata is None else metadata
 p=data['xyz'].astype(float);g=plane(p[(p[:,0]>.65)&(p[:,0]<5.)&(abs(p[:,1])<3.8)&(p[:,2]>-2.9)&(p[:,2]<-.65)])
 row=row_geometry(p,g);out=(p-row['centre'])@row['outward'];along=(p-row['centre'])@row['along'];dist=p@g['normal']+g['offset'];q=p[(abs(out)<.65)&(along>row['extent'][0])&(along<row['extent'][1])&(abs(dist)<.10)];g=plane(q);row=row_geometry(p,g)
 fits=[]
 for band in [(.65,2.35),(.65,1.65),(1.,2.4)]:
  for quant in [.25,.5,.75]:
   try:fits.append(wall_fit(p,data.get('wood_score',np.full(len(p),.5)),row,band,quant))
   except ValueError:pass
 try:fits.append(wall_fit(p,data.get('wood_score',np.full(len(p),.5)),row,(.65,2.35),.5,True))
 except ValueError:pass
 primary=next(f for f in fits if f['height_band_m']==[.65,2.35] and f['quantile']==.5 and not f['wood_only']);normal=np.array(primary['normal']);dihedral=float(np.rad2deg(np.arccos(np.clip(abs(normal@g['normal']),0,1))));ng=g['normal'];ground_cross=float(np.rad2deg(np.arctan(-ng@row['outward']/ng[2])));ground_along=float(np.rad2deg(np.arctan(-ng@row['along']/ng[2])))
 rng=np.random.default_rng(102);stems=[]
 for cluster in row['clusters']:
  stem=fit_stem(cluster['points'],g,row,rng)
  if stem:stems.append(stem)
 report={**metadata,'raw_stem_fits':stems,'ground':{k:(v.tolist() if isinstance(v,np.ndarray) else v) for k,v in g.items()},'row':{k:(v.tolist() if isinstance(v,np.ndarray) else v) for k,v in row.items() if k!='clusters'},'low_stem_candidates':[dict(centre=c['centre'].tolist(),points=c['count'],height_span_m=float(np.ptp(c['points'][:,2]))) for c in row['clusters']],'tree_wall':primary,'sensitivity':fits,'ground_tilt_from_horizontal_deg':float(np.rad2deg(np.arccos(ng[2]))),'ground_slope_outward_deg':ground_cross,'ground_slope_along_row_deg':ground_along,'wall_ground_dihedral_deg':dihedral}
 return report,row,g

