"""Population-assisted low-trunk observations for the independent joint stage.

Sizes help distinguish supports from trunks; they are representative ranges,
not exact per-tree ground truth. Final shared axes may lean freely. Detection
uses disjoint source-return height bands and never a V-shape template.
"""
import numpy as np
from scipy.optimize import least_squares

def detect_landmark(p,origin,seed,radius,zlo=.15,zhi=.45,training=True,role='trunk',x_span=None,rng=None,exclude=None,learn_radius=False,radius_sigma=.006):
 if rng is None:rng=np.random.default_rng(4)
 x_bound=x_span if x_span is not None else .14 if role=='trunk' else .28
 z=p[:,2];mask=(z>zlo)&(z<zhi)&(abs(p[:,0]-seed[0])<(.15 if role=='trunk' else .28))&(abs(p[:,1]-seed[1])<.30)
 if training:mask&=((z<.24)|(z>.36))
 else:mask&=((z>.26)&(z<.34))
 
 if exclude is not None:
  ec=np.asarray(exclude['centre'])+(p[:,2,None]-.3)*np.asarray(exclude['lean']);er=np.linalg.norm(p[:,:2]-ec,axis=1)
  mask &= er>exclude['radius']+.025
 ix=np.flatnonzero(mask);q=p[ix]
 if len(q)<20:return None
 if len(q)>350:pick=np.linspace(0,len(q)-1,350,dtype=int);q=q[pick];ix=ix[pick]
 xy=q[:,:2];centres=[]
 for _ in range(500):
  pair=xy[rng.choice(len(q),2,replace=False)];v=pair[1]-pair[0];length=np.linalg.norm(v)
  if length<.015 or length>2*radius:continue
  mid=pair.mean(axis=0);n=np.array([-v[1],v[0]])/length;h=np.sqrt(max(0,radius**2-length**2/4))
  for c in [mid+h*n,mid-h*n]:
   if abs(c[0]-seed[0])>(x_span if x_span is not None else .12 if role=='trunk' else .27) or abs(c[1]-seed[1])>.28:continue
   radial=xy-c;d=abs(np.linalg.norm(radial,axis=1)-radius);visible=np.sum(radial*(origin[:2]-xy),axis=1)>0;keep=(d<.015)&visible
   if keep.sum()<15:continue
   score=float(np.mean(np.minimum(d/.018,1)**2)+.12*np.mean(~visible))
   centres.append((score,c))
 if not centres:return None
 candidates=[]
 for _,centre in sorted(centres,key=lambda a:a[0])[:8]:
  v=np.r_[centre,0.,0.,radius,0.]
  for _ in range(3):
   c=v[:2]+(q[:,2,None]-.3)*v[2:4];radial=q[:,:2]-c;d=abs(np.linalg.norm(radial,axis=1)-(v[4]+v[5]*(q[:,2]-.3)));visible=np.sum(radial*(origin[:2]-q[:,:2]),axis=1)>0;keep=(d<.020)&visible
   if keep.sum()<15:break
   qq=q[keep]
   def res(v):
    c=v[:2]+(qq[:,2,None]-.3)*v[2:4];rr=np.linalg.norm(qq[:,:2]-c,axis=1)-(v[4]+v[5]*(qq[:,2]-.3))
    return np.r_[rr/(.006*np.sqrt(len(qq))), v[2:4]/.5, (v[0]-seed[0])/.012 if x_span is not None else 0., (v[4]-radius)/(radius_sigma if learn_radius else 1e-6),v[5]/(.025 if role=='trunk' else 1e-6)]
   fit=least_squares(res,v,bounds=(np.r_[seed-[x_bound,.31],-.5,-.5,max(.015,radius-2.5*radius_sigma),-.25],np.r_[seed+[x_bound,.31],.5,.5,radius+2.5*radius_sigma,.25]),loss='soft_l1',f_scale=1.,max_nfev=50);v=fit.x
  c=v[:2]+(q[:,2,None]-.3)*v[2:4];radial=q[:,:2]-c;d=abs(np.linalg.norm(radial,axis=1)-(v[4]+v[5]*(q[:,2]-.3)));visible=np.sum(radial*(origin[:2]-q[:,:2]),axis=1)>0;keep=(d<.015)&visible
  if keep.sum()<15 or keep.mean()<.42:continue
  angles=np.sort(np.mod(np.arctan2(radial[keep,1],radial[keep,0]),2*np.pi));span=2*np.pi-max(np.diff(np.r_[angles,angles[0]+2*np.pi]));rmse=np.sqrt(np.mean(d[keep]**2))
  if np.rad2deg(span)<65 or rmse>.010 or np.ptp(q[keep,2])<(.10 if training else .03):continue
  score=float(np.mean(np.minimum(d/.018,1)**2)+.12*np.mean(~visible))
  candidates.append((score,dict(centre=v[:2].tolist(),lean=v[2:4].tolist(),radius=float(v[4]),taper=float(v[5]),inliers=int(keep.sum()),points=len(q),rmse=float(rmse),arc=float(np.rad2deg(span)),indices=ix[keep].tolist())))
 return min(candidates,key=lambda x:x[0])[1] if candidates else None


def collect_landmarks(clouds,origins,labels,seeds,reference,original_clouds=None):
    original_clouds=clouds if original_clouds is None else original_clouds
    radii=np.asarray(reference['trunk']['circumferences_cm'])/(200*np.pi)
    radius=float(np.median(radii));sigma=max(.006,float(np.std(radii))+.003)
    rng=np.random.default_rng(2027);observations=[]
    for i,p in enumerate(clouds):
        z=np.asarray(original_clouds[i])[:,2]
        for key,seed in seeds:
            for training in [True,False]:
                eligible=((z>.15)&(z<.45)&((z<.24)|(z>.36)) if training else
                          (z>.26)&(z<.34))
                ix=np.flatnonzero(eligible)
                fit=detect_landmark(p[ix],origins[i],np.asarray(seed),radius,
                    training=training,rng=rng,learn_radius=True,radius_sigma=sigma)
                if fit and fit['arc']>=85:
                    fit['indices']=ix[fit['indices']].tolist()
                    observations.append(dict(scan=i,stem=int(key),role='trunk',
                                             training=training,**fit))
    keys=sorted(set((o['stem'],o['role']) for o in observations if o['training']))
    keys=[key for key in keys if len(set(labels[o['scan']] for o in observations
            if o['training'] and (o['stem'],o['role'])==key))==2]
    models=[]
    for key in keys:
        obs=[o for o in observations if o['training'] and (o['stem'],o['role'])==key]
        models.append(np.r_[np.median([o['centre'] for o in obs],axis=0),
            np.median([o['lean'] for o in obs],axis=0),radius,
            np.median([o['taper'] for o in obs])])
    return observations,keys,np.array(models),dict(radius_prior_m=radius,
        radius_sigma_m=max(.010,sigma),reference_usage='Representative population sizes; no exact tree mapping; loose-net and support ambiguity retained')
