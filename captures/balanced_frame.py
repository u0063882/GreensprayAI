"""Ground-first display frame with a weak observed-envelope upward preference.

One shared SE(3) transform; no per-point flattening or tree-form template.
The envelope is an orientation convention, not surveyed biological vertical.
"""
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

DEFAULTS=dict(ground_weight=4.,centre_weight=1.,ground_half_widths_m=[.35,.50,.70],
              envelope_quantiles=[.10,.90],height_range_m=[.55,2.85],height_bin_m=.15,
              maximum_rotation_deg=12.)


def sample(p,spacing):
    _,ix=np.unique(np.floor(p/spacing).astype(np.int64),axis=0,return_index=True)
    return np.asarray(p)[ix]


def soil_reference(points,seeds,half_width):
    p=sample(np.asarray(points),.025);xy=np.array([s[1] for s in seeds]);a,c=np.polyfit(xy[:,0],xy[:,1],1)
    distance=(p[:,1]-a*p[:,0]-c)/np.hypot(a,1)
    q=p[(abs(distance)<half_width)&(p[:,2]>-.18)&(p[:,2]<.18)]
    cells=np.floor(q[:,:2]/.12).astype(int);_,inv=np.unique(cells,axis=0,return_inverse=True);support=[]
    for k in np.unique(inv):
        v=q[inv==k]
        if len(v)<4:continue
        # Keep an actual measured return near the lower quartile in each XY cell.
        # Every occupied cell has equal weight; dense grass does not dominate.
        target=np.quantile(v[:,2],.25);support.append(v[np.argmin(abs(v[:,2]-target))])
    q=np.asarray(support)
    if len(q)<60 or np.ptp(q[:,0])<2. or np.ptp(q[:,1])<.3:raise ValueError('Insufficient spatial soil support')
    pivot=np.median(q[:,:2],axis=0);A=np.c_[q[:,:2]-pivot,np.ones(len(q))]
    if np.linalg.cond(A)>70:raise ValueError('Soil support nearly collinear')
    fit=least_squares(lambda v:A@v-q[:,2],np.linalg.lstsq(A,q[:,2],rcond=None)[0],loss='soft_l1',f_scale=.012)
    beta=fit.x;coef=np.r_[beta[:2],beta[2]-pivot@beta[:2]];n=np.r_[-coef[:2],1.];n/=np.linalg.norm(n)
    return dict(coefficients=coef.tolist(),normal=n.tolist(),cells=len(q),half_width_m=half_width,rmse_m=float(np.sqrt(np.mean(fit.fun**2))),measured_support=q.tolist())


def envelope_reference(points,seeds,settings):
    p=sample(np.asarray(points),.04);xy=np.array([s[1] for s in seeds]);order=np.argsort(xy[:,0]);xy=xy[order];windows=[]
    for i,seed in enumerate(xy):
        lo=(xy[i-1,0]+seed[0])/2 if i else seed[0]-.6
        hi=(xy[i+1,0]+seed[0])/2 if i+1<len(xy) else seed[0]+.6
        path=[]
        for z in np.arange(*settings['height_range_m'],settings['height_bin_m']):
            q=p[(p[:,0]>lo)&(p[:,0]<hi)&(p[:,2]>z)&(p[:,2]<z+settings['height_bin_m'])]
            if len(q)<15:continue
            low,high=np.quantile(q[:,1],settings['envelope_quantiles'])
            path.append([float(q[:,2].mean()),float((low+high)/2),float(low),float(high)])
        path=np.asarray(path)
        if len(path)<9 or np.ptp(path[:,0])<1.2:continue
        A=np.c_[path[:,0],np.ones(len(path))];fit=least_squares(lambda v:A@v-path[:,1],np.linalg.lstsq(A,path[:,1],rcond=None)[0],loss='soft_l1',f_scale=.04)
        windows.append(dict(along_window_m=[lo,hi],slope=float(fit.x[0]),lean_deg=float(np.degrees(np.arctan(fit.x[0]))),intercept_m=float(fit.x[1]),height_midpoint_low_high=path.tolist()))
    if len(windows)<3:raise ValueError('Fewer than three observed envelope windows')
    slopes=np.array([w['slope'] for w in windows]);b=float(np.median(slopes));axis=np.array([0.,b,1.]);axis/=np.linalg.norm(axis)
    return dict(axis=axis.tolist(),lean_deg=float(np.degrees(np.arctan(b))),windows=windows,window_lean_range_deg=[float(min(w['lean_deg'] for w in windows)),float(max(w['lean_deg'] for w in windows))],caution='Robust outer-quantile midpoint of the combined observed envelope; visibility-sensitive orientation preference, not a verified V centre or surveyed upright direction.')


def estimate_balanced_frame(points,seeds,settings=None):
    opt={**DEFAULTS,**(settings or {})}
    soils=[soil_reference(points,seeds,w) for w in opt['ground_half_widths_m']]
    coef=np.median([g['coefficients'] for g in soils],axis=0);ng=np.r_[-coef[:2],1.];ng/=np.linalg.norm(ng)
    envelope=envelope_reference(points,seeds,opt);up=np.array(envelope['axis'])
    desired_up=opt['ground_weight']*ng+opt['centre_weight']*up;desired_up/=np.linalg.norm(desired_up)
    z=np.array([0.,0.,1.]);v=np.cross(desired_up,z);angle=np.arctan2(np.linalg.norm(v),np.dot(desired_up,z))
    if np.degrees(angle)>opt['maximum_rotation_deg']:raise ValueError('Requested frame change exceeds reviewed rotation bound')
    R=Rotation.from_rotvec(v/max(np.linalg.norm(v),1e-12)*angle).as_matrix()
    xy=np.array([s[1] for s in seeds]);a,c=np.polyfit(xy[:,0],xy[:,1],1);row=np.array([1.,a,0.]);h=R@row
    R=Rotation.from_euler('z',-np.arctan2(h[1],h[0])).as_matrix()@R
    x0=float(xy[:,0].min());y0=float(a*x0+c);pivot=np.array([x0,y0,coef[0]*x0+coef[1]*y0+coef[2]])
    T=np.eye(4);T[:3,:3]=R;T[:3,3]=-R@pivot
    after_ground=R@ng;after_up=R@up
    report=dict(settings=opt,ground_width_checks=soils,representative_ground_coefficients=coef.tolist(),ground_normal_before=ng.tolist(),envelope=envelope,
        output_frame=T.tolist(),rotation_deg=float(np.degrees(angle)),ground_tilt_before_deg=float(np.degrees(np.arccos(ng[2]))),ground_tilt_after_deg=float(np.degrees(np.arccos(np.clip(after_ground[2],-1,1)))),
        observed_centre_tilt_before_deg=float(np.degrees(np.arccos(up[2]))),observed_centre_tilt_after_deg=float(np.degrees(np.arccos(np.clip(after_up[2],-1,1)))),
        ground_normal_after=after_ground.tolist(),observed_centre_axis_after=after_up.tolist(),imu_vertical_in_output=(R@z).tolist(),pivot_input_m=pivot.tolist(),
        policy='User-requested ground-first local coordinate convention; 4:1 weight of soil normal to observed envelope direction. Not an Earth-vertical calibration or proof of biological symmetry. Relative scan geometry is untouched by this common transform.',rigid_only=True)
    return T,report
