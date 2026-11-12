"""Central observed-tree plane from height-matched raw east/west profiles.

Uses all selected tree returns, including both halves of each visible tree.
The sides, recordings and heights are balanced. No arm angle, symmetry,
verticality or ground-normal target enters the fit. Independent horizontal
origins change the intercept, not the estimated inclination.
"""
import numpy as np
from scipy.optimize import least_squares


def central_tree_profile(data,report,band=(.3,3.),statistic='mean',wood_only=False,near_only=False):
    p=np.asarray(data['xyz']);row=report['row'];c=np.array(row['centre']);h=(p-c)@row['along'];u=(p-c)@row['outward'];z=p[:,2]-c[2]
    keep=(h>row['extent'][0])&(h<row['extent'][1])&(abs(u)<1.7)&(z>band[0])&(z<band[1])
    if near_only:keep&=u>.03
    if wood_only:
        if 'wood_score' not in data:raise ValueError('No optional wood scores supplied')
        keep&=data['wood_score']>.65
    q=np.c_[h[keep],u[keep],z[keep]]
    if len(q)<100:raise ValueError('Insufficient raw tree returns')
    _,ix=np.unique(np.floor(q/.025+1e-9).astype(np.int64),axis=0,return_index=True);q=q[ix]
    cells=np.c_[np.floor(q[:,0]/.45+1e-9),np.floor(q[:,2]/.12+1e-9)].astype(int);keys,inv=np.unique(cells,axis=0,return_inverse=True);samples=[]
    reduce=np.mean if statistic=='mean' else np.median if statistic=='median' else None
    if reduce is None:raise ValueError('Unknown centre statistic')
    for j,key in enumerate(keys):
        v=q[inv==j]
        if len(v)<8:continue
        samples.append([float(np.mean(v[:,0])),float(reduce(v[:,1])),float(np.mean(v[:,2])),int(key[1])])
    samples=np.array(samples)
    if len(samples)<20:raise ValueError('Insufficient occupied row/height cells')
    A=np.c_[samples[:,0],samples[:,2],np.ones(len(samples))];y=samples[:,1]
    f=least_squares(lambda b:A@b-y,np.linalg.lstsq(A,y,rcond=None)[0],loss='soft_l1',f_scale=.07)
    profile=[]
    for key in np.unique(samples[:,3]):
        v=samples[samples[:,3]==key]
        if len(v)<2:continue
        # Evaluate the centre at a common along-row location; the row intercept
        # remains arbitrary and free. This removes changing along-row coverage.
        profile.append([float(np.mean(v[:,2])),float(reduce(v[:,1]-f.x[0]*v[:,0]))])
    profile=np.array(profile)
    if len(profile)<8 or np.ptp(profile[:,0])<.9:raise ValueError('Insufficient height coverage')
    ng=np.array(report['ground']['normal']);a=np.array(row['along']);v=np.array(row['outward'])
    return dict(height_band_m=list(band),statistic=statistic,wood_only=wood_only,near_only=near_only,
                profile=profile.tolist(),along_coefficient=float(f.x[0]),plane_coefficients=f.x.tolist(),
                ground_normal_local_row=[float(ng@a),float(ng@v),float(ng[2])],cells=len(samples),returns=int(keep.sum()))


def central_plane(profiles,band=(.3,3.)):
    """Fit a free central plane from equal east/west, height-matched averages.

    profiles: records with geographic_side, scan and central_profile. Each
    profile uses its own raw row/ground origin and an outward-facing across axis.
    Both horizontal axes reverse on the west track to share one row/east/up basis.
    No cross-side fitted pose or point correspondence is required for slopes.
    """
    grouped={side:[r for r in profiles if r['geographic_side']==side] for side in ['east','west']}
    if any(not v for v in grouped.values()):raise ValueError('Both geographic sides required')
    lo=max(r['central_profile']['profile'][0][0] for r in profiles);hi=min(r['central_profile']['profile'][-1][0] for r in profiles)
    grid=np.arange(np.floor(band[0]/.12)-1,np.ceil(band[1]/.12)+2)*.12+.06
    grid=grid[(grid>=max(lo,band[0]+.06)-1e-9)&(grid<=min(hi,band[1]-.06)+1e-9)]
    if len(grid)<8 or np.ptp(grid)<.9:raise ValueError('No sufficiently long common observed height interval')
    track={};all_profiles=[];ground=[];along=[]
    for side,rows in grouped.items():
        sign=1 if side=='east' else -1;values=[];gg=[];aa=[]
        for r in rows:
            a=r['central_profile'];p=np.array(a['profile']);curve=sign*np.interp(grid,p[:,0],p[:,1]);values.append(curve)
            n=np.array(a['ground_normal_local_row']);n[:2]*=sign;gg.append(n);aa.append(a['along_coefficient'])
            all_profiles.append(dict(scan=r['scan'],geographic_side=side,east_positive_centres_m=curve.tolist()))
        track[side]=np.mean(values,axis=0);ground.append(np.mean(gg,axis=0));along.append(np.mean(aa))
    middle=.5*(track['east']+track['west'])
    f=least_squares(lambda b:b[0]*grid+b[1]-middle,np.polyfit(grid,middle,1),loss='soft_l1',f_scale=.03)
    bh=float(np.mean(along));bz=float(f.x[0]);normal=np.array([-bh,1.,-bz]);normal/=np.linalg.norm(normal)
    ng=np.mean(ground,axis=0);ng/=np.linalg.norm(ng)
    angle=float(np.rad2deg(np.arccos(np.clip(abs(normal@ng),0,1))))
    return dict(definition='Plane through the height-matched midpoint of equally weighted east/west mean observed-tree profiles; all selected tree returns, not individual V-arm angles',
        height_band_requested_m=list(band),common_height_range_m=[float(grid[0]),float(grid[-1])],heights_m=grid.tolist(),
        source_counts={side:len(rows) for side,rows in grouped.items()},east_profile_m=track['east'].tolist(),west_profile_m=track['west'].tolist(),central_profile_m=middle.tolist(),
        central_plane_normal_row_east_up=normal.tolist(),mean_ground_normal_row_east_up=ng.tolist(),
        along_coefficient=bh,eastward_height_coefficient=bz,intercept_m=float(f.x[1]),
        central_plane_to_ground_deg=angle,deviation_from_perpendicular_deg=90.-angle,
        central_plane_tilt_from_imu_vertical_deg=float(np.rad2deg(np.arctan(bz/np.sqrt(1+bh*bh)))),
        mean_ground_tilt_from_horizontal_deg=float(np.rad2deg(np.arccos(np.clip(ng[2],-1,1)))),
        central_fit_rmse_m=float(np.sqrt(np.mean(f.fun**2))),per_source_profiles=all_profiles,
        limits='Statistical observed-tree centre, not a surveyed trunk/symmetry plane. Occlusion and foliage/support visibility can bias centres. Origins are independent raw low-row/ground references; absolute cross-side positions are not registered.')
