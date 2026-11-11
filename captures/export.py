"""Export measured samples, complete rigid poses and capture-specific QC."""
from pathlib import Path
import json,zipfile
import numpy as np
import open3d as o3d
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares
from captures.orchard import transform,row_mask,save_json,sha256

DTYPE=np.dtype([('x','<f4'),('y','<f4'),('z','<f4'),('red','u1'),('green','u1'),('blue','u1'),('reflectivity','u1'),('scan_id','u1'),('classification','u1')])


def diagnostic_circle(points,seed,rng):
    """Independent robust slice fit; mixed branches remain plotted as outliers."""
    q=np.asarray(points);seed=np.asarray(seed);starts=[]
    for _ in range(600):
        p=q[rng.choice(len(q),3,replace=False)];A=2*(p[1:]-p[0]);b=np.sum(p[1:]**2,axis=1)-np.sum(p[0]**2)
        try:centre=np.linalg.solve(A,b)
        except np.linalg.LinAlgError:continue
        radius=np.linalg.norm(p[0]-centre)
        if not(.02<radius<.18) or max(abs(centre-seed))>.15:continue
        v=np.r_[centre,radius];d=abs(np.linalg.norm(q-centre,axis=1)-radius)
        starts.append((float(np.mean(np.minimum(d/.015,1)**2)),v))
    if not starts:raise ValueError('No supported circle')
    _,v=min(starts,key=lambda a:a[0]);lower=np.r_[seed-.15,.012];upper=np.r_[seed+.15,.18]
    for _ in range(3):
        inlier=abs(np.linalg.norm(q-v[:2],axis=1)-v[2])<.018
        if inlier.sum()<25:raise ValueError('Insufficient circle support')
        fit=least_squares(lambda x:np.linalg.norm(q[inlier]-x[:2],axis=1)-x[2],v,bounds=(lower,upper),loss='soft_l1',f_scale=.004);v=fit.x
    residual=np.linalg.norm(q-v[:2],axis=1)-v[2];inlier=abs(residual)<.018
    angles=np.sort(np.mod(np.arctan2(q[inlier,1]-v[1],q[inlier,0]-v[0]),2*np.pi))
    coverage=2*np.pi-max(np.diff(np.r_[angles,angles[0]+2*np.pi]))
    condition=float(np.linalg.cond(fit.jac.T@fit.jac));rmse=float(np.sqrt(np.mean(residual[inlier]**2)))
    reliable=bool(np.rad2deg(coverage)>100 and condition<150 and rmse<.012 and inlier.mean()>.5 and .015<v[2]<.175)
    return v,reliable,dict(radial_rmse_m=rmse,arc_deg=float(np.rad2deg(coverage)),condition=condition,inlier_fraction=float(inlier.mean()))


def write_ply(path,points,reflectivity,scan_ids,colors):
    data=np.empty(len(points),dtype=DTYPE)
    for k,name in enumerate(['x','y','z']):data[name]=points[:,k]
    for k,name in enumerate(['red','green','blue']):data[name]=colors[:,k]
    data['reflectivity']=reflectivity;data['scan_id']=scan_ids;data['classification']=np.where(points[:,2]<.12,2,5)
    header='ply\nformat binary_little_endian 1.0\ncomment Measured orchard returns; metres; local ground datum\ncomment Classification is a height band, not semantic ground truth\nelement vertex '+str(len(data))+'\n'
    header+=''.join('property '+('float' if name in ['x','y','z'] else 'uchar')+' '+name+'\n' for name in DTYPE.names)+'end_header\n'
    with Path(path).open('wb') as f:f.write(header.encode());data.tofile(f)
    return data


def plot_slices(clouds,scans,config,frame,path,title):
    seeds=[(c['id'],transform(np.r_[c['seed_m'],.3][None,:],frame)[0,:2]) for c in config['stem_candidates']]
    fig,axes=plt.subplots(len(seeds),3,figsize=(12,3.6*len(seeds)),squeeze=False);rng=np.random.default_rng(31);records=[]
    for row,(k,seed) in enumerate(seeds):
        for col,height in enumerate([.18,.28,.40]):
            ax=axes[row,col];fits=[];circle_lines=[];slice_records=[]
            for side,color in [('sideA','#147db3'),('sideB','#ed7b2b')]:
                arrays=[p[(abs(p[:,0]-seed[0])<.28)&(abs(p[:,1]-seed[1])<.34)&(abs(p[:,2]-height)<.025)] for p,s in zip(clouds,scans) if s['tag'].startswith(side+'_')]
                q=np.concatenate(arrays)[:,:2];ax.scatter((q[:,0]-seed[0])*100,(q[:,1]-seed[1])*100,s=1.2,c=color,alpha=.4)
                if len(q)>=40:
                    try:
                        fit,good,diagnostics=diagnostic_circle(q,seed,rng)
                        records.append(dict(candidate=k,height_m=height,side=side,centre_m=fit[:2].tolist(),radius_m=float(fit[2]),reliable=good,**diagnostics))
                        slice_records.append(records[-1]);fits.append((fit,good))
                        theta=np.linspace(0,2*np.pi,200);circle=(fit[:2]-seed)[None,:]+fit[2]*np.c_[np.cos(theta),np.sin(theta)]
                        circle_lines.extend(ax.plot(circle[:,0]*100,circle[:,1]*100,c=color,ls='--',lw=1.2));ax.plot(*(fit[:2]-seed)*100,'+',c=color,ms=10)
                    except (ValueError,np.linalg.LinAlgError):pass
            comparable=bool(len(fits)==2 and all(good for _,good in fits) and abs(fits[0][0][2]-fits[1][0][2])<.035 and np.linalg.norm(fits[0][0][:2]-fits[1][0][:2])<.08)
            for record in slice_records:record['opposite_side_centres_comparable']=comparable
            for line in circle_lines:line.set_linestyle('-' if comparable else '--')
            ax.set(xlim=(-28,28),ylim=(-34,34),title=f'Stem window {k} · {height:.2f} m',xlabel='Along from window centre (cm)',ylabel='Across (cm)');ax.set_aspect('equal');ax.grid(alpha=.2)
    fig.suptitle(title+'\nA blue, B orange; crosses are fitted centres; dashed circles are uncertain',fontsize=13);fig.tight_layout(rect=(0,0,1,.96));fig.savefig(path,dpi=150);plt.close(fig)
    save_json(Path(path).with_suffix('.json'),dict(method='Independent partial-arc circle diagnostics; no registration correction is applied here',slices=records))


def export_capture(workspace,config,scans,raw,poses,seed_poses,reports):
    root=Path(workspace);out=root/'final';out.mkdir(exist_ok=True);qc=out/'QC';qc.mkdir(exist_ok=True);contributions=out/'registered_contributions';contributions.mkdir(exist_ok=True)
    # Put the configured row axis along final X and its centre at Y=0.
    c=config['crop'];frame=np.eye(4);frame[:3,:3]=Rotation.from_euler('z',-np.arctan(c['centre_slope'])).as_matrix();frame[:3,3]=-frame[:3,:3]@np.array([0,c['centre_intercept_m'],0.])
    final=frame@poses;clouds=[];before=[];parts=[];refs=[];sources=[];entries=[];palette=(plt.get_cmap('tab20')(np.arange(len(scans))%20)[:,:3]*255).astype('u1')
    for i,(s,p,T) in enumerate(zip(scans,raw,poses)):
        d=np.load(root/'work/decoded'/(s['tag']+'.npz'));q=transform(p,T);keep=row_mask(q,config);q=transform(q[keep],frame);r=d['reflectivity'][keep];sid=np.full(len(q),i,dtype='u1');clouds.append(q)
        first=transform(p,seed_poses[i]);before.append(transform(first[row_mask(first,config)],frame))
        name=f'scan_{i:02d}_{s["tag"]}_registered.ply';write_ply(contributions/name,q,r,sid,np.tile(palette[i],(len(q),1)))
        rotation=np.eye(4);rotation[:3,:3]=d['rotation']
        source=next(row for row in reports['input_manifest']['files'] if row['path'].startswith(s['tag'].split('_')[0]+'/') and row['path'].endswith(s['tag'].split('_')[1]+'.pcap'))
        entry=dict(source_pcap=source['path'],source_pcap_sha256=source['sha256'],scan_id=i,scan=s['tag'],side=s['tag'].split('_')[0],filename=name,point_count=len(q),sensor_to_merged=(final[i]@rotation).tolist(),gravity_levelled_to_merged=final[i].tolist(),gps_latitude=s['lat'],gps_longitude=s['lon']);entries.append(entry)
        parts.append(q);refs.append(r);sources.append(sid)
    points=np.concatenate(parts);reflectivity=np.concatenate(refs);scan_ids=np.concatenate(sources)
    write_ply(out/'combined_by_recording.ply',points,reflectivity,scan_ids,palette[scan_ids])
    _,indices=np.unique(np.floor(points/.012).astype(np.int64),axis=0,return_index=True);points=points[indices];reflectivity=reflectivity[indices];scan_ids=scan_ids[indices]
    pc=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points));_,keep=pc.remove_radius_outlier(nb_points=5,radius=.06);points=points[keep];reflectivity=reflectivity[keep];scan_ids=scan_ids[keep]
    colours=(plt.get_cmap('turbo')(np.clip(points[:,2]/4.,0,1))[:,:3]*255).astype('u1');colours[points[:,2]<.12]=[110,125,136]
    records=write_ply(out/'final_registered.ply',points,reflectivity,scan_ids,colours)
    metadata=dict(dataset=config['dataset'],units='metres',coordinates='X along configured row axis, Y across row, Z up; local ground datum, not surveyed altitude',matrix_convention='Column vectors: [x,y,z,1]_merged = sensor_to_merged @ [x,y,z,1]_Ouster_XYZLut_sensor_frame; final PLYs already have these poses applied',frame=reports['initial']['frame'],registration_frame_to_final=frame.tolist(),crop_in_registration_frame=config['crop'],antenna_offset='Uncalibrated; RTK used as coarse prior, opposite-side corrections estimated from scene geometry',scans=entries)
    if reports.get('excluded_recordings'):metadata['excluded_recordings']=reports['excluded_recordings']
    save_json(out/'transformations.json',metadata);save_json(contributions/'scan_manifest.json',dict(dataset=config['dataset'],units='metres',total_points=sum(len(p) for p in clouds),scans=entries))
    # Check exported float coordinates and attributes against original measured returns.
    assert len(points)>100000 and np.isfinite(points).all()
    assert np.allclose(final[:,:3,:3].transpose(0,2,1)@final[:,:3,:3],np.eye(3),atol=1e-10)
    from processing.export_anchored import read_records,xyz
    for i,entry in enumerate(entries):
        _,saved=read_records(contributions/entry['filename']);np.testing.assert_allclose(xyz(saved),clouds[i],rtol=0,atol=1e-6)
        d=np.load(root/'work/decoded'/(scans[i]['tag']+'.npz'));q=transform(raw[i],poses[i]);mask=row_mask(q,config);np.testing.assert_array_equal(saved['reflectivity'],d['reflectivity'][mask])
        np.testing.assert_allclose(np.array(entry['sensor_to_merged']),final[i]@np.block([[d['rotation'],np.zeros((3,1))],[np.zeros((1,3)),np.ones((1,1))]]),rtol=0,atol=1e-12)
    reports.update(export=dict(merged_points=len(points),separate_scan_points=sum(len(p) for p in clouds),measurement_policy='First measured sample per 12mm cell, then radius-outlier removal; no averaging, interpolation or per-point warping',verification='Rigid matrices, original reflectivity, measured float coordinates and contribution counts verified',merged_ply_sha256=sha256(out/'final_registered.ply'),transformations_sha256=sha256(out/'transformations.json')))
    save_json(qc/'registration_quality.json',reports);save_json(qc/'capture_config.json',config)
    fig,axes=plt.subplots(2,1,figsize=(14,8),gridspec_kw={'height_ratios':[3,1.4]});q=points[::4];color=colours[::4]/255
    axes[0].scatter(q[:,0],q[:,2],c=color,s=.5,linewidths=0);axes[0].set(xlabel='Along row (m)',ylabel='Height (m)',title=f'{config["dataset"]} · final registered measured points')
    order=np.argsort(q[:,2]);axes[1].scatter(q[order,0],q[order,1],c=color[order],s=.5,linewidths=0);axes[1].set(xlabel='Along row (m)',ylabel='Across row (m)',title='Tree row and ground directly beneath it')
    for ax in axes:ax.set_aspect('equal');ax.grid(alpha=.15)
    fig.suptitle(f'{len(scans)} stationary recordings · {len(points):,} points · metres');fig.tight_layout();fig.savefig(qc/'final_pointcloud.png',dpi=160);plt.close(fig)
    fig,ax=plt.subplots(figsize=(7,8));ax.scatter(q[:,1],q[:,2],c=color,s=.6,linewidths=0)
    ax.set(xlabel='Across row (m)',ylabel='Height (m)',title=config['dataset']+' · final across-row profile');ax.set_aspect('equal');ax.grid(alpha=.15)
    fig.tight_layout();fig.savefig(qc/'across_row_profile.png',dpi=160);plt.close(fig)
    plot_slices(before,scans,config,frame,qc/'trunk_slices_before.png','Before ground and bounded refinement')
    plot_slices(clouds,scans,config,frame,qc/'trunk_slices_final.png','Final alignment: independent low-height circle diagnostics')
    fig,axes=plt.subplots(2,1,figsize=(14,6),sharex=True)
    for ax,Ts,title in [(axes[0],seed_poses,'Before relative ground refinement'),(axes[1],poses,'Final aligned ground')]:
        for i,(p,T) in enumerate(zip(raw,Ts)):
            q=transform(p,T);q=q[row_mask(q,config,ground=True)];q=transform(q,frame);ax.scatter(q[::8,0],q[::8,2],s=.4,c=[palette[i]/255],alpha=.4,linewidths=0)
        ax.set(title=title,ylabel='Ground height (m)',ylim=(-.15,.15));ax.grid(alpha=.2)
    axes[-1].set_xlabel('Along row (m)');fig.tight_layout();fig.savefig(qc/'ground_alignment.png',dpi=160);plt.close(fig)
    (qc/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>'+config['dataset']+' QC</title><style>body{font:16px system-ui;max-width:1200px;margin:2rem auto}img{max-width:100%}</style><h1>'+config['dataset']+' quality checks</h1>'+''.join('<h2>'+p.stem.replace('_',' ')+'</h2><img src="'+p.name+'">' for p in sorted(qc.glob('*.png')))+'<p><a href="registration_quality.json">Full registration report</a></p>')
    (out/'README.txt').write_text(config['dataset']+' FINAL POINT CLOUD\n\nCoordinates are metres; the PLY is already registered. Do not apply transformations.json again.\nOnly the tree-row strip and its underlying ground are retained. Supports and attached clutter may remain.\nNo mesh has been generated for this capture. QC/index.html contains figures and conditional quality checks.\n')
    np.save(out/'final_levelled_to_merged_poses.npy',final)
    return reports
