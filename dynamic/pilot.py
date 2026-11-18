"""Bounded Ouster SDK moving-lidar pilot with withheld return diagnostics.

Only the already trimmed PCAP is opened. Existing stationary results are not
read, registered, exported, meshed or replaced by this module.
"""
from pathlib import Path
import argparse,csv,json,os,time,sys
os.environ.setdefault('OMP_NUM_THREADS','2');os.environ.setdefault('OPENBLAS_NUM_THREADS','2');os.environ.setdefault('MPLCONFIGDIR','/tmp/greenspray-dynamic-mpl')
ROOT=Path(__file__).resolve().parent.parent;sys.path.insert(0,str(ROOT))
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from ouster.sdk import core,pcap,mapping,osf,open_source
from captures.leafwood import classify_leafwood,write_records


def holdout_mask(h,w):
    row,col=np.indices((h,w));return (row+3*col)%5==0


def measured_indices(p,spacing):
    _,ix=np.unique(np.floor(p/spacing).astype(np.int64),axis=0,return_index=True);return ix


def gravity_pose(acceleration):
    a=np.asarray(acceleration,dtype=float)
    if len(a)<3:raise ValueError('Insufficient IMU observations')
    up=np.median(a,axis=0);up/=np.linalg.norm(up);forward=np.array([1.,0.,0.]);forward-=up*np.dot(forward,up)
    if np.linalg.norm(forward)<.1:forward=np.array([0.,1.,0.]);forward-=up*np.dot(forward,up)
    forward/=np.linalg.norm(forward);R=np.array([forward,np.cross(up,forward),up]);T=np.eye(4);T[:3,:3]=R;return T


def packet_audit(path,info,first_capture_unix):
    fmt=core.PacketFormat(info);source=pcap.PcapPacketSource(str(path));acc=[];gyro=[];counts=dict(lidar=0,imu=0)
    try:
        for _,packet in source:
            if isinstance(packet,core.ImuPacket):
                counts['imu']+=1
                if packet.host_timestamp*1e-9<first_capture_unix+1.:
                    acc.append([fmt.imu_la_x(packet.buf),fmt.imu_la_y(packet.buf),fmt.imu_la_z(packet.buf)]);gyro.append([fmt.imu_av_x(packet.buf),fmt.imu_av_y(packet.buf),fmt.imu_av_z(packet.buf)])
            elif isinstance(packet,core.LidarPacket):counts['lidar']+=1
    finally:source.close()
    acc=np.asarray(acc,dtype=float).reshape(-1,3)@np.array(info.imu_to_sensor_transform)[:3,:3].T@np.array(info.sensor_to_body)[:3,:3].T
    try:T=gravity_pose(acc);status='first_second_median_legacy_IMU_acceleration'
    except ValueError:T=np.eye(4);status='unavailable_sensor_axes_retained'
    return T,dict(**counts,initial_gravity_method=status,initial_gravity_samples=len(acc),initial_acceleration_median_body=np.median(acc,axis=0).tolist() if len(acc) else None,initial_acceleration_MAD_body=np.median(abs(acc-np.median(acc,axis=0)),axis=0).tolist() if len(acc) else None,initial_gyro_norm_median_deg_s=float(np.median(np.linalg.norm(gyro,axis=1))) if len(gyro) else None,initial_body_to_map=T.tolist(),caution='Accelerometer median under vehicle motion is approximate, not surveyed Earth vertical or a fitted tree-up correction.')


def navigation(h5path,host_times,positions):
    if not h5path.exists():return dict(available=False),None
    import h5py
    with h5py.File(h5path) as h:t=np.asarray(h['time']).ravel();lat=np.asarray(h['lat']).ravel();lon=np.asarray(h['lon']).ravel()
    valid=np.isfinite(t)&np.isfinite(lat)&np.isfinite(lon);t,lat,lon=t[valid],lat[valid],lon[valid];order=np.argsort(t);t,lat,lon=t[order],lat[order],lon[order]
    keep=(host_times>=t.min())&(host_times<=t.max())
    if keep.sum()<5:return dict(available=True,matched_frames=int(keep.sum()),time_alignment='No sufficient overlap between PCAP host Unix timestamps and GPS log; no invented offset'),None
    ll=np.c_[np.interp(host_times[keep],t,lat),np.interp(host_times[keep],t,lon)];phi=np.deg2rad(ll[:,0].mean());north_factor=111132.92-559.82*np.cos(2*phi)+1.175*np.cos(4*phi);east_factor=111412.84*np.cos(phi)-93.5*np.cos(3*phi)
    gps=(ll-ll[0])*[north_factor,east_factor];gps=gps[:,::-1];p=positions[keep,:2];x=p-p.mean(0);y=gps-gps.mean(0);u,_,vt=np.linalg.svd(x.T@y);R=vt.T@u.T
    if np.linalg.det(R)<0:vt[-1]*=-1;R=vt.T@u.T
    fit=(p-p.mean(0))@R.T+gps.mean(0);error=np.linalg.norm(fit-gps,axis=1);gps_path=float(np.linalg.norm(np.diff(gps,axis=0),axis=1).sum());slam_path=float(np.linalg.norm(np.diff(p,axis=0),axis=1).sum())
    return dict(available=True,matched_frames=int(keep.sum()),time_alignment='PCAP host Unix time interpolated against H5 time; hardware synchronization and antenna lever arm not calibrated',gps_path_m=gps_path,slam_path_over_same_times_m=slam_path,slam_to_gps_path_ratio=slam_path/gps_path if gps_path>.1 else None,rigid_2D_fit_rmse_m=float(np.sqrt(np.mean(error**2))),gps_endpoint_displacement_m=float(np.linalg.norm(gps[-1]-gps[0])),no_scale_fit=True,absolute_GPS_coordinates_omitted=True),dict(time=host_times[keep],gps=gps,slam_fit=fit)


def consistency(features):
    rows=[]
    for i in range(5,len(features)):
        a,b=features[i],features[i-5]
        if min(len(a['before']),len(b['before']))<30:continue
        item=dict(frame_index=a['frame_index'],time_s=a['time_s'])
        for state in ['before','after']:
            d=cKDTree(b[state]).query(a[state],workers=2)[0];item[state+'_capped_rms_m']=float(np.sqrt(np.mean(np.minimum(d,.10)**2)));item[state+'_coverage_10cm']=float(np.mean(d<.10))
        rows.append(item)
    aggregate={}
    for state in ['before','after']:
        aggregate[state]=dict(mean_capped_RMS_m=float(np.mean([x[state+'_capped_rms_m'] for x in rows])),mean_coverage_10cm=float(np.mean([x[state+'_coverage_10cm'] for x in rows]))) if rows else None
    return dict(**aggregate,pairs=len(rows),per_pair=rows,policy='20% fixed pixel returns withheld from SDK training in every frame, including second returns at those pixels. Identical held-out queries matched against held-out points five processed scans earlier. 10 cm capped nearest-neighbour cost; density/occlusion-sensitive internal diagnostic, not survey accuracy.')


def ground_check(points,features):
    import open3d as o3d
    p=points[measured_indices(points,.04)];p=p[p[:,2]<np.quantile(p[:,2],.30)]
    if len(p)<200:return dict(supported=False)
    o3d.utility.random.seed(144);pc=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(p));model,ids=pc.segment_plane(.04,3,1200);n=np.array(model[:3]);offset=float(model[3])
    if n[2]<0:n=-n;offset=-offset
    if n[2]<.8:return dict(supported=False,reason='No near-up dominant ground plane')
    p=p[ids];rms=float(np.sqrt(np.mean((p@n+offset)**2)));quarters=[]
    for group in np.array_split(np.arange(len(features)),4):
        q=np.concatenate([features[i]['after'] for i in group]);d=q@n+offset;keep=abs(d)<.10
        quarters.append(dict(points=int(keep.sum()),median_signed_distance_m=float(np.median(d[keep])) if keep.any() else None,RMS_m=float(np.sqrt(np.mean(d[keep]**2))) if keep.any() else None))
    return dict(supported=True,normal=n.tolist(),offset=offset,RMS_m=rms,tilt_from_initial_IMU_up_deg=float(np.degrees(np.arccos(np.clip(n[2],-1,1)))),time_quartiles=quarters,caution='Dominant measured low surface only; a real orchard floor can slope or have grass ridges. This fit is diagnostic and never flattens points or changes poses.')


def run(pcap_path,output,voxel=.15,max_range=25.):
    started=time.monotonic();pcap_path=Path(pcap_path);output=Path(output);output.mkdir(parents=True,exist_ok=True);prep=json.loads((pcap_path.parent/'preparation.json').read_text());source=open_source(str(pcap_path));info=source.sensor_info[0];T0,packets=packet_audit(pcap_path,info,prep['capture_first_unix']);config=mapping.SlamConfig.create('lio');config.deskew_method='constant_velocity';config.voxel_size=voxel;config.min_range=1.;config.max_range=max_range;config.max_iterations=50;config.initial_pose=T0;engine=mapping.SlamEngine.create([info],config);lut=core.XYZLut(info,True);writer=osf.Writer(str(output/'pilot15s.osf'),[info]);model=json.loads((ROOT/'dynamic/shared_leafwood_model.json').read_text());window=info.format.column_window;expected=(window[1]-window[0])%info.format.columns_per_frame+1
    poses=[];times=[];host=[];frame_ids=[];column_times=[];statuses=[];features=[];chunks=[];dropped=[];flags=[];t0=None
    dtype=np.dtype([('x','<f4'),('y','<f4'),('z','<f4'),('red','u1'),('green','u1'),('blue','u1'),('reflectivity','u1'),('intensity','<u2'),('signal','<u2'),('near_ir','<u2'),('scan_id','<u4'),('frame_id','<u4'),('pixel_index','<u4'),('time_offset_s','<f8'),('leafwood_class','u1'),('wood_score','<f4')])
    try:
        for source_index,fs in enumerate(source):
            f=fs[0]
            if f is None:continue
            coverage=float(np.sum((f.status&1)>0)/expected)
            if coverage<.98:dropped.append(dict(frame_id=int(f.frame_id),coverage=coverage));continue
            index=len(poses);columns=np.flatnonzero((f.status&1)>0);ts=np.asarray(f.timestamp).copy();first=int(ts[columns[0]]);centre=int(np.median(ts[columns]));host_ts=float(np.median(np.asarray(f.packet_timestamp)[np.asarray(f.packet_timestamp)>0])*1e-9)
            if t0 is None:t0=first
            elapsed=(centre-t0)*1e-9;raw_ranges=f.field('RANGE').copy();other=f.field('RANGE2').copy() if f.has_field('RANGE2') else None;held=holdout_mask(f.h,f.w);body=lut(f);valid=(raw_ranges>=1000)&(raw_ranges<max_range*1000)&((f.status[None,:]&1)>0)
            f.field('RANGE')[held]=0
            if other is not None:f.field('RANGE2')[held]=0
            try:updated=engine.update(fs);f=updated[0]
            finally:
                fs[0].field('RANGE')[:]=raw_ranges
                if other is not None:fs[0].field('RANGE2')[:]=other
            C=np.asarray(f.body_to_world).copy();middle=C[columns[len(columns)//2]]
            if not np.isfinite(C).all():flags.append('Non-finite SLAM pose');break
            if poses:
                prior=poses[-1][np.flatnonzero(statuses[-1]&1)[len(np.flatnonzero(statuses[-1]&1))//2]];delta=np.linalg.inv(prior)@middle;step=np.linalg.norm(delta[:3,3]);rotation=np.degrees(np.linalg.norm(Rotation.from_matrix(delta[:3,:3]).as_rotvec()))
                if step>1.0 or rotation>25:flags.append(f'Implausible per-frame jump at processed frame {index}: {step:.3f} m / {rotation:.2f} deg');break
            world=core.dewarp(body,C);poses.append(C);times.append(elapsed);host.append(host_ts);frame_ids.append(int(f.frame_id));column_times.append(ts);statuses.append(np.asarray(f.status).copy());writer.save(fs)
            # Export true measured first returns. Spatial sampling retains a real
            # return and its original raw channels/IDs; no centroid is invented.
            ix=np.flatnonzero(valid);q=world.reshape(-1,3)[ix];take=measured_indices(q,.04);ix=ix[take];q=q[take];d=np.zeros(len(ix),dtype=dtype)
            for j,key in enumerate(['x','y','z']):d[key]=q[:,j]
            d['signal']=f.field('SIGNAL').ravel()[ix];d['intensity']=d['signal'];d['near_ir']=f.field('NEAR_IR').ravel()[ix];d['reflectivity']=f.field('REFLECTIVITY').ravel()[ix];d['scan_id']=index;d['frame_id']=f.frame_id;d['pixel_index']=ix;d['time_offset_s']=(ts[ix%f.w].astype(float)-t0)*1e-9;classes=classify_leafwood(d['signal'],d['near_ir'],model);d['leafwood_class']=classes['leafwood_class'];d['wood_score']=classes['wood_score']
            for j,key in enumerate(['red','green','blue']):d[key]=classes['rgb'][:,j]
            chunks.append(d)
            ix=np.flatnonzero(valid&held&(raw_ranges<8000));body_flat=body.reshape(-1,3)[ix];sample=measured_indices(body_flat,.06)
            if len(sample)>1000:sample=sample[np.linspace(0,len(sample)-1,1000).astype(int)]
            ix=ix[sample];before=body.reshape(-1,3)[ix]@T0[:3,:3].T+T0[:3,3];after=world.reshape(-1,3)[ix];features.append(dict(frame_index=index,time_s=elapsed,before=before,after=after))
            if index%10==0:print('SLAM',index,round(elapsed,3),'s; pose',np.round(middle[:3,3],3).tolist(),flush=True)
    finally:writer.close();source.close()
    if len(poses)<3:raise RuntimeError('Fewer than three complete processed frames')
    p=np.concatenate(chunks);p=p[measured_indices(np.c_[p['x'],p['y'],p['z']],.025)];write_records(output/'pilot_registered.ply',p,comment='15-second moving pilot; Ouster per-column motion compensation; provisional shared leaf/wood display; scan_id=processed frame index')
    np.savez_compressed(output/'per_column_poses.npz',body_to_world=np.array(poses),column_timestamp_ns=np.array(column_times),status=np.array(statuses),frame_id=np.array(frame_ids),host_unix=np.array(host),relative_seconds=np.array(times))
    centres=np.array([c[np.flatnonzero(s&1)[len(np.flatnonzero(s&1))//2]] for c,s in zip(poses,statuses)]);positions=centres[:,:3,3];dt=np.diff(times);steps=np.linalg.norm(np.diff(positions,axis=0),axis=1);angles=np.degrees(np.linalg.norm(Rotation.from_matrix(np.einsum('nij,njk->nik',centres[:-1,:3,:3].transpose(0,2,1),centres[1:,:3,:3])).as_rotvec(),axis=1));nav,nav_plot=navigation(pcap_path.with_suffix('.h5'),np.array(host),positions);held=consistency(features);ground=ground_check(np.c_[p['x'],p['y'],p['z']],features)
    if np.any(dt<=0):flags.append('Non-increasing lidar frame timestamps')
    if nav.get('slam_to_gps_path_ratio') is not None and nav.get('gps_endpoint_displacement_m',0)>1 and not .5<nav['slam_to_gps_path_ratio']<2:flags.append('Large disagreement with GPS travelled distance; timing/lever arm also need review')
    if held['after'] and held['before'] and held['after']['mean_capped_RMS_m']>held['before']['mean_capped_RMS_m']*1.05:flags.append('Withheld return consistency worsens relative to the unregistered baseline')
    moving=bool(np.linalg.norm(positions[-1]-positions[0])>.5)
    recommendation='review_flags_before_full_run' if flags else 'plausible_pilot_continue_with_QC' if moving else 'insufficient_motion_to_validate_moving_SLAM'
    report=dict(status=recommendation,flags=flags,scope='First 15 seconds only; no full-recording mapping, mesh, ground flattening or tree symmetry constraints',source=prep,sensor=dict(model='OS-0-128',firmware='3.1.0',fps=10,FOV_deg=180,expected_columns=expected,columns_per_frame=info.format.columns_per_frame),SDK_version=__import__('importlib.metadata',fromlist=['version']).version('ouster-sdk'),SLAM=dict(backend='Ouster SDK LIOSlamEngine, same engine as ouster-cli slam',deskew_method='constant_velocity',internal_voxel_m=voxel,min_range_m=1.,max_range_m=max_range,max_iterations=50,withheld_pixel_fraction=.20,second_returns_at_withheld_pixels_also_removed=True),packets=packets,processed_frames=len(poses),dropped_incomplete_frames=dropped,processed_time_span_s=float(times[-1]-times[0]),retained_first_return_points=len(p),trajectory=dict(path_length_m=float(steps.sum()),endpoint_displacement_m=float(np.linalg.norm(positions[-1]-positions[0])),maximum_frame_step_m=float(steps.max()),maximum_frame_rotation_deg=float(angles.max()),maximum_speed_m_s=float(np.max(steps/np.maximum(dt,1e-9))),vertical_range_m=float(np.ptp(positions[:,2]))),GNSS_diagnostic=nav,held_out_consistency=held,ground_diagnostic=ground,elapsed_compute_s=time.monotonic()-started,export=dict(source_return='first return only; dual-return raw channels retained in processed OSF',sample_policy='One real return per 4 cm frame voxel, then one per 2.5 cm map voxel; no averaged/invented points',raw_channels=['signal','intensity','near_ir','reflectivity','frame_id','pixel_index'],time_offset_reference_lidar_ns=t0,RGB='Existing shared stationary leaf/wood model used only for provisional display; different acquisition/illumination not revalidated; never used by this SLAM fit'),limitations=['Internal diagnostics only, no surveyed accuracy guarantee.','GNSS uses uncalibrated capture-host timing and antenna lever arm; no GNSS constraints fitted.','No loop closure or pose graph optimization in this first-pass pilot.','180-degree view and repeated orchard geometry can cause drift, and foliage can move.'])
    with (output/'trajectory.csv').open('w',newline='') as f:
        w=csv.writer(f,lineterminator='\n');w.writerow(['frame_index','frame_id','time_s','capture_host_unix','x_m','y_m','z_m','qx','qy','qz','qw'])
        for i,(C,t,h,fid) in enumerate(zip(centres,times,host,frame_ids)):w.writerow([i,fid,t,h,*C[:3,3],*Rotation.from_matrix(C[:3,:3]).as_quat()])
    plot(output,p,np.array(times),positions,nav_plot,report)
    # Small preview arrays enable remote review without sending raw scans to
    # the language model or relying on a large binary transfer to this workspace.
    ix=np.linspace(0,len(p)-1,min(2500,len(p))).astype(int);report['preview_sample']=dict(xyz=np.round(np.c_[p['x'],p['y'],p['z']][ix],4).tolist(),rgb=np.c_[p['red'],p['green'],p['blue']][ix].tolist());report['preview_trajectory']=np.round(positions,5).tolist();report['preview_time_s']=np.round(times,6).tolist()
    (output/'pilot_report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    from tools.export_rhino import export_file
    report['rhino_export']=export_file(output/'pilot_registered.ply',output/'rhino');(output/'pilot_report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    (output/'README.txt').write_text('MOVING LIDAR 15-SECOND PILOT — QC RESULT\n\nRead pilot_report.json before using the map. This is not a validated full-recording reconstruction.\nThe PLY is in metres; RGB is provisional shared leaf/wood display. Raw signal/NIR and source frame/pixel IDs are retained for selected real first returns. scan_id is the processed frame index.\nThe OSF retains the selected complete scans and per-column SDK poses. per_column_poses.npz and trajectory.csv expose those poses/times.\nNative Rhino 8 files in rhino/ are in MILLIMETRES; physical geometry is unchanged.\nNo stationary result, Current-release asset, mesh, GPS constraint, loop closure or tree shape prior is used or replaced by this pilot.\n')
    print('PILOT SUMMARY',json.dumps({k:report[k] for k in ['status','flags','processed_frames','processed_time_span_s','retained_first_return_points','trajectory','GNSS_diagnostic','elapsed_compute_s']}),flush=True);return report


def plot(output,records,times,positions,nav,report):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    p=np.c_[records['x'],records['y'],records['z']];ix=np.linspace(0,len(p)-1,min(70000,len(p))).astype(int);q=p[ix];rgb=np.c_[records['red'],records['green'],records['blue']][ix]/255
    fig,axes=plt.subplots(2,2,figsize=(12,9));ax=axes[0,0];ax.plot(positions[:,0],positions[:,1],label='SDK trajectory');ax.scatter(*positions[0,:2],c='green',s=30,label='Start');ax.scatter(*positions[-1,:2],c='red',s=30,label='End');ax.set(xlabel='Map X (m)',ylabel='Map Y (m)',title='Sensor trajectory',aspect='equal');ax.legend(fontsize=8)
    ax=axes[0,1]
    if nav:
        ax.plot(nav['gps'][:,0],nav['gps'][:,1],label='GNSS antenna, local');ax.plot(nav['slam_fit'][:,0],nav['slam_fit'][:,1],label='SLAM, rigid fit without scale');ax.legend(fontsize=8);ax.set_aspect('equal')
    else:ax.text(.1,.5,'No valid host-time/GNSS overlap',transform=ax.transAxes)
    ax.set(title='GNSS diagnostic · timing/lever arm uncalibrated',xlabel='Relative east (m)',ylabel='Relative north (m)')
    ax=axes[1,0];ax.scatter(q[:,0],q[:,1],c=rgb,s=.35,alpha=.5,linewidths=0);ax.plot(positions[:,0],positions[:,1],c='#c52843',lw=1);ax.set(title='Pilot measured map · top view',xlabel='Map X (m)',ylabel='Map Y (m)',aspect='equal')
    travel=positions[-1,:2]-positions[0,:2];travel/=max(np.linalg.norm(travel),1e-9);along=q[:,:2]@travel
    ax=axes[1,1];ax.scatter(along,q[:,2],c=rgb,s=.35,alpha=.5,linewidths=0);ax.set(title='Height profile along travelled direction',xlabel='Along travel projection (m)',ylabel='Gravity-initialized Z (m)',aspect='equal')
    for ax in axes.flat:ax.grid(alpha=.2)
    fig.suptitle('15-second moving OS0 pilot · '+report['status'],fontsize=13);fig.text(.5,.012,'Constant-velocity per-column deskew; 20% return holdout. Colours are provisional leaf/wood display. No tree straightening or ground flattening.',ha='center',fontsize=8);fig.tight_layout(rect=(0,.04,1,.95));fig.savefig(Path(output)/'pilot_preview.png',dpi=125);plt.close(fig)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--pcap',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--voxel',type=float,default=.15);a=p.parse_args();run(a.pcap,a.output,a.voxel)
