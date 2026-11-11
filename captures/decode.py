"""Decode selected stationary captures with input/code-verified caches."""
from pathlib import Path
import hashlib,importlib.metadata,json
import numpy as np
import h5py
from ouster.sdk import core,pcap,open_source


def selected_recordings(data,exclusions):
    data=Path(data);all_files=sorted(data.rglob('*.pcap'));excluded={r['path']:r['reason'] for r in exclusions}
    available={p.relative_to(data).as_posix() for p in all_files}
    if not set(excluded)<=available:raise ValueError('An excluded recording is missing from the supplied inputs')
    chosen=[p for p in all_files if p.relative_to(data).as_posix() not in excluded]
    if len(chosen)<6 or any(sum(p.parent.name==side for p in chosen)<3 for side in ['sideA','sideB']):raise ValueError('Need at least three usable recordings per side')
    return chosen


def levelling_rotation(acceleration):
    if len(acceleration)<3:raise ValueError('Insufficient IMU packets for gravity levelling; review this recording before use')
    up=np.median(np.asarray(acceleration),axis=0)
    if not np.isfinite(up).all() or np.linalg.norm(up)<.1:raise ValueError('Invalid gravity observations')
    up/=np.linalg.norm(up);x=np.array([1.,0,0]);x-=up*np.dot(x,up)
    if np.linalg.norm(x)<.1:raise ValueError('Sensor forward is too close to gravity to infer a horizontal forward axis')
    x/=np.linalg.norm(x);return up,np.array([x,np.cross(up,x),up])


def decode_capture(data,output,exclusions=()):
    data=Path(data);output=Path(output);output.mkdir(parents=True,exist_ok=True);summary=[];tags=set()
    decoder_hash=hashlib.sha256(Path(__file__).read_bytes()).hexdigest();sdk=importlib.metadata.version('ouster-sdk')
    for path in selected_recordings(data,exclusions):
        tag=path.parent.name+'_'+path.stem[-6:]
        if tag in tags:raise ValueError('Recording tag collision: '+tag)
        tags.add(tag);destination=output/(tag+'.npz')
        hashes=[]
        for suffix in ['.pcap','.json','.h5']:
            with path.with_suffix(suffix).open('rb') as f:hashes.append(hashlib.file_digest(f,'sha256').hexdigest())
        signature=hashlib.sha256((''.join(hashes)+decoder_hash+sdk).encode()).hexdigest()
        with h5py.File(path.with_suffix('.h5')) as h:lat=float(np.median(h['lat']));lon=float(np.median(h['lon']))
        if destination.exists():
            with np.load(destination) as cached:
                if 'input_signature' in cached and str(cached['input_signature'])==signature:
                    summary.append(dict(tag=tag,lat=lat,lon=lon,up=cached['up'].tolist()));continue
        source=open_source(str(path));info=source.sensor_info[0];lut=core.XYZLut(info);packet_format=core.PacketFormat(info);acceleration=[]
        packets=pcap.PcapPacketSource(str(path))
        try:
            for _,packet in packets:
                if isinstance(packet,core.ImuPacket):acceleration.append([packet_format.imu_la_x(packet.buf),packet_format.imu_la_y(packet.buf),packet_format.imu_la_z(packet.buf)])
        finally:packets.close()
        try:
            up,rotation=levelling_rotation(acceleration);points=[];reflectivity=[]
            attributes={key:[] for key in ['signal','near_ir','frame_id','pixel_index']}
            for frame_set in source:
                frame=frame_set[0]
                if frame is None:continue
                ranges=frame.field('RANGE');mask=(ranges>500)&(ranges<12000);xyz=lut(frame)[mask]@rotation.T
                keep=(xyz[:,0]>.35)&(xyz[:,0]<3.15)&(xyz[:,2]>-2.3)&(xyz[:,2]<4.)
                points.append(xyz[keep]);reflectivity.append(frame.field('REFLECTIVITY')[mask][keep])
                attributes['signal'].append(frame.field('SIGNAL')[mask][keep])
                attributes['near_ir'].append(frame.field('NEAR_IR')[mask][keep])
                attributes['frame_id'].append(np.full(keep.sum(),frame.frame_id,dtype='<u4'))
                attributes['pixel_index'].append(np.flatnonzero(mask)[keep].astype('<u4'))
        finally:source.close()
        if not points:raise ValueError('No decoded lidar frames: '+str(path))
        points=np.concatenate(points);reflectivity=np.concatenate(reflectivity)
        if len(points)<10000:raise ValueError('Insufficient measured point coverage: '+str(path))
        _,indices=np.unique(np.floor(points/.01).astype(np.int32),axis=0,return_index=True);points=points[indices];reflectivity=reflectivity[indices]
        attributes={key:np.concatenate(value)[indices] for key,value in attributes.items()}
        np.savez_compressed(destination,xyz=points.astype('f4'),reflectivity=reflectivity,up=up,rotation=rotation,lat=lat,lon=lon,input_signature=signature,**attributes)
        summary.append(dict(tag=tag,lat=lat,lon=lon,up=up.tolist()));print('Decoded',tag,len(points),'points',flush=True)
    (output/'scans.json').write_text(json.dumps(summary,indent=2)+'\n')
    return summary
