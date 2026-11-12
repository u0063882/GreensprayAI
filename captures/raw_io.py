"""Original PCAP first returns and factory-frame IMU gravity, no fitted poses."""
from pathlib import Path
import hashlib,importlib.metadata,inspect,json
import numpy as np
import h5py
from captures.decode import levelling_rotation


def _sha(path):
    with Path(path).open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()


def decode_raw_frame(path,cache_directory=None,frame_choice='first_complete',voxel_m=.015):
    """Return measured samples after IMU rotation only, plus source provenance.

    Completeness is packet-column coverage, not a favourable geometry score.
    15mm sampling retains a measured return; it never replaces it by a centroid.
    The original, uncorrected IMU median is used; no learned bias is applied.
    """
    from ouster.sdk import core,pcap,open_source
    if frame_choice not in ['first_complete','last_complete']:raise ValueError('Unknown frame policy')
    if not np.isfinite(voxel_m) or voxel_m<=0:raise ValueError('Positive measured-sample spacing required')
    path=Path(path);identity=dict(pcap_sha256=_sha(path),metadata_sha256=_sha(path.with_suffix('.json')),
        gps_sha256=_sha(path.with_suffix('.h5')),sdk=importlib.metadata.version('ouster-sdk'),
        frame_choice=frame_choice,voxel_m=voxel_m,
        decoder_sha256=hashlib.sha256(inspect.getsource(decode_raw_frame).encode()).hexdigest())
    cache=Path(cache_directory)/path.parent.name/(path.stem+'_'+frame_choice+'.npz') if cache_directory else None
    meta=cache.with_suffix('.json') if cache else None
    if cache and cache.exists() and meta.exists():
        report=json.loads(meta.read_text())
        if report.get('identity')==identity:
            with np.load(cache,allow_pickle=False) as d:return {k:d[k] for k in d.files},report
    source=open_source(str(path));info=source.sensor_info[0];fmt=core.PacketFormat(info)
    acc=[];packets=pcap.PcapPacketSource(str(path))
    try:
        for _,packet in packets:
            if isinstance(packet,core.ImuPacket):
                acc.append([fmt.imu_la_x(packet.buf),fmt.imu_la_y(packet.buf),fmt.imu_la_z(packet.buf)])
    finally:packets.close()
    try:
        acc=np.asarray(acc,dtype=float).reshape(-1,3)@np.asarray(info.imu_to_sensor_transform)[:3,:3].T
        up,R=levelling_rotation(acc);lut=core.XYZLut(info);best=None;coverage=-1;frames=[]
        for fs in source:
            scan=fs[0]
            if scan is None:continue
            ranges=scan.field('RANGE');columns=int(((scan.status&1)>0).sum())
            valid=(ranges>500)&(ranges<12000)
            frames.append(dict(frame_id=int(scan.frame_id),valid_columns=columns,range_returns=int(valid.sum())))
            if columns>coverage or (columns==coverage and frame_choice=='last_complete'):
                coverage=columns;best=(lut(scan)[valid]@R.T,scan.field('SIGNAL')[valid].copy(),
                    scan.field('NEAR_IR')[valid].copy(),np.flatnonzero(valid),int(scan.frame_id))
        window=info.format.column_window;expected=int((window[1]-window[0])%info.format.columns_per_frame+1)
    finally:source.close()
    if best is None:raise ValueError('No lidar frame in '+str(path))
    p,signal,nir,pixel,frame=best
    _,ix=np.unique(np.floor(p/voxel_m).astype(np.int64),axis=0,return_index=True)
    data=dict(xyz=p[ix].astype('<f4'),signal=signal[ix],near_ir=nir[ix],pixel_index=pixel[ix].astype('<u4'))
    with h5py.File(path.with_suffix('.h5')) as h:lat=float(np.median(h['lat']));lon=float(np.median(h['lon']))
    report=dict(identity=identity,pcap=str(path),imu_samples=len(acc),
        median_acceleration_sensor=np.median(acc,axis=0).tolist(),
        acceleration_mad_sensor=np.median(abs(acc-np.median(acc,axis=0)),axis=0).tolist(),
        up_sensor=up.tolist(),sensor_to_gravity_levelled=R.tolist(),latitude=lat,longitude=lon,
        selected_frame=frame,frame_coverage=frames,expected_columns=expected,
        selected_column_coverage=coverage/expected,points=len(ix),
        measurement_policy='Factory XYZLut first returns, 0.5–12m range; original IMU gravity rotation only; frame selected by packet completeness; first measured return per sampling cell. No registered pose, terrain levelling, learned gravity correction or PCA-up rotation.')
    if cache:
        cache.parent.mkdir(parents=True,exist_ok=True);np.savez_compressed(cache,**data);meta.write_text(json.dumps(report,indent=2)+'\n')
    return data,report
