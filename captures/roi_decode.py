"""Decode measured first returns inside a world ROI without a sensor-forward crop."""
from pathlib import Path
import hashlib,importlib.metadata,inspect,json
import numpy as np


def decode_roi_attributes(path,rotation,pose,roi,cache=None):
    from ouster.sdk import core,open_source
    path=Path(path);rotation=np.asarray(rotation,dtype=float);pose=np.asarray(pose,dtype=float)
    def digest(p):
        with p.open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
    code=hashlib.sha256(inspect.getsource(decode_roi_attributes).encode()).hexdigest()
    identity=dict(pcap=digest(path),metadata=digest(path.with_suffix('.json')),code=code,sdk=importlib.metadata.version('ouster-sdk'),rotation=rotation.tolist(),pose=pose.tolist(),roi={k:roi[k] for k in ['along_m','across_m','height_m']})
    signature=hashlib.sha256(json.dumps(identity,sort_keys=True).encode()).hexdigest()
    if cache and Path(cache).exists():
        with np.load(cache) as d:
            if str(d['signature'])==signature:return {k:d[k] for k in d.files if k!='signature'}
    columns={k:[] for k in ['xyz','reflectivity','signal','near_ir','frame_id','pixel_index']}
    source=open_source(str(path));lut=core.XYZLut(source.sensor_info[0])
    try:
        for frames in source:
            scan=frames[0]
            if scan is None:continue
            ranges=scan.field('RANGE');valid=(ranges>500)&(ranges<12000);q=lut(scan)[valid]@rotation.T
            world=q@pose[:3,:3].T+pose[:3,3];x,y,z=world.T
            keep=(x>roi['along_m'][0])&(x<roi['along_m'][1])&(y>roi['across_m'][0])&(y<roi['across_m'][1])&(z>roi['height_m'][0])&(z<roi['height_m'][1])
            columns['xyz'].append(q[keep])
            for key,field in [('reflectivity','REFLECTIVITY'),('signal','SIGNAL'),('near_ir','NEAR_IR')]:columns[key].append(scan.field(field)[valid][keep])
            columns['frame_id'].append(np.full(int(keep.sum()),scan.frame_id,dtype='<u4'));columns['pixel_index'].append(np.flatnonzero(valid)[keep].astype('<u4'))
    finally:source.close()
    if not columns['xyz']:raise ValueError('No complete lidar frames in '+str(path))
    result={key:np.concatenate(parts) for key,parts in columns.items()}
    _,index=np.unique(np.floor(result['xyz']/.01).astype(np.int64),axis=0,return_index=True)
    result={key:value[index] for key,value in result.items()};result['xyz']=result['xyz'].astype('<f4')
    if cache:
        cache=Path(cache);cache.parent.mkdir(parents=True,exist_ok=True);np.savez_compressed(cache,signature=signature,**result)
    return result
