"""Sensor attributes and one shared, deterministic leaf/wood colour mapping.

OS0 has no RGB camera. These RGBs are derived colours, never camera colours.
Higher ambient NIR is a *provisional* leaf cue, not a validated material label.
"""
from pathlib import Path
import colorsys
import hashlib
import json
import numpy as np


def normalise_channel(values):
    return np.log1p(np.clip(np.asarray(values, dtype=float), 0, 65535)/1000)/np.log1p(65535/1000)


def fit_leafwood_model(samples, maximum_per_scan=4096):
    """Fit one Toolbox-style Lab model to paired (SIGNAL, NEAR_IR) arrays.

    The same saved model classifies every scan. No per-scan threshold fitting.
    """
    from captures.leafwood_lab import fit_lab_model
    return fit_lab_model(samples,maximum_per_scan=maximum_per_scan)


def fit_nir_reference(samples, maximum_per_scan=4096):
    """Equal scan contributions, fixed scaling, deterministic two-class NIR fit.

    Call once for the collection (or supply an already saved model). No fitting
    is performed by classify_leafwood. Confidence is heuristic, not probability
    calibrated against botanical labels. No geometry/registration scores tune it.
    """
    blocks=[]
    for values in samples:
        values=np.asarray(values).ravel()
        if len(values):
            ix=np.linspace(0,len(values)-1,min(len(values),maximum_per_scan),dtype=int)
            blocks.append(normalise_channel(values[ix]))
    if not blocks:raise ValueError('No NIR samples for shared leaf/wood model')
    x=np.concatenate(blocks);centres=np.quantile(x,[.2,.8])
    if centres[1]-centres[0]<1e-5:raise ValueError('NIR does not support two distinct classes')
    for _ in range(100):
        high=abs(x-centres[1])<abs(x-centres[0])
        if not high.any() or high.all():raise ValueError('Degenerate NIR classes')
        updated=np.array([x[~high].mean(),x[high].mean()])
        if max(abs(updated-centres))<1e-9:break
        centres=updated
    centres.sort();threshold=float(centres.mean())
    model=dict(version=1,method='Shared fixed-log-scale two-centre ambient NIR threshold; higher NIR provisionally leaf',
               log_centres=centres.tolist(),log_threshold=threshold,
               raw_nir_threshold=float(1000*np.expm1(threshold*np.log1p(65535/1000))),
               scans=len(blocks),training_samples=len(x),maximum_samples_per_scan=maximum_per_scan,
               confidence_note='Distance from shared threshold relative to centre separation; heuristic, not calibrated probability',
               rgb_note='Green/brown HSV hues and saturation from GreenSprayToolbox leafWoodSignal; brightness from fixed-scaled raw SignalPhoton. Global NIR classes replace per-scan k-means.',
               class_codes={'0':'unknown / missing NIR','1':'provisional wood','2':'provisional leaf'})
    model['model_sha256']=hashlib.sha256(json.dumps(model,sort_keys=True).encode()).hexdigest()
    return model


def classify_leafwood(signal,near_ir,model):
    if 'lab_centres' in model:
        from captures.leafwood_lab import classify_lab
        return classify_lab(signal,near_ir,model)
    n=normalise_channel(near_ir);centres=np.asarray(model['log_centres']);threshold=model['log_threshold']
    leaf=n>threshold
    confidence=np.clip(2*abs(n-threshold)/max(centres[1]-centres[0],1e-9),0,1)
    wood_score=np.where(leaf,.5-.5*confidence,.5+.5*confidence)
    colours=[]
    for rgb in [(151,98,53),(48,183,70)]:
        h,s,_=colorsys.rgb_to_hsv(*(np.array(rgb)/255))
        colours.append(colorsys.hsv_to_rgb(h,s,1.))
    brightness=normalise_channel(signal)**.65
    rgb=np.floor(255*np.asarray(colours)[leaf.astype(int)]*brightness[:,None]+.5).astype('u1')
    return dict(leafwood_class=np.where(leaf,2,1).astype('u1'),wood_score=wood_score.astype('f4'),
                leafwood_confidence=confidence.astype('f4'),rgb=rgb)


def decode_attributes(path,rotation,cache=None):
    """Reproduce baseline first-return/first-measured 10mm sampling exactly.

    Rotation is the original recorded levelling matrix, never re-estimated.
    Keep raw integer channels with point provenance before any registration.
    """
    from ouster.sdk import core,open_source
    import importlib.metadata
    path=Path(path);rotation=np.asarray(rotation)
    def digest(p):
        with p.open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
    import inspect
    decoder_identity=hashlib.sha256(inspect.getsource(decode_attributes).encode()).hexdigest()
    signature=hashlib.sha256((digest(path)+digest(path.with_suffix('.json'))+decoder_identity+importlib.metadata.version('ouster-sdk')).encode()+rotation.tobytes()).hexdigest()
    if cache and Path(cache).exists():
        with np.load(cache) as z:
            if str(z['signature'])==signature:return {k:z[k] for k in z.files if k!='signature'}
    columns={k:[] for k in ['xyz','reflectivity','signal','near_ir','frame_id','pixel_index']}
    source=open_source(str(path))
    try:
        lut=core.XYZLut(source.sensor_info[0])
        for frame_set in source:
            frame=frame_set[0]
            if frame is None:continue
            ranges=frame.field('RANGE');mask=(ranges>500)&(ranges<12000)
            xyz=lut(frame)[mask]@rotation.T
            keep=(xyz[:,0]>.35)&(xyz[:,0]<3.15)&(xyz[:,2]>-2.3)&(xyz[:,2]<4.)
            columns['xyz'].append(xyz[keep])
            for key,field in [('reflectivity','REFLECTIVITY'),('signal','SIGNAL'),('near_ir','NEAR_IR')]:
                if field not in frame.fields:raise ValueError(f'{path.name}: required sensor channel {field} missing')
                columns[key].append(frame.field(field)[mask][keep])
            columns['frame_id'].append(np.full(keep.sum(),frame.frame_id,dtype='<u4'))
            columns['pixel_index'].append(np.flatnonzero(mask)[keep].astype('<u4'))
    finally:source.close()
    if not columns['xyz']:raise ValueError('No complete measured frames: '+str(path))
    data={k:np.concatenate(v) for k,v in columns.items()}
    _,ix=np.unique(np.floor(data['xyz']/.01).astype(np.int32),axis=0,return_index=True)
    data={k:v[ix] for k,v in data.items()};data['xyz']=data['xyz'].astype('f4')
    if cache:
        Path(cache).parent.mkdir(parents=True,exist_ok=True)
        np.savez_compressed(cache,signature=signature,**data)
    return data


EXTRA_DTYPE=[('intensity','<u2'),('signal','<u2'),('near_ir','<u2'),('scan_id','<u4'),
             ('leafwood_class','u1'),('wood_score','<f4'),('leafwood_confidence','<f4'),
             ('leaf_red','u1'),('leaf_green','u1'),('leaf_blue','u1'),
             ('frame_id','<u4'),('pixel_index','<u4')]
PLY_NAMES={'f4':'float','f8':'double','u1':'uchar','u2':'ushort','u4':'uint','i4':'int'}


def enriched_records(original,attributes,scan_ids,by_scan=False):
    """Append fields while keeping exact coordinates, reflectivity and ordering."""
    replace={k for k,_ in EXTRA_DTYPE}
    dtype=[(k,original.dtype.fields[k][0]) for k in original.dtype.names if k not in replace]+EXTRA_DTYPE
    out=np.empty(len(original),dtype=np.dtype(dtype))
    for k in original.dtype.names:
        if k not in replace:out[k]=original[k]
    for k in ['signal','near_ir','frame_id','pixel_index','leafwood_class','wood_score','leafwood_confidence']:out[k]=attributes[k]
    out['intensity']=attributes['signal'];out['scan_id']=scan_ids
    for j,k in enumerate(['leaf_red','leaf_green','leaf_blue']):out[k]=attributes['rgb'][:,j]
    rgb=attributes['rgb']
    if by_scan:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        rgb=(plt.get_cmap('tab20')(np.asarray(scan_ids,dtype=int)%20)[:,:3]*255).astype('u1')
    for j,k in enumerate(['red','green','blue']):out[k]=rgb[:,j]
    return out


def write_records(path,records,comment='RGB is derived leafWoodSignal; intensity equals raw SIGNAL; scan_id identifies original PCAP'):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    header='ply\nformat binary_little_endian 1.0\ncomment Measured orchard points; metres; local ground datum\ncomment '+comment+'\nelement vertex '+str(len(records))+'\n'
    header+=''.join('property '+PLY_NAMES[records.dtype[k].str[1:]]+' '+k+'\n' for k in records.dtype.names)+'end_header\n'
    with path.open('wb') as f:f.write(header.encode());records.tofile(f)


def coordinate_digest(records):
    return hashlib.sha256(np.c_[records['x'],records['y'],records['z']].astype('<f4').tobytes()).hexdigest()
