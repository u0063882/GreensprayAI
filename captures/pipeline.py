"""Capture provenance, isolated caches and promotion after geometry checks."""
from pathlib import Path
import json,hashlib
import numpy as np
from captures.orchard import (sha256,save_json,gps_frame,initial_registration,
    coarse_stems,refine_ground,refine_moving)
from captures.export import export_capture
from captures.decode import decode_capture,selected_recordings
ROOT=Path(__file__).resolve().parent.parent


def run_capture(workspace,config_path):
    workspace=Path(workspace).resolve();config=json.loads(Path(config_path).read_text());data=workspace/'work/data';cache=workspace/'work/capture_cache';cache.mkdir(parents=True,exist_ok=True)
    inputs=[]
    for side in ['sideA','sideB']:
        files=sorted((data/side).glob('*.pcap'))
        if len(files)<3:raise RuntimeError('Need at least three recordings per side')
        for p in files:
            for suffix in ['.pcap','.json','.h5']:
                f=p.with_suffix(suffix)
                if not f.is_file():raise RuntimeError('Missing matching input: '+str(f))
                inputs.append(dict(path=f.relative_to(data).as_posix(),sha256=sha256(f),bytes=f.stat().st_size))
    provenance=workspace/'work/input_manifest.json';identity=dict(dataset=config['dataset'],files=inputs)
    if provenance.exists() and json.loads(provenance.read_text())!=identity:raise RuntimeError('Capture inputs changed. Use a fresh workspace to avoid mixing cached data.')
    if not provenance.exists() and (workspace/'work/decoded/scans.json').exists():raise RuntimeError('Untracked decoded cache exists; use a fresh workspace.')
    save_json(provenance,identity)
    decode_capture(data,workspace/'work/decoded',config.get('excluded_recordings',[]))
    scans=json.loads((workspace/'work/decoded/scans.json').read_text());raw=[np.load(workspace/'work/decoded'/(s['tag']+'.npz'))['xyz'] for s in scans]
    if len(scans)!=len(selected_recordings(data,config.get('excluded_recordings',[]))):raise RuntimeError('Decoded recording count disagrees with inputs')
    _,_,a,b,_=gps_frame(scans);reports={}
    source=''.join(p.read_text() for p in [ROOT/'captures/orchard.py',ROOT/'captures/bounded.py',ROOT/'captures/decode.py'])
    base=json.dumps(identity,sort_keys=True)+source+json.dumps(config,sort_keys=True)
    def stage(name,function,parent=None):
        key=hashlib.sha256((base+name).encode()+(parent.tobytes() if parent is not None else b'')).hexdigest();meta=cache/(name+'.json');poses_file=cache/(name+'.npy')
        if meta.exists() and poses_file.exists() and json.loads(meta.read_text()).get('cache_key')==key:
            report=json.loads(meta.read_text())['report'];poses=np.load(poses_file);print('Verified capture cache:',name,flush=True)
        else:
            print('Capture stage:',name,flush=True)
            if name=='initial':poses,report=function(raw,scans,config)
            elif name=='coarse':poses,report=function(raw,parent,a,b,config)
            elif name=='ground':poses,report=function(raw,parent,config)
            else:poses,report=function(raw,parent,a,b,config)
            np.save(poses_file,poses);save_json(meta,dict(cache_key=key,report=report))
        reports[name]=report;return poses
    initial=stage('initial',initial_registration);coarse=stage('coarse',coarse_stems,initial);ground=stage('ground',refine_ground,coarse);final=stage('bounded',refine_moving,ground)
    reports['input_manifest']=identity
    reports['excluded_recordings']=config.get('excluded_recordings',[])
    save_json(workspace/'registration_quality_candidate.json',reports)
    if not reports['bounded']['validation_passed']:raise RuntimeError('Candidate not promoted: '+str(reports['bounded']['rejection_reasons']))
    # Confirm the assembled anchor is exactly fixed by the bounded stage.
    np.testing.assert_array_equal(final[a],ground[a])
    result=export_capture(workspace,config,scans,raw,final,coarse,reports)
    print('Capture finished:',config['dataset'],result['export'],flush=True)
    return workspace/'final'
