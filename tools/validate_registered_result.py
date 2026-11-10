#!/usr/bin/env python3
"""Check anchored exports against the original measured points and pose matrices."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import zipfile
import numpy as np

ROOT=Path(__file__).resolve().parent.parent
sys.path.insert(0,str(ROOT))
from processing.export_anchored import read_records,xyz

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--baseline-scan-dir',type=Path,required=True)
parser.add_argument('--baseline-transforms',type=Path,required=True)
parser.add_argument('--result-dir',type=Path,required=True)
args=parser.parse_args();out=args.result_dir
quality=json.loads((out/'anchored_registration_quality.json').read_text())
assert quality['validation_passed'] and quality['candidate_exported']
corrections=np.tile(np.eye(4),(18,1,1))
for row in quality['per_recording']:corrections[row['scan_id']]=row['correction']
old=json.loads(args.baseline_transforms.read_text())['scans'];new=json.loads((out/'scan_transforms.json').read_text())['scans']
manifest=json.loads((out/'registered_scans/scan_manifest.json').read_text());counts=[]
for i,entry in enumerate(manifest['scans']):
    name=entry['filename'];source=args.baseline_scan_dir/name;target=out/'registered_scans'/name
    assert hashlib.sha256(source.read_bytes()).hexdigest()==quality['input_scans'][i]['sha256']
    _,a=read_records(source);_,b=read_records(target);C=corrections[i]
    expected=xyz(a)@C[:3,:3].T+C[:3,3]
    keep=(expected[:,0]>-.5)&(expected[:,0]<6.6)&(expected[:,1]>1.2875)&(expected[:,1]<2.6875)&(expected[:,2]>-.12)&(expected[:,2]<4.5)
    if i<9:assert source.read_bytes()==target.read_bytes()
    assert len(b)==int(keep.sum())==entry['point_count']
    np.testing.assert_allclose(xyz(b),expected[keep],rtol=0,atol=1e-6)
    for field in a.dtype.names:
        if field not in ['x','y','z','classification']:np.testing.assert_array_equal(b[field],a[field][keep])
    for key in ['sensor_to_merged','gravity_levelled_to_merged']:
        np.testing.assert_allclose(new[i][key],C@np.array(old[i][key]),rtol=0,atol=1e-12)
    counts.append(len(b))
_,merged=read_records(out/'orchard_row_registered.ply')
assert len(merged)==quality['export']['merged_points']
assert sum(counts)==manifest['total_points']==quality['export']['separate_scan_points']
assert np.isfinite(xyz(merged)).all()
assert min(merged['scan_id'])>=0 and max(merged['scan_id'])<=17
for name in ['orchard_row_result.zip','separate_registered_scans.zip','combined_by_recording.zip']:
    with zipfile.ZipFile(out/name) as archive:assert archive.testzip() is None
print('Nine east files unchanged; nine west rigid transforms, measured attributes, counts, metadata and ZIPs verified.')
