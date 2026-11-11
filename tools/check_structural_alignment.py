#!/usr/bin/env python3
"""Audit a staged dataset or an optional proposed per-recording rigid correction.

Writes QC only. Primary clouds, matrices and meshes are never replaced.
"""
import argparse,json,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent.parent
sys.path.insert(0,str(ROOT))
from captures.structural_validation import validate_structure
from processing.export_anchored import read_records,xyz


def check_dataset(stage,dataset,output,correction_path=None):
    stage=Path(stage);output=Path(output);output.mkdir(parents=True,exist_ok=True)
    metadata=json.loads((stage/'transformations'/dataset/'transformations.json').read_text())
    folder=stage/'contributions'/dataset/'separate_scans'
    manifest=json.loads((folder/'scan_manifest.json').read_text())
    entries={s['scan']:s for s in metadata['scans']};clouds=[];scores=[];origins=[];labels=[];up=[]
    for s in manifest['scans']:
        pose=entries[s['scan']];_,records=read_records(folder/s['filename'])
        clouds.append(xyz(records));scores.append(records['wood_score'])
        origins.append(np.array(pose['sensor_to_merged'])[:3,3]);labels.append(s['scan'].split('_')[0])
        up.append(np.array(pose['gravity_levelled_to_merged'])[:3,2])
    C=(np.load(correction_path,allow_pickle=False) if correction_path else
       np.tile(np.eye(4),(len(clouds),1,1)))
    if len(C)!=len(clouds):raise ValueError('Correction order/count must match the scan manifest')
    report=validate_structure(clouds,scores,origins,labels,C)
    tilt=np.rad2deg(np.arccos(np.clip(np.array(up)[:,2],-1,1)))
    report.update(dataset=dataset,mode='Candidate check' if correction_path else 'Coverage audit; identity is not proof of correct registration',
                  scan_order=[s['scan'] for s in manifest['scans']],
                  delivered_imu_up_tilt_deg=tilt.tolist(),
                  minimum_delivered_imu_up_tilt_deg=float(tilt.min()),
                  maximum_delivered_imu_up_tilt_deg=float(tilt.max()),
                  primary_outputs_changed=False)
    (output/'structural_alignment.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,3,figsize=(13,6),sharex=True,sharey=True)
    side_order=list(dict.fromkeys(labels));colours=['#188fc9','#ca7829']
    for p,label in zip(clouds,labels):
        side=side_order.index(label);q=p[::6]
        for a in [axes[side],axes[2]]:a.scatter(q[:,1],q[:,2],s=.16,c=colours[side],alpha=.3,linewidths=0)
    for a,title in zip(axes,side_order+['Both sides']):
        a.set(title=title,xlabel='Across row Y (m)',ylabel='Z (m)');a.set_aspect('equal');a.grid(alpha=.2)
    fig.suptitle(dataset+' · delivered coordinates · orthographic profile · all returns')
    fig.tight_layout();fig.savefig(output/'orthographic_profile.png',dpi=160);plt.close(fig)
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stage',type=Path,required=True)
    parser.add_argument('--dataset',required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--corrections',type=Path)
    args=parser.parse_args()
    if not args.dataset.replace('_','').isalnum():parser.error('Invalid dataset name')
    r=check_dataset(args.stage,args.dataset,args.output,args.corrections)
    print(json.dumps({k:r[k] for k in ['dataset','mode','coverage','observable','validation_passed','support_reasons','degradation_reasons']},indent=2))

if __name__=='__main__':main()
