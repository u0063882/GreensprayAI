"""Export a bounded pose candidate, preserving per-point fields and east files."""
from pathlib import Path
import os
import tempfile
os.environ.setdefault('MPLCONFIGDIR',os.path.join(tempfile.gettempdir(),'greenspray-mpl'))
import csv
import json
import shutil
import zipfile
import numpy as np
import open3d as o3d
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

PLY_TYPES={'float':'<f4','double':'<f8','uchar':'u1','uint8':'u1','ushort':'<u2','int':'<i4','uint':'<u4'}


def read_records(path):
    with Path(path).open('rb') as stream:
        lines=[];properties=[];vertices=False;count=None
        for _ in range(250):
            line=stream.readline().decode('ascii');lines.append(line);tokens=line.split()
            if tokens[:2]==['element','vertex']:count=int(tokens[2]);vertices=True
            elif tokens[:1]==['element']:vertices=False
            if vertices and tokens[:1]==['property']:
                if len(tokens)!=3 or tokens[1] not in PLY_TYPES:raise ValueError('Unsupported PLY property.')
                properties.append((tokens[2],PLY_TYPES[tokens[1]]))
            if line.strip()=='end_header':break
        if 'format binary_little_endian 1.0\n' not in lines or count is None:raise ValueError('Expected little-endian scalar-vertex PLY.')
        data=np.fromfile(stream,dtype=np.dtype(properties),count=count)
    if len(data)!=count:raise ValueError('Truncated PLY.')
    return lines,data


def write_records(path,header,data):
    header=[f'element vertex {len(data)}\n' if line.startswith('element vertex ') else line for line in header]
    with Path(path).open('wb') as stream:stream.write(''.join(header).encode('ascii'));data.tofile(stream)


def xyz(data):
    return np.c_[data['x'],data['y'],data['z']]


def export_candidate(scan_dir,out,corrections,transforms_path,quality_path):
    scan_dir=Path(scan_dir);out=Path(out);dest=out/'registered_scans';dest.mkdir(parents=True,exist_ok=True)
    files=sorted(scan_dir.glob('scan_*_registered.ply'))
    manifest=json.loads((scan_dir/'scan_manifest.json').read_text())
    transforms=json.loads(Path(transforms_path).read_text());all_records=[];header=None
    for i,(path,C) in enumerate(zip(files,corrections)):
        h,data=read_records(path);header=h
        p=xyz(data)@C[:3,:3].T+C[:3,3]
        keep=(p[:,0]>-.5)&(p[:,0]<6.6)&(p[:,1]>1.2875)&(p[:,1]<2.6875)&(p[:,2]>-.12)&(p[:,2]<4.5)
        if i<9:
            if not np.array_equal(C,np.eye(4)) or not keep.all():raise ValueError('East anchor would change during export.')
            shutil.copy2(path,dest/path.name)
        else:
            data=data[keep].copy();p=p[keep]
            for j,key in enumerate(['x','y','z']):data[key]=p[:,j]
            data['classification']=np.where(p[:,2]<.12,2,5)
            write_records(dest/path.name,h,data)
        all_records.append(data)
        scan=manifest['scans'][i];scan['point_count']=len(data);scan['sensor_to_merged']=(C@np.array(scan['sensor_to_merged'])).tolist()
        transform=transforms['scans'][i]
        for key in ['sensor_to_merged','gravity_levelled_to_merged']:
            if key in transform:transform[key]=(C@np.asarray(transform[key])).tolist()
    manifest['total_points']=sum(len(a) for a in all_records)
    manifest['registration']='Fixed delivered east/A anchor; individually bounded west/B rigid poses. Same row crop reapplied.'
    manifest['source_crop_limit']='Input PLYs were already cropped. Returns excluded by the original crop cannot be recovered by this export.'
    (dest/'scan_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    transforms['registration']=manifest['registration'];(out/'scan_transforms.json').write_text(json.dumps(transforms,indent=2)+'\n')
    with (dest/'scan_colours.csv').open('w',newline='') as stream:
        writer=csv.writer(stream,lineterminator='\n');writer.writerow(['scan_id','scan','filename','point_count','red','green','blue','hex'])
        for r in manifest['scans']:writer.writerow([r['scan_id'],r['scan'],r['filename'],r['point_count'],*r['rgb'],r['hex']])
    combined=np.concatenate(all_records)
    write_records(dest/'all_scans_coloured_by_recording.ply',header,combined)
    points=xyz(combined);_,ix=np.unique(np.floor(points/.012).astype(np.int64),axis=0,return_index=True)
    merged=combined[ix].copy();pc=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz(merged)))
    _,keep=pc.remove_radius_outlier(nb_points=5,radius=.06);merged=merged[keep];points=xyz(merged)
    colours=(plt.get_cmap('turbo')(np.clip(points[:,2]/4,0,1))[:,:3]*255).astype('u1');colours[points[:,2]<.12]=[110,125,136]
    for j,key in enumerate(['red','green','blue']):merged[key]=colours[:,j]
    write_records(out/'orchard_row_registered.ply',header,merged)
    description='BOUNDED EAST-ANCHORED ORCHARD REGISTRATION\n\nMetres; X along row, Y across, Z up.\nEast/Side A is unchanged. Each west/Side B recording has a bounded rigid correction.\nLow-stem constraints are below 0.5 m; full-height clouds are exported.\nPer-recording colours and point fields are retained. The same row crop is reapplied.\nInput was already cropped, so missing returns outside the original crop are not recovered.\nModel-based internal checks are not independent survey accuracy. Read the quality report.\n'
    (dest/'README.txt').write_text(description);(out/'README.txt').write_text(description)
    counts=dict(merged_points=len(merged),separate_scan_points=len(combined),east_files_unchanged=True)
    quality=json.loads(Path(quality_path).read_text());quality['export']=counts
    Path(quality_path).write_text(json.dumps(quality,indent=2)+'\n')
    summary=dict(method='East-anchored bounded per-recording west registration',
                 anchor='East / Side A unchanged',validation_passed=quality['validation_passed'],
                 export=counts,details_file='anchored_registration_quality.json',
                 held_out_height_validation=quality['held_out_height_validation'],
                 held_out_stem_validation=quality['held_out_stem_validation'],
                 held_out_ground_validation=quality['held_out_ground_validation'],
                 accuracy_note=quality['validation_caveat'])
    (out/'registration_quality.json').write_text(json.dumps(summary,indent=2)+'\n')
    rng=np.random.default_rng(44);ix=rng.choice(len(points),min(240000,len(points)),replace=False)
    q=points[ix];col=colours[ix]/255
    fig,axes=plt.subplots(2,1,figsize=(14,8),layout='constrained',gridspec_kw={'height_ratios':[3,1.3]})
    axes[0].scatter(q[:,0],q[:,2],c=col,s=.6,linewidths=0);axes[0].set(xlabel='Along row (m)',ylabel='Height (m)',title='Measured tree row: fixed east + bounded per-recording west')
    axes[1].scatter(q[:,0],q[:,1],c=col,s=.6,linewidths=0);axes[1].set(xlabel='Along row (m)',ylabel='Across row (m)')
    for axis in axes:axis.set_aspect('equal');axis.grid(alpha=.2)
    fig.savefig(out/'orchard_row_preview.png',dpi=160);plt.close(fig)
    metadata=['README.txt','scan_manifest.json','scan_colours.csv']
    with zipfile.ZipFile(out/'separate_registered_scans.zip','w',zipfile.ZIP_DEFLATED,compresslevel=5) as archive:
        for path in files:archive.write(dest/path.name,path.name)
        for name in metadata:archive.write(dest/name,name)
    with zipfile.ZipFile(out/'combined_by_recording.zip','w',zipfile.ZIP_DEFLATED,compresslevel=5) as archive:
        archive.write(dest/'all_scans_coloured_by_recording.ply','all_scans_coloured_by_recording.ply')
        for name in metadata:archive.write(dest/name,name)
    with zipfile.ZipFile(out/'orchard_row_result.zip','w',zipfile.ZIP_DEFLATED,compresslevel=5) as archive:
        for name in ['orchard_row_registered.ply','scan_transforms.json','registration_quality.json','README.txt','orchard_row_preview.png']:
            archive.write(out/name,name)
        archive.write(quality_path,'anchored_registration_quality.json')
    for name in ['orchard_row_result.zip','separate_registered_scans.zip','combined_by_recording.zip']:
        with zipfile.ZipFile(out/name) as archive:
            if archive.testzip() is not None:raise RuntimeError('Exported ZIP failed CRC check.')
    return counts
