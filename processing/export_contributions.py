import os
os.environ['OMP_NUM_THREADS']='2';os.environ.setdefault('MPLCONFIGDIR',os.path.join(os.getcwd(),'work','mpl'))
from pathlib import Path
import json, csv, zipfile, hashlib, shutil
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
root=Path.cwd();out=root/'output/registered_scans';out.mkdir(parents=True,exist_ok=True)
poses=np.load(root/'work/registered/final_poses.npy');files=sorted((root/'work/decoded').glob('*.npz'))
previous=json.loads((root/'output/scan_transforms.json').read_text())['scans']
dtype=np.dtype([('x','<f4'),('y','<f4'),('z','<f4'),('red','u1'),('green','u1'),('blue','u1'),('reflectivity','u1'),('scan_id','u1'),('classification','u1')])
def write_ply(path,a):
 header=f'''ply
format binary_little_endian 1.0
comment Registered orchard scan contributions; metres; same frame as orchard_row_registered.ply
comment RGB identifies scan; scan_id 0-17; classification 2 ground height band, 5 above-ground row
comment Independent per-scan samples; overlap between scans preserved
element vertex {len(a)}
property float x
property float y
property float z
property uchar red
property uchar green
property uchar blue
property uchar reflectivity
property uchar scan_id
property uchar classification
end_header
'''
 with open(path,'wb') as f:f.write(header.encode('ascii'));a.tofile(f)
 # Verify output count/precision from its serialized representation.
 with open(path,'rb') as f:
  while f.readline()!=b'end_header\n':pass
  b=np.fromfile(f,dtype=dtype)
 assert len(a)==len(b) and np.array_equal(a,b)
records=[];arrays=[];palette=(plt.get_cmap('tab20')(np.arange(18))[:,:3]*255+.5).astype('u1')
for i,(file,T) in enumerate(zip(files,poses)):
 data=np.load(file);p=data['xyz']@T[:3,:3].T+T[:3,3]
 mask=(p[:,0]>-.5)&(p[:,0]<6.6)&(p[:,1]>1.2875)&(p[:,1]<2.6875)&(p[:,2]>-.12)&(p[:,2]<4.5)
 p=p[mask];r=data['reflectivity'][mask]
 # Original per-recording 1 cm voxel sample, before merging; keep overlaps.
 a=np.empty(len(p),dtype=dtype)
 for j,key in enumerate(['x','y','z']):a[key]=p[:,j]
 for j,key in enumerate(['red','green','blue']):a[key]=palette[i,j]
 a['reflectivity']=r;a['scan_id']=i;a['classification']=np.where(p[:,2]<.12,2,5)
 assert np.isfinite(p).all() and len(p)>0
 # Rotation/translation are identical to the previously delivered transform.
 assert previous[i]['scan']==file.stem
 localR=np.eye(4);localR[:3,:3]=data['rotation']
 assert np.allclose(T@localR,previous[i]['sensor_to_merged'],atol=1e-10)
 name=f'scan_{i:02d}_{file.stem}_registered.ply';write_ply(out/name,a);arrays.append(a)
 rgb=palette[i].tolist();records.append(dict(scan_id=i,scan=file.stem,filename=name,point_count=len(a),rgb=rgb,hex='#'+''.join(f'{x:02x}' for x in rgb),sensor_to_merged=previous[i]['sensor_to_merged']))
 print(name,len(a),flush=True)
combined=np.concatenate(arrays);write_ply(out/'all_scans_coloured_by_recording.ply',combined)
assert len(combined)==sum(r['point_count'] for r in records)
manifest=dict(units='metres',coordinate_frame='Identical to orchard_row_registered.ply',crop=dict(x=[-.5,6.6],y=[1.2875,2.6875],z=[-.12,4.5]),sampling='Independent 1 cm samples from each stationary recording. No cross-scan deduplication; overlapping contributions are preserved.',total_points=len(combined),scans=records)
(out/'scan_manifest.json').write_text(json.dumps(manifest,indent=2))
with open(out/'scan_colours.csv','w') as f:
 w=csv.writer(f);w.writerow(['scan_id','scan','filename','point_count','red','green','blue','hex'])
 for r in records:w.writerow([r['scan_id'],r['scan'],r['filename'],r['point_count'],*r['rgb'],r['hex']])
readme='''SEPARATE REGISTERED SCANS

18 PLY files: one per original stationary PCAP recording, already in the exact
same registered coordinate frame as orchard_row_registered.ply. No additional
registration, centring, or transform is needed. Units are metres.

Each cloud has one uniform RGB colour. Load all 18 together in CloudCompare or
MeshLab and enable RGB colours. Toggle individual clouds to inspect overlap.
Colours and source recording IDs are listed in scan_colours.csv and the JSON.

The same tree-row/ground crop is used: X -0.5..6.6, Y 1.2875..2.6875,
Z -0.12..4.5 m. All stationary frames decoded for each recording are combined.
Independent per-recording 1 cm voxel samples are retained. This export is made
BEFORE cross-recording voxel selection and aggregate radius filtering, so
points from overlapping scans remain available in every contributing cloud.
Consequently the total point count is larger than in the merged deliverable,
and a small number of sparse points filtered from that deliverable may remain.

The companion combined-by-recording ZIP contains one PLY with all these same
points and the same colours. It also contains a scan_id field (0..17) if your
software supports selecting points by scalar attribute. Classification is a
simple height band (2 = near ground, 5 = above-ground row), not a validated
semantic classification. Reflectivity is preserved.

All previous registration limitations still apply. Initial alignment, approximate
local ground datum, unknown antenna offset, no independent survey validation.
'''
(out/'README.txt').write_text(readme)
shutil.copy2(Path(__file__),out/'export_contributions.py')
metadata=['README.txt','scan_manifest.json','scan_colours.csv','export_contributions.py']
with zipfile.ZipFile(root/'output/separate_registered_scans.zip','w',zipfile.ZIP_DEFLATED,compresslevel=5) as z:
 for r in records:z.write(out/r['filename'],r['filename'])
 for name in metadata:z.write(out/name,name)
with zipfile.ZipFile(root/'output/combined_by_recording.zip','w',zipfile.ZIP_DEFLATED,compresslevel=5) as z:
 z.write(out/'all_scans_coloured_by_recording.ply','all_scans_coloured_by_recording.ply')
 for name in metadata:z.write(out/name,name)
for name in ['separate_registered_scans.zip','combined_by_recording.zip']:
 p=root/'output'/name
 with zipfile.ZipFile(p) as z:assert z.testzip() is None
 print(name,p.stat().st_size,hashlib.sha256(p.read_bytes()).hexdigest(),flush=True)
# Preview uses random order so the last scan does not hide all others.
rng=np.random.default_rng(4);ii=rng.choice(len(combined),min(260000,len(combined)),replace=False);q=combined[ii];colors=np.c_[q['red'],q['green'],q['blue']]/255
fig,ax=plt.subplots(figsize=(15,7),facecolor='#101823');ax.set_facecolor('#101823');ax.scatter(q['x'],q['z'],c=colors,s=.6,alpha=.7,linewidths=0);ax.set_aspect('equal');ax.set(xlim=(-.6,6.7),ylim=(-.15,4.4),xlabel='Along tree row (m)',ylabel='Height above local ground (m)');ax.tick_params(colors='#d3dce8');ax.xaxis.label.set_color('#d3dce8');ax.yaxis.label.set_color('#d3dce8');ax.grid(alpha=.15)
fig.suptitle('REGISTERED SCANS · ONE COLOUR PER RECORDING',color='white',fontsize=19)
handles=[Line2D([],[],marker='o',linestyle='',color=palette[i]/255,label=records[i]['scan']) for i in range(18)]
fig.legend(handles=handles,loc='lower center',ncol=6,frameon=False,labelcolor='#d3dce8',fontsize=8)
fig.subplots_adjust(bottom=.21,top=.9)
fig.savefig(root/'output/scan_contributions_preview.png',dpi=160,facecolor=fig.get_facecolor(),bbox_inches='tight')
print('TOTAL',len(combined),flush=True)
