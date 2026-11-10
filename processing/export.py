import os
os.environ.setdefault('MPLCONFIGDIR',os.path.join(os.getcwd(),'work','mpl'))
from pathlib import Path
import numpy as np,json,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
out=Path('output');d=np.load('work/registered/cropped.npz');p=d['xyz'];r=d['reflectivity'];s=d['source'];centres=d['centres']
ground=p[:,2]<.12
colors=(plt.get_cmap('turbo')(np.clip(p[:,2]/4.0,0,1))[:,:3]*255).astype('u1');colors[ground]=[110,125,136]
a=np.empty(len(p),dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('red','u1'),('green','u1'),('blue','u1'),('reflectivity','u1'),('scan_id','u1'),('classification','u1')])
for j,k in enumerate(['x','y','z']):a[k]=p[:,j]
for j,k in enumerate(['red','green','blue']):a[k]=colors[:,j]
a['reflectivity']=r;a['scan_id']=s;a['classification']=np.where(ground,2,5)
header=f'''ply
format binary_little_endian 1.0
comment Registered orchard row; metres; local ground datum
comment classification 2 ground band, 5 above-ground row; not semantic validation
element vertex {len(p)}
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
with open(out/'orchard_row_registered.ply','wb') as f:f.write(header.encode());a.tofile(f)
poses=np.load('work/registered/final_poses.npy');scans=json.loads(Path('work/decoded/scans.json').read_text());result=[]
for T,scan in zip(poses,scans):
 d0=np.load('work/decoded/'+scan['tag']+'.npz');R=np.eye(4);R[:3,:3]=d0['rotation'];result.append(dict(scan=scan['tag'],sensor_to_merged=(T@R).tolist(),gravity_levelled_to_merged=T.tolist(),gps_latitude=scan['lat'],gps_longitude=scan['lon']))
(out/'scan_transforms.json').write_text(json.dumps(dict(units='metres',coordinates='X along side-A travel direction, Y left of X, Z up; approximate local ground datum',scans=result),indent=2))
# Views use the same crop; only display is downsampled.
idx=np.arange(0,len(p),3);q=p[idx];c=colors[idx]/255
fig,axes=plt.subplots(2,1,figsize=(15,9),facecolor='#101823',gridspec_kw={'height_ratios':[3,1.4]})
for ax in axes:
 ax.set_facecolor('#101823');ax.tick_params(colors='#d1dbe7');ax.grid(alpha=.14,color='white');ax.set_aspect('equal');ax.set_xlim(-.6,6.7)
 for sp in ax.spines.values():sp.set_color('#43556b')
 ax.xaxis.label.set_color('#d1dbe7');ax.yaxis.label.set_color('#d1dbe7');ax.title.set_color('white')
axes[0].scatter(q[:,0],q[:,2],c=c,s=.6,linewidths=0,alpha=.8,rasterized=True)
axes[0].set(xlabel='Along tree row (m)',ylabel='Height above local ground (m)',title='Front view · measured points coloured by height',ylim=(-.15,4.4))
# Height sort makes canopy visible over the ground.
ix=np.argsort(q[:,2]);axes[1].scatter(q[ix,0],q[ix,1]-1.9875,c=c[ix],s=.6,linewidths=0,alpha=.8,rasterized=True)
axes[1].set(xlabel='Along tree row (m)',ylabel='Across row (m)',title='Top-down view · surrounding orchard removed',ylim=(-.8,.8))
fig.suptitle('REGISTERED TREE ROW + GROUND BENEATH',color='white',fontsize=20,y=.985)
fig.text(.5,.94,f'18 stationary scans · both sides · {len(p):,} points · 7.1 m crop length · 1.4 m strip width',ha='center',color='#b9c7d8',fontsize=11)
fig.text(.5,.015,'Ground shown in grey. Local coordinates in metres. Initial registration; tree identities not yet assigned.',ha='center',color='#b9c7d8',fontsize=11)
fig.subplots_adjust(top=.88,bottom=.085,hspace=.42)
fig.savefig(out/'orchard_row_preview.png',dpi=180,facecolor=fig.get_facecolor(),bbox_inches='tight')
# Independent display of the two sides to inspect duplication.
fig,ax=plt.subplots(figsize=(14,4),facecolor='white')
mask=(p[:,2]>.08)&(p[:,2]<.55)
for name,cond,col in [('Side A',s<9,'#0673b9'),('Side B',s>=9,'#ed711a')]:
 q=p[mask&cond][::2];ax.scatter(q[:,0],q[:,1],s=.5,c=col,label=name,alpha=.5,linewidths=0)
ax.set(xlabel='Along row (m)',ylabel='Across row (m)',title='Registration check: lower stems from opposite sides');ax.set_aspect('equal');ax.legend(markerscale=5);ax.grid(alpha=.2)
fig.savefig(out/'lower_stems_alignment_check.png',dpi=150,bbox_inches='tight')
print('exported',len(p),flush=True)
