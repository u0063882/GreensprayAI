import os
os.environ['OMP_NUM_THREADS']='2'
from pathlib import Path
import numpy as np,h5py,json
from ouster.sdk import core,pcap,open_source
import open3d as o3d
out=Path('work/decoded');out.mkdir(exist_ok=True)
summary=[]
for p in sorted(Path('work/data').rglob('*.pcap')):
 tag=p.parent.name+'_'+p.stem[-6:];dest=out/(tag+'.npz')
 with h5py.File(p.with_suffix('.h5')) as h: lat=float(np.median(h['lat']));lon=float(np.median(h['lon']))
 if dest.exists():
  a=np.load(dest);summary.append(dict(tag=tag,lat=lat,lon=lon,up=a['up'].tolist()));continue
 s=open_source(str(p));info=s.sensor_info[0];lut=core.XYZLut(info)
 pf=core.PacketFormat(info);acc=[]
 ps=pcap.PcapPacketSource(str(p))
 for _,pk in ps:
  if isinstance(pk,core.ImuPacket):acc.append([pf.imu_la_x(pk.buf),pf.imu_la_y(pk.buf),pf.imu_la_z(pk.buf)])
 up=np.median(acc,axis=0);up/=np.linalg.norm(up)
 x=np.array([1.,0,0]);x-=up*np.dot(x,up);x/=np.linalg.norm(x);y=np.cross(up,x);R=np.array([x,y,up])
 pts=[];ref=[]
 for fs in s:
  a=fs[0]
  if a is None: continue
  r=a.field('RANGE');mask=(r>500)&(r<12000)
  xyz=lut(a)[mask]@R.T
  m=(xyz[:,0]>.35)&(xyz[:,0]<3.15)&(xyz[:,2]>-2.3)&(xyz[:,2]<4.0)
  pts.append(xyz[m]);ref.append(a.field('REFLECTIVITY')[mask][m])
 s.close();ps.close()
 pts=np.concatenate(pts);ref=np.concatenate(ref)
 # Keep measured samples, rather than voxel centroids.
 _,idx=np.unique(np.floor(pts/.01).astype(np.int32),axis=0,return_index=True)
 pts=pts[idx];ref=ref[idx]
 np.savez_compressed(dest,xyz=pts.astype('f4'),reflectivity=ref,up=up,rotation=R,lat=lat,lon=lon)
 nonground=pts[pts[:,2]>-.8];hist,bins=np.histogram(nonground[:,0],bins=np.arange(.35,3.2,.1))
 print(tag,len(pts),'row depth',round(bins[np.argmax(hist)]+.05,2),'up',up.round(3),flush=True)
 summary.append(dict(tag=tag,lat=lat,lon=lon,up=up.tolist()))
(out/'scans.json').write_text(json.dumps(summary,indent=2))
