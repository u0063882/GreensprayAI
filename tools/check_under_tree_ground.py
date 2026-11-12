#!/usr/bin/env python3
"""Compare a narrow under-trunk soil fit with the earlier wider ground fit.

Read-only diagnostic. Hold each tree plane, cloud and local row frame fixed.
"""
from pathlib import Path
import argparse,csv,itertools,json,os,sys
os.environ.setdefault('OMP_NUM_THREADS','2')
os.environ.setdefault('OPENBLAS_NUM_THREADS','2')
os.environ.setdefault('MPLCONFIGDIR','/tmp/greenspray-under-tree-mpl')
ROOT=Path(__file__).resolve().parent.parent
sys.path.insert(0,str(ROOT))
import numpy as np
import open3d as o3d
from captures.raw_geometry import plane
from captures.raw_io import decode_raw_frame
from captures.decode import selected_recordings
from processing.export_anchored import read_records,xyz
from tools.check_tree_ground_angles import NAMES,TITLES,measured_sample,fit_section,detect_row


def low_row(p,g):
    """Detect an XY line of compact low stems/supports in export coordinates."""
    height=p[:,2]-(p[:,:2]@g['coef'][:2]+g['coef'][2])
    q=p[(height>.12)&(height<.35)]
    cloud=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.c_[q[:,:2],np.zeros(len(q))]))
    labels=np.asarray(cloud.cluster_dbscan(eps=.045,min_points=5,print_progress=False));centres=[]
    for k in np.unique(labels):
        if k<0:continue
        c=q[labels==k]
        if len(c)>=18 and np.ptp(c[:,2])>.10 and np.max(np.ptp(c[:,:2],axis=0))<.50:centres.append(np.median(c[:,:2],axis=0))
    xy=np.asarray(centres);choices=[]
    for i,j in itertools.combinations(range(len(xy)),2):
        h=xy[j]-xy[i];length=np.linalg.norm(h)
        if length<.7:continue
        h/=length
        if abs(h[0])<.7:continue
        h*=np.sign(h[0]);n=np.array([-h[1],h[0]])
        keep=abs((xy-xy[i])@n)<.17
        if keep.sum()<3 or np.ptp(xy[keep]@h)<1.3:continue
        choices.append((int(keep.sum())+np.ptp(xy[keep]@h)/.5,keep))
    if not choices:raise ValueError('Insufficient coherent low-stem/support row; no under-trunk strip claimed')
    keep=max(choices,key=lambda v:v[0])[1];centre=xy[keep].mean(0)
    _,_,vt=np.linalg.svd(xy[keep]-centre);h=vt[0];h*=np.sign(h[0]);n=np.array([-h[1],h[0]])
    if np.std((xy[keep]-centre)@n)>.12:raise ValueError('Low-stem row too dispersed')
    return dict(centre=centre,along=h,across=n,extent=np.quantile((xy[keep]-centre)@h,[0,1])+[-.25,.25],candidates=int(keep.sum()),centres=xy[keep].tolist())


def narrow_ground(p,g,row,half_width=.25,allowed=None):
    """Restrict spatial ground ROI, preserving freely fitted soil slopes."""
    if half_width<.15:raise ValueError('Strip too narrow to constrain across-row slope')
    delta=p[:,:2]-row['centre'];across=delta@row['across'];along=delta@row['along']
    height=p[:,2]-(p[:,:2]@g['coef'][:2]+g['coef'][2])
    candidate=(abs(across)<half_width)&(along>row['extent'][0])&(along<row['extent'][1])&(abs(height)<.15)
    if allowed is not None:candidate &= np.asarray(allowed,dtype=bool)
    ix=np.flatnonzero(candidate);q=p[ix]
    if len(q)<100:raise ValueError('Fewer than 100 near-ground returns underneath row')
    fit=plane(q)
    ids=ix[abs(q@fit['normal']+fit['offset'])<.025]
    if len(ids)<80:raise ValueError('Fewer than 80 supported soil-plane returns')
    u=across[ids];v=along[ids]
    across_span=float(np.ptp(np.quantile(u,[.05,.95])));along_span=float(np.ptp(np.quantile(v,[.05,.95])))
    if across_span<.18 or along_span<1.3 or min(np.mean(u<0),np.mean(u>0))<.1:raise ValueError('Ground coverage does not constrain a plane on both sides of the row centre')
    design=np.c_[v-v.mean(),u-u.mean()]
    singular=np.linalg.svd(design,compute_uv=False)
    if singular[-1]/np.sqrt(len(ids))<.04:raise ValueError('Ground returns too close to a line to constrain across slope')
    detail=dict(half_width_m=half_width,candidate_returns=len(ix),supported_returns=len(ids),across_90pct_span_m=across_span,along_90pct_span_m=along_span,rmse_m=fit['rmse_m'],row_centres=row.get('centres',[]),row_centre_XY=np.asarray(row['centre']).tolist(),row_along_XY=np.asarray(row['along']).tolist(),row_across_XY=np.asarray(row['across']).tolist(),extent_m=np.asarray(row['extent']).tolist())
    return fit,ids,detail


def compare_fixed_tree(base,g):
    """Change ONLY the soil reference. Never reselect/refit tree points."""
    r=dict(base);ga=float(g['coef'][1]);gb=float(g['coef'][0]*base['section_X_m']+ga*base['display_Y_origin_m']+g['coef'][2]);b,c=base['tree_line_Y_bZ_c']
    cosine=abs(np.dot([-ga,1.],[1.,-b]))/np.hypot(ga,1.)/np.hypot(1.,b)
    r.update(angle_deg=float(np.degrees(np.arccos(np.clip(cosine,0,1)))),ground_cross_slope_deg=float(np.degrees(np.arctan(ga))),ground_line_Z_aY_b=[ga,gb],ground_normal=np.asarray(g['normal']).tolist(),ground_offset=float(g['offset']),ground_rmse_m=g['rmse_m'])
    return r


def local_ground(g,centre,along,across):
    a=float(g['coef'][:2]@along[:2]);b=float(g['coef'][:2]@across[:2]);c=float(centre[:2]@g['coef'][:2]+g['coef'][2]-centre[2])
    n=np.array([-a,-b,1.]);n/=np.linalg.norm(n)
    return dict(coef=np.array([a,b,c]),normal=n,offset=-c*n[2],rmse_m=g['rmse_m'])


def comparison(p,g,base,row,allowed=None):
    results=[];primary=None;ids=None
    for width in [.20,.25,.35]:
        try:
            fit,ix,detail=narrow_ground(p,g,row,width,allowed);r=compare_fixed_tree(base,fit)
            value=dict(status='ok',angle_deg=r['angle_deg'],ground_cross_slope_deg=r['ground_cross_slope_deg'],**detail)
            if width==.25:primary=r;ids=ix
        except ValueError as e:value=dict(half_width_m=width,status='insufficient',error=str(e))
        results.append(value)
    good=[x['ground_cross_slope_deg'] for x in results if x['status']=='ok']
    return primary,ids,dict(status='ok' if primary else 'insufficient',width_checks=results,width_slope_range_deg=float(np.ptp(good)) if len(good)>1 else None,width_sensitive=bool(np.ptp(good)>2.) if len(good)>1 else None)


def plot_profile(ax,p,base,new,ix,row,title):
    sample=measured_sample(p,.035);q=p[sample];u=q[:,1]-base['display_Y_origin_m']
    ax.scatter(u,q[:,2],s=.5,c='#99a4a7',alpha=.3,linewidths=0)
    if ix is not None:
        q=p[ix];ax.scatter(q[:,1]-base['display_Y_origin_m'],q[:,2],s=3,c='#078b8c',alpha=.5,linewidths=0,label='Selected near-ground points')
    yy=np.linspace(-1.8,1.8,100);a,b=base['ground_line_Z_aY_b'];ax.plot(yy,a*yy+b,'--',c='#cf8023',lw=2,label='Previous wider ground')
    if new:
        a,b=new['ground_line_Z_aY_b'];ax.plot(yy,a*yy+b,c='#126bb6',lw=2,label='Under-trunk ground')
    z=np.linspace(-.2,4.5,100);b,c=base['tree_line_Y_bZ_c'];ax.plot(b*z+c,z,c='#be2869',lw=1.6,label='Tree plane held fixed')
    label=f"{base['angle_deg']:.1f}° → {new['angle_deg']:.1f}°" if new else 'Under-row fit unavailable'
    ax.set(title=title+'\n'+label,xlim=(-1.8,1.8),ylim=(-.25,4.5),xlabel='Across row, centred (m)',ylabel='Z (m)');ax.set_aspect('equal');ax.grid(alpha=.18)


def run(collection,data_roots,output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    output.mkdir(parents=True,exist_ok=True)
    combined=[];raw=[];fig,axes=plt.subplots(2,2,figsize=(11,13));groundfig,gaxes=plt.subplots(2,2,figsize=(12,8))
    for name,ax,gax in zip(NAMES,axes.flat,gaxes.flat):
        path=collection/'pointclouds'/name/'final_registered.ply';_,rec=read_records(path);p=xyz(rec).astype(float);sample=measured_sample(p);q=p[sample];allowed=(q[:,2]>-.3)&(q[:,2]<.2);g=plane(q[allowed]);base=fit_section(p,g);row=low_row(p,g);new,ix,detail=comparison(q,g,base,row,allowed);ix=sample[ix] if ix is not None else None
        with path.open('rb') as f:sha=__import__('hashlib').file_digest(f,'sha256').hexdigest()
        combined.append(dict(dataset=name,scan='combined',source='combined_registered',input_sha256=sha,baseline=base,under_tree=new,**detail));plot_profile(ax,p,base,new,ix,row,TITLES[name])
        q=p[measured_sample(p,.025)];height=q[:,2]-(q[:,:2]@g['coef'][:2]+g['coef'][2]);floor=q[abs(height)<.22];gax.scatter(floor[:,1]-base['display_Y_origin_m'],floor[:,2],s=.6,c='#8e999b',alpha=.35,linewidths=0)
        if ix is not None:gax.scatter(p[ix,1]-base['display_Y_origin_m'],p[ix,2],s=2,c='#078b8c',alpha=.35,linewidths=0)
        yy=np.linspace(-1.8,1.8,100)
        for fit,colour,style in [(base,'#cf8023','--'),(new,'#126bb6','-')]:
            if fit:
                a,b=fit['ground_line_Z_aY_b'];gax.plot(yy,a*yy+b,style,c=colour,lw=2)
        gax.set(title=TITLES[name],xlabel='Across row, centred (m)',ylabel='Z (m)',ylim=(-.22,.22),xlim=(-1.8,1.8));gax.grid(alpha=.2)
        print(name,'combined',base['angle_deg'],new['angle_deg'] if new else detail,flush=True)
    axes.flat[0].legend(fontsize=8,loc='upper right');fig.suptitle('Ground reference restricted to a 0.50 m strip beneath the trunk row',fontsize=14);fig.text(.5,.015,'Same cloud, row frame and tree plane. Only the ground-fit ROI changes. Orange: previous ground; blue: restricted ground.\nTeal: supported near-ground points. Planes remain free to slope; no registered geometry is changed.',ha='center',fontsize=10);fig.tight_layout(rect=(0,.05,1,.96));fig.savefig(output/'combined_under_tree_ground.png',dpi=125);plt.close(fig)
    groundfig.suptitle('Ground detail · teal points support the under-trunk fit',fontsize=14);groundfig.text(.5,.01,'Orange dashed: previous wider fit. Blue: under-trunk fit. Z magnified; row-wide projections. Lines beyond selected points are extrapolations.',ha='center',fontsize=10);groundfig.tight_layout(rect=(0,.04,1,.95));groundfig.savefig(output/'selected_ground_detail.png',dpi=125);plt.close(groundfig)
    for name,root in zip(NAMES,data_roots):
        config_path=ROOT/'presets/captures'/(name+'.json');config=json.loads(config_path.read_text()) if config_path.exists() else {};files=selected_recordings(root,config.get('excluded_recordings',[]))
        import h5py
        sides={}
        for path in files:
            with h5py.File(path.with_suffix('.h5')) as h:sides.setdefault(path.parent.name,[]).append(float(np.median(h['lon'])))
        order=sorted(sides,key=lambda s:np.median(sides[s]));mapping={order[0]:'west',order[1]:'east'}
        fig,axes=plt.subplots(2,max(sum(p.parent.name==s for p in files) for s in sides),figsize=(22,11),squeeze=False);pos=dict(east=0,west=0)
        for path in files:
            side=mapping[path.parent.name];ax=axes[0 if side=='east' else 1,pos[side]];pos[side]+=1;tag=path.parent.name+'_'+path.stem[-6:];record=dict(dataset=name,source='raw_imu_only',scan=tag,geographic_side=side,status='insufficient')
            try:
                data,meta=decode_raw_frame(path)
                if meta['selected_column_coverage']<.95:raise ValueError('Incomplete source frame')
                p=data['xyz'].astype(float);g=plane(p[(p[:,0]>.65)&(p[:,0]<5.)&(abs(p[:,1])<3.8)&(p[:,2]>-2.9)&(p[:,2]<-.65)])
                row,method=detect_row(p,g);delta=p-row['centre'];u=delta@row['outward'];along=delta@row['along'];ground_candidates=(abs(u)<.65)&(along>row['extent'][0])&(along<row['extent'][1])&(abs(p@g['normal']+g['offset'])<.10);g=plane(p[ground_candidates]);row,method=detect_row(p,g)
                centre=row['centre'];along_vec=row['along'];across_vec=row['outward']*(1 if side=='east' else -1);delta=p-centre;u=delta@across_vec;along=delta@along_vec;z=p[:,2]-centre[2];keep=(abs(u)<1.8)&(along>row['extent'][0])&(along<row['extent'][1])&(z>-.3)&(z<4.6);q=np.c_[along[keep],u[keep],z[keep]]
                lg=local_ground(g,centre,along_vec,across_vec);base=fit_section(q,lg)
                record.update(baseline=base,row_axis_method=method,pcap_sha256=meta['identity']['pcap_sha256'],selected_frame=meta['selected_frame'])
                if method!='low_stem_support_candidates':raise ValueError('Canopy row fallback does not establish the under-trunk soil strip')
                strip=dict(centre=np.zeros(2),along=np.array([1.,0.]),across=np.array([0.,1.]),extent=row['extent'])
                new,ix,detail=comparison(q,lg,base,strip,ground_candidates[keep]);record.update(under_tree=new,**detail);plot_profile(ax,q,base,new,ix,strip,side+' · '+tag)
                print(name,tag,record['status'],base['angle_deg'],new['angle_deg'] if new else '',flush=True)
            except ValueError as e:record['error']=str(e);ax.text(.05,.4,side+' · '+tag+'\n'+str(e),transform=ax.transAxes,wrap=True,fontsize=9);ax.axis('off');print(name,tag,'insufficient',e,flush=True)
            raw.append(record)
        for i,side in enumerate(['east','west']):
            for ax in axes[i,pos[side]:]:ax.axis('off')
        fig.suptitle(TITLES[name]+' · original IMU-levelled scans · wider → under-trunk ground reference',fontsize=14);fig.tight_layout(rect=(0,.02,1,.95));fig.savefig(output/(name+'_under_tree_scans.png'),dpi=110);plt.close(fig)
        for ex in config.get('excluded_recordings',[]):raw.append(dict(dataset=name,source='raw_imu_only',scan=Path(ex['path']).parent.name+'_'+Path(ex['path']).stem[-6:],status='excluded',error=ex['reason']))
    result=dict(policy='Read-only spatial soil ROI comparison; candidate sampling, tree fit and raw local row frame held fixed within each pair. Narrow candidates are subsets of the preceding wider-ground input candidates. No registration, mesh, IMU bias correction, or output rotation.',primary_strip_full_width_m=.5,sensitivity_full_widths_m=[.4,.5,.7],ground_candidate_height_limit_m=.15,limitations=['Trunk/support row is inferred, not manually surveyed. Narrow soil planes remain sensitive to grass and surface roughness.','Single-view tree-plane visibility bias is unchanged; this does not establish a structural V centre.','Canopy-only row fallbacks are not used to claim an under-trunk soil strip.','Width sensitivity is diagnostic, not a formal confidence interval.'],combined=combined,raw_scans=raw)
    (output/'under_tree_ground.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    fields=['dataset','source','scan','geographic_side','status','previous_angle_deg','under_tree_angle_deg','previous_ground_slope_deg','under_tree_ground_slope_deg','supported_ground_returns','width_slope_range_deg','width_sensitive','row_axis_method','error']
    with (output/'under_tree_angles.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore',lineterminator='\n');writer.writeheader()
        for r in combined+raw:
            a=r.get('baseline',{});b=r.get('under_tree') or {};detail=next((d for d in r.get('width_checks',[]) if d['half_width_m']==.25),{})
            writer.writerow(dict(r,previous_angle_deg=a.get('angle_deg'),under_tree_angle_deg=b.get('angle_deg'),previous_ground_slope_deg=a.get('ground_cross_slope_deg'),under_tree_ground_slope_deg=b.get('ground_cross_slope_deg'),supported_ground_returns=detail.get('supported_returns')))
    print('DONE',sum(r['status']=='ok' for r in raw),'raw narrow fits',flush=True)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--collection',type=Path,required=True);p.add_argument('--data-roots',type=Path,nargs=4,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();run(a.collection,a.data_roots,a.output)
if __name__=='__main__':main()
