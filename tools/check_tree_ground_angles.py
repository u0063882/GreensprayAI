#!/usr/bin/env python3
"""Read-only across-row plane diagnostics for combined clouds and raw scans.

This fits all observed tree returns, never a selected V arm. A statistical
point-plane is not a verified biological centre plane, especially in one view.
"""
from pathlib import Path
import argparse,csv,hashlib,json,os,sys
os.environ.setdefault('OMP_NUM_THREADS','2')
os.environ.setdefault('OPENBLAS_NUM_THREADS','2')
os.environ.setdefault('MPLCONFIGDIR','/tmp/greenspray-plane-mpl')
ROOT=Path(__file__).resolve().parent.parent
sys.path.insert(0,str(ROOT))
import numpy as np
from captures.raw_geometry import plane,row_geometry
from captures.raw_io import decode_raw_frame
from captures.decode import selected_recordings
from processing.export_anchored import read_records,xyz

NAMES=['spindle1','spindle2','chandelier','vhedge']
TITLES=dict(spindle1='Spindle 1',spindle2='Spindle 2',chandelier='Chandelier',vhedge='V-hedge')

def measured_sample(p,spacing=.04):
    _,ix=np.unique(np.floor(p/spacing).astype(np.int64),axis=0,return_index=True)
    return ix


def fit_section(p,ground):
    """p is along-row, across-row, gravity Z. No geometry is changed."""
    q=p[measured_sample(p)]
    height=q[:,2]-(q[:,:2]@ground['coef'][:2]+ground['coef'][2])
    tree=q[(height>.30)&(height<4.5)]
    if len(tree)<500 or np.ptp(tree[:,2])<1.:raise ValueError('Insufficient tree height / returns')
    design=np.c_[tree[:,0],tree[:,2],np.ones(len(tree))]
    if np.linalg.cond(design)>10000:raise ValueError('Ill-conditioned tree plane')
    beta=np.linalg.lstsq(design,tree[:,1],rcond=None)[0]
    x0=float(np.median(tree[:,0]));y0=float(np.median(tree[:,1]))
    ga=float(ground['coef'][1]);gb=float(ground['coef'][0]*x0+ga*y0+ground['coef'][2])
    b=float(beta[1]);c=float(beta[0]*x0+beta[2]-y0)
    cosine=abs(np.dot([-ga,1.],[1.,-b]))/np.hypot(ga,1.)/np.hypot(1.,b)
    angle=float(np.degrees(np.arccos(np.clip(cosine,0,1))))
    zhit=(ga*c+gb)/(1-ga*b);yhit=b*zhit+c
    return dict(angle_deg=angle,ground_cross_slope_deg=float(np.degrees(np.arctan(ga))),tree_cross_lean_deg=float(np.degrees(np.arctan(b))),ground_line_Z_aY_b=[ga,gb],tree_line_Y_bZ_c=[b,c],intersection_YZ=[yhit,zhit],section_X_m=x0,display_Y_origin_m=y0,tree_plane_Y_aX_bZ_c=beta.tolist(),tree_returns=len(tree),tree_across_rms_m=float(np.sqrt(np.mean((design@beta-tree[:,1])**2))),ground_rmse_m=ground['rmse_m'],ground_normal=np.asarray(ground['normal']).tolist(),ground_offset=float(ground['offset']),structural_centre_verified=False)


def detect_row(p,g):
    try:
        return row_geometry(p,g),'low_stem_support_candidates'
    except ValueError:
        # Some views have fewer than three separately resolved low stems.
        # Fall back only for horizontal azimuth, never infer a vertical axis.
        height=p[:,2]-(p[:,:2]@g['coef'][:2]+g['coef'][2])
        q=p[(height>.30)&(height<3.)&(p[:,0]>.6)&(p[:,0]<5.)&(abs(p[:,1])<3.8)]
        q=q[measured_sample(q)]
        if len(q)<500:raise ValueError('Insufficient returns for horizontal row azimuth')
        centre=np.median(q[:,:2],axis=0)
        _,values,vt=np.linalg.svd(q[:,:2]-q[:,:2].mean(0),full_matrices=False)
        if (values[0]/values[1])**2<3:raise ValueError('Ambiguous horizontal canopy row axis')
        h=vt[0];h*=np.sign(h[1]);out=np.array([-h[1],h[0]])
        c=np.r_[centre,centre@g['coef'][:2]+g['coef'][2]]
        return dict(centre=c,along=np.r_[h,0.],outward=np.r_[out,0.],extent=np.quantile((q[:,:2]-centre)@h,[.02,.98])), 'horizontal_canopy_PCA_fallback'


def overlay(ax,p,r,colours=None,title=''):
    from matplotlib.patches import Arc
    ix=measured_sample(p,.035)
    if len(ix)>65000:ix=ix[np.linspace(0,len(ix)-1,65000).astype(int)]
    q=p[ix];h=q[:,2];u=q[:,1]-r['display_Y_origin_m']
    ax.scatter(u,h,s=.45,c=colours[ix] if colours is not None else '#586f76',alpha=.32,linewidths=0,rasterized=True)
    yy=np.linspace(-1.9,1.9,100);ga,gb=r['ground_line_Z_aY_b'];b,c=r['tree_line_Y_bZ_c'];zz=np.linspace(-.2,4.5,100)
    ax.plot(yy,ga*yy+gb,c='#176bba',lw=2,label='Detected ground')
    ax.plot(b*zz+c,zz,c='#c5296b',lw=2,label='Whole observed-tree plane')
    ax.axvline(0,c='#888',ls=':',lw=.6)
    angle1=np.degrees(np.arctan(ga));angle2=np.degrees(np.arctan2(1.,b))
    if angle2-angle1>90:angle1+=180
    low,high=sorted([angle1,angle2]);origin=np.asarray(r['intersection_YZ'])
    ax.add_patch(Arc(origin,.7,.7,theta1=low,theta2=high,color='#333',lw=1.2))
    ax.text(.04,.96,f"{r['angle_deg']:.1f}°",transform=ax.transAxes,va='top',fontsize=13,fontweight='bold',bbox=dict(facecolor='white',alpha=.88,edgecolor='none',pad=3))
    ax.set(xlim=(-1.9,1.9),ylim=(-.3,4.6),xlabel='Across row, centred (m)',ylabel='Z (m)',title=title)
    ax.set_aspect('equal');ax.grid(alpha=.18)


def write_csv(path,rows):
    fields=['dataset','source','scan','geographic_side','status','angle_deg','ground_cross_slope_deg','tree_cross_lean_deg','tree_across_rms_m','ground_rmse_m','selected_frame','column_coverage','row_axis_method','structural_centre_verified','error']
    with path.open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore',lineterminator='\n');w.writeheader();w.writerows(rows)


def run(collection,data_roots,output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    output.mkdir(parents=True,exist_ok=True)
    combined=[];raw=[]
    fig,axes=plt.subplots(2,2,figsize=(11,13))
    for name,ax in zip(NAMES,axes.flat):
        path=collection/'pointclouds'/name/'final_registered.ply'
        _,rec=read_records(path);p=xyz(rec).astype(float);q=p[measured_sample(p)]
        g=plane(q[(q[:,2]>-.30)&(q[:,2]<.20)])
        r=dict(dataset=name,source='combined_registered',scan='combined',status='ok',**fit_section(p,g))
        with path.open('rb') as f:r['input_sha256']=hashlib.file_digest(f,'sha256').hexdigest()
        combined.append(r);rgb=np.c_[rec['red'],rec['green'],rec['blue']]/255
        overlay(ax,p,r,rgb,TITLES[name]);del p,q,rec,rgb
    axes.flat[0].legend(loc='upper right',fontsize=8)
    fig.suptitle('Across-row profile: ground and one whole observed-tree plane',fontsize=15)
    fig.text(.5,.015,'Current combined clouds. Blue: ground. Pink: mean-point plane through all tree returns above 0.30 m.\nAcute section angle; no rotation or upright/symmetry constraint. This is not a verified structural V-centre fit.',ha='center',fontsize=10)
    fig.tight_layout(rect=(0,.05,1,.96));fig.savefig(output/'combined_ground_tree_planes.png',dpi=145);plt.close(fig)
    for name,data_root in zip(NAMES,data_roots):
        config_path=ROOT/'presets'/'captures'/(name+'.json')
        config=json.loads(config_path.read_text()) if config_path.exists() else {}
        exclusions=config.get('excluded_recordings',[])
        files=selected_recordings(data_root,exclusions)
        # Geographic sides inferred from input H5, never assumed from A/B labels.
        import h5py
        longitudes={}
        for path in files:
            with h5py.File(path.with_suffix('.h5')) as h:lon=float(np.median(h['lon']))
            longitudes.setdefault(path.parent.name,[]).append(lon)
        sides=sorted(longitudes,key=lambda s:np.median(longitudes[s]));mapping={sides[0]:'west',sides[1]:'east'}
        fig,axes=plt.subplots(2,max(sum(p.parent.name==s for p in files) for s in sides),figsize=(22,11),squeeze=False)
        positions=dict(east=0,west=0)
        for path in files:
            side=mapping[path.parent.name];j=positions[side];positions[side]+=1;ax=axes[0 if side=='east' else 1,j]
            tag=path.parent.name+'_'+path.stem[-6:]
            r=dict(dataset=name,source='raw_imu_only',scan=tag,geographic_side=side,status='failed')
            try:
                data,meta=decode_raw_frame(path) # no cache files: bounded disk use
                if meta['selected_column_coverage']<.95:raise ValueError('Insufficient column completeness')
                p=data['xyz'].astype(float)
                g=plane(p[(p[:,0]>.65)&(p[:,0]<5.)&(abs(p[:,1])<3.8)&(p[:,2]>-2.9)&(p[:,2]<-.65)])
                row,row_method=detect_row(p,g);delta=p-row['centre'];u=delta@row['outward'];along=delta@row['along']
                keep=(abs(u)<.65)&(along>row['extent'][0])&(along<row['extent'][1])&(abs(p@g['normal']+g['offset'])<.10)
                g=plane(p[keep]);row,row_method=detect_row(p,g)
                delta=p-row['centre'];u=delta@row['outward'];along=delta@row['along'];z=p[:,2]-row['centre'][2]
                keep=(abs(u)<1.8)&(along>row['extent'][0])&(along<row['extent'][1])&(z>-.3)&(z<4.6)
                sign=1 if side=='east' else -1
                q=np.c_[along[keep],sign*u[keep],z[keep]]
                # Transform fitted soil coefficients into this horizontal local frame.
                a=float(g['coef'][:2]@row['along'][:2]);b=float(sign*g['coef'][:2]@row['outward'][:2]);c=float(row['centre'][:2]@g['coef'][:2]+g['coef'][2]-row['centre'][2])
                normal=np.array([-a,-b,1.]);normal/=np.linalg.norm(normal)
                local_ground=dict(coef=np.array([a,b,c]),normal=normal,offset=-c*normal[2],rmse_m=g['rmse_m'])
                r.update(fit_section(q,local_ground));r.update(row_axis_method=row_method,status='ok',selected_frame=meta['selected_frame'],column_coverage=meta['selected_column_coverage'],pcap_sha256=meta['identity']['pcap_sha256'],imu_samples=meta['imu_samples'],visible_plane_caution='Single-view observed points can be occlusion-biased; not a verified centre plane.')
                overlay(ax,q,r,title=f"{side} · {tag}\nGround slope {r['ground_cross_slope_deg']:+.1f}°")
                print(name,tag,side,round(r['angle_deg'],2),flush=True)
            except Exception as e:
                r['error']=str(e);ax.text(.1,.5,tag+'\nFAILED: '+str(e),transform=ax.transAxes,wrap=True);print(name,tag,'FAILED',e,flush=True)
            raw.append(r)
        for i,side in enumerate(['east','west']):
            for ax in axes[i,positions[side]:]:ax.axis('off')
        fig.suptitle(TITLES[name]+' · every usable raw scan · IMU gravity rotation only',fontsize=16)
        fig.text(.5,.012,'All observed tree returns included; no selection of a V arm. Single-view occlusion can still bias the plane.\nBlue: independent ground; pink: observed-tree plane. Across axis east-positive, local ground datum Z=0; no ground flattening.',ha='center',fontsize=10)
        fig.tight_layout(rect=(0,.045,1,.95));fig.savefig(output/(name+'_every_raw_scan.png'),dpi=125);plt.close(fig)
        for ex in exclusions:raw.append(dict(dataset=name,source='raw_imu_only',scan=Path(ex['path']).parent.name+'_'+Path(ex['path']).stem[-6:],status='excluded',error=ex['reason']))
        write_csv(output/'every_scan_angles.csv',raw)
    fig,axes=plt.subplots(2,2,figsize=(13,9))
    for name,ax in zip(NAMES,axes.flat):
        rr=[r for r in raw if r['dataset']==name and r['status']=='ok']
        for side,colour in [('east','#147db3'),('west','#cf7523')]:
            ids=[i for i,r in enumerate(rr) if r['geographic_side']==side]
            ax.scatter(ids,[rr[i]['angle_deg'] for i in ids],c=colour,s=35,label=side.capitalize())
        ax.axhline(90,c='#888',ls=':',lw=1);ax.set_xticks(range(len(rr)),[r['scan'][-6:] for r in rr],rotation=65,fontsize=8)
        ax.set(title=TITLES[name],ylabel='Acute ground–observed-tree angle (°)',xlabel='Original scan time HHMMSS',ylim=(60,92));ax.grid(alpha=.2);ax.legend(fontsize=8)
    fig.suptitle('Raw IMU-levelled scans · across-row section angles',fontsize=15)
    fig.text(.5,.008,'Each point is an independent raw scan; one-view planes can follow visible structure and do not establish the centre of a V.',ha='center',fontsize=10)
    fig.tight_layout(rect=(0,.04,1,.96));fig.savefig(output/'every_scan_angle_summary.png',dpi=145);plt.close(fig)
    write_csv(output/'combined_angles.csv',combined)
    result=dict(method='40 mm measured-point sampling. Ground RANSAC plus robust slope fit. Single least-squares tree plane Y=aX+bZ+c through ALL observed returns 0.30–4.50 m above ground. Acute angle between plane sections at median along-row X; not the full 3D dihedral.',limitations=['Statistical observed-point plane, not a verified biological centre plane.','Raw single-view visibility and different fields of view prevent using raw/final angle differences alone as registration error.','Geographic east/west inferred independently per dataset from H5 longitude.','No registered transforms applied to raw scans; only original IMU leveling. Horizontal row axes from low-stem/support candidates; flagged horizontal canopy PCA fallback if insufficient resolved stems.','The excluded duplicate and no-IMU capture are listed rather than assigned invented angles.'],combined=combined,raw_scans=raw)
    (output/'tree_ground_angles.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print('COMPLETE',len([r for r in raw if r['status']=='ok']),'successful scans',flush=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--collection',type=Path,required=True);parser.add_argument('--data-roots',type=Path,nargs=4,required=True,metavar=('SPINDLE1','SPINDLE2','CHANDELIER','VHEDGE'));parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    run(args.collection,args.data_roots,args.output)
if __name__=='__main__':main()
