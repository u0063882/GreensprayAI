"""Diagnostic figures from independent raw IMU-levelled acquisitions only."""
from pathlib import Path
import os,tempfile
os.environ.setdefault('MPLCONFIGDIR',str(Path(tempfile.gettempdir())/'greenspray-raw-geometry-mpl'))
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from captures.raw_io import decode_raw_frame

COLOURS=dict(east='#117FB2',west='#D47925')


def profile(ax,r,output,draw_fit=True,cache_directory=None):
    data,_=decode_raw_frame(r['pcap'],cache_directory or Path(output)/'frames');p=data['xyz'];row=r['row'];centre=np.array(row['centre']);out=np.array(row['outward']);h=(p-centre)@np.array(row['along']);u=(p-centre)@out;z=p[:,2]-centre[2];sign=1 if r['geographic_side']=='east' else -1
    keep=(h>row['extent'][0])&(h<row['extent'][1])&(abs(u)<1.8)&(z>-.25)&(z<3.9);q=np.c_[sign*u[keep],z[keep]];near=u[keep]>.03
    ax.scatter(q[~near,0],q[~near,1],s=.35,c='#AEB8BF',alpha=.25,linewidths=0)
    ax.scatter(q[near,0],q[near,1],s=.4,c=COLOURS[r['geographic_side']],alpha=.4,linewidths=0)
    if draw_fit:
        fit=r['tree_wall'];zz=np.linspace(*fit['height_band_m'],50);uu=fit['coef'][2]+fit['coef'][1]*zz;ax.plot(sign*uu,zz,c='#B21C4B',lw=2,label='Fitted visible centre sheet')
    x=np.linspace(-1.65,1.65,100);slope=np.tan(np.deg2rad(r['ground_slope_outward_deg']));ax.plot(x,slope*sign*x,c='#333333',lw=1.4,label='Detected ground')
    ax.axvline(0,c='#666666',ls=':',lw=.8);ax.set(xlim=(-1.7,1.7),ylim=(-.2,3.9),xlabel='Across local row (east positive, m)',ylabel='Height above local row datum (m)');ax.set_aspect('equal');ax.grid(alpha=.2)


def plot_raw_audit(result,output,cache_directory=None):
    output=Path(output);rows=[r for r in result['reports'] if not r.get('error')];selected={}
    for side in ['east','west']:
        candidates=[r for r in rows if r['geographic_side']==side]
        if candidates:
            median=np.median([r['tree_wall']['outward_tilt_from_vertical_deg'] for r in candidates]);selected[side]=min(candidates,key=lambda r:abs(r['tree_wall']['outward_tilt_from_vertical_deg']-median))
    if len(selected)!=2:return
    fig,axes=plt.subplots(1,3,figsize=(14,7),gridspec_kw={'width_ratios':[1,1,1.25]})
    for ax,side in zip(axes[:2],['east','west']):
        r=selected[side];profile(ax,r,output,cache_directory=cache_directory);ax.set_title(side.capitalize()+' · '+r['scan']+'\n'+f"Wall {r['tree_wall']['outward_tilt_from_vertical_deg']:.1f}° from vertical; ground {r['ground_tilt_from_horizontal_deg']:.1f}°")
    ax=axes[2]
    for side in ['east','west']:
        rr=[r for r in rows if r['geographic_side']==side];xx=np.arange(len(rr))+(0 if side=='east' else len([r for r in rows if r['geographic_side']=='east'])+1)
        ax.scatter(xx,[r['tree_wall']['outward_tilt_from_vertical_deg'] for r in rr],c=COLOURS[side],label=side.capitalize()+' visible wall',s=45)
        ax.scatter(xx,[r['ground_tilt_from_horizontal_deg'] for r in rr],c=COLOURS[side],marker='x',label=side.capitalize()+' ground',s=45)
        if result['repeat_frames']:
            repeat={r['scan']:r for r in result['repeat_frames'] if 'error' not in r};last=[repeat.get(r['scan'],{}).get('last_wall_deg',np.nan) for r in rr];ax.scatter(xx,last,facecolors='none',edgecolors=COLOURS[side],s=70,label=side.capitalize()+' last-frame wall')
    ax.set(xlabel='Independent stationary recordings, grouped by side',ylabel='Degrees',ylim=(-1,35),title='All usable original '+result['dataset']+' captures');ax.grid(alpha=.2);ax.legend(fontsize=8)
    fig.suptitle('RAW PCAP GEOMETRY · IMU gravity rotation only · no fitted registration poses',fontsize=13)
    fig.text(.5,.015,'Independent local row/height origins are translated for display; no tilt correction or inter-scan alignment. Grey: opposite half / context.',ha='center',fontsize=9)
    fig.tight_layout(rect=(0,.035,1,.95));fig.savefig(output/'raw_imu_geometry.png',dpi=130);fig.savefig(output/'raw_imu_geometry.pdf');plt.close(fig)
    fig,ax=plt.subplots(figsize=(10,5));bands=[[.65,2.35],[.65,1.65],[1.,2.4]]
    for side,offset in [('east',-.10),('west',.10)]:
        for j,band in enumerate(bands):
            for k,quant in enumerate([.25,.5,.75]):
                vv=[f['outward_tilt_from_vertical_deg'] for r in rows if r['geographic_side']==side for f in r['sensitivity'] if f['height_band_m']==band and f['quantile']==quant and not f['wood_only']]
                if not vv:continue
                mid=np.median(vv);x=j*4+k+offset;ax.errorbar(x,mid,yerr=[[mid-min(vv)],[max(vv)-mid]],fmt='o',c=COLOURS[side],capsize=3,label=side.capitalize() if j==k==0 else None)
    ticks=[j*4+k for j in range(3) for k in range(3)];ax.set_xticks(ticks,[str(int(q*100))+'%' for _ in bands for q in [.25,.5,.75]])
    for j,band in enumerate(bands):ax.text(j*4+1,-.17,f'{band[0]:g}–{band[1]:g} m height',transform=ax.get_xaxis_transform(),ha='center')
    ax.set(ylabel='Apparent near-side wall tilt from IMU vertical (°)',xlabel='Within-cell outward position quantile',title='Raw-profile sensitivity to height band and visible-surface definition');ax.xaxis.labelpad=38;ax.grid(alpha=.2);ax.legend();fig.tight_layout();fig.savefig(output/'raw_angle_sensitivity.png',dpi=140);plt.close(fig)
    columns=max(sum(r['geographic_side']==side for r in rows) for side in ['east','west']);fig,axes=plt.subplots(2,columns,figsize=(2.6*columns,9),sharex=True,sharey=True,squeeze=False)
    for i,r in enumerate(rows):
        side=r['geographic_side'];j=sum(1 for q in rows[:i] if q['geographic_side']==side);ax=axes[0 if side=='east' else 1,j];profile(ax,r,output,cache_directory=cache_directory);ax.set_title(r['scan']+'\n'+f"{r['tree_wall']['outward_tilt_from_vertical_deg']:.1f}°",fontsize=9);ax.set_xlabel('Across (m)');ax.set_ylabel('Height (m)')
    for ri,side in enumerate(['east','west']):
        for j in range(sum(r['geographic_side']==side for r in rows),columns):axes[ri,j].axis('off')
    fig.suptitle('Every raw '+result['dataset']+' scan · independent local origins · no scan registration');fig.tight_layout(rect=(0,0,1,.96));fig.savefig(output/'raw_all_scan_profiles.png',dpi=120);plt.close(fig)

    fig,axes=plt.subplots(1,2,figsize=(13,5))
    for ax,side in zip(axes,['east','west']):
        r=selected[side];d,_=decode_raw_frame(r['pcap'],cache_directory or output/'frames');p=d['xyz'];g=r['ground'];row=r['row'];centre=np.array(row['centre']);height=p[:,2]-(p[:,:2]@np.array(g['coef'][:2])+g['coef'][2]);keep=(height>.13)&(height<.47)
        h=(p-centre)@np.array(row['along']);u=(p-centre)@np.array(row['outward']);sign=1 if side=='east' else -1
        ax.scatter(h[keep],sign*u[keep],s=2,c=COLOURS[side],alpha=.5,linewidths=0)
        ax.axhline(0,c='#333333',ls='--',lw=1,label='Fitted horizontal row direction')
        for stem in r['raw_stem_fits']:
            c=np.array(stem['centre_at_reference_z']);v=np.array(stem['axis']);hh=(c-centre)@row['along'];uu=sign*((c-centre)@row['outward']);colour='#247B3C' if stem['role']=='plausible trunk' else '#777777'
            ax.plot(hh,uu,'+',c=colour,ms=10,mew=2);ax.text(hh+.025,uu+.015,f"{stem['diameter_cm']:.0f} cm",fontsize=7,color=colour)
        ax.set(xlim=(row['extent'][0]-.2,row['extent'][1]+.2),ylim=(-.5,.5),xlabel='Along detected row (m)',ylabel='Across row, east positive (m)',title=side.capitalize()+' raw low-return candidates · '+r['scan']);ax.set_aspect('equal');ax.grid(alpha=.2)
    fig.suptitle('Low-stem/support detection from raw scans; + fitted free-lean tubes, inferred diameters');fig.tight_layout();fig.savefig(output/'raw_row_detection.png',dpi=130);plt.close(fig)
