"""Diagnose opposing-view trunk geometry without collapsing real trunk diameters."""
import os
os.environ.setdefault('MPLCONFIGDIR',os.path.join(os.getcwd(),'work','mpl'))
os.environ.setdefault('OMP_NUM_THREADS','2')
from pathlib import Path
import json,csv,argparse
import open3d as o3d
import numpy as np
from scipy.optimize import least_squares
from sklearn.cluster import DBSCAN
from scipy.ndimage import gaussian_filter1d
from scipy.signal import find_peaks
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path.cwd()
def circle_fit(p,initial):
    # Partial-arc fits can be ill-conditioned. Return that diagnostic, not just a centre.
    def residual(v):return np.linalg.norm(p-v[:2],axis=1)-v[2]
    fit=least_squares(residual,initial,bounds=([initial[0]-.15,initial[1]-.15,.012],[initial[0]+.15,initial[1]+.15,.18]),loss='soft_l1',f_scale=.006,max_nfev=120)
    inlier=abs(residual(fit.x))<.022
    if np.sum(inlier)>=30:
        fit=least_squares(lambda v:np.linalg.norm(p[inlier]-v[:2],axis=1)-v[2],fit.x,bounds=([initial[0]-.15,initial[1]-.15,.012],[initial[0]+.15,initial[1]+.15,.18]),loss='soft_l1',f_scale=.004,max_nfev=100)
    inlier=abs(residual(fit.x))<.022
    angles=np.sort(np.mod(np.arctan2(p[inlier,1]-fit.x[1],p[inlier,0]-fit.x[0]),2*np.pi))
    coverage=2*np.pi-np.max(np.diff(np.r_[angles,angles[0]+2*np.pi])) if len(angles)>2 else 0
    cond=float(np.linalg.cond(fit.jac.T@fit.jac))
    return fit.x,float(np.sqrt(np.mean(residual(fit.x)[inlier]**2))),float(np.rad2deg(coverage)),cond

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scan-dir',type=Path,help='Use already registered per-recording PLYs instead of decoded caches.')
    args=parser.parse_args()
    out=ROOT/'output';out.mkdir(exist_ok=True)
    if args.scan_dir:
        files=sorted(args.scan_dir.glob('scan_*_registered.ply'))
        if len(files)!=18:parser.error('Expected all 18 registered scan PLYs.')
        poses=np.tile(np.eye(4),(18,1,1))
    else:
        files=sorted((ROOT/'work/decoded').glob('*.npz'))
        poses=np.load(ROOT/'work/registered/final_poses.npy')
    parts=[];sources=[]
    for i,(file,T) in enumerate(zip(files,poses)):
        raw=np.asarray(o3d.io.read_point_cloud(str(file)).points) if args.scan_dir else np.load(file)['xyz']
        q=raw@T[:3,:3].T+T[:3,3]
        m=(q[:,0]>-.5)&(q[:,0]<6.6)&(q[:,1]>1.35)&(q[:,1]<2.6)&(q[:,2]>.12)&(q[:,2]<.38)
        parts.append(q[m]);sources.append(np.full(m.sum(),i))
    p=np.concatenate(parts);source=np.concatenate(sources)
    # 12 mm sample limits dominance of repeated frames/nearby stations in seed detection.
    _,ix=np.unique(np.floor(p[:,:2]/.012).astype(int),axis=0,return_index=True)
    h,bins=np.histogram(p[:,0],bins=np.arange(-.5,6.62,.02))
    smooth=gaussian_filter1d(h.astype(float),1.5)
    peaks,_=find_peaks(smooth,distance=17,prominence=max(smooth)*.06)
    seeds=[]
    for peak in peaks:
        x=bins[peak]+.01;near=p[abs(p[:,0]-x)<.14,:2]
        if len(near)>100:seeds.append(np.median(near,axis=0))
    seeds=sorted(seeds,key=lambda q:q[0]);records=[];panels=[];rng=np.random.default_rng(321)
    for seed in seeds:
        m=(abs(p[:,0]-seed[0])<.19)&(abs(p[:,1]-seed[1])<.22)
        q=p[m,:2];sid=source[m]
        if np.sum(sid<9)<45 or np.sum(sid>=9)<45:continue
        # Reject broad, irregular clusters; these can be weeds, stakes or mixed stems.
        common,rmse,_,_=circle_fit(q,[*seed,.055]);a=q[sid<9];b=q[sid>=9]
        fa,ra,aa,ca=circle_fit(a,common);fb,rb,ab,cb=circle_fit(b,common)
        delta=fb[:2]-fa[:2]
        boot=[]
        # Block bootstrap by acquisition (not individual, strongly correlated LiDAR returns).
        for _ in range(45):
            centres=[]
            for side in [sid<9,sid>=9]:
                ids=np.unique(sid[side]);picked=rng.choice(ids,len(ids),replace=True)
                sample=np.concatenate([q[sid==i] for i in picked])
                centres.append(circle_fit(sample,common)[0][:2])
            boot.append(centres[1]-centres[0])
        ci=np.percentile(boot,[2.5,97.5],axis=0)
        reliable=bool(min(aa,ab)>100 and max(ca,cb)<150 and max(ra,rb)<.018 and abs(fa[2]-fb[2])<.035 and (ci[1,1]-ci[0,1])<.04)
        r=dict(candidate=len(records)+1,along_m=float(common[0]),across_m=float(common[1]),joint_radius_m=float(common[2]),joint_radial_rmse_m=rmse,centre_delta_along_m=float(delta[0]),centre_delta_across_m=float(delta[1]),across_ci95_low_m=float(ci[0,1]),across_ci95_high_m=float(ci[1,1]),radius_A_m=float(fa[2]),radius_B_m=float(fb[2]),arc_A_deg=aa,arc_B_deg=ab,condition_A=ca,condition_B=cb,side_centroid_y_difference_m=float(np.median(b[:,1])-np.median(a[:,1])),reliable_centre_comparison=reliable,acquisitions_A=int(len(np.unique(sid[sid<9]))),acquisitions_B=int(len(np.unique(sid[sid>=9]))))
        records.append(r);panels.append((a,b,common,fa,fb,r))
    trustworthy=[r for r in records if r['reliable_centre_comparison']]
    report=dict(decision='insufficient independent, well-constrained trunks for a global cross-row correction' if len(trustworthy)<3 else 'review centre residuals before applying any correction',coordinates='X along row, Y across row, Z up; metres',height_band_m=[.12,.38],candidates=records,reliable_candidates=len(trustworthy),candidate_count=len(records),median_reliable_centre_delta_across_m=float(np.median([r['centre_delta_across_m'] for r in trustworthy])) if trustworthy else None,interpretation='Facing bark returns are distinct physical surfaces. Circle centres are a diagnostic under a circular-trunk assumption. Partial arcs, bark shape, stakes and occlusions can invalidate centre fits. No alignment correction is applied by this check.',automatic_correction_applied=False)
    (out/'cross_row_quality.json').write_text(json.dumps(report,indent=2))
    if records:
        with (out/'cross_row_trunks.csv').open('w') as f:
            w=csv.DictWriter(f,fieldnames=list(records[0]),lineterminator='\n');w.writeheader();w.writerows(records)
        fig,axs=plt.subplots(int(np.ceil(len(panels)/3)),3,figsize=(13,4*np.ceil(len(panels)/3)),squeeze=False)
        for ax,(a,b,c,fa,fb,r) in zip(axs.flat,panels):
            ax.scatter(a[:,0],a[:,1],s=2,c='#2078bf',alpha=.5,label='Side A');ax.scatter(b[:,0],b[:,1],s=2,c='#ef812c',alpha=.5,label='Side B')
            theta=np.linspace(0,2*np.pi,180)
            for cc,color in [(fa,'#2078bf'),(fb,'#ef812c')]:
                ax.plot(cc[0]+cc[2]*np.cos(theta),cc[1]+cc[2]*np.sin(theta),c=color,lw=1);ax.plot(cc[0],cc[1],'+',c=color,ms=9)
            ax.set_aspect('equal');ax.grid(alpha=.2);ax.set(xlabel='Along row (m)',ylabel='Across row (m)',title=f"Candidate {r['candidate']} · x={c[0]:.2f} m\ncentre Δacross={100*r['centre_delta_across_m']:.1f} cm; {'usable' if r['reliable_centre_comparison'] else 'uncertain'}")
        for ax in list(axs.flat)[len(panels):]:ax.axis('off')
        axs.flat[0].legend();fig.suptitle('Opposing trunk cross-sections · fitted circles are diagnostic assumptions',fontsize=14);fig.tight_layout();fig.savefig(out/'cross_row_trunk_sections.png',dpi=160)
    print(json.dumps(report,indent=2),flush=True)
if __name__=='__main__':main()
