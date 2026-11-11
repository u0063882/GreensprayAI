"""Dimensioned raw low-trunk slices; fitted circles are provisional diagnostics."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from captures.export import diagnostic_circle
from captures.orchard import save_json


def plot_dimensioned_slices(clouds,side_labels,seeds,path,dataset,heights=(.18,.28,.40),thickness_m=.02):
    """Show all measured points, both sides, common axes and explicit diameters.

    No leaf/wood filtering or registration is applied. A diameter from a partial
    or mixed arc is never presented as a verified physical trunk diameter.
    """
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    fig,axes=plt.subplots(len(seeds),len(heights),figsize=(12,3.5*len(seeds)),squeeze=False)
    rng=np.random.default_rng(31);records=[]
    palette={'sideA':'#177BB3','sideB':'#DC7A28'}
    for row,(k,seed) in enumerate(seeds):
        seed=np.asarray(seed)
        for col,height in enumerate(heights):
            ax=axes[row,col];fits=[];lines=[];texts=[]
            for side in ['sideA','sideB']:
                arrays=[p[(abs(p[:,0]-seed[0])<.28)&(abs(p[:,1]-seed[1])<.34)&(abs(p[:,2]-height)<thickness_m/2)] for p,label in zip(clouds,side_labels) if label==side]
                q=np.concatenate(arrays)[:,:2] if arrays else np.empty((0,2))
                ax.scatter((q[:,0]-seed[0])*100,(q[:,1]-seed[1])*100,s=2,c=palette[side],alpha=.48,linewidths=0)
                record=dict(stem_window=int(k),height_m=float(height),thickness_m=thickness_m,side=side,points=len(q),fit=None)
                if len(q)>=40:
                    try:
                        v,reliable,quality=diagnostic_circle(q,seed,rng)
                        at_limit=bool(v[2]>=.178 or v[2]<=.0122)
                        record['fit']=dict(centre_m=v[:2].tolist(),radius_m=float(v[2]),diameter_cm=float(200*v[2]),diagnostic_supported=bool(reliable),radius_at_search_limit=at_limit,**quality)
                        theta=np.linspace(0,2*np.pi,240);circle=(v[:2]-seed)[None,:]+v[2]*np.c_[np.cos(theta),np.sin(theta)]
                        line,=ax.plot(circle[:,0]*100,circle[:,1]*100,c=palette[side],ls='--',lw=1.2);lines.append(line)
                        ax.plot(*(v[:2]-seed)*100,'+',c=palette[side],ms=9,mew=1.5)
                        fits.append((v,reliable));texts.append((side[-1]+f' fit Ø {200*v[2]:.1f} cm'+(' LIMIT' if at_limit else ''),palette[side]))
                    except (ValueError,np.linalg.LinAlgError):texts.append((side[-1]+' fit unsupported',palette[side]))
                else:texts.append((side[-1]+' too few points',palette[side]))
                records.append(record)
            agreement=bool(len(fits)==2 and all(s for _,s in fits) and abs(fits[0][0][2]-fits[1][0][2])<.01 and np.linalg.norm(fits[0][0][:2]-fits[1][0][:2])<.02)
            for line in lines:line.set_linestyle('-' if agreement else '--')
            for offset,(text,colour) in enumerate(texts):ax.text(.025,.975-offset*.075,text,transform=ax.transAxes,va='top',fontsize=9,color=colour,bbox=dict(facecolor='white',alpha=.85,edgecolor='none',pad=1))
            if not agreement:ax.text(.97,.025,'FIT UNCERTAIN',transform=ax.transAxes,ha='right',va='bottom',fontsize=8,color='#A63F26')
            for record in records[-2:]:record['opposite_sides_diagnostic_agreement']=agreement
            ax.set(xlim=(-28,28),ylim=(-34,34),aspect='equal',title=f'Stem window {k} · height {height*100:.0f} cm',xlabel='Along from window centre (cm)',ylabel='Across (cm)')
            ax.set_xticks([-20,-10,0,10,20]);ax.set_yticks([-30,-20,-10,0,10,20,30]);ax.grid(alpha=.18)
    fig.suptitle(f'{dataset} · FINAL baseline alignment · {thickness_m*100:.0f} cm thick raw slices\nA blue, B orange · + fitted centres · dashed circles uncertain · all panels use the same centimetre scale',fontsize=13,y=.995)
    fig.text(.03,.006,'Circle diameters are model diagnostics, not verified trunk sizes. Branches, supports, short arcs and residual misalignment can inflate them.',fontsize=9,color='#555555')
    fig.tight_layout(rect=(0,.023,1,.97));fig.savefig(path,dpi=150);fig.savefig(path.with_suffix('.pdf'));plt.close(fig)
    result=dict(dataset=dataset,registration='Final delivered baseline; no experimental correction',units='metres in records; plot axes and diameters centimetres',agreement_limits=dict(maximum_centre_difference_m=.02,maximum_diameter_difference_m=.02),
                caution='Raw mixed returns and partial-arc circle fits; these are not measured physical trunk diameters. No classification filter was applied.',slices=records)
    save_json(path.with_suffix('.json'),result)
    return result
