"""Before/after plots of the same measured samples for final pose refinement."""
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from captures.trunk_refinement import _apply


def plot_slices(clouds,sides,seeds,corrections,report,output,dataset):
    output=Path(output);output.mkdir(parents=True,exist_ok=True);records=[]
    for height in [.18,.30,.40]:
        fig,axes=plt.subplots(2,len(seeds),figsize=(3.25*len(seeds),7.8),squeeze=False)
        for col,(k,seed) in enumerate(seeds):
            counts={}
            for side,colour in [('sideA','#177BB3'),('sideB','#DC7A28')]:
                before=[];after=[]
                for p,label,C in zip(clouds,sides,corrections):
                    if label!=side:continue
                    q=_apply(p,C);near_before=(abs(p[:,0]-seed[0])<.28)&(abs(p[:,1]-seed[1])<.34)
                    near_after=(abs(q[:,0]-seed[0])<.28)&(abs(q[:,1]-seed[1])<.34)
                    keep=(abs(p[:,2]-height)<.01)&(near_before|near_after)
                    before.append(p[keep]);after.append(q[keep])
                before=np.concatenate(before);after=np.concatenate(after);counts[side]=len(before)
                for ax,q in zip(axes[:,col],[before,after]):
                    ax.scatter((q[:,0]-seed[0])*100,(q[:,1]-seed[1])*100,c=colour,s=2,alpha=.48,linewidths=0)
            model=report['trunk_models'].get(str(k));reference='No supported trunk model'
            if model:
                centre=np.asarray(model['centre_m'])+(height-.3)*np.array(model['lean_xy_per_m']);radius=model['radius_m']+(height-.3)*model['taper_m_per_m']
                theta=np.linspace(0,2*np.pi,180);circle=(centre-seed)[None,:]+radius*np.c_[np.cos(theta),np.sin(theta)]
                axes[1,col].plot(circle[:,0]*100,circle[:,1]*100,color='#485763',ls='--',lw=1)
                reference=f'Training model Ø {2*radius*100:.1f} cm'
            for row,ax in enumerate(axes[:,col]):
                ax.set(xlim=(-28,28),ylim=(-34,34),aspect='equal',xlabel='Along (cm)',ylabel='Across (cm)');ax.set_xticks([-20,0,20]);ax.set_yticks([-30,-15,0,15,30]);ax.grid(alpha=.17)
                ax.set_title(f'Stem window {k} · '+('Before' if row==0 else 'After'),fontsize=11)
            axes[1,col].text(.025,.025,reference,transform=axes[1,col].transAxes,fontsize=8,color='#485763')
            checks=[v for v in report.get('validation',{}).get('objects',[]) if v['stem']==k and v['role']=='trunk']
            if checks:
                check=checks[0];label=f"Held centre gap {check['before_gap_m']*100:+.1f} → {check['after_gap_m']*100:+.1f} cm"
                if not check['well_supported']:label+='\nLimited held-out views'
            else:label='No reliable held-out centre comparison'
            axes[1,col].text(.025,.975,label,transform=axes[1,col].transAxes,va='top',fontsize=7.5,color='#485763',bbox=dict(facecolor='white',alpha=.85,edgecolor='none',pad=1))
            records.append(dict(stem_window=k,height_m=height,points_by_side=counts))
        status='VALIDATED' if report['validation_passed'] else 'FLAGGED CANDIDATE — NOT PROMOTED'
        fig.suptitle(f'{dataset} · {height*100:.0f} cm height · {status}\nTop: current baseline · bottom: final rigid refinement · identical measured samples, 2 cm thick slices',fontsize=13,y=.99)
        fig.legend(handles=[Line2D([0],[0],marker='.',ls='',color='#177BB3',label='Side A'),Line2D([0],[0],marker='.',ls='',color='#DC7A28',label='Side B'),Line2D([0],[0],ls='--',color='#485763',label='Training-derived trunk model; soft reference sizes')],loc='lower center',ncol=3,bbox_to_anchor=(.5,.025),frameon=False,fontsize=10)
        fig.text(.025,.008,'No point deletion, averaging, scaling or deformation. 26–34 cm observations are withheld from the refinement fit; the 30 cm panel is a validation view.',fontsize=8.5,color='#555')
        fig.tight_layout(rect=(0,.085,1,.92))
        for ext in ['png','pdf']:fig.savefig(output/f'slices_{int(height*100):02d}cm.{ext}',dpi=150)
        plt.close(fig)
    return records


def plot_summary(reports,path):
    names=list(reports);fig,ax=plt.subplots(figsize=(10,5));y=np.arange(len(names));before=[];after=[]
    for name in names:
        check=reports[name].get('validation',{});a=check.get('before');b=check.get('after');before.append(a['median_absolute_m']*1000 if a else np.nan);after.append(b['median_absolute_m']*1000 if b else np.nan)
    ax.barh(y-.17,before,.3,color='#8B96A5',label='Before refinement')
    for i,(name,value) in enumerate(zip(names,after)):
        passed=reports[name].get('validation_passed',False);ax.barh(i+.17,value,.3,color='#15846B' if passed else '#D79036',hatch=None if passed else '//')
        ax.text(value+1,i+.17,f'{value:.1f} mm'+(' · accepted' if passed else ' · flagged'),va='center',fontsize=10)
        if np.isfinite(before[i]):ax.text(before[i]+1,i-.17,f'{before[i]:.1f}',va='center',fontsize=10,color='#657080')
    ax.set(yticks=y,yticklabels=names,xlabel='Median absolute across-row discrepancy of matched held-out object centres (mm)',title='Final trunk/post refinement — conditional checks on withheld 26–34 cm observations');ax.invert_yaxis();ax.grid(axis='x',alpha=.15);ax.set_axisbelow(True);ax.legend(loc='lower right');ax.set_xlim(0,max(before)*1.15)
    for spine in ax.spines.values():spine.set_visible(False)
    fig.text(.02,.01,'Representative field sizes inform object association. These are internal consistency checks, not independently surveyed centre positions.',fontsize=9,color='#555');fig.tight_layout(rect=(0,.045,1,1))
    for ext in ['png','pdf']:fig.savefig(Path(path).with_suffix('.'+ext),dpi=150)
    plt.close(fig)
