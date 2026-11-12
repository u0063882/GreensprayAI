"""Before/after figures for the independent joint pose stage."""
from pathlib import Path
import os,tempfile
os.environ.setdefault("MPLCONFIGDIR",os.path.join(tempfile.gettempdir(),"greenspray-joint-mpl"))
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from captures.joint_registration import apply


def plot_joint(clouds,labels,seeds,corrections,report,output,dataset):
    output=Path(output);output.mkdir(parents=True,exist_ok=True);colours=['#188fc9','#ca7829'];sides=list(dict.fromkeys(labels));updated=[apply(p,C) for p,C in zip(clouds,corrections)]
    fig,axes=plt.subplots(2,len(seeds),figsize=(3*len(seeds),7.5),squeeze=False)
    models={x['stem']:np.array(x['parameters']) for x in report.get('retained_landmarks',[])}
    excluded={int(k[0]) for k in report.get('excluded_ambiguous_training_landmarks',[])}
    for row,points in enumerate([clouds,updated]):
        for col,(key,seed) in enumerate(seeds):
            ax=axes[row,col];seed=np.asarray(seed)
            for p,label in zip(points,labels):
                keep=(abs(p[:,2]-.30)<.01)&(abs(p[:,0]-seed[0])<.28)&(abs(p[:,1]-seed[1])<.34);q=p[keep]
                ax.scatter((q[:,0]-seed[0])*100,(q[:,1]-seed[1])*100,s=2,c=colours[sides.index(label)],alpha=.5,linewidths=0)
            if row==1 and key in models:
                m=models[key];axis=np.r_[m[2:4],1.];axis/=np.linalg.norm(axis);theta=np.linspace(0,2*np.pi,200);direction=np.c_[np.cos(theta),np.sin(theta)];a=direction@axis[:2];rho=m[4]/(np.sqrt(1-a*a)-m[5]*a);curve=m[:2]+rho[:,None]*direction
                ax.plot((curve[:,0]-seed[0])*100,(curve[:,1]-seed[1])*100,'--',c='#566471',lw=1.2);ax.plot(*(m[:2]-seed)*100,'+',c='#566471',ms=9)
            if key in excluded:ax.text(.02,.98,'Ambiguous object\nexcluded from fitting',transform=ax.transAxes,va='top',fontsize=8,color='#a24d2d')
            ax.set(xlim=(-28,28),ylim=(-34,34),aspect='equal',title=f'Window {key} · '+('Before' if row==0 else 'After'),xlabel='Along from seed (cm)',ylabel='Across from seed (cm)');ax.grid(alpha=.2)
    fig.suptitle(dataset+' · joint rigid registration · slices at Z = 0.30 m, thickness 2 cm\nBlue A / orange B · dashed outline: shared training tube, not an independently measured diameter',fontsize=12)
    fig.text(.02,.01,'Both stages show their actual horizontal slices, so point identities may differ after 3D rotation. Quantitative validation uses fixed, excluded source returns.',fontsize=9)
    fig.subplots_adjust(top=.83,bottom=.11,hspace=.40,wspace=.45);fig.savefig(output/'slices_30cm.png',dpi=130,bbox_inches='tight');fig.savefig(output/'slices_30cm.pdf');plt.close(fig)
    fig,axes=plt.subplots(2,2,figsize=(11,10),sharey=True)
    for row,points in enumerate([clouds,updated]):
        for p,label in zip(points,labels):
            q=p[::6];c=colours[sides.index(label)]
            axes[row,0].scatter(q[:,1],q[:,2],s=.2,c=c,alpha=.3,linewidths=0)
            axes[row,1].scatter(q[:,0],q[:,2],s=.2,c=c,alpha=.3,linewidths=0)
        for col in [0,1]:axes[row,col].set(title=('Before' if row==0 else 'After')+(' · across row' if col==0 else ' · along row'),xlabel='Y (m)' if col==0 else 'X (m)',ylabel='Z (m)');axes[row,col].set_aspect('equal');axes[row,col].grid(alpha=.2)
    fig.suptitle(dataset+' · all measured returns · orthographic profiles · identical point sets')
    fig.tight_layout();fig.savefig(output/'profiles.png',dpi=130);plt.close(fig)
    for recovery in report.get('recoveries',[]):
        if not recovery.get('accepted'):continue
        index=recovery['scan_id'];other=[i for i,l in enumerate(labels) if l==labels[index] and i!=index]
        fig,axes=plt.subplots(1,2,figsize=(14,5),sharex=True,sharey=True)
        for ax,points,title in zip(axes,[clouds,updated],['Before','After']):
            q=np.concatenate([points[i][::15] for i in other]);ax.scatter(q[:,0],q[:,1],s=.2,c='#2c8fbd',alpha=.25,linewidths=0)
            q=points[index][::4];ax.scatter(q[:,0],q[:,1],s=.35,c='#d16638',alpha=.5,linewidths=0)
            ax.set(title=title,xlabel='Along row X (m)',ylabel='Across Y (m)');ax.set_aspect('equal');ax.grid(alpha=.2)
        fig.suptitle(dataset+f' · recovered recording {index}: orange · other same-side recordings: blue\nAll returns retained; correction is rigid, not a shape warp')
        fig.tight_layout();fig.savefig(output/f'recovered_scan_{index:02d}.png',dpi=125,bbox_inches='tight');plt.close(fig)
