"""Compare raw 30cm slices to user-measured trunk/post sizes; change no geometry."""
from pathlib import Path
import argparse,json,hashlib,sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle,Wedge
ROOT=Path(__file__).resolve().parent.parent
sys.path.insert(0,str(ROOT))
from processing.export_anchored import read_records,xyz
from captures.orchard import transform,save_json,sha256


def add_reference_checks(stage,reference_path=ROOT/'presets/spindle_trunk_reference.json'):
    stage=Path(stage);reference=json.loads(Path(reference_path).read_text());height=reference['measurement_height_m'];thickness=.02
    trunk_cm=np.array(reference['trunk_circumference_m'])*100/np.pi;post_cm=reference['support_post_circumference_m']*100/np.pi
    outputs=[]
    for dataset in ['spindle1','spindle2']:
        directory=stage/'contributions'/dataset/'separate_scans'
        if not directory.is_dir():continue
        manifest=json.loads((directory/'scan_manifest.json').read_text());clouds=[];sides=[];inputs=[]
        for entry in manifest['scans']:
            path=directory/entry['filename'];_,records=read_records(path);p=xyz(records)
            clouds.append(p[abs(p[:,2]-height)<thickness/2]);sides.append(entry['scan'].split('_')[0]);inputs.append(dict(filename=entry['filename'],coordinate_sha256=hashlib.sha256(p.astype('<f4').tobytes()).hexdigest()))
        config=json.loads((ROOT/f'presets/captures/{dataset}.json').read_text());matrices=json.loads((stage/'transformations'/dataset/'transformations.json').read_text());frame=np.array(matrices['registration_frame_to_final'])
        seeds=[(s['id'],transform(np.r_[s['seed_m'],height][None,:],frame)[0,:2]) for s in config['stem_candidates']]
        output=stage/'QC'/dataset/'trunk_reference_30cm';output.mkdir(parents=True,exist_ok=True)
        key=hashlib.sha256((json.dumps(inputs,sort_keys=True)+json.dumps(reference,sort_keys=True)+sha256(Path(__file__))).encode()).hexdigest()
        meta=output/'reference.json'
        if meta.exists() and json.loads(meta.read_text()).get('cache_key')==key and all((output/f'raw_slices.{suffix}').exists() for suffix in ['png','pdf']):continue
        fig,axes=plt.subplots(2,3,figsize=(13,10));axes=axes.ravel();ax=axes[0]
        # Reference shapes have arbitrary positions and occupy their own panel;
        # they are never placed on a measured cluster or claimed to locate it.
        ax.add_patch(Wedge((0,12),trunk_cm[1]/2,0,360,width=(trunk_cm[1]-trunk_cm[0])/2,facecolor='#20977C',alpha=.3))
        for diameter in trunk_cm:ax.add_patch(Circle((0,12),diameter/2,fill=False,color='#147D65',lw=1.5))
        ax.add_patch(Circle((0,-12),post_cm/2,fill=False,color='#866444',lw=1.5,ls='--'))
        ax.text(0,24,f'Trunk circumference 40–45 cm\nEquivalent Ø {trunk_cm[0]:.1f}–{trunk_cm[1]:.1f} cm',ha='center',va='center',fontsize=10,color='#147D65')
        ax.text(0,-23,f'Post circumference 22 cm\nCircular-equivalent Ø {post_cm:.1f} cm',ha='center',va='center',fontsize=10,color='#866444')
        ax.set_title('Physical size references\nPositions are arbitrary',fontsize=11)
        counts=[]
        for ax,(k,seed) in zip(axes[1:],seeds):
            count={}
            for side,colour in [('sideA','#177BB3'),('sideB','#DC7A28')]:
                arrays=[p[(abs(p[:,0]-seed[0])<.28)&(abs(p[:,1]-seed[1])<.34)] for p,label in zip(clouds,sides) if label==side]
                q=np.concatenate(arrays) if arrays else np.empty((0,3));count[side]=len(q)
                ax.scatter((q[:,0]-seed[0])*100,(q[:,1]-seed[1])*100,s=3,c=colour,alpha=.55,linewidths=0)
            ax.set_title(f'Stem window {k} · raw measured returns',fontsize=11);counts.append(dict(stem_window=k,points_by_side=count))
        for ax in axes:
            ax.set(xlim=(-28,28),ylim=(-34,34),aspect='equal',xlabel='Along (cm)',ylabel='Across (cm)');ax.set_xticks([-20,-10,0,10,20]);ax.set_yticks([-30,-20,-10,0,10,20,30]);ax.grid(alpha=.17)
        fig.suptitle(f'{dataset} · raw 2 cm slices at 30 cm above the local ground datum\nFinal baseline alignment · A blue, B orange · common centimetre scale',fontsize=14,y=.985)
        fig.text(.03,.025,'Trunk and support are both woody. A single-circle fit can mix their returns; these panels do not assign either object or force a radius.',fontsize=9,color='#555')
        fig.tight_layout(rect=(0,.045,1,.94))
        for ext in ['png','pdf']:fig.savefig(output/('raw_slices.'+ext),dpi=150)
        plt.close(fig)
        existing_models=[];baseline_quality=stage/'QC'/dataset/'registration_quality.json'
        if baseline_quality.exists():
            for model in json.loads(baseline_quality.read_text()).get('bounded',{}).get('reference_models',[]):
                if not model.get('usable',False):continue
                diameter=200*model['parameters'][2]
                existing_models.append(dict(stem_window=model['candidate'],baseline_model_diameter_cm=diameter,within_user_typical_trunk_range=bool(trunk_cm[0]<=diameter<=trunk_cm[1]),note='Comparison to typical user measurement, not a surveyed identity or a new fitting constraint'))
        save_json(meta,dict(cache_key=key,dataset=dataset,reference=reference,trunk_equivalent_diameter_cm=trunk_cm.tolist(),post_equivalent_diameter_cm=float(post_cm),slice_height_m=height,slice_thickness_m=thickness,inputs=inputs,slices=counts,existing_baseline_model_comparison=existing_models,geometry_policy='Unmodified final baseline measured returns; no fits, clustering, object labels or corrections applied'))
        gallery=stage/'QC'/dataset/'index.html';content=gallery.read_text();base=content.split('<!-- trunk-reference-30cm -->')[0];tail=content[len(base):];suffix=''
        if '<!-- final-trunk-refinement -->' in tail:suffix='<!-- final-trunk-refinement -->'+tail.split('<!-- final-trunk-refinement -->',1)[1]
        gallery.write_text(base+'<!-- trunk-reference-30cm --><h2>30 cm slices and measured size references</h2><p><a href="trunk_reference_30cm/raw_slices.pdf">PDF</a> · <a href="trunk_reference_30cm/reference.json">Measurements and provenance</a></p><img style="max-width:100%" src="trunk_reference_30cm/raw_slices.png">'+suffix)
        outputs.append(output/'raw_slices.png')
    return outputs


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--stage',type=Path,required=True);args=parser.parse_args()
    for path in add_reference_checks(args.stage):print(path)
