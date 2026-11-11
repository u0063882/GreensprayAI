#!/usr/bin/env python3
"""Enrich a staged Current collection and compare an optional semantic method.

The baseline coordinates, sample order, reflectivity, matrices and meshes stay
unchanged. Experimental geometry is nested in QC, never promoted automatically.
"""
from pathlib import Path
import argparse,hashlib,json,sys
import numpy as np
from scipy.spatial import cKDTree
ROOT=Path(__file__).resolve().parent.parent
sys.path.insert(0,str(ROOT))
from captures.orchard import transform,sha256,save_json
from captures.leafwood import (decode_attributes,fit_leafwood_model,classify_leafwood,
    enriched_records,write_records,coordinate_digest)
from captures.register_leafwood import register_leafwood
from processing.export_anchored import read_records,xyz


def input_scan(data,entry):
    tag=entry['scan'];side,time=tag.split('_');matches=sorted((Path(data)/side).glob('*'+time+'.pcap'))
    if len(matches)!=1:raise ValueError('Cannot identify original recording: '+tag)
    path=matches[0]
    if entry.get('source_pcap_sha256') and sha256(path)!=entry['source_pcap_sha256']:raise ValueError('Original recording checksum mismatch')
    return path


def match_attributes(records,decoded,pose):
    q=transform(decoded['xyz'],pose);distance,index=cKDTree(q).query(xyz(records),workers=2)
    if len(distance) and distance.max()>3e-6:raise ValueError(f'Cannot recover measured source attributes; max coordinate discrepancy {distance.max():.8f} m')
    np.testing.assert_array_equal(records['reflectivity'],decoded['reflectivity'][index])
    return {k:v[index] for k,v in decoded.items() if k!='xyz'},dict(maximum_source_coordinate_discrepancy_m=float(max(distance,default=0)),points=len(records))


def source_config(stage,identifier):
    if identifier=='chandelier':
        seed=json.loads((ROOT/'presets/orchard_stem_candidates.json').read_text())
        return [(s['candidate'],np.array([s['along_m'],s['across_m']])) for s in seed['candidates']],5
    config=json.loads((ROOT/f'presets/captures/{identifier}.json').read_text())
    metadata=json.loads((stage/'transformations'/identifier/'transformations.json').read_text());frame=np.array(metadata['registration_frame_to_final'])
    seeds=[(s['id'],transform(np.r_[s['seed_m'],.3][None,:],frame)[0,:2]) for s in config['stem_candidates']]
    return seeds,config.get('held_out_stem',-1)


def plot_comparison(clouds,attributes,corrections,seeds,path,title):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(len(seeds),2,figsize=(9,3.1*len(seeds)),squeeze=False)
    for row,(k,seed) in enumerate(seeds):
        for col in [0,1]:
            ax=axes[row,col]
            for i,p in enumerate(clouds):
                hold=(p[:,2]>.2)&(p[:,2]<.4)&((p[:,2]<.26)|(p[:,2]>=.34))&(abs(p[:,0]-seed[0])<.23)&(abs(p[:,1]-seed[1])<.28)
                q=p[hold] if col==0 else transform(p[hold],corrections[i])
                ax.scatter((q[:,0]-seed[0])*100,(q[:,1]-seed[1])*100,s=1,c=attributes[i]['rgb'][hold]/255,alpha=.5,linewidths=0)
            ax.set(xlim=(-23,23),ylim=(-28,28),aspect='equal',xlabel='Along (cm)',ylabel='Across (cm)',title=f'Stem {k} · '+('baseline' if col==0 else 'leaf/wood candidate'));ax.grid(alpha=.15)
    fig.suptitle(title+' · same withheld low-stem samples, derived leaf/wood colour');fig.tight_layout(rect=(0,0,1,.97));fig.savefig(path,dpi=140);plt.close(fig)


def plot_attributes(records,path,title):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    q=records[::5];fig,axes=plt.subplots(2,1,figsize=(13,8),sharex=True)
    leaf=np.c_[q['leaf_red'],q['leaf_green'],q['leaf_blue']]/255
    palette=plt.get_cmap('tab20')(q['scan_id'].astype(int)%20)[:,:3]
    for ax,colour,label in zip(axes,[leaf,palette],['Leaf/wood hue; brightness from raw SIGNAL','Same measured points coloured by original scan_id']):
        ax.scatter(q['x'],q['z'],s=.55,c=colour,linewidths=0);ax.set(ylabel='Height (m)',title=label,aspect='equal');ax.grid(alpha=.15)
    axes[-1].set_xlabel('Along row (m)');fig.suptitle(title+' · primary baseline coordinates unchanged');fig.tight_layout();fig.savefig(path,dpi=140);plt.close(fig)


def summarise(reports,model,path):
    lines=['# Leaf/wood registration comparison — draft report','',
        'The delivered geometry-only registration remains the primary result. The alternative is an independent optional function that refines that baseline using shared NIR classes. Both methods retain exactly the same measured samples. No mesh is used or regenerated.','',
        '## Shared classification','',
        f"One deterministic model was fitted to balanced samples from all {model['scans']} recordings. The ambient NIR threshold is {model['raw_nir_threshold']:.1f} raw units (fixed logarithmic scaling). Higher NIR is provisionally leaf; lower NIR is provisionally wood. Confidence is heuristic. SignalPhoton controls display brightness using the same fixed green/brown hues and saturation as Toolbox leafWoodSignal. No per-scan fitting or brightness rescaling is used.",'',
        'This is an alternative to Toolbox’s per-scan Lab k-means, not a bit-identical reconstruction of its stochastic labels. OS0 records NIR/signal/reflectivity, not camera RGB. Export RGB is derived once and then preserved unchanged through registration. Ground/grass can fall into either provisional class; the model has no botanically labelled validation set.','',
        '## Registration and safeguards','',
        'The first assembled side is fixed. Each opposite-side acquisition gets one tightly bounded rigid pose. Same-side overlaps below 0.5 m use distance/normal-gated point-to-plane correspondences with soft class compatibility and wood priority. Cross-side tree correspondences are never nearest-neighbour matches: fixed-radius leaning stem surfaces and scanner-facing constraints retain distinct bark hemispheres. Ground is constrained to local terrain. There is no scale, per-point displacement or deformation.','',
        'Unused low-height bands and spatial patches are withheld from the added refinement. Metrics use fixed, geometry-selected validation correspondences and identical measured samples; no rematching or label-dependent filtering improves the reported score. Reference stem models use fixed-side training heights. These are conditional internal consistency checks; the baseline already used the same acquisitions, so this is not independent survey accuracy or a from-scratch algorithm comparison.','',
        'A geometry-only control runs the identical extra refinement with semantic matching and weights disabled. This distinguishes the effect of additional optimisation from the effect of NIR information. It uses the same input points, reference models, bounds and validation samples. No hyperparameters were selected using the reported validation scores.','',
        '## Results','',
        '| Dataset | Check | Points | Delivered baseline median / capped RMS (mm) | Extra geometry-only control (mm) | Leaf/wood candidate (mm) |','| --- | --- | ---: | ---: | ---: | ---: |']
    for identifier,report in reports.items():
        for name,check in report['checks'].items():
            a,b=check['baseline'],check['leafwood']
            control=report['geometry_only_control']['checks'][name]['leafwood']
            if a and b:lines.append(f"| {identifier} | {name.replace('_',' ')} | {a['points']:,} | {a['median_absolute_m']*1000:.2f} / {a['capped_rmse_5cm_m']*1000:.2f} | {control['median_absolute_m']*1000:.2f} / {control['capped_rmse_5cm_m']*1000:.2f} | {b['median_absolute_m']*1000:.2f} / {b['capped_rmse_5cm_m']*1000:.2f} |")
    lines+=['','Capped RMS limits each residual to 50 mm; uncapped RMS and sample counts remain in the JSON reports. Lower is better. A gate permits at most 1 mm worsening in median/capped RMS, rejects corrections at bounds, and flags low-stem across-width reductions over 3 mm. Passing means the candidate cleared these guards, not that it is more accurate.','',
            '| Dataset | Candidate checks | Largest low-stem width reduction (mm) | Status |','| --- | --- | ---: | --- |']
    flags=[]
    for identifier,report in reports.items():
        reduction=max([max(0,-s['change_m'])*1000 for s in report['shape_checks'] if s['change_m'] is not None],default=0)
        lines.append(f"| {identifier} | {'Pass' if report['validation_passed'] else 'Flagged'} | {reduction:.2f} | Baseline retained |")
        if report['rejection_reasons']:flags.append(identifier+': '+'; '.join(report['rejection_reasons'])+'.')
    lines+=['','Width reduction is a conservative warning, not proof of deformation: correcting an existing gap can also reduce measured width. Short arcs/clutter make the reference diameters uncertain. Flagged candidates are not suitable for automatic replacement.','']+flags
    improving=[]
    for identifier,report in reports.items():
        checks=[c for c in report['checks'].values() if c['baseline'] and c['leafwood']]
        if checks and all(c['leafwood']['capped_rmse_5cm_m']<=c['baseline']['capped_rmse_5cm_m'] for c in checks):improving.append(identifier)
    lines+=['','## Interpretation','',
        'The table, rather than the fitting objective, determines whether this experiment helped. Colour can favour stable wood but does not resolve missing opposite-face overlap. Improvement is not guaranteed. '+('All reported capped RMS checks decreased or stayed equal for: '+', '.join(improving)+'.' if improving else 'No dataset improved every reported capped RMS check.'),
        'All published primary clouds keep their original baseline coordinates and ordering. Experimental clouds/matrices are retained under each dataset’s QC/leafwood_comparison folder and have not replaced the final results. An external reference or independent repeat acquisition is needed to quantify absolute accuracy.','',
        '## Output fields','',
        '`intensity` is an exact alias of raw first-return `signal` (MATLAB SignalPhoton); `near_ir` and `reflectivity` remain distinct raw sensor channels. `red/green/blue` show derived leafWoodSignal in primary/separate clouds. `leaf_red/leaf_green/leaf_blue` preserve that colour even in recording-coloured views. `scan_id` is an unsigned 32-bit original PCAP index, mapped to filenames/checksums in the scan manifest. `frame_id` and `pixel_index` identify the original sensor frame and row-major raw H×W pixel. `leafwood_class`, `wood_score`, and `leafwood_confidence` describe the shared classifier. The old `classification` height-band field is retained and is not leaf/wood truth.','',
        'Changing display colouring in CloudCompare by the scan_id scalar field does not alter the stored leaf/wood RGB. Some generic PLY readers ignore custom properties; use a reader that preserves scalar fields.','']
    Path(path).write_text('\n'.join(lines))


def upgrade_collection(stage,data_directories,work,identifiers=None):
    stage=Path(stage);work=Path(work);work.mkdir(parents=True,exist_ok=True)
    identifiers=identifiers or sorted(data_directories);samples=[];prepared={};source_proofs={}
    for identifier in identifiers:
        folder=stage/'contributions'/identifier/'separate_scans'
        manifest=json.loads((folder/'scan_manifest.json').read_text())
        matrices=json.loads((stage/'transformations'/identifier/'transformations.json').read_text())
        entries={s['scan']:s for s in matrices['scans']};prepared[identifier]=[];source_proofs[identifier]=[]
        for entry in manifest['scans']:
            i=entry['scan_id'];pose=entries[entry['scan']];path=input_scan(data_directories[identifier],pose)
            rotation=np.linalg.solve(np.array(pose['gravity_levelled_to_merged'])[:3,:3],np.array(pose['sensor_to_merged'])[:3,:3])
            decoded=decode_attributes(path,rotation,work/'decoded'/identifier/(entry['scan']+'.npz'))
            _,records=read_records(folder/entry['filename']);attrs,proof=match_attributes(records,decoded,np.array(pose['gravity_levelled_to_merged']))
            proof.update(scan_id=i,pcap=path.name,pcap_sha256=sha256(path));source_proofs[identifier].append(proof)
            q=xyz(records);sample=attrs['near_ir'][(q[:,2]>.12)&(q[:,2]<4.)]
            samples.append(sample[np.linspace(0,len(sample)-1,min(len(sample),4096),dtype=int)] if len(sample) else sample)
            target=work/'attributes'/identifier/(entry['scan']+'.npz');target.parent.mkdir(parents=True,exist_ok=True);np.savez_compressed(target,**attrs)
            prepared[identifier].append((entry,pose,target))
            print('Recovered exact sensor attributes:',identifier,entry['scan'],len(records),flush=True)
    model=fit_leafwood_model(samples);reports={};all_proofs={}
    save_json(stage/'QC/provenance/leafwood_model.json',model)
    for identifier,scan_info in prepared.items():
        print('Independent semantic experiment:',identifier,flush=True)
        directory=stage/'contributions'/identifier/'separate_scans';clouds=[];features=[];per_scan=[];sides=[];origins=[];mapping=[]
        proof=[]
        for entry,pose,target in scan_info:
            _,records=read_records(directory/entry['filename'])
            with np.load(target) as data:attrs={k:data[k] for k in data.files}
            attrs.update(classify_leafwood(attrs['signal'],attrs['near_ir'],model))
            old_digest=coordinate_digest(records);out=enriched_records(records,attrs,np.full(len(records),entry['scan_id'],dtype='<u4'))
            assert coordinate_digest(out)==old_digest
            np.testing.assert_array_equal(out['reflectivity'],records['reflectivity'])
            write_records(directory/entry['filename'],out)
            proof.append(dict(scan_id=entry['scan_id'],coordinate_sha256=old_digest,points=len(records)))
            clouds.append(xyz(records));features.append(attrs);per_scan.append(out);sides.append(entry['scan'].split('_')[0]);origins.append(np.array(pose['sensor_to_merged'])[:3,3])
            mapping.append(dict(scan_id=entry['scan_id'],scan=entry['scan'],source_pcap=str(input_scan(data_directories[identifier],pose).relative_to(data_directories[identifier])),source_pcap_sha256=sha256(input_scan(data_directories[identifier],pose))))
        seeds,held=source_config(stage,identifier)
        corrections,report=register_leafwood(clouds,[a['wood_score'] for a in features],origins,sides,seeds,held_out_stem=held)
        _,control=register_leafwood(clouds,[a['wood_score'] for a in features],origins,sides,seeds,settings={'use_semantics':False},held_out_stem=held)
        report['geometry_only_control']=dict(checks=control['checks'],shape_checks=control['shape_checks'],validation_passed=control['validation_passed'],rejection_reasons=control['rejection_reasons'])
        report['shared_classifier_sha256']=model['model_sha256'];report['source_attribute_verification']=source_proofs[identifier]
        reports[identifier]=report;qc=stage/'QC'/identifier/'leafwood_comparison';qc.mkdir(parents=True,exist_ok=True)
        save_json(qc/'comparison.json',report);save_json(qc/'leafwood_model.json',model)
        attribute_manifest=dict(scans=mapping,scan_id_type='uint32',index_scope='Original PCAP recording within this dataset, not lidar rotation frame',intensity_source='raw SIGNAL / MATLAB SignalPhoton',rgb_source='Derived shared NIR leaf/wood class, fixed SignalPhoton brightness',sensor_return='First return, matching delivered baseline; not MATLAB strongest-return selection')
        save_json(qc/'scan_manifest.json',attribute_manifest)
        save_json(stage/'pointclouds'/identifier/'source_scans.json',attribute_manifest)
        save_json(stage/'pointclouds'/identifier/'leafwood_model.json',model)
        save_json(qc/'baseline_coordinate_verification.json',dict(geometry_and_point_order_unchanged=True,scans=proof))
        # Match each primary point to its SAME source recording. Never transfer
        # attributes from a geometrically adjacent point in another acquisition.
        primary=stage/'pointclouds'/identifier/'final_registered.ply';_,original=read_records(primary)
        attrs_primary={k:np.empty((len(original),3),dtype='u1') if k=='rgb' else np.empty(len(original),dtype=features[0][k].dtype) for k in features[0]}
        source_ids=original['scan_id'].astype('<u4')
        if not set(np.unique(source_ids))<=set(e['scan_id'] for e,_,_ in scan_info):raise ValueError('Unknown source scan index in primary PLY')
        for scan_index,(entry,_,_) in enumerate(scan_info):
            keep=source_ids==entry['scan_id'];distance,ix=cKDTree(clouds[scan_index]).query(xyz(original)[keep],workers=2)
            if len(distance) and distance.max()>3e-6:raise ValueError('Primary points cannot be matched to their original scan')
            np.testing.assert_array_equal(original['reflectivity'][keep],per_scan[scan_index]['reflectivity'][ix])
            for k in attrs_primary:attrs_primary[k][keep]=features[scan_index][k][ix]
        primary_digest=coordinate_digest(original);out=enriched_records(original,attrs_primary,source_ids);assert coordinate_digest(out)==primary_digest
        write_records(primary,out)
        plot_attributes(out,qc/'leafwood_and_scan_index.png',identifier)
        candidate=out.copy()
        for entry,_,_ in scan_info:
            i=entry['scan_id'];keep=source_ids==i;p=transform(xyz(candidate)[keep],corrections[i])
            for j,k in enumerate(['x','y','z']):candidate[k][keep]=p[:,j]
        for k in out.dtype.names:
            if k not in ['x','y','z']:np.testing.assert_array_equal(candidate[k],out[k])
        write_records(qc/'candidate_registered.ply',candidate)
        experimental=dict(matrices);experimental['registration_variant']='Experimental incremental leaf/wood refinement; baseline is primary';experimental['scans']=[]
        for entry,pose,_ in scan_info:
            i=entry['scan_id'];new=dict(pose,scan_id=i)
            for key in ['sensor_to_merged','gravity_levelled_to_merged']:new[key]=(corrections[i]@np.array(pose[key])).tolist()
            experimental['scans'].append(new)
        save_json(qc/'candidate_transformations.json',experimental)
        # Combined scan-coloured cloud intentionally has display RGB by scan;
        # the original derived colour remains in leaf_red/leaf_green/leaf_blue.
        combined=np.concatenate(per_scan);combined_attrs={k:np.concatenate([a[k] for a in features]) for k in features[0]}
        write_records(stage/'contributions'/identifier/'combined/all_scans_coloured_by_recording.ply',enriched_records(combined,combined_attrs,combined['scan_id'],by_scan=True),comment='RGB by recording; original derived leafWoodSignal in leaf_red/leaf_green/leaf_blue; raw intensity retained')
        old=json.loads((directory/'scan_manifest.json').read_text());old.update(dict(leafwood_model_sha256=model['model_sha256'],attribute_manifest='See QC/'+identifier+'/leafwood_comparison/scan_manifest.json',colour_policy='Separate scans use derived leafWoodSignal; combined file uses recording colours and retains leaf_RGB fields'))
        for row,original_source in zip(old['scans'],mapping):row.update(original_source)
        save_json(directory/'scan_manifest.json',old)
        plot_comparison(clouds,features,corrections,seeds,qc/'low_stem_comparison.png',identifier)
        report['baseline_primary_coordinate_sha256']=primary_digest;save_json(qc/'comparison.json',report)
        all_proofs[identifier]=dict(primary_coordinate_sha256=primary_digest,point_count=len(original),separate_scans=proof)
        gallery=stage/'QC'/identifier/'index.html'
        base=gallery.read_text().split('<!-- leafwood-comparison -->')[0].split('<h2>Leaf/wood experiment</h2>')[0]
        gallery.write_text(base+'<!-- leafwood-comparison --><h2>Leaf/wood experiment</h2><p>Baseline remains primary. <a href="leafwood_comparison/comparison.json">Comparison JSON</a></p><img style="max-width:100%" src="leafwood_comparison/leafwood_and_scan_index.png"><img style="max-width:100%" src="leafwood_comparison/low_stem_comparison.png">')
        (stage/'contributions'/identifier/'README.txt').write_text('REGISTERED CONTRIBUTIONS\n\nSeparate scans: original measured points in the baseline frame, derived shared leafWoodSignal RGB, raw sensor channels and original scan_id.\nCombined: same overlaps retained, RGB by recording, with derived leaf/wood RGB preserved separately in leaf_red/leaf_green/leaf_blue.\nscan_colours.csv (where present) is the recording-display palette; use scan_manifest.json for original PCAP names and hashes.\n')
        (stage/'pointclouds'/identifier/'README.txt').write_text('FINAL BASELINE POINT CLOUD\n\nOriginal baseline geometry and sample order, enriched with raw intensity/SIGNAL, NEAR_IR, reflectivity, derived shared leafWoodSignal RGB, and uint32 scan_id.\nAlready registered, metres. Do not apply the matrices again. Mesh (if available) still matches this geometry.\nExperimental registration and comparison are under QC/'+identifier+'/leafwood_comparison, not the primary result.\n')
        marker=stage/'QC'/identifier/'provenance/build.json'
        if marker.exists():
            build=json.loads(marker.read_text());build['attribute_enrichment']=dict(model_sha256=model['model_sha256'],geometry_policy='Original baseline float coordinates and order retained exactly',original_build_source_commit=build.get('data_source_commit'))
            for relative in list(build.get('output_hashes',{})):build['output_hashes'][relative]=sha256(stage/relative)
            save_json(marker,build)
    summarise(reports,model,stage/'QC/leafwood_comparison_report.md')
    save_json(stage/'QC/provenance/leafwood_upgrade.json',dict(version=1,model=model,geometry_verification=all_proofs))
    return reports


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--stage',type=Path,required=True);p.add_argument('--work',type=Path,required=True);p.add_argument('--data-map',type=Path,required=True)
    a=p.parse_args();upgrade_collection(a.stage,json.loads(a.data_map.read_text()),a.work)


if __name__=='__main__':main()
