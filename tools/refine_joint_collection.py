"""Independent final joint pose stage; immutable pre-stage baselines prevent double application."""
from pathlib import Path
import argparse,hashlib,json,os,shutil,sys,tempfile
import numpy as np
ROOT=Path(__file__).resolve().parent.parent
sys.path.insert(0,str(ROOT))
from captures.joint_registration import refine_joint,apply
from captures.joint_registration_qc import plot_joint
from captures.leafwood import write_records,coordinate_digest
from captures.orchard import save_json,sha256
from processing.export_anchored import read_records,xyz
from tools.refine_trunk_collection import _create_baseline,_load_baseline,_raw_hash,_dataset_hashes
from tools.compare_leafwood import source_config


def transform_records(records,points,C):
    output=records.copy();q=apply(points,C)
    for j,name in enumerate(['x','y','z']):output[name]=q[:,j]
    for name in records.dtype.names:
        if name not in ['x','y','z']:np.testing.assert_array_equal(output[name],records[name])
    np.testing.assert_allclose(xyz(output),q,rtol=0,atol=2e-6)
    return output


def export_joint(stage,dataset,clouds,primary,matrices,manifest,corrections,report,baseline_sha):
    stage=Path(stage);folder=stage/'contributions'/dataset/'separate_scans'
    new_matrices=json.loads(json.dumps(matrices));new_manifest=json.loads(json.dumps(manifest));lookup={x['scan']:x for x in new_matrices['scans']}
    with tempfile.TemporaryDirectory(prefix='joint-export-') as temp:
        temp=Path(temp);parts={}
        for i,(entry,p,C) in enumerate(zip(manifest['scans'],clouds,corrections)):
            _,records=read_records(folder/entry['filename']);baseline=records.copy()
            for j,name in enumerate(['x','y','z']):baseline[name]=p[:,j]
            output=transform_records(baseline,p,C);write_records(temp/entry['filename'],output,comment='Final joint rigid poses; original raw channels, derived RGB and source IDs retained')
            _,saved=read_records(temp/entry['filename'])
            for name in records.dtype.names:
                if name not in ['x','y','z']:np.testing.assert_array_equal(saved[name],records[name])
            target=lookup[entry['scan']]
            for key in ['sensor_to_merged','gravity_levelled_to_merged']:target[key]=(C@np.asarray(target[key])).tolist()
            new_manifest['scans'][i].update({key:target[key] for key in ['sensor_to_merged','gravity_levelled_to_merged']})
            # Independently reconstruct through the original sensor frame.
            sensor=apply(p,np.linalg.inv(np.array(matrices['scans'][i]['sensor_to_merged'])))
            np.testing.assert_allclose(apply(sensor,np.array(target['sensor_to_merged'])),xyz(saved),rtol=0,atol=2e-6)
            parts[entry['scan_id']]=(p,xyz(saved),records)
        merged=primary.copy()
        for i,entry in enumerate(manifest['scans']):
            keep=primary['scan_id']==entry['scan_id'];q=apply(xyz(primary)[keep],corrections[i])
            for j,name in enumerate(['x','y','z']):merged[name][keep]=q[:,j]
        for name in primary.dtype.names:
            if name not in ['x','y','z']:np.testing.assert_array_equal(merged[name],primary[name])
        write_records(temp/'final_registered.ply',merged,comment='Final joint rigid registration; original measured samples, RGB, raw channels and scan_id preserved')
        combined_path=stage/'contributions'/dataset/'combined/all_scans_coloured_by_recording.ply';_,combined=read_records(combined_path)
        combined_original=combined.copy()
        for sid,(p,q,records) in parts.items():
            keep=combined['scan_id']==sid
            if keep.sum()!=len(p):raise RuntimeError('Combined recording point count changed')
            for name in ['signal','near_ir','frame_id','pixel_index']:
                if name in records.dtype.names:np.testing.assert_array_equal(combined[name][keep],records[name])
            for j,name in enumerate(['x','y','z']):combined[name][keep]=q[:,j]
        for name in combined.dtype.names:
            if name not in ['x','y','z']:np.testing.assert_array_equal(combined[name],combined_original[name])
        write_records(temp/'combined.ply',combined,comment='Final joint rigid poses; RGB by recording, original leaf_RGB and raw channels retained')
        details=dict(method=report['method'],baseline_archive_sha256=baseline_sha,
            verification='One composed rigid matrix per recording; unchanged sample identities, point order, RGB and all non-coordinate fields',
            validation_report='90_QC.zip: QC/'+dataset+'/joint_registration/report.json',
            classification_note='The legacy height-band classification field is retained; it is not leaf/wood ground truth.')
        new_matrices['final_joint_registration']=details;new_manifest['final_joint_registration']=details
        save_json(temp/'transformations.json',new_matrices);save_json(temp/'scan_manifest.json',new_manifest)
        for entry in manifest['scans']:shutil.copy2(temp/entry['filename'],folder/entry['filename'])
        shutil.copy2(temp/'final_registered.ply',stage/'pointclouds'/dataset/'final_registered.ply');shutil.copy2(temp/'combined.ply',combined_path)
        shutil.copy2(temp/'transformations.json',stage/'transformations'/dataset/'transformations.json');shutil.copy2(temp/'scan_manifest.json',folder/'scan_manifest.json')
    return dict(points=len(merged),primary_coordinate_sha256=coordinate_digest(merged),raw_attribute_sha256=_raw_hash(merged),
        primary_sha256=sha256(stage/'pointclouds'/dataset/'final_registered.ply'),non_coordinate_attributes_unchanged=True)


def _identity():
    names=['captures/joint_registration.py','captures/terrain.py','captures/recover_views.py',
        'captures/joint_landmarks.py','captures/overlap.py','captures/structural_validation.py',
        'captures/joint_registration_qc.py','tools/refine_joint_collection.py',
        'tools/refine_trunk_collection.py','presets/joint_registration.json',
        'presets/manual_measurements.json','requirements-captures.txt','requirements.txt']
    return hashlib.sha256(''.join(name+sha256(ROOT/name) for name in names).encode()).hexdigest()


def _update_prior_caches(stage,dataset):
    hashes=_dataset_hashes(stage,dataset)
    for relative in ['provenance/build.json','trunk_refinement/build.json']:
        marker=stage/'QC'/dataset/relative
        if marker.exists():
            old=json.loads(marker.read_text());old['output_hashes']=hashes
            old['final_joint_registration_note']='Primary poses now include the subsequent independent joint stage. Earlier baseline and diagnostics remain in QC.';save_json(marker,old)
    marker=stage/'QC/provenance/leafwood_upgrade.json'
    if marker.exists():
        from tools.leafwood_collection import output_hashes
        config=json.loads((ROOT/'results/capture_collection.json').read_text());old=json.loads(marker.read_text());old['output_hashes']=output_hashes(stage,config);save_json(marker,old)


def refine_joint_collection(stage,preset=None):
    stage=Path(stage);preset=preset or json.loads((ROOT/'presets/joint_registration.json').read_text());references=json.loads((ROOT/'presets/manual_measurements.json').read_text());code=_identity()+json.dumps(preset,sort_keys=True);summary=[]
    for dataset in preset['datasets']:
        qc=stage/'QC'/dataset/'joint_registration';qc.mkdir(parents=True,exist_ok=True)
        promote=dataset in preset['promote_datasets'];archive=stage/'QC'/dataset/'provenance/joint_registration_baseline.zip'
        if promote and not archive.exists():_create_baseline(stage,dataset,archive)
        primary_path=stage/'pointclouds'/dataset/'final_registered.ply';classifier=primary_path.parent/'leafwood_model.json'
        source_sha=sha256(archive) if promote else sha256(primary_path)
        seed_file=ROOT/('presets/orchard_stem_candidates.json' if dataset=='chandelier' else 'presets/captures/'+dataset+'.json')
        key=hashlib.sha256((code+source_sha+sha256(classifier)+sha256(seed_file)).encode()).hexdigest();marker=qc/'build.json'
        if marker.exists():
            old=json.loads(marker.read_text())
            if old['cache_key']==key:
                for relative,digest in old['output_hashes'].items():
                    if sha256(stage/relative)!=digest:raise RuntimeError('Joint output changed after verified build: '+relative)
                summary.append(old);print('Reusing verified joint stage:',dataset,flush=True);continue
        folder=stage/'contributions'/dataset/'separate_scans'
        if promote:
            clouds,primary,matrices,manifest,_=_load_baseline(stage,dataset,archive)
        else:
            manifest=json.loads((folder/'scan_manifest.json').read_text());matrices=json.loads((stage/'transformations'/dataset/'transformations.json').read_text());clouds=[xyz(read_records(folder/e['filename'])[1]) for e in manifest['scans']];_,primary=read_records(primary_path)
        lookup={x['scan']:x for x in matrices['scans']};scores=[];origins=[];ups=[];labels=[]
        for entry in manifest['scans']:
            _,records=read_records(folder/entry['filename']);scores.append(records['wood_score']);pose=lookup[entry['scan']];origins.append(np.array(pose['sensor_to_merged'])[:3,3]);ups.append(np.array(pose['gravity_levelled_to_merged'])[:3,2]);labels.append(entry['scan'].split('_')[0])
        seeds,_=source_config(stage,dataset);family='spindle' if dataset.startswith('spindle') else dataset
        quality_path=stage/'QC'/dataset/'registration_quality.json'
        if not quality_path.exists():quality_path=stage/'QC'/dataset/'registration/registration_quality.json'
        quality=json.loads(quality_path.read_text()) if quality_path.exists() else {};edges=quality.get('initial',{}).get('edges')
        print('Joint registration:',dataset,flush=True)
        try:
            corrections,report=refine_joint(clouds,scores,origins,ups,labels,seeds,references['families'][family],edges,preset.get('settings'))
        except ValueError as error:
            corrections=np.tile(np.eye(4),(len(clouds),1,1));report=dict(method='Independent joint rigid registration',validation_passed=False,rejection_reasons=[str(error)],recoveries=[],retained_landmarks=[],excluded_ambiguous_training_landmarks=[],anchor_checks=[])
        promoted=bool(promote and report['validation_passed'] and not (stage/'meshes'/dataset).is_dir())
        report.update(dataset=dataset,promoted=promoted,reference_source_sha256=references['source_sha256'])
        if report['validation_passed'] and not promote:report['promotion_note']='QC comparison only: accepted spindle and meshed chandelier outputs are retained by the explicit dataset selection preset.'
        if promote and (stage/'meshes'/dataset).is_dir():report['promotion_note']='Existing mesh requires an independent rebuild before replacing primary geometry.'
        plot_joint(clouds,labels,seeds,corrections,report,qc,dataset)
        if promoted:
            report['export']=export_joint(stage,dataset,clouds,primary,matrices,manifest,corrections,report,source_sha)
            (primary_path.parent/'README.txt').write_text('FINAL JOINT REGISTERED POINT CLOUD — METRES\n\nCoordinates already include the final rigid poses. Do not apply transformations.json again.\nBoth sides can adjust in the joint stage; measured terrain may slope and trunks may lean.\nOriginal measured samples, point order, intensity/SIGNAL, NIR, reflectivity, derived leaf/wood RGB and source IDs are retained.\nThe pre-joint baseline is archived inside 90_QC.zip under QC/'+dataset+'/provenance/joint_registration_baseline.zip. Figures and checks are under QC/'+dataset+'/joint_registration/.\n')
            _update_prior_caches(stage,dataset)
        save_json(qc/'report.json',report);save_json(qc/'corrections.json',dict(dataset=dataset,convention='Apply once to the archived pre-joint baseline; final PLYs already contain these poses',corrections=corrections.tolist(),scan_ids=[e['scan_id'] for e in manifest['scans']]))
        gallery=stage/'QC'/dataset/'index.html';text=gallery.read_text();start='<!-- final-joint-start -->';end='<!-- final-joint-end -->'
        if start in text:text=text[:text.index(start)]+text[text.index(end)+len(end):]
        block=start+'<h2>Final joint registration — '+('current primary result' if promoted else 'QC comparison; primary retained')+'</h2><p><a href="joint_registration/report.json">Report</a> · <a href="joint_registration/slices_30cm.pdf">30 cm slice PDF</a></p><img style="max-width:100%" src="joint_registration/slices_30cm.png"><img style="max-width:100%" src="joint_registration/profiles.png">'+end
        position=text.find('</h1>');position=position+5 if position>=0 else len(text);gallery.write_text(text[:position]+block+text[position:])
        hashes=_dataset_hashes(stage,dataset);record=dict(dataset=dataset,cache_key=key,promoted=promoted,validation_passed=report['validation_passed'],rejection_reasons=report['rejection_reasons'],points=len(primary),output_hashes=hashes,
            stem_validation=report.get('stem_validation'),ground_validation=report.get('ground_validation'),anchor_checks=report.get('anchor_checks'),recovered_scan_ids=[r['scan_id'] for r in report.get('recoveries',[]) if r.get('accepted')])
        save_json(marker,record);summary.append(record);print(dataset,'joint promoted',promoted,'flags',report['rejection_reasons'],flush=True)
    save_json(stage/'QC/joint_registration_summary.json',dict(datasets=[{k:v for k,v in item.items() if k not in ['output_hashes','cache_key']} for item in summary]))
    return summary


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--stage',required=True,type=Path);args=parser.parse_args();refine_joint_collection(args.stage)
