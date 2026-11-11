"""Apply independent final trunk refinement to a staged collection, preserving baselines."""
from pathlib import Path
import argparse,hashlib,io,json,os,sys,tempfile,zipfile
import numpy as np
ROOT=Path(__file__).resolve().parent.parent
sys.path.insert(0,str(ROOT))
from captures.trunk_refinement import refine_trunks,_apply
from captures.trunk_refinement_qc import plot_slices,plot_summary
from captures.leafwood import write_records,coordinate_digest
from captures.orchard import save_json,sha256
from processing.export_anchored import read_records,xyz,PLY_TYPES
from tools.compare_leafwood import source_config
from tools.capture_collection import directory_hashes


def _read_bytes(data):
    stream=io.BytesIO(data);properties=[];count=None;vertex=False
    for _ in range(250):
        line=stream.readline().decode('ascii');tokens=line.split()
        if tokens[:2]==['element','vertex']:count=int(tokens[2]);vertex=True
        elif tokens[:1]==['element']:vertex=False
        if vertex and tokens[:1]==['property']:
            if len(tokens)!=3 or tokens[1] not in PLY_TYPES:raise ValueError('Unsupported archived PLY')
            properties.append((tokens[2],PLY_TYPES[tokens[1]]))
        if line.strip()=='end_header':break
    if count is None:raise ValueError('Missing archived vertices')
    return np.frombuffer(data,dtype=np.dtype(properties),count=count,offset=stream.tell())


def _raw_hash(records):
    h=hashlib.sha256()
    for name in ['reflectivity','intensity','signal','near_ir','scan_id','frame_id','pixel_index']:
        if name in records.dtype.names:h.update(name.encode());h.update(records[name].astype('<u4').tobytes())
    return h.hexdigest()


def _create_baseline(stage,dataset,archive):
    folder=stage/'contributions'/dataset/'separate_scans';manifest=json.loads((folder/'scan_manifest.json').read_text());primary=stage/'pointclouds'/dataset/'final_registered.ply'
    _,merged=read_records(primary);metadata=dict(version=1,dataset=dataset,primary_sha256=sha256(primary),primary_coordinate_sha256=coordinate_digest(merged),primary_raw_attribute_sha256=_raw_hash(merged),scans=[])
    archive.parent.mkdir(parents=True,exist_ok=True);temporary=archive.with_suffix('.tmp')
    with zipfile.ZipFile(temporary,'w',zipfile.ZIP_DEFLATED,compresslevel=5) as z:
        z.write(primary,'final_registered.ply');z.write(stage/'transformations'/dataset/'transformations.json','transformations.json');z.write(folder/'scan_manifest.json','scan_manifest.json')
        for name in ['leafwood_model.json','source_scans.json']:
            if (primary.parent/name).exists():z.write(primary.parent/name,name)
        for index,entry in enumerate(manifest['scans']):
            if Path(entry['filename']).name!=entry['filename']:raise ValueError('Invalid scan filename')
            _,records=read_records(folder/entry['filename']);buffer=io.BytesIO();np.save(buffer,xyz(records).astype('<f4'),allow_pickle=False);z.writestr(f'scan_xyz/{index:03d}.npy',buffer.getvalue())
            metadata['scans'].append(dict(scan_id=entry['scan_id'],scan=entry['scan'],filename=entry['filename'],points=len(records),coordinate_sha256=coordinate_digest(records),raw_attribute_sha256=_raw_hash(records)))
        z.writestr('baseline.json',json.dumps(metadata,indent=2)+'\n')
        z.writestr('README.txt','PRE-REFINEMENT BASELINE\n\nOriginal primary PLY and matrices are retained exactly. scan_xyz/ contains original per-recording measured coordinates in matching manifest order. Current separate PLYs retain the same raw channels/source IDs and point ordering. Restore their XYZ from these arrays to recover baseline contributions. The archived colour model reproduces the original derived colours if a later classifier changes. No registration is applied when reading this archive.\n')
    with zipfile.ZipFile(temporary) as z:
        if z.testzip() is not None:raise RuntimeError('Baseline archive CRC failed')
    os.replace(temporary,archive)


def _load_baseline(stage,dataset,archive):
    folder=stage/'contributions'/dataset/'separate_scans'
    with zipfile.ZipFile(archive) as z:
        if z.testzip() is not None:raise RuntimeError('Archived baseline corrupted')
        meta=json.loads(z.read('baseline.json'));matrices=json.loads(z.read('transformations.json'));manifest=json.loads(z.read('scan_manifest.json'));merged=_read_bytes(z.read('final_registered.ply')).copy();clouds=[]
        for index,(entry,proof) in enumerate(zip(manifest['scans'],meta['scans'])):
            p=np.load(io.BytesIO(z.read(f'scan_xyz/{index:03d}.npy')),allow_pickle=False)
            _,current=read_records(folder/entry['filename'])
            if len(current)!=len(p) or _raw_hash(current)!=proof['raw_attribute_sha256']:raise RuntimeError('Source identity/order changed since baseline archive: '+entry['scan'])
            clouds.append(p)
    current_primary=stage/'pointclouds'/dataset/'final_registered.ply';_,current=read_records(current_primary)
    if _raw_hash(current)!=meta['primary_raw_attribute_sha256']:raise RuntimeError('Merged source identity/order changed since baseline')
    # Preserve the current colour/classifier attributes even if a later colour
    # model is used. Only historical XYZ is restored for a fresh refinement.
    for name in ['x','y','z']:current[name]=merged[name]
    return clouds,current,matrices,manifest,meta


def _transform_records(records,points,C):
    out=records.copy();q=_apply(points,C)
    out['x']=q[:,0];out['y']=q[:,1]
    # Across/yaw correction preserves Z exactly, including its float32 bits.
    out['z']=points[:,2]
    for name in records.dtype.names:
        if name not in ['x','y']:np.testing.assert_array_equal(out[name],records[name])
    np.testing.assert_allclose(xyz(out),q,rtol=0,atol=1e-6)
    return out


def _merged_candidate(original,corrections,manifest):
    out=original.copy()
    ids=out['scan_id']
    for index,entry in enumerate(manifest['scans']):
        keep=ids==entry['scan_id'];q=_apply(xyz(original)[keep],corrections[index]);out['x'][keep]=q[:,0];out['y'][keep]=q[:,1]
    for name in original.dtype.names:
        if name not in ['x','y']:np.testing.assert_array_equal(out[name],original[name])
    return out


def _export_promoted(stage,dataset,clouds,primary,matrices,manifest,corrections,report,baseline_sha):
    folder=stage/'contributions'/dataset/'separate_scans';new_manifest=json.loads(json.dumps(manifest));new_matrices=json.loads(json.dumps(matrices));lookup={r['scan']:r for r in new_matrices['scans']}
    # Validate and prepare every file before replacing the staged primary data.
    with tempfile.TemporaryDirectory(prefix='trunk-export-') as temporary:
        temporary=Path(temporary);parts={}
        for i,(entry,p,C) in enumerate(zip(manifest['scans'],clouds,corrections)):
            _,records=read_records(folder/entry['filename']);baseline=records.copy()
            for j,name in enumerate(['x','y','z']):baseline[name]=p[:,j]
            refined=_transform_records(baseline,p,C);parts[entry['scan_id']]=(p,xyz(refined))
            write_records(temporary/entry['filename'],refined,comment='Final trunk/post rigid refinement; original raw channels, leafWoodSignal RGB, point order and scan_id retained')
            target=lookup[entry['scan']]
            for key in ['sensor_to_merged','gravity_levelled_to_merged']:target[key]=(C@np.asarray(target[key])).tolist()
            new_manifest['scans'][i].update({key:target[key] for key in ['sensor_to_merged','gravity_levelled_to_merged']})
        refined=_merged_candidate(primary,corrections,manifest);write_records(temporary/'final_registered.ply',refined)
        combined_path=stage/'contributions'/dataset/'combined/all_scans_coloured_by_recording.ply';_,combined=read_records(combined_path)
        for sid,(p,q) in parts.items():
            keep=combined['scan_id']==sid
            if keep.sum()!=len(p):raise RuntimeError('Combined scan contribution count changed')
            combined['x'][keep]=q[:,0];combined['y'][keep]=q[:,1];combined['z'][keep]=p[:,2]
        write_records(temporary/'combined.ply',combined,comment='Final rigid poses; RGB by recording; original leafWoodSignal in leaf_RGB fields; raw channels retained')
        details=dict(method=report['variant'],baseline_archive_sha256=baseline_sha,reference_source='RefManualMeasurements.xlsx; representative population sizes; no tree-number mapping',validation_report='90_QC.zip: QC/'+dataset+'/trunk_refinement/report.json',z_coordinates_unchanged=True,point_order_and_raw_attributes_unchanged=True)
        new_matrices['final_trunk_refinement']=details;new_manifest['final_trunk_refinement']=details
        save_json(temporary/'transformations.json',new_matrices);save_json(temporary/'scan_manifest.json',new_manifest)
        import shutil
        for entry in manifest['scans']:shutil.copy2(temporary/entry['filename'],folder/entry['filename'])
        shutil.copy2(temporary/'final_registered.ply',stage/'pointclouds'/dataset/'final_registered.ply');shutil.copy2(temporary/'combined.ply',combined_path)
        shutil.copy2(temporary/'transformations.json',stage/'transformations'/dataset/'transformations.json');shutil.copy2(temporary/'scan_manifest.json',folder/'scan_manifest.json')
    return dict(primary_points=len(refined),primary_coordinate_sha256=coordinate_digest(refined),raw_attribute_sha256=_raw_hash(refined),primary_sha256=sha256(stage/'pointclouds'/dataset/'final_registered.ply'))


def _dataset_hashes(stage,dataset):
    return {folder+'/'+dataset+'/'+relative:sha for folder in ['pointclouds','transformations','contributions'] for relative,sha in directory_hashes(stage/folder/dataset).items()}


def refine_collection(stage,config):
    stage=Path(stage);references=json.loads((ROOT/'presets/manual_measurements.json').read_text());reports={};results=[]
    code_files=['captures/trunk_refinement.py','captures/trunk_refinement_qc.py','tools/refine_trunk_collection.py','presets/manual_measurements.json','tools/compare_leafwood.py','captures/orchard.py','captures/leafwood.py','processing/export_anchored.py','requirements-captures.txt','requirements.txt','requirements-meshing.txt']
    code=''.join(name+sha256(ROOT/name) for name in code_files)
    for dataset in [d['id'] for d in config['datasets']]:
        qc=stage/'QC'/dataset/'trunk_refinement';qc.mkdir(parents=True,exist_ok=True);archive=stage/'QC'/dataset/'provenance/trunk_refinement_baseline.zip'
        if not archive.exists():_create_baseline(stage,dataset,archive)
        baseline_sha=sha256(archive);classifier=stage/'pointclouds'/dataset/'leafwood_model.json';seed_file=ROOT/('presets/orchard_stem_candidates.json' if dataset=='chandelier' else 'presets/captures/'+dataset+'.json')
        key=hashlib.sha256((code+baseline_sha+sha256(classifier)+sha256(seed_file)).encode()).hexdigest();marker=qc/'build.json'
        if marker.exists():
            old=json.loads(marker.read_text())
            if old['cache_key']==key:
                for relative,sha in old['output_hashes'].items():
                    if sha256(stage/relative)!=sha:raise RuntimeError('Refinement output changed since verified build: '+relative)
                report=json.loads((qc/'report.json').read_text());reports[dataset]=report;results.append(old);print('Reusing verified trunk refinement:',dataset,flush=True);continue
        print('Final trunk refinement:',dataset,flush=True)
        clouds,primary,matrices,manifest,baseline_meta=_load_baseline(stage,dataset,archive);lookup={r['scan']:r for r in matrices['scans']};origins=[np.asarray(lookup[e['scan']]['sensor_to_merged'])[:3,3] for e in manifest['scans']];sides=[e['scan'].split('_')[0] for e in manifest['scans']];seeds,_=source_config(stage,dataset)
        family='spindle' if dataset.startswith('spindle') else dataset
        try:corrections,report=refine_trunks(clouds,origins,sides,seeds,references['families'][family])
        except ValueError as error:
            corrections=np.tile(np.eye(4),(len(clouds),1,1));report=dict(validation_passed=False,rejection_reasons=[str(error)],validation={},trunk_models={},variant='Unsupported independent trunk refinement',object_observations=[])
        report.update(dataset=dataset,manual_reference_source_sha256=references['source_sha256'],baseline_archive_sha256=baseline_sha,baseline_primary_sha256=baseline_meta['primary_sha256'])
        promoted=bool(report['validation_passed'])
        if promoted and (stage/'meshes'/dataset).is_dir():
            promoted=False;report['promotion_note']='Validated candidate retained in QC: existing mesh derives from baseline and must be rebuilt independently before replacing this meshed dataset.'
        report['promoted']=promoted
        plot_slices(clouds,sides,seeds,corrections,report,qc,dataset)
        save_json(qc/'corrections.json',dict(dataset=dataset,convention='Final correction in the already registered baseline frame; column vectors. Apply once only.',baseline_archive_sha256=baseline_sha,corrections=corrections.tolist(),scan_ids=[e['scan_id'] for e in manifest['scans']]))
        if promoted:
            export=_export_promoted(stage,dataset,clouds,primary,matrices,manifest,corrections,report,baseline_sha);report['export']=export
            (stage/'pointclouds'/dataset/'README.txt').write_text('FINAL REFINED POINT CLOUD — METRES\n\nFinal trunk/post across translation and yaw refinement applied to the previous registered baseline. Coordinates are already registered; do not apply the matrices again. Original measured samples, point order, height values, raw sensor channels, source IDs and shared Lab leafWoodSignal colours are retained.\nThe baseline primary cloud/matrices and original separate-scan XYZ are preserved in 90_QC.zip under QC/'+dataset+'/provenance/trunk_refinement_baseline.zip. Before/after slices and conditional held-out checks are under QC/'+dataset+'/trunk_refinement/.\n')
        else:write_records(qc/'candidate_registered.ply',_merged_candidate(primary,corrections,manifest),comment='Flagged or unpromoted candidate; primary final result remains the previous registered baseline')
        save_json(qc/'report.json',report);reports[dataset]=report
        gallery=stage/'QC'/dataset/'index.html';base=gallery.read_text().split('<!-- final-trunk-refinement -->')[0]
        gallery.write_text(base+'<!-- final-trunk-refinement --><h2>Final trunk/post refinement — '+('primary result' if promoted else 'flagged candidate; baseline retained')+'</h2><p><a href="trunk_refinement/report.json">Report</a> · <a href="trunk_refinement/slices_30cm.pdf">30 cm before/after PDF</a></p>'+''.join('<img style="max-width:100%" src="trunk_refinement/slices_'+str(h)+'cm.png">' for h in [18,30,40]))
        hashes=_dataset_hashes(stage,dataset);build=dict(dataset=dataset,cache_key=key,promoted=promoted,validation_passed=report['validation_passed'],baseline_archive_sha256=baseline_sha,points=len(primary),output_hashes=hashes)
        save_json(marker,build);results.append(build)
        capture_marker=stage/'QC'/dataset/'provenance/build.json'
        if capture_marker.exists():
            capture=json.loads(capture_marker.read_text());capture['output_hashes']=hashes;capture['final_trunk_refinement']=dict(promoted=promoted,baseline_archive_sha256=baseline_sha,cache_key=key);save_json(capture_marker,capture)
        print(dataset,'trunk refinement promoted',promoted,'flags',report['rejection_reasons'],flush=True)
    # Preserve the independent colour-cache contract after authorised geometry
    # changes. It must verify/reuse current attributes rather than re-register.
    colour_marker=stage/'QC/provenance/leafwood_upgrade.json'
    if colour_marker.exists():
        from tools.leafwood_collection import output_hashes
        colour=json.loads(colour_marker.read_text());colour['output_hashes']=output_hashes(stage,config);colour['final_trunk_refinement_note']='Primary spindle coordinates may now include a verified independent final correction; original baseline digests/measurements remain in each baseline ZIP.';save_json(colour_marker,colour)
    save_json(stage/'QC/trunk_refinement_summary.json',dict(reference_source_sha256=references['source_sha256'],datasets={name:{key:report.get(key) for key in ['promoted','validation_passed','rejection_reasons','validation','ground_validation','along_row_displacement','unchanged_unsupported_scan_ids']} for name,report in reports.items()}))
    plot_summary(reports,stage/'QC/trunk_refinement_summary.png')
    return results


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--stage',type=Path,required=True);args=parser.parse_args();refine_collection(args.stage,json.loads((ROOT/'results/capture_collection.json').read_text()))
