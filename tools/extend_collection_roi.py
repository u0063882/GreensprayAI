"""Re-export wider measured ROIs through accepted poses, then mesh the PLY independently."""
from pathlib import Path
import csv,hashlib,io,json,shutil,sys,tempfile,zipfile
import numpy as np
from scipy.spatial import cKDTree
ROOT=Path(__file__).resolve().parent.parent
sys.path.insert(0,str(ROOT))
from captures.leafwood import decode_attributes,classify_leafwood,write_records
from captures.orchard import sha256,save_json,transform
from processing.export_anchored import read_records,xyz
from tools.refine_trunk_collection import _read_bytes,_raw_hash,_create_baseline,_dataset_hashes
from tools.capture_collection import directory_hashes,input_archives,extract_inputs
from tools.compare_leafwood import input_scan


def _snapshot(stage,dataset,path):
    path.parent.mkdir(parents=True,exist_ok=True)
    with zipfile.ZipFile(path,'w',zipfile.ZIP_DEFLATED,compresslevel=5) as archive:
        for folder in ['pointclouds','contributions','transformations','meshes']:
            root=stage/folder/dataset
            for p in sorted(root.rglob('*')):
                if p.is_file():archive.write(p,p.relative_to(stage).as_posix())
        for relative in ['meshing','provenance/trunk_refinement_baseline.zip']:
            p=stage/'QC'/dataset/relative
            if p.is_dir():
                for f in sorted(p.rglob('*')):
                    if f.is_file():archive.write(f,f.relative_to(stage).as_posix())
            elif p.is_file():archive.write(p,p.relative_to(stage).as_posix())
    with zipfile.ZipFile(path) as archive:
        if archive.testzip() is not None:raise RuntimeError('Original ROI snapshot failed CRC validation')


def _original(stage,dataset,archive):
    folder=stage/'contributions'/dataset/'separate_scans'
    with zipfile.ZipFile(archive) as z:
        matrices=json.loads(z.read(f'transformations/{dataset}/transformations.json'));manifest=json.loads(z.read(f'contributions/{dataset}/separate_scans/scan_manifest.json'))
        old_primary=_read_bytes(z.read(f'pointclouds/{dataset}/final_registered.ply')).copy();_,current=read_records(stage/'pointclouds'/dataset/'final_registered.ply');current=current[:len(old_primary)].copy()
        if _raw_hash(current)!=_raw_hash(old_primary):raise RuntimeError('Original ROI primary prefix/order changed')
        for key in ['x','y','z']:current[key]=old_primary[key]
        combined=_read_bytes(z.read(f'contributions/{dataset}/combined/all_scans_coloured_by_recording.ply'))
        palettes={}
        for entry in manifest['scans']:
            mask=combined['scan_id']==entry['scan_id'];rgb=np.c_[combined['red'][mask],combined['green'][mask],combined['blue'][mask]]
            if len(rgb) and not np.all(rgb==rgb[0]):raise RuntimeError('Original combined RGB is not uniform by recording')
            palettes[entry['scan_id']]=rgb[0].tolist()
        records=[]
        for entry in manifest['scans']:
            old=_read_bytes(z.read(f'contributions/{dataset}/separate_scans/'+entry['filename']));_,now=read_records(folder/entry['filename']);now=now[:len(old)].copy()
            if _raw_hash(now)!=_raw_hash(old):raise RuntimeError('Original scan ROI prefix/order changed')
            for key in ['x','y','z']:now[key]=old[key]
            records.append(now)
    return current,records,matrices,manifest,palettes


def extended_scan(original,decoded,pose,scan_id,palette,model,roi):
    p=transform(decoded['xyz'],np.array(pose['gravity_levelled_to_merged']));x,y,z=p.T
    old=roi['original_along_m'];keep=((x<old[0]-2e-6)|(x>old[1]+2e-6))&(x>roi['along_m'][0])&(x<roi['along_m'][1])&(y>roi['across_m'][0])&(y<roi['across_m'][1])&(z>roi['height_m'][0])&(z<roi['height_m'][1]);ix=np.flatnonzero(keep)
    extra=np.zeros(len(ix),dtype=original.dtype)
    for j,key in enumerate(['x','y','z']):extra[key]=p[ix,j]
    for key in ['reflectivity','signal','near_ir','frame_id','pixel_index']:extra[key]=decoded[key][ix]
    extra['intensity']=extra['signal'];extra['scan_id']=scan_id
    if 'classification' in extra.dtype.names:extra['classification']=np.where(extra['z']<.12,2,5)
    attrs=classify_leafwood(extra['signal'],extra['near_ir'],model)
    for key in ['leafwood_class','wood_score','leafwood_confidence']:
        if key in extra.dtype.names:extra[key]=attrs[key]
    semantic_rgb=all('leaf_'+key in original.dtype.names and np.array_equal(original[key],original['leaf_'+key]) for key in ['red','green','blue'])
    for j,key in enumerate(['red','green','blue']):
        extra[key]=attrs['rgb'][:,j] if semantic_rgb else palette[j]
        if 'leaf_'+key in extra.dtype.names:extra['leaf_'+key]=attrs['rgb'][:,j]
    old_ids=(original['frame_id'].astype('u8')<<32)|original['pixel_index'];new_ids=(extra['frame_id'].astype('u8')<<32)|extra['pixel_index']
    if len(np.intersect1d(old_ids,new_ids)):raise RuntimeError('Extended ROI duplicates an existing measured return')
    expanded=np.concatenate([original,extra]);np.testing.assert_array_equal(expanded[:len(original)],original)
    return expanded,extra,dict(added=len(extra),maximum_source_coordinate_error_m=float(np.max(np.linalg.norm(xyz(extra)-p[ix],axis=1))) if len(extra) else 0.)


def merge_extension(original,extras):
    extra=np.concatenate(extras);points=xyz(extra);old=xyz(original)
    # Keep every old measured sample, then first measured sample in each new cell.
    cells=np.floor(np.concatenate([old,points])/.012).astype(np.int64);_,first=np.unique(cells,axis=0,return_index=True);ix=first[first>=len(original)]-len(original);extra=extra[ix]
    if len(extra):
        combined=np.concatenate([old,xyz(extra)]);counts=cKDTree(combined).query_ball_point(xyz(extra),.06,return_length=True,workers=2);extra=extra[counts>=5]
    for key in ['red','green','blue']:
        if 'leaf_'+key in extra.dtype.names:extra[key]=extra['leaf_'+key]
    result=np.concatenate([original,extra]);np.testing.assert_array_equal(result[:len(original)],original)
    return result


def _refresh_caches(stage,dataset):
    hashes=_dataset_hashes(stage,dataset)
    for relative in ['provenance/build.json','trunk_refinement/build.json']:
        p=stage/'QC'/dataset/relative
        if p.exists():
            data=json.loads(p.read_text());data['output_hashes']=hashes;data['roi_extension_note']='Measured X-end returns added through unchanged accepted poses; original crop archived in roi_extension_baseline.zip';save_json(p,data)
    p=stage/'QC/provenance/leafwood_upgrade.json'
    if p.exists():
        from tools.leafwood_collection import output_hashes
        config=json.loads((ROOT/'results/capture_collection.json').read_text());data=json.loads(p.read_text());data['output_hashes']=output_hashes(stage,config);save_json(p,data)
    # The trunk stage's immutable baseline must describe the same expanded source
    # return sets on a future rebuild. Its exact former archive is in the ROI ZIP.
    archive=stage/'QC'/dataset/'provenance/trunk_refinement_baseline.zip'
    archive.unlink(missing_ok=True);_create_baseline(stage,dataset,archive)


def extend_collection_roi(stage,work,collection,repo,data_directories=None,decoded_directories=None):
    stage=Path(stage);work=Path(work);work.mkdir(parents=True,exist_ok=True);presets=json.loads((ROOT/'presets/roi_extensions.json').read_text());results=[]
    files=['tools/extend_collection_roi.py','captures/leafwood.py','captures/leafwood_lab.py','presets/roi_extensions.json','processing/ground_surface.py','processing/shrinkwrap.py','requirements-meshing.txt']
    code=hashlib.sha256(''.join(name+sha256(ROOT/name) for name in files).encode()).hexdigest()
    for dataset,roi in presets.items():
        qc=stage/'QC'/dataset/'roi_extension';qc.mkdir(parents=True,exist_ok=True);archive=stage/'QC'/dataset/'provenance/roi_extension_baseline.zip'
        if not archive.exists():_snapshot(stage,dataset,archive)
        model_path=stage/'pointclouds'/dataset/'leafwood_model.json';key=hashlib.sha256((code+sha256(archive)+sha256(model_path)).encode()).hexdigest();marker=qc/'build.json'
        if marker.exists():
            previous=json.loads(marker.read_text())
            if previous['cache_key']==key:
                for relative,digest in previous['output_hashes'].items():
                    if sha256(stage/relative)!=digest:raise RuntimeError('Expanded ROI output changed: '+relative)
                results.append(previous);print('Reusing verified measured ROI extension:',dataset,flush=True);continue
        print('Recovering measured ROI ends:',dataset,flush=True)
        if data_directories and dataset in data_directories:data=Path(data_directories[dataset])
        else:
            definition=next(d for d in collection['datasets'] if d['id']==dataset);workspace=work/dataset;workspace.mkdir(parents=True,exist_ok=True);inputs=input_archives(definition,stage/'QC'/dataset,workspace,repo);extract_inputs(inputs,workspace);data=workspace/'work/data'
        decoded_root=Path(decoded_directories[dataset]) if decoded_directories and dataset in decoded_directories else work/dataset/'decoded'
        primary,originals,matrices,manifest,palettes=_original(stage,dataset,archive);lookup={p['scan']:p for p in matrices['scans']};model=json.loads(model_path.read_text());expanded=[];extras=[];proof=[]
        for original,entry in zip(originals,manifest['scans']):
            pose=lookup[entry['scan']];path=input_scan(data,pose);rotation=np.linalg.solve(np.array(pose['gravity_levelled_to_merged'])[:3,:3],np.array(pose['sensor_to_merged'])[:3,:3]);decoded=decode_attributes(path,rotation,decoded_root/(entry['scan']+'.npz'))
            palette=entry.get('rgb',[int(original[k][0]) for k in ['red','green','blue']]);out,extra,check=extended_scan(original,decoded,pose,entry['scan_id'],palette,model,roi)
            expanded.append(out);extras.append(extra);proof.append(dict(scan_id=entry['scan_id'],source_pcap_sha256=sha256(path),**check));entry['point_count']=len(out);lookup[entry['scan']]['point_count']=len(out)
        merged=merge_extension(primary,extras);manifest['total_points']=sum(len(p) for p in expanded);manifest['crop']=dict(x=roi['along_m'],y=roi['across_m'],z=roi['height_m'])
        details=dict(original_along_m=roi['original_along_m'],along_m=roi['along_m'],across_m=roi['across_m'],height_m=roi['height_m'],original_primary_points=len(primary),final_primary_points=len(merged),added_primary_points=len(merged)-len(primary),poses_unchanged=True,old_primary_prefix_unchanged=True,source_policy='Additional first-return measured samples through accepted poses; no interpolation, padding or shape extrapolation',baseline_archive_sha256=sha256(archive))
        matrices['measured_roi_extension']=details;manifest['measured_roi_extension']=details
        matrices['registration']=matrices.get('registration','Accepted rigid poses')+'; measured X-end ROI expanded through unchanged poses'
        manifest['source_crop_limit']='Original PCAP returns re-decoded to extend the former X crop; only measured coverage is available.'
        with tempfile.TemporaryDirectory(prefix='expanded-roi-',dir=work) as temporary:
            temporary=Path(temporary);cloud_path=temporary/'final_registered.ply';write_records(cloud_path,merged,comment='Accepted rigid poses; original samples retained, measured X-end canopy/ground returns restored')
            meshdir=temporary/'mesh';meshdir.mkdir()
            import open3d as o3d
            from processing.ground_surface import observed_ground
            soil=observed_ground(xyz(merged));o3d.io.write_triangle_mesh(str(meshdir/'ground_surface.ply'),soil)
            from processing.shrinkwrap import main as mesh_pointcloud
            settings=roi['mesh'];mesh_pointcloud(['--input',str(cloud_path),'--output-dir',str(meshdir),'--ground',str(meshdir/'ground_surface.ply'),'--archive',str(temporary/'mesh.zip'),'--voxel',str(settings['voxel_m']),'--offset',str(settings['outward_offset_m']),'--closing-radius',str(settings['closing_radius_m']),'--min-volume',str(settings['minimum_volume_m3'])])
            mesh_quality=json.loads((meshdir/'shrinkwrap_quality.json').read_text())
            if not mesh_quality['watertight'] or mesh_quality['source_sha256']!=sha256(cloud_path):raise RuntimeError('Expanded mesh failed topology or point-cloud source-hash checks')
            combined_parts=[]
            for entry,records in zip(manifest['scans'],expanded):
                part=records.copy()
                for j,key in enumerate(['red','green','blue']):part[key]=palettes[entry['scan_id']][j]
                combined_parts.append(part)
            combined=np.concatenate(combined_parts);write_records(temporary/'combined.ply',combined,comment='Registered contributions with measured ROI extension; RGB by scan, original leaf_RGB retained')
            save_json(temporary/'transformations.json',matrices);save_json(temporary/'scan_manifest.json',manifest)
            # All geometry is ready and validated before replacing staged outputs.
            folder=stage/'contributions'/dataset/'separate_scans'
            for entry,records in zip(manifest['scans'],expanded):write_records(folder/entry['filename'],records)
            shutil.copy2(cloud_path,stage/'pointclouds'/dataset/'final_registered.ply');shutil.copy2(temporary/'combined.ply',stage/'contributions'/dataset/'combined/all_scans_coloured_by_recording.ply');shutil.copy2(temporary/'transformations.json',stage/'transformations'/dataset/'transformations.json');shutil.copy2(temporary/'scan_manifest.json',folder/'scan_manifest.json')
            mesh_output=stage/'meshes'/dataset;mesh_output.mkdir(parents=True,exist_ok=True)
            for name in ['canopy_wrap.ply','canopy_wrap.stl','canopy_wrap.obj','canopy_wrap_with_ground.ply','ground_surface.ply','README.txt']:shutil.copy2(meshdir/name,mesh_output/name)
            for name in ['shrinkwrap_quality.json','shrinkwrap_preview.png']:
                shutil.copy2(meshdir/name,qc/name);shutil.copy2(meshdir/name,stage/'QC'/dataset/'meshing'/name)
        (stage/'pointclouds'/dataset/'README.txt').write_text('FINAL CHANDELIER POINT CLOUD — METRES\n\nAccepted rigid poses are unchanged. Original points and attributes are retained; measured canopy and ground returns extend the X ROI to -1.25…7.35 m. No synthetic padding was added.\nThe independently rebuilt mesh uses this exact PLY. Matrices are already applied to point coordinates.\nThe complete former crop, contributions, matrices and mesh are archived inside 90_QC.zip: QC/chandelier/provenance/roi_extension_baseline.zip.\n')
        counts={int(e['scan_id']):e['point_count'] for e in manifest['scans']}
        for part in ['separate_scans','combined']:
            directory=stage/'contributions'/dataset/part
            colour_csv=directory/'scan_colours.csv'
            if colour_csv.exists():
                with colour_csv.open(newline='') as stream:
                    reader=csv.DictReader(stream);fieldnames=reader.fieldnames;rows=list(reader)
                for row in rows:
                    if 'point_count' in row:row['point_count']=str(counts[int(row['scan_id'])])
                with colour_csv.open('w',newline='') as stream:
                    writer=csv.DictWriter(stream,fieldnames=fieldnames,lineterminator='\n');writer.writeheader();writer.writerows(rows)
            (directory/'README.txt').write_text('FINAL REGISTERED CONTRIBUTIONS — METRES\n\nAccepted poses are already applied. The measured X ROI is now -1.25…7.35 m; original samples are retained.\nSeparate PLY RGB is leaf/wood colour. Combined PLY RGB identifies the recording, with leaf/wood colour in leaf_RGB fields. Raw channels and source frame/pixel indices are preserved.\nUse scan_manifest.json for current point counts and matching matrices. The complete former crop and metadata are in QC provenance/roi_extension_baseline.zip.\n')
        report=dict(dataset=dataset,**details,source_verification=proof,mesh=mesh_quality,shared_classifier_sha256=model.get('model_sha256'))
        save_json(qc/'report.json',report)
        # A simple view shows added measured ends without implying extra accuracy.
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        q=merged[::5];fig,ax=plt.subplots(figsize=(14,6));ax.scatter(q['x'],q['z'],s=.35,c='#388dbc',alpha=.4,linewidths=0)
        ax.axvline(roi['original_along_m'][0],c='#be6540',ls='--',label='Former crop edges');ax.axvline(roi['original_along_m'][1],c='#be6540',ls='--');ax.set(xlabel='Along row X (m)',ylabel='Z (m)',title=dataset+' · measured X-end returns restored; accepted poses unchanged');ax.set_aspect('equal');ax.legend();ax.grid(alpha=.2);fig.tight_layout();fig.savefig(qc/'expanded_roi.png',dpi=125,bbox_inches='tight');plt.close(fig)
        gallery=stage/'QC'/dataset/'index.html';text=gallery.read_text();start='<!-- measured-roi-start -->';end='<!-- measured-roi-end -->'
        if start in text:text=text[:text.index(start)]+text[text.index(end)+len(end):]
        block=start+'<h2>Current measured ROI extension and matching mesh</h2><p><a href="roi_extension/report.json">Report</a> · <a href="roi_extension/shrinkwrap_quality.json">Current mesh topology/source hash</a></p><img style="max-width:100%" src="roi_extension/expanded_roi.png"><img style="max-width:100%" src="roi_extension/shrinkwrap_preview.png">'+end
        pos=text.find('</h1>');pos=pos+5 if pos>=0 else len(text);gallery.write_text(text[:pos]+block+text[pos:])
        _refresh_caches(stage,dataset)
        record=dict(dataset=dataset,cache_key=key,points=len(merged),added_primary_points=len(merged)-len(primary),output_hashes={folder+'/'+dataset+'/'+name:sha for folder in ['pointclouds','contributions','transformations','meshes'] for name,sha in directory_hashes(stage/folder/dataset).items()})
        save_json(marker,record);results.append(record);print(dataset,'measured ROI added',record['added_primary_points'],'primary samples',flush=True)
    return results
