"""Retain the physical working frame; keep canopy PCA as a diagnostic only."""
from pathlib import Path
import hashlib,io,json,os,shutil,sys,tempfile,zipfile
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from captures.structural_frame import estimate_structural_frame
from captures.leafwood import write_records,coordinate_digest
from captures.orchard import sha256,save_json
from processing.export_anchored import read_records,xyz
from tools.refine_trunk_collection import _raw_hash
from tools.capture_collection import directory_hashes
ROOT=Path(__file__).resolve().parents[1]


def _paths(stage,name):
    folder=stage/'contributions'/name/'separate_scans';manifest=json.loads((folder/'scan_manifest.json').read_text())
    return [stage/'pointclouds'/name/'final_registered.ply',*[folder/e['filename'] for e in manifest['scans']],stage/'contributions'/name/'combined/all_scans_coloured_by_recording.ply']


def _snapshot(stage,name,archive):
    archive.parent.mkdir(parents=True,exist_ok=True);meta=dict(dataset=name,files=[],meshes=[])
    with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED,compresslevel=5) as z:
        for i,p in enumerate(_paths(stage,name)):
            _,d=read_records(p);f=io.BytesIO();np.save(f,xyz(d),allow_pickle=False);member=f'xyz/{i:03d}.npy';z.writestr(member,f.getvalue());meta['files'].append(dict(path=p.relative_to(stage).as_posix(),xyz=member,raw_sha256=_raw_hash(d),coordinate_sha256=coordinate_digest(d),source_sha256=sha256(p)))
        for p in [stage/'transformations'/name/'transformations.json',stage/'contributions'/name/'separate_scans/scan_manifest.json']:
            z.write(p,p.relative_to(stage).as_posix())
        for p in (stage/'meshes'/name).glob('*'):
            if p.is_file():z.write(p,p.relative_to(stage).as_posix());meta['meshes'].append(p.relative_to(stage).as_posix())
        for relative in ['roi_extension/shrinkwrap_quality.json','roi_extension/report.json','meshing/shrinkwrap_quality.json']:
            p=stage/'QC'/name/relative
            if p.exists():z.write(p,p.relative_to(stage).as_posix())
        z.writestr('baseline.json',json.dumps(meta,indent=2)+'\n')
    return meta


def _refresh_caches(stage,name):
    hashes={folder+'/'+name+'/'+p:h for folder in ['pointclouds','contributions','transformations'] for p,h in directory_hashes(stage/folder/name).items()}
    for relative in ['provenance/build.json','trunk_refinement/build.json','joint_registration/build.json']:
        p=stage/'QC'/name/relative
        if p.exists():d=json.loads(p.read_text());d['output_hashes']=hashes;d['structural_frame_note']='Working-frame cache refreshed after coordinate export; see structural_frame/report.json for the current orientation policy.';save_json(p,d)
    p=stage/'QC'/name/'roi_extension/build.json'
    if p.exists():
        d=json.loads(p.read_text());d['output_hashes']={folder+'/'+name+'/'+p:h for folder in ['pointclouds','contributions','transformations','meshes'] for p,h in directory_hashes(stage/folder/name).items()};save_json(p,d)
    p=stage/'QC/provenance/leafwood_upgrade.json'
    if p.exists():
        from tools.leafwood_collection import output_hashes
        d=json.loads(p.read_text());config=json.loads((ROOT/'results/capture_collection.json').read_text());d['output_hashes']=output_hashes(stage,config);save_json(p,d)


def restore_working_frames(stage):
    """Run upstream physics in its original coordinates on every rebuild."""
    stage=Path(stage)
    for matrices in (stage/'transformations').glob('*/transformations.json'):
        current=json.loads(matrices.read_text())
        if 'structural_up_frame' not in current:continue
        name=matrices.parent.name;archive=stage/'QC'/name/'provenance/structural_frame_baseline.zip'
        if not archive.exists():raise RuntimeError('Missing original structural-frame baseline: '+name)
        with zipfile.ZipFile(archive) as z:
            baseline=json.loads(z.read('baseline.json'))
            for item in baseline['files']:
                p=stage/item['path'];_,d=read_records(p);q=np.load(io.BytesIO(z.read(item['xyz'])),allow_pickle=False)
                if _raw_hash(d)!=item['raw_sha256']:raise RuntimeError('Source set changed before frame restoration')
                for j,k in enumerate(['x','y','z']):d[k]=q[:,j]
                write_records(p,d,comment='Accepted gravity-referenced working-frame coordinates; no canopy-PCA output rotation')
            for relative in [f'transformations/{name}/transformations.json',f'contributions/{name}/separate_scans/scan_manifest.json',*baseline['meshes']]:
                p=stage/relative;p.write_bytes(z.read(relative))
            for relative in ['roi_extension/shrinkwrap_quality.json','roi_extension/report.json','meshing/shrinkwrap_quality.json']:
                p=stage/'QC'/name/relative;member=p.relative_to(stage).as_posix()
                if member in z.namelist():p.write_bytes(z.read(member))
        primary=stage/'pointclouds'/name/'final_registered.ply'
        for relative in ['roi_extension/shrinkwrap_quality.json','meshing/shrinkwrap_quality.json']:
            quality_path=stage/'QC'/name/relative
            if quality_path.exists():
                quality=json.loads(quality_path.read_text());quality['working_frame_restoration']=dict(previous_source_sha256=quality['source_sha256'],policy='Original archived XYZ and mesh restored exactly; PLY header regenerated');quality['source_sha256']=sha256(primary);save_json(quality_path,quality)
        roi=stage/'QC'/name/'roi_extension/report.json'
        quality=stage/'QC'/name/'roi_extension/shrinkwrap_quality.json'
        if roi.exists() and quality.exists():
            data=json.loads(roi.read_text());data['mesh']=json.loads(quality.read_text());data['orientation_policy']='Original gravity-referenced working frame retained; no canopy-PCA rotation applied';save_json(roi,data)
        _refresh_caches(stage,name)
        print('Restored physics working frame:',name,flush=True)


def _orient_structure_explicitly(stage):
    stage=Path(stage);results=[];code=sha256(ROOT/'captures/structural_frame.py')+sha256(Path(__file__))
    for primary in sorted((stage/'pointclouds').glob('*/final_registered.ply')):
        name=primary.parent.name;qc=stage/'QC'/name/'structural_frame';qc.mkdir(parents=True,exist_ok=True);archive=stage/'QC'/name/'provenance/structural_frame_baseline.zip';marker=qc/'build.json';classifier=sha256(primary.parent/'leafwood_model.json')
        key=hashlib.sha256((code+classifier+(sha256(archive) if archive.exists() else 'none')).encode()).hexdigest()
        if marker.exists():
            record=json.loads(marker.read_text())
            if record.get('cache_key')==key:
                valid=all(sha256(stage/p)==h for p,h in record['output_hashes'].items())
                if valid:results.append(record);print('Reusing structural frame:',name,flush=True);continue
        if archive.exists():
            current_meta=json.loads((stage/'transformations'/name/'transformations.json').read_text())
            if 'structural_up_frame' not in current_meta:
                with zipfile.ZipFile(archive) as z:old=json.loads(z.read('baseline.json'))
                _,working=read_records(primary)
                if coordinate_digest(working)!=old['files'][0]['coordinate_sha256']:
                    history=archive.parent/('structural_frame_previous_'+sha256(archive)[:12]+'.zip');archive.rename(history);_snapshot(stage,name,archive)
        folder=stage/'contributions'/name/'separate_scans';current_manifest=json.loads((folder/'scan_manifest.json').read_text());clouds=[];scores=[];labels=[]
        if archive.exists():
            with zipfile.ZipFile(archive) as z:
                baseline=json.loads(z.read('baseline.json'));lookup={f['path']:f for f in baseline['files']}
                for e in current_manifest['scans']:
                    p=folder/e['filename'];_,d=read_records(p);item=lookup[p.relative_to(stage).as_posix()]
                    if _raw_hash(d)!=item['raw_sha256']:raise RuntimeError('Source return set changed; structural baseline needs a new provenance snapshot')
                    clouds.append(np.load(io.BytesIO(z.read(item['xyz'])),allow_pickle=False));scores.append(d['wood_score']);labels.append(e['scan'].split('_')[0])
        else:
            for e in current_manifest['scans']:
                _,d=read_records(folder/e['filename']);clouds.append(xyz(d));scores.append(d['wood_score']);labels.append(e['scan'].split('_')[0])
        meta=json.loads((stage/'transformations'/name/'transformations.json').read_text());roi=meta.get('measured_roi_extension',{});pivot=float(np.mean(roi['across_m'])) if roi else 0.
        try:T,report=estimate_structural_frame(clouds,scores,labels,pivot)
        except ValueError as error:T=np.eye(4);report=dict(supported=False,applied_rotation_deg=0.,reason=str(error),reference_policy='Current registration frame retained: insufficient structural evidence')
        report['dataset']=name;report['primary_frame_changed']=bool(not np.array_equal(T,np.eye(4)))
        _,original=read_records(primary);before=xyz(original)
        if report['primary_frame_changed'] or archive.exists():
            if not archive.exists():_snapshot(stage,name,archive)
            with zipfile.ZipFile(archive) as z:
                baseline=json.loads(z.read('baseline.json'))
                # Point fields and their order remain exactly as currently stored;
                # only original XYZ is restored, so reruns never rotate twice.
                for item in baseline['files']:
                    p=stage/item['path'];_,d=read_records(p);q=np.load(io.BytesIO(z.read(item['xyz'])),allow_pickle=False)
                    if _raw_hash(d)!=item['raw_sha256'] or len(d)!=len(q):raise RuntimeError('Structural source identity changed: '+item['path'])
                    if p==primary:before=q
                    out=d.copy();moved=q@T[:3,:3].T+T[:3,3]
                    for j,k in enumerate(['x','y','z']):out[k]=moved[:,j]
                    for k in d.dtype.names:
                        if k not in ['x','y','z']:np.testing.assert_array_equal(out[k],d[k])
                    temp=p.with_suffix('.frame-tmp');write_records(temp,out,comment='Shared wood-PCA structural-up output frame; measured relative geometry and non-coordinate fields preserved');_,saved=read_records(temp);np.testing.assert_allclose(xyz(saved),moved,rtol=0,atol=2e-6);os.replace(temp,p)
                for relative in [f'transformations/{name}/transformations.json',f'contributions/{name}/separate_scans/scan_manifest.json']:
                    data=json.loads(z.read(relative));old=[np.array(e['sensor_to_merged']) for e in data['scans']]
                    for e in data['scans']:
                        for k in ['sensor_to_merged','gravity_levelled_to_merged']:
                            if k in e:e[k]=(T@np.array(e[k])).tolist()
                    for a,e in zip(old,data['scans']):np.testing.assert_allclose(np.linalg.inv(np.array(data['scans'][0]['sensor_to_merged']))@np.array(e['sensor_to_merged']),np.linalg.inv(old[0])@a,rtol=0,atol=1e-10)
                    data['structural_up_frame']=report;data['coordinates']='X along row; Y across; Z is paired-view woody structural up, not surveyed gravity vertical. ROI bounds refer to the archived registration frame.'
                    if 'registration_frame_to_final' in data:data['registration_frame_to_final']=(T@np.array(data['registration_frame_to_final'])).tolist()
                    save_json(stage/relative,data)
                if baseline['meshes']:
                    import open3d as o3d
                    for relative in baseline['meshes']:
                        dest=stage/relative
                        if dest.suffix.lower() not in ['.ply','.stl','.obj']:continue
                        with tempfile.TemporaryDirectory(prefix='frame-mesh-') as tmp:
                            source=Path(tmp)/dest.name;source.write_bytes(z.read(relative));mesh=o3d.io.read_triangle_mesh(str(source));vertices=np.asarray(mesh.vertices).copy();mesh.transform(T);mesh.compute_vertex_normals();mesh.compute_triangle_normals()
                            np.testing.assert_allclose(np.asarray(mesh.vertices),vertices@T[:3,:3].T+T[:3,3],rtol=0,atol=1e-10)
                            if not o3d.io.write_triangle_mesh(str(dest),mesh,write_triangle_uvs=False):raise RuntimeError('Frame mesh export failed')
                            if dest.name=='canopy_wrap.ply' and not (mesh.is_edge_manifold(allow_boundary_edges=False) and mesh.is_vertex_manifold() and mesh.is_orientable()):raise RuntimeError('Rigidly reoriented mesh failed topology check')
                    for relative in ['roi_extension/shrinkwrap_quality.json','meshing/shrinkwrap_quality.json']:
                        path=stage/'QC'/name/relative
                        if path.relative_to(stage).as_posix() in z.namelist():
                            quality=json.loads(z.read(path.relative_to(stage).as_posix()));quality['mesh_rigid_reorientation']=dict(source_sha256_before=quality['source_sha256'],registration_to_output=T.tolist(),policy='Source point cloud and mesh rigidly transformed together; connectivity and relative geometry unchanged');quality['source_sha256']=sha256(primary);quality['ground_source_sha256']=sha256(stage/'meshes'/name/'ground_surface.ply');quality['coordinate_frame']=meta.get('coordinates','')+'; common woody structural-up frame';save_json(path,quality)
                    roi_report=stage/'QC'/name/'roi_extension/report.json'
                    if roi_report.exists():
                        d=json.loads(roi_report.read_text());d['structural_frame_after_roi']=report;d['prefix_proof_frame']='Exact prefix was verified before the common structural-frame rotation; all original coordinates are recovered by its inverse within float32 precision';d['mesh']=json.loads((qc.parent/'roi_extension/shrinkwrap_quality.json').read_text());save_json(roi_report,d)
            _refresh_caches(stage,name)
        _,saved=read_records(primary);after=xyz(saved);report['export']=dict(points=len(saved),primary_sha256=sha256(primary),raw_attribute_sha256=_raw_hash(saved),non_coordinate_fields_preserved=True,relative_sensor_poses_preserved=True);save_json(qc/'report.json',report)
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        selection=np.arange(0,len(saved),5);brown=(saved['wood_score'][selection]>.65);fig,axes=plt.subplots(1,2,figsize=(12,6))
        for ax,q,title in zip(axes,[before,after],['Before common orientation','Structural-up output']):
            q=q[selection];ax.scatter(q[brown,1],q[brown,2],s=.25,c='#a46d35',alpha=.45,linewidths=0);ground=before[selection,2]<.12;ax.scatter(q[ground,1],q[ground,2],s=.15,c='#63798a',alpha=.3,linewidths=0);ax.set(xlabel='Across Y (m)',ylabel='Z (m)',title=title);ax.set_aspect('equal');ax.grid(alpha=.2)
        both=np.concatenate([before[selection][:,[1,2]],after[selection][:,[1,2]]]);lo=both.min(axis=0);hi=both.max(axis=0);pad=np.maximum((hi-lo)*.04,.05)
        for ax in axes:ax.set(xlim=(lo[0]-pad[0],hi[0]+pad[0]),ylim=(lo[1]-pad[1],hi[1]+pad[1]))
        fig.suptitle(name+f" · shared wood-centre PCA rotation {report.get('applied_rotation_deg',0):.2f}°\nStructural direction is a coordinate convention; gravity is retained separately in metadata");fig.tight_layout();fig.savefig(qc/'profiles.png',dpi=125);plt.close(fig)
        gallery=stage/'QC'/name/'index.html';text=gallery.read_text();start='<!-- structural-frame-start -->';end='<!-- structural-frame-end -->'
        if start in text:text=text[:text.index(start)]+text[text.index(end)+len(end):]
        block=start+'<h2>Current shared structural-up frame</h2><p>Z follows paired-view woody height-centre PCA; this is not independently surveyed vertical. One common rotation preserves registration and branch angles. Other registration/meshing figures describe the earlier physics working frame.</p><p><a href="structural_frame/report.json">Frame transform, gravity reference and sensitivity</a></p><img style="max-width:100%" src="structural_frame/profiles.png">'+end
        pos=text.find('</h1>');pos=pos+5 if pos>=0 else len(text);gallery.write_text(text[:pos]+block+text[pos:])
        key=hashlib.sha256((code+classifier+(sha256(archive) if archive.exists() else 'none')).encode()).hexdigest();record=dict(dataset=name,cache_key=key,applied_rotation_deg=report.get('applied_rotation_deg',0.),supported=report.get('supported',False),output_hashes={folder+'/'+name+'/'+p:h for folder in ['pointclouds','contributions','transformations','meshes'] for p,h in directory_hashes(stage/folder/name).items()});save_json(marker,record);results.append(record);print('Structural-up frame:',name,record['applied_rotation_deg'],flush=True)
    save_json(stage/'QC/structural_frame_summary.json',dict(datasets=[{k:v for k,v in r.items() if k!='output_hashes'} for r in results]));return results


def orient_collection(stage,apply_rotation=False):
    if apply_rotation:return _orient_structure_explicitly(stage)
    stage=Path(stage);restore_working_frames(stage);results=[]
    for primary in sorted((stage/'pointclouds').glob('*/final_registered.ply')):
        name=primary.parent.name;folder=stage/'contributions'/name/'separate_scans';manifest=json.loads((folder/'scan_manifest.json').read_text());clouds=[];scores=[];labels=[]
        for e in manifest['scans']:
            _,d=read_records(folder/e['filename']);clouds.append(xyz(d));scores.append(d['wood_score']);labels.append(e['scan'].split('_')[0])
        meta=json.loads((stage/'transformations'/name/'transformations.json').read_text());roi=meta.get('measured_roi_extension',{});pivot=float(np.mean(roi['across_m'])) if roi else 0.
        try:T,estimate=estimate_structural_frame(clouds,scores,labels,pivot)
        except ValueError as error:T=np.eye(4);estimate=dict(reason=str(error),estimated_rotation_deg=None)
        report=dict(dataset=name,estimated_pca=estimate,applied_rotation_deg=0.,primary_frame_changed=False,automatic_structural_rotation_enabled=False,policy='Retain the original gravity-referenced registration frame. Canopy or wood PCA is not a physical vertical reference; large PCA rotations were rejected after the real ground slope was clarified.',export=dict(points=len(read_records(primary)[1]),primary_sha256=sha256(primary)))
        qc=stage/'QC'/name/'structural_frame';qc.mkdir(parents=True,exist_ok=True);save_json(qc/'report.json',report)
        # Show the rejected artificial ground tilt against the retained result.
        _,d=read_records(primary);q=xyz(d);ground=(q[:,2]>-.12)&(q[:,2]<.12);q=q[ground][::4];candidate=q@T[:3,:3].T+T[:3,3]
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig,axes=plt.subplots(1,2,figsize=(12,4),sharex=True,sharey=True)
        for ax,p,title in zip(axes,[candidate,q],['Rejected PCA orientation — not the real slope','Current output: original gravity reference']):
            ax.scatter(p[:,1],p[:,2],s=.25,c='#536f87',alpha=.35,linewidths=0);ax.set(xlabel='Across Y (m)',ylabel='Ground Z (m)',title=title);ax.grid(alpha=.25)
        fig.suptitle(name+' · ground comparison · no terrain flattening');fig.tight_layout();fig.savefig(qc/'profiles.png',dpi=125);plt.close(fig)
        gallery=stage/'QC'/name/'index.html';text=gallery.read_text();start='<!-- structural-frame-start -->';end='<!-- structural-frame-end -->'
        if start in text:text=text[:text.index(start)]+text[text.index(end)+len(end):]
        block=start+'<h2>Orientation correction: PCA rotation withdrawn</h2><p>Current data retain the original gravity-referenced registration frame and measured terrain variation. The large canopy-PCA rotation was not a valid estimate of real ground slope.</p><p><a href="structural_frame/report.json">Policy and diagnostic estimate</a></p><img style="max-width:100%" src="structural_frame/profiles.png">'+end
        at=text.find('</h1>');at=at+5 if at>=0 else len(text);gallery.write_text(text[:at]+block+text[at:])
        record=dict(dataset=name,applied_rotation_deg=0.,supported=False,output_hashes={folder+'/'+name+'/'+p:h for folder in ['pointclouds','contributions','transformations','meshes'] for p,h in directory_hashes(stage/folder/name).items()});save_json(qc/'build.json',record);results.append(record)
        print('Retained physical working frame; PCA diagnostic only:',name,flush=True)
    save_json(stage/'QC/structural_frame_summary.json',dict(policy='No automatic PCA output rotation',datasets=[{k:v for k,v in r.items() if k!='output_hashes'} for r in results]));return results


if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--stage',required=True,type=Path);parser.add_argument('--research-structural-frame',action='store_true',help='Explicit nonphysical PCA output-frame experiment; not used by the release');args=parser.parse_args();orient_collection(args.stage,apply_rotation=args.research_structural_frame)
