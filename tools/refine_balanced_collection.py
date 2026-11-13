"""Final rigid registration and user-requested ground-first output convention.

Earlier geometry is archived in QC. Meshing remains a separate point-cloud
operation. Upstream algorithms always receive their original working frame.
"""
from pathlib import Path
import argparse,hashlib,io,json,os,shutil,sys,tempfile,zipfile
os.environ.setdefault('OMP_NUM_THREADS','2')
os.environ.setdefault('OPENBLAS_NUM_THREADS','2')
ROOT=Path(__file__).resolve().parent.parent
sys.path.insert(0,str(ROOT))
import numpy as np
from captures.balanced_frame import estimate_balanced_frame
from captures.joint_registration import refine_joint,apply
from captures.leafwood import write_records,coordinate_digest
from captures.orchard import save_json,sha256
from processing.export_anchored import read_records,xyz
from tools.compare_leafwood import source_config
from tools.orient_collection import _snapshot,_paths,_refresh_caches
from tools.refine_trunk_collection import _raw_hash
from tools.capture_collection import directory_hashes


def unchanged_attributes(before,after):
    if before.dtype!=after.dtype or len(before)!=len(after):raise RuntimeError('Source return schema/count changed')
    for key in before.dtype.names:
        if key not in ['x','y','z']:np.testing.assert_array_equal(before[key],after[key])


def transform_records(records,T):
    R=np.asarray(T)[:3,:3]
    np.testing.assert_allclose(R.T@R,np.eye(3),atol=1e-9)
    if not np.isclose(np.linalg.det(R),1.,atol=1e-9):raise ValueError('Expected proper rigid transform')
    q=apply(xyz(records),T);out=records.copy()
    for j,k in enumerate(['x','y','z']):out[k]=q[:,j]
    unchanged_attributes(records,out)
    np.testing.assert_allclose(xyz(out),q,rtol=0,atol=3e-6)
    return out


def restore_balanced_frames(stage):
    """Restore exact archived XYZ/poses/mesh before any upstream processing."""
    stage=Path(stage)
    for path in (stage/'transformations').glob('*/transformations.json'):
        meta=json.loads(path.read_text())
        if 'balanced_registration' not in meta:continue
        name=path.parent.name;archive=stage/'QC'/name/'provenance/balanced_registration_baseline.zip'
        if not archive.exists():raise RuntimeError('Missing balanced working-frame baseline: '+name)
        with zipfile.ZipFile(archive) as z:
            b=json.loads(z.read('baseline.json'))
            for item in b['files']:
                p=stage/item['path'];_,d=read_records(p)
                if _raw_hash(d)!=item['raw_sha256']:raise RuntimeError('Source identities changed before balanced restoration')
                q=np.load(io.BytesIO(z.read(item['xyz'])),allow_pickle=False)
                for j,k in enumerate(['x','y','z']):d[k]=q[:,j]
                write_records(p,d,comment='Accepted working frame restored before upstream processing')
            for relative in [f'transformations/{name}/transformations.json',f'contributions/{name}/separate_scans/scan_manifest.json',*b['meshes']]:
                (stage/relative).write_bytes(z.read(relative))
            for relative in ['roi_extension/shrinkwrap_quality.json','roi_extension/report.json','meshing/shrinkwrap_quality.json']:
                p=stage/'QC'/name/relative;member=p.relative_to(stage).as_posix()
                if member in z.namelist():p.write_bytes(z.read(member))
        # Regenerated PLY header changes its file hash, not its measured data.
        primary=stage/'pointclouds'/name/'final_registered.ply'
        for relative in ['roi_extension/shrinkwrap_quality.json','meshing/shrinkwrap_quality.json']:
            p=stage/'QC'/name/relative
            if p.exists():
                q=json.loads(p.read_text());q['source_sha256']=sha256(primary);save_json(p,q)
        _refresh_caches(stage,name)
        print('Restored pre-balanced working coordinates:',name,flush=True)


def snapshot(stage,name):
    archive=stage/'QC'/name/'provenance/balanced_registration_baseline.zip'
    if archive.exists():
        with zipfile.ZipFile(archive) as z:b=json.loads(z.read('baseline.json'))
        matches=all(coordinate_digest(read_records(stage/item['path'])[1])==item['coordinate_sha256'] and _raw_hash(read_records(stage/item['path'])[1])==item['raw_sha256'] for item in b['files'])
        if not matches:archive.rename(archive.with_name('balanced_registration_previous_'+sha256(archive)[:12]+'.zip'))
    if not archive.exists():_snapshot(stage,name,archive)
    return archive


def export_dataset(stage,name,total,report):
    """Compose one SE(3) matrix per scan; preserve all non-XYZ PLY fields."""
    folder=stage/'contributions'/name/'separate_scans';manifest=json.loads((folder/'scan_manifest.json').read_text());meta=json.loads((stage/'transformations'/name/'transformations.json').read_text());lookup={r['scan']:r for r in meta['scans']}
    primary=stage/'pointclouds'/name/'final_registered.ply';_,before=read_records(primary);merged=before.copy();seen=np.zeros(len(merged),dtype=bool);checks=[]
    with tempfile.TemporaryDirectory(prefix='balanced-export-') as temporary:
        temp=Path(temporary)
        for entry,T in zip(manifest['scans'],total):
            _,d=read_records(folder/entry['filename']);out=transform_records(d,T);write_records(temp/entry['filename'],out,comment='Final balanced rigid registration; measured channels, RGB and source IDs retained')
            _,saved=read_records(temp/entry['filename']);unchanged_attributes(d,saved)
            pose=lookup[entry['scan']];old=np.array(pose['sensor_to_merged']);sensor=apply(xyz(d),np.linalg.inv(old))
            for key in ['sensor_to_merged','gravity_levelled_to_merged']:pose[key]=(T@np.array(pose[key])).tolist();entry[key]=pose[key]
            error=float(np.max(abs(apply(sensor,np.array(pose['sensor_to_merged']))-xyz(saved))))
            if error>3e-6:raise RuntimeError('Composed pose does not reconstruct source scan')
            mask=before['scan_id']==entry['scan_id'];seen|=mask;q=apply(xyz(before)[mask],T)
            for j,key in enumerate(['x','y','z']):merged[key][mask]=q[:,j]
            checks.append(dict(scan=entry['scan'],scan_id=entry['scan_id'],points=len(saved),sensor_pose_maximum_absolute_error_m=error))
        if not seen.all():raise RuntimeError('Primary has a source scan without a final transform')
        unchanged_attributes(before,merged);write_records(temp/'final_registered.ply',merged,comment='Final ground-first registered cloud; coordinates already contain the composed per-recording rigid matrices')
        combined_path=stage/'contributions'/name/'combined/all_scans_coloured_by_recording.ply';_,d=read_records(combined_path);out=d.copy()
        for entry,T in zip(manifest['scans'],total):
            mask=d['scan_id']==entry['scan_id'];q=apply(xyz(d)[mask],T)
            for j,key in enumerate(['x','y','z']):out[key][mask]=q[:,j]
        unchanged_attributes(d,out);write_records(temp/'combined.ply',out,comment='Final rigid contributions; RGB by scan_id, original leaf_RGB and measured channels retained')
        info=dict(method='Validated joint scan refinement plus one common ground-first frame',joint_refinement_applied=report['relative_refinement_applied'],frame_transform=report['frame']['output_frame'],working_baseline_sha256=report['baseline_sha256'],matrices_already_applied=True,raw_channels_RGB_and_source_ids_unchanged=True,QC_report=f'90_QC.zip: QC/{name}/balanced_registration/report.json',coordinate_policy=report['frame']['policy'])
        meta['balanced_registration']=info;manifest['balanced_registration']=info
        if 'registration_frame_to_final' in meta:meta['registration_frame_to_final']=(np.array(report['frame']['output_frame'])@np.array(meta['registration_frame_to_final'])).tolist()
        meta['registration']='Current final: '+info['method'];meta['output_coordinate_frame']='X along row; Z ground-first balanced local up; not calibrated Earth vertical'
        manifest['source_crop_limit']='All original retained measured samples preserved; the old crop is expressed in the archived working frame.'
        save_json(temp/'transformations.json',meta);save_json(temp/'scan_manifest.json',manifest)
        for entry in manifest['scans']:shutil.copy2(temp/entry['filename'],folder/entry['filename'])
        shutil.copy2(temp/'final_registered.ply',primary);shutil.copy2(temp/'combined.ply',combined_path);shutil.copy2(temp/'transformations.json',stage/'transformations'/name/'transformations.json');shutil.copy2(temp/'scan_manifest.json',folder/'scan_manifest.json')
    return dict(points=len(merged),source_raw_sha256=_raw_hash(before),output_raw_sha256=_raw_hash(merged),all_non_coordinate_attributes_preserved=True,primary_sha256=sha256(primary),per_scan_reconstruction=checks)


def mesh_from_cloud(primary,meshdir,qc,settings):
    """Independent point-cloud mesher: takes no registration state or poses."""
    import open3d as o3d
    from processing.ground_surface import observed_ground
    from processing.shrinkwrap import main as wrap
    meshdir.mkdir(parents=True,exist_ok=True);qc.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='balanced-mesh-') as temporary:
        temp=Path(temporary);_,d=read_records(primary);soil=observed_ground(xyz(d));o3d.io.write_triangle_mesh(str(temp/'ground_surface.ply'),soil)
        wrap(['--input',str(primary),'--output-dir',str(temp),'--ground',str(temp/'ground_surface.ply'),'--archive',str(temp/'mesh.zip'),'--voxel',str(settings['voxel_m']),'--offset',str(settings['outward_offset_m']),'--closing-radius',str(settings['closing_radius_m']),'--min-volume',str(settings['minimum_volume_m3'])])
        quality=json.loads((temp/'shrinkwrap_quality.json').read_text())
        if not quality['watertight'] or quality['source_sha256']!=sha256(primary):raise RuntimeError('Rebuilt mesh/source validation failed')
        for name in ['canopy_wrap.ply','canopy_wrap.stl','canopy_wrap.obj','canopy_wrap_with_ground.ply','ground_surface.ply','README.txt']:shutil.copy2(temp/name,meshdir/name)
        for name in ['shrinkwrap_quality.json','shrinkwrap_preview.png']:shutil.copy2(temp/name,qc/name)
    return quality


def plot_candidate(stage,name,before,after,report):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    qc=stage/'QC'/name/'balanced_registration';fig,axes=plt.subplots(1,2,figsize=(11,6))
    # Only a common translation for the before-panel's readable row origin.
    pivot=np.array(report['frame']['pivot_input_m']);palette=['#2483ac','#ba732b']
    for ax,p,title in zip(axes,[before-pivot,after],['Previous registered cloud','New final registered cloud']):
        q=p[::max(1,len(p)//120000)];ax.scatter(q[:,1],q[:,2],s=.3,c='#526d63',alpha=.3,linewidths=0);ax.axhline(0,c='#236eaf',ls='--',lw=1);ax.axvline(0,c='#777',ls=':',lw=1);ax.set(title=title,xlabel='Across row Y (m)',ylabel='Z (m)',xlim=(-1.8,1.8),ylim=(-.25,4.6),aspect='equal');ax.grid(alpha=.2)
    f=report['frame'];axes[1].text(.03,.98,f"Ground tilt {f['ground_tilt_after_deg']:.2f}°\nObserved centre tilt {f['observed_centre_tilt_after_deg']:.2f}°",transform=axes[1].transAxes,va='top',fontsize=10,bbox=dict(facecolor='white',alpha=.8,edgecolor='none'))
    fig.suptitle(name+' · rigid-only registration and ground-first local frame');fig.text(.5,.01,'No scaling, tree straightening, terrain deformation or V-symmetry constraint. Observed centre is an orientation preference, not surveyed truth.',ha='center',fontsize=8);fig.tight_layout(rect=(0,.04,1,.94));fig.savefig(qc/'profiles.png',dpi=120);plt.close(fig)


def refine_balanced_collection(stage,precomputed_directory=None):
    stage=Path(stage);restore_balanced_frames(stage);preset=json.loads((ROOT/'presets/balanced_registration.json').read_text());references=json.loads((ROOT/'presets/manual_measurements.json').read_text());summaries=[]
    if not preset.get('enabled'):return summaries
    for name in preset['datasets']:
        print('Balanced registration START:',name,flush=True);qc=stage/'QC'/name/'balanced_registration';qc.mkdir(parents=True,exist_ok=True);archive=snapshot(stage,name)
        folder=stage/'contributions'/name/'separate_scans';manifest=json.loads((folder/'scan_manifest.json').read_text());meta=json.loads((stage/'transformations'/name/'transformations.json').read_text());lookup={r['scan']:r for r in meta['scans']};clouds=[];scores=[];origins=[];ups=[];labels=[]
        for e in manifest['scans']:
            _,d=read_records(folder/e['filename']);clouds.append(xyz(d));scores.append(d['wood_score']);pose=lookup[e['scan']];origins.append(np.array(pose['sensor_to_merged'])[:3,3]);ups.append(np.array(pose['gravity_levelled_to_merged'])[:3,2]);labels.append(e['scan'].split('_')[0])
        seeds,_=source_config(stage,name);primary=stage/'pointclouds'/name/'final_registered.ply';_,records=read_records(primary);before=xyz(records)
        if precomputed_directory:
            r=json.loads((Path(precomputed_directory)/(name+'_joint.json')).read_text())
            if r.get('input_primary_coordinate_sha256')!=coordinate_digest(records) or r.get('input_scan_coordinate_sha256')!=[coordinate_digest(read_records(folder/e['filename'])[1]) for e in manifest['scans']]:raise RuntimeError('Precomputed joint result belongs to different input geometry')
            C=np.load(Path(precomputed_directory)/(name+'_joint.npy'),allow_pickle=False)
            np.testing.assert_allclose(C,np.array(r['total_corrections']),atol=1e-12)
        else:
            C,r=refine_joint(clouds,scores,origins,ups,labels,seeds,references['families']['spindle' if name.startswith('spindle') else name],initial_edges=None,settings=preset.get('joint'))
        applied=bool(r['validation_passed']);relative=C if applied else np.tile(np.eye(4),(len(clouds),1,1));q=before.copy()
        for e,T in zip(manifest['scans'],relative):
            mask=records['scan_id']==e['scan_id'];q[mask]=apply(before[mask],T)
        # Trunk seeds locate the existing row; refined axes remain freely leaning.
        T,frame=estimate_balanced_frame(q,seeds,preset.get('frame'));total=T@relative
        report=dict(dataset=name,baseline_sha256=sha256(archive),relative_refinement_applied=applied,relative_decision='Passed all existing independent joint checks' if applied else 'Retained accepted relative poses; candidate did not pass all existing checks',relative_report=r,frame=frame,total_per_scan_corrections=total.tolist(),scan_ids=[e['scan_id'] for e in manifest['scans']],nonrigid_deformation=False)
        save_json(qc/'relative_candidate.json',r);save_json(qc/'corrections.json',dict(scan_ids=report['scan_ids'],corrections=total.tolist(),baseline_sha256=report['baseline_sha256'],convention='Already applied to final PLYs; these corrections apply once to the archived working frame'))
        report['export']=export_dataset(stage,name,total,report);after=xyz(read_records(primary)[1]);plot_candidate(stage,name,before,after,report)
        if (stage/'meshes'/name).is_dir():
            settings=json.loads((ROOT/'presets/roi_extensions.json').read_text())[name]['mesh'];report['mesh']=mesh_from_cloud(primary,stage/'meshes'/name,stage/'QC'/name/'meshing',settings)
        save_json(qc/'report.json',report)
        (primary.parent/'README.txt').write_text('FINAL REGISTERED CLOUD — METRES\n\nCoordinates already contain the final composed matrices. Original measured samples, SIGNAL/intensity, NIR, reflectivity, leaf/wood RGB and scan_id are retained.\nX runs along the row. Z uses the requested ground-first local convention with a weak observed-envelope up preference; it is not calibrated Earth vertical.\nNo scaling, shear, V symmetry, tree straightening or terrain deformation is applied. Historical geometry and pose validation are under 90_QC.zip.\n')
        for part in ['combined','separate_scans']:(stage/'contributions'/name/part/'README.txt').write_text('CURRENT FINAL REGISTERED CONTRIBUTIONS — METRES\n\nFinal matrices are already applied. Separate PLYs retain leaf/wood RGB; combined RGB identifies the recording, with original leaf_RGB retained. Raw channels and source IDs are unchanged.\nUse transformations/<dataset>/transformations.json for the composed original-sensor matrices.\n')
        gallery=stage/'QC'/name/'index.html';text=gallery.read_text();start='<!-- balanced-registration-start -->';end='<!-- balanced-registration-end -->'
        if start in text:text=text[:text.index(start)]+text[text.index(end)+len(end):]
        block=start+'<h2>Current final: joint registration and ground-first frame</h2><p>No terrain deformation, tree-shape template or canopy-PCA upright correction. Earlier figures below describe earlier working frames.</p><p><a href="balanced_registration/report.json">Full report and composed corrections</a></p><img style="max-width:100%" src="balanced_registration/profiles.png">'+end
        at=text.find('</h1>');at=at+5 if at>=0 else 0;gallery.write_text(text[:at]+block+text[at:])
        summaries.append(dict(dataset=name,points=report['export']['points'],relative_refinement_applied=applied,relative_rejection_reasons=r['rejection_reasons'],ground_tilt_deg=frame['ground_tilt_after_deg'],observed_centre_tilt_deg=frame['observed_centre_tilt_after_deg'],rotation_deg=frame['rotation_deg'],primary_sha256=report['export']['primary_sha256'],frame_transform=T.tolist(),mesh_watertight=report.get('mesh',{}).get('watertight'),baseline_sha256=report['baseline_sha256']))
        print('Balanced registration DONE:',name,'joint',applied,'ground',frame['ground_tilt_after_deg'],'centre',frame['observed_centre_tilt_after_deg'],flush=True)
    save_json(stage/'QC/balanced_registration_summary.json',dict(datasets=summaries,policy=preset['policy']))
    return summaries

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--stage',required=True,type=Path);p.add_argument('--precomputed-directory',type=Path);a=p.parse_args();refine_balanced_collection(a.stage,a.precomputed_directory)
