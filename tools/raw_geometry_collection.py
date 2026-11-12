"""Add independent original-capture angle checks to QC; never export geometry."""
from pathlib import Path
import hashlib,json,shutil
from tools.capture_collection import input_archives,extract_inputs,digest
from tools.audit_raw_capture_geometry import run_audit
ROOT=Path(__file__).resolve().parent.parent
ARTIFACTS=('raw_geometry_report.json','raw_angles.csv','raw_imu_geometry.png','raw_imu_geometry.pdf','raw_angle_sensitivity.png','raw_all_scan_profiles.png','raw_row_detection.png','raw_central_plane.png','raw_central_plane.pdf','central_plane_angles.csv')


def add_raw_geometry_checks(stage,work,config,repo,data_directories=None):
    stage=Path(stage);work=Path(work);settings=json.loads((ROOT/'presets/raw_geometry_audit.json').read_text());model=stage/'QC/provenance/leafwood_model.json';model=model if model.exists() else None;summaries={}
    code=['captures/raw_central_plane.py','captures/raw_io.py','captures/raw_geometry.py','captures/raw_geometry_qc.py','tools/audit_raw_capture_geometry.py','tools/raw_geometry_collection.py','presets/raw_geometry_audit.json']
    signature=''.join(digest(ROOT/name) for name in code)+(digest(model) if model else 'no-colour-model')
    for dataset in config['datasets']:
        identifier=dataset['id']
        if identifier not in settings['datasets']:continue
        options=json.loads((ROOT/dataset['config']).read_text()) if dataset.get('config') else {}
        identity=hashlib.sha256((signature+json.dumps(dataset['inputs'],sort_keys=True)+json.dumps(options.get('excluded_recordings',[]),sort_keys=True)).encode()).hexdigest()
        destination=stage/'QC'/identifier/'raw_geometry';marker=destination/'build.json';destination.mkdir(parents=True,exist_ok=True)
        for source in dataset['inputs']:
            archived=stage/'QC'/identifier/'provenance/raw_inputs'/source['name']
            if archived.exists() and digest(archived)!=source['sha256']:raise RuntimeError('Archived raw input changed: '+source['name'])
        if marker.exists() and json.loads(marker.read_text()).get('identity')==identity:
            old=json.loads(marker.read_text())
            if any(digest(destination/name)!=sha for name,sha in old['artifacts'].items()):raise RuntimeError('Raw audit artifact changed')
            print('Reusing verified original-capture audit:',identifier,flush=True)
        else:
            workspace=work/identifier;workspace.mkdir(parents=True,exist_ok=True)
            if data_directories and identifier in data_directories:data=Path(data_directories[identifier])
            else:
                paths=input_archives(dataset,stage/'QC'/identifier,workspace,repo);extract_inputs(paths,workspace);data=workspace/'work/data'
            result=run_audit(data,workspace/'audit',identifier,options,model,settings['repeat_last_complete_frame'],cache_directory=workspace/'frames')
            for name in ARTIFACTS:shutil.copy2(workspace/'audit'/name,destination/name)
            marker.write_text(json.dumps(dict(identity=identity,artifacts={name:digest(destination/name) for name in ARTIFACTS},primary_geometry_changed=False),indent=2)+'\n')
        report=json.loads((destination/'raw_geometry_report.json').read_text());summaries[identifier]=dict(side_mapping=report['side_mapping'],summary=report['summary'],central_tree_plane=report['central_tree_plane'],primary_geometry_changed=False)
        index=stage/'QC'/identifier/'index.html';tag='<!-- independent-raw-geometry -->'
        if index.exists() and tag not in index.read_text():index.write_text(index.read_text()+'\n'+tag+'<h2>Original PCAP angle check</h2><p>IMU gravity rotation only; no registered poses, ground flattening or PCA-up correction. The plots measure visible structure, not verified individual branch angles.</p><p><a href="raw_geometry/raw_geometry_report.json">Raw-source report</a> · <a href="raw_geometry/raw_angles.csv">Per-scan angles</a></p><img style="max-width:100%" src="raw_geometry/raw_imu_geometry.png"><p><a href="raw_geometry/raw_all_scan_profiles.png">Every original scan</a> · <a href="raw_geometry/raw_angle_sensitivity.png">Height-band sensitivity</a> · <a href="raw_geometry/raw_row_detection.png">Low-stem row detection</a></p>\n')
        central_tag='<!-- raw-central-plane -->'
        if index.exists() and central_tag not in index.read_text():index.write_text(index.read_text()+'\n'+central_tag+'<h2>Central tree plane versus ground — primary raw check</h2><p>Both tracks and matched heights are weighted equally, using all selected tree returns. Individual V-arm inclinations are supporting diagnostics. No symmetry or upright-growth target is applied.</p><img style="max-width:100%" src="raw_geometry/raw_central_plane.png"><p><a href="raw_geometry/central_plane_angles.csv">Central-plane angles and sensitivity</a></p>\n')
    (stage/'QC/raw_geometry_summary.json').write_text(json.dumps(summaries,indent=2)+'\n');return summaries
