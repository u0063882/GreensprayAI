"""One current release, with each independent dataset in its own ZIP folder."""
from pathlib import Path
import hashlib,json,shutil,subprocess,sys,tempfile,zipfile
ROOT=Path(__file__).resolve().parent.parent
sys.path.insert(0,str(ROOT))
from tools.package_release import unpack_to,checked_member,make_zip,digest
NAMES=('01_final_aligned_pointclouds.zip','02_final_mesh.zip','03_transformations.zip','90_QC.zip','91_registered_contributions.zip')
FOLDERS=('pointclouds','meshes','transformations','QC','contributions')


def copy(source,target):
    target=Path(target);target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,target)


def import_current(source,stage):
    """Migrate the former single-dataset five downloads, or read a collection."""
    source=Path(source);stage=Path(stage)
    for name in FOLDERS:(stage/name).mkdir(parents=True,exist_ok=True)
    if (source/'01_final_aligned_pointcloud.ply').is_file():
        copy(source/'01_final_aligned_pointcloud.ply',stage/'pointclouds/chandelier/final_registered.ply')
        copy(source/'03_transformations.json',stage/'transformations/chandelier/transformations.json')
        for filename,folder in [('02_final_mesh.zip','meshes'),('90_QC.zip','QC'),('91_registered_contributions.zip','contributions')]:
            with tempfile.TemporaryDirectory() as temporary:
                unpack_to(source/filename,temporary);root=Path(temporary)
                if folder=='QC':root=root/'QC'
                elif folder=='meshes':root=root/'final_mesh'
                shutil.copytree(root,stage/folder/'chandelier',dirs_exist_ok=True)
        return 'single-dataset'
    for filename,folder in zip(NAMES,FOLDERS):
        if not (source/filename).is_file():raise RuntimeError('Missing current collection download: '+filename)
        if folder=='QC':
            with tempfile.TemporaryDirectory() as temporary:
                unpack_to(source/filename,temporary);shutil.copytree(Path(temporary)/'QC',stage/'QC',dirs_exist_ok=True)
        else:unpack_to(source/filename,stage/folder)
    return 'collection'


def directory_hashes(directory):
    return {p.relative_to(directory).as_posix():digest(p) for p in sorted(Path(directory).rglob('*')) if p.is_file()}


def frozen_hashes(stage,identifiers=('chandelier',)):
    """Byte protection for all pre-existing frozen geometry, matrices and contributions."""
    root=Path(stage);result={}
    for folder in ['pointclouds','meshes','transformations','contributions']:
        for identifier in identifiers:
            for relative,sha in directory_hashes(root/folder/identifier).items():result[folder+'/'+identifier+'/'+relative]=sha
    return result


def assert_frozen(stage,expected,identifiers=('chandelier',)):
    actual=frozen_hashes(stage,identifiers)
    if actual!=expected:raise RuntimeError('Pre-existing chandelier outputs changed')


def input_archives(dataset,qc,workspace,repo):
    paths={}
    for entry in dataset['inputs']:
        name=entry['name']
        if Path(name).name!=name:raise ValueError('Invalid input filename')
        archived=qc/'provenance/raw_inputs'/name;path=workspace/name
        if archived.is_file():copy(archived,path)
        else:
            if not entry.get('bootstrap_blobs'):raise RuntimeError('No archived input or bootstrap transport for '+name)
            with path.open('wb') as output:
                for sha in entry['bootstrap_blobs']:
                    if len(sha)!=40 or any(c not in '0123456789abcdef' for c in sha):raise ValueError('Invalid source blob SHA')
                    subprocess.run(['gh','api',f'repos/{repo}/git/blobs/{sha}','--header','Accept: application/vnd.github.raw+json'],stdout=output,check=True)
        if path.stat().st_size!=entry['bytes'] or digest(path)!=entry['sha256']:raise RuntimeError('Capture input checksum mismatch: '+name)
        paths[entry['side']]=path
    return paths


def extract_inputs(archives,workspace):
    import py7zr
    for side,path in archives.items():
        if side not in ['sideA','sideB']:raise ValueError('Unknown side label')
        destination=workspace/'work/data'/side;destination.mkdir(parents=True,exist_ok=True)
        with tempfile.TemporaryDirectory() as temporary:
            if path.suffix=='.7z':
                with py7zr.SevenZipFile(path) as archive:
                    for member in archive.list():
                        checked_member(member.filename)
                        if getattr(member,'is_symlink',False):raise ValueError('Archive symlinks are not accepted')
                    archive.extractall(temporary)
            else:unpack_to(path,temporary)
            for p in Path(temporary).rglob('*'):
                if p.is_symlink():raise ValueError('Archive symlink')
                if p.is_file() and p.suffix.lower() in ['.pcap','.json','.h5']:
                    target=destination/p.name
                    if target.exists() and digest(target)!=digest(p):raise RuntimeError('Conflicting capture input: '+p.name)
                    copy(p,target)


def pipeline_identity(config):
    files=['run_capture.py','captures/orchard.py','captures/bounded.py','captures/pipeline.py','captures/decode.py','captures/export.py','captures/leafwood.py','captures/leafwood_lab.py','processing/decode.py','processing/register_anchored.py','requirements-captures.txt','requirements.txt','requirements-meshing.txt','processing/export_anchored.py']
    return dict(config_sha256=digest(config),code_sha256=hashlib.sha256(''.join(name+digest(ROOT/name) for name in files).encode()).hexdigest())


def add_capture(dataset,stage,work,repo,mode='auto'):
    config=ROOT/dataset['config'];identity=pipeline_identity(config);identifier=dataset['id'];qc=stage/'QC'/identifier
    marker=qc/'provenance/build.json'
    if marker.exists() and (mode!='rebuild' or dataset.get('kind')=='frozen'):
        old=json.loads(marker.read_text())
        if dataset.get('kind')=='frozen' or (old.get('pipeline')==identity and old.get('input_sha256')==[a['sha256'] for a in dataset['inputs']]):
            for relative,sha in old['output_hashes'].items():
                path=stage/relative
                if not path.is_file() or digest(path)!=sha:raise RuntimeError('Archived capture output mismatch: '+relative)
            # Retain the exact original code snapshot when freezing an older build.
            snapshot=qc/'provenance/data_pipeline_code.zip';previous=stage/'QC/provenance/current_code_and_notes.zip'
            if not snapshot.exists() and previous.exists():copy(previous,snapshot)
            if 'data_source_commit' not in old:
                previous_meta=stage/'QC/provenance/collection.json'
                if previous_meta.exists():old['data_source_commit']=json.loads(previous_meta.read_text())['packaging_commit']
                marker.write_text(json.dumps(old,indent=2)+'\n')
            print('Reusing verified capture:',identifier,flush=True);return old
    if dataset.get('kind')=='frozen':raise RuntimeError('Frozen dataset is missing its verified output metadata: '+identifier)
    if mode=='repackage':raise RuntimeError('Capture needs processing; repackage mode cannot create it: '+identifier)
    workspace=work/identifier;workspace.mkdir(parents=True,exist_ok=True);archives=input_archives(dataset,qc,workspace,repo);extract_inputs(archives,workspace)
    subprocess.run([sys.executable,str(ROOT/'run_capture.py'),'--config',str(config),'--workspace',str(workspace)],cwd=ROOT,check=True)
    final=workspace/'final';report=json.loads((final/'QC/registration_quality.json').read_text())
    if not report['bounded']['validation_passed']:raise RuntimeError('Capture validation failed')
    copy(final/'final_registered.ply',stage/'pointclouds'/identifier/'final_registered.ply');copy(final/'README.txt',stage/'pointclouds'/identifier/'README.txt')
    copy(final/'transformations.json',stage/'transformations'/identifier/'transformations.json')
    shutil.copytree(final/'registered_contributions',stage/'contributions'/identifier/'separate_scans',dirs_exist_ok=True)
    copy(final/'combined_by_recording.ply',stage/'contributions'/identifier/'combined/all_scans_coloured_by_recording.ply')
    shutil.copytree(final/'QC',qc,dirs_exist_ok=True);copy(final/'final_levelled_to_merged_poses.npy',qc/'provenance/final_levelled_to_merged_poses.npy')
    for path in archives.values():copy(path,qc/'provenance/raw_inputs'/path.name)
    hashes={}
    for folder in ['pointclouds','transformations','contributions']:
        for relative,sha in directory_hashes(stage/folder/identifier).items():hashes[folder+'/'+identifier+'/'+relative]=sha
    snapshot=qc/'provenance/data_pipeline_code.zip';subprocess.run(['git','archive','--format=zip','--output',str(snapshot),'HEAD'],cwd=ROOT,check=True)
    source_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
    build=dict(dataset=identifier,data_source_commit=source_commit,pipeline=identity,input_sha256=[a['sha256'] for a in dataset['inputs']],output_hashes=hashes,points=report['export']['merged_points'],mesh_available=False)
    marker.parent.mkdir(parents=True,exist_ok=True);marker.write_text(json.dumps(build,indent=2)+'\n');return build


def package_collection(stage,output,code_archive):
    stage=Path(stage);output=Path(output);output.mkdir(parents=True,exist_ok=True)
    copy(code_archive,stage/'QC/provenance/current_code_and_notes.zip')
    datasets=sorted(p.name for p in (stage/'pointclouds').iterdir() if p.is_dir())
    (stage/'QC/index.html').write_text('<!doctype html><meta charset="utf-8"><title>Orchard QC datasets</title><h1>Quality checks by dataset</h1><ul>'+''.join('<li><a href="'+name+'/index.html">'+name+'</a></li>' for name in datasets)+'</ul><p>Each capture uses its own local metric frame. Final cloud/matrix ZIPs contain matching dataset folders.</p>'+('<p><a href="leafwood_comparison_report.md">Leaf/wood registration comparison report</a> — primary geometry remains the baseline.</p>' if (stage/'QC/leafwood_comparison_report.md').exists() else ''))
    for folder in ['pointclouds','transformations','meshes','contributions']:
        (stage/folder/'README.txt').write_text('FINAL ORCHARD DATASETS\n\nEach dataset has its own folder and independent local frame, in metres.\nFinal PLY coordinates already have their matching transformation matrices applied.\nMeshes are included only for datasets that have been meshed; availability is listed on the release page.\nQC/index.html in 90_QC.zip opens the figures by dataset.\n')
    for index in [0,1,2,4]:make_zip(output/NAMES[index],stage/FOLDERS[index])
    (stage/'QC/SHA256SUMS.txt').write_text(''.join(digest(output/NAMES[index])+'  '+NAMES[index]+'\n' for index in [0,1,2,4]))
    make_zip(output/NAMES[3],stage/'QC','QC')
    for filename in NAMES:
        with zipfile.ZipFile(output/filename) as archive:
            if archive.testzip() is not None:raise RuntimeError('Corrupt collection ZIP: '+filename)
    return [output/name for name in NAMES]
