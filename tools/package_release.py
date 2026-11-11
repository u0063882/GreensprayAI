#!/usr/bin/env python3
"""Create a compact release without changing measured geometry or matrices."""
from pathlib import Path, PurePosixPath
import hashlib
import html
import json
import shutil
import stat
import tempfile
import zipfile
from urllib.parse import quote

ASSET_NAMES=(
    '01_final_aligned_pointcloud.ply',
    '02_final_mesh.zip',
    '03_transformations.json',
    '90_QC.zip',
    '91_registered_contributions.zip',
)
FINAL_MESH_MEMBERS=('canopy_wrap.ply','canopy_wrap.obj','canopy_wrap.stl','ground_surface.ply','canopy_wrap_with_ground.ply')
LEGACY_PACKAGES={'orchard_row_result.zip','orchard_mesh.zip','separate_registered_scans.zip','combined_by_recording.zip','trunk_slice_diagnostics.zip','processing_code_and_notes.zip'}


def digest(path):
    with Path(path).open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()


def checked_member(name):
    p=PurePosixPath(name)
    if p.is_absolute() or '..' in p.parts or '\\' in name:raise ValueError('Unsafe ZIP member: '+name)
    return p


def extract_checked(archive,member,destination):
    checked_member(member.filename)
    if stat.S_ISLNK(member.external_attr >> 16):raise ValueError('ZIP symlinks are not accepted')
    target=Path(destination);target.parent.mkdir(parents=True,exist_ok=True)
    # Identical diagnostics often occur both individually and in an old ZIP.
    if target.exists():
        with archive.open(member) as stream:
            if hashlib.file_digest(stream,'sha256').hexdigest()==digest(target):return target
        suffix=hashlib.sha256(archive.read(member)).hexdigest()[:10]
        target=target.with_name(target.stem+'__'+suffix+target.suffix)
    with archive.open(member) as source,target.open('wb') as sink:shutil.copyfileobj(source,sink)
    return target


def unpack_to(path,directory):
    with zipfile.ZipFile(path) as archive:
        for member in archive.infolist():
            checked_member(member.filename)
            if not member.is_dir():extract_checked(archive,member,Path(directory)/member.filename)


def qc_location(name):
    if name.startswith(('anchored_','registration_','orchard_row_preview')):return Path('registration')/name
    if name.startswith('cross_row_'):return Path('registration/cross_row')/name
    if name.startswith(('trunk_','candidate_','low_trunk_')):return Path('trunk_slices')/name
    if name.startswith(('mesh_','shrinkwrap_')):return Path('meshing')/name
    if name=='SHA256SUMS.txt':return Path('provenance/legacy_release_SHA256SUMS.txt')
    return Path('additional_assets')/name


def add_file(source,target):
    target=Path(target);target.parent.mkdir(parents=True,exist_ok=True)
    if target.exists() and digest(source)!=digest(target):target=target.with_name(target.stem+'__'+digest(source)[:10]+target.suffix)
    shutil.copy2(source,target)
    return target


def make_zip(path,folder,prefix=''):
    with zipfile.ZipFile(path,'w',zipfile.ZIP_DEFLATED,compresslevel=5) as archive:
        for p in sorted(Path(folder).rglob('*')):
            if p.is_file():archive.write(p,str(PurePosixPath(prefix)/p.relative_to(folder).as_posix()))


def gallery(qc):
    sections=[]
    for folder,title in [('registration','Registration'),('trunk_slices','Trunk slices'),('meshing','Meshing')]:
        figures=[]
        for p in sorted((qc/folder).rglob('*.png')):
            relative=p.relative_to(qc).as_posix();url=quote(relative,safe='/')
            figures.append(f'<figure><a href="{url}"><img loading="lazy" src="{url}" alt="{html.escape(p.name)}"></a><figcaption>{html.escape(relative)}</figcaption></figure>')
        sections.append(f'<details open><summary>{title}</summary>'+''.join(figures)+'</details>')
    links=[]
    for p in sorted(qc.rglob('*')):
        if p.is_file() and p.suffix.lower() in ['.json','.csv','.pdf','.txt','.md']:
            relative=p.relative_to(qc).as_posix();links.append(f'<li><a href="{quote(relative,safe="/")}">{html.escape(relative)}</a></li>')
    (qc/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>GreensprayAI QC</title><style>body{font:16px system-ui;max-width:1200px;margin:2rem auto;padding:0 1rem}img{max-width:100%;height:auto}summary{font-size:1.4rem;cursor:pointer}figure{margin:1.5rem 0}figcaption{color:#555}</style><h1>Quality checks</h1><p>Final data are the separate 01, 02 and 03 downloads. This gallery contains diagnostics; dashed circle fits remain uncertain.</p>'+''.join(sections)+'<h2>Reports and tables</h2><ul>'+''.join(links)+'</ul>')


def package_release(source_dir,output_dir,code_archive,*,data_commit,packaging_commit):
    """Accept either the former flat release or this compact layout; return five files."""
    source=Path(source_dir);out=Path(output_dir);out.mkdir(parents=True,exist_ok=True)
    if source.resolve()==out.resolve():raise ValueError('Use distinct source and output folders')
    with tempfile.TemporaryDirectory(prefix='greenspray-qc-') as temporary:
        work=Path(temporary);qc=work/'QC';qc.mkdir()
        compact=all((source/name).is_file() for name in ASSET_NAMES)
        if compact:
            for name in [*ASSET_NAMES[:3],ASSET_NAMES[4]]:shutil.copy2(source/name,out/name)
            unpack_to(source/ASSET_NAMES[3],work)
            origin_path=qc/'provenance/data_origin.json'
            origin=json.loads(origin_path.read_text()) if origin_path.exists() else {}
            data_commit=origin.get('data_source_commit',data_commit)
            for p in source.iterdir():
                if p.is_file() and p.name not in ASSET_NAMES:add_file(p,qc/qc_location(p.name))
        else:
            with zipfile.ZipFile(source/'orchard_row_result.zip') as archive:
                for member in archive.infolist():
                    checked_member(member.filename)
                    if member.is_dir():continue
                    if member.filename=='orchard_row_registered.ply':destination=out/ASSET_NAMES[0]
                    elif member.filename=='scan_transforms.json':destination=out/ASSET_NAMES[2]
                    else:destination=qc/'registration'/member.filename
                    extract_checked(archive,member,destination)
            final_mesh=work/'final_mesh';final_mesh.mkdir()
            with zipfile.ZipFile(source/'orchard_mesh.zip') as archive:
                names=archive.namelist()
                if not all(n in names for n in FINAL_MESH_MEMBERS):raise ValueError('Final mesh package is incomplete')
                for member in archive.infolist():
                    checked_member(member.filename)
                    if member.is_dir():continue
                    destination=(final_mesh if member.filename in FINAL_MESH_MEMBERS else qc/'meshing')/member.filename
                    extract_checked(archive,member,destination)
            (final_mesh/'README.txt').write_text('FINAL MESH — METRES\n\ncanopy_wrap.ply / .obj / .stl: closed canopy envelope, without ground.\nground_surface.ply: separate open ground sheet.\ncanopy_wrap_with_ground.ply: canopy plus ground, an open combined scene.\nPLY preserves colours. STL has no unit metadata; coordinates are metres.\nAll geometry uses the final aligned point-cloud frame.\nThis is the existing voxel/SDF envelope proxy, not native Rhino ShrinkWrap.\nSee 90_QC.zip for validation, previews and observed-surface alternatives.\n')
            make_zip(out/ASSET_NAMES[1],final_mesh,'final_mesh')
            with zipfile.ZipFile(out/ASSET_NAMES[4],'w',zipfile.ZIP_DEFLATED,compresslevel=5) as target:
                for package,prefix in [('separate_registered_scans.zip','separate_scans'),('combined_by_recording.zip','combined')]:
                    with zipfile.ZipFile(source/package) as archive:
                        for member in archive.infolist():
                            checked_member(member.filename)
                            if member.is_dir():continue
                            if stat.S_ISLNK(member.external_attr >> 16):raise ValueError('ZIP symlink')
                            target.writestr(str(PurePosixPath(prefix)/member.filename),archive.read(member))
                target.writestr('README.txt','REGISTERED CONTRIBUTIONS\n\nseparate_scans/: one PLY per recording, with uniform recording colours.\ncombined/: the same contributions in one PLY, overlaps preserved.\nBoth folders include their original manifests and notes.\nThese PLYs are already aligned; do not apply 03_transformations.json again.\n')
            if (source/'trunk_slice_diagnostics.zip').exists():unpack_to(source/'trunk_slice_diagnostics.zip',qc/'trunk_slices')
            if (source/'processing_code_and_notes.zip').exists():add_file(source/'processing_code_and_notes.zip',qc/'provenance/data_pipeline_code.zip')
            for p in source.iterdir():
                if p.is_file() and p.name not in LEGACY_PACKAGES:add_file(p,qc/qc_location(p.name))
        seeds=source/'registration_seed'
        if seeds.is_dir():
            for seed in sorted(seeds.iterdir()):
                if seed.is_file():add_file(seed,qc/'provenance/registration_seed'/seed.name)
        archived_seeds=qc/'provenance/registration_seed'
        if archived_seeds.is_dir():
            (archived_seeds/'README.txt').write_text('INTERNAL REBUILD INPUTS\n\nThese intermediate inputs are retained only to reproduce the processing pipeline.\nUse the separate 01, 02 and 03 downloads for final point clouds, mesh and transformations.\n')
        # A geometry-source snapshot and the current packaging code may differ.
        add_file(code_archive,qc/'provenance/current_code_and_notes.zip')
        # Do not accumulate an old current-code copy on each future repackaging.
        current=qc/'provenance/current_code_and_notes.zip'
        if digest(current)!=digest(code_archive):shutil.copy2(code_archive,current)
        for p in current.parent.glob('current_code_and_notes__*.zip'):p.unlink()
        provenance=qc/'provenance/data_origin.json';provenance.parent.mkdir(parents=True,exist_ok=True)
        provenance.write_text(json.dumps(dict(data_source_commit=data_commit,packaging_commit=packaging_commit,
            geometry_policy='Packaging only: existing point-cloud, mesh-member and transformation bytes are preserved.',
            input_assets=[dict(name=p.name,bytes=p.stat().st_size,sha256=digest(p)) for p in sorted(source.iterdir()) if p.is_file()]),indent=2)+'\n')
        (qc/'README.md').write_text('# Quality checks\n\nOpen `index.html` after extracting this ZIP to browse figures offline.\n\n- `registration/`: pose/stem comparisons, row preview and quality reports.\n- `trunk_slices/`: low-height circles, centre trajectories, PDF, CSV and JSON.\n- `meshing/`: mesh previews/reports and observed-surface alternatives.\n- `provenance/`: source commits, input hashes and code snapshots.\n- `additional_assets/`: any other former release attachments.\n\nMain downloads: `01_final_aligned_pointcloud.ply`, `02_final_mesh.zip`, `03_transformations.json`. Registered contributions are in `91_registered_contributions.zip`.\n\nThe point clouds are already registered. The transformation JSON is metadata for raw/levelled sensor points; do not transform the final PLY again. All coordinates are metres.\n')
        (qc/'SHA256SUMS.txt').write_text(''.join(f'{digest(out/name)}  {name}\n' for name in [*ASSET_NAMES[:3],ASSET_NAMES[4]]))
        gallery(qc)
        inventory=[dict(path=p.relative_to(qc).as_posix(),bytes=p.stat().st_size,sha256=digest(p)) for p in sorted(qc.rglob('*')) if p.is_file() and p.name!='inventory.json']
        (qc/'inventory.json').write_text(json.dumps(inventory,indent=2)+'\n')
        make_zip(out/ASSET_NAMES[3],qc,'QC')
    paths=[out/name for name in ASSET_NAMES]
    for p in paths:
        if p.suffix=='.zip':
            with zipfile.ZipFile(p) as archive:
                if archive.testzip() is not None:raise ValueError('Corrupt packaged ZIP: '+p.name)
    # Validate the critical byte-preservation properties before any release cleanup.
    verify_preservation(source,out)
    return paths


def verify_preservation(source,out):
    source=Path(source);out=Path(out)
    if all((source/name).exists() for name in ASSET_NAMES):
        for name in [*ASSET_NAMES[:3],ASSET_NAMES[4]]:
            if digest(source/name)!=digest(out/name):raise ValueError('Repackaging changed primary data: '+name)
        return
    with zipfile.ZipFile(source/'orchard_row_result.zip') as archive:
        for old,new in [('orchard_row_registered.ply',ASSET_NAMES[0]),('scan_transforms.json',ASSET_NAMES[2])]:
            with archive.open(old) as stream:
                if hashlib.file_digest(stream,'sha256').hexdigest()!=digest(out/new):raise ValueError('Primary data changed')
    with zipfile.ZipFile(source/'orchard_mesh.zip') as old,zipfile.ZipFile(out/ASSET_NAMES[1]) as new:
        for name in FINAL_MESH_MEMBERS:
            if old.read(name)!=new.read('final_mesh/'+name):raise ValueError('Mesh geometry changed: '+name)
    with zipfile.ZipFile(out/ASSET_NAMES[4]) as new:
        for package,prefix in [('separate_registered_scans.zip','separate_scans'),('combined_by_recording.zip','combined')]:
            with zipfile.ZipFile(source/package) as old:
                for member in old.infolist():
                    if not member.is_dir() and old.read(member)!=new.read(prefix+'/'+member.filename):raise ValueError('Contribution changed')
