#!/usr/bin/env python3
"""Publish the repository's single current result set, including a computed mesh."""
import argparse
import hashlib
import json
import os
import shutil
from pathlib import Path
import subprocess
import sys
import tempfile
import zipfile

ROOT = Path(__file__).resolve().parent.parent
try:
    from .package_release import ASSET_NAMES, package_release
except ImportError:
    from package_release import ASSET_NAMES, package_release

def digest(path):
    with path.open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()

def verify(path, asset):
    if path.stat().st_size != asset['bytes'] or digest(path) != asset['sha256']:
        raise RuntimeError(f'Byte count or checksum mismatch: {path.name}')
    with zipfile.ZipFile(path) as archive:
        bad=archive.testzip()
        if bad:raise RuntimeError(f'Corrupt ZIP member: {bad}')

def release_metadata(repo, tag):
    return json.loads(subprocess.check_output(['gh','api',f'repos/{repo}/releases/tags/{tag}'],text=True))


def choose_mode(requested):
    if requested != 'auto': return requested
    exists=subprocess.run(['git','rev-parse','--verify','refs/tags/current'],cwd=ROOT,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL).returncode==0
    if not exists: return 'rebuild'
    paths=['processing','presets/orchard_stem_candidates.json','run_pipeline.py','run_meshing.py',
           'requirements.txt','requirements-meshing.txt','tools/validate_mesh.py','tools/validate_registered_result.py']
    changed=subprocess.check_output(['git','diff','--name-only','current','HEAD','--',*paths],cwd=ROOT,text=True)
    if changed.strip(): return 'rebuild'
    old=json.loads(subprocess.check_output(['git','show','current:results/download_manifest.json'],cwd=ROOT,text=True))
    new=json.loads((ROOT/'results/download_manifest.json').read_text())
    if any(old.get(k)!=new.get(k) for k in ['assets','mesh_build','registration_build']): return 'rebuild'
    return 'repackage'


def publish_packages(source_dir, code, asset_dir, manifest, data_commit, preserved_names):
    repo=os.environ.get('GITHUB_REPOSITORY','u0063882/GreensprayAI');tag=manifest['tag']
    commit=os.environ.get('GITHUB_SHA') or subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
    paths=package_release(source_dir,asset_dir/'compact',code,data_commit=data_commit,packaging_commit=commit)
    hashes={p.name:digest(p) for p in paths}
    notes=asset_dir/'current_release_notes.md'
    checksums='\n\n<details><summary>SHA-256 checksums</summary>\n\n```text\n'+''.join(f'{hashes[p.name]}  {p.name}\n' for p in paths)+'```\n</details>\n'
    notes.write_text((ROOT/'results/RELEASE_NOTES.md').read_text()+checksums)
    exists=subprocess.run(['gh','release','view',tag,'--repo',repo],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL).returncode==0
    if exists:
        subprocess.run(['gh','release','upload',tag,'--repo',repo,*map(str,paths),'--clobber'],check=True)
    else:
        subprocess.run(['gh','release','create',tag,'--repo',repo,'--target',commit,'--title',manifest['title'],'--notes-file',str(notes),'--latest',*map(str,paths)],check=True)
    # Verify every replacement before removing any old flat attachment.
    uploaded=release_metadata(repo,tag);assets={a['name']:a for a in uploaded['assets']}
    for path in paths:
        asset=assets.get(path.name)
        if not asset or asset['size']!=path.stat().st_size or asset['state']!='uploaded':raise RuntimeError('Upload verification failed: '+path.name)
        if asset.get('digest'):
            if asset['digest']!='sha256:'+hashes[path.name]:raise RuntimeError('Uploaded checksum mismatch: '+path.name)
        else:
            with tempfile.TemporaryDirectory(dir=asset_dir) as folder:
                subprocess.run(['gh','release','download',tag,'--repo',repo,'--pattern',path.name,'--dir',folder],check=True)
                if digest(Path(folder)/path.name)!=hashes[path.name]:raise RuntimeError('Downloaded verification mismatch')
    for asset in uploaded['assets']:
        if asset['name'] not in ASSET_NAMES and asset['name'] in preserved_names:
            subprocess.run(['gh','api',f"repos/{repo}/releases/assets/{asset['id']}",'--method','DELETE'],check=True)
    subprocess.run(['gh','release','edit',tag,'--repo',repo,'--title',manifest['title'],'--notes-file',str(notes),'--latest'],check=True)
    subprocess.run(['gh','api',f'repos/{repo}/git/refs/tags/{tag}','--method','PATCH','-f',f'sha={commit}','-F','force=true'],check=True)
    actual={a['name'] for a in release_metadata(repo,tag)['assets']}
    if not set(ASSET_NAMES).issubset(actual):raise RuntimeError('A final download is missing')
    print('Published compact release:',', '.join(sorted(actual)),flush=True)


def repackage_current(args, manifest):
    repo=os.environ.get('GITHUB_REPOSITORY','u0063882/GreensprayAI');tag=manifest['tag']
    original=release_metadata(repo,tag)
    data_commit=subprocess.check_output(['git','rev-parse','refs/tags/'+tag],cwd=ROOT,text=True).strip()
    with tempfile.TemporaryDirectory(prefix='current-assets-',dir=args.asset_dir) as folder:
        source=Path(folder)
        subprocess.run(['gh','release','download',tag,'--repo',repo,'--dir',str(source)],check=True)
        for asset in original['assets']:
            path=source/asset['name']
            if Path(asset['name']).name!=asset['name'] or not path.is_file() or path.stat().st_size!=asset['size']:raise RuntimeError('Invalid/missing source asset')
            if not asset.get('digest') or asset['digest']!='sha256:'+digest(path):raise RuntimeError('Source asset checksum unavailable or mismatched: '+path.name)
        code=args.asset_dir/'processing_code_and_notes.zip'
        subprocess.run(['git','archive','--format=zip','--output',str(code),'HEAD'],cwd=ROOT,check=True)
        publish_packages(source,code,args.asset_dir,manifest,data_commit,{a['name'] for a in original['assets']})


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--asset-dir',type=Path,default=ROOT/'work/release-assets')
    parser.add_argument('--verify-only',action='store_true')
    parser.add_argument('--mode',choices=['auto','rebuild','repackage'],default=os.environ.get('PUBLISH_MODE','auto'))
    args=parser.parse_args();args.asset_dir.mkdir(parents=True,exist_ok=True)
    manifest=json.loads((ROOT/'results/download_manifest.json').read_text());paths=[]
    mode='rebuild' if args.verify_only else choose_mode(args.mode)
    print('Release mode:',mode,flush=True)
    if mode=='repackage':
        repackage_current(args,manifest)
        return
    for asset in manifest['assets']:
        name=asset['name']
        if Path(name).name!=name or not asset['url'].startswith('https://'):raise RuntimeError('Invalid asset manifest')
        path=args.asset_dir/name
        if not args.verify_only:
            subprocess.run(['curl','--fail','--location','--retry','3','--max-time','180','--output',str(path),asset['url']],check=True)
        verify(path,asset);paths.append(path);print(f'Verified {name}',flush=True)
    if args.verify_only:return
    # Publish the checked-out code/docs, not an obsolete historical snapshot.
    code=args.asset_dir/'processing_code_and_notes.zip'
    subprocess.run(['git','archive','--format=zip','--output',str(code),'HEAD'],cwd=ROOT,check=True)
    paths.append(code)
    out=ROOT/'output';out.mkdir(exist_ok=True)
    baseline=out/'baseline_inputs';baseline.mkdir(exist_ok=True)
    # Historical, checksummed seed clouds remain inputs, not current downloads.
    with zipfile.ZipFile(args.asset_dir/'orchard_row_result.zip') as archive:
        (baseline/'scan_transforms.json').write_bytes(archive.read('scan_transforms.json'))
    seed_scans=baseline/'registered_scans';seed_scans.mkdir(exist_ok=True)
    with zipfile.ZipFile(args.asset_dir/'separate_registered_scans.zip') as archive:
        raw_manifest=archive.read('scan_manifest.json')
        entries=json.loads(raw_manifest)['scans']
        (seed_scans/'scan_manifest.json').write_bytes(raw_manifest)
        for entry in entries:
            name=entry['filename']
            if Path(name).name!=name or not name.endswith('.ply'):raise RuntimeError('Invalid scan filename')
            (seed_scans/name).write_bytes(archive.read(name))
    anchored=out/'anchored_registration'
    subprocess.run([sys.executable,str(ROOT/'processing/register_anchored.py'),
                    '--scan-dir',str(seed_scans),'--transforms',str(baseline/'scan_transforms.json'),
                    '--output-dir',str(anchored),'--export'],cwd=ROOT,check=True)
    quality=json.loads((anchored/'anchored_registration_quality.json').read_text())
    if not quality['validation_passed'] or not quality['candidate_exported']:
        raise RuntimeError('Anchored registration did not pass; refusing to publish inconsistent assets.')
    subprocess.run([sys.executable,str(ROOT/'tools/validate_registered_result.py'),'--baseline-scan-dir',str(seed_scans),'--baseline-transforms',str(baseline/'scan_transforms.json'),'--result-dir',str(anchored)],cwd=ROOT,check=True)
    # Publish newly registered clouds, preserving the familiar asset names.
    paths=[anchored/name for name in ['orchard_row_result.zip','separate_registered_scans.zip','combined_by_recording.zip']]+[code]
    paths.extend([anchored/name for name in ['registration_quality.json','anchored_registration_quality.json','anchored_registration_comparison.png','anchored_trunk_sections.png','orchard_row_preview.png']])
    for name in ['orchard_row_registered.ply','scan_transforms.json']:
        shutil.copy2(anchored/name,out/name)
    subprocess.run([sys.executable,str(ROOT/'processing/mesh.py'),'--transforms',str(out/'scan_transforms.json'),'--voxel',str(manifest['mesh_build']['voxel_m'])],cwd=ROOT,check=True)
    subprocess.run([sys.executable,str(ROOT/'run_meshing.py'),'--input',str(out/'orchard_row_registered.ply'),'--output-dir',str(out/'mesh'),'--ground',str(out/'mesh/ground_surface.ply'),'--include-observed','--archive',str(out/'orchard_mesh.zip'),'--voxel',str(manifest['mesh_build']['wrap_voxel_m']),'--offset',str(manifest['mesh_build']['wrap_outward_offset_m']),'--closing-radius',str(manifest['mesh_build']['wrap_closing_radius_m'])],cwd=ROOT,check=True)
    subprocess.run([sys.executable,str(ROOT/'tools/validate_mesh.py')],cwd=ROOT,check=True)
    paths.extend([out/'orchard_mesh.zip',out/'mesh/mesh_preview.png',out/'mesh/mesh_quality.json',out/'mesh/shrinkwrap_preview.png',out/'mesh/shrinkwrap_quality.json'])
    scan_dir=anchored/'registered_scans'
    subprocess.run([sys.executable,str(ROOT/'processing/check_cross_row.py'),'--scan-dir',str(scan_dir)],cwd=ROOT,check=True)
    paths.extend([out/'cross_row_trunk_sections.png',out/'cross_row_quality.json',out/'cross_row_trunks.csv'])
    subprocess.run([sys.executable,str(ROOT/'processing/plot_trunk_slices.py'),'--scan-dir',str(scan_dir),'--candidate-report',str(ROOT/'presets/orchard_stem_candidates.json')],cwd=ROOT,check=True)
    paths.extend([out/'trunk_slice_diagnostics.zip',*[out/'trunk_slices'/name for name in [
        'trunk_slice_fits.png','trunk_slice_fits.pdf','trunk_slice_centres.png',
        'trunk_slice_quality.json','trunk_slice_centres.csv','candidate_04_slices.png',
        'low_trunk_shift_check.png','low_trunk_shift_quality.json']]])
    # Gather generated outputs in the legacy shape, then make the same compact layout.
    with tempfile.TemporaryDirectory(prefix='generated-assets-',dir=args.asset_dir) as folder:
        source=Path(folder)
        for path in paths:shutil.copy2(path,source/path.name)
        repo=os.environ.get('GITHUB_REPOSITORY','u0063882/GreensprayAI')
        try:
            previous=release_metadata(repo,manifest['tag'])['assets']
        except subprocess.CalledProcessError:
            previous=[]
        known={p.name for p in paths}|{'SHA256SUMS.txt'}
        commit=os.environ.get('GITHUB_SHA') or subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
        publish_packages(source,code,args.asset_dir,manifest,commit,{a['name'] for a in previous if a['name'] in known})

if __name__=='__main__':main()
