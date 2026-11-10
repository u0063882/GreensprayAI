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
import zipfile

ROOT = Path(__file__).resolve().parent.parent

def digest(path):
    with path.open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()

def verify(path, asset):
    if path.stat().st_size != asset['bytes'] or digest(path) != asset['sha256']:
        raise RuntimeError(f'Byte count or checksum mismatch: {path.name}')
    with zipfile.ZipFile(path) as archive:
        bad=archive.testzip()
        if bad:raise RuntimeError(f'Corrupt ZIP member: {bad}')

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--asset-dir',type=Path,default=ROOT/'work/release-assets')
    parser.add_argument('--verify-only',action='store_true')
    args=parser.parse_args();args.asset_dir.mkdir(parents=True,exist_ok=True)
    manifest=json.loads((ROOT/'results/download_manifest.json').read_text());paths=[]
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
    sums=args.asset_dir/'SHA256SUMS.txt'
    sums.write_text(''.join(f'{digest(p)}  {p.name}\n' for p in paths));paths.append(sums)
    repo=os.environ.get('GITHUB_REPOSITORY','u0063882/GreensprayAI');tag=manifest['tag']
    exists=subprocess.run(['gh','release','view',tag,'--repo',repo],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL).returncode==0
    if exists:
        subprocess.run(['gh','release','upload',tag,'--repo',repo,*map(str,paths),'--clobber'],check=True)
        subprocess.run(['gh','release','edit',tag,'--repo',repo,'--title',manifest['title'],'--notes-file',str(ROOT/'results/RELEASE_NOTES.md'),'--latest'],check=True)
        sha=os.environ.get('GITHUB_SHA')
        if sha:subprocess.run(['gh','api',f'repos/{repo}/git/refs/tags/{tag}','--method','PATCH','-f',f'sha={sha}','-F','force=true'],check=True)
    else:
        subprocess.run(['gh','release','create',tag,'--repo',repo,'--target',os.environ.get('GITHUB_SHA','main'),'--title',manifest['title'],'--notes-file',str(ROOT/'results/RELEASE_NOTES.md'),'--latest',*map(str,paths)],check=True)

if __name__=='__main__':main()
