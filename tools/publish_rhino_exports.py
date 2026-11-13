#!/usr/bin/env python3
"""Add Rhino interchange copies to the existing point-cloud ZIP only.

Never run a registration or mesh stage, rewrite the enriched PLY, or replace
matrix/mesh/contribution/QC assets. The Current geometry tag is retained.
"""
from pathlib import Path,PurePosixPath
import argparse,hashlib,json,os,re,stat,subprocess,sys,tempfile,zipfile
ROOT=Path(__file__).resolve().parent.parent
sys.path.insert(0,str(ROOT))
from tools.export_rhino import digest,export_collection

ASSET='01_final_aligned_pointclouds.zip'


def api(repo,path):return json.loads(subprocess.check_output(['gh','api',f'repos/{repo}/{path}'],text=True))


def checked_extract(archive,destination):
    with zipfile.ZipFile(archive) as z:
        for entry in z.infolist():
            path=PurePosixPath(entry.filename)
            if path.is_absolute() or '..' in path.parts or '\\' in entry.filename:raise ValueError('Unsafe ZIP member')
            if stat.S_ISLNK(entry.external_attr>>16):raise ValueError('ZIP symlinks are not accepted')
        if z.testzip() is not None:raise RuntimeError('Point-cloud ZIP failed CRC')
        z.extractall(destination)


def package_rhino(source_zip,output_zip,work,packaging_commit):
    work=Path(work);work.mkdir(parents=True,exist_ok=True);checked_extract(source_zip,work)
    before={p.relative_to(work).as_posix():digest(p) for p in work.rglob('*') if p.is_file() and 'rhino' not in p.relative_to(work).parts and p.name!='RHINO_EXPORTS.json'}
    reports=export_collection(work)
    if set(reports)!={'chandelier','spindle1','spindle2','vhedge'}:raise RuntimeError('Expected all four orchard systems')
    for relative,sha in before.items():
        if digest(work/relative)!=sha:raise RuntimeError('Existing point-cloud data changed: '+relative)
    manifest=dict(packaging_commit=packaging_commit,source_zip_sha256=digest(source_zip),registration_and_mesh_rerun=False,original_enriched_files_byte_identical=True,datasets=reports)
    (work/'RHINO_EXPORTS.json').write_text(json.dumps(manifest,indent=2)+'\n')
    output_zip=Path(output_zip);output_zip.parent.mkdir(parents=True,exist_ok=True)
    with zipfile.ZipFile(output_zip,'w',zipfile.ZIP_DEFLATED,compresslevel=5) as z:
        for p in sorted(work.rglob('*')):
            if p.is_file():z.write(p,p.relative_to(work).as_posix())
    with zipfile.ZipFile(output_zip) as z:
        if z.testzip() is not None:raise RuntimeError('Rhino point-cloud bundle failed CRC')
    return manifest


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--repo',default=os.environ.get('GITHUB_REPOSITORY','u0063882/GreensprayAI'));parser.add_argument('--tag',default='current');parser.add_argument('--work',type=Path,default=ROOT/'work/rhino-release');args=parser.parse_args();args.work.mkdir(parents=True,exist_ok=True)
    release=api(args.repo,'releases/tags/'+args.tag);assets={a['name']:a for a in release['assets']};source_meta=assets[ASSET];commit=os.environ.get('GITHUB_SHA') or subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
    with tempfile.TemporaryDirectory(prefix='rhino-',dir=args.work) as temporary:
        temp=Path(temporary);source_dir=temp/'source';source_dir.mkdir();subprocess.run(['gh','release','download',args.tag,'--repo',args.repo,'--pattern',ASSET,'--dir',str(source_dir)],check=True);source=source_dir/ASSET
        if source.stat().st_size!=source_meta['size'] or source_meta.get('digest')!='sha256:'+digest(source):raise RuntimeError('Source point-cloud asset checksum mismatch')
        output=temp/'output'/ASSET;manifest=package_rhino(source,output,temp/'pointclouds',commit);expected='sha256:'+digest(output)
        subprocess.run(['gh','release','upload',args.tag,'--repo',args.repo,str(output),'--clobber'],check=True)
        updated=api(args.repo,'releases/tags/'+args.tag);actual={a['name']:a for a in updated['assets']}
        if set(actual)!=set(assets):raise RuntimeError('Release asset list changed')
        if actual[ASSET]['state']!='uploaded' or actual[ASSET]['size']!=output.stat().st_size or actual[ASSET].get('digest')!=expected:raise RuntimeError('Rhino asset upload checksum mismatch')
        for name,old in assets.items():
            if name!=ASSET and any(actual[name].get(k)!=old.get(k) for k in ['id','digest','size']):raise RuntimeError('Unrelated release asset changed: '+name)
        body=updated['body'];start='<!-- rhino-exports-start -->';end='<!-- rhino-exports-end -->';notice=start+'\n## Rhino 8 files\n\nThe point-cloud ZIP now also contains `<dataset>/rhino/final_registered.3dm` (native Rhino 8, **millimetres**, scan layers, original RGB and raw intensity point values), plus `final_registered_xyzrgb_ascii.ply` (simple XYZ/RGB, **metres**). Open the 3DM directly in Rhino. The enriched parent PLY is unchanged and retains all original sensor channels. No registration or mesh was rerun; matrices and the other four assets are byte-identical. See [Rhino import guide](https://github.com/u0063882/GreensprayAI/blob/main/docs/rhino_import.md).\n'+end
        if start in body:body=re.sub(re.escape(start)+'.*?'+re.escape(end),lambda m:notice,body,flags=re.S)
        else:body=notice+'\n\n'+body
        body=body.replace(source_meta['digest'].removeprefix('sha256:'),expected.removeprefix('sha256:'));notes=temp/'notes.md';notes.write_text(body);subprocess.run(['gh','release','edit',args.tag,'--repo',args.repo,'--notes-file',str(notes)],check=True)
        print(json.dumps(dict(new_asset_bytes=output.stat().st_size,new_asset_digest=expected,all_other_assets_unchanged=True,original_enriched_PLYs_byte_identical=True,counts={k:r['source_points'] for k,r in manifest['datasets'].items()}),indent=2),flush=True)

if __name__=='__main__':main()
