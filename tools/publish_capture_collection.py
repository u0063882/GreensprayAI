#!/usr/bin/env python3
"""Publish validated dataset folders to the repository's single current release."""
import argparse,hashlib,json,os,subprocess,sys,tempfile
from pathlib import Path
ROOT=Path(__file__).resolve().parent.parent
sys.path.insert(0,str(ROOT))
from tools.capture_collection import (NAMES,FOLDERS,import_current,frozen_hashes,assert_frozen,add_capture,package_collection,digest)


def api(repo,path):return json.loads(subprocess.check_output(['gh','api',f'repos/{repo}/{path}'],text=True))


def release_notes(datasets,paths):
    base=(ROOT/'results/RELEASE_NOTES.md').read_text()
    table='\n\n| Dataset | Final measured points | Mesh |\n| --- | ---: | --- |\n'
    for item in datasets:table+='| '+item['dataset']+' | '+f"{item['points']:,}"+' | '+('Available' if item['mesh_available'] else 'Not generated')+' |\n'
    sums='\n<details><summary>SHA-256 checksums</summary>\n\n```text\n'+''.join(digest(p)+'  '+p.name+'\n' for p in paths)+'```\n</details>\n'
    return base+table+sums


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode',choices=['auto','rebuild','repackage'],default=os.environ.get('PUBLISH_MODE','auto'))
    parser.add_argument('--asset-dir',type=Path,default=ROOT/'work/collection-release')
    args=parser.parse_args();args.asset_dir.mkdir(parents=True,exist_ok=True)
    config=json.loads((ROOT/'results/capture_collection.json').read_text());repo=os.environ.get('GITHUB_REPOSITORY','u0063882/GreensprayAI');tag=config['tag']
    original=api(repo,'releases/tags/'+tag);source_temp=tempfile.TemporaryDirectory(prefix='source-',dir=args.asset_dir);source=Path(source_temp.name)
    subprocess.run(['gh','release','download',tag,'--repo',repo,'--dir',str(source),'--clobber'],check=True)
    for item in original['assets']:
        if Path(item['name']).name!=item['name']:raise ValueError('Unsafe source asset name')
        p=source/item['name']
        if not item.get('digest') or item['digest']!='sha256:'+digest(p) or item['size']!=p.stat().st_size:raise RuntimeError('Current asset verification failed: '+p.name)
    commit=os.environ.get('GITHUB_SHA') or subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
    with tempfile.TemporaryDirectory(prefix='collection-',dir=args.asset_dir) as temporary:
        work=Path(temporary);stage=work/'stage';layout=import_current(source,stage)
        frozen_ids=tuple(d['id'] for d in config['datasets'] if d.get('kind')=='frozen');before=frozen_hashes(stage,frozen_ids)
        # Keep the old geometry bytes frozen while introducing named captures.
        datasets=[dict(dataset='chandelier',points=1219008,mesh_available=True,geometry_policy='Frozen validated outputs; only ZIP paths changed')]
        for dataset in config['datasets']:
            if dataset['id']=='chandelier':continue
            if not dataset['id'].replace('_','').isalnum() or dataset['id']!=json.loads((ROOT/dataset['config']).read_text())['dataset']:raise ValueError('Invalid or inconsistent dataset identifier')
            datasets.append(add_capture(dataset,stage,args.asset_dir/'captures',repo,args.mode))
        assert_frozen(stage,before,frozen_ids)
        provenance=stage/'QC/provenance';provenance.mkdir(parents=True,exist_ok=True)
        (provenance/'collection.json').write_text(json.dumps(dict(packaging_commit=commit,datasets=datasets,frozen_dataset_sha256=before),indent=2)+'\n')
        code=work/'code.zip';subprocess.run(['git','archive','--format=zip','--output',str(code),'HEAD'],cwd=ROOT,check=True)
        paths=package_collection(stage,args.asset_dir/'downloads',code);assert_frozen(stage,before,frozen_ids)
        notes=work/'notes.md';notes.write_text(release_notes(datasets,paths))
        subprocess.run(['gh','release','upload',tag,'--repo',repo,*map(str,paths),'--clobber'],check=True)
        uploaded=api(repo,'releases/tags/'+tag);assets={a['name']:a for a in uploaded['assets']}
        for path in paths:
            item=assets.get(path.name)
            if not item or item['state']!='uploaded' or item['size']!=path.stat().st_size or item.get('digest')!='sha256:'+digest(path):raise RuntimeError('Published collection checksum mismatch: '+path.name)
        # All replacements passed before deleting the two obsolete flat downloads.
        for item in original['assets']:
            if item['name'] not in NAMES:
                if item['name'] not in ['01_final_aligned_pointcloud.ply','03_transformations.json']:raise RuntimeError('Unrecognized attachment needs explicit preservation before cleanup: '+item['name'])
                subprocess.run(['gh','api',f"repos/{repo}/releases/assets/{item['id']}",'--method','DELETE'],check=True)
        subprocess.run(['gh','release','edit',tag,'--repo',repo,'--title',config['title'],'--notes-file',str(notes),'--latest'],check=True)
        subprocess.run(['gh','api',f'repos/{repo}/git/refs/tags/{tag}','--method','PATCH','-f','sha='+commit,'-F','force=true'],check=True)
        final=api(repo,'releases/tags/'+tag)
        if {a['name'] for a in final['assets']}!=set(NAMES):raise RuntimeError('Unexpected final download list')
        print('Published dataset collection:',[(d['dataset'],d['points']) for d in datasets],flush=True)

if __name__=='__main__':main()
