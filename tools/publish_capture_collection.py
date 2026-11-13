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
    table='\n\n| Dataset | Final measured points | Mesh | Final registration | Ground tilt in final frame |\n| --- | ---: | --- | --- | ---: |\n'
    for item in datasets:
        method=('Balanced final rigid registration' if item.get('balanced_registration') else
                'Joint rigid refinement' if item.get('joint_registration',{}).get('promoted') else
                'Trunk-refined rigid poses' if item.get('trunk_refinement',{}).get('promoted') else 'Accepted rigid poses')
        table+='| '+item['dataset']+' | '+f"{item['points']:,}"+' | '+('Available' if item['mesh_available'] else 'Not generated')+' | '+method+' | '+f"{item.get('balanced_registration',{}).get('ground_tilt_deg',0):.2f}°"+' |\n'
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
        from tools.orient_collection import restore_working_frames,orient_collection
        from tools.refine_balanced_collection import restore_balanced_frames
        restore_balanced_frames(stage)
        restore_working_frames(stage)
        frozen_ids=tuple(d['id'] for d in config['datasets'] if d.get('kind')=='frozen');before=frozen_hashes(stage,frozen_ids)
        # Keep the old geometry bytes frozen while introducing named captures.
        datasets=[dict(dataset='chandelier',points=1219008,mesh_available=True,geometry_policy='Frozen validated outputs; only ZIP paths changed')]
        for dataset in config['datasets']:
            if dataset['id']=='chandelier':continue
            if not dataset['id'].replace('_','').isalnum() or dataset['id']!=json.loads((ROOT/dataset['config']).read_text())['dataset']:raise ValueError('Invalid or inconsistent dataset identifier')
            datasets.append(add_capture(dataset,stage,args.asset_dir/'captures',repo,args.mode))
        assert_frozen(stage,before,frozen_ids)
        if config.get('leafwood_comparison',False):
            from tools.leafwood_collection import enrich_collection
            # This changes attributes only in the primary outputs. The new
            # registration is isolated under QC and cannot replace their poses.
            protected={k:v for k,v in before.items() if k.startswith(('meshes/','transformations/'))}
            enrich_collection(stage,args.asset_dir/'leafwood',config,repo)
            after=frozen_hashes(stage,frozen_ids)
            if any(after.get(k)!=v for k,v in protected.items()):raise RuntimeError('Leaf/wood upgrade changed a baseline mesh or transformation')
            before=after
        if (ROOT/'presets/manual_measurements.json').exists():
            from tools.refine_trunk_collection import refine_collection
            refinement=refine_collection(stage,config)
            for item in datasets:
                result=next((r for r in refinement if r['dataset']==item['dataset']),None)
                if result:item['trunk_refinement']=dict(promoted=result['promoted'],validation_passed=result['validation_passed'])
            # These are now verified, explicitly authorised final corrections.
            # Their original geometry is retained in per-dataset baseline ZIPs.
            before=frozen_hashes(stage,frozen_ids)
        if (ROOT/'presets/roi_extensions.json').exists():
            from tools.extend_collection_roi import extend_collection_roi
            extended=extend_collection_roi(stage,args.asset_dir/'roi_extensions',config,repo)
            for item in datasets:
                result=next((r for r in extended if r['dataset']==item['dataset']),None)
                if result:
                    item['points']=result['points'];item['measured_roi_extension']=True
                    item['geometry_policy']='Accepted poses and original samples retained; measured row ROI extended and matching mesh rebuilt independently'
            before=frozen_hashes(stage,frozen_ids)
        if (ROOT/'presets/joint_registration.json').exists():
            from tools.refine_joint_collection import refine_joint_collection
            joint=refine_joint_collection(stage)
            for item in datasets:
                result=next((r for r in joint if r['dataset']==item['dataset']),None)
                if result:item['joint_registration']=dict(promoted=result['promoted'],validation_passed=result['validation_passed'])
            before=frozen_hashes(stage,frozen_ids)
        if (ROOT/'presets/spindle_trunk_reference.json').exists():
            from tools.trunk_reference_qc import add_reference_checks
            add_reference_checks(stage)
        # Diagnostic coverage only; no correction or shape prior is applied.
        from tools.structural_collection import add_structural_checks
        add_structural_checks(stage)
        frames=orient_collection(stage)
        for item in datasets:
            result=next((r for r in frames if r['dataset']==item['dataset']),None)
            if result:item['structural_frame_deg']=result['applied_rotation_deg']
        before=frozen_hashes(stage,frozen_ids)
        if (ROOT/'presets/raw_geometry_audit.json').exists():
            from tools.raw_geometry_collection import add_raw_geometry_checks
            add_raw_geometry_checks(stage,args.asset_dir/'raw_geometry',config,repo)
            assert_frozen(stage,before,frozen_ids)
        if (ROOT/'presets/balanced_registration.json').exists():
            from tools.refine_balanced_collection import refine_balanced_collection
            balanced=refine_balanced_collection(stage)
            for item in datasets:
                result=next((r for r in balanced if r['dataset']==item['dataset']),None)
                if result:item['balanced_registration']=result
            before=frozen_hashes(stage,frozen_ids)
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
