"""Optional attribute-only upgrade and QC experiment for a frozen collection."""
from pathlib import Path
import hashlib,json,subprocess
from tools.capture_collection import input_archives,extract_inputs,directory_hashes
from tools.compare_leafwood import upgrade_collection
ROOT=Path(__file__).resolve().parent.parent


def fingerprint(config):
    source=['captures/leafwood.py','captures/register_leafwood.py','tools/compare_leafwood.py','tools/leafwood_collection.py','presets/orchard_stem_candidates.json']
    source += [d['config'] for d in config['datasets'] if d.get('config')]
    h=hashlib.sha256(json.dumps(config,sort_keys=True).encode())
    for name in source:h.update(name.encode());h.update((ROOT/name).read_bytes())
    return h.hexdigest()


def output_hashes(stage,config):
    return {folder:{dataset['id']+'/'+name:sha for dataset in config['datasets']
                   for name,sha in directory_hashes(Path(stage)/folder/dataset['id']).items()}
            for folder in ['pointclouds','contributions']}


def enrich_collection(stage,work,config,repo):
    stage=Path(stage);work=Path(work);marker=stage/'QC/provenance/leafwood_upgrade.json';identity=fingerprint(config)
    if marker.exists():
        old=json.loads(marker.read_text())
        if old.get('pipeline_fingerprint')==identity:
            if output_hashes(stage,config)!=old['output_hashes']:raise RuntimeError('Enriched frozen output checksum mismatch')
            print('Reusing verified shared leaf/wood attributes and comparison',flush=True);return old
    data={}
    for dataset in config['datasets']:
        identifier=dataset['id'];workspace=work/'inputs'/identifier;workspace.mkdir(parents=True,exist_ok=True)
        archives=input_archives(dataset,stage/'QC'/identifier,workspace,repo)
        extract_inputs(archives,workspace);data[identifier]=workspace/'work/data'
        from tools.capture_collection import copy
        for path in archives.values():copy(path,stage/'QC'/identifier/'provenance/raw_inputs'/path.name)
    upgrade_collection(stage,data,work/'experiment')
    result=json.loads(marker.read_text());result['pipeline_fingerprint']=identity
    result['attribute_code_commit']=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
    result['output_hashes']=output_hashes(stage,config)
    marker.write_text(json.dumps(result,indent=2)+'\n');return result
