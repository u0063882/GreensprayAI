"""Run a fixed 15-second pilot and publish separately from Current results."""
from pathlib import Path
import hashlib,json,os,shutil,subprocess,zipfile
from dynamic.prepare import prepare_local
from dynamic.pilot import run

REPO='u0063882/GreensprayAI'
TAG='moving-scan-inputs'
EXPECTED_SHA='41a702f3a3818e555cbfdd2e8cad00d66eea94e2b70327216af33607a5d88bfa'
ROOT=Path(__file__).resolve().parent.parent

def command(*args):
    return subprocess.check_output(args,cwd=ROOT,text=True).strip()

def release(tag):
    return json.loads(command('gh','api',f'repos/{REPO}/releases/tags/{tag}'))

def fingerprint(assets):
    return {a['name']:{k:a.get(k) for k in ['id','size','digest']} for a in assets}

def main():
    work=Path('/tmp/greenspray-moving-pilot');work.mkdir(parents=True,exist_ok=True)
    before=release('current');source=release(TAG)
    original=next(a for a in source['assets'] if a['name']=='Dynamic.zip')
    if original.get('digest')!='sha256:'+EXPECTED_SHA:raise RuntimeError('Unexpected moving input digest; refusing different input')
    command('gh','release','download',TAG,'--repo',REPO,'--pattern','Dynamic.zip','--dir',str(work),'--clobber')
    pcap,prep=prepare_local(work/'Dynamic.zip',work/'input',15.,EXPECTED_SHA)
    prep['source_release']=f'https://github.com/{REPO}/releases/tag/{TAG}'
    prep['source_asset_id']=original['id']
    (pcap.parent/'preparation.json').write_text(json.dumps(prep,indent=2)+'\n')
    report=run(pcap,work/'output')
    after=release('current');source_after=release(TAG)
    if fingerprint(before['assets'])!=fingerprint(after['assets']):raise RuntimeError('Current assets changed during pilot; investigate concurrent update')
    if fingerprint([original])!=fingerprint([a for a in source_after['assets'] if a['name']=='Dynamic.zip']):raise RuntimeError('Moving input changed during pilot')
    report['publication']=dict(source_code_commit=os.environ.get('GITHUB_SHA'),current_release_unchanged=True,original_input_unchanged=True,download_release=TAG)
    (work/'output/pilot_report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    archive=work/'pilot-15s-results.zip'
    with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
        for p in sorted((work/'output').rglob('*')):
            if p.is_file():z.write(p,Path('dynamic_pilot_15s')/p.relative_to(work/'output'))
    shutil.copyfile(work/'output/pilot_report.json',work/'pilot-15s-report.json')
    shutil.copyfile(work/'output/pilot_preview.png',work/'pilot-15s-preview.png')
    command('gh','release','upload',TAG,'--repo',REPO,str(archive),str(work/'pilot-15s-report.json'),str(work/'pilot-15s-preview.png'),'--clobber')
    # Small derived files in git allow review without another large data download.
    # Full scans, full trajectory poses and raw GPS locations are never committed.
    command('git','pull','--ff-only','origin','main')
    result=ROOT/'results/dynamic_pilot_15s.json';result.parent.mkdir(parents=True,exist_ok=True)
    result.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    preview=ROOT/'docs/QC/dynamic_pilot/preview.png';preview.parent.mkdir(parents=True,exist_ok=True)
    shutil.copyfile(work/'output/pilot_preview.png',preview)
    command('git','config','user.name','github-actions[bot]')
    command('git','config','user.email','41898282+github-actions[bot]@users.noreply.github.com')
    command('git','add',str(result.relative_to(ROOT)),str(preview.relative_to(ROOT)))
    command('git','commit','-m','Record bounded moving lidar pilot results [skip ci]')
    command('git','push','origin','HEAD:main')
    summary={k:report[k] for k in ['status','flags','processed_frames','processed_time_span_s','retained_first_return_points','trajectory','GNSS_diagnostic','publication']}
    print(json.dumps(summary,indent=2),flush=True)
    if os.environ.get('GITHUB_STEP_SUMMARY'):
        with open(os.environ['GITHUB_STEP_SUMMARY'],'a') as f:
            f.write('# 15-second moving pilot\n\n```json\n'+json.dumps(summary,indent=2)+'\n```\n\nDownload: https://github.com/'+REPO+'/releases/tag/'+TAG+'\n')

if __name__=='__main__':main()
