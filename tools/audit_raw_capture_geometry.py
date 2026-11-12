#!/usr/bin/env python3
"""Check original east/west PCAP geometry using IMU up, without registration."""
from pathlib import Path
import argparse,csv,json,sys,os,hashlib
os.environ.setdefault('OMP_NUM_THREADS','2')
os.environ.setdefault('OPENBLAS_NUM_THREADS','2')
import numpy as np
ROOT=Path(__file__).resolve().parent.parent
sys.path.insert(0,str(ROOT))
from captures.decode import selected_recordings
from captures.raw_io import decode_raw_frame
from captures.raw_geometry import analyse_raw_cloud


def geographic_sides(reports):
    groups={}
    for r in reports:groups.setdefault(r['side'],[]).append([r['latitude'],r['longitude']])
    if len(groups)!=2:raise ValueError('Two acquisition sides required')
    median={k:np.median(v,axis=0) for k,v in groups.items()}
    ordered=sorted(median,key=lambda k:median[k][1])
    separation=(median[ordered[1]][1]-median[ordered[0]][1])*np.pi/180*6378137*np.cos(np.deg2rad(np.mean([v[0] for v in median.values()])))
    if separation<.5:raise ValueError('GPS easting does not reliably distinguish east and west tracks')
    return {ordered[0]:'west',ordered[1]:'east'},dict(median_lat_lon={k:v.tolist() for k,v in median.items()},median_easting_separation_m=float(separation),method='Geographic east/west from median original H5 RTK longitude; no assumed A/B meaning')


def summarise(reports,mapping):
    result={}
    for label in ['east','west']:
        rows=[r for r in reports if r.get('geographic_side')==label and not r.get('error')]
        if not rows:continue
        stats={}
        for name,get in [('wall_from_vertical_deg',lambda r:r['tree_wall']['outward_tilt_from_vertical_deg']),
                ('ground_from_horizontal_deg',lambda r:r['ground_tilt_from_horizontal_deg']),
                ('wall_ground_dihedral_deg',lambda r:r['wall_ground_dihedral_deg'])]:
            v=np.array([get(r) for r in rows]);stats[name]=dict(median=float(np.median(v)),minimum=float(v.min()),maximum=float(v.max()),q25=float(np.quantile(v,.25)),q75=float(np.quantile(v,.75)))
        stats['scans']=len(rows);result[label]=stats
    if len(result)==2:
        e=result['east']['wall_from_vertical_deg']['median'];w=result['west']['wall_from_vertical_deg']['median']
        result['apparent_median_sheet_opening_deg']=float(e+w)
        result['difference_between_outward_tilts_deg']=float(e-w)
        result['opening_caution']='Sum of independently measured median near-side profile inclinations; not a matched individual-tree opening or a surveyed biological branch angle.'
    return result


def run_audit(data_root,output,dataset='vhedge',config=None,wood_model=None,repeat_last=False,plots=True,cache_directory=None):
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    cache_directory=Path(cache_directory) if cache_directory else output/'frames'
    exclusions=(config or {}).get('excluded_recordings',[]);files=selected_recordings(data_root,exclusions)
    model=json.loads(Path(wood_model).read_text()) if wood_model else None
    reports=[];repeats=[]
    for path in files:
        data,meta=decode_raw_frame(path,cache_directory)
        if model:
            from captures.leafwood import classify_leafwood
            data['wood_score']=classify_leafwood(data['signal'],data['near_ir'],model)['wood_score']
        try:
            if meta['selected_column_coverage']<.95:raise ValueError('Insufficient packet-column completeness')
            report,_,_=analyse_raw_cloud(data,meta)
        except Exception as e:report={**meta,'error':str(e)}
        report['side']=path.parent.name;report['scan']=path.parent.name+'_'+path.stem[-6:];reports.append(report)
        if repeat_last and not report.get('error'):
            second,md=decode_raw_frame(path,cache_directory,frame_choice='last_complete')
            if model:
                from captures.leafwood import classify_leafwood
                second['wood_score']=classify_leafwood(second['signal'],second['near_ir'],model)['wood_score']
            try:
                rr,_,_=analyse_raw_cloud(second,md)
                repeats.append(dict(scan=report['scan'],first_frame=meta['selected_frame'],last_frame=md['selected_frame'],
                    first_wall_deg=report['tree_wall']['outward_tilt_from_vertical_deg'],last_wall_deg=rr['tree_wall']['outward_tilt_from_vertical_deg'],
                    first_ground_deg=report['ground_tilt_from_horizontal_deg'],last_ground_deg=rr['ground_tilt_from_horizontal_deg']))
            except Exception as e:repeats.append(dict(scan=report['scan'],error=str(e)))
        print('Raw geometry:',report['scan'],report.get('tree_wall',{}).get('outward_tilt_from_vertical_deg',report.get('error')),flush=True)
    mapping,gps=geographic_sides(reports)
    for r in reports:r['geographic_side']=mapping[r['side']]
    summary=summarise(reports,mapping)
    result=dict(dataset=dataset,policy='Read-only raw-source audit; no current output cloud, registration transform, terrain levelling or PCA-up frame is read or changed.',
        gps_sides=gps,side_mapping=mapping,primary_method='Near-side vegetation centre sheet from equally weighted along-row/height-cell medians; local row axis from low-stem/support candidates; wall fit free to lean. This measures visible structure, not a verified individual branch.',
        angle_definitions=dict(wall_from_vertical='Outward near-side centre-sheet tilt from IMU vertical; positive leans toward the viewing track',ground_from_horizontal='Detected ground-plane normal angle from IMU up; ground is not flattened',wall_ground_dihedral='Acute dihedral between independently fitted ground and vegetation-sheet planes'),
        summary=summary,reports=reports,repeat_frames=repeats,excluded_inputs=exclusions,
        secondary_wood_model_sha256=hashlib.sha256(Path(wood_model).read_bytes()).hexdigest() if wood_model else None,
        limits=['The IMU median is used without inferred bias correction; factory calibration uncertainty remains.',
                'Occlusion, supports, crown curvature and feature selection affect a vegetation-sheet inclination.',
                'Different scans view overlapping trees; scan-to-scan ranges are not independent survey confidence intervals.',
                'Low-cylinder roles are inferred from fitted size, not verified tree/post identities.',
                'Presence of a feature in these raw scans rules out its creation by later registration, but does not alone establish biological shape.'])
    (output/'raw_geometry_report.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    fields=['scan','geographic_side','selected_frame','wall_from_vertical_deg','ground_from_horizontal_deg','wall_ground_dihedral_deg','low_stem_candidates','supported_low_cylinders']
    with (output/'raw_angles.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader()
        for r in reports:
            if r.get('error'):continue
            writer.writerow(dict(scan=r['scan'],geographic_side=r['geographic_side'],selected_frame=r['selected_frame'],wall_from_vertical_deg=r['tree_wall']['outward_tilt_from_vertical_deg'],ground_from_horizontal_deg=r['ground_tilt_from_horizontal_deg'],wall_ground_dihedral_deg=r['wall_ground_dihedral_deg'],low_stem_candidates=len(r['low_stem_candidates']),supported_low_cylinders=len(r['raw_stem_fits'])))
    if plots:
        from captures.raw_geometry_qc import plot_raw_audit
        plot_raw_audit(result,output,cache_directory)
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--data-root',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--dataset',default='vhedge');p.add_argument('--config',type=Path);p.add_argument('--wood-model',type=Path,help='Optional intensity-only shared model for a secondary sensitivity check; primary geometry uses all returns');p.add_argument('--repeat-last-frame',action='store_true');a=p.parse_args()
    config=json.loads(a.config.read_text()) if a.config else None
    r=run_audit(a.data_root,a.output,a.dataset,config,a.wood_model,a.repeat_last_frame)
    print(json.dumps(r['summary'],indent=2))

if __name__=='__main__':main()
