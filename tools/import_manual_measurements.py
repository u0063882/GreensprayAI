"""Read measurement cells from the supplied workbook; do not execute Office content."""
from pathlib import Path
import argparse,hashlib,json,math,zipfile
import xml.etree.ElementTree as ET

NS={'m':'http://schemas.openxmlformats.org/spreadsheetml/2006/main'}


def read_manual_measurements(path):
    path=Path(path)
    with zipfile.ZipFile(path) as z:
        strings=[]
        if 'xl/sharedStrings.xml' in z.namelist():strings=[''.join(s.itertext()) for s in ET.fromstring(z.read('xl/sharedStrings.xml')).findall('m:si',NS)]
        workbook=ET.fromstring(z.read('xl/workbook.xml'));sheets=workbook.findall('m:sheets/m:sheet',NS)
        relationships={r.get('Id'):r.get('Target') for r in ET.fromstring(z.read('xl/_rels/workbook.xml.rels')) if r.get('TargetMode')!='External'}
        families={}
        for sheet in sheets:
            name=sheet.get('name');target=relationships[sheet.get('{http://schemas.openxmlformats.org/officeDocument/2006/relationships}id')]
            member=target.lstrip('/') if target.startswith('/') else 'xl/'+target
            root=ET.fromstring(z.read(member));rows=[]
            for row in root.findall('.//m:row',NS):
                cells={}
                for c in row.findall('m:c',NS):
                    v=c.find('m:v',NS);inline=c.find('m:is',NS);value=v.text if v is not None else ''.join(inline.itertext()) if inline is not None else ''
                    if c.get('t')=='s':value=strings[int(value)]
                    cells[''.join(ch for ch in c.get('r') if ch.isalpha())]=value
                rows.append(cells)
            observations=[];additional=[];definitions={};row_note=None
            unit=next((r.get('C','').strip().lower() for r in rows if r.get('B','').strip().lower()=='units'),None)
            if unit!='cm':raise ValueError('Expected centimetre circumferences in '+name)
            for row in rows:
                number=row.get('A','')
                if str(number).isdigit() and int(number)>0:
                    columns=['B','C','D','E','F','G'] if name=='Spindle' else ['B','C','D','E','F']
                    values={key:float(row[key]) for key in columns if row.get(key)}
                    if any(not math.isfinite(v) or v<=0 for v in values.values()):raise ValueError('Circumferences must be finite and positive')
                    observations.append(dict(measurement_tree_number=int(number),circumferences_cm={chr(ord(k)-1):v for k,v in values.items()},remark=row.get('H','') if name=='Spindle' else row.get('G','')))
                elif str(number).lower().startswith('pole'):
                    if row.get('F'):additional.append(dict(label=number,circumference_cm=float(row['F'])))
                if row.get('B') in ['A','B','C','D','E','F']:definitions[row['B']]=row.get('C','')
                if row.get('B')=='row':row_note=' '.join(str(row.get(k,'')) for k in ['C','D']).strip()
            trunk=[r['circumferences_cm']['A'] for r in observations]
            support=[r['circumferences_cm']['B'] for r in observations] if name=='Spindle' else [r['circumference_cm'] for r in additional]
            if not trunk or not support:raise ValueError('Missing trunk/support reference: '+name)
            def summary(values):return dict(circumferences_cm=values,diameters_cm=[v/math.pi for v in values],mean_diameter_cm=sum(values)/len(values)/math.pi,minimum_diameter_cm=min(values)/math.pi,maximum_diameter_cm=max(values)/math.pi)
            families[name.lower()]=dict(sheet=name,units='cm circumference',row_note=row_note,definitions=definitions,observations=observations,additional_supports=additional,trunk=summary(trunk),support=summary(support))
    return dict(version=1,source_filename=path.name,source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),tree_mapping='User confirmed representative sizes; no exact mapping to uploaded scan sections or stem-window numbers',
                measurement_height_m=.3,net_caveat='Trunk circumference can include loose netting; measurements are soft population references, not exact bark radii',post_caveat='C/pi gives circular-equivalent size; wooden posts may not be circular',families=families)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--input',type=Path,required=True);parser.add_argument('--output',type=Path,required=True);a=parser.parse_args()
    a.output.write_text(json.dumps(read_manual_measurements(a.input),indent=2)+'\n')
