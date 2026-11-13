#!/usr/bin/env python3
"""Export the unchanged registered cloud as native Rhino and simple ASCII PLY.

No registration, meshing, normal estimation, resampling or filtering is run.
The enriched source PLY remains the complete sensor-data record.
"""
from pathlib import Path
import argparse,hashlib,json,os
import numpy as np

PLY_TYPES={'float':'<f4','float32':'<f4','double':'<f8','float64':'<f8','uchar':'u1','uint8':'u1','ushort':'<u2','uint16':'<u2','uint':'<u4','uint32':'<u4','int':'<i4','int32':'<i4','short':'<i2','char':'i1'}


def digest(path):
    with Path(path).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()


def read_vertices(path):
    """Read scalar point-only binary PLY with exact header/record-size checks."""
    path=Path(path)
    with path.open('rb') as f:
        if f.readline()!=b'ply\n':raise ValueError('Expected PLY signature')
        properties=[];count=None;fmt=None
        for _ in range(250):
            line=f.readline().decode('ascii').strip();t=line.split()
            if not t:raise ValueError('Truncated PLY header')
            if t[0]=='format':fmt=t[1]
            elif t[:2]==['element','vertex']:count=int(t[2])
            elif t[0]=='element':raise ValueError('Expected a point-only source PLY')
            elif t[0]=='property':
                if len(t)!=3 or t[1] not in PLY_TYPES:raise ValueError('Unsupported scalar PLY property')
                properties.append((t[2],PLY_TYPES[t[1]]))
            elif line=='end_header':break
        else:raise ValueError('PLY header too long')
        if fmt!='binary_little_endian' or count is None or count<=0:raise ValueError('Expected nonempty binary little-endian PLY')
        dtype=np.dtype(properties)
        if path.stat().st_size-f.tell()!=count*dtype.itemsize:raise ValueError('PLY payload size disagrees with the declared vertex layout')
        records=np.fromfile(f,dtype=dtype,count=count)
    for key in ['x','y','z','red','green','blue']:
        if key not in records.dtype.names:raise ValueError('Missing '+key)
    p=np.c_[records['x'],records['y'],records['z']]
    if not np.isfinite(p).all():raise ValueError('Non-finite point coordinates')
    return records


def xyz(records):return np.c_[records['x'],records['y'],records['z']]
def rgb(records):return np.c_[records['red'],records['green'],records['blue']]


def write_ascii(path,records):
    """Only XYZ/RGB; 9 significant digits round-trip every source float32."""
    header='ply\nformat ascii 1.0\ncomment Registered measured points; coordinates in metres\nelement vertex '+str(len(records))+'\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n'
    with Path(path).open('w',encoding='ascii',newline='\n') as f:
        f.write(header)
        for lo in range(0,len(records),50000):
            d=records[lo:lo+50000];data=np.c_[xyz(d),rgb(d)]
            np.savetxt(f,data,fmt=['%.9g','%.9g','%.9g','%d','%d','%d'])
    # Independent text round-trip, without assuming binary record offsets.
    dtype=np.dtype([('x','<f4'),('y','<f4'),('z','<f4'),('red','u1'),('green','u1'),('blue','u1')])
    saved=np.loadtxt(path,skiprows=len(header.splitlines()),dtype=dtype,ndmin=1)
    if len(saved)!=len(records):raise RuntimeError('ASCII PLY point count changed')
    for key in dtype.names:np.testing.assert_array_equal(saved[key],records[key])
    return dict(points=len(saved),coordinate_and_RGB_roundtrip_exact=True,coordinate_units='metres',properties=list(dtype.names),encoding='ASCII')


def write_native(path,records,source_sha,scan_names=None):
    import rhino3dm as rh
    model=rh.File3dm();model.ApplicationName='GreensprayAI';model.ApplicationDetails='Registered measured cloud; native Rhino compatibility export';model.Settings.ModelUnitSystem=rh.UnitSystem.Millimeters;model.Settings.ModelAbsoluteTolerance=.1
    ids=records['scan_id'] if 'scan_id' in records.dtype.names else np.zeros(len(records),dtype=np.uint32);scan_names=scan_names or {};point_counts={}
    for sid in np.unique(ids):
        sid=int(sid);d=records[ids==sid];p=xyz(d).astype(float)*1000.;c=rgb(d);values=d['signal'] if 'signal' in d.dtype.names else d['intensity'] if 'intensity' in d.dtype.names else None
        cloud=rh.PointCloud()
        # Initialize the optional scalar array through an actual first point;
        # rhino3dm setters do not allocate that array on an RGB-only cloud.
        if values is not None:
            cloud.Add(rh.Point3d(*p[0]),float(values[0]));cloud[0].Color=tuple(map(int,c[0]))+(255,);start=1
        else:start=0
        for lo in range(start,len(d),30000):
            hi=min(len(d),lo+30000);cloud.AddRange([rh.Point3d(*v) for v in p[lo:hi]],[tuple(map(int,v))+(255,) for v in c[lo:hi]])
            if values is not None:
                for j in range(lo,hi):cloud[j].Value=float(values[j])
        if not cloud.IsValid or len(cloud)!=len(d) or not cloud.ContainsColors or cloud.HiddenPointCount:raise RuntimeError('Invalid/hidden native point-cloud geometry')
        if values is not None and not cloud.ContainsValues:raise RuntimeError('Native intensity values missing')
        layer=rh.Layer();layer.Name=f'scan_{sid:03d}';layer.Color=tuple(map(int,np.median(c,axis=0)))+(255,);layer_index=model.Layers.Add(layer)
        attributes=rh.ObjectAttributes();attributes.LayerIndex=layer_index;attributes.Name=f'scan_{sid:03d} '+scan_names.get(sid,'');attributes.ColorSource=rh.ObjectColorSource.ColorFromLayer
        for key,value in dict(scan_id=str(sid),source_scan=scan_names.get(sid,''),source_ply='../final_registered.ply',source_ply_sha256=source_sha,source_coordinates='metres',native_coordinates='millimetres',metres_to_model_scale='1000',point_values='raw SIGNAL/intensity' if values is not None else 'not present').items():attributes.SetUserString(key,value)
        model.Objects.AddPointCloud(cloud,attributes);point_counts[str(sid)]=len(d)
    model.Strings['GreensprayAI.source_ply_sha256']=source_sha;model.Strings['GreensprayAI.source_units']='metres';model.Strings['GreensprayAI.coordinate_conversion']='Native coordinates = source XYZ * 1000; model units millimetres; physical geometry unchanged.'
    # A saved camera/frustum encloses the data; Rhino supplies the window's
    # screen dimensions on open. No existing user's viewport is modified.
    p=xyz(records).astype(float)*1000.;centre=(p.min(0)+p.max(0))/2;span=max(float(np.linalg.norm(np.ptp(p,axis=0))),1.);view=rh.ViewInfo();view.Name='Registered orchard';vp=view.Viewport;camera=centre+span*np.array([1.15,-1.6,.85]);vp.SetCameraLocation(rh.Point3d(*camera));vp.SetCameraDirection(rh.Vector3d(*(centre-camera)));vp.SetCameraUp(rh.Vector3d(0,0,1));vp.ChangeToParallelProjection(True);vp.SetFrustum(-.8*span,.8*span,-.6*span,.6*span,.01*span,10*span);vp.TargetPoint=rh.Point3d(*centre);model.Views.Add(view)
    if not model.Write(str(path),8):raise RuntimeError('Could not write Rhino 8 native model')
    saved=rh.File3dm.Read(str(path))
    if saved is None or saved.Settings.ModelUnitSystem!=rh.UnitSystem.Millimeters:raise RuntimeError('Native model read-back/units failed')
    total=0
    for obj in saved.Objects:
        cloud=obj.Geometry
        if not isinstance(cloud,rh.PointCloud):raise RuntimeError('Native export contains non-point-cloud geometry')
        sid=int(obj.Attributes.GetUserString('scan_id'));d=records[ids==sid];p=xyz(d).astype(float)*1000.;c=rgb(d)
        if not cloud.IsValid or len(cloud)!=len(d) or cloud.HiddenPointCount:raise RuntimeError('Native saved cloud is invalid/hidden/truncated')
        actual=np.array([[v.X,v.Y,v.Z] for v in cloud.GetPoints2()]);np.testing.assert_array_equal(actual,p)
        np.testing.assert_array_equal(np.array(cloud.GetColors2(),dtype=np.uint8)[:,:3],c)
        if 'signal' in d.dtype.names or 'intensity' in d.dtype.names:np.testing.assert_array_equal(np.array(cloud.GetValues2()),d['signal'] if 'signal' in d.dtype.names else d['intensity'])
        total+=len(cloud)
    if total!=len(records):raise RuntimeError('Native total point count changed')
    return dict(points=total,objects=len(point_counts),per_scan_points=point_counts,model_units='millimetres',coordinate_scale_from_metres=1000.,point_coordinates_and_RGB_roundtrip_exact=True,raw_intensity_in_native_point_values=bool({'signal','intensity'} & set(records.dtype.names)),scan_id_preserved_as_object_and_layer=True,rhino3dm_version=rh.__version__,Rhino_8_UI_tested=False)


def export_file(source,output,scan_names=None):
    source=Path(source);output=Path(output);output.mkdir(parents=True,exist_ok=True);source_sha=digest(source);d=read_vertices(source)
    report=dict(source_file=source.name,source_sha256=source_sha,source_points=len(d),source_record_bytes=d.dtype.itemsize,source_bounds_metres=dict(min=xyz(d).min(0).tolist(),max=xyz(d).max(0).tolist()),native=write_native(output/'final_registered.3dm',d,source_sha,scan_names),ascii=write_ascii(output/'final_registered_xyzrgb_ascii.ply',d))
    if digest(source)!=source_sha:raise RuntimeError('The enriched source cloud was changed during conversion')
    report['source_file_unchanged']=True;report['outputs']={p.name:dict(bytes=p.stat().st_size,sha256=digest(p)) for p in output.iterdir() if p.name in ['final_registered.3dm','final_registered_xyzrgb_ascii.ply']}
    (output/'export_quality.json').write_text(json.dumps(report,indent=2)+'\n')
    (output/'README.txt').write_text('RHINO 8 POINT CLOUD\n\nPreferred: File > Open > final_registered.3dm\nNative model units are MILLIMETRES. Coordinates are the exact registered metre coordinates multiplied by 1000; physical geometry is unchanged.\nEach original scan is one visible point-cloud object on its own scan_ID layer. Point RGB is the original leaf/wood colour, and native Point Values contain raw SIGNAL/intensity. Select all scan clouds together for ShrinkWrap.\nIf needed: Ctrl+A, then run Zoom Selected (or Zoom Extents).\nYour 15 mm ShrinkWrap edge length and -5 mm offset are entered as 15 and -5 in this native model.\n\nFallback: final_registered_xyzrgb_ascii.ply contains only X Y Z R G B, in METRES. If Rhino asks for source units, choose metres. No binary interpretation or custom PLY fields are required.\n\nThe parent final_registered.ply remains the complete enriched sensor record, with original intensity, NIR, scan_id, frame_id and pixel_index. These analysis fields have not been removed from it.\nNo registration, filtering, meshing, resampling or geometric correction is performed during this export.\nThe native file was reopened and all points, colours, raw intensity and unit conversion checked using McNeel rhino3dm. The Rhino 8 desktop UI is not available in this Linux environment; a successful import in your Rhino build is still to be confirmed.\n')
    return report


def export_collection(pointclouds_root):
    root=Path(pointclouds_root);reports={}
    for source in sorted(root.glob('*/final_registered.ply')):
        names={};metadata=source.parent/'source_scans.json'
        if metadata.exists():names={int(e['scan_id']):e['scan'] for e in json.loads(metadata.read_text())['scans']}
        print('Rhino export START',source.parent.name,flush=True);reports[source.parent.name]=export_file(source,source.parent/'rhino',names);print('Rhino export DONE',source.parent.name,reports[source.parent.name]['source_points'],flush=True)
    if not reports:raise ValueError('No dataset final_registered.ply files')
    return reports

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--pointclouds-root',type=Path,required=True);a=p.parse_args();export_collection(a.pointclouds_root)
