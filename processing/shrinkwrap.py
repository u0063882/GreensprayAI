"""Voxel/SDF shrink-wrap-style canopy proxy; not an implementation of Rhino internals."""
import os
import tempfile
os.environ.setdefault('OMP_NUM_THREADS','2');os.environ.setdefault('OPENBLAS_NUM_THREADS','2')
os.environ.setdefault('MPLCONFIGDIR',os.path.join(tempfile.gettempdir(),'greenspray-mpl'))
import argparse,json,hashlib,time,zipfile
from pathlib import Path
import numpy as np
from scipy import ndimage
from scipy.spatial import cKDTree
from skimage.measure import marching_cubes
import open3d as o3d
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


def build_wrap(points,voxel=.025,offset=.04,closing=.05,min_volume=.0005):
    points=np.asarray(points,dtype=float)
    if points.ndim!=2 or points.shape[1]!=3 or len(points)<4 or not np.isfinite(points).all():raise ValueError('Need at least four finite 3-D points.')
    if not np.isfinite([voxel,offset,closing,min_volume]).all() or voxel<=0 or offset<voxel or closing<0 or min_volume<0:raise ValueError('Need voxel>0, offset>=voxel, closing>=0 and min_volume>=0.')
    margin=offset+closing+4*voxel
    origin=points.min(0)-margin;shape=np.ceil((points.max(0)+margin-origin)/voxel).astype(int)+1
    if np.prod(shape)>35_000_000:raise ValueError('Grid exceeds 35 million voxels; increase --voxel.')
    print('Wrap grid',shape.tolist(),flush=True)
    occupied=np.zeros(tuple(shape),dtype=bool);idx=np.rint((points-origin)/voxel).astype(int);occupied[tuple(idx.T)]=True
    # Union of measured-point neighbourhoods, followed by explicit hole/gap inference.
    mask=ndimage.distance_transform_edt(~occupied,sampling=voxel)<=offset
    if closing>0:
        r=int(np.ceil(closing/voxel));a=np.arange(-r,r+1);xx,yy,zz=np.meshgrid(a,a,a,indexing='ij');ball=xx*xx+yy*yy+zz*zz<=(closing/voxel)**2
        mask=ndimage.binary_closing(mask,structure=ball)
    mask=ndimage.binary_fill_holes(mask)
    labels,n=ndimage.label(mask);counts=np.bincount(labels.ravel());keep=counts*voxel**3>=min_volume;keep[0]=False;mask=keep[labels]
    if not np.any(mask):raise ValueError('No occupied wrap remains after component filtering.')
    signed=ndimage.distance_transform_edt(mask,sampling=voxel)-ndimage.distance_transform_edt(~mask,sampling=voxel)
    signed=ndimage.gaussian_filter(signed.astype('f4'),sigma=.65)
    print('Extracting signed-field surface',flush=True)
    v,f,_,_=marching_cubes(signed,level=0,spacing=(voxel,voxel,voxel),allow_degenerate=False,method='lewiner')
    v=v.astype(np.float64)+origin
    mesh=o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(v),o3d.utility.Vector3iVector(f))
    mesh.remove_duplicated_vertices();mesh.remove_duplicated_triangles();mesh.remove_degenerate_triangles();mesh.remove_unreferenced_vertices();mesh.orient_triangles();mesh.compute_vertex_normals();mesh.compute_triangle_normals()
    # Translate before the orientation sum to retain precision in large local frames.
    if oriented_volume(mesh)<0:
        f=np.asarray(mesh.triangles)
        mesh.triangles=o3d.utility.Vector3iVector(f[:,[0,2,1]]);mesh.compute_vertex_normals();mesh.compute_triangle_normals()
    return mesh,dict(grid_shape=shape.tolist(),grid_voxels=int(np.prod(shape)),retained_occupied_voxels=int(mask.sum()),occupied_proxy_volume_m3=float(mask.sum()*voxel**3))


def oriented_volume(mesh):
    # Translate near the centroid for a numerically stable tetrahedral sum.
    v=np.asarray(mesh.vertices)-mesh.get_center();f=np.asarray(mesh.triangles)
    return float(np.sum(np.einsum('ij,ij->i',v[f[:,0]],np.cross(v[f[:,1]],v[f[:,2]])))/6)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',type=Path,required=True,help='Point cloud in metres; no registration files required.')
    parser.add_argument('--output-dir',type=Path,default=Path('mesh_output'))
    parser.add_argument('--metadata',type=Path,help='Optional passive JSON: units (m), coordinate_frame and other provenance. Never applies transforms.')
    parser.add_argument('--ground',type=Path,help='Optional ground triangle mesh in the input frame. Only with this option, wrap points above the Z-up ground.')
    parser.add_argument('--ground-clearance',type=float,default=.05,help='Height above optional ground, metres. Ignored without --ground.')
    parser.add_argument('--voxel',type=float,default=.025)
    parser.add_argument('--offset',type=float,default=.04)
    parser.add_argument('--closing-radius',type=float,default=.05)
    parser.add_argument('--min-volume',type=float,default=.0005)
    parser.add_argument('--archive',type=Path,help='ZIP destination; default: sibling of output directory, named after it.')
    parser.add_argument('--include-observed',action='store_true',help='Orchard release recipe only: explicitly include the observed meshes already in output-dir.')
    args=parser.parse_args(argv)
    settings=[args.voxel,args.offset,args.closing_radius,args.min_volume,args.ground_clearance]
    if not np.isfinite(settings).all() or args.voxel<=0 or args.offset<args.voxel or min(args.closing_radius,args.min_volume,args.ground_clearance)<0:
        parser.error('Need finite voxel>0, offset>=voxel and nonnegative closing radius, volume and ground clearance.')
    metadata=json.loads(args.metadata.read_text()) if args.metadata else {}
    if not isinstance(metadata,dict):parser.error('Metadata must be a JSON object.')
    if metadata.get('units','m')!='m':parser.error('Convert input coordinates and length parameters to metres before meshing.')
    if args.ground and metadata.get('up_axis','Z').upper()!='Z':parser.error('The optional ground recipe requires Z-up coordinates.')
    observed=['tree_observed_surface.ply','ground_surface.ply','orchard_mesh.ply','orchard_mesh.obj','mesh_preview.png','mesh_quality.json'] if args.include_observed else []
    if args.include_observed and (not args.ground or any(not (args.output_dir/name).is_file() for name in observed)):
        parser.error('--include-observed requires --ground and all observed surface outputs in --output-dir.')
    out=args.output_dir;out.mkdir(parents=True,exist_ok=True);started=time.monotonic()
    pc=o3d.io.read_point_cloud(str(args.input));p=np.asarray(pc.points)
    if p.ndim!=2 or p.shape[1]!=3 or len(p)<4 or not np.isfinite(p).all():parser.error('Input must contain at least four finite 3-D points.')
    soil=None;q=p
    if args.ground:
        soil=o3d.io.read_triangle_mesh(str(args.ground));g=np.asarray(soil.vertices)
        if len(g)<3 or not len(soil.triangles) or not np.isfinite(g).all():parser.error('Ground must be a finite triangle mesh in the input frame.')
        _,ix=cKDTree(g[:,:2]).query(p[:,:2],workers=2);q=p[p[:,2]-g[ix,2]>args.ground_clearance]
    mesh,grid=build_wrap(q,args.voxel,args.offset,args.closing_radius,args.min_volume)
    heights=np.asarray(mesh.vertices)[:,2];colors=plt.get_cmap('turbo')((heights-heights.min())/max(np.ptp(heights),1e-12))[:,:3]
    mesh.vertex_colors=o3d.utility.Vector3dVector(colors)
    print('Validating wrap topology',len(mesh.vertices),len(mesh.triangles),flush=True)
    closed_manifold=bool(mesh.is_edge_manifold(allow_boundary_edges=False) and mesh.is_vertex_manifold() and mesh.is_orientable())
    if not closed_manifold:raise RuntimeError('Closed-envelope validation failed; do not publish this as a solid.')
    names=[]
    for name in ['canopy_wrap.ply','canopy_wrap.stl','canopy_wrap.obj']:
        if not o3d.io.write_triangle_mesh(str(out/name),mesh,write_triangle_uvs=False):raise RuntimeError('Wrap export failed: '+name)
        names.append(name)
    combined=mesh
    if soil is not None:
        combined=mesh+soil;combined.compute_vertex_normals()
        if not o3d.io.write_triangle_mesh(str(out/'canopy_wrap_with_ground.ply'),combined):raise RuntimeError('Combined export failed.')
        names.append('canopy_wrap_with_ground.ply')
    o3d.utility.random.seed(2026)
    samples=mesh.sample_points_uniformly(number_of_points=30000);d=cKDTree(q).query(np.asarray(samples.points),workers=2)[0]
    _,component_counts,_=mesh.cluster_connected_triangles()
    report=dict(method='Voxel occupancy + distance-field offset + binary closing/fill + smoothed signed-distance marching cubes',rhino_equivalence='Shrink-wrap-style approximation; Rhino 8 proprietary ShrinkWrap internals are not reproduced or validated.',source_sha256=hashlib.sha256(args.input.read_bytes()).hexdigest(),source_file=args.input.name,coordinate_frame=metadata.get('coordinate_frame','Input coordinates, preserved without rotation or translation'),units='m',metadata=metadata,input_points=len(p),meshed_input_points=len(q),ground_source_sha256=hashlib.sha256(args.ground.read_bytes()).hexdigest() if args.ground else None,ground_clearance_m=args.ground_clearance if args.ground else None,voxel_m=args.voxel,outward_offset_m=args.offset,gap_closing_radius_m=args.closing_radius,min_component_volume_m3=args.min_volume,vertices=len(mesh.vertices),triangles=len(mesh.triangles),components=len(component_counts),watertight=closed_manifold,topology_validation='Closed-edge, vertex-manifold and orientability checks; full global triangle self-intersection search is not run. The watertight field records these checks, not an independent intersection-free guarantee.',edge_manifold=bool(mesh.is_edge_manifold()),vertex_manifold=bool(mesh.is_vertex_manifold()),surface_area_m2=float(mesh.get_surface_area()),envelope_proxy_volume_m3=abs(oriented_volume(mesh)),wrap_surface_to_observation_distance_m=dict(median=float(np.median(d)),p95=float(np.percentile(d,95))),grid=grid,elapsed_seconds=time.monotonic()-started,limitations=['Offset, gap closing, enclosed-void filling and crop-end caps infer unobserved geometry.','This is a solid envelope proxy, not measured trunk diameters, biomass, leaf area or validated CFD geometry.','Adding an optional open ground sheet produces an open combined scene.','Nearby surfaces can merge at the chosen scale. Small isolated components are removed.','Input alignment uncertainty is inherited; meshing does not estimate or correct registration.'])
    (out/'shrinkwrap_quality.json').write_text(json.dumps(report,indent=2)+'\n')
    preview=combined.simplify_quadric_decimation(target_number_of_triangles=40000);v=np.asarray(preview.vertices);f=np.asarray(preview.triangles);c=np.asarray(preview.vertex_colors)
    fig=plt.figure(figsize=(14,8),facecolor='#101823');ax=fig.add_subplot(111,projection='3d');ax.set_facecolor('#101823');fc=c[f].mean(1) if len(c) else np.tile([.3,.6,.4],(len(f),1))
    n=np.cross(v[f[:,1]]-v[f[:,0]],v[f[:,2]]-v[f[:,0]]);n/=np.maximum(np.linalg.norm(n,axis=1)[:,None],1e-12);light=np.array([-.2,-.7,.7]);light/=np.linalg.norm(light);fc*= (.55+.45*abs(n@light))[:,None]
    low=v.min(0);high=v.max(0);span=np.maximum(high-low,args.voxel);pad=span*.04
    ax.add_collection3d(Poly3DCollection(v[f],facecolors=fc,edgecolors='none'));ax.set(xlim=(low[0]-pad[0],high[0]+pad[0]),ylim=(low[1]-pad[1],high[1]+pad[1]),zlim=(low[2]-pad[2],high[2]+pad[2]),xlabel='X (m)',ylabel='Y (m)',zlabel='Z (m)');ax.set_box_aspect(span);ax.view_init(elev=14,azim=-75);ax.tick_params(colors='#cfdae5')
    for a in [ax.xaxis,ax.yaxis,ax.zaxis]:a.label.set_color('#cfdae5')
    fig.suptitle('CLOSED POINT-CLOUD ENVELOPE · VOXEL/SDF PROTOTYPE',color='white',fontsize=17)
    fig.text(.5,.03,f'Voxel {args.voxel:g} m · outward offset {args.offset:g} m · closing radius {args.closing_radius:g} m',ha='center',color='#cfdae5');fig.savefig(out/'shrinkwrap_preview.png',dpi=150,facecolor=fig.get_facecolor(),bbox_inches='tight');plt.close(fig)
    (out/'README.txt').write_text('POINT-CLOUD ENVELOPE — INPUT COORDINATES PRESERVED, METRES\n\ncanopy_wrap.ply / .stl / .obj: closed envelope (legacy filenames).\nshrinkwrap_quality.json: input hash, parameters, actual checks and limits.\ncanopy_wrap_with_ground.ply, when requested: envelope plus an optional ground mesh.\nOther observed surface files, when explicitly included: open orchard surfaces.\n\nNo raw scans, registration caches, GPS or poses are required.\nThis voxel/SDF prototype is not Rhino ShrinkWrap or validated equivalence.\nOffset, closing and caps infer unobserved geometry; volume/area depend on settings.\nRead the quality report: a full global self-intersection check is not performed.\n')
    names.extend(['README.txt','shrinkwrap_quality.json','shrinkwrap_preview.png',*observed])
    archive=args.archive or out.parent/(out.name+'.zip');archive.parent.mkdir(parents=True,exist_ok=True)
    with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED,compresslevel=5) as z:
        for name in names:z.write(out/name,name)
    print(json.dumps(report,indent=2),flush=True);print('Archive',archive,archive.stat().st_size,flush=True)
if __name__=='__main__':main()
