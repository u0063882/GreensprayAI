"""Voxel/SDF shrink-wrap-style canopy proxy; not an implementation of Rhino internals."""
import os
os.environ.setdefault('OMP_NUM_THREADS','2');os.environ.setdefault('OPENBLAS_NUM_THREADS','2')
os.environ.setdefault('MPLCONFIGDIR',os.path.join(os.getcwd(),'work','mpl'))
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
    if len(points)<4 or not np.isfinite(points).all():raise ValueError('Need finite 3-D canopy points.')
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
    v+=origin
    mesh=o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(v),o3d.utility.Vector3iVector(f))
    mesh.remove_duplicated_vertices();mesh.remove_duplicated_triangles();mesh.remove_degenerate_triangles();mesh.remove_unreferenced_vertices();mesh.orient_triangles();mesh.compute_vertex_normals();mesh.compute_triangle_normals()
    # Positive oriented volume, without claiming this inferred proxy is measured biomass.
    v=np.asarray(mesh.vertices);f=np.asarray(mesh.triangles)
    if np.sum(np.einsum('ij,ij->i',v[f[:,0]],np.cross(v[f[:,1]],v[f[:,2]])))<0:
        mesh.triangles=o3d.utility.Vector3iVector(f[:,[0,2,1]]);mesh.compute_vertex_normals();mesh.compute_triangle_normals()
    return mesh,dict(grid_shape=shape.tolist(),grid_voxels=int(np.prod(shape)),retained_occupied_voxels=int(mask.sum()),occupied_proxy_volume_m3=float(mask.sum()*voxel**3))


def oriented_volume(mesh):
    # Translate near the centroid for a numerically stable tetrahedral sum.
    v=np.asarray(mesh.vertices)-mesh.get_center();f=np.asarray(mesh.triangles)
    return float(np.sum(np.einsum('ij,ij->i',v[f[:,0]],np.cross(v[f[:,1]],v[f[:,2]])))/6)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',type=Path,default=Path('output/orchard_row_registered.ply'))
    parser.add_argument('--output-dir',type=Path,default=Path('output/mesh'))
    parser.add_argument('--voxel',type=float,default=.025)
    parser.add_argument('--offset',type=float,default=.04)
    parser.add_argument('--closing-radius',type=float,default=.05)
    args=parser.parse_args()
    if args.voxel<=0 or args.offset<args.voxel or args.closing_radius<0:parser.error('Need voxel>0, offset>=voxel, closing-radius>=0.')
    out=args.output_dir;out.mkdir(parents=True,exist_ok=True);started=time.monotonic()
    pc=o3d.io.read_point_cloud(str(args.input));p=np.asarray(pc.points)
    soil=o3d.io.read_triangle_mesh(str(out/'ground_surface.ply'));g=np.asarray(soil.vertices)
    if len(g)<100:raise RuntimeError('Run mesh.py first to build the supported ground surface.')
    _,ix=cKDTree(g[:,:2]).query(p[:,:2],workers=2);q=p[p[:,2]-g[ix,2]>.05]
    mesh,grid=build_wrap(q,args.voxel,args.offset,args.closing_radius)
    colors=plt.get_cmap('turbo')(np.clip(np.asarray(mesh.vertices)[:,2]/4,0,1))[:,:3];mesh.vertex_colors=o3d.utility.Vector3dVector(colors)
    print('Validating wrap topology',len(mesh.vertices),len(mesh.triangles),flush=True)
    watertight=bool(mesh.is_edge_manifold(allow_boundary_edges=False) and mesh.is_vertex_manifold() and mesh.is_orientable())
    if not watertight:raise RuntimeError('Closed-envelope validation failed; do not publish this as a solid.')
    for name in ['canopy_wrap.ply','canopy_wrap.stl','canopy_wrap.obj']:
        if not o3d.io.write_triangle_mesh(str(out/name),mesh,write_triangle_uvs=False):raise RuntimeError('Wrap export failed: '+name)
    combined=mesh+soil;combined.compute_vertex_normals();o3d.io.write_triangle_mesh(str(out/'canopy_wrap_with_ground.ply'),combined)
    # Inspect unsupported expansion. Distances describe an inferred offset envelope, not fit error.
    samples=mesh.sample_points_uniformly(number_of_points=30000);d=cKDTree(q).query(np.asarray(samples.points),workers=2)[0]
    _,component_counts,_=mesh.cluster_connected_triangles()
    report=dict(method='Voxel occupancy + distance-field offset + binary closing/fill + smoothed signed-distance marching cubes',rhino_equivalence='Shrink-wrap-style approximation; Rhino 8 proprietary ShrinkWrap internals are not reproduced or validated.',source_sha256=hashlib.sha256(args.input.read_bytes()).hexdigest(),voxel_m=args.voxel,outward_offset_m=args.offset,gap_closing_radius_m=args.closing_radius,min_component_volume_m3=.0005,vertices=len(mesh.vertices),triangles=len(mesh.triangles),components=len(component_counts),watertight=watertight,topology_validation='Closed-edge, vertex-manifold and orientability checks; full global triangle self-intersection search is not run. Surface construction uses topology-preserving Lewiner marching cubes.',edge_manifold=bool(mesh.is_edge_manifold()),vertex_manifold=bool(mesh.is_vertex_manifold()),surface_area_m2=float(mesh.get_surface_area()),envelope_proxy_volume_m3=abs(oriented_volume(mesh)),wrap_surface_to_observation_distance_m=dict(median=float(np.median(d)),p95=float(np.percentile(d,95))),grid=grid,elapsed_seconds=time.monotonic()-started,limitations=['Offset, gap closing, enclosed-void filling and crop-end caps infer unobserved geometry.','This is a canopy/solid proxy, not measured trunk diameters, biomass, leaf area or validated CFD geometry.','The closed canopy wrap is watertight; the ground sheet and combined canopy-plus-ground scene are open.','Nearby leaves/branches/trees can be merged at the chosen scale. Small isolated components are removed.','Existing cross-row registration uncertainty remains; the wrap does not correct it.'])
    (out/'shrinkwrap_quality.json').write_text(json.dumps(report,indent=2))
    preview=combined.simplify_quadric_decimation(target_number_of_triangles=40000);v=np.asarray(preview.vertices);f=np.asarray(preview.triangles);c=np.asarray(preview.vertex_colors)
    fig=plt.figure(figsize=(14,8),facecolor='#101823');ax=fig.add_subplot(111,projection='3d');ax.set_facecolor('#101823');fc=c[f].mean(1)
    n=np.cross(v[f[:,1]]-v[f[:,0]],v[f[:,2]]-v[f[:,0]]);n/=np.maximum(np.linalg.norm(n,axis=1)[:,None],1e-12);light=np.array([-.2,-.7,.7]);light/=np.linalg.norm(light);fc*= (.55+.45*abs(n@light))[:,None]
    ax.add_collection3d(Poly3DCollection(v[f],facecolors=fc,edgecolors='none'));ax.set(xlim=(-.7,6.8),ylim=(1.1,2.85),zlim=(-.15,4.4),xlabel='Along row (m)',ylabel='Across (m)',zlabel='Height (m)');ax.set_box_aspect([7.5,1.75,4.55]);ax.view_init(elev=14,azim=-75);ax.tick_params(colors='#cfdae5')
    for a in [ax.xaxis,ax.yaxis,ax.zaxis]:a.label.set_color('#cfdae5')
    fig.suptitle('ORCHARD · CLOSED SHRINK-WRAP-STYLE CANOPY PROXY',color='white',fontsize=17);fig.text(.5,.03,f'Voxel {args.voxel*100:g} cm · outward offset {args.offset*100:g} cm · closing radius {args.closing_radius*100:g} cm · ground shown separately',ha='center',color='#cfdae5');fig.savefig(out/'shrinkwrap_preview.png',dpi=150,facecolor=fig.get_facecolor(),bbox_inches='tight')
    (out/'README.txt').write_text('ORCHARD MESHES — SAME REGISTERED METRIC FRAME\n\nCLOSED SHRINK-WRAP-STYLE PROXY\ncanopy_wrap.ply / .stl / .obj: closed, manifold canopy proxy.\ncanopy_wrap_with_ground.ply: that proxy plus the open observed ground sheet.\nshrinkwrap_quality.json: actual validation, parameters and inference limits.\n\nOBSERVED SURFACES\norchard_mesh.ply / .obj: open observed tree patches plus ground.\ntree_observed_surface.ply and ground_surface.ply: separate observed components.\nmesh_quality.json: support/coverage checks; these surfaces are fragmented.\n\nThe wrap is an open-source voxel/SDF approximation, not Rhino ShrinkWrap\ncode or validated equivalence. It fills gaps and adds offset/crop-end caps.\nIts volume/area are parameter-dependent proxy measures, not biological truth.\nRegistration uncertainty is inherited by both mesh forms.\n')
    archive=out.parent/'orchard_mesh.zip'
    with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED,compresslevel=5) as z:
        names=['tree_observed_surface.ply','ground_surface.ply','orchard_mesh.ply','orchard_mesh.obj','README.txt','mesh_preview.png','mesh_quality.json','canopy_wrap.ply','canopy_wrap.stl','canopy_wrap.obj','canopy_wrap_with_ground.ply','shrinkwrap_quality.json','shrinkwrap_preview.png']
        for name in names:z.write(out/name,name)
    print(json.dumps(report,indent=2),flush=True);print('Archive',archive,archive.stat().st_size,flush=True)
if __name__=='__main__':main()
