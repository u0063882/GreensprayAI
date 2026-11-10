"""Reconstruct supported, open tree surfaces and a separate observed ground surface."""
import os
os.environ.setdefault('OMP_NUM_THREADS','2')
os.environ.setdefault('OPENBLAS_NUM_THREADS','2')
os.environ.setdefault('MPLCONFIGDIR',os.path.join(os.getcwd(),'work','mpl'))
import argparse,hashlib,json,time,zipfile
from pathlib import Path
import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree,Delaunay
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


def read_source_ids(path):
    """Read optional scan_id from this project's binary PLY without losing other fields."""
    types={'float':'<f4','double':'<f8','uchar':'u1','uint8':'u1','ushort':'<u2','int':'<i4','uint':'<u4'}
    with path.open('rb') as f:
        props=[];count=0;vertex=False;binary=False
        for _ in range(200):
            line=f.readline().decode('ascii').strip();s=line.split()
            if s[:2]==['format','binary_little_endian']:binary=True
            if s[:2]==['element','vertex']:count=int(s[2]);vertex=True
            elif s and s[0]=='element':vertex=False
            if vertex and s and s[0]=='property':
                if len(s)!=3 or s[1] not in types:return None
                props.append((s[2],types[s[1]]))
            if line=='end_header':break
        if not binary or 'scan_id' not in dict(props):return None
        a=np.fromfile(f,dtype=np.dtype(props),count=count)
        return np.array(a['scan_id'],dtype=int)


def clean(mesh):
    mesh.remove_degenerate_triangles();mesh.remove_duplicated_triangles();mesh.remove_duplicated_vertices();mesh.remove_unreferenced_vertices()
    mesh.compute_vertex_normals();mesh.compute_triangle_normals()
    return mesh


def summary(mesh):
    _,counts,_=mesh.cluster_connected_triangles()
    return dict(vertices=len(mesh.vertices),triangles=len(mesh.triangles),connected_components=len(counts),edge_manifold=bool(mesh.is_edge_manifold()),vertex_manifold=bool(mesh.is_vertex_manifold()),watertight=bool(mesh.is_watertight()),surface_area_m2=float(mesh.get_surface_area()))


def distance_check(mesh,points,rng):
    if not len(points) or not len(mesh.triangles):return None
    if len(points)>60000:points=points[rng.choice(len(points),60000,replace=False)]
    scene=o3d.t.geometry.RaycastingScene();scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
    d=scene.compute_distance(o3d.core.Tensor(points.astype('f4'))).numpy()
    return dict(sample_points=len(points),median_distance_m=float(np.median(d)),p95_distance_m=float(np.percentile(d,95)),fraction_within_3cm=float(np.mean(d<=.03)))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',type=Path,default=Path('output/orchard_row_registered.ply'))
    parser.add_argument('--transforms',type=Path,help='Optional scanner provenance; never required or auto-discovered.')
    parser.add_argument('--output-dir',type=Path,default=Path('output/mesh'))
    parser.add_argument('--voxel',type=float,default=.02)
    parser.add_argument('--min-component-triangles',type=int,default=1,help='Retain small supported leaf patches by default.')
    args=parser.parse_args()
    if args.min_component_triangles<1:parser.error('--min-component-triangles must be at least 1.')
    if not .005<=args.voxel<=.1:parser.error('--voxel must be between 5 mm and 10 cm.')
    started=time.monotonic();out=args.output_dir;out.mkdir(parents=True,exist_ok=True);rng=np.random.default_rng(76)
    pc=o3d.io.read_point_cloud(str(args.input));p=np.asarray(pc.points)
    if len(p)<100:raise RuntimeError('Too few input points.')
    if not np.isfinite(p).all():raise RuntimeError('Input contains non-finite coordinates.')
    colors=np.asarray(pc.colors)
    if not len(colors):colors=plt.get_cmap('turbo')(np.clip(p[:,2]/4,0,1))[:,:3]
    # Soil height is inferred locally from observed low returns, not forced onto Z=0.
    near=p[p[:,2]<.16];cells=np.floor(near[:,:2]/.04).astype(int)
    _,inv=np.unique(cells,axis=0,return_inverse=True);order=np.argsort(inv);groups=np.split(order,np.where(np.diff(inv[order]))[0]+1)
    terrain=[]
    for ix in groups:
        if len(ix)<4:continue
        q=near[ix];low=np.quantile(q[:,2],.30)
        q=q[abs(q[:,2]-low)<.025]
        if len(q)>=3:terrain.append(np.median(q,axis=0))
    ground=np.array(terrain)
    if len(ground)<100:raise RuntimeError('Insufficient observed ground for a supported surface.')
    gt=cKDTree(ground[:,:2]);_,ix=gt.query(p[:,:2],workers=2);height=p[:,2]-ground[ix,2]
    vegetation=height>.05;treepc=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(p[vegetation]));treepc.colors=o3d.utility.Vector3dVector(colors[vegetation]);treepc=treepc.voxel_down_sample(args.voxel)
    print('Tree reconstruction points:',len(treepc.points),flush=True)
    treepc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=max(.06,args.voxel*3.5),max_nn=40))
    ids=read_source_ids(args.input);orientation='local tangent-plane consistency (sensor provenance unavailable)'
    if ids is not None and len(ids)==len(p) and args.transforms is not None and args.transforms.is_file():
        transforms=json.loads(args.transforms.read_text())['scans'];origins=np.array([s['sensor_to_merged'] for s in transforms])[:,:3,3]
        _,nearest=cKDTree(p).query(np.asarray(treepc.points),workers=2)
        rays=origins[ids[nearest]]-np.asarray(treepc.points);normals=np.asarray(treepc.normals)
        normals[np.sum(normals*rays,axis=1)<0]*=-1;treepc.normals=o3d.utility.Vector3dVector(normals)
        orientation='each normal oriented toward the scanner that supplied the nearest measured point'
    else:treepc.orient_normals_consistent_tangent_plane(25)
    radii=np.array([1.5,2.25,3.25])*args.voxel
    tree=o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(treepc,o3d.utility.DoubleVector(radii))
    v=np.asarray(tree.vertices);f=np.asarray(tree.triangles)
    if not len(f):raise RuntimeError('Ball pivoting produced no supported triangles.')
    edges=np.maximum.reduce([np.linalg.norm(v[f[:,0]]-v[f[:,1]],axis=1),np.linalg.norm(v[f[:,1]]-v[f[:,2]],axis=1),np.linalg.norm(v[f[:,2]]-v[f[:,0]],axis=1)])
    # Reject large bridges and faces whose centre/midpoints lack nearby observations.
    support=cKDTree(np.asarray(treepc.points));centres=v[f].mean(axis=1);d=support.query(centres,workers=2)[0]
    keep=(edges<=5*args.voxel)&(d<=1.25*args.voxel)
    for a,b in [(0,1),(1,2),(2,0)]:keep &= support.query((v[f[:,a]]+v[f[:,b]])/2,workers=2)[0]<=1.25*args.voxel
    raw_triangle_count=len(f)
    tree.remove_triangles_by_mask(~keep);tree=clean(tree)
    supported_triangle_count=len(tree.triangles)
    labels,counts,_=tree.cluster_connected_triangles();labels=np.asarray(labels);counts=np.asarray(counts)
    tree.remove_triangles_by_mask(counts[labels]<args.min_component_triangles);tree=clean(tree)
    # Ground uses actual local height statistics and only short, supported XY edges.
    tri=Delaunay(ground[:,:2]).simplices
    edge=np.maximum.reduce([np.linalg.norm(ground[tri[:,a],:2]-ground[tri[:,b],:2],axis=1) for a,b in [(0,1),(1,2),(2,0)]])
    tri=tri[edge<.11]
    groundmesh=o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(ground),o3d.utility.Vector3iVector(tri));groundmesh.paint_uniform_color([.44,.49,.52]);groundmesh=clean(groundmesh)
    combined=clean(tree+groundmesh)
    for name,m in [('tree_observed_surface.ply',tree),('ground_surface.ply',groundmesh),('orchard_mesh.ply',combined)]:
        if not o3d.io.write_triangle_mesh(str(out/name),m,write_ascii=False):raise RuntimeError(f'Failed to write {name}')
    if not o3d.io.write_triangle_mesh(str(out/'orchard_mesh.obj'),combined,write_triangle_uvs=False):raise RuntimeError('OBJ export failed.')
    report=dict(method='Ball-pivoting open observed-surface reconstruction; separate supported 2.5-D ground triangulation',source_file=args.input.name,source_sha256=hashlib.sha256(args.input.read_bytes()).hexdigest(),coordinate_frame='Same local metric frame as the registered point clouds; X along row, Y across, Z up',voxel_m=args.voxel,ball_radii_m=radii.tolist(),maximum_tree_edge_m=5*args.voxel,support_distance_m=1.25*args.voxel,normal_orientation=orientation,input_points=len(p),raw_tree_triangles=raw_triangle_count,supported_tree_triangles=supported_triangle_count,min_component_triangles=args.min_component_triangles,tree=summary(tree),ground=summary(groundmesh),combined=summary(combined),tree_source_to_mesh=distance_check(tree,p[vegetation],rng),ground_source_to_mesh=distance_check(groundmesh,p[abs(height)<.025],rng),elapsed_seconds=time.monotonic()-started,limitations=['Observed-surface interpolation only; occluded regions and unsupported gaps remain open.','The result is not a watertight solid or validated leaf-area/volume/CFD geometry.','Mesh surface area depends on sampling, voxel size and reconstruction thresholds.','Ground and tree surfaces are separate components; stakes/trellis inside the point crop can remain.','Alignment uncertainty is inherited from the point clouds; meshing does not resolve it.'])
    (out/'mesh_quality.json').write_text(json.dumps(report,indent=2))
    (out/'README.txt').write_text('ORCHARD MESH\n\norchard_mesh.ply: coloured combined mesh, metres.\norchard_mesh.obj: portable combined surface geometry.\ntree_observed_surface.ply and ground_surface.ply: separate components.\n\nAll use the same registration as the current point clouds. This is an OPEN\nobserved-surface reconstruction, not a solid canopy envelope or a watertight\nCFD/volume model. Unsupported regions are left open. Read mesh_quality.json\nfor triangle counts, reconstruction settings and source-to-mesh distances.\n')
    render=combined.simplify_quadric_decimation(target_number_of_triangles=40000)
    vv=np.asarray(render.vertices);ff=np.asarray(render.triangles);cc=np.asarray(render.vertex_colors)
    facecolors=cc[ff].mean(axis=1) if len(cc) else np.tile([.2,.6,.3],(len(ff),1))
    normals=np.cross(vv[ff[:,1]]-vv[ff[:,0]],vv[ff[:,2]]-vv[ff[:,0]]);normals/=np.maximum(np.linalg.norm(normals,axis=1)[:,None],1e-12);light=np.array([-.2,-.7,.7]);light/=np.linalg.norm(light)
    facecolors*= (.5+.5*abs(normals@light))[:,None]
    fig=plt.figure(figsize=(14,8),facecolor='#101823');ax=fig.add_subplot(111,projection='3d');ax.set_facecolor('#101823')
    ax.add_collection3d(Poly3DCollection(vv[ff],facecolors=facecolors,edgecolors='none',linewidth=0))
    ax.set(xlim=(-.6,6.7),ylim=(1.1,2.85),zlim=(-.15,4.3),xlabel='Along row (m)',ylabel='Across (m)',zlabel='Height (m)');ax.set_box_aspect([7.3,1.75,4.45]);ax.view_init(elev=14,azim=-75)
    ax.tick_params(colors='#cfdae5');ax.xaxis.label.set_color('#cfdae5');ax.yaxis.label.set_color('#cfdae5');ax.zaxis.label.set_color('#cfdae5')
    fig.suptitle('ORCHARD · OBSERVED-SURFACE MESH',color='white',fontsize=19);fig.text(.5,.03,'Open reconstruction; unsupported gaps preserved. Display simplified only; exported meshes retain full resolution.',ha='center',color='#cfdae5',fontsize=10);fig.savefig(out/'mesh_preview.png',dpi=150,facecolor=fig.get_facecolor(),bbox_inches='tight')
    archive=out.parent/'orchard_mesh.zip'
    with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED,compresslevel=5) as z:
        names=['tree_observed_surface.ply','ground_surface.ply','orchard_mesh.ply','orchard_mesh.obj','README.txt','mesh_preview.png','mesh_quality.json']
        for name in names:z.write(out/name,name)
    print(json.dumps(report,indent=2),flush=True);print('Mesh archive:',archive,archive.stat().st_size,flush=True)
if __name__=='__main__':main()
