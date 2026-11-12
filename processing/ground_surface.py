"""Observed ground mesh from a point cloud; independent of registration inputs."""
import numpy as np
import open3d as o3d
from scipy.spatial import Delaunay


def observed_ground(points,cell_m=.04,maximum_edge_m=.11):
    points=np.asarray(points);near=points[points[:,2]<.16];cells=np.floor(near[:,:2]/cell_m).astype(np.int64)
    _,inv=np.unique(cells,axis=0,return_inverse=True);order=np.argsort(inv);groups=np.split(order,np.where(np.diff(inv[order]))[0]+1);terrain=[]
    for ix in groups:
        if len(ix)<4:continue
        q=near[ix];low=np.quantile(q[:,2],.30);q=q[abs(q[:,2]-low)<.025]
        if len(q)>=3:terrain.append(np.median(q,axis=0))
    ground=np.asarray(terrain)
    if len(ground)<100:raise ValueError('Insufficient observed ground support')
    triangles=Delaunay(ground[:,:2]).simplices
    edge=np.maximum.reduce([np.linalg.norm(ground[triangles[:,a],:2]-ground[triangles[:,b],:2],axis=1) for a,b in [(0,1),(1,2),(2,0)]])
    triangles=triangles[edge<maximum_edge_m]
    mesh=o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(ground),o3d.utility.Vector3iVector(triangles));mesh.paint_uniform_color([.44,.49,.52]);mesh.remove_duplicated_vertices();mesh.remove_degenerate_triangles();mesh.remove_unreferenced_vertices();mesh.compute_vertex_normals()
    return mesh
