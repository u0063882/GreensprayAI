#!/usr/bin/env python3
"""Check exported geometry and its provenance before publishing it."""
from pathlib import Path
import argparse,hashlib,json
import numpy as np
import open3d as o3d

root=Path(__file__).resolve().parent.parent
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--source',type=Path,default=root/'output/orchard_row_registered.ply')
parser.add_argument('--mesh-dir',type=Path,default=root/'output/mesh')
args=parser.parse_args();source=args.source;folder=args.mesh_dir
report=json.loads((folder/'mesh_quality.json').read_text())
assert hashlib.sha256(source.read_bytes()).hexdigest()==report['source_sha256']
p=np.asarray(o3d.io.read_point_cloud(str(source)).points)
for name,key in [('tree_observed_surface.ply','tree'),('ground_surface.ply','ground'),('orchard_mesh.ply','combined')]:
    mesh=o3d.io.read_triangle_mesh(str(folder/name));v=np.asarray(mesh.vertices);f=np.asarray(mesh.triangles)
    assert len(v)==report[key]['vertices'] and len(f)==report[key]['triangles'] and len(f)>0
    assert np.isfinite(v).all() and f.min()>=0 and f.max()<len(v)
    assert (v.min(0)>=p.min(0)-.1).all() and (v.max(0)<=p.max(0)+.1).all()
    if key=='tree':
        maximum=max(np.linalg.norm(v[f[:,a]]-v[f[:,b]],axis=1).max() for a,b in [(0,1),(1,2),(2,0)])
        assert maximum<=report['maximum_tree_edge_m']+1e-6
    assert not report[key]['watertight'] or mesh.is_watertight()
assert report['tree_source_to_mesh']['fraction_within_3cm']>.1
print('Mesh provenance, bounds, face indices, edge limits and reported counts verified.')

wrap_report=json.loads((folder/'shrinkwrap_quality.json').read_text())
wrap=o3d.io.read_triangle_mesh(str(folder/'canopy_wrap.ply'))
assert wrap_report['source_sha256']==report['source_sha256']
assert len(wrap.vertices)==wrap_report['vertices'] and len(wrap.triangles)==wrap_report['triangles']
assert np.isfinite(np.asarray(wrap.vertices)).all()
assert wrap.is_edge_manifold(allow_boundary_edges=False) and wrap.is_vertex_manifold() and wrap.is_orientable()
assert wrap_report['watertight'] and wrap_report['envelope_proxy_volume_m3']>0
assert (folder/'canopy_wrap.stl').stat().st_size==84+50*len(wrap.triangles)
print('Closed wrap topology, indexed PLY counts, STL face count and source provenance verified.')
