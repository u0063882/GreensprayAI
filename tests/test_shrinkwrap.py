"""Validate closed proxy topology and scale on an analytic observed sphere."""
import unittest
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import numpy as np
import open3d as o3d
from processing.shrinkwrap import build_wrap

class WrapGeometryTests(unittest.TestCase):
    def test_sphere_is_closed_and_has_expected_offset_scale(self):
        rng=np.random.default_rng(42);p=rng.normal(size=(2000,3));p=.1*p/np.linalg.norm(p,axis=1)[:,None]
        mesh,_=build_wrap(p,voxel=.0125,offset=.025,closing=.025,min_volume=0)
        self.assertTrue(mesh.is_watertight())
        self.assertTrue(mesh.is_vertex_manifold())
        radius=np.linalg.norm(np.asarray(mesh.vertices),axis=1)
        self.assertAlmostEqual(float(np.median(radius)),.125,delta=.02)
        self.assertGreater(mesh.get_volume(),4/3*np.pi*.1**3)

    def test_grid_budget_rejects_unbounded_input(self):
        p=np.array([[0,0,0],[100,0,0],[0,100,0],[0,0,100]])
        with self.assertRaises(ValueError):build_wrap(p,voxel=.01)

    def test_cli_needs_only_cloud_and_preserves_external_coordinate_frame(self):
        rng=np.random.default_rng(812)
        p=rng.normal(size=(2000,3));p=.1*p/np.linalg.norm(p,axis=1)[:,None]
        centre=np.array([450000.,5500000.,125.]);p+=centre
        script=Path(__file__).resolve().parents[1]/'run_meshing.py'
        with tempfile.TemporaryDirectory() as folder:
            folder=Path(folder);source=folder/'arbitrary_cloud.ply';out=folder/'surface'
            o3d.io.write_point_cloud(str(source),o3d.geometry.PointCloud(o3d.utility.Vector3dVector(p)))
            result=subprocess.run([sys.executable,str(script),'--input',str(source),'--output-dir',str(out),'--voxel','.0125','--offset','.025','--closing-radius','.025','--min-volume','0'],cwd=folder,capture_output=True,text=True,timeout=60)
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)
            report=json.loads((out/'shrinkwrap_quality.json').read_text())
            self.assertEqual(report['input_points'],len(p))
            self.assertEqual(report['meshed_input_points'],len(p))
            self.assertEqual(report['source_sha256'],hashlib.sha256(source.read_bytes()).hexdigest())
            self.assertIsNone(report['ground_source_sha256'])
            mesh=o3d.io.read_triangle_mesh(str(out/'canopy_wrap.ply'))
            self.assertTrue(mesh.is_watertight())
            radius=np.linalg.norm(np.asarray(mesh.vertices)-centre,axis=1)
            self.assertAlmostEqual(float(np.median(radius)),.125,delta=.02)
            self.assertEqual({x.name for x in folder.iterdir()},{'arbitrary_cloud.ply','surface','surface.zip'})
            self.assertFalse((out/'canopy_wrap_with_ground.ply').exists())

if __name__=='__main__':unittest.main()
