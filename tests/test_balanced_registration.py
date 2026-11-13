"""Rigid frame choices must not flatten terrain, shrink trees or lose sources."""
import json,tempfile,unittest
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from captures.balanced_frame import estimate_balanced_frame
from captures.joint_registration import apply
from captures.leafwood import write_records
from processing.export_anchored import read_records,xyz
from tools.refine_balanced_collection import transform_records,export_dataset,restore_balanced_frames
from tools.orient_collection import _snapshot


class BalancedFrameTests(unittest.TestCase):
    def test_sloping_soil_and_asymmetric_tree_shape_are_preserved_by_one_rigid_frame(self):
        rng=np.random.default_rng(141);x=rng.uniform(0,5,22000);y=rng.uniform(-.9,.9,len(x));soil=np.c_[x,y,.02*x+.04*y+rng.normal(0,.002,len(x))]
        x=rng.uniform(0,5,18000);z=rng.uniform(.4,3.1,len(x));arm=rng.choice([-1.,1.],len(x),p=[.65,.35]);y=.18*z+np.where(arm>0,.30*z,-.45*z)+rng.normal(0,.02,len(x));p=np.r_[soil,np.c_[x,y,z]];before=p.copy();seeds=[(i,np.array([.5+i*1.5,0.])) for i in range(3)]
        T,r=estimate_balanced_frame(p,seeds);q=apply(p,T)
        self.assertLess(r['ground_tilt_after_deg'],3.)
        self.assertGreater(r['observed_centre_tilt_after_deg'],.1)
        np.testing.assert_allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-12)
        ix=rng.integers(0,len(p),300);jx=rng.integers(0,len(p),300)
        np.testing.assert_allclose(np.linalg.norm(q[ix]-q[jx],axis=1),np.linalg.norm(p[ix]-p[jx],axis=1),atol=1e-12)
        np.testing.assert_array_equal(p,before)
        # Soil remains sloped/rough in its own geometry; no per-point Z reset.
        self.assertGreater(np.ptp(q[:len(soil),2]),.02)
        self.assertEqual(r['settings']['ground_weight'],4.)

    def test_rejects_scale_and_mirror_transforms(self):
        d=np.zeros(20,dtype=[('x','f4'),('y','f4'),('z','f4'),('signal','u4')])
        for v in [1.1,-1.]:
            T=np.eye(4);T[0,0]=v
            with self.assertRaises((AssertionError,ValueError)):transform_records(d,T)


class BalancedExportTests(unittest.TestCase):
    def test_composed_sensor_poses_attributes_and_repeat_after_restoration(self):
        with tempfile.TemporaryDirectory() as temporary:
            stage=Path(temporary);name='test';folder=stage/'contributions'/name/'separate_scans';folder.mkdir(parents=True);(stage/'pointclouds'/name).mkdir(parents=True);(stage/'transformations'/name).mkdir(parents=True);(stage/'QC'/name/'provenance').mkdir(parents=True);(stage/'contributions'/name/'combined').mkdir(parents=True)
            dtype=[('x','f4'),('y','f4'),('z','f4'),('red','u1'),('green','u1'),('blue','u1'),('signal','u4'),('near_ir','u4'),('scan_id','u2'),('frame_id','u4'),('pixel_index','u4')];rng=np.random.default_rng(142);parts=[];entries=[];poses=[]
            for i in range(2):
                d=np.zeros(80,dtype=dtype);p=rng.normal(size=(80,3))*.2+[i,0,1]
                for j,k in enumerate(['x','y','z']):d[k]=p[:,j]
                d['red']=137;d['green']=74;d['blue']=32;d['signal']=rng.integers(0,20000,len(d));d['near_ir']=rng.integers(0,65535,len(d));d['scan_id']=i;d['pixel_index']=np.arange(len(d));parts.append(d);filename=f'scan{i}.ply';write_records(folder/filename,d);entry=dict(scan=f'side{i}_scan',scan_id=i,filename=filename,sensor_to_merged=np.eye(4).tolist(),gravity_levelled_to_merged=np.eye(4).tolist());entries.append(entry);poses.append(dict(entry))
            merged=np.concatenate(parts);primary=stage/'pointclouds'/name/'final_registered.ply';write_records(primary,merged);write_records(stage/'contributions'/name/'combined/all_scans_coloured_by_recording.ply',merged)
            (folder/'scan_manifest.json').write_text(json.dumps(dict(scans=entries)));(stage/'transformations'/name/'transformations.json').write_text(json.dumps(dict(scans=poses,registration_frame_to_final=np.eye(4).tolist())))
            archive=stage/'QC'/name/'provenance/balanced_registration_baseline.zip';_snapshot(stage,name,archive)
            F=np.eye(4);F[:3,:3]=Rotation.from_euler('x',4,degrees=True).as_matrix();F[:3,3]=[.2,-.1,.05];C=np.tile(np.eye(4),(2,1,1));C[1,:3,:3]=Rotation.from_euler('z',.7,degrees=True).as_matrix();C[1,:3,3]=[.01,-.015,0.];total=F@C
            report=dict(relative_refinement_applied=True,baseline_sha256='test',frame=dict(output_frame=F.tolist(),policy='test ground-first frame'))
            for iteration in range(2):
                export_dataset(stage,name,total,report);_,saved=read_records(primary)
                for i in range(2):np.testing.assert_allclose(xyz(saved[saved['scan_id']==i]),apply(xyz(merged[merged['scan_id']==i]),total[i]),atol=3e-6,rtol=0)
                for k in saved.dtype.names:
                    if k not in ['x','y','z']:np.testing.assert_array_equal(saved[k],merged[k])
                if iteration==0:first=xyz(saved).copy()
                else:np.testing.assert_array_equal(first,xyz(saved))
                restore_balanced_frames(stage);np.testing.assert_array_equal(xyz(read_records(primary)[1]),xyz(merged))

if __name__=='__main__':unittest.main()
