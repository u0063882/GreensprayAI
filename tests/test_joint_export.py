"""Joint exports preserve raw fields and compose poses exactly once on rebuild."""
import json,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
from scipy.spatial.transform import Rotation
from captures.leafwood import write_records
from processing.export_anchored import read_records,xyz
from tools.refine_joint_collection import refine_joint_collection


class JointExportTests(unittest.TestCase):
    def test_rigid_export_and_classifier_rebuild_use_the_archived_baseline(self):
        dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('red','u1'),('green','u1'),('blue','u1'),('reflectivity','u1'),('signal','<u2'),('intensity','<u2'),('near_ir','<u2'),('wood_score','<f4'),('scan_id','<u4'),('frame_id','<u4'),('pixel_index','<u4')]
        arrays=[];entries=[]
        for i,side in enumerate(['sideA','sideB']):
            rec=np.zeros(4,dtype=dtype);rec['x']=np.arange(4)*.1;rec['y']=i*.1;rec['z']=[0,.2,.3,.4];rec['red']=77;rec['green']=88;rec['blue']=99;rec['reflectivity']=[1,2,3,255];rec['signal']=[100,500,10000,65535];rec['intensity']=rec['signal'];rec['near_ir']=[3,50,100,65535];rec['wood_score']=.8;rec['scan_id']=i;rec['frame_id']=41;rec['pixel_index']=np.arange(4)
            arrays.append(rec);entries.append(dict(scan_id=i,scan=side+'_000001',filename=f'scan_{i:02d}.ply',point_count=4,sensor_to_merged=np.eye(4).tolist(),gravity_levelled_to_merged=np.eye(4).tolist()))
        C=np.tile(np.eye(4),(2,1,1));C[1,:3,:3]=Rotation.from_euler('x',1,degrees=True).as_matrix();C[1,:3,3]=[.01,-.04,.005]
        report=dict(method='Test joint rigid correction',validation_passed=True,rejection_reasons=[],recoveries=[],retained_landmarks=[],excluded_ambiguous_training_landmarks=[],anchor_checks=[])
        preset=dict(datasets=['vhedge'],promote_datasets=['vhedge'],settings={})
        with tempfile.TemporaryDirectory() as temporary:
            stage=Path(temporary);folder=stage/'contributions/vhedge/separate_scans';folder.mkdir(parents=True)
            for entry,rec in zip(entries,arrays):write_records(folder/entry['filename'],rec)
            (folder/'scan_manifest.json').write_text(json.dumps(dict(scans=entries)))
            primary=stage/'pointclouds/vhedge/final_registered.ply';write_records(primary,np.concatenate(arrays));(primary.parent/'leafwood_model.json').write_text('{}')
            combined=stage/'contributions/vhedge/combined/all_scans_coloured_by_recording.ply';write_records(combined,np.concatenate(arrays))
            matrices=stage/'transformations/vhedge/transformations.json';matrices.parent.mkdir(parents=True);matrices.write_text(json.dumps(dict(registration_frame_to_final=np.eye(4).tolist(),scans=entries)))
            qc=stage/'QC/vhedge';qc.mkdir(parents=True);(qc/'index.html').write_text('<h1>QC</h1>');(qc/'registration_quality.json').write_text('{}')
            with patch('tools.refine_joint_collection.refine_joint',return_value=(C,report.copy())) as fit,patch('tools.refine_joint_collection.plot_joint'):
                refine_joint_collection(stage,preset);_,first=read_records(primary);refine_joint_collection(stage,preset);self.assertEqual(fit.call_count,1)
                original=np.concatenate(arrays)
                for field in original.dtype.names:
                    if field not in ['x','y','z']:np.testing.assert_array_equal(first[field],original[field])
                expected=xyz(arrays[1])@C[1,:3,:3].T+C[1,:3,3];np.testing.assert_allclose(xyz(first)[4:],expected,rtol=0,atol=1e-7)
                self.assertFalse(np.array_equal(first['z'],original['z']))
                # Rebuild for a new classifier using the same original XYZ.
                (primary.parent/'leafwood_model.json').write_text('{"updated":true}')
                for path in [primary]+[folder/e['filename'] for e in entries]:
                    _,rec=read_records(path);rec['red']=42;write_records(path,rec)
                refine_joint_collection(stage,preset);_,rebuilt=read_records(primary)
                self.assertEqual(fit.call_count,2);np.testing.assert_array_equal(xyz(rebuilt),xyz(first));np.testing.assert_array_equal(rebuilt['red'],42)
                pose=json.loads(matrices.read_text());np.testing.assert_array_equal(pose['scans'][1]['sensor_to_merged'],C[1])
                self.assertTrue((qc/'provenance/joint_registration_baseline.zip').exists())

if __name__=='__main__':unittest.main()
