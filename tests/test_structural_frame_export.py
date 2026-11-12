import json,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
from scipy.spatial.transform import Rotation
from captures.leafwood import write_records
from processing.export_anchored import read_records,xyz
from tools.orient_collection import orient_collection,restore_working_frames

class StructuralFrameExportTests(unittest.TestCase):
    def test_shared_frame_rebuild_and_physics_restore_preserve_attributes_and_relative_poses(self):
        dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('red','u1'),('green','u1'),('blue','u1'),('signal','<u2'),('near_ir','<u2'),('wood_score','<f4'),('scan_id','<u4'),('frame_id','<u4'),('pixel_index','<u4')]
        arrays=[];entries=[];R=Rotation.from_euler('x',-8,degrees=True).as_matrix();T=np.eye(4);T[:3,:3]=R
        for i,side in enumerate(['sideA','sideB']):
            d=np.zeros(4,dtype=dtype);d['x']=np.arange(4)*.2;d['y']=i*.1;d['z']=[0,.5,1.,2.];d['red']=70;d['green']=35;d['blue']=15;d['signal']=[1,500,1000,65000];d['near_ir']=77;d['wood_score']=.8;d['scan_id']=i;d['frame_id']=12;d['pixel_index']=np.arange(4);arrays.append(d)
            pose=np.eye(4);pose[0,3]=i;entries.append(dict(scan_id=i,scan=side+'_000001',filename=f'scan_{i}.ply',sensor_to_merged=pose.tolist(),gravity_levelled_to_merged=pose.tolist()))
        report=dict(supported=True,applied_rotation_deg=-8.,reference_policy='Structural, not surveyed up')
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);folder=root/'contributions/test/separate_scans';folder.mkdir(parents=True)
            for e,d in zip(entries,arrays):write_records(folder/e['filename'],d)
            (folder/'scan_manifest.json').write_text(json.dumps(dict(scans=entries)))
            primary=root/'pointclouds/test/final_registered.ply';write_records(primary,np.concatenate(arrays));(primary.parent/'leafwood_model.json').write_text('{}')
            write_records(root/'contributions/test/combined/all_scans_coloured_by_recording.ply',np.concatenate(arrays))
            meta=root/'transformations/test/transformations.json';meta.parent.mkdir(parents=True);meta.write_text(json.dumps(dict(scans=entries)))
            qc=root/'QC/test';qc.mkdir(parents=True);(qc/'index.html').write_text('<h1>Test</h1>')
            with patch('tools.orient_collection.estimate_structural_frame',return_value=(T,report.copy())) as fit:
                orient_collection(root,apply_rotation=True);_,first=read_records(primary);orient_collection(root,apply_rotation=True);self.assertEqual(fit.call_count,1)
                old=np.concatenate(arrays);np.testing.assert_allclose(xyz(first),xyz(old)@R.T,rtol=0,atol=1e-7)
                for k in old.dtype.names:
                    if k not in ['x','y','z']:np.testing.assert_array_equal(first[k],old[k])
                restore_working_frames(root);_,working=read_records(primary);np.testing.assert_array_equal(xyz(working),xyz(old));self.assertNotIn('structural_up_frame',json.loads(meta.read_text()))
                orient_collection(root,apply_rotation=True);_,again=read_records(primary);np.testing.assert_array_equal(xyz(again),xyz(first))
                output=json.loads(meta.read_text());np.testing.assert_allclose(output['scans'][1]['sensor_to_merged'],T@np.array(entries[1]['sensor_to_merged']),atol=1e-12)
                orient_collection(root);_,physical=read_records(primary);np.testing.assert_array_equal(xyz(physical),xyz(old));self.assertNotIn('structural_up_frame',json.loads(meta.read_text()))
                policy=json.loads((qc/'structural_frame/report.json').read_text());self.assertFalse(policy['automatic_structural_rotation_enabled']);self.assertEqual(policy['applied_rotation_deg'],0.)

if __name__=='__main__':unittest.main()
