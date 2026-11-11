"""Final exports must preserve measurements and avoid applying corrections twice."""
from pathlib import Path
from unittest.mock import patch
import json,tempfile,unittest
import numpy as np
from captures.leafwood import write_records
from processing.export_anchored import read_records,xyz
from tools.refine_trunk_collection import refine_collection,_load_baseline


class RefinementExportTests(unittest.TestCase):
    def test_export_cache_and_rebuild_keep_original_baseline_and_sensor_attributes(self):
        dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('red','u1'),('green','u1'),('blue','u1'),('reflectivity','u1'),('signal','<u2'),('intensity','<u2'),('near_ir','<u2'),('scan_id','<u4'),('frame_id','<u4'),('pixel_index','<u4')]
        arrays=[];entries=[]
        for i,side in enumerate(['sideA','sideB']):
            rec=np.zeros(4,dtype=dtype);rec['x']=np.arange(4)*.1;rec['y']=i*.1;rec['z']=[0,.2,.3,.4];rec['red']=77;rec['green']=88;rec['blue']=99;rec['reflectivity']=[1,2,3,255];rec['signal']=[100,500,10000,65535];rec['intensity']=rec['signal'];rec['near_ir']=[3,50,100,65535];rec['scan_id']=i;rec['frame_id']=41;rec['pixel_index']=np.arange(4)
            arrays.append(rec);entries.append(dict(scan_id=i,scan=side+'_000001',filename=f'scan_{i:02d}.ply',point_count=4,sensor_to_merged=np.eye(4).tolist(),gravity_levelled_to_merged=np.eye(4).tolist()))
        corrections=np.tile(np.eye(4),(2,1,1));corrections[1,1,3]=-.08
        report=dict(validation_passed=True,rejection_reasons=[],validation={},trunk_models={},variant='Test rigid refinement',object_observations=[])
        config={'datasets':[{'id':'spindle1'}]}
        with tempfile.TemporaryDirectory() as temporary:
            stage=Path(temporary);folder=stage/'contributions/spindle1/separate_scans';folder.mkdir(parents=True)
            for entry,rec in zip(entries,arrays):write_records(folder/entry['filename'],rec)
            (folder/'scan_manifest.json').write_text(json.dumps({'scans':entries}));primary=stage/'pointclouds/spindle1/final_registered.ply';write_records(primary,np.concatenate(arrays));(primary.parent/'leafwood_model.json').write_text('{}')
            combined=stage/'contributions/spindle1/combined/all_scans_coloured_by_recording.ply';write_records(combined,np.concatenate(arrays))
            matrices=stage/'transformations/spindle1/transformations.json';matrices.parent.mkdir(parents=True);matrices.write_text(json.dumps({'registration_frame_to_final':np.eye(4).tolist(),'scans':entries}))
            gallery=stage/'QC/spindle1/index.html';gallery.parent.mkdir(parents=True);gallery.write_text('<h1>QC</h1>')
            with patch('tools.refine_trunk_collection.refine_trunks',return_value=(corrections,report.copy())) as fit,patch('tools.refine_trunk_collection.plot_slices'),patch('tools.refine_trunk_collection.plot_summary'):
                refine_collection(stage,config);_,first=read_records(primary);refine_collection(stage,config);self.assertEqual(fit.call_count,1)
                original=np.concatenate(arrays)
                for name in original.dtype.names:
                    if name not in ['x','y']:np.testing.assert_array_equal(first[name],original[name])
                np.testing.assert_allclose(first['y'][4:],.02,atol=1e-8)
                archive=stage/'QC/spindle1/provenance/trunk_refinement_baseline.zip';clouds,restored,baseline_matrices,manifest,meta=_load_baseline(stage,'spindle1',archive)
                np.testing.assert_array_equal(xyz(restored),xyz(original));np.testing.assert_array_equal(clouds[1],xyz(arrays[1]))
                # A later classifier change triggers rebuilding from historical
                # XYZ, while current sensor/colour values survive unchanged.
                (primary.parent/'leafwood_model.json').write_text('{"updated":true}')
                for entry in entries:
                    path=folder/entry['filename'];_,rec=read_records(path);rec['red']=42;write_records(path,rec)
                _,rec=read_records(primary);rec['red']=42;write_records(primary,rec)
                refine_collection(stage,config);_,rebuilt=read_records(primary)
                np.testing.assert_array_equal(rebuilt['red'],42);np.testing.assert_array_equal(xyz(rebuilt),xyz(first));self.assertEqual(fit.call_count,2)
                poses=json.loads(matrices.read_text());np.testing.assert_array_equal(poses['scans'][1]['sensor_to_merged'],corrections[1])


if __name__=='__main__':unittest.main()
