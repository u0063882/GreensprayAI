"""Dataset-folder migration must preserve existing primary and contribution bytes."""
from pathlib import Path
import tempfile,unittest,zipfile
from tools.capture_collection import import_current,frozen_hashes,assert_frozen,package_collection,NAMES

def archive(path,files):
    with zipfile.ZipFile(path,'w') as z:
        for key,value in files.items():z.writestr(key,value)

class CollectionPackagingTests(unittest.TestCase):
    def test_single_dataset_migration_and_future_repack_keep_existing_data(self):
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);source=root/'old';source.mkdir()
            (source/'01_final_aligned_pointcloud.ply').write_bytes(b'MEASURED OLD POINTS');(source/'03_transformations.json').write_bytes(b'{"original":true}')
            archive(source/'02_final_mesh.zip',{'final_mesh/canopy_wrap.ply':b'OLD MESH','final_mesh/README.txt':b'old mesh notes'})
            archive(source/'90_QC.zip',{'QC/index.html':b'old gallery','QC/provenance/seed.zip':b'OLD SEED'})
            archive(source/'91_registered_contributions.zip',{'separate_scans/scan.ply':b'OLD CONTRIBUTION','combined/all.ply':b'OLD COMBINED'})
            stage=root/'stage';self.assertEqual(import_current(source,stage),'single-dataset');before=frozen_hashes(stage)
            code=root/'code.zip';archive(code,{'source.py':b'current source'})
            first=root/'first';paths=package_collection(stage,first,code);self.assertEqual([p.name for p in paths],list(NAMES));assert_frozen(stage,before)
            with zipfile.ZipFile(first/NAMES[0]) as z:self.assertEqual(z.read('chandelier/final_registered.ply'),b'MEASURED OLD POINTS')
            with zipfile.ZipFile(first/NAMES[3]) as z:self.assertEqual(z.read('QC/chandelier/provenance/seed.zip'),b'OLD SEED')
            second=root/'second';self.assertEqual(import_current(first,second),'collection');assert_frozen(second,before)
            (second/'pointclouds/chandelier/final_registered.ply').write_bytes(b'CHANGED')
            with self.assertRaises(RuntimeError):assert_frozen(second,before)


class FrozenCaptureTests(unittest.TestCase):
    def test_existing_named_dataset_is_verified_and_not_rebuilt_after_code_changes(self):
        import json
        from unittest.mock import patch
        from tools.capture_collection import add_capture,digest
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);stage=root/'stage';target=stage/'pointclouds/spindle1/final_registered.ply';target.parent.mkdir(parents=True);target.write_bytes(b'EXACT EXISTING POINTS')
            qc=stage/'QC/spindle1/provenance';qc.mkdir(parents=True)
            old=dict(dataset='spindle1',pipeline={'old':'algorithm'},points=123,output_hashes={'pointclouds/spindle1/final_registered.ply':digest(target)})
            (qc/'build.json').write_text(json.dumps(old));global_qc=stage/'QC/provenance';global_qc.mkdir()
            (global_qc/'collection.json').write_text(json.dumps({'packaging_commit':'original-data-commit'}));(global_qc/'current_code_and_notes.zip').write_bytes(b'ORIGINAL CODE SNAPSHOT')
            dataset=dict(id='spindle1',kind='frozen',config='presets/captures/spindle1.json',inputs=[])
            with patch('tools.capture_collection.pipeline_identity',return_value={'new':'algorithm'}),patch('tools.capture_collection.subprocess.run',side_effect=AssertionError('Frozen data must not be recomputed')):
                result=add_capture(dataset,stage,root/'work','owner/repo',mode='rebuild')
            self.assertEqual(result['points'],123);self.assertEqual(target.read_bytes(),b'EXACT EXISTING POINTS');self.assertEqual(result['data_source_commit'],'original-data-commit')
            self.assertEqual((qc/'data_pipeline_code.zip').read_bytes(),b'ORIGINAL CODE SNAPSHOT')

if __name__=='__main__':unittest.main()
