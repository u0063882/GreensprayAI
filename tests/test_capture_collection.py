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

if __name__=='__main__':unittest.main()
