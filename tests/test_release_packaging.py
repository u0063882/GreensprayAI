"""Release reorganization must preserve the validated geometry and useful extras."""
from pathlib import Path
import json
import tempfile
import unittest
import zipfile
from tools.package_release import ASSET_NAMES,FINAL_MESH_MEMBERS,package_release,digest


def archive(path,entries):
    with zipfile.ZipFile(path,'w') as z:
        for name,value in entries.items():z.writestr(name,value)


class PackagingTests(unittest.TestCase):
    def fixture(self,source):
        source.mkdir()
        archive(source/'orchard_row_result.zip',{'orchard_row_registered.ply':b'EXACT MEASURED POINTS','scan_transforms.json':b'{"unchanged_matrix":true}\n','registration_quality.json':b'{"validated":true}'})
        archive(source/'orchard_mesh.zip',{**{name:('EXACT '+name).encode() for name in FINAL_MESH_MEMBERS},'tree_observed_surface.ply':b'OPEN OBSERVED SURFACE','mesh_quality.json':b'{"mesh":true}'})
        archive(source/'separate_registered_scans.zip',{'scan_00_registered.ply':b'COLOURED SCAN','scan_manifest.json':b'{}'})
        archive(source/'combined_by_recording.zip',{'all_scans_coloured_by_recording.ply':b'COMBINED COLOURS','scan_manifest.json':b'{}'})
        archive(source/'trunk_slice_diagnostics.zip',{'candidate_04_slices.png':b'ORIGINAL FIGURE','trunk_slice_quality.json':b'{}'})
        archive(source/'processing_code_and_notes.zip',{'old_source.py':b'original data source'})
        (source/'anchored_trunk_sections.png').write_bytes(b'BEFORE AFTER FIGURE')
        (source/'manual_notes.txt').write_text('Useful extra attachment')

    def test_five_downloads_preserve_geometry_transforms_contributions_and_qc(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);source=root/'old';self.fixture(source);code=root/'code.zip';archive(code,{'new_source.py':b'packaging code'})
            out=root/'new';paths=package_release(source,out,code,data_commit='geometry-sha',packaging_commit='packaging-sha')
            self.assertEqual([p.name for p in paths],list(ASSET_NAMES))
            self.assertEqual((out/ASSET_NAMES[0]).read_bytes(),b'EXACT MEASURED POINTS')
            self.assertEqual((out/ASSET_NAMES[2]).read_bytes(),b'{"unchanged_matrix":true}\n')
            with zipfile.ZipFile(out/ASSET_NAMES[3]) as qc:
                self.assertEqual(qc.read('QC/trunk_slices/candidate_04_slices.png'),b'ORIGINAL FIGURE')
                self.assertEqual(qc.read('QC/additional_assets/manual_notes.txt'),b'Useful extra attachment')
                self.assertEqual(qc.read('QC/meshing/tree_observed_surface.ply'),b'OPEN OBSERVED SURFACE')
                self.assertIn('QC/index.html',qc.namelist())
                self.assertEqual(json.loads(qc.read('QC/provenance/data_origin.json'))['data_source_commit'],'geometry-sha')
            with zipfile.ZipFile(out/ASSET_NAMES[4]) as scans:
                self.assertEqual(scans.read('separate_scans/scan_00_registered.ply'),b'COLOURED SCAN')
                self.assertEqual(scans.read('combined/all_scans_coloured_by_recording.ply'),b'COMBINED COLOURS')

    def test_repackaging_new_layout_keeps_primary_and_contribution_archives_identical(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);source=root/'old';self.fixture(source);code=root/'code.zip';archive(code,{'source.py':b'first'})
            first=root/'first';package_release(source,first,code,data_commit='original-data',packaging_commit='first-package')
            archive(code,{'source.py':b'updated docs and packaging'})
            second=root/'second';package_release(first,second,code,data_commit='first-package',packaging_commit='second-package')
            for name in [*ASSET_NAMES[:3],ASSET_NAMES[4]]:self.assertEqual(digest(first/name),digest(second/name))
            with zipfile.ZipFile(second/ASSET_NAMES[3]) as qc:
                origin=json.loads(qc.read('QC/provenance/data_origin.json'))
                self.assertEqual(origin['data_source_commit'],'original-data')
                self.assertEqual(origin['packaging_commit'],'second-package')
                self.assertEqual(qc.read('QC/provenance/current_code_and_notes.zip'),code.read_bytes())

    def test_malicious_archive_path_is_rejected_before_publication(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);source=root/'old';self.fixture(source);code=root/'code.zip';archive(code,{})
            with zipfile.ZipFile(source/'orchard_mesh.zip','a') as z:z.writestr('../escape.txt','bad')
            with self.assertRaises(ValueError):package_release(source,root/'new',code,data_commit='a',packaging_commit='b')
            self.assertFalse((root/'escape.txt').exists())

if __name__=='__main__':unittest.main()
