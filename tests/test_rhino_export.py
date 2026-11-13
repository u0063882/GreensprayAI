"""CAD compatibility exports retain geometry, colour, signal and source grouping."""
import hashlib,json,tempfile,unittest
from pathlib import Path
import numpy as np
from captures.leafwood import write_records
from tools.export_rhino import export_file,read_vertices,write_ascii


class RhinoExportTests(unittest.TestCase):
    def test_native_units_scan_groups_signal_and_ascii_preserve_the_source(self):
        import rhino3dm as rh
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);d=np.zeros(97,dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('red','u1'),('green','u1'),('blue','u1'),('intensity','<u2'),('near_ir','<u2'),('scan_id','<u4'),('signal','<u2'),('pixel_index','<u4')]);rng=np.random.default_rng(251)
            for key in ['x','y','z']:d[key]=rng.normal(size=len(d))
            for key in ['red','green','blue']:d[key]=rng.integers(0,256,len(d))
            d['scan_id']=np.arange(len(d))%3;d['signal']=rng.integers(0,65536,len(d));d['near_ir']=65535;d['intensity']=d['signal'];d['pixel_index']=np.arange(len(d));source=root/'source.ply';write_records(source,d);before=source.read_bytes();report=export_file(source,root/'rhino',{0:'east',1:'west',2:'other'})
            self.assertEqual(source.read_bytes(),before);self.assertEqual(report['source_points'],len(d));self.assertTrue(report['ascii']['coordinate_and_RGB_roundtrip_exact']);self.assertTrue(report['native']['point_coordinates_and_RGB_roundtrip_exact']);self.assertTrue(report['native']['raw_intensity_in_native_point_values'])
            model=rh.File3dm.Read(str(root/'rhino/final_registered.3dm'));self.assertEqual(model.Settings.ModelUnitSystem,rh.UnitSystem.Millimeters);self.assertEqual(len(model.Objects),3)
            for obj in model.Objects:
                sid=int(obj.Attributes.GetUserString('scan_id'));self.assertEqual(len(obj.Geometry),int(np.sum(d['scan_id']==sid)));self.assertFalse(obj.Geometry.ContainsNormals);self.assertFalse(obj.Geometry.ContainsHiddenFlags);self.assertEqual(model.Layers[obj.Attributes.LayerIndex].Name,f'scan_{sid:03d}')
            raw=read_vertices(source)
            for key in d.dtype.names:np.testing.assert_array_equal(raw[key],d[key])

    def test_rejects_truncated_binary_record_layout(self):
        with tempfile.TemporaryDirectory() as temporary:
            p=Path(temporary)/'broken.ply';d=np.zeros(5,dtype=[('x','f4'),('y','f4'),('z','f4'),('red','u1'),('green','u1'),('blue','u1')]);write_records(p,d);p.write_bytes(p.read_bytes()[:-1])
            with self.assertRaisesRegex(ValueError,'payload size'):read_vertices(p)

    def test_ascii_roundtrip_handles_float32_precision_without_custom_fields(self):
        with tempfile.TemporaryDirectory() as temporary:
            p=Path(temporary)/'plain.ply';d=np.zeros(8,dtype=[('x','f4'),('y','f4'),('z','f4'),('red','u1'),('green','u1'),('blue','u1')]);d['x']=[0.,-0.,1.23456789,-1.23456789,1e-30,1e8,np.nextafter(np.float32(1),np.float32(2)),np.pi];d['y']=d['x'];d['green']=255
            result=write_ascii(p,d);self.assertTrue(result['coordinate_and_RGB_roundtrip_exact']);header=p.read_text().split('end_header')[0];self.assertIn('format ascii 1.0',header);self.assertNotIn('scan_id',header)

class RhinoPackagingTests(unittest.TestCase):
    def test_repackaging_preserves_enriched_source_bytes_for_all_datasets(self):
        import zipfile
        from tools.publish_rhino_exports import package_rhino
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);d=np.zeros(12,dtype=[('x','f4'),('y','f4'),('z','f4'),('red','u1'),('green','u1'),('blue','u1'),('scan_id','u4'),('signal','u2'),('near_ir','u2')]);d['x']=np.arange(len(d))*.12;d['z']=.3;d['green']=155;d['scan_id']=np.arange(len(d))%2;d['signal']=np.arange(len(d))*900;d['near_ir']=9876;p=root/'sample.ply';write_records(p,d);original=p.read_bytes();source=root/'source.zip'
            with zipfile.ZipFile(source,'w') as z:
                for name in ['chandelier','spindle1','spindle2','vhedge']:z.writestr(name+'/final_registered.ply',original)
            output=root/'output.zip';report=package_rhino(source,output,root/'stage','test-commit');self.assertTrue(report['original_enriched_files_byte_identical'])
            with zipfile.ZipFile(output) as z:
                for name in report['datasets']:
                    self.assertEqual(z.read(name+'/final_registered.ply'),original);self.assertIn(name+'/rhino/final_registered.3dm',z.namelist());self.assertIn(name+'/rhino/final_registered_xyzrgb_ascii.ply',z.namelist())

    def test_zip_path_traversal_is_rejected_before_extraction(self):
        import zipfile
        from tools.publish_rhino_exports import checked_extract
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);source=root/'unsafe.zip'
            with zipfile.ZipFile(source,'w') as z:z.writestr('../outside','bad')
            with self.assertRaisesRegex(ValueError,'Unsafe ZIP'):checked_extract(source,root/'stage')


if __name__=='__main__':unittest.main()
