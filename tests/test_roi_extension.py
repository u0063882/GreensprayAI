"""ROI widening adds measured returns and preserves the complete original prefix."""
import unittest
import numpy as np
from tools.extend_collection_roi import extended_scan,merge_extension
from processing.export_anchored import xyz


class MeasuredRoiTests(unittest.TestCase):
    def test_only_original_outside_crop_returns_are_added_with_raw_fields(self):
        dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('red','u1'),('green','u1'),('blue','u1'),('leaf_red','u1'),('leaf_green','u1'),('leaf_blue','u1'),('reflectivity','u1'),('classification','u1'),('intensity','<u2'),('signal','<u2'),('near_ir','<u2'),('scan_id','<u4'),('leafwood_class','u1'),('wood_score','<f4'),('leafwood_confidence','<f4'),('frame_id','<u4'),('pixel_index','<u4')]
        original=np.zeros(2,dtype=dtype);original['x']=[.5,1.5];original['y']=2;original['z']=.3;original['red']=12;original['green']=90;original['blue']=4;original['signal']=[100,200];original['intensity']=original['signal'];original['near_ir']=[1,2];original['frame_id']=1;original['pixel_index']=[0,1]
        x,y=np.meshgrid(np.arange(3)*.015,np.arange(3)*.015);extra=np.concatenate([np.c_[-.8+x.ravel(),2+y.ravel(),np.full(9,.3)],np.c_[2.8+x.ravel(),2+y.ravel(),np.full(9,.3)]])
        p=np.concatenate([xyz(original),extra]);n=len(p);decoded=dict(xyz=p,reflectivity=np.arange(n,dtype='u1'),signal=np.arange(n,dtype='u2')+100,near_ir=np.arange(n,dtype='u2')+20,frame_id=np.ones(n,dtype='u4'),pixel_index=np.arange(n,dtype='u4'))
        model=dict(log_centres=[.1,.8],log_threshold=.45);roi=dict(original_along_m=[0,2],along_m=[-1,3],across_m=[1.5,2.5],height_m=[-.1,2])
        expanded,added,proof=extended_scan(original,decoded,dict(gravity_levelled_to_merged=np.eye(4).tolist()),0,[31,119,180],model,roi)
        np.testing.assert_array_equal(expanded[:2],original);self.assertEqual(proof['added'],18);np.testing.assert_allclose(xyz(added),extra,atol=1e-7)
        for key in ['signal','near_ir','reflectivity','frame_id','pixel_index']:np.testing.assert_array_equal(added[key],decoded[key][2:])
        np.testing.assert_array_equal(added['intensity'],decoded['signal'][2:])
        merged=merge_extension(original,[added]);np.testing.assert_array_equal(merged[:2],original);self.assertGreater(len(merged),2)
        np.testing.assert_array_equal(merged['red'][2:],merged['leaf_red'][2:])
        semantic=original.copy()
        for key in ['red','green','blue']:semantic['leaf_'+key]=semantic[key]
        _,added,_=extended_scan(semantic,decoded,dict(gravity_levelled_to_merged=np.eye(4).tolist()),0,[31,119,180],model,roi)
        for key in ['red','green','blue']:np.testing.assert_array_equal(added[key],added['leaf_'+key])

if __name__=='__main__':unittest.main()
