"""Attribute preservation, global labels and same-face semantic registration."""
from pathlib import Path
import tempfile,unittest
import numpy as np
from captures.leafwood import fit_leafwood_model,classify_leafwood,enriched_records,write_records,coordinate_digest
from captures.register_leafwood import semantic_weights,make_pairs,DEFAULTS,register_leafwood
from processing.export_anchored import read_records,xyz
from scipy.spatial.transform import Rotation


class LeafwoodAttributeTests(unittest.TestCase):
    def test_shared_model_is_scan_independent_and_fixed_signal_brightness(self):
        model=fit_leafwood_model([np.tile([100,200,15000,18000],100),np.tile([120,220,16000,19000],100)])
        a=classify_leafwood(np.array([1000,2000,3000,4000]),np.array([100,200,18000,19000]),model)
        b=classify_leafwood(np.array([1000]),np.array([100]),model)
        self.assertEqual(a['leafwood_class'].tolist(),[1,1,2,2]);np.testing.assert_array_equal(a['rgb'][:1],b['rgb'])
        self.assertGreater(a['wood_score'][0],a['wood_score'][-1])
        self.assertEqual(model['model_sha256'],fit_leafwood_model([np.tile([100,200,15000,18000],100),np.tile([120,220,16000,19000],100)])['model_sha256'])

    def test_ply_roundtrip_preserves_raw_channels_colour_and_large_source_index(self):
        model=fit_leafwood_model([np.array([100,150,16000,18000])]);signal=np.array([0,65535,12345,500],dtype='u2');nir=np.array([100,150,16000,18000],dtype='u2')
        original=np.zeros(4,dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('red','u1'),('green','u1'),('blue','u1'),('reflectivity','u1')]);original['x']=[.01,2,3,4];original['reflectivity']=[0,255,33,17]
        attrs=dict(signal=signal,near_ir=nir,frame_id=np.arange(4,dtype='u4'),pixel_index=np.array([0,131071,200,4096],dtype='u4'),**classify_leafwood(signal,nir,model))
        records=enriched_records(original,attrs,np.array([0,256,1000,70000],dtype='u4'))
        self.assertEqual(coordinate_digest(records),coordinate_digest(original))
        by_scan=enriched_records(records,attrs,records['scan_id'],by_scan=True)
        with tempfile.TemporaryDirectory() as temporary:
            path=Path(temporary)/'cloud.ply';write_records(path,by_scan);_,saved=read_records(path)
        for k in ['x','y','z','reflectivity']:np.testing.assert_array_equal(saved[k],original[k])
        np.testing.assert_array_equal(saved['intensity'],signal);np.testing.assert_array_equal(saved['near_ir'],nir);np.testing.assert_array_equal(saved['scan_id'],[0,256,1000,70000])
        np.testing.assert_array_equal(np.c_[saved['leaf_red'],saved['leaf_green'],saved['leaf_blue']],attrs['rgb'])
        np.testing.assert_array_equal(saved['pixel_index'],attrs['pixel_index'])

    def test_missing_class_separation_is_reported(self):
        with self.assertRaisesRegex(ValueError,'distinct'):fit_leafwood_model([np.ones(100)])


class SemanticRegistrationTests(unittest.TestCase):
    def test_soft_weights_prioritise_wood_without_deleting_uncertain_or_leaf(self):
        w=semantic_weights(np.array([1.,0.,.5,1.]),np.array([1.,0.,.5,0.]))
        self.assertGreater(w[0],w[1]);self.assertGreater(w[2],0);self.assertLess(w[3],w[1])

    def test_independent_refinement_is_rigid_anchor_fixed_and_preserves_opposite_faces(self):
        # Three known stems, opposite bark hemispheres, two overlapping B scans.
        # Supplying identical positions would incorrectly collapse their diameter.
        rng=np.random.default_rng(8);clouds=[];seeds=[(k,np.array([x,0.])) for k,x in enumerate([0.,1.,2.])]
        for side,dy in [('A',0.),('B',.004),('B',-.004)]:
            theta=np.linspace(.15,np.pi-.15,32)+(0 if side=='B' else np.pi)
            z=np.linspace(.125,.455,32);t,h=np.meshgrid(theta,z)
            parts=[]
            for k,centre in seeds:
                parts.append(np.c_[centre[0]+.075*np.cos(t).ravel(),.075*np.sin(t).ravel()+dy,h.ravel()]+rng.normal(0,.0005,(t.size,3)))
            x,y=np.meshgrid(np.linspace(-.2,2.2,161),np.linspace(-.3,.3,41));parts.append(np.c_[x.ravel(),y.ravel(),np.zeros(x.size)])
            clouds.append(np.concatenate(parts))
        original=[p.copy() for p in clouds];scores=[np.full(len(p),.9) for p in clouds]
        corrections,report=register_leafwood(clouds,scores,[[1,-2,1.6],[1,2,1.6],[1,2,1.6]],['A','B','B'],seeds,settings={'maximum_nfev':45})
        np.testing.assert_array_equal(corrections[0],np.eye(4))
        for p,q,T in zip(clouds,original,corrections):
            np.testing.assert_array_equal(p,q);np.testing.assert_allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-12);self.assertAlmostEqual(np.linalg.det(T[:3,:3]),1.)
        self.assertGreater(report['training_pair_points'],100)
        self.assertLess(corrections[1,1,3],0);self.assertGreater(corrections[2,1,3],0)
        # The known 150mm stem cannot be collapsed by a cross-hemisphere match.
        for row in report['shape_checks']:self.assertGreater(row['leafwood_width_m'],.13)
        self.assertLess(report['maximum_transformed_fitting_z_m'],.5)


class EnrichedCollectionTests(unittest.TestCase):
    def test_repackage_reuses_verified_attributes_and_rejects_changed_files(self):
        import json
        from unittest.mock import patch
        from tools.leafwood_collection import enrich_collection,output_hashes
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);config={'datasets':[{'id':'orchard'}]}
            for name in ['pointclouds/orchard/final_registered.ply','contributions/orchard/scan.ply']:
                path=root/name;path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(b'EXACT ENRICHED POINTS')
            marker=root/'QC/provenance/leafwood_upgrade.json';marker.parent.mkdir(parents=True)
            marker.write_text(json.dumps({'pipeline_fingerprint':'same','output_hashes':output_hashes(root,config)}))
            # General README changes do not invalidate per-dataset provenance.
            (root/'pointclouds/README.txt').write_text('Current help text')
            with patch('tools.leafwood_collection.fingerprint',return_value='same'),patch('tools.leafwood_collection.upgrade_collection',side_effect=AssertionError('Must reuse verified attributes')):
                enrich_collection(root,root/'work',config,'owner/repo')
                (root/'pointclouds/orchard/final_registered.ply').write_bytes(b'CHANGED')
                with self.assertRaisesRegex(RuntimeError,'checksum'):enrich_collection(root,root/'work',config,'owner/repo')


if __name__=='__main__':unittest.main()
