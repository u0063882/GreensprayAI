"""Measured-size final refinement must close a gap without merging the post."""
from pathlib import Path
import json,tempfile,unittest
import numpy as np
from captures.trunk_refinement import refine_trunks,_apply,_correction


class TrunkRefinementTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rng=np.random.default_rng(21);seeds=[(k,np.array([x,0.])) for k,x in enumerate([0.,1.5,3.])]
        clouds=[];origins=[];labels=[];truth=[];original=[]
        # Sizes vary across trees. Every source is rigidly misplaced; the post
        # remains a distinct 7cm object, not another observation of the trunk.
        dy=[-.012,0.,.018,.080,.105,.115];yaw=np.deg2rad([.1,0.,-.1,-.5,0.,.4])
        centres=np.array([[x,0.,.3] for _,(x,_) in seeds])
        for i in range(6):
            side='A' if i<3 else 'B';offset=np.pi if side=='A' else 0.;parts=[]
            theta=np.linspace(.12,np.pi-.12,28)+offset;heights=np.linspace(.155,.445,20);t,z=np.meshgrid(theta,heights)
            for k,seed in seeds:
                for radius,delta in [([.066,.071,.076][k],[0,0]),(.036,[.20,.025])]:
                    p=np.c_[seed[0]+delta[0]+radius*np.cos(t).ravel()+.04*(z.ravel()-.3),delta[1]+radius*np.sin(t).ravel()-.06*(z.ravel()-.3),z.ravel()]
                    parts.append(p+rng.normal(0,.0008,p.shape))
            x,y=np.meshgrid(np.linspace(-.3,3.3,180),np.linspace(-.4,.4,45));parts.append(np.c_[x.ravel(),y.ravel(),np.zeros(x.size)])
            p=np.concatenate(parts);C=_correction([dy[i],yaw[i]],[1.5,0.]);clouds.append(_apply(p,C));origins.append(_apply(np.array([[1.5,-2. if side=='A' else 2.,1.6]]),C)[0]);labels.append(side);truth.append(_apply(centres,C));original.append(clouds[-1].copy())
        reference=dict(trunk={'circumferences_cm':[41,44.8,45,47.6,46.2,46.5]},support={'circumferences_cm':[22.6,22.9,21.7,21.6,22.6,24.1]})
        corrections,report=refine_trunks(clouds,origins,labels,seeds,reference)
        cls.clouds=clouds;cls.original=original;cls.corrections=corrections;cls.report=report;cls.truth=truth

    def test_gap_larger_than_old_25mm_limit_improves_on_independent_centres(self):
        before=np.array(self.truth);after=np.array([_apply(p,C) for p,C in zip(self.truth,self.corrections)])
        first=np.median(before[3:,:,1],axis=0)-np.median(before[:3,:,1],axis=0);last=np.median(after[3:,:,1],axis=0)-np.median(after[:3,:,1],axis=0)
        self.assertGreater(np.median(abs(first)),.07);self.assertLess(np.median(abs(last)),.015)
        self.assertGreater(max(abs(r['across_translation_m']) for r in self.report['per_recording']),.025)

    def test_rigidity_height_and_input_samples_are_preserved(self):
        for p,original,C in zip(self.clouds,self.original,self.corrections):
            np.testing.assert_array_equal(p,original);np.testing.assert_allclose(C[:3,:3].T@C[:3,:3],np.eye(3),atol=1e-12);self.assertAlmostEqual(np.linalg.det(C[:3,:3]),1.)
            np.testing.assert_array_equal(_apply(p,C)[:,2],p[:,2])
            self.assertAlmostEqual(np.linalg.norm(p[0]-p[300]),np.linalg.norm(_apply(p[[0,300]],C)[0]-_apply(p[[0,300]],C)[1]),places=10)

    def test_known_post_is_a_separate_landmark_and_sizes_are_not_identical(self):
        self.assertGreaterEqual(len(self.report['support_models']),1)
        sizes=[m['diameter_cm'] for m in self.report['trunk_models'].values()]
        self.assertGreater(max(sizes)-min(sizes),.4)
        for observation in self.report['object_observations']:
            if observation['training']:
                p=self.clouds[observation['scan']][observation['indices']]
                self.assertTrue(np.all((p[:,2]<.24)|(p[:,2]>.36)))
        self.assertEqual(self.report['validation']['height_bands_m'],[[.26,.34]])


class ManualReferenceTests(unittest.TestCase):
    def test_population_references_keep_net_caveat_and_correct_support_columns(self):
        reference=json.loads((Path(__file__).resolve().parents[1]/'presets/manual_measurements.json').read_text())
        self.assertIn('no exact mapping',reference['tree_mapping']);self.assertIn('loose',reference['net_caveat'])
        self.assertEqual(reference['families']['spindle']['observations'][1]['remark'],'Net')
        self.assertEqual(reference['families']['chandelier']['support']['circumferences_cm'],[28.3,26.5])
        self.assertEqual(reference['families']['vhedge']['support']['circumferences_cm'],[25.9,21.])


if __name__=='__main__':unittest.main()
