"""Recover rigid scans on sloping soil while keeping genuine trunk lean."""
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from captures.joint_registration import refine_joint,apply,anchor_difference
from captures.recover_views import disconnected_views,recover_views
from captures.structural_validation import _holdout


class JointRegistrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rng=np.random.default_rng(212);clouds=[];true=[];scores=[];origins=[];ups=[];labels=[]
        seeds=[(i,np.array([x,0.])) for i,x in enumerate([.3,1.6,2.9])]
        lean=np.array([.12,-.08]);z=np.linspace(.155,1.7,58);angle=np.linspace(0,2*np.pi,70,endpoint=False)
        theta,zz=np.meshgrid(angle,z);normal=np.c_[np.cos(theta).ravel(),np.sin(theta).ravel(),np.zeros(theta.size)]
        for i in range(6):
            side='A' if i<3 else 'B';origin=np.array([.3+(i%3)*1.3,-2. if side=='A' else 2.,1.6]);parts=[]
            for key,seed in seeds:
                radius=[.065,.070,.075][key];p=np.c_[seed[0]+lean[0]*(zz.ravel()-.3)+radius*normal[:,0],lean[1]*(zz.ravel()-.3)+radius*normal[:,1],zz.ravel()]
                keep=np.sum(normal*(origin-p),axis=1)>0;parts.append(p[keep]+rng.normal(0,.0005,p[keep].shape))
            wood=np.concatenate(parts);x,y=np.meshgrid(np.linspace(-.1,3.3,100)+i*.001,np.linspace(-.45,.45,32)+i*.001)
            ground=np.c_[x.ravel(),y.ravel(),.025*x.ravel()+.04*y.ravel()+.005];p=np.concatenate([wood,ground]);true.append(p)
            rotation=Rotation.from_euler('xyz',[[0,0,0],[.35,-.15,.25],[-.2,.25,-.2],[.4,-.3,.5],[-.3,.2,-.4],[.25,.15,.2]][i],degrees=True).as_matrix();T=np.eye(4);T[:3,:3]=rotation;T[:3,3]=[0,.008,-.008,.055,.075,.065][i]*np.array([.1,1,.1])
            clouds.append(apply(p,T));origins.append(apply(origin[None,:],T)[0]);ups.append(rotation[:,2]);labels.append(side);scores.append(np.r_[np.ones(len(wood)),np.zeros(len(ground))])
        ref=dict(trunk={'circumferences_cm':(np.array([.065,.070,.075])*200*np.pi).tolist()},support={'circumferences_cm':[22.]})
        C,report=refine_joint(clouds,scores,origins,ups,labels,seeds,ref,check_anchors=True)
        cls.clouds=clouds;cls.true=true;cls.C=C;cls.report=report
        cls.inputs=dict(wood_scores=scores,origins=origins,ups=ups,labels=labels,seeds=seeds,reference=ref)

    def test_known_relative_geometry_is_recovered(self):
        corrected=[apply(p,C) for p,C in zip(self.clouds,self.C)]
        # Compare all measured returns to independent synthetic truth after
        # removing only the arbitrary common coordinate frame.
        a=np.concatenate(self.true);b=np.concatenate(corrected);u,_,vt=np.linalg.svd((b-b.mean(0)).T@(a-a.mean(0)));R=vt.T@u.T;t=a.mean(0)-R@b.mean(0);d=np.linalg.norm(b@R.T+t-a,axis=1)
        self.assertLess(np.sqrt(np.mean(d*d)),.008)

    def test_axes_can_lean_and_terrain_is_not_flattened(self):
        self.assertTrue(self.report['retained_landmarks'])
        for item in self.report['retained_landmarks']:
            m=item['parameters'];self.assertGreater(np.linalg.norm(m[2:4]),.07)
        q=apply(self.clouds[0],self.C[0]);n=len(self.true[0])-3200;g=q[n:];fit=np.linalg.lstsq(np.c_[g[:,:2],np.ones(len(g))],g[:,2],rcond=None)[0]
        self.assertGreater(np.linalg.norm(fit[:2]),.025)

    def test_gauge_changes_do_not_change_the_reconstruction(self):
        self.assertTrue(self.report['anchor_checks'])
        for item in self.report['anchor_checks']:self.assertLess(item['rmse_m'],.002)

    def test_ground_only_recording_cannot_silently_pass_horizontal_registration(self):
        inputs=self.inputs;points=self.clouds+[self.clouds[0][-3200:].copy()]
        C,report=refine_joint(points,inputs['wood_scores']+[np.zeros(3200)],
            list(inputs['origins'])+[[1.5,2.,1.6]],list(inputs['ups'])+[[0,0,1]],
            inputs['labels']+['B'],inputs['seeds'],inputs['reference'],check_anchors=False)
        self.assertFalse(report['validation_passed'])
        self.assertTrue(any('No horizontal pose support' in reason for reason in report['rejection_reasons']))

    def test_inputs_and_rigid_point_distances_are_preserved(self):
        for p,C in zip(self.clouds,self.C):
            q=apply(p,C);np.testing.assert_allclose(C[:3,:3].T@C[:3,:3],np.eye(3),atol=1e-12)
            self.assertAlmostEqual(np.linalg.norm(p[0]-p[300]),np.linalg.norm(q[0]-q[300]),places=10)
        for o in self.report['object_observations']:
            z=self.clouds[o['scan']][o['indices'],2]
            if o['training']:self.assertTrue(np.all((z<.24)|(z>.36)))
            else:self.assertTrue(np.all((z>.26)&(z<.34)))


class RegistrationGraphTests(unittest.TestCase):
    def test_disconnected_view_recovers_known_rigid_pose_without_shape_targets(self):
        rng=np.random.default_rng(31);parts=[]
        for centre,radii in [([.2,-.1,1.0],[.16,.22,.32]),([1.2,.2,1.7],[.22,.16,.45]),([2.4,-.2,2.5],[.18,.26,.35])]:
            d=rng.normal(size=(1000,3));d/=np.linalg.norm(d,axis=1)[:,None];parts.append(np.asarray(centre)+d*np.asarray(radii))
        truth=np.concatenate(parts);error=np.eye(4);error[:3,:3]=Rotation.from_euler('z',8,degrees=True).as_matrix();error[:3,3]=[.25,.10,0]
        clouds=[truth.copy() for _ in range(6)]+[apply(truth,error)]
        edges=[dict(i=0,j=1,accepted=True),dict(i=1,j=2,accepted=True),dict(i=3,j=4,accepted=True),dict(i=4,j=5,accepted=True)]
        C,reports=recover_views(clouds,[np.ones(len(truth)) for _ in clouds],np.zeros((7,3)),['A']*3+['B']*4,edges)
        self.assertEqual(len(reports),1);self.assertTrue(reports[0]['accepted'])
        recovered=apply(clouds[6],C[6]);self.assertLess(np.median(np.linalg.norm(recovered-truth,axis=1)),.005)
        np.testing.assert_array_equal(C[:6],np.tile(np.eye(4),(6,1,1)))

    def test_isolated_view_is_not_treated_as_a_successful_anchor(self):
        edges=[dict(i=0,j=1,accepted=True),dict(i=1,j=2,accepted=True),dict(i=3,j=4,accepted=True),dict(i=4,j=5,accepted=True),dict(i=5,j=6,accepted=False)]
        todo,core=disconnected_views(['A']*3+['B']*4,edges)
        self.assertEqual(todo,[6]);self.assertEqual(core['B'],[3,4,5])

if __name__=='__main__':unittest.main()
