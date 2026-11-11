"""Capture labels must not encode geography; diagnostics must retain bark width."""
import unittest
import numpy as np
from captures.orchard import gps_frame
from captures.export import diagnostic_circle
from captures.bounded import solve_moving

class CaptureGeometryTests(unittest.TestCase):
    def test_inward_headings_follow_tracks_for_either_label_assignment(self):
        tracks=[[(0.,0.),(0.,1.),(0.,2.)],[(4.,2.),(4.,1.),(4.,0.)]]
        for reverse in [False,True]:
            scans=[]
            for side,track in zip(['sideA','sideB'],tracks[::-1] if reverse else tracks):
                for i,(east,north) in enumerate(track):scans.append(dict(tag=f'{side}_{i:06d}',lat=50.+north/111319.49,lon=5.+east/(111319.49*np.cos(np.deg2rad(50.)))))
            gps,yaw,a,b,_=gps_frame(scans)
            for ids,other in [(a,b),(b,a)]:
                toward=np.median(gps[other],axis=0)-gps[ids]
                heading=np.c_[np.cos(yaw[ids]),np.sin(yaw[ids])]
                self.assertTrue(np.all(np.sum(toward*heading,axis=1)>3.5))

    def test_opposite_partial_arcs_keep_one_centre_with_outliers(self):
        rng=np.random.default_rng(7);centres=[]
        for offset in [0,np.pi]:
            theta=np.linspace(.2,np.pi-.2,130)+offset
            q=.075*np.c_[np.cos(theta),np.sin(theta)]+rng.normal(0,.0015,(130,2))
            q=np.vstack([q,rng.uniform(-.2,.2,(25,2))])
            fit,_,_=diagnostic_circle(q,[0,0],rng);centres.append(fit[:2])
            self.assertLess(np.linalg.norm(fit[:2]),.015);self.assertLess(abs(fit[2]-.075),.012)
        self.assertLess(np.linalg.norm(centres[0]-centres[1]),.02)

    def test_bounded_solver_accepts_variable_recording_count_and_independent_poses(self):
        theta=np.linspace(.2,np.pi-.2,90);base=np.c_[.08*np.cos(theta),.08*np.sin(theta),np.full(90,.3)]
        stems=[{1:base+[0,dy,0]} for dy in [.035,-.025]]
        x,y=np.meshgrid(np.linspace(-.2,.2,5),np.linspace(-.2,.2,5));ground=np.c_[x.ravel(),y.ravel(),np.zeros(x.size)];normal=np.tile([0.,0.,1.],(len(ground),1))
        grounds=[(ground,ground,normal)]*2;pivots=np.array([[0,0,.3],[0,0,.3]])
        bounds=np.r_[np.deg2rad([.7,.7,3.]),.1,.2,.035];prior=np.r_[np.deg2rad([.2,.2,.8]),.04,.08,.01]
        parameters,_,success=solve_moving(stems,grounds,pivots,{1:{'parameters':[0,0,.08,0,0,0]}},[[0,2,1.5],[0,2,1.5]],bounds,prior,-1,verbose=False)
        self.assertTrue(success);self.assertEqual(parameters.shape,(2,6));self.assertLess(parameters[0,4],0);self.assertGreater(parameters[1,4],0)
        self.assertGreater(abs(parameters[0,4]-parameters[1,4]),.01)

if __name__=='__main__':unittest.main()
