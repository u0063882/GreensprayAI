"""Rigid per-recording degrees of freedom and preservation of physical stems."""
import tempfile
from pathlib import Path
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from processing.register_anchored import correction_matrix,move,relative_correction_residual,stem_residual,visibility_penalty,solve_west,BOUNDS
from processing.export_anchored import read_records,write_records


class AnchoredGeometryTests(unittest.TestCase):
    def test_surface_distance_does_not_allow_the_wrong_bark_hemisphere(self):
        points=np.array([[0.,-.08,.3],[0.,.08,.3]])
        model=np.array([0.,0.,.08,0.,0.,0.]);scanner=np.array([0.,2.,1.5])
        np.testing.assert_allclose(stem_residual(points,model),0,atol=1e-12)
        penalty=visibility_penalty(points,scanner,model)
        self.assertGreater(penalty[0],.05)
        self.assertEqual(penalty[1],0.)

    def test_leaning_tapered_stem_keeps_opposite_physical_faces(self):
        model=np.array([2.,2.,.08,.18,-.22,-.04]);axis=np.r_[model[3:5],1.];axis/=np.linalg.norm(axis)
        first=np.array([1.,0.,-model[3]]);first/=np.linalg.norm(first);second=np.cross(axis,first)
        angle=np.linspace(0,2*np.pi,180);t=np.linspace(-.12,.12,180);radius=model[2]+model[5]*t
        points=np.r_[model[:2],.3]+t[:,None]*axis+radius[:,None]*(np.cos(angle)[:,None]*first+np.sin(angle)[:,None]*second)
        np.testing.assert_allclose(stem_residual(points,model),0,atol=1e-12)
        self.assertGreater(np.ptp(points[:,1]),.14)

    def test_common_rigid_motion_is_not_penalized_by_different_pivots(self):
        pivots=[np.array([.5,2.,.3]),np.array([5.,2.,.3])];rotvec=np.array([.002,-.001,.018]);R=Rotation.from_rotvec(rotvec).as_matrix();shift=np.array([.01,-.08,.005])
        values=[np.r_[rotvec,R@p+shift-p] for p in pivots]
        np.testing.assert_allclose(relative_correction_residual(*values,*pivots),0,atol=1e-12)
        points=np.array([[0.,0.,0.],[2.,3.,1.],[-1.,1.,.5]])
        moved=move(points,values[0],pivots[0]);np.testing.assert_allclose(np.linalg.norm(np.diff(moved,axis=0),axis=1),np.linalg.norm(np.diff(points,axis=0),axis=1))

    def test_ply_export_preserves_non_coordinate_measurements(self):
        header=['ply\n','format binary_little_endian 1.0\n','element vertex 3\n','property float x\n','property float y\n','property float z\n','property uchar reflectivity\n','property uchar scan_id\n','end_header\n']
        records=np.zeros(3,dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('reflectivity','u1'),('scan_id','u1')]);records['reflectivity']=[5,120,255];records['scan_id']=11
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'scan.ply';write_records(path,header,records);_,actual=read_records(path)
            np.testing.assert_array_equal(actual,records)

    def test_separate_recording_errors_are_not_replaced_by_one_offset(self):
        models={k:dict(parameters=[x,2.,.07,0,0,0]) for k,x in [(1,.5),(4,3.),(5,4.)]}
        errors=np.array([-.02,.01,.04,.06,.04,.02,-.01,.01,.03]);stems=[];grounds=[];pivots=[];origins=[]
        for i,error in enumerate(errors):
            pivot=np.array([.5+i*.4,2.,.3]);origin=pivot+[0.,2.,1.2]
            groups={}
            for key,model in models.items():
                facing=np.arctan2(origin[1]-2.,origin[0]-model['parameters'][0])
                theta=facing+np.linspace(-.8,.8,50)
                p=[]
                for z in [.16,.29,.43]:
                    p.append(np.c_[model['parameters'][0]+.07*np.cos(theta),2.+.07*np.sin(theta)+error,np.full(len(theta),z)])
                groups[key]=np.concatenate(p)
            stems.append(groups);q=np.array([[x,y,0.] for x in [.5,2.,4.] for y in [1.6,2.,2.4]])
            grounds.append((q,q.copy(),np.tile([0.,0.,1.],(len(q),1))));pivots.append(pivot);origins.append(origin+[0.,error,0.])
        parameters,_,success=solve_west(stems,grounds,pivots,models,origins=origins,verbose=False)
        self.assertTrue(success)
        self.assertGreater(np.ptp(parameters[:,4]),.025)
        self.assertTrue(np.all(abs(parameters)<=BOUNDS+1e-9))
        self.assertLess(np.sqrt(np.mean((parameters[:,4]+errors)**2)),.02)

if __name__=='__main__':unittest.main()
