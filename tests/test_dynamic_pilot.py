"""Check the time boundary, packet preservation and per-column geometry."""
import io,struct,tempfile,unittest,zipfile,hashlib
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from dynamic.prepare import trim_classic_pcap,prepare_local
from dynamic.pilot import gravity_pose,holdout_mask
from ouster.sdk import core

def capture(packets,endian='<',nano=False):
    magic=(b'\x4d\x3c\xb2\xa1' if nano else b'\xd4\xc3\xb2\xa1') if endian=='<' else (b'\xa1\xb2\x3c\x4d' if nano else b'\xa1\xb2\xc3\xd4')
    out=magic+struct.pack(endian+'HHIIII',2,4,0,0,65535,1)
    for sec,frac,payload in packets:out+=struct.pack(endian+'IIII',sec,frac,len(payload),len(payload))+payload
    return out

class DynamicPilotTests(unittest.TestCase):
    def test_time_cutoff_preserves_all_packet_types_and_bytes(self):
        for endian in ['<','>']:
            for nano in [False,True]:
                multiplier=1000 if nano else 1
                packets=[(100,500000*multiplier,b'lidar'),(100,501000*multiplier,b'imu'),(115,499000*multiplier,b'last'),(115,500000*multiplier,b'excluded')]
                original=capture(packets,endian,nano)
                with tempfile.TemporaryDirectory() as d:
                    out=Path(d)/'trim.pcap';r=trim_classic_pcap(io.BytesIO(original),out,15.)
                    self.assertEqual(out.read_bytes(),capture(packets[:3],endian,nano));self.assertEqual(r['packets'],3);self.assertLess(r['capture_duration_s'],15.)

    def test_truncated_packets_and_wrong_formats_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            for raw in [b'pcapng',capture([(1,0,b'abc')])[:-1],capture([])+b'short']:
                with self.assertRaises(ValueError):trim_classic_pcap(io.BytesIO(raw),Path(d)/'bad',15.)

    def test_whole_input_checksum_verified_before_prefix_extraction(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);p=root/'input.zip'
            with zipfile.ZipFile(p,'w',zipfile.ZIP_DEFLATED) as z:
                z.writestr('nested/test.pcap',capture([(1,0,b'lidar'),(16,0,b'do not process')]))
                z.writestr('nested/test.json','{}')
            with self.assertRaisesRegex(RuntimeError,'SHA-256'):prepare_local(p,root/'bad',15.,'bad')
            self.assertFalse(list((root/'bad').glob('*.pcap')))
            out,r=prepare_local(p,root/'good',15.,hashlib.sha256(p.read_bytes()).hexdigest())
            self.assertTrue(r['full_archive_checksum_verified']);self.assertEqual(r['packets'],1);self.assertEqual(out.read_bytes(),capture([(1,0,b'lidar')]))

    def test_gravity_initialization_is_rigid_and_preserves_shape(self):
        up=np.array([.2,-.3,.93]);up/=np.linalg.norm(up);T=gravity_pose(np.tile(up,(50,1)))
        np.testing.assert_allclose(T[:3,:3]@up,[0,0,1],atol=1e-12)
        np.testing.assert_allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-12);self.assertAlmostEqual(np.linalg.det(T[:3,:3]),1.)

    def test_sdk_dewarp_matches_column_pose_without_destaggering(self):
        rng=np.random.default_rng(31);p=rng.normal(size=(128,32,3));T=np.tile(np.eye(4),(32,1,1));T[:,:3,:3]=Rotation.from_rotvec(rng.normal(0,.03,(32,3))).as_matrix();T[:,:3,3]=rng.normal(size=(32,3))
        expected=np.einsum('wij,hwj->hwi',T[:,:3,:3],p)+T[None,:,:3,3]
        np.testing.assert_allclose(core.dewarp(p,T),expected,atol=1e-12)
        held=holdout_mask(128,1024);self.assertAlmostEqual(held.mean(),.20,places=4)

if __name__=='__main__':unittest.main()
