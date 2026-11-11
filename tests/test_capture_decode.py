"""Incomplete and duplicate source selections must not become fabricated poses."""
from pathlib import Path
import tempfile,unittest
import numpy as np
from captures.decode import selected_recordings,levelling_rotation

class CaptureDecodeTests(unittest.TestCase):
    def test_excluded_duplicate_and_partial_recordings_are_not_decoded(self):
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary)
            for side in ['sideA','sideB']:
                (root/side).mkdir()
                for i in range(4):(root/side/f'{i}.pcap').write_bytes(b'capture')
            exclusions=[dict(path='sideA/3.pcap',reason='Duplicate on opposite track'),dict(path='sideB/1.pcap',reason='Incomplete packet capture')]
            chosen=[p.relative_to(root).as_posix() for p in selected_recordings(root,exclusions)]
            self.assertEqual(len(chosen),6);self.assertNotIn('sideA/3.pcap',chosen);self.assertNotIn('sideB/1.pcap',chosen)
            with self.assertRaisesRegex(ValueError,'missing'):selected_recordings(root,[dict(path='sideB/missing.pcap',reason='bad exclusion')])

    def test_gravity_requires_observations_and_produces_right_handed_frame(self):
        with self.assertRaisesRegex(ValueError,'Insufficient IMU'):levelling_rotation([])
        with self.assertRaisesRegex(ValueError,'Insufficient IMU'):levelling_rotation([[0,-1,0]])
        up,rotation=levelling_rotation([[.01,-1,.02],[.01,-1,.02],[.01,-1,.02]])
        np.testing.assert_allclose(up@rotation.T,[0,0,1],atol=1e-12)
        np.testing.assert_allclose(rotation@rotation.T,np.eye(3),atol=1e-12);self.assertAlmostEqual(np.linalg.det(rotation),1.)

if __name__=='__main__':unittest.main()
