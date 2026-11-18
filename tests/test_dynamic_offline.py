"""Local input isolation and clock alignment with unequal sampling rates."""
import io
import json
import tempfile
import unittest
import zipfile
from pathlib import Path
from unittest.mock import patch

import h5py
import numpy as np

from dynamic.offline import parser, prepare_input
from dynamic.pilot import navigation, ground_check
from dynamic.prepare import trim_classic_pcap
from tests.test_dynamic_pilot import capture


class OfflineMovingTests(unittest.TestCase):
    def test_raw_input_preparation_preserves_sources_and_time_boundary(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / 'capture.pcap'
            source.write_bytes(capture([(3271, 0, b'lidar'), (3285, 900000, b'imu'), (3286, 0, b'outside')]))
            metadata = root / 'different-name.json'
            metadata.write_text('{"calibration": true}')
            gps = root / 'navigation.h5'
            gps.write_bytes(b'GPS preserved byte-for-byte')
            originals = {p: p.read_bytes() for p in [source, metadata, gps]}
            args = parser().parse_args(['--pcap', str(source), '--metadata', str(metadata), '--gps', str(gps), '--output', str(root / 'new result'), '--gps-clock', 'shared-start'])
            with patch('socket.socket.connect', side_effect=AssertionError('Network must not be used')):
                prepared, out = prepare_input(args)
            self.assertEqual(prepared.read_bytes(), capture([(3271, 0, b'lidar'), (3285, 900000, b'imu')]))
            self.assertEqual(prepared.with_suffix('.json').read_bytes(), originals[metadata])
            self.assertEqual(prepared.with_suffix('.h5').read_bytes(), originals[gps])
            for p, data in originals.items():
                self.assertEqual(p.read_bytes(), data)
            self.assertEqual(json.loads((out / 'input/preparation.json').read_text())['gps_clock_policy'], 'shared-start')
            with self.assertRaisesRegex(ValueError, 'new or empty'):
                prepare_input(args)

    def test_explicit_all_keeps_complete_zip_capture_without_network(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            raw = capture([(100, 0, b'first'), (114, 900000, b'imu'), (116, 0, b'after default cutoff')])
            archive = root / 'Dynamic.zip'
            with zipfile.ZipFile(archive, 'w') as z:
                z.writestr('sub/capture.pcap', raw)
                z.writestr('sub/capture.json', '{}')
            original = archive.read_bytes()
            args = parser().parse_args(['--zip', str(archive), '--all', '--output', str(root / 'new')])
            with patch('socket.socket.connect', side_effect=AssertionError('Network must not be used')):
                pcap, out = prepare_input(args)
            self.assertEqual(pcap.read_bytes(), raw)
            self.assertIsNone(json.loads((out / 'input/preparation.json').read_text())['requested_seconds'])
            self.assertEqual(archive.read_bytes(), original)

    def test_rejects_invalid_duration_instead_of_accidentally_running_all(self):
        with tempfile.TemporaryDirectory() as directory:
            for seconds in [0, -1, float('nan'), float('inf')]:
                with self.assertRaises(ValueError):
                    trim_classic_pcap(io.BytesIO(capture([(100, 0, b'point')])), Path(directory) / 'bad', seconds)

    def test_shared_start_uses_actual_times_not_sample_indices_or_duration_scaling(self):
        with tempfile.TemporaryDirectory() as directory:
            p = Path(directory) / 'gps.h5'
            gps_relative = np.array([0., .03, .12, .2, .58, .9, 1.1])
            lat = 50. + gps_relative * 1e-5
            lon = 4. + gps_relative * 2e-5
            with h5py.File(p, 'w') as h:
                h['time'] = 1700000000. + gps_relative
                h['lat'] = lat
                h['lon'] = lon
            relative = np.arange(.05, 1.05, .1)
            host = 3271. + relative
            positions = np.c_[relative, np.zeros((len(relative), 2))]
            direct, _ = navigation(p, host, positions)
            self.assertEqual(direct['matched_frames'], 0)
            report, graph = navigation(p, host, positions, clock='shared-start', capture_start=3271.)
            self.assertEqual(report['matched_frames'], len(host))
            self.assertEqual(report['gps_samples'], len(gps_relative))
            np.testing.assert_allclose(graph['time'] - 1700000000., relative, atol=3e-7)
            # Linear motion must follow elapsed time despite irregular GPS samples.
            delta = graph['gps'][-1] - graph['gps'][0]
            fractions = (relative - relative[0]) / (relative[-1] - relative[0])
            np.testing.assert_allclose(graph['gps'], fractions[:, None] * delta, atol=1e-6)
            self.assertTrue(report['no_scale_fit'])
            self.assertNotIn('lat', report)
            self.assertNotIn('lon', report)

    def test_missing_requested_metadata_does_not_create_partial_output(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            p = root / 'input.pcap'
            p.write_bytes(capture([(100, 0, b'packet')]))
            args = parser().parse_args(['--pcap', str(p), '--output', str(root / 'new')])
            with self.assertRaises(FileNotFoundError):
                prepare_input(args)
            self.assertFalse((root / 'new').exists())

    def test_three_scan_run_can_report_ground_without_four_time_groups(self):
        rng = np.random.default_rng(71)
        points = np.c_[rng.uniform(-3, 3, (1200, 2)), rng.normal(0, .003, 1200)]
        report = ground_check(points, [{'after': points}] * 3)
        self.assertTrue(report['supported'])
        self.assertEqual(len(report['time_quartiles']), 4)
        self.assertEqual(report['time_quartiles'][-1]['points'], 0)


if __name__ == '__main__':
    unittest.main()
