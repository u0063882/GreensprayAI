"""Geometry, normal-sign invariance and preservation of raw source files."""
import hashlib
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np
from PIL import Image

from captures.leafwood import write_records
from dynamic.animate import load_results, render
from dynamic.lighting import estimate_display_normals, diffuse_factors
from tests.test_dynamic_animation import result


class NormalLightingTests(unittest.TestCase):
    def test_plane_normal_and_confidence_are_batch_and_translation_invariant(self):
        # Irregular samples avoid exact kNN distance ties whose neighbor set
        # can legitimately change after floating-point coordinate translation.
        xy = np.random.default_rng(72).uniform(-1, 1, (3000, 2))
        plane = np.c_[xy, np.zeros(len(xy))]
        query = plane[::150]
        normals, confidence = estimate_display_normals(plane, query, radius=.2, batch_size=3)
        np.testing.assert_allclose(abs(normals[:, 2]), 1., atol=1e-12)
        self.assertGreater(np.median(confidence), .9)
        shifted, c = estimate_display_normals(plane + 1e6, query + 1e6, radius=.2, batch_size=13)
        np.testing.assert_allclose(abs(shifted), abs(normals), atol=1e-9)
        np.testing.assert_allclose(c, confidence, atol=1e-8)

    def test_unsupported_sparse_linear_and_volumetric_neighborhoods_stay_unshaded(self):
        cases = [np.zeros((3, 3)), np.c_[np.linspace(-1, 1, 31), np.zeros((31, 2))],
                 np.array(np.meshgrid([-1., 0., 1.], [-1., 0., 1.], [-1., 0., 1.])).reshape(3, -1).T]
        for cloud in cases:
            normals, confidence = estimate_display_normals(cloud, np.zeros((1, 3)), radius=3., neighbors=32)
            np.testing.assert_allclose(confidence, 0., atol=1e-12)
            np.testing.assert_allclose(diffuse_factors(normals, confidence), 1.)

    def test_two_sided_fixed_lighting_is_normal_sign_invariant_and_has_ambient_floor(self):
        normals = np.eye(3)
        confidence = np.ones(3)
        factors = diffuse_factors(normals, confidence, ambient=.6, azimuth=0, elevation=0)
        np.testing.assert_allclose(factors, [1., .6, .6])
        np.testing.assert_allclose(diffuse_factors(-normals, confidence, ambient=.6, azimuth=0, elevation=0), factors)
        np.testing.assert_allclose(diffuse_factors(normals, np.zeros(3)), 1.)

    def test_shaded_gif_changes_display_without_editing_ply_or_trajectory(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary) / 'results'
            result(directory)
            records = load_results(directory)['records']
            records['z'] = -.8
            write_records(directory / 'registered.ply', records)
            paths = [directory / 'registered.ply', directory / 'trajectory.csv']
            digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
            before = [digest(p) for p in paths]
            with patch('socket.socket.connect', side_effect=AssertionError('Lighting must stay offline')):
                rgb = render(directory, fps=4, speed=2, width=640, height=400, view='top', hold=0.)
                shaded = render(directory, fps=4, speed=2, width=640, height=400, view='top', hold=0.,
                                shading='normals', normal_radius=.8)
            self.assertEqual([digest(p) for p in paths], before)
            self.assertGreater(shaded['lighting']['shaded_points'], 0)
            self.assertFalse(shaded['lighting']['cast_shadows'])
            self.assertEqual(rgb['estimated_speed'], shaded['estimated_speed'])
            images = []
            for name in ['accumulation.gif', 'accumulation_normals.gif']:
                with Image.open(directory / name) as gif:
                    gif.seek(gif.n_frames - 1)
                    images.append(np.array(gif.convert('RGB')))
            self.assertFalse(np.array_equal(*images))


if __name__ == '__main__':
    unittest.main()
