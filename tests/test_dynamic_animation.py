"""Time alignment, complete-map ending and actual GIF decoding."""
import csv
import hashlib
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np
from PIL import Image

from captures.leafwood import write_records
from dynamic.animate import (load_results, animation_clock, sensor_position, render,
                             trajectory_speeds, sensor_quaternion, rotate_vector,
                             initial_camera_up, project_sensor_view)


def result(directory, point_times=True):
    directory.mkdir(parents=True)
    fields = [('x', 'f4'), ('y', 'f4'), ('z', 'f4'), ('red', 'u1'),
              ('green', 'u1'), ('blue', 'u1'), ('scan_id', 'u4')]
    if point_times:
        fields.append(('time_offset_s', 'f8'))
    records = np.zeros(300, dtype=fields)
    rng = np.random.default_rng(22)
    records['x'] = rng.uniform(1, 3, len(records))
    records['y'] = rng.uniform(-2, 2, len(records))
    records['z'] = rng.uniform(-1, 2, len(records))
    records['green'] = 200
    records['red'] = 30
    records['scan_id'] = np.repeat([0, 1, 2], 100)
    if point_times:
        records['time_offset_s'] = np.repeat([0., .5, 1.], 100)
        records['time_offset_s'][-1] = 1.1  # Later than the last scan-centre pose.
    write_records(directory / 'registered.ply', records)
    with (directory / 'trajectory.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['frame_index', 'time_s', 'x_m', 'y_m', 'z_m'])
        writer.writerows([[0, 0., 0., 0., 0.], [1, .5, 0., 1., 0.], [2, 1., 0., 2., 0.]])


class MovingAnimationTests(unittest.TestCase):
    def test_speed_fit_uses_capture_seconds_and_handles_irregular_sampling(self):
        times = np.array([0., .08, .22, .39, .58, .81, 1.05, 1.2])
        velocity = np.array([1.2, -.5, .1])
        positions = np.array([70., -20., 3.]) + times[:, None] * velocity
        speed = trajectory_speeds(times, positions, window=.6)
        np.testing.assert_allclose(speed, np.linalg.norm(velocity), atol=1e-12)
        np.testing.assert_allclose(trajectory_speeds(times, np.tile(positions[0], (len(times), 1))), 0.)
        self.assertTrue(np.isnan(trajectory_speeds(times[:1], positions[:1])[0]))

    def test_quaternion_interpolation_tracks_orientation_and_antipodal_signs(self):
        q = np.array([[0., 0., 0., 1.], [0., 0., 1., 0.]])
        middle = sensor_quaternion(np.array([0., 2.]), q, 1.)
        np.testing.assert_allclose(rotate_vector(middle, np.array([1., 0., 0.])), [0, 1, 0], atol=1e-12)
        same = np.array([[0., 0., 0., 1.], [0., 0., 0., -1.]])
        np.testing.assert_allclose(sensor_quaternion(np.array([0., 2.]), same, 1.), same[0])

    def test_sensor_eye_projection_moves_with_sensor_and_clips_behind_it(self):
        identity = np.array([0., 0., 0., 1.])
        forward = np.array([1., 0., 0.])
        up = initial_camera_up(identity, forward)
        points = np.array([[2., 0., 0.], [-2., 0., 0.], [.01, 0., 0.], [2., 10., 0.], [30., 0., 0.]])
        uv, indices, depth = project_sensor_view(points, np.zeros(3), identity, up, forward, 90., 1., .1, 25.)
        np.testing.assert_array_equal(indices, [0])
        np.testing.assert_allclose(uv, [[0, 0]])
        uv, indices, depth = project_sensor_view(points, np.array([0., .5, 0.]), identity, up, forward, 90., 1., .1, 25.)
        np.testing.assert_allclose(uv, [[.25, 0]])
        # Actual mounting can rotate vertical away from body +Z. The initial
        # camera calibration makes world +Z point up without changing points.
        mounted = np.array([-np.sqrt(.5), 0., 0., np.sqrt(.5)])
        up_body = initial_camera_up(mounted, forward)
        uv, _, _ = project_sensor_view(np.array([[2., 0., 1.]]), np.zeros(3), mounted, up_body, forward, 90., 1., .1, 25.)
        np.testing.assert_allclose(uv, [[0, .5]], atol=1e-12)

    def test_point_times_reveal_only_acquired_points_and_include_last_return(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary) / 'run/results'
            result(directory)
            data = load_results(directory.parent)
            self.assertEqual(data['directory'], directory)
            times = np.sort(data['point_times'])
            self.assertEqual(np.searchsorted(times, .49, side='right'), 100)
            self.assertEqual(np.searchsorted(times, .5, side='right'), 200)
            clock, duration = animation_clock(0, float(times[-1]), 10, 2)
            self.assertEqual(duration, 100)
            self.assertEqual(clock[-1], 1.1)
            self.assertEqual(np.searchsorted(times, clock[-1], side='right'), 300)
            np.testing.assert_allclose(sensor_position(data['times'], data['positions'], .25), [0, .5, 0])
            np.testing.assert_allclose(sensor_position(data['times'], data['positions'], 1.1), [0, 2, 0])

    def test_missing_point_times_fall_back_to_scan_id_join_and_reject_other_trajectory(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary) / 'results'
            result(directory, point_times=False)
            data = load_results(directory)
            np.testing.assert_array_equal(data['point_times'], np.repeat([0., .5, 1.], 100))
            csv = directory / 'trajectory.csv'
            csv.write_text(csv.read_text().replace('2,1.0', '9,1.0'))
            with self.assertRaisesRegex(ValueError, 'do not match'):
                load_results(directory)

    def test_streamed_gif_has_loop_timing_motion_and_does_not_edit_inputs(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary) / 'results'
            result(directory)
            inputs = [directory / 'registered.ply', directory / 'trajectory.csv']
            digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
            before = [digest(p) for p in inputs]
            with patch('socket.socket.connect', side_effect=AssertionError('Animation must stay offline')):
                report = render(directory, fps=4, speed=2, max_points=250,
                                width=640, height=400, view='both', hold=.5)
            self.assertEqual([digest(p) for p in inputs], before)
            self.assertEqual(report['sampled_points'], 250)
            self.assertEqual(report['capture_time_range_s'][-1], 1.1)
            with Image.open(directory / 'accumulation.gif') as gif:
                self.assertEqual(gif.info['loop'], 0)
                self.assertEqual(gif.n_frames, report['frames'])
                first = np.array(gif.convert('RGB'))
                durations = []
                for i in range(gif.n_frames):
                    gif.seek(i)
                    durations.append(gif.info['duration'])
                last = np.array(gif.convert('RGB'))
                self.assertFalse(np.array_equal(first, last))
                self.assertEqual(durations[-1], durations[0] + 500)
                self.assertAlmostEqual(sum(durations) / 1000, report['encoded_duration_s'])
            self.assertEqual(list(directory.glob('.*.gif')), [])

    def test_invalid_clock_options_are_rejected(self):
        for fps, speed in [(0, 1), (10, -1), (float('nan'), 1)]:
            with self.assertRaises(ValueError):
                animation_clock(0, 1, fps, speed)

    def test_follow_render_uses_pose_quaternions_and_speed_is_not_playback_rate(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary) / 'results'
            result(directory)
            trajectory = directory / 'trajectory.csv'
            trajectory.write_text('frame_index,time_s,x_m,y_m,z_m,qx,qy,qz,qw\n'
                                  '0,0,0,0,0,0,0,0,1\n1,0.5,0,1,0,0,0,0,1\n2,1,0,2,0,0,0,0,1\n')
            with patch('socket.socket.connect', side_effect=AssertionError('Animation must stay offline')):
                a = render(directory, fps=4, speed=1, max_points=200, width=640, height=400, hold=0., camera='sensor')
                b = render(directory, directory / 'faster.gif', fps=4, speed=2, max_points=200, width=640, height=400, hold=0., camera='sensor')
            self.assertEqual(a['camera'], 'sensor')
            self.assertTrue(a['top_view_fixed'])
            self.assertEqual(a['estimated_speed']['values_m_s'], b['estimated_speed']['values_m_s'])
            np.testing.assert_allclose(a['estimated_speed']['values_m_s'], 2.)
            with Image.open(directory / 'accumulation_follow.gif') as gif:
                self.assertEqual(gif.n_frames, a['frames'])
                first = np.array(gif.convert('RGB'))
                gif.seek(gif.n_frames - 1)
                last = np.array(gif.convert('RGB'))
                self.assertFalse(np.array_equal(first, last))

    def test_follow_view_requires_recorded_orientation(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary) / 'results'
            result(directory)
            with self.assertRaisesRegex(ValueError, 'qx,qy,qz,qw'):
                render(directory, camera='sensor')


if __name__ == '__main__':
    unittest.main()
