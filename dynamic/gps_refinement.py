"""Experimental GPS refinement of an existing SDK trajectory; baseline is read-only.

GPS is genuinely used as a soft residual here, unlike pilot.navigation().
Horizontal corrections preserve height/roll/pitch; no tree-shape prior is used.
"""
from pathlib import Path
import json
import numpy as np
from scipy.spatial.transform import Rotation, Slerp


def load_baseline(directory):
    directory = Path(directory)
    report_name = 'report.json' if (directory / 'report.json').exists() else 'pilot_report.json'
    report = json.loads((directory / report_name).read_text())
    with np.load(directory / 'per_column_poses.npz', allow_pickle=False) as f:
        arrays = {key: f[key] for key in f.files}
    poses = arrays['body_to_world']
    status = arrays['status']
    if poses.ndim != 4 or poses.shape[2:] != (4, 4) or len(poses) < 3 or not np.isfinite(poses).all():
        raise ValueError('Need at least three finite per-column poses')
    centre_columns = np.array([np.flatnonzero(s & 1)[len(np.flatnonzero(s & 1)) // 2] for s in status])
    centres = poses[np.arange(len(poses)), centre_columns]
    capture_start = report['source']['capture_first_unix']
    capture = arrays['host_unix'] - capture_start
    if not np.isfinite(capture).all() or np.any(np.diff(capture) <= 0):
        raise ValueError('Capture times must be finite and increasing')
    return dict(directory=directory, report=report, arrays=arrays, centres=centres,
                positions=centres[:, :3, 3], times=capture, capture_start=capture_start,
                rotations=Rotation.from_matrix(centres[:, :3, :3]))


def interpolate_poses(baseline, times):
    times = np.clip(np.asarray(times), baseline['times'][0], baseline['times'][-1])
    positions = np.column_stack([np.interp(times, baseline['times'], baseline['positions'][:, i]) for i in range(3)])
    rotations = Slerp(baseline['times'], baseline['rotations'])(times).as_matrix()
    return positions, rotations


def load_gps(path, baseline, clock='shared-start'):
    import h5py
    with h5py.File(path) as f:
        t, lat, lon = [np.asarray(f[key]).ravel() for key in ['time', 'lat', 'lon']]
    if not len(t) == len(lat) == len(lon):
        raise ValueError('GPS array lengths differ')
    keep = np.isfinite(t) & np.isfinite(lat) & np.isfinite(lon)
    t, lat, lon = t[keep], lat[keep], lon[keep]
    order = np.argsort(t)
    t, lat, lon = t[order], lat[order], lon[order]
    t, unique = np.unique(t, return_index=True)
    lat, lon = lat[unique], lon[unique]
    if len(t) < 5:
        raise ValueError('Insufficient GPS samples')
    relative = t - t[0] if clock == 'shared-start' else t - baseline['capture_start']
    if clock not in ['shared-start', 'absolute']:
        raise ValueError('Unknown GPS clock policy')
    phi = np.deg2rad(lat.mean())
    north_factor = 111132.92 - 559.82 * np.cos(2 * phi) + 1.175 * np.cos(4 * phi)
    east_factor = 111412.84 * np.cos(phi) - 93.5 * np.cos(3 * phi)
    local = np.column_stack([(lon - lon[0]) * east_factor, (lat - lat[0]) * north_factor])
    overlap = (relative >= baseline['times'][0]) & (relative <= baseline['times'][-1])
    times, xy = relative[overlap], local[overlap]
    if len(times) < 10:
        raise ValueError('Insufficient GPS time coverage; check clock alignment')
    # Entire alternating one-second GPS blocks are held out. Use RAW GPS
    # samples, never interpolated measurements that leak held-out fixes.
    training = np.floor(times).astype(int) % 2 == 0
    if min(training.sum(), (~training).sum()) < 4:
        raise ValueError('Insufficient training and held-out GPS blocks')
    positions, rotations = interpolate_poses(baseline, times)
    return dict(times=times, xy=xy, training=training, positions=positions, rotations=rotations,
                clock=clock, full_log_duration_s=float(t[-1] - t[0]), full_log_samples=len(t))


def yaw_matrix(yaw):
    yaw = np.asarray(yaw)
    c, s = np.cos(yaw), np.sin(yaw)
    matrices = np.zeros(yaw.shape + (3, 3))
    matrices[..., 0, 0] = c
    matrices[..., 0, 1] = -s
    matrices[..., 1, 0] = s
    matrices[..., 1, 1] = c
    matrices[..., 2, 2] = 1.
    return matrices


def rigid_alignment(source, target):
    x, y = source - source.mean(0), target - target.mean(0)
    u, _, vt = np.linalg.svd(x.T @ y)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0:
        vt[-1] *= -1
        rotation = vt.T @ u.T
    translation = target.mean(0) - source.mean(0) @ rotation.T
    return rotation, translation


def gps_scores(predicted, measured, training):
    error = np.linalg.norm(predicted - measured, axis=1)
    def summary(keep):
        d = error[keep]
        return dict(samples=len(d), RMSE_m=float(np.sqrt(np.mean(d ** 2))),
                    median_m=float(np.median(d)), p95_m=float(np.quantile(d, .95)))
    return dict(training=summary(training), held_out=summary(~training))


def audit(baseline, gps):
    train = gps['training']
    rotation, translation = rigid_alignment(gps['positions'][train, :2], gps['xy'][train])
    predicted = gps['positions'][:, :2] @ rotation.T + translation
    return dict(scope='Existing first-15-second SDK result; no baseline geometry modified',
                frames=len(baseline['times']), gps_samples_in_interval=len(gps['times']),
                gps_log_samples=gps['full_log_samples'], gps_log_duration_s=gps['full_log_duration_s'],
                clock=gps['clock'], baseline_GPS_scores=gps_scores(predicted, gps['xy'], train),
                lidar_path_m=float(np.linalg.norm(np.diff(baseline['positions'], axis=0), axis=1).sum()),
                gps_path_m=float(np.linalg.norm(np.diff(gps['xy'], axis=0), axis=1).sum()),
                lever_arm='Unmeasured; baseline rigid overlay assumes zero only for this diagnostic',
                absolute_GPS_coordinates_omitted=True)
