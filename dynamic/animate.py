"""Accumulating registered cloud and sensor trajectory, rendered locally to GIF."""
import argparse
import csv
import json
import math
import os
import tempfile
from pathlib import Path

import numpy as np

from tools.export_rhino import read_vertices


def load_results(directory):
    directory = Path(directory).expanduser().resolve()
    if not (directory / 'trajectory.csv').exists() and (directory / 'results').is_dir():
        directory = directory / 'results'
    clouds = [directory / name for name in ['registered.ply', 'pilot_registered.ply']
              if (directory / name).is_file()]
    if len(clouds) != 1:
        raise ValueError('Expected one registered.ply or pilot_registered.ply beside trajectory.csv')
    with (directory / 'trajectory.csv').open(newline='', encoding='utf-8-sig') as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError('trajectory.csv is empty')
    ids = np.array([int(row['frame_index']) for row in rows], dtype=np.int64)
    times = np.array([float(row['time_s']) for row in rows])
    positions = np.array([[float(row[key]) for key in ['x_m', 'y_m', 'z_m']] for row in rows])
    if len(np.unique(ids)) != len(ids) or not np.isfinite(times).all() or not np.isfinite(positions).all():
        raise ValueError('Trajectory needs unique frame indices and finite times/coordinates')
    if np.any(np.diff(times) <= 0):
        raise ValueError('Trajectory times must increase strictly; inspect registration flags')
    quaternion_keys = ['qx', 'qy', 'qz', 'qw']
    quaternions = None
    if all(key in rows[0] for key in quaternion_keys):
        quaternions = np.array([[float(row[key]) for key in quaternion_keys] for row in rows])
        norms = np.linalg.norm(quaternions, axis=1)
        if not np.isfinite(quaternions).all() or np.any(norms < 1e-12):
            raise ValueError('Trajectory quaternions must be finite and nonzero')
        quaternions /= norms[:, None]
    records = read_vertices(clouds[0])
    if 'scan_id' not in records.dtype.names:
        raise ValueError('Enriched PLY needs scan_id; use registered.ply, not the Rhino ASCII copy')
    order = np.argsort(ids)
    lookup = np.searchsorted(ids[order], records['scan_id'])
    if np.any(lookup >= len(ids)) or np.any(ids[order][lookup] != records['scan_id']):
        raise ValueError('PLY scan_id values do not match this trajectory.csv')
    if 'time_offset_s' in records.dtype.names:
        point_times = records['time_offset_s'].astype(float)
        time_source = 'Per-point time_offset_s in the same lidar time reference as trajectory.csv'
    else:
        point_times = times[order[lookup]]
        time_source = 'Whole-scan accumulation at scan-centre time, joined by scan_id/frame_index'
    if not np.isfinite(point_times).all():
        raise ValueError('Point timestamps must be finite')
    return dict(directory=directory, cloud=clouds[0], records=records,
                point_times=point_times, times=times, positions=positions,
                quaternions=quaternions, time_source=time_source)


def animation_clock(start, end, fps, speed):
    """Include a final complete-map frame; GIF timing is quantized to 10 ms."""
    if not all(math.isfinite(x) for x in [start, end, fps, speed]) or end < start or fps <= 0 or speed <= 0:
        raise ValueError('Invalid animation interval, fps or speed')
    duration_ms = max(10, int(round(1000 / fps / 10)) * 10)
    step = speed * duration_ms / 1000
    times = np.r_[np.arange(start, end, step), end]
    return times, duration_ms


def sensor_position(times, positions, time):
    return np.array([np.interp(time, times, positions[:, axis]) for axis in range(3)])


def trajectory_speeds(times, positions, window=1.):
    """Norm of a local least-squares XYZ velocity in capture seconds, not GIF time."""
    if not math.isfinite(window) or window <= 0:
        raise ValueError('speed-window must be positive and finite')
    speeds = np.full(len(times), np.nan)
    for i, time in enumerate(times):
        if len(times) < 2:
            continue
        lo = int(np.searchsorted(times, time - window / 2, side='left'))
        hi = int(np.searchsorted(times, time + window / 2, side='right'))
        if hi - lo < 2:
            lo, hi = (0, 2) if i == 0 else (i - 1, i + 1)
        relative = times[lo:hi] - time
        relative -= relative.mean()
        local = positions[lo:hi] - positions[i]
        velocity = (relative[:, None] * local).sum(axis=0) / np.dot(relative, relative)
        speeds[i] = np.linalg.norm(velocity)
    return speeds


def sensor_quaternion(times, quaternions, time):
    """Shortest-arc SLERP of body-to-map qx,qy,qz,qw, with endpoint clamping."""
    if time <= times[0]:
        return quaternions[0]
    if time >= times[-1]:
        return quaternions[-1]
    hi = int(np.searchsorted(times, time))
    lo = hi - 1
    fraction = (time - times[lo]) / (times[hi] - times[lo])
    a, b = quaternions[lo], quaternions[hi]
    dot = np.dot(a, b)
    if dot < 0:
        b, dot = -b, -dot
    if dot > .9995:
        q = a + fraction * (b - a)
        return q / np.linalg.norm(q)
    angle = np.arccos(np.clip(dot, -1, 1))
    return (np.sin((1 - fraction) * angle) * a + np.sin(fraction * angle) * b) / np.sin(angle)


def rotate_vector(quaternion, vector):
    xyz, w = quaternion[:3], quaternion[3]
    cross = np.cross(xyz, vector)
    return vector + 2 * (w * cross + np.cross(xyz, cross))


def initial_camera_up(quaternion, forward_body):
    """Calibrate upright once, then keep that camera rigidly attached to the body."""
    forward = rotate_vector(quaternion, forward_body)
    up = np.array([0., 0., 1.])
    if abs(np.dot(forward, up)) > .95:
        up = np.eye(3)[np.argmin(abs(forward))]
    inverse = quaternion * [-1., -1., -1., 1.]
    return rotate_vector(inverse, up)


def project_sensor_view(points, eye, quaternion, up_body, forward_body,
                        fov, aspect, near, far):
    """Perspective at the actual sensor position; return visible points far-to-near."""
    forward = rotate_vector(quaternion, forward_body)
    right = np.cross(forward, rotate_vector(quaternion, up_body))
    right /= np.linalg.norm(right)
    up = np.cross(right, forward)
    delta = points - eye
    depth = delta @ forward
    indices = np.flatnonzero((depth >= near) & (depth <= far))
    scale = np.tan(np.deg2rad(fov) / 2)
    uv = np.column_stack([delta[indices] @ right, (delta[indices] @ up) * aspect])
    uv /= (depth[indices] * scale)[:, None]
    visible = np.all(abs(uv) <= 1, axis=1)
    uv, indices = uv[visible], indices[visible]
    order = np.argsort(-depth[indices], kind='stable')
    return uv[order], indices[order], depth[indices[order]]


def gif_blocks(frames, output, palette, duration_ms, hold_ms):
    """Stream encoded frames; keep only one RGB frame in RAM at a time."""
    from PIL import Image, GifImagePlugin
    count = 0
    with Path(output).open('wb') as stream:
        for rgb, final in frames:
            image = Image.fromarray(rgb).quantize(palette=palette, dither=Image.Dither.NONE)
            if not count:
                header, _ = GifImagePlugin.getheader(image, info={'loop': 0, 'optimize': False})
                for block in header:
                    stream.write(block)
            # Complete opaque frames and disposal=2 prevent a moving sensor
            # marker from leaving ghosts. All frames use one stable palette.
            for block in GifImagePlugin.getdata(image, duration=duration_ms + (hold_ms if final else 0), disposal=2):
                stream.write(block)
            count += 1
        stream.write(b';')
    return count


def render(directory, output=None, *, fps=10., speed=2., max_points=60000,
           width=1200, height=650, view='both', elevation=22., azimuth=-60.,
           hold=2., full_bounds=False, camera='fixed', speed_window=1.,
           fov=100., near=.15, far=25., look_axis='x'):
    if not isinstance(max_points, int) or max_points < 1:
        raise ValueError('max-points must be a positive integer')
    if width < 320 or height < 240 or view not in ['both', '3d', 'top']:
        raise ValueError('Use width >= 320, height >= 240, and view both/3d/top')
    if not all(math.isfinite(x) for x in [hold, elevation, azimuth]) or not 0 <= hold <= 600:
        raise ValueError('Camera angles must be finite and hold must be between 0 and 600 seconds')
    if camera not in ['fixed', 'sensor']:
        raise ValueError('Camera must be fixed or sensor')
    if not math.isfinite(speed_window) or speed_window <= 0:
        raise ValueError('speed-window must be positive and finite')
    if not all(math.isfinite(x) for x in [fov, near, far]) or not (1 < fov < 179 and 0 < near < far):
        raise ValueError('Use 1 < fov < 179 degrees and 0 < near < far in metres')
    axes = dict(x=[1., 0., 0.], y=[0., 1., 0.], z=[0., 0., 1.],
                minus_x=[-1., 0., 0.], minus_y=[0., -1., 0.], minus_z=[0., 0., -1.])
    if look_axis not in axes:
        raise ValueError('Unknown body look axis')
    forward_body = np.array(axes[look_axis])
    data = load_results(directory)
    records = data['records']
    all_point_times = data['point_times']
    times, positions = data['times'], data['positions']
    speeds = trajectory_speeds(times, positions, speed_window)
    quaternions = data['quaternions']
    if camera == 'sensor' and quaternions is None:
        raise ValueError('Sensor-eye view requires qx,qy,qz,qw body-to-map rotations in trajectory.csv')
    up_body = initial_camera_up(quaternions[0], forward_body) if camera == 'sensor' else None
    start = min(0., float(times[0]), float(all_point_times.min()))
    end = max(float(times[-1]), float(all_point_times.max()))
    clock, duration_ms = animation_clock(start, end, fps, speed)
    hold_ms = int(round(hold * 100)) * 10
    if duration_ms + hold_ms > 655350:
        raise ValueError('Frame duration plus final hold exceeds the GIF timing limit')
    default_name = 'accumulation_follow.gif' if camera == 'sensor' else 'accumulation.gif'
    output = Path(output).expanduser().resolve() if output else data['directory'] / default_name
    if output.suffix.lower() != '.gif':
        raise ValueError('Output filename must end in .gif')
    output.parent.mkdir(parents=True, exist_ok=True)
    # Select once, then reveal the same measured points in acquisition order.
    # This sampling is for drawing only; the source PLY is never rewritten.
    chosen = np.arange(len(records)) if len(records) <= max_points else np.sort(
        np.random.default_rng(0).choice(len(records), max_points, replace=False))
    order = np.argsort(all_point_times[chosen], kind='stable')
    chosen = chosen[order]
    point_times = all_point_times[chosen]
    xyz = np.column_stack([records[key][chosen] for key in ['x', 'y', 'z']])
    rgb = np.column_stack([records[key][chosen] for key in ['red', 'green', 'blue']]) / 255.
    quantiles = [0., 1.] if full_bounds else [.005, .995]
    limits = np.quantile(np.column_stack([records[key] for key in ['x', 'y', 'z']]), quantiles, axis=0)
    low = np.minimum(limits[0], positions.min(axis=0))
    high = np.maximum(limits[1], positions.max(axis=0))
    span = np.maximum(high - low, .5)
    centre = (low + high) / 2
    low, high = centre - span * .55, centre + span * .55

    os.environ.setdefault('MPLCONFIGDIR', str(Path(tempfile.gettempdir()) / 'greenspray-gif-mpl'))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from PIL import Image
    with plt.style.context('dark_background'):
        fig = plt.figure(figsize=(width / 100, height / 100), dpi=100)
        ax3 = fig.add_subplot(121 if view == 'both' else 111, projection='3d', computed_zorder=False) if view != 'top' and camera == 'fixed' else None
        eye_ax = fig.add_subplot(121 if view == 'both' else 111) if view != 'top' and camera == 'sensor' else None
        ax2 = fig.add_subplot(122 if view == 'both' else 111) if view != '3d' else None
        if eye_ax is not None:
            eye_ax.set(xlim=(-1, 1), ylim=(-1, 1), title='Sensor-eye view')
            eye_ax.set_xticks([])
            eye_ax.set_yticks([])
        if ax3 is not None:
            ax3.view_init(elev=elevation, azim=azimuth)
            ax3.set(xlim=(low[0], high[0]), ylim=(low[1], high[1]), zlim=(low[2], high[2]),
                    xlabel='Map X (m)', ylabel='Map Y (m)', zlabel='Map Z (m)', title='Accumulating registered cloud')
            from matplotlib.ticker import MaxNLocator
            ax3.set_box_aspect(high - low, zoom=.9)
            ax3.grid(False)
            for axis in [ax3.xaxis, ax3.yaxis, ax3.zaxis]:
                axis.set_pane_color((.035, .04, .045, 1.))
                axis.set_major_locator(MaxNLocator(nbins=5))
            ax3.tick_params(labelsize=8)
            ax3.scatter(*positions[0], c='#58dc88', s=40, marker='s', depthshade=False, label='Start', zorder=5)
            trail3, = ax3.plot([], [], [], c='#ff6464', linewidth=1.8, label='Travelled path', zorder=6)
            sensor3, = ax3.plot([], [], [], 'o', c='#ff3636', markersize=8, markeredgecolor='white', markeredgewidth=.6, label='Sensor/body origin', zorder=7)
        if ax2 is not None:
            ax2.set(xlim=(low[0], high[0]), ylim=(low[1], high[1]), xlabel='Map X (m)', ylabel='Map Y (m)', title='Top view')
            ax2.set_aspect('equal', adjustable='box')
            ax2.grid(alpha=.18)
            ax2.scatter(*positions[0, :2], c='#58dc88', s=30, marker='s', label='Start', zorder=5)
            trail2, = ax2.plot([], [], c='#ff6464', linewidth=1.8, label='Travelled path', zorder=6)
            sensor2, = ax2.plot([], [], 'o', c='#ff3636', markersize=8, label='Sensor/body origin', zorder=7)
        legend_ax = ax2 if ax2 is not None else ax3
        if legend_ax is not None:
            legend_ax.legend(loc='upper right', fontsize=8)
        title = fig.suptitle('', fontsize=11 if width >= 900 else 9)
        fig.subplots_adjust(left=.045, right=.97, bottom=.08, top=.84, wspace=.28)
        clouds = []

        def frame(time):
            for artist in clouds:
                artist.remove()
            clouds.clear()
            count = np.searchsorted(point_times, time, side='right')
            q, colour = xyz[:count], rgb[:count]
            sensor = sensor_position(times, positions, time)
            if eye_ax is not None:
                quaternion = sensor_quaternion(times, quaternions, time)
                aspect = eye_ax.get_position().width * width / (eye_ax.get_position().height * height)
                uv, visible, depth = project_sensor_view(q, sensor, quaternion, up_body,
                                                        forward_body, fov, aspect, near, far)
                clouds.append(eye_ax.scatter(uv[:, 0], uv[:, 1], c=colour[visible],
                                            s=np.clip(3. / depth, .6, 5.), linewidths=0))
            if ax3 is not None:
                clouds.append(ax3.scatter(q[:, 0], q[:, 1], q[:, 2], c=colour, s=1., linewidths=0, depthshade=False, rasterized=True, zorder=2))
            if ax2 is not None:
                clouds.append(ax2.scatter(q[:, 0], q[:, 1], c=colour, s=1., linewidths=0, zorder=2))
            path = np.vstack([positions[times < time], sensor])
            if ax3 is not None:
                trail3.set_data_3d(path[:, 0], path[:, 1], path[:, 2])
                sensor3.set_data_3d([sensor[0]], [sensor[1]], [sensor[2]])
            if ax2 is not None:
                trail2.set_data(path[:, 0], path[:, 1])
                sensor2.set_data([sensor[0]], [sensor[1]])
            physical_speed = float(np.interp(time, times, speeds))
            speed_label = f'{physical_speed:.2f} m/s ({physical_speed * 3.6:.2f} km/h)' if math.isfinite(physical_speed) else 'unavailable'
            title.set_text(f'Map accumulation | t = {time - start:.2f} / {end - start:.2f} s\nSpeed (SLAM): {speed_label} | {count:,} / {len(xyz):,} samples')
            fig.canvas.draw()
            return np.asarray(fig.canvas.buffer_rgba())[:, :, :3].copy()

        # Use the complete scene to choose a fixed colour palette. Only the
        # chronological sequence below is written to the movie.
        palette = Image.fromarray(frame(end)).quantize(colors=240)
        # Preserve small overlays even when millions of green/brown pixels
        # would otherwise consume the entire palette during quantization.
        reserved = [(0, 0, 0), (255, 255, 255), (255, 54, 54), (255, 100, 100),
                    (88, 220, 136), (151, 98, 53), (48, 183, 70), (128, 128, 128),
                    (64, 64, 64), (192, 192, 192), (32, 32, 32), (224, 224, 224),
                    (9, 10, 11), (96, 96, 96), (160, 160, 160), (16, 16, 16)]
        palette.putpalette(palette.getpalette()[:720] + [value for colour in reserved for value in colour])
        fd, temporary = tempfile.mkstemp(prefix='.' + output.stem + '-', suffix='.gif', dir=output.parent)
        os.close(fd)
        try:
            def frames():
                for i, time in enumerate(clock):
                    if i % max(1, len(clock) // 10) == 0:
                        print(f'GIF frame {i + 1}/{len(clock)} | capture time {time - start:.2f} s', flush=True)
                    yield frame(time), i == len(clock) - 1
            count = gif_blocks(frames(), temporary, palette, duration_ms, hold_ms)
            os.replace(temporary, output)
        finally:
            plt.close(fig)
            if Path(temporary).exists():
                Path(temporary).unlink()
    report = dict(source_cloud=data['cloud'].name, source_trajectory='trajectory.csv',
                  source_points=len(records), sampled_points=len(chosen), frames=count,
                  pixels=[width, height], actual_fps=1000 / duration_ms, playback_speed=speed,
                  capture_time_range_s=[start, end], final_hold_s=hold,
                  encoded_duration_s=(count * duration_ms + hold_ms) / 1000,
                  accumulation_clock=data['time_source'], fixed_axes=camera == 'fixed',
                  camera=camera, top_view_fixed=True,
                  sensor_camera=dict(look_axis_body=look_axis, horizontal_fov_deg=fov, near_m=near, far_m=far,
                                     orientation='Recorded body-to-map quaternion SLERP; upright calibrated at first pose then body-fixed') if camera == 'sensor' else None,
                  estimated_speed=dict(source='SLAM trajectory, not GPS', units='m/s',
                                       method='Norm of XYZ velocity from a centered local linear fit; window clipped at recording boundaries',
                                       window_s=speed_window, independent_of_playback_speed=True,
                                       trajectory_time_s=times.tolist(),
                                       values_m_s=[float(x) if np.isfinite(x) else None for x in speeds]),
                  display_bounds='full' if full_bounds else '0.5th–99.5th point percentiles, expanded to include sensor path',
                  units='metres', source_files_unchanged=True,
                  note='Only the selected returns stored in the fused PLY are animated; this is not a replay of every raw scan return.')
    output.with_suffix('.gif.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(f'Saved {output} ({output.stat().st_size / 1e6:.1f} MB)', flush=True)
    return report


def main(argv=None, *, follow=False):
    description = 'Sensor-eye accumulating cloud and fixed top view, rendered to GIF.' if follow else __doc__
    p = argparse.ArgumentParser(description=description)
    p.add_argument('--results', type=Path, required=True, help='Run folder or its results/ subfolder')
    p.add_argument('--output', type=Path, help='GIF path; default: results/accumulation_follow.gif' if follow else 'GIF path; default: results/accumulation.gif')
    p.add_argument('--fps', type=float, default=10., help='Requested GIF frames/second; default: 10')
    p.add_argument('--speed', type=float, default=2., help='Playback speed multiplier; default: 2')
    p.add_argument('--speed-window', type=float, default=1., help='SLAM speed estimation window in capture seconds; default: 1')
    p.add_argument('--max-points', type=int, default=60000, help='Fixed display sample budget; default: 60000')
    p.add_argument('--view', choices=['both', '3d', 'top'], default='both')
    p.add_argument('--width', type=int, default=1200)
    p.add_argument('--height', type=int, default=650)
    p.add_argument('--elevation', type=float, default=22.)
    p.add_argument('--azimuth', type=float, default=-60.)
    p.add_argument('--hold', type=float, default=2., help='Extra seconds to hold completed map; default: 2')
    p.add_argument('--full-bounds', action='store_true', help='Include all spatial outliers in fixed display limits')
    if follow:
        p.add_argument('--fov', type=float, default=100., help='Sensor-eye horizontal field of view in degrees; default: 100')
        p.add_argument('--near', type=float, default=.15, help='Sensor-eye near clipping distance in metres; default: 0.15')
        p.add_argument('--far', type=float, default=25., help='Sensor-eye far clipping distance in metres; default: 25')
        p.add_argument('--look-axis', choices=['x', 'y', 'z', 'minus_x', 'minus_y', 'minus_z'], default='x', help='Camera forward axis in the recorded body frame; default: x')
    a = p.parse_args(argv)
    try:
        render(a.results, a.output, fps=a.fps, speed=a.speed, max_points=a.max_points,
               view=a.view, width=a.width, height=a.height, elevation=a.elevation,
               azimuth=a.azimuth, hold=a.hold, full_bounds=a.full_bounds,
               camera='sensor' if follow else 'fixed', speed_window=a.speed_window,
               fov=getattr(a, 'fov', 100.), near=getattr(a, 'near', .15),
               far=getattr(a, 'far', 25.), look_axis=getattr(a, 'look_axis', 'x'))
    except (ValueError, OSError, KeyError) as error:
        p.error(str(error))


if __name__ == '__main__':
    main()
