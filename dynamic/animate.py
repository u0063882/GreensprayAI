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
                point_times=point_times, times=times, positions=positions, time_source=time_source)


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
           hold=2., full_bounds=False):
    if not isinstance(max_points, int) or max_points < 1:
        raise ValueError('max-points must be a positive integer')
    if width < 320 or height < 240 or view not in ['both', '3d', 'top']:
        raise ValueError('Use width >= 320, height >= 240, and view both/3d/top')
    if not all(math.isfinite(x) for x in [hold, elevation, azimuth]) or not 0 <= hold <= 600:
        raise ValueError('Camera angles must be finite and hold must be between 0 and 600 seconds')
    data = load_results(directory)
    records = data['records']
    all_point_times = data['point_times']
    times, positions = data['times'], data['positions']
    start = min(0., float(times[0]), float(all_point_times.min()))
    end = max(float(times[-1]), float(all_point_times.max()))
    clock, duration_ms = animation_clock(start, end, fps, speed)
    hold_ms = int(round(hold * 100)) * 10
    if duration_ms + hold_ms > 655350:
        raise ValueError('Frame duration plus final hold exceeds the GIF timing limit')
    output = Path(output).expanduser().resolve() if output else data['directory'] / 'accumulation.gif'
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
        ax3 = fig.add_subplot(121 if view == 'both' else 111, projection='3d', computed_zorder=False) if view != 'top' else None
        ax2 = fig.add_subplot(122 if view == 'both' else 111) if view != '3d' else None
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
        legend_ax.legend(loc='upper right', fontsize=8)
        title = fig.suptitle('', fontsize=13)
        bounds_label = 'full cloud bounds' if full_bounds else 'central 99% display bounds'
        fig.text(.5, .025, f'Original saved point colours | fixed view, {bounds_label} | {speed:g}x playback | display sample only', ha='center', fontsize=8)
        fig.subplots_adjust(left=.045, right=.97, bottom=.12, top=.84, wspace=.28)
        clouds = []

        def frame(time):
            for artist in clouds:
                artist.remove()
            clouds.clear()
            count = np.searchsorted(point_times, time, side='right')
            q, colour = xyz[:count], rgb[:count]
            if ax3 is not None:
                clouds.append(ax3.scatter(q[:, 0], q[:, 1], q[:, 2], c=colour, s=1., linewidths=0, depthshade=False, rasterized=True, zorder=2))
            if ax2 is not None:
                clouds.append(ax2.scatter(q[:, 0], q[:, 1], c=colour, s=1., linewidths=0, zorder=2))
            sensor = sensor_position(times, positions, time)
            path = np.vstack([positions[times < time], sensor])
            if ax3 is not None:
                trail3.set_data_3d(path[:, 0], path[:, 1], path[:, 2])
                sensor3.set_data_3d([sensor[0]], [sensor[1]], [sensor[2]])
            if ax2 is not None:
                trail2.set_data(path[:, 0], path[:, 1])
                sensor2.set_data([sensor[0]], [sensor[1]])
            title.set_text(f'Registered map accumulation | t = {time - start:.2f} / {end - start:.2f} s\n{count:,} / {len(xyz):,} sampled returns accumulated')
            fig.canvas.draw()
            return np.asarray(fig.canvas.buffer_rgba())[:, :, :3].copy()

        # Use the complete scene to choose a fixed colour palette. Only the
        # chronological sequence below is written to the movie.
        palette = Image.fromarray(frame(end)).quantize(colors=256)
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
                  accumulation_clock=data['time_source'], fixed_axes=True,
                  display_bounds='full' if full_bounds else '0.5th–99.5th point percentiles, expanded to include sensor path',
                  units='metres', source_files_unchanged=True,
                  note='Only the selected returns stored in the fused PLY are animated; this is not a replay of every raw scan return.')
    output.with_suffix('.gif.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(f'Saved {output} ({output.stat().st_size / 1e6:.1f} MB)', flush=True)
    return report


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--results', type=Path, required=True, help='Run folder or its results/ subfolder')
    p.add_argument('--output', type=Path, help='GIF path; default: results/accumulation.gif')
    p.add_argument('--fps', type=float, default=10., help='Requested GIF frames/second; default: 10')
    p.add_argument('--speed', type=float, default=2., help='Playback speed multiplier; default: 2')
    p.add_argument('--max-points', type=int, default=60000, help='Fixed display sample budget; default: 60000')
    p.add_argument('--view', choices=['both', '3d', 'top'], default='both')
    p.add_argument('--width', type=int, default=1200)
    p.add_argument('--height', type=int, default=650)
    p.add_argument('--elevation', type=float, default=22.)
    p.add_argument('--azimuth', type=float, default=-60.)
    p.add_argument('--hold', type=float, default=2., help='Extra seconds to hold completed map; default: 2')
    p.add_argument('--full-bounds', action='store_true', help='Include all spatial outliers in fixed display limits')
    a = p.parse_args(argv)
    try:
        render(a.results, a.output, fps=a.fps, speed=a.speed, max_points=a.max_points,
               view=a.view, width=a.width, height=a.height, elevation=a.elevation,
               azimuth=a.azimuth, hold=a.hold, full_bounds=a.full_bounds)
    except (ValueError, OSError, KeyError) as error:
        p.error(str(error))


if __name__ == '__main__':
    main()
