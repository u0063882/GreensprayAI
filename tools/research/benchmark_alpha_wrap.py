#!/usr/bin/env python3
"""Small, deterministic Alpha Wrap study; optional CGAL, never a production dependency."""
import argparse
from importlib.metadata import version
import json
import os
from pathlib import Path
import tempfile
import time
import numpy as np
from CGAL import CGAL_Alpha_wrap_3 as aw
from CGAL.CGAL_Kernel import Point_3
from CGAL.CGAL_Polyhedron_3 import Polyhedron_3
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components


def measure(path):
    with path.open() as stream:
        lines = iter(line for line in stream if line.strip() and not line.startswith('#'))
        assert next(lines).strip() == 'OFF'
        nv, nf, _ = map(int, next(lines).split())
        v = np.array([list(map(float, next(lines).split())) for _ in range(nv)])
        rows = [list(map(int, next(lines).split())) for _ in range(nf)]
    assert all(row[0] == 3 for row in rows)
    f = np.array([row[1:] for row in rows])
    edges = np.sort(np.concatenate([f[:, [0, 1]], f[:, [1, 2]], f[:, [2, 0]]]), axis=1)
    unique, counts = np.unique(edges, axis=0, return_counts=True)
    graph = coo_matrix((np.ones(len(unique)), (unique[:, 0], unique[:, 1])), shape=(nv, nv))
    components = connected_components(graph, directed=False, return_labels=False)
    euler = int(nv-len(unique)+nf)
    centre = v.mean(0); translated = v-centre
    volume = abs(np.einsum('ij,ij->i', translated[f[:, 0]], np.cross(translated[f[:, 1]], translated[f[:, 2]])).sum()/6)
    radius = np.linalg.norm(v, axis=1)
    return dict(vertices=nv, triangles=nf, edges_closed=bool(np.all(counts == 2)), components=int(components), euler=euler, total_genus=int(components-euler//2), volume_m3=float(volume), median_radial_error_m=float(np.median(radius-.1)), p95_abs_radial_error_m=float(np.percentile(abs(radius-.1), 95)), x_extent_m=float(np.ptp(v[:, 0])))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--mesh-dir', type=Path, required=True)
    parser.add_argument('--plot', type=Path, help='Optional scientific figure; requires matplotlib.')
    args = parser.parse_args(); args.mesh_dir.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(923)
    p = rng.normal(size=(400, 3)); p = .1*p/np.linalg.norm(p, axis=1)[:, None]
    # One hemisphere is substantially more densely sampled; same true surface.
    dense = rng.normal(size=(1600, 3)); dense[:, 2] = abs(dense[:, 2]); dense = .1*dense/np.linalg.norm(dense, axis=1)[:, None]
    datasets = {'uniform': p, 'biased_density': np.concatenate([p, dense]), 'one_outlier': np.concatenate([p, [[.25, 0, 0]]])}
    rows = []
    for name, points in datasets.items():
        for alpha, offset in [(a, d) for a in [.015, .03, .06] for d in [.003, .01]]:
            point_vector = aw.Point_3_Vector()
            for x, y, z in points: point_vector.append(Point_3(float(x), float(y), float(z)))
            mesh = Polyhedron_3(); start = time.perf_counter()
            aw.alpha_wrap_3(point_vector, alpha, offset, mesh)
            elapsed = time.perf_counter()-start
            path = args.mesh_dir/f'{name}_{alpha}_{offset}.off'; mesh.write_to_file(str(path))
            rows.append(dict(dataset=name, points=len(points), alpha_m=alpha, offset_m=offset, elapsed_seconds=elapsed, cgal_closed=mesh.is_closed(), cgal_valid=mesh.is_valid(), **measure(path)))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(dict(cgal_python_version=version('cgal'), seed=923, true_sphere_radius_m=.1, note='Analytic study only. Radial error includes deliberate offset and is not an orchard or Rhino comparison. Genus from connected-component count and Euler characteristic; no independent global intersection test.', runs=rows), indent=2)+'\n')
    if args.plot:
        os.environ.setdefault('MPLCONFIGDIR', str(Path(tempfile.gettempdir())/'greenspray-research-mpl'))
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(1, 2, figsize=(10, 4), layout='constrained')
        for name in datasets:
            selected = [r for r in rows if r['dataset'] == name and r['offset_m'] == .003]
            x = [r['alpha_m']*1000 for r in selected]
            axes[0].plot(x, [r['total_genus'] for r in selected], 'o-', label=name.replace('_', ' '))
            axes[1].plot(x, [r['x_extent_m']*1000 for r in selected], 'o-')
        axes[0].set(ylabel='Total handles (genus)', title='Closed surfaces can still have tunnels')
        axes[1].set(ylabel='Envelope X extent (mm)', title='One retained outlier expands the result')
        for axis in axes:
            axis.set_xlabel('Alpha (mm)'); axis.set_xticks([15, 30, 60]); axis.grid(alpha=.2)
        axes[0].legend(fontsize=8)
        fig.suptitle('Synthetic 100 mm-radius sphere · offset 3 mm · no outlier removal')
        args.plot.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(args.plot); plt.close(fig)
    for row in rows: print(row['dataset'], row['alpha_m'], row['offset_m'], 'components', row['components'], 'genus', row['total_genus'], 'radial_p95_mm', round(row['p95_abs_radial_error_m']*1000, 2), 'x_extent_mm', round(row['x_extent_m']*1000, 1))

if __name__ == '__main__': main()
