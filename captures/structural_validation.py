"""Full-height, same-face checks for a proposed rigid registration correction.

This is a validator, not a shape prior or a registration/meshing dependency.
Matches are chosen in baseline coordinates and frozen for both evaluations.
Opposite-facing surfaces are never treated as coincident bark. A sparse match
set is reported as unobservable, rather than evidence that a pose is correct.
"""
import numpy as np
from scipy.spatial import cKDTree

DEFAULTS = dict(minimum_height_m=.55, maximum_height_m=3.6,
                minimum_wood_score=.65, voxel_m=.025, normal_radius_m=.085,
                maximum_distance_m=.06, minimum_normal_dot=.85,
                holdout_cell_m=.12, height_band_m=.5, maximum_per_pair=400,
                minimum_pair_points=8, minimum_cross_points=100,
                minimum_height_span_m=.75, minimum_height_bands=3,
                rms_tolerance_m=.001, median_tolerance_m=.001,
                euclidean_tolerance_m=.002, band_tolerance_m=.002)


def _holdout(p, size):
    c = np.floor(p / size).astype(np.int64)
    return (c[:, 0] + 2*c[:, 1] + 3*c[:, 2]) % 4 == 0


def _sample(p, voxel):
    return np.unique(np.floor(p/voxel).astype(np.int64), axis=0,
                     return_index=True)[1]


def _normals(p, origin, radius):
    import open3d as o3d
    pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(p))
    pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius,
                                                          max_nn=30))
    pc.orient_normals_towards_camera_location(origin)
    return np.asarray(pc.normals).copy()


def structural_pairs(clouds, wood_scores, origins, labels, settings=None):
    """Select disjoint validation returns before constructing correspondences.

    The same spatial split must be excluded at BOTH ends of any fitting pairs
    used by a caller. Normal estimation here uses validation returns only.
    This selection does not depend on proposed corrections or tree symmetry.
    """
    opt = {**DEFAULTS, **(settings or {})}
    if not (len(clouds) == len(wood_scores) == len(origins) == len(labels)):
        raise ValueError('One score array, origin and label per recording required')
    arrays, normals, trees = [], [], []
    for p, score, origin in zip(clouds, wood_scores, origins):
        p = np.asarray(p, dtype=float); score = np.asarray(score)
        if p.ndim != 2 or p.shape[1] != 3 or len(score) != len(p):
            raise ValueError('Expected Nx3 points and N wood scores')
        keep = (np.isfinite(p).all(axis=1) & np.isfinite(score) &
                (p[:, 2] > opt['minimum_height_m']) &
                (p[:, 2] < opt['maximum_height_m']) &
                (score > opt['minimum_wood_score']) &
                _holdout(p, opt['holdout_cell_m']))
        p = p[keep]; p = p[_sample(p, opt['voxel_m'])]
        n = (_normals(p, np.asarray(origin), opt['normal_radius_m'])
             if len(p) >= 30 else np.zeros_like(p))
        arrays.append(p); normals.append(n); trees.append(cKDTree(p))
    pairs = []
    for i, p in enumerate(arrays):
        if len(p) < opt['minimum_pair_points']: continue
        for j in range(i+1, len(arrays)):
            q = arrays[j]
            if len(q) < opt['minimum_pair_points']: continue
            distance, target = trees[j].query(p)
            back = trees[i].query(q[target])[1]
            dot = np.sum(normals[i]*normals[j][target], axis=1)
            ix = np.flatnonzero((distance < opt['maximum_distance_m']) &
                                (dot > opt['minimum_normal_dot']) &
                                (back == np.arange(len(p))))
            if len(ix) < opt['minimum_pair_points']: continue
            if len(ix) > opt['maximum_per_pair']:
                ix = ix[np.linspace(0, len(ix)-1,
                                    opt['maximum_per_pair'], dtype=int)]
            pairs.append(dict(i=i, j=j, cross_side=labels[i] != labels[j],
                              p=p[ix], q=q[target[ix]], n=normals[j][target[ix]]))
    return pairs


def _stats(plane, distances):
    a = np.asarray(plane); d = np.asarray(distances)
    if not len(a): return None
    return dict(points=len(a), median_absolute_m=float(np.median(abs(a))),
                rmse_m=float(np.sqrt(np.mean(a*a))),
                euclidean_rmse_m=float(np.sqrt(np.mean(d*d))))


def evaluate_pairs(pairs, corrections, settings=None):
    """Evaluate fixed pair identities; preserve real asymmetry and thickness."""
    opt = {**DEFAULTS, **(settings or {})}; C = np.asarray(corrections, dtype=float)
    if C.ndim != 3 or C.shape[1:] != (4, 4) or not np.isfinite(C).all():
        raise ValueError('Expected finite Nx4x4 correction matrices')
    R = C[:, :3, :3]
    if (not np.allclose(R.transpose(0, 2, 1) @ R, np.eye(3), atol=1e-8) or
        not np.allclose(np.linalg.det(R), 1, atol=1e-8) or
        not np.allclose(C[:, 3, :], [0, 0, 0, 1], atol=1e-8)):
        raise ValueError('Corrections must be rigid; scaling/deformation forbidden')
    groups = {}; heights = []
    for pair in pairs:
        i, j = pair['i'], pair['j']; p, q, n = [pair[k] for k in ['p','q','n']]
        if max(i,j) >= len(C): raise ValueError('Missing recording correction')
        after = p @ R[i].T + C[i,:3,3] - q @ R[j].T - C[j,:3,3]
        before = p-q; rn = n @ R[j].T
        bands = np.floor(p[:,2]/opt['height_band_m']).astype(int)
        cross = bool(pair['cross_side'])
        if cross: heights.extend(p[:,2].tolist())
        for key, keep in [('cross_side' if cross else 'same_side', np.ones(len(p),bool))] + (
            [('cross_band_'+str(b), bands == b) for b in np.unique(bands)] if cross else []):
            record = groups.setdefault(key,[[],[],[],[]])
            for dest, value in zip(record,[np.sum(before*n,axis=1),
                    np.sum(after*rn,axis=1),np.linalg.norm(before,axis=1),
                    np.linalg.norm(after,axis=1)]): dest.extend(value[keep].tolist())
    checks = {key:dict(before=_stats(v[0],v[2]),after=_stats(v[1],v[3]))
              for key,v in groups.items()}
    cross = checks.get('cross_side',{}).get('before'); bands = [k for k,v in checks.items()
        if k.startswith('cross_band_') and v['before']['points'] >= 30]
    coverage = dict(cross_side_points=cross['points'] if cross else 0,
                    cross_side_height_span_m=float(np.ptp(heights)) if heights else 0.,
                    supported_height_bands=len(bands))
    support = []
    if coverage['cross_side_points'] < opt['minimum_cross_points']:
        support.append('Too few shared, same-face cross-side wood observations')
    if coverage['cross_side_height_span_m'] < opt['minimum_height_span_m']:
        support.append('Insufficient height lever arm to check orientation')
    if len(bands) < opt['minimum_height_bands']:
        support.append('Insufficient supported height bands')
    degradation = []
    for key, check in checks.items():
        a,b = check['before'],check['after']
        if key.startswith('cross_band_'):
            if a['points'] >= 30 and b['rmse_m'] > a['rmse_m']+opt['band_tolerance_m']:
                degradation.append(key+': RMS worsens')
            continue
        for metric,tolerance in [('rmse_m','rms_tolerance_m'),
                ('median_absolute_m','median_tolerance_m'),
                ('euclidean_rmse_m','euclidean_tolerance_m')]:
            if b[metric] > a[metric]+opt[tolerance]:
                degradation.append(key+': '+metric+' worsens')
    return dict(method=__doc__.strip(),settings=opt,coverage=coverage,checks=checks,
                observable=not support,validation_passed=not(support+degradation),
                support_reasons=support,degradation_reasons=degradation,
                accuracy_limit='Conditional internal consistency, not survey accuracy. '
                'Absence of matching returns does not prove a pose error or a correct pose. '
                'The green/brown classifier is provisional. No branch symmetry, bar '
                'levelness, vertical trunk or planar terrain is imposed.')


def validate_structure(clouds, wood_scores, origins, labels, corrections, settings=None):
    return evaluate_pairs(structural_pairs(clouds, wood_scores, origins, labels,
                                          settings), corrections, settings)
