"""Optional geometry-based GIF shading; no registration or sensor-data edits."""
import math
import numpy as np


def estimate_display_normals(reference, query, radius=.1, neighbors=32, batch_size=2048):
    """Local PCA using full-resolution support, with confidence for surface shape.

    Radius bounds the physical footprint; k bounds memory/density influence.
    Sparse, line-like or volumetric neighborhoods receive little/no shading.
    """
    if not math.isfinite(radius) or radius <= 0 or not isinstance(neighbors, (int, np.integer)) or not 6 <= neighbors <= 256:
        raise ValueError('Use normal-radius > 0 and 6 <= normal-neighbors <= 256')
    if batch_size < 1:
        raise ValueError('Normal batch size must be positive')
    reference = np.asarray(reference, dtype=float)
    query = np.asarray(query, dtype=float)
    if reference.ndim != 2 or reference.shape[1] != 3 or query.ndim != 2 or query.shape[1] != 3:
        raise ValueError('Normal estimation requires N x 3 coordinates')
    if not np.isfinite(reference).all() or not np.isfinite(query).all():
        raise ValueError('Normal coordinates must be finite')
    normals = np.zeros_like(query)
    confidence = np.zeros(len(query))
    if len(reference) < 6:
        return normals, confidence
    from scipy.spatial import cKDTree
    tree = cKDTree(reference)
    k = min(neighbors, len(reference))
    for lo in range(0, len(query), batch_size):
        q = query[lo:lo + batch_size]
        distances, indices = tree.query(q, k=k, distance_upper_bound=radius, workers=2)
        valid = np.isfinite(distances)
        points = reference[np.where(valid, indices, 0)] - q[:, None, :]
        points = np.where(valid[:, :, None], points, 0.)
        count = valid.sum(axis=1)
        divisor = np.maximum(count, 1)
        mean = points.sum(axis=1) / divisor[:, None]
        covariance = np.einsum('bki,bkj->bij', points, points) / divisor[:, None, None]
        covariance -= mean[:, :, None] * mean[:, None, :]
        eigenvalues, eigenvectors = np.linalg.eigh(covariance)
        eigenvalues = np.maximum(eigenvalues, 0.)
        planarity = (eigenvalues[:, 1] - eigenvalues[:, 0]) / np.maximum(eigenvalues[:, 2], 1e-15)
        quality = np.clip((planarity - .05) / .45, 0., 1.)
        quality[(count < 6) | (eigenvalues[:, 2] < 1e-12)] = 0.
        normals[lo:lo + len(q)] = eigenvectors[:, :, 0]
        confidence[lo:lo + len(q)] = quality
    return normals, confidence


def diffuse_factors(normals, confidence, ambient=.6, azimuth=315., elevation=50.):
    """Fixed world light and two-sided diffuse shading, independent of normal sign."""
    if not all(math.isfinite(x) for x in [ambient, azimuth, elevation]) or not 0 <= ambient <= 1:
        raise ValueError('Light angles must be finite and ambient must be between 0 and 1')
    az, el = np.deg2rad([azimuth, elevation])
    light = np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
    normals = np.asarray(normals, dtype=float)
    confidence = np.asarray(confidence, dtype=float)
    lengths = np.linalg.norm(normals, axis=1)
    unit = np.divide(normals, lengths[:, None], out=np.zeros_like(normals), where=lengths[:, None] > 0)
    supported = np.where(lengths > 0, np.clip(confidence, 0., 1.), 0.)
    diffuse = np.clip(abs(unit @ light), 0., 1.)
    illumination = ambient + (1. - ambient) * diffuse
    return 1. - supported * (1. - illumination)
