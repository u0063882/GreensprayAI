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
    report_name = "report.json" if (directory / "report.json").exists() else "pilot_report.json"
    report = json.loads((directory / report_name).read_text())
    with np.load(directory / "per_column_poses.npz", allow_pickle=False) as f:
        arrays = {key: f[key] for key in f.files}
    poses = arrays["body_to_world"]
    status = arrays["status"]
    if (
        poses.ndim != 4
        or poses.shape[2:] != (4, 4)
        or len(poses) < 3
        or not np.isfinite(poses).all()
    ):
        raise ValueError("Need at least three finite per-column poses")
    centre_columns = np.array(
        [np.flatnonzero(s & 1)[len(np.flatnonzero(s & 1)) // 2] for s in status]
    )
    centres = poses[np.arange(len(poses)), centre_columns]
    capture_start = report["source"]["capture_first_unix"]
    capture = arrays["host_unix"] - capture_start
    if not np.isfinite(capture).all() or np.any(np.diff(capture) <= 0):
        raise ValueError("Capture times must be finite and increasing")
    return dict(
        directory=directory,
        report=report,
        arrays=arrays,
        centres=centres,
        positions=centres[:, :3, 3],
        times=capture,
        capture_start=capture_start,
        rotations=Rotation.from_matrix(centres[:, :3, :3]),
    )


def interpolate_poses(baseline, times):
    times = np.clip(np.asarray(times), baseline["times"][0], baseline["times"][-1])
    positions = np.column_stack(
        [np.interp(times, baseline["times"], baseline["positions"][:, i]) for i in range(3)]
    )
    rotations = Slerp(baseline["times"], baseline["rotations"])(times).as_matrix()
    return positions, rotations


def load_gps(path, baseline, clock="shared-start"):
    import h5py

    with h5py.File(path) as f:
        t, lat, lon = [np.asarray(f[key]).ravel() for key in ["time", "lat", "lon"]]
    if not len(t) == len(lat) == len(lon):
        raise ValueError("GPS array lengths differ")
    keep = np.isfinite(t) & np.isfinite(lat) & np.isfinite(lon)
    t, lat, lon = t[keep], lat[keep], lon[keep]
    order = np.argsort(t)
    t, lat, lon = t[order], lat[order], lon[order]
    t, unique = np.unique(t, return_index=True)
    lat, lon = lat[unique], lon[unique]
    if len(t) < 5:
        raise ValueError("Insufficient GPS samples")
    relative = t - t[0] if clock == "shared-start" else t - baseline["capture_start"]
    if clock not in ["shared-start", "absolute"]:
        raise ValueError("Unknown GPS clock policy")
    phi = np.deg2rad(lat.mean())
    north_factor = 111132.92 - 559.82 * np.cos(2 * phi) + 1.175 * np.cos(4 * phi)
    east_factor = 111412.84 * np.cos(phi) - 93.5 * np.cos(3 * phi)
    local = np.column_stack([(lon - lon[0]) * east_factor, (lat - lat[0]) * north_factor])
    overlap = (relative >= baseline["times"][0]) & (relative <= baseline["times"][-1])
    times, xy = relative[overlap], local[overlap]
    if len(times) < 10:
        raise ValueError("Insufficient GPS time coverage; check clock alignment")
    # Whole one-second blocks: 60% training, 20% model selection, 20% final
    # test. Raw fixes are never interpolated between these measurement sets.
    role = np.floor(times).astype(int) % 5
    training, validation, testing = role < 3, role == 3, role == 4
    if min(training.sum(), validation.sum(), testing.sum()) < 4:
        raise ValueError("Need GPS coverage for training, validation and final-test blocks")
    positions, rotations = interpolate_poses(baseline, times)
    return dict(
        times=times,
        xy=xy,
        training=training,
        validation=validation,
        testing=testing,
        positions=positions,
        rotations=rotations,
        clock=clock,
        full_log_duration_s=float(t[-1] - t[0]),
        full_log_samples=len(t),
    )


def yaw_matrix(yaw):
    yaw = np.asarray(yaw)
    c, s = np.cos(yaw), np.sin(yaw)
    matrices = np.zeros(yaw.shape + (3, 3))
    matrices[..., 0, 0] = c
    matrices[..., 0, 1] = -s
    matrices[..., 1, 0] = s
    matrices[..., 1, 1] = c
    matrices[..., 2, 2] = 1.0
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


def gps_scores(predicted, measured, training, validation=None, testing=None):
    error = np.linalg.norm(predicted - measured, axis=1)

    def summary(keep):
        d = error[keep]
        return dict(
            samples=len(d),
            RMSE_m=float(np.sqrt(np.mean(d**2))),
            median_m=float(np.median(d)),
            p95_m=float(np.quantile(d, 0.95)),
        )

    report = dict(training=summary(training), held_out=summary(~training))
    if validation is not None:
        report["validation"] = summary(validation)
        report["final_test"] = summary(testing)
    return report


def robust_gps_residual(error):
    """Isotropic pseudo-Huber residual; other priors remain ordinary quadratic."""
    norm2 = np.sum(error**2, axis=1)
    weight = np.sqrt(2.0 / (np.sqrt(1.0 + norm2) + 1.0))
    return (error * weight[:, None]).ravel()


def fit_calibration(gps, sigma=0.15, lever_sigma=1.0):
    """Fit only yaw/origin/uncertain antenna lever, keeping SDK poses fixed."""
    from scipy.optimize import least_squares

    if not (np.isfinite(sigma) and sigma > 0 and np.isfinite(lever_sigma) and lever_sigma > 0):
        raise ValueError("GPS and antenna uncertainty scales must be positive and finite")

    keep = gps["training"]
    rotation, translation = rigid_alignment(gps["positions"][keep, :2], gps["xy"][keep])
    yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
    start = np.r_[yaw, translation, np.zeros(3)]

    def residual(x):
        antenna = gps["positions"][keep] + gps["rotations"][keep] @ x[3:6]
        predicted = antenna[:, :2] @ yaw_matrix(x[0])[:2, :2].T + x[1:3]
        return np.r_[
            robust_gps_residual((predicted - gps["xy"][keep]) / sigma), x[3:6] / lever_sigma
        ]

    lower = np.r_[yaw - np.pi, [-np.inf] * 2, [-3.0] * 3]
    upper = np.r_[yaw + np.pi, [np.inf] * 2, [3.0] * 3]
    solution = least_squares(
        residual, start, bounds=(lower, upper), max_nfev=200, xtol=1e-9, ftol=1e-9, gtol=1e-9
    )
    return solution.x, dict(success=bool(solution.success), cost=float(solution.cost))


def controls_at(model, times):
    return np.column_stack(
        [np.interp(times, model["knots"], model["controls"][:, i]) for i in range(3)]
    )


def refined_gps_prediction(model, gps):
    controls = controls_at(model, gps["times"])
    rotation = yaw_matrix(controls[:, 2]) @ gps["rotations"]
    positions = gps["positions"].copy()
    positions[:, :2] += controls[:, :2]
    x = model["calibration"]
    antenna = positions + rotation @ x[3:6]
    return antenna[:, :2] @ yaw_matrix(x[0])[:2, :2].T + x[1:3]


def model_scores(model, gps):
    return gps_scores(
        refined_gps_prediction(model, gps),
        gps["xy"],
        gps["training"],
        gps["validation"],
        gps["testing"],
    )


def fit_refinement(baseline, gps, sigma=0.15, lever_sigma=1.0, knot_seconds=1.0):
    """GPS + SDK relative-motion factors, with slow XY/yaw correction knots.

    First correction is fixed to zero, retaining the baseline local gauge.
    Z/roll/pitch are preserved. The lever arm is a bounded nuisance with a
    broad prior; its fitted value is not a surveyed calibration.
    """
    from scipy.optimize import least_squares

    calibration, calibration_status = fit_calibration(gps, sigma, lever_sigma)
    t = baseline["times"]
    count = max(2, int(np.ceil((t[-1] - t[0]) / knot_seconds)) + 1)
    knots = np.linspace(t[0], t[-1], count)
    size = 3 * (count - 1)

    def decode(x):
        return dict(
            knots=knots,
            controls=np.vstack([np.zeros(3), x[:size].reshape(-1, 3)]),
            calibration=x[size:],
        )

    start = np.r_[np.zeros(size), calibration]
    limits = np.tile([1.0, 1.0, np.deg2rad(5.0)], count - 1)
    lower = np.r_[-limits, calibration[0] - np.pi, [-np.inf] * 2, [-3.0] * 3]
    upper = np.r_[limits, calibration[0] + np.pi, [np.inf] * 2, [3.0] * 3]
    base_rotation = baseline["centres"][:, :3, :3]
    base_delta = np.diff(baseline["positions"], axis=0)
    base_local_edges = np.einsum("nji,nj->ni", base_rotation[:-1], base_delta)
    keep = gps["training"]

    def residual(x):
        model = decode(x)
        controls = controls_at(model, t)
        positions = baseline["positions"].copy()
        positions[:, :2] += controls[:, :2]
        rotations = yaw_matrix(controls[:, 2]) @ base_rotation
        edges = np.einsum("nji,nj->ni", rotations[:-1], np.diff(positions, axis=0))
        # Local SDK motion is a soft prior, not treated as error-free truth.
        odometry = (edges - base_local_edges).ravel() / 0.02
        relative_yaw = np.diff(controls[:, 2]) / np.deg2rad(0.2)
        acceleration = np.diff(model["controls"], n=2, axis=0) / (knots[1] - knots[0]) ** 2
        smoothness = (acceleration / [0.05, 0.05, np.deg2rad(0.3)]).ravel()
        correction_prior = (model["controls"][1:] / [0.5, 0.5, np.deg2rad(2.0)]).ravel()
        error = refined_gps_prediction(model, gps)[keep] - gps["xy"][keep]
        return np.r_[
            robust_gps_residual(error / sigma),
            odometry,
            relative_yaw,
            smoothness,
            correction_prior,
            model["calibration"][3:6] / lever_sigma,
        ]

    solution = least_squares(
        residual, start, bounds=(lower, upper), max_nfev=200, xtol=1e-8, ftol=1e-8, gtol=1e-8
    )
    candidate = decode(solution.x)
    candidate.update(
        sigma=sigma,
        lever_sigma=lever_sigma,
        success=bool(solution.success),
        evaluations=int(solution.nfev),
        optimizer_cost=float(solution.cost),
    )
    base_model = dict(knots=knots, controls=np.zeros((count, 3)), calibration=calibration)
    return candidate, base_model


def apply_correction(points, origins, times, model):
    controls = controls_at(model, times)
    rotated = np.einsum("nij,nj->ni", yaw_matrix(controls[:, 2]), points - origins) + origins
    rotated[:, :2] += controls[:, :2]
    rotated[:, 2] = points[
        :, 2
    ]  # Preserve height exactly, including floating-point representation.
    return rotated


def corrected_column_poses(baseline, model):
    arrays = baseline["arrays"]
    corrected = arrays["body_to_world"].copy()
    for i, times in enumerate(column_capture_times(baseline)):
        control = controls_at(model, times)
        corrected[i, :, :3, :3] = yaw_matrix(control[:, 2]) @ corrected[i, :, :3, :3]
        corrected[i, :, :2, 3] += control[:, :2]
    return corrected


def column_capture_times(baseline):
    arrays = baseline["arrays"]
    times = []
    for i, (ts, status) in enumerate(zip(arrays["column_timestamp_ns"], arrays["status"])):
        columns = np.flatnonzero(status & 1)
        centre_ns = np.median(ts[columns])
        times.append(baseline["times"][i] + (ts.astype(float) - centre_ns) * 1e-9)
    return np.asarray(times)


def heldout_features(baseline):
    """Read original restored OSF returns; split SDK-withheld pixels for QC."""
    from ouster.sdk import core, open_source
    from dynamic.pilot import holdout_mask, measured_indices

    path = baseline["directory"] / "registered.osf"
    if not path.exists():
        path = baseline["directory"] / "pilot15s.osf"
    source = open_source(str(path))
    lut = core.XYZLut(source.sensor_info[0], True)
    result = []
    captures = column_capture_times(baseline)
    try:
        for i, frames in enumerate(source):
            scan = frames[0]
            if i >= len(baseline["times"]) or int(scan.frame_id) != int(
                baseline["arrays"]["frame_id"][i]
            ):
                raise ValueError("OSF and per-column trajectory do not match")
            raw = scan.field("RANGE")
            rows, cols = np.indices(raw.shape)
            held = holdout_mask(*raw.shape)
            valid = (raw >= 1000) & (raw < 8000) & ((scan.status[None, :] & 1) > 0)
            points = core.dewarp(lut(scan), baseline["arrays"]["body_to_world"][i])
            item = dict(time=float(baseline["times"][i]))
            for name, part in [
                ("validation", (rows + cols) % 2 == 0),
                ("test", (rows + cols) % 2 == 1),
            ]:
                indices = np.flatnonzero(valid & held & part)
                selected = measured_indices(points.reshape(-1, 3)[indices], 0.06)
                if len(selected) > 1200:
                    selected = selected[np.linspace(0, len(selected) - 1, 1200).astype(int)]
                indices = indices[selected]
                columns = indices % scan.w
                item[name] = dict(
                    points=points.reshape(-1, 3)[indices],
                    origins=baseline["arrays"]["body_to_world"][i, columns, :3, 3],
                    times=captures[i, columns],
                )
            result.append(item)
    finally:
        source.close()
    if len(result) != len(baseline["times"]):
        raise ValueError("Incomplete OSF validation coverage")
    return result


def lidar_scores(features, part="validation", model=None):
    from scipy.spatial import cKDTree

    points = []
    for item in features:
        data = item[part]
        points.append(
            data["points"]
            if model is None
            else apply_correction(data["points"], data["origins"], data["times"], model)
        )
    pairs = []
    for lag in [5, 20]:
        for i in range(lag, len(points)):
            a, b = points[i], points[i - lag]
            if min(len(a), len(b)) < 30:
                continue
            distances = np.r_[cKDTree(a).query(b, workers=2)[0], cKDTree(b).query(a, workers=2)[0]]
            pairs.append(
                dict(
                    frame_index=i,
                    time_s=features[i]["time"],
                    lag=lag,
                    capped_RMS_m=float(np.sqrt(np.mean(np.minimum(distances, 0.1) ** 2))),
                    coverage_10cm=float(np.mean(distances < 0.1)),
                )
            )
    if not pairs:
        raise ValueError("No supported held-out LiDAR overlap pairs")
    return dict(
        mean_capped_RMS_m=float(np.mean([x["capped_RMS_m"] for x in pairs])),
        mean_coverage_10cm=float(np.mean([x["coverage_10cm"] for x in pairs])),
        pairs=pairs,
    )


def paired_lidar_change(before, after):
    groups = {}
    for a, b in zip(before["pairs"], after["pairs"]):
        if (a["frame_index"], a["lag"]) != (b["frame_index"], b["lag"]):
            raise ValueError("LiDAR comparison pairs changed")
        groups.setdefault(int(a["time_s"]), []).append(a["capped_RMS_m"] - b["capped_RMS_m"])
    blocks = np.array([np.mean(v) for v in groups.values()])
    bootstrap = np.random.default_rng(0).choice(blocks, (500, len(blocks)), replace=True).mean(1)
    return dict(
        mean_improvement_m=float(blocks.mean()),
        block_bootstrap_95pct_m=np.quantile(bootstrap, [0.025, 0.975]).tolist(),
        time_blocks=len(blocks),
        positive_means="lower capped nearest-neighbor cost",
    )


def export_candidate(baseline, model, output, status, write_osf=True, rhino=True):
    """Apply the same rigid correction to stored points and full column poses."""
    import copy, csv
    from tools.export_rhino import read_vertices
    from captures.leafwood import write_records

    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    source = baseline["directory"] / "registered.ply"
    if not source.exists():
        source = baseline["directory"] / "pilot_registered.ply"
    records = read_vertices(source)
    for field in ["scan_id", "pixel_index"]:
        if field not in records.dtype.names:
            raise ValueError("Source provenance missing: " + field)
    ids = records["scan_id"].astype(int)
    width = baseline["arrays"]["body_to_world"].shape[1]
    cols = records["pixel_index"].astype(int) % width
    if ids.max() >= len(baseline["times"]):
        raise ValueError("PLY scan IDs differ from saved poses")
    captures = column_capture_times(baseline)
    old_xyz = np.c_[records["x"], records["y"], records["z"]]
    origins = baseline["arrays"]["body_to_world"][ids, cols, :3, 3]
    new_xyz = apply_correction(old_xyz, origins, captures[ids, cols], model)
    for j, key in enumerate(["x", "y", "z"]):
        records[key] = new_xyz[:, j]
    write_records(
        output / "registered.ply",
        records,
        comment="Experimental GPS/relative-motion refinement; original channels and IDs retained; status="
        + status,
    )
    arrays = dict(baseline["arrays"])
    arrays["body_to_world"] = corrected_column_poses(baseline, model)
    np.savez_compressed(output / "per_column_poses.npz", **arrays)
    columns = np.array(
        [np.flatnonzero(s & 1)[len(np.flatnonzero(s & 1)) // 2] for s in arrays["status"]]
    )
    centres = arrays["body_to_world"][np.arange(len(columns)), columns]
    with (output / "trajectory.csv").open("w", newline="") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(
            [
                "frame_index",
                "frame_id",
                "time_s",
                "capture_host_unix",
                "x_m",
                "y_m",
                "z_m",
                "qx",
                "qy",
                "qz",
                "qw",
            ]
        )
        for i, pose in enumerate(centres):
            writer.writerow(
                [
                    i,
                    int(arrays["frame_id"][i]),
                    arrays["relative_seconds"][i],
                    arrays["host_unix"][i],
                    *pose[:3, 3],
                    *Rotation.from_matrix(pose[:3, :3]).as_quat(),
                ]
            )
    transformations = dict(
        units="metres",
        status=status,
        pose_source="SDK plus experimental GPS correction",
        convention="Column-vector body-to-map matrices; registered.ply already transformed",
        column_poses="per_column_poses.npz",
        frames=[
            dict(
                scan_id=i,
                frame_id=int(arrays["frame_id"][i]),
                time_s=float(arrays["relative_seconds"][i]),
                body_to_world=p.tolist(),
            )
            for i, p in enumerate(centres)
        ],
    )
    (output / "transformations.json").write_text(json.dumps(transformations, indent=2) + "\n")
    report = copy.deepcopy(baseline["report"])
    for key in [
        "preview_sample",
        "preview_trajectory",
        "preview_time_s",
        "rhino_export",
        "publication",
    ]:
        report.pop(key, None)
    report["status"] = status
    report["pose_source"] = "SDK + horizontal GPS/relative-motion refinement; not a new SDK run"
    report["GPS_refinement"] = dict(
        knots_capture_seconds=model["knots"].tolist(),
        XY_m_and_yaw_rad=model["controls"].tolist(),
        uncertain_antenna_offset_body_m=model["calibration"][3:6].tolist(),
        antenna_offset_surveyed=False,
        gps_sigma_m=model["sigma"],
    )
    old_diagnostics = {
        key: report.pop(key)
        for key in ["GNSS_diagnostic", "held_out_consistency", "ground_diagnostic"]
        if key in report
    }
    report["baseline_diagnostics_only"] = old_diagnostics
    position = centres[:, :3, 3]
    steps = np.linalg.norm(np.diff(position, axis=0), axis=1)
    angles = np.rad2deg(
        np.linalg.norm(
            Rotation.from_matrix(
                centres[:-1, :3, :3].transpose(0, 2, 1) @ centres[1:, :3, :3]
            ).as_rotvec(),
            axis=1,
        )
    )
    report["trajectory"] = dict(
        path_length_m=float(steps.sum()),
        endpoint_displacement_m=float(np.linalg.norm(position[-1] - position[0])),
        maximum_frame_step_m=float(steps.max()),
        maximum_frame_rotation_deg=float(angles.max()),
        maximum_speed_m_s=float(np.max(steps / np.diff(arrays["relative_seconds"]))),
        vertical_range_m=float(np.ptp(position[:, 2])),
        maximum_frame_rotation_at_index=int(np.argmax(angles) + 1),
    )
    report["export"][
        "sample_policy"
    ] = "Exactly the baseline selected points; no new sampling or deletion"
    (output / "report.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    if write_osf:
        from ouster.sdk import open_source, osf

        source_osf = baseline["directory"] / "registered.osf"
        if not source_osf.exists():
            source_osf = baseline["directory"] / "pilot15s.osf"
        scans = open_source(str(source_osf))
        writer = osf.Writer(str(output / "registered.osf"), scans.sensor_info)
        try:
            written = 0
            for i, frames in enumerate(scans):
                if i >= len(centres) or int(frames[0].frame_id) != int(arrays["frame_id"][i]):
                    raise ValueError("OSF pose mismatch")
                frames[0].body_to_world[:] = arrays["body_to_world"][i]
                writer.save(frames)
                written += 1
            if written != len(centres):
                raise ValueError("Incomplete corrected OSF")
        finally:
            writer.close()
            scans.close()
    if rhino:
        from tools.export_rhino import export_file

        export_file(output / "registered.ply", output / "rhino")
    return dict(
        points=len(records),
        raw_attributes_retained=True,
        baseline_points_deleted=0,
        height_coordinates_unchanged=True,
        poses_and_points_share_same_corrections=True,
    )


def run_experiment(
    directory, gps_path, output, clock="shared-start", sigma=0.15, lever_sigma=1.0, rhino=True
):
    """Fit three soft-GPS weights, select on validation, evaluate final tests once."""
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    baseline = load_baseline(directory)
    gps = load_gps(gps_path, baseline, clock)
    if np.linalg.norm(baseline["positions"][-1] - baseline["positions"][0]) < 1.0:
        raise ValueError("At least 1 m travel is needed for this GPS experiment")
    features = heldout_features(baseline)
    lidar_before_val = lidar_scores(features, "validation")
    candidates = []
    for weight in [0.5, 1.0, 2.0]:
        model, calibrated = fit_refinement(baseline, gps, sigma * weight, lever_sigma)
        lidar = lidar_scores(features, "validation", model)
        scores = model_scores(model, gps)
        before_gps = model_scores(calibrated, gps)
        increase = lidar["mean_capped_RMS_m"] - lidar_before_val["mean_capped_RMS_m"]
        candidates.append(
            dict(
                model=model,
                baseline_model=calibrated,
                gps=scores,
                baseline_gps=before_gps,
                lidar=lidar,
                eligible=bool(model["success"] and increase <= 0.001),
            )
        )
        print(
            "GPS candidate",
            model["sigma"],
            "validation GPS RMSE",
            scores["validation"]["RMSE_m"],
            "LiDAR cost increase",
            increase,
            flush=True,
        )
    eligible = [c for c in candidates if c["eligible"]]
    chosen = min(eligible or candidates, key=lambda c: c["gps"]["validation"]["RMSE_m"])
    model = chosen["model"]
    # This pixel subset and the final GPS blocks were not used to tune weights.
    lidar_before_test = lidar_scores(features, "test")
    lidar_after_test = lidar_scores(features, "test", model)
    change = paired_lidar_change(lidar_before_test, lidar_after_test)
    alternative, _ = fit_refinement(baseline, gps, model["sigma"], lever_sigma * 2)
    delta = controls_at(model, baseline["times"]) - controls_at(alternative, baseline["times"])
    sensitivity = float(np.max(np.linalg.norm(delta[:, :2], axis=1) + 8 * abs(np.sin(delta[:, 2]))))
    before = chosen["baseline_gps"]["final_test"]["RMSE_m"]
    after = chosen["gps"]["final_test"]["RMSE_m"]
    reasons = []
    if not chosen["eligible"]:
        reasons.append("No candidate passed the LiDAR validation guard")
    if not (after < 0.9 * before and before - after > 0.01):
        reasons.append("No clear improvement on reserved GPS test blocks")
    if change["block_bootstrap_95pct_m"][0] < -0.001:
        reasons.append("Reserved LiDAR consistency may worsen by more than 1 mm")
    if sensitivity > 0.05:
        reasons.append("Correction depends too strongly on uncertain antenna calibration")
    if not alternative["success"]:
        reasons.append("Antenna uncertainty sensitivity fit did not converge")
    if np.max(abs(model["calibration"][3:6])) > 2.95:
        reasons.append("Uncertain antenna fit reached its bounds")
    if np.max(abs(model["controls"][:, :2])) > 0.99 or np.max(
        abs(model["controls"][:, 2])
    ) > np.deg2rad(4.95):
        reasons.append("Trajectory correction reached its experiment bounds")
    accepted = not reasons
    status = (
        "experimental_candidate_passed_internal_checks"
        if accepted
        else "experimental_candidate_not_recommended"
    )
    export = export_candidate(baseline, model, output / "candidate", status, rhino=rhino)

    def summary(c):
        return dict(
            gps_sigma_m=c["model"]["sigma"],
            converged=c["model"]["success"],
            eligible=c["eligible"],
            GPS=c["gps"],
            baseline_GPS=c["baseline_gps"],
            validation_lidar_RMS_m=c["lidar"]["mean_capped_RMS_m"],
        )

    controls = controls_at(model, baseline["times"])
    report = dict(
        status=status,
        accepted=accepted,
        reasons=reasons,
        scope="Existing SDK baseline; experiment, not surveyed accuracy",
        baseline_geometry_changed=False,
        source_frames=len(baseline["times"]),
        source_time_span_s=float(baseline["times"][-1] - baseline["times"][0]),
        clock=clock,
        GPS_measurements=len(gps["times"]),
        split_policy="Raw GPS one-second blocks mod 5: 0/1/2 training, 3 validation, 4 final test; SDK-withheld pixels split 50/50 for selection/final test",
        baseline_GPS=chosen["baseline_gps"],
        candidate_GPS=chosen["gps"],
        candidate_weights=[summary(c) for c in candidates],
        selected_GPS_sigma_m=model["sigma"],
        lidar_validation=dict(
            baseline_RMS_m=lidar_before_val["mean_capped_RMS_m"],
            candidate_RMS_m=chosen["lidar"]["mean_capped_RMS_m"],
        ),
        lidar_final_test=dict(
            baseline_RMS_m=lidar_before_test["mean_capped_RMS_m"],
            candidate_RMS_m=lidar_after_test["mean_capped_RMS_m"],
            baseline_coverage=lidar_before_test["mean_coverage_10cm"],
            candidate_coverage=lidar_after_test["mean_coverage_10cm"],
            paired_change=change,
        ),
        corrections=dict(
            maximum_XY_m=float(np.linalg.norm(controls[:, :2], axis=1).max()),
            maximum_yaw_deg=float(np.rad2deg(abs(controls[:, 2])).max()),
            positions_only_in_XY=True,
            height_roll_pitch_preserved=True,
        ),
        antenna=dict(
            surveyed=False,
            uncertain_fit_body_m=model["calibration"][3:6].tolist(),
            prior_sigma_m=lever_sigma,
            bounds_per_axis_m=3.0,
            doubled_prior_effect_at_8m_m=sensitivity,
            caution="Unmeasured lever arm is a nuisance parameter, not a recovered surveyed calibration; weakly observable components follow the prior",
        ),
        export=export,
        absolute_GPS_coordinates_omitted=True,
        limitations=[
            "Timing uses the shared software-start assumption; latency is not estimated.",
            "Horizontal GPS only; no altitude constraint, scale fit, tree symmetry or ground flattening.",
            "Internal held-out comparisons cannot establish absolute LiDAR accuracy.",
            "Candidate is exported even when rejected; inspect accepted/reasons before using it.",
        ],
    )
    comparison_plot(output, baseline, gps, model, chosen["baseline_model"], report)
    (output / "comparison.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    (output / "README.txt").write_text(
        "EXPERIMENTAL GPS REFINEMENT\n\n"
        + status
        + "\n"
        + "\n".join(reasons)
        + "\n\nRead comparison.json and comparison.png. candidate/ contains the trial, even if rejected. "
        "The original SDK map and Current stationary outputs were not replaced. "
        "All source raw attributes/IDs and heights are retained; corrections are rigid per measurement column. "
        "The antenna lever arm is uncertain, not surveyed.\n"
    )
    return report


def comparison_plot(output, baseline, gps, model, calibrated_baseline, report):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    a = axes[0, 0]
    for data, label in [
        (gps["xy"], "Raw GPS antenna"),
        (refined_gps_prediction(calibrated_baseline, gps), "SDK + antenna calibration"),
        (refined_gps_prediction(model, gps), "GPS-refined + antenna calibration"),
    ]:
        a.plot(data[:, 0], data[:, 1], ".-", ms=2, label=label)
    a.set(
        xlabel="Relative east (m)",
        ylabel="Relative north (m)",
        title="Antenna trajectories · rigid alignment, no scale",
        aspect="equal",
    )
    a.legend(fontsize=7)
    a = axes[0, 1]
    c = controls_at(model, baseline["times"])
    a.plot(baseline["times"], 100 * c[:, 0], label="Map X correction")
    a.plot(baseline["times"], 100 * c[:, 1], label="Map Y correction")
    a.set(
        xlabel="Capture time (s)",
        ylabel="Position correction (cm)",
        title="Slow corrections · height/roll/pitch retained",
    )
    a.legend(fontsize=7)
    twin = a.twinx()
    twin.plot(baseline["times"], np.rad2deg(c[:, 2]), "--", c="gray")
    twin.set_ylabel("Yaw correction (degrees)")
    a = axes[1, 0]
    labels = ["training", "validation", "final_test"]
    x = np.arange(3)
    a.bar(
        x - 0.18,
        [100 * report["baseline_GPS"][k]["RMSE_m"] for k in labels],
        0.36,
        label="SDK baseline",
    )
    a.bar(
        x + 0.18,
        [100 * report["candidate_GPS"][k]["RMSE_m"] for k in labels],
        0.36,
        label="GPS candidate",
    )
    a.set(
        xticks=x,
        xticklabels=["Training", "Selection", "Final test"],
        ylabel="Antenna position RMSE (cm)",
        title="GPS blocks · calibrated antenna models",
    )
    a.legend(fontsize=8)
    a = axes[1, 1]
    q = report["lidar_final_test"]
    a.bar(
        ["SDK baseline", "GPS candidate"],
        [100 * q["baseline_RMS_m"], 100 * q["candidate_RMS_m"]],
        color=["#4477aa", "#ee9933"],
    )
    a.set(
        ylabel="10 cm capped NN RMS (cm)",
        title="Reserved LiDAR pixels · internal overlap, not truth",
    )
    for i, v in enumerate([q["baseline_RMS_m"], q["candidate_RMS_m"]]):
        a.text(i, 100 * v, f"{100*v:.3f}", ha="center", va="bottom")
    for a in axes.flat:
        a.grid(alpha=0.15)
    fig.suptitle(report["status"], fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(Path(output) / "comparison.png", dpi=130)
    plt.close(fig)


def audit(baseline, gps):
    train = gps["training"]
    rotation, translation = rigid_alignment(gps["positions"][train, :2], gps["xy"][train])
    predicted = gps["positions"][:, :2] @ rotation.T + translation
    return dict(
        scope="Existing first-15-second SDK result; no baseline geometry modified",
        frames=len(baseline["times"]),
        gps_samples_in_interval=len(gps["times"]),
        gps_log_samples=gps["full_log_samples"],
        gps_log_duration_s=gps["full_log_duration_s"],
        clock=gps["clock"],
        baseline_GPS_scores=gps_scores(predicted, gps["xy"], train),
        lidar_path_m=float(np.linalg.norm(np.diff(baseline["positions"], axis=0), axis=1).sum()),
        gps_path_m=float(np.linalg.norm(np.diff(gps["xy"], axis=0), axis=1).sum()),
        lever_arm="Unmeasured; baseline rigid overlay assumes zero only for this diagnostic",
        absolute_GPS_coordinates_omitted=True,
    )
