"""Moving Ouster registration: SDK pose estimation plus our checks and exports.

Start reading at ``run()``. Its numbered sections follow this workflow:

    run_moving.py -> dynamic.offline -> dynamic.prepare -> run()
        1. Read prepared input and sensor calibration; initialize gravity.
        2. Configure the Ouster SDK mapping engine.
        3. For each scan: withhold test pixels -> SDK update -> apply poses
           -> retain measured points and raw attributes -> collect QC samples.
        4. Merge measured samples and save full per-column transformations.
        5. Check GPS, held-out overlap and ground; report problems.
        6. Write trajectory, QC report, preview and optional Rhino copies.

Comments marked [SDK] identify calls supplied by ouster-sdk. Comments marked
[PROJECT] explain decisions made here. We configure and call the SDK SLAM
backend; we do not implement its registration optimizer in this module.

GPS IS DIAGNOSTIC ONLY. ``navigation()`` runs after the scan loop and does not
send GPS positions to the SDK, constrain poses, move points, or georeference
the exported map. ``--gps-clock shared-start`` aligns clocks for that check;
it does not enable GPS-assisted SLAM. See docs/moving_registration_workflow.md.

Only an already prepared PCAP is opened. Stationary registration, meshing and
GIF rendering are separate programs. This readability revision retains the
existing processing parameters and numerical operations.
"""

from pathlib import Path
import argparse
import csv
import json
import os
import time
import sys
import tempfile

# Set runtime defaults before importing numerical libraries. These are local
# CPU/cache settings, not registration parameters or cloud-service settings.
os.environ.setdefault("OMP_NUM_THREADS", "2")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "2")
os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "greenspray-dynamic-mpl"))
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from ouster.sdk import core, pcap, mapping, osf, open_source  # Ouster-provided APIs.
from captures.leafwood import classify_leafwood, write_records


def holdout_mask(h, w):
    """[PROJECT] Select a fixed 20% of organized pixels for independent QC.

    These pixel locations are removed from both returns during SDK fitting,
    then restored before export. The remaining 80% drive registration.
    """
    row, col = np.indices((h, w))
    return (row + 3 * col) % 5 == 0


def measured_indices(p, spacing):
    """[PROJECT] Keep one actual measured point per voxel, not a centroid.

    Return indices so XYZ, signal, NIR, colour and scan/pixel IDs stay paired.
    This export sampling is separate from the SDK's internal mapping voxel.
    """
    _, ix = np.unique(np.floor(p / spacing).astype(np.int64), axis=0, return_index=True)
    return ix


def gravity_pose(acceleration):
    """[PROJECT] Choose the initial map orientation from median acceleration.

    Input is already in the sensor body frame. Map +Z follows approximate up;
    yaw is chosen from body +X projected horizontally, not from GPS heading.
    This is one initial rigid rotation, not ongoing IMU integration, a ground
    leveling operation, or an assumption that trunks are vertical.
    """
    accelerations = np.asarray(acceleration, dtype=float)
    if len(accelerations) < 3:
        raise ValueError("Insufficient IMU observations")
    up = np.median(accelerations, axis=0)
    up /= np.linalg.norm(up)
    forward = np.array([1.0, 0.0, 0.0])
    forward -= up * np.dot(forward, up)
    if np.linalg.norm(forward) < 0.1:
        forward = np.array([0.0, 1.0, 0.0])
        forward -= up * np.dot(forward, up)
    forward /= np.linalg.norm(forward)
    rotation = np.array([forward, np.cross(up, forward), up])
    T = np.eye(4)
    T[:3, :3] = rotation
    return T


def packet_audit(path, info, first_capture_unix):
    """Read raw packets for counts and approximate first-second gravity.

    [SDK] PcapPacketSource/PacketFormat decode legacy IMU and lidar packets.
    [PROJECT] Select the first capture second, rotate acceleration through
    metadata extrinsics, and build the initial pose. Gyro values are only
    summarized here; this code does not integrate them into the trajectory.
    The historical argument name *_unix can contain a relative PCAP clock.
    """
    packet_format = core.PacketFormat(info)
    source = pcap.PcapPacketSource(str(path))
    accelerations = []
    angular_velocities = []
    packet_counts = dict(lidar=0, imu=0)
    try:
        for _, packet in source:
            if isinstance(packet, core.ImuPacket):
                packet_counts["imu"] += 1
                if packet.host_timestamp * 1e-09 < first_capture_unix + 1.0:
                    accelerations.append(
                        [
                            packet_format.imu_la_x(packet.buf),
                            packet_format.imu_la_y(packet.buf),
                            packet_format.imu_la_z(packet.buf),
                        ]
                    )
                    angular_velocities.append(
                        [
                            packet_format.imu_av_x(packet.buf),
                            packet_format.imu_av_y(packet.buf),
                            packet_format.imu_av_z(packet.buf),
                        ]
                    )
            elif isinstance(packet, core.LidarPacket):
                packet_counts["lidar"] += 1
    finally:
        source.close()
    # Rotation only: acceleration is a direction/vector, not a point needing
    # translation. Row-vector form uses transposed metadata rotation matrices.
    accelerations = (
        np.asarray(accelerations, dtype=float).reshape(-1, 3)
        @ np.array(info.imu_to_sensor_transform)[:3, :3].T
        @ np.array(info.sensor_to_body)[:3, :3].T
    )
    try:
        initial_pose = gravity_pose(accelerations)
        status = "first_second_median_legacy_IMU_acceleration"
    except ValueError:
        initial_pose = np.eye(4)
        status = "unavailable_sensor_axes_retained"
    return (
        initial_pose,
        dict(
            **packet_counts,
            initial_gravity_method=status,
            initial_gravity_samples=len(accelerations),
            initial_acceleration_median_body=(
                np.median(accelerations, axis=0).tolist() if len(accelerations) else None
            ),
            initial_acceleration_MAD_body=(
                np.median(abs(accelerations - np.median(accelerations, axis=0)), axis=0).tolist()
                if len(accelerations)
                else None
            ),
            initial_gyro_norm_median_deg_s=(
                float(np.median(np.linalg.norm(angular_velocities, axis=1)))
                if len(angular_velocities)
                else None
            ),
            initial_body_to_map=initial_pose.tolist(),
            caution="Accelerometer median under vehicle motion is approximate, not surveyed Earth vertical or a fitted tree-up correction.",
        ),
    )


def navigation(h5path, host_times, positions, *, clock="absolute", capture_start=None):
    """[PROJECT] Compare the completed trajectory with GPS; never change it.

    ``host_times`` are frame-centre PCAP capture times (not necessarily Unix).
    ``positions`` are SDK map positions in metres. Returns a QC dictionary
    and plot arrays, not corrected poses. All GPS processing here is our code;
    h5py/NumPy supply file access, interpolation and linear algebra.

    shared-start assumes the first valid GPS sample and first PCAP packet
    share an acquisition start. Absolute mode compares timestamps directly.
    Actual sample times are interpolated, so unequal sample counts are fine.
    """
    if not h5path.exists():
        return (dict(available=False), None)
    import h5py

    with h5py.File(h5path) as gps_file:
        gps_times = np.asarray(gps_file["time"]).ravel()
        latitude = np.asarray(gps_file["lat"]).ravel()
        longitude = np.asarray(gps_file["lon"]).ravel()
    if not len(gps_times) == len(latitude) == len(longitude):
        raise ValueError("GPS time/lat/lon lengths differ")
    valid_samples = np.isfinite(gps_times) & np.isfinite(latitude) & np.isfinite(longitude)
    gps_times, latitude, longitude = (
        gps_times[valid_samples],
        latitude[valid_samples],
        longitude[valid_samples],
    )
    time_order = np.argsort(gps_times)
    gps_times, latitude, longitude = (
        gps_times[time_order],
        latitude[time_order],
        longitude[time_order],
    )
    if len(gps_times) < 2:
        return (
            dict(
                available=True, matched_frames=0, time_alignment="Fewer than two valid GPS samples"
            ),
            None,
        )
    gps_times, unique_indices = np.unique(gps_times, return_index=True)
    latitude, longitude = (latitude[unique_indices], longitude[unique_indices])
    if len(gps_times) < 2:
        return (
            dict(
                available=True, matched_frames=0, time_alignment="Fewer than two distinct GPS times"
            ),
            None,
        )
    # Time synchronization for comparison ONLY. No matching by sample index,
    # time stretching, or fitting a clock offset to make the tracks agree.
    if clock == "shared-start":
        if capture_start is None or not np.isfinite(capture_start):
            raise ValueError("Shared-start timing requires the first PCAP packet timestamp")
        query_times = gps_times[0] + (np.asarray(host_times) - capture_start)
        timing = "Assumed common software start: GPS first valid time + elapsed PCAP capture time; no rate scaling or endpoint warping. Startup latency and antenna lever arm uncalibrated."
    elif clock == "absolute":
        query_times = np.asarray(host_times)
        timing = "PCAP capture times interpolated directly against H5 time; no invented clock offset; synchronization and antenna lever arm uncalibrated"
    else:
        raise ValueError("Unknown GPS clock policy")
    clock_report = dict(
        clock_policy=clock,
        time_alignment=timing,
        gps_samples=len(gps_times),
        gps_log_duration_s=float(gps_times[-1] - gps_times[0]),
    )
    overlap = (query_times >= gps_times.min()) & (query_times <= gps_times.max())
    if overlap.sum() < 5:
        return (
            dict(
                available=True,
                matched_frames=int(overlap.sum()),
                **clock_report,
                reason="Insufficient time overlap for a trajectory comparison",
            ),
            None,
        )
    # Work only inside the GPS time coverage; do not extrapolate positions.
    interpolated_lat_lon = np.c_[
        np.interp(query_times[overlap], gps_times, latitude),
        np.interp(query_times[overlap], gps_times, longitude),
    ]
    mean_latitude_rad = np.deg2rad(interpolated_lat_lon[:, 0].mean())
    north_factor = (
        111132.92 - 559.82 * np.cos(2 * mean_latitude_rad) + 1.175 * np.cos(4 * mean_latitude_rad)
    )
    east_factor = 111412.84 * np.cos(mean_latitude_rad) - 93.5 * np.cos(3 * mean_latitude_rad)
    # Short-baseline latitude/longitude -> local east/north metres. Absolute
    # coordinates stay in memory and are omitted from the derived report.
    gps_xy = (interpolated_lat_lon - interpolated_lat_lon[0]) * [north_factor, east_factor]
    gps_xy = gps_xy[:, ::-1]
    slam_xy = positions[overlap, :2]
    centered_slam = slam_xy - slam_xy.mean(0)
    centered_gps = gps_xy - gps_xy.mean(0)
    # Align an XY COPY of SLAM for comparison: rotation + translation, NO
    # scale. Correct a possible SVD reflection to get a proper rotation.
    # This fitted alignment is not applied to the cloud or saved SDK poses.
    svd_u, _, svd_vt = np.linalg.svd(centered_slam.T @ centered_gps)
    horizontal_rotation = svd_vt.T @ svd_u.T
    if np.linalg.det(horizontal_rotation) < 0:
        svd_vt[-1] *= -1
        horizontal_rotation = svd_vt.T @ svd_u.T
    aligned_slam_xy = (slam_xy - slam_xy.mean(0)) @ horizontal_rotation.T + gps_xy.mean(0)
    horizontal_errors = np.linalg.norm(aligned_slam_xy - gps_xy, axis=1)
    gps_path_length = float(np.linalg.norm(np.diff(gps_xy, axis=0), axis=1).sum())
    slam_path_length = float(np.linalg.norm(np.diff(slam_xy, axis=0), axis=1).sum())
    return (
        dict(
            available=True,
            matched_frames=int(overlap.sum()),
            **clock_report,
            gps_path_m=gps_path_length,
            slam_path_over_same_times_m=slam_path_length,
            slam_to_gps_path_ratio=(
                slam_path_length / gps_path_length if gps_path_length > 0.1 else None
            ),
            rigid_2D_fit_rmse_m=float(np.sqrt(np.mean(horizontal_errors**2))),
            gps_endpoint_displacement_m=float(np.linalg.norm(gps_xy[-1] - gps_xy[0])),
            no_scale_fit=True,
            absolute_GPS_coordinates_omitted=True,
        ),
        dict(time=query_times[overlap], gps=gps_xy, slam_fit=aligned_slam_xy),
    )


def consistency(features):
    """[PROJECT] Compare held-out points before/after SDK motion compensation.

    Pairs are five accepted scans apart (~0.5 s at 10 Hz). The before baseline
    applies only the initial common rotation; after uses SDK column poses.
    The 10 cm capped NN score is an internal overlap measure, not surveyed
    accuracy. cKDTree supplies nearest-neighbor lookup, not SLAM fitting.
    """
    pair_results = []
    for i in range(5, len(features)):
        current, previous = (features[i], features[i - 5])
        if min(len(current["before"]), len(previous["before"])) < 30:
            continue
        pair_result = dict(frame_index=current["frame_index"], time_s=current["time_s"])
        for state in ["before", "after"]:
            distances = cKDTree(previous[state]).query(current[state], workers=2)[0]
            pair_result[state + "_capped_rms_m"] = float(
                np.sqrt(np.mean(np.minimum(distances, 0.1) ** 2))
            )
            pair_result[state + "_coverage_10cm"] = float(np.mean(distances < 0.1))
        pair_results.append(pair_result)
    aggregate = {}
    for state in ["before", "after"]:
        aggregate[state] = (
            dict(
                mean_capped_RMS_m=float(
                    np.mean([x[state + "_capped_rms_m"] for x in pair_results])
                ),
                mean_coverage_10cm=float(
                    np.mean([x[state + "_coverage_10cm"] for x in pair_results])
                ),
            )
            if pair_results
            else None
        )
    return dict(
        **aggregate,
        pairs=len(pair_results),
        per_pair=pair_results,
        policy="20% fixed pixel returns withheld from SDK training in every frame, including second returns at those pixels. Identical held-out queries matched against held-out points five processed scans earlier. 10 cm capped nearest-neighbour cost; density/occlusion-sensitive internal diagnostic, not survey accuracy.",
    )


def ground_check(points, features):
    """[PROJECT + Open3D] Diagnose the dominant low surface without leveling it.

    Try RANSAC on the lower 30% of a 4 cm sampled map; inspect how held-out
    points in four time groups sit around that plane. Real ground may slope
    and grass can bias this estimate. The plane never changes SDK poses/XYZ.
    """
    import open3d as o3d

    candidate_points = points[measured_indices(points, 0.04)]
    candidate_points = candidate_points[
        candidate_points[:, 2] < np.quantile(candidate_points[:, 2], 0.3)
    ]
    if len(candidate_points) < 200:
        return dict(supported=False)
    o3d.utility.random.seed(144)
    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(candidate_points))
    plane_model, plane_inliers = cloud.segment_plane(0.04, 3, 1200)
    normal = np.array(plane_model[:3])
    offset = float(plane_model[3])
    if normal[2] < 0:
        normal = -normal
        offset = -offset
    if normal[2] < 0.8:
        return dict(supported=False, reason="No near-up dominant ground plane")
    candidate_points = candidate_points[plane_inliers]
    ground_rms = float(np.sqrt(np.mean((candidate_points @ normal + offset) ** 2)))
    time_groups = []
    for frame_group in np.array_split(np.arange(len(features)), 4):
        if not len(frame_group):
            time_groups.append(dict(points=0, median_signed_distance_m=None, RMS_m=None))
            continue
        group_points = np.concatenate(
            [features[frame_index]["after"] for frame_index in frame_group]
        )
        signed_distance = group_points @ normal + offset
        near_plane = abs(signed_distance) < 0.1
        time_groups.append(
            dict(
                points=int(near_plane.sum()),
                median_signed_distance_m=(
                    float(np.median(signed_distance[near_plane])) if near_plane.any() else None
                ),
                RMS_m=(
                    float(np.sqrt(np.mean(signed_distance[near_plane] ** 2)))
                    if near_plane.any()
                    else None
                ),
            )
        )
    return dict(
        supported=True,
        normal=normal.tolist(),
        offset=offset,
        RMS_m=ground_rms,
        tilt_from_initial_IMU_up_deg=float(np.degrees(np.arccos(np.clip(normal[2], -1, 1)))),
        time_quartiles=time_groups,
        caution="Dominant measured low surface only; a real orchard floor can slope or have grass ridges. This fit is diagnostic and never flattens points or changes poses.",
    )


def run(
    pcap_path,
    output,
    voxel=0.15,
    max_range=25.0,
    *,
    gps_clock="absolute",
    rhino=True,
    generic_outputs=False,
):
    """Process a prepared PCAP, preserving the existing SLAM configuration.

    Parameters use metres except times/angles reported explicitly elsewhere.
    ``gps_clock`` selects a post-hoc comparison policy, not a pose restriction.
    ``rhino`` adds compatibility copies; ``generic_outputs`` chooses offline
    filenames. Neither option changes the fitted geometry.

    Two pose levels are saved: one 4x4 body-to-map transform per lidar column
    for motion compensation, plus frame-centre summaries for plots/JSON/CSV.
    """
    # 1. [PROJECT] Read the interval already selected by dynamic.prepare.
    # This function does not choose/download an input or open the full ZIP.
    started = time.monotonic()
    pcap_path = Path(pcap_path)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    prep = json.loads((pcap_path.parent / "preparation.json").read_text())
    requested = prep.get("requested_seconds", 15.0)
    interval = "Full input recording" if requested is None else f"First {requested:g} seconds"
    cloud_path = output / ("registered.ply" if generic_outputs else "pilot_registered.ply")
    report_path = output / ("report.json" if generic_outputs else "pilot_report.json")
    osf_path = output / ("registered.osf" if generic_outputs else "pilot15s.osf")
    preview_name = "preview.png" if generic_outputs else "pilot_preview.png"
    # [SDK] Decode scans and discover matching sensor JSON/calibration.
    # This pipeline uses sensor 0 from each SDK frame set (one lidar).
    source = open_source(str(pcap_path))
    sensor_info = source.sensor_info[0]
    initial_pose, packets = packet_audit(pcap_path, sensor_info, prep["capture_first_unix"])
    # 2. [SDK + PROJECT] Create the SDK backend and set our chosen parameters.
    # "lio" is the SDK engine name. We explicitly select constant-velocity
    # deskew for this legacy-IMU recording, not a fully coupled inertial mode.
    # No GPS positions, leaf/wood labels, ground planes or tree priors enter
    # this configuration or the update call below.
    slam_config = mapping.SlamConfig.create("lio")
    slam_config.deskew_method = "constant_velocity"
    slam_config.voxel_size = voxel
    slam_config.min_range = 1.0
    slam_config.max_range = max_range
    slam_config.max_iterations = 50
    slam_config.initial_pose = initial_pose
    slam_engine = mapping.SlamEngine.create([sensor_info], slam_config)
    # [SDK] True applies sensor-to-body extrinsics. XYZ is in metres and keeps
    # organized row/column indexing; do not destagger before applying poses.
    xyz_lookup = core.XYZLut(sensor_info, True)
    osf_writer = osf.Writer(str(osf_path), [sensor_info])
    leafwood_model = json.loads((ROOT / "dynamic/shared_leafwood_model.json").read_text())
    column_window = sensor_info.format.column_window
    # [PROJECT] Completeness is relative to the configured column window,
    # not all 1024 columns when the lidar records only a 180-degree sector.
    expected_valid_columns = (
        column_window[1] - column_window[0]
    ) % sensor_info.format.columns_per_frame + 1
    column_pose_history = []
    relative_times = []
    capture_times = []
    frame_ids = []
    column_time_history = []
    column_status_history = []
    heldout_features = []
    point_blocks = []
    incomplete_frames = []
    flags = []
    first_lidar_timestamp_ns = None
    # PLY custom fields retain selected original returns and their provenance.
    # Full integer lidar timestamps live in NPZ; PLY stores relative seconds.
    dtype = np.dtype(
        [
            ("x", "<f4"),
            ("y", "<f4"),
            ("z", "<f4"),
            ("red", "u1"),
            ("green", "u1"),
            ("blue", "u1"),
            ("reflectivity", "u1"),
            ("intensity", "<u2"),
            ("signal", "<u2"),
            ("near_ir", "<u2"),
            ("scan_id", "<u4"),
            ("frame_id", "<u4"),
            ("pixel_index", "<u4"),
            ("time_offset_s", "<f8"),
            ("leafwood_class", "u1"),
            ("wood_score", "<f4"),
        ]
    )
    try:
        # 3. Stream complete scans through the SDK one at a time.
        for source_frame_index, frame_set in enumerate(source):
            scan = frame_set[0]
            if scan is None:
                continue
            # 3a. [PROJECT] Reject incomplete scans, then track TWO clocks:
            # lidar ns for deskew/export, capture seconds for GPS comparison.
            coverage = float(np.sum(scan.status & 1 > 0) / expected_valid_columns)
            if coverage < 0.98:
                incomplete_frames.append(dict(frame_id=int(scan.frame_id), coverage=coverage))
                continue
            frame_index = len(column_pose_history)
            valid_columns = np.flatnonzero(scan.status & 1 > 0)
            column_timestamps_ns = np.asarray(scan.timestamp).copy()
            first_column_timestamp_ns = int(column_timestamps_ns[valid_columns[0]])
            centre_timestamp_ns = int(np.median(column_timestamps_ns[valid_columns]))
            capture_time_s = float(
                np.median(np.asarray(scan.packet_timestamp)[np.asarray(scan.packet_timestamp) > 0])
                * 1e-09
            )
            if first_lidar_timestamp_ns is None:
                first_lidar_timestamp_ns = first_column_timestamp_ns
            relative_time_s = (centre_timestamp_ns - first_lidar_timestamp_ns) * 1e-09
            # 3b. [PROJECT] Decode geometry before masking the held-out pixels.
            # SDK RANGE values are mm; XYZLut coordinates are metres.
            raw_ranges = scan.field("RANGE").copy()
            raw_second_ranges = scan.field("RANGE2").copy() if scan.has_field("RANGE2") else None
            held_out_pixels = holdout_mask(scan.h, scan.w)
            body_xyz = xyz_lookup(scan)
            valid_returns = (
                (raw_ranges >= 1000)
                & (raw_ranges < max_range * 1000)
                & (scan.status[None, :] & 1 > 0)
            )
            # Exclude both returns at test pixels so either return cannot
            # leak into fitting. All original ranges are restored in finally.
            scan.field("RANGE")[held_out_pixels] = 0
            if raw_second_ranges is not None:
                scan.field("RANGE2")[held_out_pixels] = 0
            try:
                # 3c. [SDK] THIS IS THE SLAM POSE-ESTIMATION CALL.
                # SDK internals estimate motion/registration and attach
                # per-column body_to_world matrices to the updated scan.
                updated_frames = slam_engine.update(frame_set)
                scan = updated_frames[0]
            finally:
                frame_set[0].field("RANGE")[:] = raw_ranges
                if raw_second_ranges is not None:
                    frame_set[0].field("RANGE2")[:] = raw_second_ranges
            # 3d. [PROJECT] Read SDK poses and guard against obvious failure.
            # A >1 m or >25 degree step stops processing (not a correction).
            # Accepted earlier scans will still be exported with a flag.
            column_poses = np.asarray(scan.body_to_world).copy()
            centre_pose = column_poses[valid_columns[len(valid_columns) // 2]]
            if not np.isfinite(column_poses).all():
                flags.append("Non-finite SLAM pose")
                break
            if column_pose_history:
                previous_centre_pose = column_pose_history[-1][
                    np.flatnonzero(column_status_history[-1] & 1)[
                        len(np.flatnonzero(column_status_history[-1] & 1)) // 2
                    ]
                ]
                relative_pose = np.linalg.inv(previous_centre_pose) @ centre_pose
                translation_step = np.linalg.norm(relative_pose[:3, 3])
                rotation_step_deg = np.degrees(
                    np.linalg.norm(Rotation.from_matrix(relative_pose[:3, :3]).as_rotvec())
                )
                if translation_step > 1.0 or rotation_step_deg > 25:
                    flags.append(
                        f"Implausible per-frame jump at processed frame {frame_index}: {translation_step:.3f} m / {rotation_step_deg:.2f} deg"
                    )
                    break
            # 3e. [SDK] Apply the pose of each COLUMN to its measured XYZ.
            # map_point = body_to_world[column] @ homogeneous_body_point.
            # This moves measured returns into the common map, not a fitted
            # tree model. Per-column poses account for within-scan motion.
            map_xyz = core.dewarp(body_xyz, column_poses)
            column_pose_history.append(column_poses)
            relative_times.append(relative_time_s)
            capture_times.append(capture_time_s)
            frame_ids.append(int(scan.frame_id))
            column_time_history.append(column_timestamps_ns)
            column_status_history.append(np.asarray(scan.status).copy())
            # [SDK] Save complete posed scans with original ranges restored.
            osf_writer.save(frame_set)
            # 3f. [PROJECT] Retain one real first return per 4 cm frame voxel.
            # Keep the SAME indices for XYZ and every sensor channel/ID.
            point_indices = np.flatnonzero(valid_returns)
            sampled_map_xyz = map_xyz.reshape(-1, 3)[point_indices]
            voxel_selection = measured_indices(sampled_map_xyz, 0.04)
            point_indices = point_indices[voxel_selection]
            sampled_map_xyz = sampled_map_xyz[voxel_selection]
            point_records = np.zeros(len(point_indices), dtype=dtype)
            for component, field in enumerate(["x", "y", "z"]):
                point_records[field] = sampled_map_xyz[:, component]
            point_records["signal"] = scan.field("SIGNAL").ravel()[point_indices]
            point_records["intensity"] = point_records["signal"]
            point_records["near_ir"] = scan.field("NEAR_IR").ravel()[point_indices]
            point_records["reflectivity"] = scan.field("REFLECTIVITY").ravel()[point_indices]
            point_records["scan_id"] = frame_index
            point_records["frame_id"] = scan.frame_id
            point_records["pixel_index"] = point_indices
            point_records["time_offset_s"] = (
                column_timestamps_ns[point_indices % scan.w].astype(float)
                - first_lidar_timestamp_ns
            ) * 1e-09
            # Our shared classifier supplies display RGB/labels AFTER fitting.
            # It does not affect SDK correspondences or the trajectory.
            leafwood = classify_leafwood(
                point_records["signal"], point_records["near_ir"], leafwood_model
            )
            point_records["leafwood_class"] = leafwood["leafwood_class"]
            point_records["wood_score"] = leafwood["wood_score"]
            for component, field in enumerate(["red", "green", "blue"]):
                point_records[field] = leafwood["rgb"][:, component]
            point_blocks.append(point_records)
            # 3g. [PROJECT] Keep up to 1000 held-out returns within 8 m for QC.
            # The same measured pixels form the before/after comparison.
            point_indices = np.flatnonzero(valid_returns & held_out_pixels & (raw_ranges < 8000))
            heldout_body_xyz = body_xyz.reshape(-1, 3)[point_indices]
            heldout_selection = measured_indices(heldout_body_xyz, 0.06)
            if len(heldout_selection) > 1000:
                heldout_selection = heldout_selection[
                    np.linspace(0, len(heldout_selection) - 1, 1000).astype(int)
                ]
            point_indices = point_indices[heldout_selection]
            unregistered = (
                body_xyz.reshape(-1, 3)[point_indices] @ initial_pose[:3, :3].T
                + initial_pose[:3, 3]
            )
            registered = map_xyz.reshape(-1, 3)[point_indices]
            heldout_features.append(
                dict(
                    frame_index=frame_index,
                    time_s=relative_time_s,
                    before=unregistered,
                    after=registered,
                )
            )
            if frame_index % 10 == 0:
                print(
                    "SLAM",
                    frame_index,
                    round(relative_time_s, 3),
                    "s; pose",
                    np.round(centre_pose[:3, 3], 3).tolist(),
                    flush=True,
                )
    finally:
        osf_writer.close()
        source.close()
    if len(column_pose_history) < 3:
        raise RuntimeError("Fewer than three complete processed frames")
    # 4. [PROJECT] Fuse accepted measured records by 2.5 cm sampling and save
    # the enriched PLY and exact per-column SDK matrices. Voxel spacings are
    # sampling/configuration choices, not statements of survey accuracy.
    merged_records = np.concatenate(point_blocks)
    merged_records = merged_records[
        measured_indices(
            np.c_[merged_records["x"], merged_records["y"], merged_records["z"]], 0.025
        )
    ]
    write_records(
        cloud_path,
        merged_records,
        comment=interval
        + "; Ouster per-column motion compensation; provisional shared leaf/wood display; scan_id=processed frame index",
    )
    np.savez_compressed(
        output / "per_column_poses.npz",
        body_to_world=np.array(column_pose_history),
        column_timestamp_ns=np.array(column_time_history),
        status=np.array(column_status_history),
        frame_id=np.array(frame_ids),
        host_unix=np.array(capture_times),
        relative_seconds=np.array(relative_times),
    )
    # Frame-centre poses are convenient summaries, not substitutes for the
    # column arrays when reproducing the motion-compensated point positions.
    centre_poses = np.array(
        [
            c[np.flatnonzero(s & 1)[len(np.flatnonzero(s & 1)) // 2]]
            for c, s in zip(column_pose_history, column_status_history)
        ]
    )
    trajectory_positions = centre_poses[:, :3, 3]
    frame_intervals = np.diff(relative_times)
    translation_steps = np.linalg.norm(np.diff(trajectory_positions, axis=0), axis=1)
    rotation_steps_deg = np.degrees(
        np.linalg.norm(
            Rotation.from_matrix(
                np.einsum(
                    "nij,njk->nik",
                    centre_poses[:-1, :3, :3].transpose(0, 2, 1),
                    centre_poses[1:, :3, :3],
                )
            ).as_rotvec(),
            axis=1,
        )
    )
    # 5. [PROJECT] All pose estimation and map-point placement is finished.
    # GPS is used HERE, as a report/plot diagnostic only. The SDK never sees
    # this result, and navigation() cannot alter trajectory_positions.
    gps_diagnostic, gps_preview = navigation(
        pcap_path.with_suffix(".h5"),
        np.array(capture_times),
        trajectory_positions,
        clock=gps_clock,
        capture_start=prep["capture_first_unix"],
    )
    heldout_diagnostic = consistency(heldout_features)
    ground_diagnostic = ground_check(
        np.c_[merged_records["x"], merged_records["y"], merged_records["z"]], heldout_features
    )
    # These post-hoc checks set report flags/status. Unlike the earlier jump
    # guard, they do not stop the already completed SDK loop or change poses.
    if np.any(frame_intervals <= 0):
        flags.append("Non-increasing lidar frame timestamps")
    if (
        gps_diagnostic.get("slam_to_gps_path_ratio") is not None
        and gps_diagnostic.get("gps_endpoint_displacement_m", 0) > 1
        and (not 0.5 < gps_diagnostic["slam_to_gps_path_ratio"] < 2)
    ):
        flags.append(
            "Large disagreement with GPS travelled distance; timing/lever arm also need review"
        )
    if (
        heldout_diagnostic["after"]
        and heldout_diagnostic["before"]
        and (
            heldout_diagnostic["after"]["mean_capped_RMS_m"]
            > heldout_diagnostic["before"]["mean_capped_RMS_m"] * 1.05
        )
    ):
        flags.append("Withheld return consistency worsens relative to the unregistered baseline")
    has_motion = bool(np.linalg.norm(trajectory_positions[-1] - trajectory_positions[0]) > 0.5)
    recommendation = (
        "review_flags_before_full_run"
        if flags
        else (
            "plausible_pilot_continue_with_QC"
            if has_motion
            else "insufficient_motion_to_validate_moving_SLAM"
        )
    )
    # 6. [PROJECT] Describe accepted outputs and diagnostic limitations.
    # Keep report keys/units stable for existing analysis and GIF scripts.
    report = dict(
        status=recommendation,
        flags=flags,
        scope=interval + "; no mesh, ground flattening or tree symmetry constraints",
        source=prep,
        sensor=dict(
            model=str(sensor_info.prod_line),
            firmware=str(sensor_info.fw_rev),
            fps=sensor_info.format.fps,
            lidar_mode=str(sensor_info.config.lidar_mode),
            expected_columns=expected_valid_columns,
            columns_per_frame=sensor_info.format.columns_per_frame,
        ),
        SDK_version=__import__("importlib.metadata", fromlist=["version"]).version("ouster-sdk"),
        SLAM=dict(
            backend="Ouster SDK LIOSlamEngine, same engine as ouster-cli slam",
            deskew_method="constant_velocity",
            internal_voxel_m=voxel,
            min_range_m=1.0,
            max_range_m=max_range,
            max_iterations=50,
            withheld_pixel_fraction=0.2,
            second_returns_at_withheld_pixels_also_removed=True,
        ),
        packets=packets,
        processed_frames=len(column_pose_history),
        dropped_incomplete_frames=incomplete_frames,
        processed_time_span_s=float(relative_times[-1] - relative_times[0]),
        retained_first_return_points=len(merged_records),
        trajectory=dict(
            path_length_m=float(translation_steps.sum()),
            endpoint_displacement_m=float(
                np.linalg.norm(trajectory_positions[-1] - trajectory_positions[0])
            ),
            maximum_frame_step_m=float(translation_steps.max()),
            maximum_frame_rotation_deg=float(rotation_steps_deg.max()),
            maximum_speed_m_s=float(np.max(translation_steps / np.maximum(frame_intervals, 1e-09))),
            vertical_range_m=float(np.ptp(trajectory_positions[:, 2])),
        ),
        GNSS_diagnostic=gps_diagnostic,
        held_out_consistency=heldout_diagnostic,
        ground_diagnostic=ground_diagnostic,
        elapsed_compute_s=time.monotonic() - started,
        export=dict(
            source_return="first return only; dual-return raw channels retained in processed OSF",
            sample_policy="One real return per 4 cm frame voxel, then one per 2.5 cm map voxel; no averaged/invented points",
            raw_channels=[
                "signal",
                "intensity",
                "near_ir",
                "reflectivity",
                "frame_id",
                "pixel_index",
            ],
            time_offset_reference_lidar_ns=first_lidar_timestamp_ns,
            RGB="Existing shared stationary leaf/wood model used only for provisional display; different acquisition/illumination not revalidated; never used by this SLAM fit",
        ),
        limitations=[
            "Internal diagnostics only, no surveyed accuracy guarantee.",
            "GNSS uses uncalibrated capture-host timing and antenna lever arm; no GNSS constraints fitted.",
            "No loop closure or pose graph optimization in this first-pass pilot.",
            "180-degree view and repeated orchard geometry can cause drift, and foliage can move.",
        ],
    )
    # CSV and JSON use frame-centre translation and qx,qy,qz,qw rotation.
    # The exported PLY already has the SDK transformations applied.
    with (output / "trajectory.csv").open("w", newline="") as trajectory_file:
        csv_writer = csv.writer(trajectory_file, lineterminator="\n")
        csv_writer.writerow(
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
        for i, (centre_pose, t, h, fid) in enumerate(
            zip(centre_poses, relative_times, capture_times, frame_ids)
        ):
            csv_writer.writerow(
                [
                    i,
                    fid,
                    t,
                    h,
                    *centre_pose[:3, 3],
                    *Rotation.from_matrix(centre_pose[:3, :3]).as_quat(),
                ]
            )
    report["limitations"][
        1
    ] = "GPS comparison uses the reported clock policy; startup latency and antenna lever arm are uncalibrated. GPS does not constrain the fitted map."
    report["trajectory"]["maximum_frame_rotation_at_index"] = int(np.argmax(rotation_steps_deg) + 1)
    plot(
        output,
        merged_records,
        np.array(relative_times),
        trajectory_positions,
        gps_preview,
        report,
        title=interval + " moving OS0 trial",
        filename=preview_name,
    )
    # Small measured-point previews make remote QC possible without sending
    # the whole cloud. They are not a second reconstruction or re-fit.
    point_indices = np.linspace(0, len(merged_records) - 1, min(2500, len(merged_records))).astype(
        int
    )
    report["preview_sample"] = dict(
        xyz=np.round(
            np.c_[merged_records["x"], merged_records["y"], merged_records["z"]][point_indices], 4
        ).tolist(),
        rgb=np.c_[merged_records["red"], merged_records["green"], merged_records["blue"]][
            point_indices
        ].tolist(),
    )
    report["preview_trajectory"] = np.round(trajectory_positions, 5).tolist()
    report["preview_time_s"] = np.round(relative_times, 6).tolist()
    report_path.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    if rhino:
        # [PROJECT + rhino3dm] Coordinate/colour-preserving CAD copies only.
        from tools.export_rhino import export_file

        report["rhino_export"] = export_file(cloud_path, output / "rhino")
        report_path.write_text(
            json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8"
        )
    if generic_outputs:
        transformations = dict(
            units="metres",
            convention="Column-vector homogeneous transforms: map_point = body_to_world @ body_point. Ouster XYZLut applies sensor-to-body extrinsics first.",
            note="Frame-centre poses are summaries; use the per-column poses in per_column_poses.npz to reproduce motion-compensated points. Do not apply poses again to registered.ply.",
            column_poses="per_column_poses.npz",
            frames=[
                dict(scan_id=i, frame_id=fid, time_s=float(t), body_to_world=centre_pose.tolist())
                for i, (fid, t, centre_pose) in enumerate(
                    zip(frame_ids, relative_times, centre_poses)
                )
            ],
        )
        (output / "transformations.json").write_text(
            json.dumps(transformations, indent=2, allow_nan=False) + "\n", encoding="utf-8"
        )
    (output / "README.txt").write_text(
        "MOVING LIDAR — "
        + interval
        + " — QC RESULT\n\nRead "
        + report_path.name
        + " before using the map. This is not an independently validated reconstruction.\nThe PLY is in metres; RGB is provisional shared leaf/wood display. Raw signal/NIR and source frame/pixel IDs are retained for selected real first returns. scan_id is the processed frame index.\nThe OSF retains the selected complete scans and per-column SDK poses. per_column_poses.npz and trajectory.csv expose those poses/times.\nNative Rhino 8 files, when requested, are in rhino/ in MILLIMETRES; physical geometry is unchanged.\nNo stationary result, Current-release asset, mesh, GPS constraint, loop closure or tree shape prior is used or replaced by this run.\n",
        encoding="utf-8",
    )
    print(
        "PILOT SUMMARY",
        json.dumps(
            {
                k: report[k]
                for k in [
                    "status",
                    "flags",
                    "processed_frames",
                    "processed_time_span_s",
                    "retained_first_return_points",
                    "trajectory",
                    "GNSS_diagnostic",
                    "elapsed_compute_s",
                ]
            }
        ),
        flush=True,
    )
    return report


def plot(
    output,
    records,
    times,
    positions,
    nav,
    report,
    *,
    title="15-second moving OS0 pilot",
    filename="pilot_preview.png",
):
    """[PROJECT + Matplotlib] Plot QC only; no estimates flow back into SLAM."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    points = np.c_[records["x"], records["y"], records["z"]]
    preview_indices = np.linspace(0, len(points) - 1, min(70000, len(points))).astype(int)
    preview_points = points[preview_indices]
    preview_rgb = np.c_[records["red"], records["green"], records["blue"]][preview_indices] / 255
    fig, axes = plt.subplots(2, 2, figsize=(12, 9))
    ax = axes[0, 0]
    ax.plot(positions[:, 0], positions[:, 1], label="SDK trajectory")
    ax.scatter(*positions[0, :2], c="green", s=30, label="Start")
    ax.scatter(*positions[-1, :2], c="red", s=30, label="End")
    ax.set(xlabel="Map X (m)", ylabel="Map Y (m)", title="Sensor trajectory", aspect="equal")
    ax.legend(fontsize=8)
    ax = axes[0, 1]
    if nav:
        ax.plot(nav["gps"][:, 0], nav["gps"][:, 1], label="GNSS antenna, local")
        ax.plot(nav["slam_fit"][:, 0], nav["slam_fit"][:, 1], label="SLAM, rigid fit without scale")
        ax.legend(fontsize=8)
        ax.set_aspect("equal")
    else:
        ax.text(
            0.1,
            0.5,
            "No GPS comparison available\nSee report for clock policy and coverage",
            transform=ax.transAxes,
        )
    ax.set(
        title="GNSS diagnostic · timing/lever arm uncalibrated",
        xlabel="Relative east (m)",
        ylabel="Relative north (m)",
    )
    ax = axes[1, 0]
    ax.scatter(
        preview_points[:, 0], preview_points[:, 1], c=preview_rgb, s=0.35, alpha=0.5, linewidths=0
    )
    ax.plot(positions[:, 0], positions[:, 1], c="#c52843", lw=1)
    ax.set(
        title="Pilot measured map · top view",
        xlabel="Map X (m)",
        ylabel="Map Y (m)",
        aspect="equal",
    )
    travel = positions[-1, :2] - positions[0, :2]
    travel /= max(np.linalg.norm(travel), 1e-09)
    along = preview_points[:, :2] @ travel
    ax = axes[1, 1]
    ax.scatter(along, preview_points[:, 2], c=preview_rgb, s=0.35, alpha=0.5, linewidths=0)
    ax.set(
        title="Height profile along travelled direction",
        xlabel="Along travel projection (m)",
        ylabel="Gravity-initialized Z (m)",
        aspect="equal",
    )
    for ax in axes.flat:
        ax.grid(alpha=0.2)
    fig.suptitle(title + " · " + report["status"], fontsize=13)
    fig.text(
        0.5,
        0.012,
        "Constant-velocity per-column deskew; 20% return holdout. Colours are provisional leaf/wood display. No tree straightening or ground flattening.",
        ha="center",
        fontsize=8,
    )
    fig.tight_layout(rect=(0, 0.04, 1, 0.95))
    fig.savefig(Path(output) / filename, dpi=125)
    plt.close(fig)


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--pcap", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--voxel", type=float, default=0.15)
    a = p.parse_args()
    run(a.pcap, a.output, a.voxel)
