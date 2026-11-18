"""Independent GPS factors, rigid per-column application and held-out scoring."""

import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from dynamic.gps_refinement import (
    fit_refinement,
    model_scores,
    yaw_matrix,
    apply_correction,
    corrected_column_poses,
    column_capture_times,
    robust_gps_residual,
)


def synthetic():
    rng = np.random.default_rng(501)
    t = np.linspace(0, 12, 121)
    true = np.c_[0.4 * t, 0.1 * np.sin(t / 2), np.zeros(len(t))]
    drift = np.c_[0.4 * (t / 12) ** 2, 0.08 * np.sin(t / 5), np.zeros(len(t))]
    rotations = Rotation.from_euler(
        "zyx", np.c_[0.15 * np.sin(t / 4), 0.04 * np.sin(t), 0.03 * np.cos(t / 2)]
    ).as_matrix()
    centres = np.tile(np.eye(4), (len(t), 1, 1))
    centres[:, :3, :3] = rotations
    centres[:, :3, 3] = true + drift
    lever = np.array([0.6, -0.2, 0.15])
    antenna = true + rotations @ lever
    enu_rotation = yaw_matrix(0.8)[:2, :2]
    gps_xy = antenna[:, :2] @ enu_rotation.T + [2, -3] + rng.normal(0, 0.006, (len(t), 2))
    role = np.floor(t).astype(int) % 5
    baseline = dict(times=t, centres=centres, positions=true + drift)
    gps = dict(
        times=t,
        positions=true + drift,
        rotations=rotations,
        xy=gps_xy,
        training=role < 3,
        validation=role == 3,
        testing=role == 4,
    )
    return baseline, gps


class GPSRefinementTests(unittest.TestCase):
    def test_actual_gps_factors_reduce_error_on_unfitted_blocks(self):
        baseline, gps = synthetic()
        model, calibrated_baseline = fit_refinement(baseline, gps, sigma=0.03)
        before = model_scores(calibrated_baseline, gps)
        after = model_scores(model, gps)
        self.assertTrue(model["success"])
        self.assertLess(after["final_test"]["RMSE_m"], before["final_test"]["RMSE_m"] * 0.85)
        np.testing.assert_array_equal(model["controls"][0], [0, 0, 0])

    def test_column_pose_and_point_corrections_agree_and_preserve_height_and_shape(self):
        t = np.array([0.0, 1.0, 2.0])
        poses = np.tile(np.eye(4), (3, 3, 1, 1))
        for i in range(3):
            poses[i, :, :3, :3] = Rotation.from_euler("xyz", [0.1, 0.2, 0.3]).as_matrix()
            poses[i, :, :3, 3] = [i, 0.0, 1.0]
        ts = np.array(
            [1000000000 + (i + np.array([-0.01, 0.0, 0.01])) * 1e9 for i in range(3)]
        ).astype("u8")
        baseline = dict(
            times=t,
            arrays=dict(
                body_to_world=poses, status=np.ones((3, 3), dtype="u4"), column_timestamp_ns=ts
            ),
        )
        model = dict(
            knots=np.array([0.0, 2.0]), controls=np.array([[0.0, 0.0, 0.0], [0.2, -0.1, 0.05]])
        )
        corrected = corrected_column_poses(baseline, model)
        np.testing.assert_array_equal(corrected[:, :, 2, 3], poses[:, :, 2, 3])
        body = np.array([[1.0, 0.2, 0.4], [1.3, -0.1, 0.5]])
        p = body @ poses[1, 1, :3, :3].T + poses[1, 1, :3, 3]
        q = apply_correction(p, np.tile(poses[1, 1, :3, 3], (2, 1)), np.ones(2), model)
        expected = body @ corrected[1, 1, :3, :3].T + corrected[1, 1, :3, 3]
        np.testing.assert_allclose(q, expected, atol=1e-12)
        self.assertAlmostEqual(np.linalg.norm(q[1] - q[0]), np.linalg.norm(p[1] - p[0]))
        np.testing.assert_allclose(q[:, 2], p[:, 2])

    def test_pseudo_huber_limits_gps_outlier_influence(self):
        small = robust_gps_residual(np.array([[0.1, 0.0]]))
        large = robust_gps_residual(np.array([[100.0, 0.0]]))
        self.assertAlmostEqual(np.linalg.norm(small), 0.1, places=3)
        self.assertLess(np.linalg.norm(large), 15.0)

    def test_unfitted_gps_samples_cannot_change_optimizer_solution(self):
        baseline, gps = synthetic()
        first, _ = fit_refinement(baseline, gps, sigma=0.03)
        changed = dict(gps, xy=gps["xy"].copy())
        changed["xy"][~gps["training"]] += [10.0, -8.0]
        second, _ = fit_refinement(baseline, changed, sigma=0.03)
        np.testing.assert_allclose(first["controls"], second["controls"], atol=1e-12)
        np.testing.assert_allclose(first["calibration"], second["calibration"], atol=1e-12)


if __name__ == "__main__":
    unittest.main()
