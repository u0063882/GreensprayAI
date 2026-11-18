# GPS-constrained moving-map experiment

Branch: **`experiment/gps-constrained-slam`**. The main branch, the original
SDK result and Current stationary downloads remain unchanged. This is a
separate postprocessing function, not a replacement for Ouster's SDK engine.

Unlike the main pipeline's GPS diagnostic, this experiment **uses GPS in
the optimization objective**. It tests whether small, smooth trajectory
corrections improve independent checks. Better GPS fit alone is insufficient.

## Offline use

In Anaconda Prompt with your `greenspray` environment active, from the repo:

```bat
git fetch origin
git switch --track origin/experiment/gps-constrained-slam
python refine_moving_gps.py --results "C:\Users\u0063882\Documents\moving_15s" --output "C:\Users\u0063882\Documents\moving_gps_experiment"
```

The baseline run must already contain its enriched PLY, per-column NPZ, report
and posed OSF. The matching GPS H5 is discovered under the run's `input/`;
otherwise supply `--gps "C:\path\matching.h5"`. Use a new/empty output folder.
Existing moving-registration dependencies are sufficient. No data is uploaded
by this command. The local command supports your longer existing recordings;
the automated branch comparison uses the already published **15-second** pilot.

If the branch already exists locally, use `git switch experiment/gps-constrained-slam`
and `git pull` instead of creating its tracking branch again.

## Processing and constraints

1. Read the original SDK poses/measurements without modifying them.
2. Match GPS using the confirmed common-software-start assumption and actual
   timestamps. No scaling/stretching of time or matching by sample index.
3. Treat the unmeasured GPS antenna offset as a bounded nuisance parameter:
   a zero-centred 1 m Gaussian prior and +/-3 m bounds per body axis. A global
   horizontal rotation/translation aligns the antenna prediction to local ENU.
   There is no global scale fit. The fitted offset is **not surveyed calibration**.
4. Optimize roughly one-second knots of XY translation and world-Z yaw
   corrections. Anchor the first correction to zero. Preserve SDK heights
   and orientation relative to vertical; do not force flat ground or tree symmetry.
5. Combine robust GPS residuals with SDK relative-motion residuals, weak
   correction priors and smoothness terms. Initial experimental scales are
   2 cm per-frame translation, 0.2 degree relative yaw, 0.05 m/s² correction
   smoothness, 0.3 degree/s² yaw smoothness, 0.5 m absolute correction prior
   and 2 degree yaw prior. Bounds are +/-1 m per XY component and +/-5 degrees
   yaw. These tuning scales are not claimed as calibrated SDK uncertainties.
6. Apply the same correction to each original column pose and each selected
   measured point. Within a column the correction is rigid: no point scaling,
   per-tree deformation, resampling or deletion. Raw signal/NIR/reflectivity,
   RGB/classes and frame/pixel IDs stay paired and unchanged.

The antenna prediction includes orientation and its offset:

```python
antenna_map = refined_body_position + refined_body_rotation @ lever_arm_body
antenna_ENU = yaw_alignment @ antenna_map + translation_alignment
gps_residual = antenna_ENU[:2] - measured_GPS_ENU
```

The H5 has no altitude constraint here. Straight, level motion poorly observes
parts of the antenna offset. Consequently the experiment doubles its prior
uncertainty and measures how much the trajectory/8 m scene correction changes.
If that sensitivity exceeds 5 cm, the candidate is not recommended. Time
latency is not fitted; `--gps-clock absolute` is available if clocks already agree.

## Comparison without fitting the evaluation points

Raw GPS fixes are split into whole one-second blocks modulo five: blocks
0/1/2 train, block 3 selects weights, block 4 is the final test. Duplicate GPS
timestamps are deduplicated. No interpolated GPS measurement leaks held-out
fixes into training. Three GPS weights are tried: nominal sigma times 0.5,
1 and 2 (`--gps-sigma` defaults to 0.15 m).

The original SDK run withheld 20% of pixel locations. This experiment splits
those pixel locations again: half for candidate selection, half reserved for
the chosen candidate's final test. Symmetric, 10 cm capped nearest-neighbor
costs compare scan pairs 5 and 20 frames apart. One-second block bootstrap
intervals help avoid treating every dense point as independent evidence.
These are internal overlap measures, not surveyed tree-shape accuracy.

Selection requires a converged model and at most 1 mm increase in validation
LiDAR capped RMS. The chosen candidate then needs at least a 10% and 1 cm
GPS RMSE improvement on final-test blocks, no convincing >1 mm degradation
of reserved LiDAR overlap, stability to the unknown antenna prior, and no
correction/calibration bound hits. A failed final test is reported, not followed
by trying other candidates on that same test subset.

## Outputs and interpretation

```text
moving_gps_experiment/
  comparison.json
  comparison.png
  README.txt
  candidate/
    registered.ply
    registered.osf
    per_column_poses.npz
    transformations.json
    trajectory.csv
    report.json
    rhino/                 # unless --no-rhino
```

**The candidate is exported even if rejected.** Check `accepted`, `status`
and `reasons` in `comparison.json` before using it. It never overwrites your
baseline. Passing these internal checks does not establish absolute LiDAR
accuracy; GPS timing, antenna calibration and foliage motion remain limitations.

Render the trial with the same GIF tools:

```bat
python make_moving_gif.py --results "C:\Users\u0063882\Documents\moving_gps_experiment\candidate"
```

The automated comparison and trial downloads are published alongside the
original moving pilot as `gps-15s-comparison.json`, `gps-15s-comparison.png`
and `gps-15s-experiment.zip` on the `moving-scan-inputs` prerelease. The original
Dynamic.zip and pilot ZIP digests are verified and left unchanged. Numerical
comparison snapshots are committed only on this experimental branch under
`results/gps_experiment/`. Absolute latitude/longitude are omitted.

The initial rigid-overlay audit found about 7 cm disagreement with GPS over
the existing 15-second trajectory. This is a small residual, so a useful
improvement must be demonstrated rather than assumed. See the generated
comparison for the actual optimization outcome.
