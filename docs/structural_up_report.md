# Shared PCA structural-up frame and wider chandelier coverage

The output frame now follows the paired-view woody structure, addressing a common lean that inter-scan registration checks cannot detect. Chandelier's measured ROI is also wider **across** the row. These are separate operations: neither changes individual tree shapes, branch angles or measured intensity/colour.

## Why the previous checks missed a common lean

A rigid rotation applied to an entire reconstruction preserves all distances and inter-scan residuals. Therefore the earlier improvements in trunk matching and anchor stability did not establish the correct absolute up direction. The ground fit retains local slopes, but the delivered assembly can still inherit an unsuitable global orientation.

Plain PCA of the whole cloud is also unreliable here. Row length competes with height in 3D PCA, and one visible V arm can dominate the across-row covariance. For V-hedge the initial whole-canopy across-row PCA suggested approximately 16.5°, with substantial section-to-section variation. Rotating each scan to its own PCA axis would change the relative tree geometry and is not used.

## Structural-up estimate

`captures/structural_frame.py` uses the same method for every training system:

1. Select provisional wood returns using the saved shared leaf/wood model, and remove sampling-density differences with measured-point voxels.
2. Divide the row into short X windows and height bands in the existing registration frame.
3. Calculate a median wood centre independently for each viewing side at each height. Retain bands supported by both sides and weight the sides equally.
4. Fit PCA to the resulting height-centre paths, centred within each row window so the row's length cannot dominate the up estimate. Reject inconsistent window directions and bootstrap the remaining windows.
5. Apply one shared across-row rotation only when the direction is supported and exceeds 1°. Every point cloud and every mesh in that dataset receives the same rigid transform. No individual trunk or V arm is made vertical.

The estimated correction is about **−10.33° for V-hedge** and **−8.39° for chandelier**. Neither spindle dataset has a sufficiently large, consistently supported correction, so their accepted data are retained unchanged.

![V-hedge common-frame comparison](figures/vhedge_structural_up.png)

![Chandelier common-frame comparison](figures/chandelier_structural_up.png)

Both panels use identical axes and source-return identities. The angle between branches, all pairwise distances and every relative sensor pose are preserved. The ground is rotated with the measured scene and is not flattened.

**This is a structural-up coordinate convention, not an independent measurement of Earth vertical.** A tree row can genuinely lean, and occlusion or provisional wood labels can bias the estimate. The old gravity reference is explicitly retained as a vector in the output metadata; it is generally not parallel to the new Z axis. The bootstrap interval describes variation among structural windows, not uncertainty of surveyed vertical. The inverse saved transform recovers the previous registration frame.

Expressing the final joint result in this frame addresses the global coordinate ambiguity directly. It does not claim that a different PCA starting pose supplies new relative-registration information. Future optimisations first restore the original working frame, so source gravity constraints, terrain models and low-height fitting bands keep their original meanings. Trunk axes remain freely inclined during relative registration.

## Across-row ROI recovery

The former chandelier across-row limits were **Y = 1.2875…2.6875 m**, a 1.4 m strip. They are now **Y = 0.65…3.35 m**, while retaining the previously widened X limits **−1.25…7.35 m** and height range **−0.12…4.5 m**. These bounds refer to the pre-structural registration frame; the JSON contains the transform to current output coordinates.

The new decoder applies the world ROI directly to original first-return PCAP measurements. It avoids the old 0.35–3.15 m sensor-forward gate, which would otherwise discard some newly requested coverage. The existing accepted poses and shared colour model are retained.

The chandelier primary grows from **1,326,879 to 1,527,905 points**, adding **201,026 measured samples**. Before the common frame rotation, every former primary and separate-scan sample is preserved exactly as a prefix. After rotation, all non-coordinate fields remain unchanged and the inverse transform recovers those coordinates within float32 precision. The complete previous state is archived in QC.

![Recovered across-row coverage](figures/chandelier_across_roi.png)

The independent mesher rebuilds from the widened PLY. Its closed envelope has **609,606 triangles** and passes the existing closed-edge, vertex-manifold and orientability checks. The point cloud, canopy mesh and separate ground sheet are then rigidly transformed together. The current mesh report hashes the exact final PLY and records the common frame transform. A global triangle self-intersection search is not performed; this remains the documented voxel/SDF shrinkwrap approximation.

## Downloads and metadata

Use the existing [Current release](https://github.com/u0063882/GreensprayAI/releases/tag/current):

- `01_final_aligned_pointclouds.zip`: final clouds in their reported output frames.
- `02_final_mesh.zip`: matching widened chandelier mesh and ground.
- `03_transformations.zip`: composed sensor-to-output matrices, structural-frame transform and the prior gravity reference.
- `91_registered_contributions.zip`: separate scans and recording-colour views in the same final frame.
- `90_QC.zip`: frame estimates, before/after plots, ROI proofs and provenance snapshots.

`QC/<dataset>/structural_frame/report.json` records the estimate, bootstrap interval, individual window paths and `registration_to_structural` matrix. `structural_frame_baseline.zip` prevents double rotation on reruns. Chandelier's `roi_across_extension_baseline.zip` preserves the complete former X-expanded cloud, contributions, matrices and mesh; the older pre-X archive is retained separately.

Earlier joint-registration figures and residuals describe the physics working frame. Their relative-geometry conclusions remain valid under the common rotation, but they should not be read as absolute-up validation. RGB, SIGNAL/intensity, NIR, reflectivity, scan IDs, frame IDs and pixel indices are preserved. The legacy height classification still refers to the earlier height band and is not a new material label.

Validation: **61 tests passed locally**, including known-axis recovery, avoiding independent single-side PCA, rigid attribute/matrix preservation, working-frame restoration, idempotent exports and across-only ROI recovery. Real-data checks verify all composed sensor matrices, sub-micrometre coordinate-export errors and the current chandelier mesh/source-Ply hash.
