# User's Rhino 8 reference settings

The supplied screenshots and the user's sequence clarification establish this reference for future comparisons. The ShrinkWrap status line explicitly reports millimetres; converted SI lengths are provided for this project's metre-based point clouds.

| Operation / setting | Rhino value | SI interpretation |
| --- | --- | --- |
| ShrinkWrap target edge length | 15 mm | 0.015 m approximate output edge scale |
| Offset | −5 mm | −0.005 m final offset |
| Smoothing iterations | 1 | One Rhino smoothing iteration |
| Polygon optimization | 50% | Rhino's optimization strength, not a promised 50% face reduction |
| Inflate vertices and points | Enabled | Point-neighbourhood inflation |
| Fill holes in input objects | Enabled | See point-input caveat below |
| Compute vertex colours | Disabled | Geometry reference |
| Delete input objects | Disabled | Preserve input |
| Separate Reduce Mesh operation | Reduce by 75% | Keep approximately 25% of that operation's input polygons |
| Reduce Mesh “Planar only” | Disabled | No planar-only restriction |

**Confirmed sequence: point cloud → ShrinkWrap → Reduce Mesh by 75%, retaining approximately 25% of the ShrinkWrap output polygons.**

Machine-readable record: [`presets/rhino8_reference.json`](../presets/rhino8_reference.json). This file is deliberately a **reference**, not a preset accepted by `run_meshing.py`. Registration and meshing remain independent; there is no dependency on raw LiDAR scans, pose fitting, or the low-trunk diagnostic.

The 75% reduction is applied after ShrinkWrap. ShrinkWrap's 50% optimization remains part of the preceding wrapping operation; it is not a promised 50% face reduction, so do not multiply the percentages to infer a final face count. The screenshots show 395,880 faces in the ShrinkWrap preview and 4,297,912 starting polygons in Reduce Mesh; those displayed counts are not a matched before/after pair. The latter dialog's 75% reduction corresponds to 1,074,478 remaining polygons. Those counts depend on the input and are not targets for an arbitrary new cloud. “Show” under locked mesh points only indicates a display option; it does not reveal whether vertices were locked.

According to the [Rhino 8 command reference](https://docs.mcneel.com/rhino/8/help/en-us/commands/shrinkwrap.htm), filling holes in input objects has no effect for point-cloud input or point inflation mode. Negative offset pulls an inflated result back toward the observations. This does not establish an exact formula for support radius or make a negative offset equivalent to a negative particle radius.

For a genuine Rhino comparison, use the same cropped point cloud and physical units, record the Rhino build, and evaluate the mesh before and after the separate reduction. Compare distances, shape, topology, area and volume as well as triangle count. Approximate target edge length alone does not determine the fit, particularly after optimization/reduction.

The current voxel/SDF prototype's support expansion is +40 mm on a 25 mm grid. It cannot accept a −5 mm compensation offset or reproduce Rhino's smoothing/optimization controls directly. No numerical equivalence is claimed and its defaults have not been silently changed. A future backend should separate support size, final signed offset, spatial resolution and output reduction; this reference gives concrete 15 mm / −5 mm scales to test. See the [research report](shrinkwrap_research.md).
