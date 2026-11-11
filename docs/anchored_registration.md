# Fixed east side, individually bounded west recordings

The current registration keeps the completed east/Side A solution fixed and gives each of the nine west/Side B recordings its own bounded rigid correction. RTK longitude confirms the side labels for this capture. This replaces the former assumption that the completed west side could only move as one aggregate.

East is the already registered nine-recording seed solution, including its relative-ground refinement. The new stage does not claim to independently recalibrate that anchor. In the current exports, all nine east PLY files are byte-identical to their seed inputs.

## Restored degrees of freedom

A single rigid correction for all west recordings preserves every relative error inside that side. The new stage optimizes **nine separate six-component pose updates (54 variables)**. Bounds and soft overlap/pose priors regularize the weak directions; they do not assert that every component is independently observable from the measured stems.

Trees supply geometry constraints. The unit moved is a whole stationary recording: measured distances between trees within one acquisition are preserved. There is no per-tree deformation or independent motion of pieces from the same scan.

Each correction rotates about a local row pivot, then translates:

| Component | Bound |
| --- | ---: |
| Along-row translation | ±4 cm |
| Across-row translation | ±15 cm |
| Height translation | ±2.5 cm |
| Tilt rotation-vector components | ±0.5° |
| Yaw rotation-vector component | ±2° |

The translation bounds apply at that pivot, not at the coordinate origin or GPS antenna. Current corrections use roughly 2.5–7.4 cm across, with all normalized pose components below 53% of their bounds. The exported transformation matrices are authoritative; the legacy seed pose cache remains an initializer.

## Reference geometry and insertion

1. Assemble the fixed east measurements. Fit robust leaning, tapered circular-axis stem models from Z=0.12…0.48 m. Poor support, high residuals or parameters at the model bounds reject a candidate. Four of the seven coarse windows are usable in this capture; their IDs are not verified biological tree identities.
2. Build local ground planes from robust east ground-cell heights. Terrain is retained rather than flattened.
3. Insert west recordings in order of available low-stem support. Every recording has its own bounded state and prior to its seed pose.
4. Refine all west states jointly using east stem geometry, ground, actual same-face west overlap and soft relative-pose constraints. Joint refinement reduces dependence on insertion order without locking west into one rigid block.
5. Validate and export. Failed acceptance checks prevent cloud export/publication.

The coarse windows in [`presets/orchard_stem_candidates.json`](../presets/orchard_stem_candidates.json) initialize searches. Their coordinates are not fixed target centres: stem shapes are fitted from east measurements. Unreliable candidates are excluded from the registration objective but remain visible in the circle diagnostics.

A stem model uses an inclined axis and radius varying along it. It therefore supports lean and taper without pooling all heights into one vertical circle. Radial distance alone has a hemisphere ambiguity: a point on the wrong bark face can still have zero surface distance. A soft visibility penalty uses each supplying scanner's actual pose to discourage back-facing matches. This is a surface-normal/view-direction constraint, not full ray tracing or a complete occlusion model; clutter and non-circular stems remain limitations.

All fitting stem observations remain below 0.5 m, including after transformation; the maximum transformed fitted height in this run is below 0.46 m. Hanging branches and canopy are excluded from this optimization. The full-height measured cloud is still exported for independent meshing.

## Validation and current result

West data for candidate 5 are excluded from the stem objective and west-overlap constraints. The disjoint height bands Z=0.20…0.26 and 0.34…0.40 m are also withheld. Checkerboard ground patches and same-face west overlap matches provide additional checks.

| Check | Seed | Bounded east-anchored result |
| --- | ---: | ---: |
| Excluded height bands: median absolute radial residual | 4.47 cm | 1.55 cm |
| Excluded stem: median absolute radial residual | 5.74 cm | 2.21 cm |
| Withheld west overlap: point/plane RMS | 1.35 cm | 1.11 cm |
| Withheld ground: median absolute plane residual | 8.2 mm | about 8.8 mm |

These compare measured points with fixed fitted reference geometry; they are not independently surveyed accuracy. Held-out heights/features share acquisitions and may remain correlated. Freezing east preserves any remaining error in that anchor, and the circular tapered-stem model may be biased by root flare, non-circular bark, weeds or stakes. The full reports retain per-stem results and scanner-facing statistics; uncertainty has not been removed from the circle plots.

Acceptance requires solver convergence, no pose approaching 95% of a bound, fitted heights below 0.5 m, at least 20% median improvement in both withheld stem/height checks, limited ground degradation (2 mm), preserved withheld west overlap (2 mm overall RMS tolerance and 1 cm per-pair tolerance), no stem median degradation over 5 mm, and limited scanner-facing degradation. The current result passes these internal checks. The export validator also verifies exact preservation of east files, every west rigid transform, measured attributes, crop, point counts, transformation metadata and ZIP integrity.

The merged result contains 1,219,008 measured samples. Separate contributions contain 1,739,386 points with overlaps retained. The original 7.1 m × 1.4 m crop is reapplied; some transformed boundary points leave the strip. Because the seed PLYs were already cropped, this stage cannot recover returns omitted by the original crop. It does not invent points or assign biological tree identities.

## Run and inspect

```sh
python processing/register_anchored.py --scan-dir /path/to/seed/scans --transforms /path/to/seed/scan_transforms.json --export
python tools/validate_registered_result.py --baseline-scan-dir /path/to/seed/scans --baseline-transforms /path/to/seed/scan_transforms.json --result-dir output/anchored_registration
```

Without `--export`, the command writes a candidate comparison and quality report. `--export` additionally writes the three cloud ZIPs only if validation passes. This is a capture-specific profile for 18 supplied recordings, not an arbitrary-capture registration service.

`run_pipeline.py` builds the acquisition/IMU/RTK seed, then performs this final bounded stage and promotes validated outputs into `output/`. `--export-only` reuses the legacy seed caches and repeats one bounded stage; `work/registered/anchored_poses.npy` stores the resulting levelled-to-world poses. `output/scan_transforms.json` contains current raw-sensor-to-world and levelled-to-world matrices. Already registered PLYs must not have those transforms applied again.

For processing changes, the release workflow starts from immutable checksummed historical seed ZIPs, runs the same bounded stage and validates it, then publishes the compact downloads. Packaging-only changes reuse the validated release bytes. Mesh generation is a separate program that subsequently consumes the final point-cloud file; this registration stage never imports or runs a mesher.

[Before/after circles — `QC/registration/anchored_trunk_sections.png`](https://github.com/u0063882/GreensprayAI/releases/download/current/90_QC.zip) · [Pose and stem comparison — `QC/registration/anchored_registration_comparison.png`](https://github.com/u0063882/GreensprayAI/releases/download/current/90_QC.zip) · [Full quality report](../results/anchored_registration_quality.json) · [Current downloads](https://github.com/u0063882/GreensprayAI/releases/tag/current)
