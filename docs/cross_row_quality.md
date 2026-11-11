# Across-row registration quality

The current result uses [fixed east geometry and separate bounded west recording poses](anchored_registration.md). It replaces the former single aggregate cross-side restriction. All nine east files remain unchanged; each west recording has one rigid correction. The QA below diagnoses the resulting registered clouds and applies no additional correction.

## Bark faces and fitted centres

East and west generally see different bark faces. Those surfaces should remain separated by a real trunk diameter. Direct nearest-neighbour matching of opposing faces can shrink that diameter. The final optimizer instead uses fitted east stem geometry, scanner-facing consistency, local ground and measured west-to-west overlap.

The independent circle plots provide a further check, conditional on circular cross-sections. They show both sides' observed points, fitted circles and estimated centres at matched heights. Trunk lean remains visible because every height uses the same fixed candidate reference and equal XY scales. An inclined/non-circular/root-flared stem, weeds or stakes can bias partial-view fits; candidate IDs are not confirmed biological tree labels.

## Only low slices

All complete diagnostic slices remain below Z=0.5 m. Default centres are 0.14, 0.18, 0.22, 0.26, 0.30, 0.34, 0.38, 0.42 and 0.46 m, with thickness 6 cm. Adjacent slices overlap by 2 cm and are correlated; they do not count as independent trees. Z is the shared registered height datum, approximately above ground, rather than separately normalized per side.

Blue points/circles are east/Side A; orange is west/Side B. A **+** marks each fitted centre; the black segment connects centres rather than bark faces. Solid/green comparisons pass the circle diagnostic gates; dashed circles and hollow markers are uncertain. The plot includes all points in each coarse window so clutter and model failures remain visible.

## Current comparison

At the candidate near X=3.01 m, B-minus-A across-centre differences change approximately as follows:

| Slice centre Z | Seed registration | East-anchored result |
| --- | ---: | ---: |
| 0.18 m | +8.7 cm | +2.0 cm |
| 0.26 m | +8.4 cm | +0.9 cm |
| 0.34 m | +11.3 cm | +3.4 cm |

The current 63-section check has nine comparisons passing its strict gates, from four candidate IDs. Remaining dashed fits, including several far-end sections, remain uncertain; this is not a claim that all trunks have zero residual or that candidate identities are established. The [bounded-stage report](../results/anchored_registration_quality.json) separately validates excluded heights, an excluded stem, ground and west overlap, and records every correction and bound.

Circle gates include at least 30 points and two acquisitions per side, arc coverage >100°, condition <150, radial RMSE <18 mm, at least 55% radial inliers within 22 mm, no fitted parameter at its search bound, radius agreement within 35 mm, and an across-centre acquisition-bootstrap interval narrower than 40 mm. The 45 replicates resample whole recordings. Intervals describe sensitivity conditional on the model, not survey accuracy.

The earlier trial common sideways shift was inadequate because residuals varied along the row. The current `low_trunk_shift_check.png` is an extra common-shift diagnostic **on the already refined result**; it applies no further pose change. Its `automatic_correction_applied=false` fields describe that diagnostic, not the final bounded registration stage, whose applied transforms are in `scan_transforms.json`.

## View and download

- [Before/after low circles — `QC/registration/anchored_trunk_sections.png`](https://github.com/u0063882/GreensprayAI/releases/download/current/90_QC.zip).
- [Per-recording corrections and stem comparison — `QC/registration/anchored_registration_comparison.png`](https://github.com/u0063882/GreensprayAI/releases/download/current/90_QC.zip).
- [All current low sections — `QC/trunk_slices/trunk_slice_fits.png`](https://github.com/u0063882/GreensprayAI/releases/download/current/90_QC.zip) and [detailed PDF — `QC/trunk_slices/trunk_slice_fits.pdf`](https://github.com/u0063882/GreensprayAI/releases/download/current/90_QC.zip).
- [Centre trajectories versus height — `QC/trunk_slices/trunk_slice_centres.png`](https://github.com/u0063882/GreensprayAI/releases/download/current/90_QC.zip).
- [Complete low-slice bundle — `QC/trunk_slices/`](https://github.com/u0063882/GreensprayAI/releases/download/current/90_QC.zip).
- [Current JSON](../results/trunk_slice_quality.json), [CSV](../results/trunk_slice_centres.csv), and [coarse pooled-band check](../results/cross_row_quality.json).

```sh
python processing/check_cross_row.py --scan-dir /path/to/current/scans
python processing/plot_trunk_slices.py --scan-dir /path/to/current/scans --candidate-report presets/orchard_stem_candidates.json
# Custom sampling, still wholly below 0.5 m:
python processing/plot_trunk_slices.py --scan-dir /path/to/current/scans --candidate-report presets/orchard_stem_candidates.json --heights 0.15 0.25 0.35 0.45 --thickness 0.04
```

Positive deltas mean west/B minus east/A in the Python frame, X along and Y across. The MATLAB toolbox has along=+Y and across=−X, so the across sign reverses. No canopy points above 0.5 m contribute to this final alignment or these low-slice diagnostics; the full-height cloud remains the independent meshing input.

[Before and after bounded per-recording registration — inside QC bundle: `QC/registration/anchored_trunk_sections.png`](https://github.com/u0063882/GreensprayAI/releases/download/current/90_QC.zip)
