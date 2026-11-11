# Optional leaf/wood registration and sensor attributes

This documents the earlier colour-only comparison on the pre-refinement
baseline. Spindle 1 and Spindle 2 now receive a separate accepted
[final trunk/post refinement](trunk_refinement.md). Their original baselines and
this historical experiment remain preserved in QC; the shared Lab colour model
and raw sensor fields are unchanged by the final geometric step.

The original geometry-only registration is retained. The independent function
[`register_leafwood`](../captures/register_leafwood.py) accepts already registered
point arrays, per-point wood scores, scanner origins, side labels and low-stem
window seeds, all in the same metric frame. It returns candidate rigid correction
matrices and a validation report. It imports no meshing code and changes no inputs.
This is a bounded refinement of the baseline, not a separate initial alignment.

```python
from captures.register_leafwood import register_leafwood

corrections, report = register_leafwood(
    clouds, wood_scores, scanner_origins, side_labels, stem_seeds)
# Candidate coordinates: p @ corrections[i][:3, :3].T + corrections[i][:3, 3]
# Inspect report['validation_passed'] and ['rejection_reasons'] before use.
# The collection publisher always keeps the delivered baseline as primary.
```

The assembled first side is immutable. Each opposite-side recording can move by
its own bounded SE(3). Tree fitting stays below 0.5 m, with separate unused height
bands and spatial patches. Same-side overlaps require close positions and aligned
scanner-oriented normals; soft class compatibility favours wood-to-wood matches.
An uncertain label never deletes the point. Opposite bark hemispheres use fixed
leaning stem surfaces and visibility constraints, not tree point-to-point matches.
Local ground and small pose priors constrain the remaining degrees of freedom.
There is no scale, per-point adjustment or deformation. Measured low-stem widths
are checked conservatively; a width decrease can indicate either a corrected gap
or unwanted narrowing, so flagged candidates remain experimental.

One Toolbox-style Lab model is fitted to balanced paired SIGNAL/NEAR_IR samples
from all 67 original recordings in the four datasets. Its scheme-13 feature
definition retains the original fixed logarithmic channel scaling, nonlinear NIR
hue mapping, SignalPhoton lightness, gamut clipping and displayed Lab a*/b*
features. One pair of k-means centres is shared across all scans. The cluster with
higher mean raw NIR is provisionally leaf. This replaces the earlier NIR-only
threshold, which mislabelled substantial foliage as wood. No botanical ground
truth establishes either method's labels. Confidence is a heuristic score.
Different illumination, season or sensor settings warrant review. The saved
model is reused by `classify_leafwood` without fitting individual scans.

Display RGB follows Toolbox `leafWoodSignal` green/brown hue and saturation, with
brightness from fixed-scaled raw `SIGNAL`. OS0 does not supply camera RGB. The
shared classifier changes labels relative to Toolbox's stochastic per-scan k-means;
we cannot recover exact previously displayed MATLAB colours that were never saved.
Once derived, the RGB travels unchanged through either registration method.

See the [measured comparison report](leafwood_comparison_report.md). It includes
the delivered baseline, an additional geometry-only control using the identical
refinement with semantics disabled, and the NIR-guided candidate. The same fixed
geometry-only validation matches and measured samples evaluate all three.
Residuals measure internal consistency, not absolute accuracy against a survey.
Neither improved fitting loss nor a passed gate automatically promotes a candidate.

Spindle 1 and Spindle 2 include dimensioned raw slices at 18, 28 and 40 cm above
the local ground datum, each 2 cm thick, in
`QC/<dataset>/leafwood_comparison/trunk_slices_dimensioned.png` and `.pdf`.
They show final **baseline** geometry, not experimental corrections. Axes and
diameters are centimetres; A is blue, B orange. Fits are independent per side.
Dashed circles indicate uncertain support/agreement. Even a supported circle
is a diagnostic model, not a verified physical trunk diameter: mixed returns,
short arcs, supports, basal branches and residual registration can bias it.
No foliage filter removes inconvenient points from these figures.

The user subsequently supplied field measurements at **30 cm** above ground:
trunk circumference **40–45 cm**, equivalent circular diameter **12.7–14.3 cm**;
adjacent support post circumference **22 cm**, circular-equivalent diameter
**7.0 cm**. The post need not have a circular section. Both objects are woody,
so leaf/wood colour alone does not distinguish them. A single-circle fit can
combine trunk, guard and post returns, in addition to residual scan disagreement.
The new `QC/<dataset>/trunk_reference_30cm/raw_slices.png/.pdf` compares unfiltered
2 cm slices at the measured height against separate size-reference diagrams.
Reference circle positions are arbitrary, not inferred trunk/post locations.
These dimensions are recorded in `presets/spindle_trunk_reference.json` and do
not force radii, alter cloud scale or trigger new registration.

## PLY fields

| Field | Meaning |
| --- | --- |
| `x`, `y`, `z` | Already registered measured coordinates, metres |
| `red`, `green`, `blue` | Derived leafWoodSignal in primary and separate clouds |
| `intensity`, `signal` | Identical original first-return SIGNAL, unsigned 16-bit; MATLAB SignalPhoton |
| `near_ir` | Original ambient NEAR_IR, unsigned 16-bit |
| `reflectivity` | Original calibrated first-return reflectivity, unsigned 8-bit |
| `scan_id` | Original PCAP recording index within the dataset, unsigned 32-bit |
| `frame_id`, `pixel_index` | Original sensor frame and row-major raw H×W pixel index |
| `leafwood_class` | Provisional 1 wood / 2 leaf; 0 reserved for unknown |
| `wood_score`, `leafwood_confidence` | Shared classifier scores |
| `leaf_red`, `leaf_green`, `leaf_blue` | Preserved derived RGB, also present in scan-coloured views |
| `classification` | Legacy height-band code; not semantic ground truth |

The first sensor return matches the Python baseline. MATLAB's strongest-return
selection can choose a different return. No raw intensity is normalized, averaged
or replaced by a visual brightness value. Downsampling retains measured samples
and all attributes from their supplying returns together. `source_scans.json`
beside each final PLY maps indices to original files and SHA-256 hashes;
`leafwood_model.json` stores the shared classifier. The 91 ZIP has separate
leaf/wood-coloured recordings and a combined recording-coloured cloud, retaining
overlaps and all channels. In CloudCompare, select `scan_id` as the displayed
scalar field to colour a final PLY by recording without altering its stored RGB.
Generic PLY readers may ignore custom properties; use the included
`processing.export_anchored.read_records` to retain all fields in Python.

## Reproduction and publication

`tools/compare_leafwood.py --stage COLLECTION_STAGE --work EXPERIMENT_CACHE
--data-map INPUT_PATHS.json` runs independently against an extracted collection.
The data map associates each dataset ID with its directory containing
`sideA/` and `sideB/` original PCAP/JSON/H5 files. It recovers attributes using
the exact original levelling matrix and first-measured 10 mm sample selection,
then checks source positions (maximum discrepancy 3 micrometres) and exact raw
reflectivity before matching attributes. Each primary point is matched only
within its original scan, never to a neighbouring scan's sample.

The publisher archives raw inputs, verifies hashes, and enriches the existing
primary files while asserting identical float coordinates and point ordering.
Primary matrices and existing mesh bytes remain unchanged. Experimental PLYs,
candidate matrices, low-stem/colour figures and JSON reports are under
`QC/<dataset>/leafwood_comparison/` in the existing 90 ZIP. No extra release or
top-level asset list is introduced. Subsequent publications reuse verified
enriched outputs until the experiment fingerprint or inputs change.
