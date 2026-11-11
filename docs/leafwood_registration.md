# Optional leaf/wood registration and sensor attributes

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

One deterministic ambient NIR threshold is fitted to balanced samples from all
67 original recordings in the four datasets. Raw channels use fixed logarithmic
scaling. This replaces per-scan Lab clustering and is deliberately simple and
inspectable. Higher ambient NIR is provisionally leaf; low NIR is provisionally
wood. No botanical ground truth establishes those labels. Confidence is a score,
not a calibrated probability. The learned threshold is not a universal OS0
calibration: different illumination, season or sensor settings warrant review.
The saved model can be reused with `classify_leafwood` without fitting any scans.

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
