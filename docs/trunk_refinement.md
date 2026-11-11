# Final trunk/post refinement

This is an independent refinement of already registered point clouds. It keeps
the previous registration as its starting point and uses one bounded rigid
correction per original stationary recording. It needs registered samples,
scanner origins/source IDs, rough trunk-window seeds and reference size ranges.
It imports no decoding or meshing code.

The current collection applies accepted refinements to **Spindle 1 and Spindle 2**.
Their `final_registered.ply`, separate contributions and `transformations.json`
all use the refined poses. Chandelier and V-hedge retain their previous primary
registrations because the proposed refinements trigger checks. Their candidates
and before/after figures remain in QC. The existing chandelier mesh is unchanged
and still corresponds to its retained primary point cloud.

## Manual references

The user confirmed that `RefManualMeasurements.xlsx` gives representative sizes,
with **no exact mapping** between workbook tree numbers and point-cloud windows.
Its values/definitions are preserved in
[`presets/manual_measurements.json`](../presets/manual_measurements.json).
The spreadsheet can be imported with `tools/import_manual_measurements.py`;
Office metadata and executable content are not carried into the parsed data.

| Training system | Trunk circumference at 30 cm | Equivalent trunk diameter | Large support circumference |
| --- | ---: | ---: | ---: |
| Spindle | 41.0–47.6 cm | 13.1–15.2 cm | 21.6–24.1 cm |
| Chandelier | 44.1–46.3 cm | 14.0–14.7 cm | 26.5–28.3 cm |
| V-hedge | 45.0–54.0 cm | 14.3–17.2 cm | 21.0–25.9 cm |

The workbook warns that trunk circumference can include loose netting. `C/pi`
also gives only a circular-equivalent diameter for a noncircular post. The model
therefore uses population sizes as **soft priors**, not exact radii imposed on
every tree. Trunk radii are learned from the reference-side training geometry
with a prior; each supported stem has its own radius. Slant and taper are fitted
over the low trunk. No points are rescaled or warped to fit these shapes.

## Method

1. Detect low-trunk observations per source recording from **15–24 and 36–45 cm**
   height. Candidate surfaces must face their supplying scanner and have enough
   angular and vertical support. Along-row centre consensus helps distinguish
   the trunk from neighbouring wood while retaining the existing row alignment.
2. Find optional smaller support candidates outside the trunk surface. Use them
   only when their trunk-relative location recurs across recordings and sides.
   An absent or uncertain post is not invented. Colour alone cannot distinguish
   trunk from wooden post.
3. Assemble/refine the first side, anchoring one well-supported original scan.
   The allowed correction is across translation plus yaw about a local row
   pivot. Then freeze its shared landmarks and fit each opposite recording
   independently. A source with insufficient object support stays unchanged.
4. Allow up to **8 cm** across correction on the reference side, **15 cm** on the
   opposite side and **3 degrees yaw**. Yaw is locked when a recording lacks
   observations spanning enough along-row distance to establish it. Priors
   discourage unnecessary motion. Height, roll, pitch and scale stay unchanged.

The old colour-only refinement's 2.5 cm across limit and 3 mm width-reduction veto
are not used here. Closing a false gap can correctly narrow the combined cloud.
The new checks use withheld object agreement, known-size association, ground
consistency and preservation of along-row alignment.

For MATLAB-style column-vector matrices, with local row pivot `p0`:

```matlab
R = [cos(theta), -sin(theta), 0; ...
     sin(theta),  cos(theta), 0; ...
     0,           0,          1];
t = p0 - R*p0 + [0; dy; 0];
C = [R,t; 0,0,0,1];
Tfinal = C*Tbaseline;
Pfinal = Pbaseline*R.' + t.'; % N-by-3 samples, metres
```

## Validation and limits

The **26–34 cm band is excluded from fitting**, so the 30 cm figures show unused
geometrical observations. Exactly the same independently detected source/object
observations and A/B pairing identities are compared before and after. The size
prior and object identity remain assumptions: these are conditional internal
consistency checks on shared acquisitions, not independent survey accuracy.

Acceptance requires at least 20 held-out cross-side centre pairs, median across
discrepancy improvement of both 20% and 2 mm, and no well-supported object worsening
by over 5 mm. A well-supported object has at least two held-out views from each
side. Limited-view objects remain in the report and plots, marked uncertain;
some end windows have weak evidence even when the dataset passes overall.

Additional checks reject correction-bound saturation, ground median/RMS worsening
over 5 mm, along-row displacement RMS above 2 cm, and held-out along-centre
discrepancy worsening over 5 mm. Unsupported recordings remain unchanged and are
listed. Exact sample counts, source ordering, all non-XY fields, rigid matrices
and composed transforms are checked during export. Z is bit-preserved.

The synthetic test includes distinct trunks/posts, varying true radii and known
source pose errors exceeding the old 2.5 cm bound. It checks held-out known-centre
alignment, rigidity and unchanged heights. Export tests also verify that a later
rebuild starts from the preserved baseline and does not apply corrections twice.

## Results and files

See the [comparison report](trunk_refinement_report.md). In `90_QC.zip`, each
dataset has `QC/<dataset>/trunk_refinement/report.json`, correction matrices and
before/after `slices_18cm`, `slices_30cm`, `slices_40cm` PNG/PDF figures. Raw points
are retained in both panels, including clutter and supports; no favourable
post-correction crop is used to improve the plot. The summary is at
`QC/trunk_refinement_summary.png/.json`.

The original primary PLY and transforms are preserved exactly in
`QC/<dataset>/provenance/trunk_refinement_baseline.zip`, together with original
per-recording XYZ arrays, manifests and the colour model. Current contributions
keep their raw sensor channels and source ordering. The baseline XYZ arrays can
therefore restore pre-refinement separate contributions. Rebuilding always uses
that archive, so repeated publishing cannot accumulate the same correction.

Accepted results keep the existing canonical download paths, with no extra
top-level asset list. Raw `intensity`/`signal`, ambient NIR, reflectivity,
shared Lab leafWoodSignal RGB and original `scan_id` remain intact. The existing
mesh is never silently transformed by a collection of per-recording poses.

The refinement function is
[`captures/trunk_refinement.py`](../captures/trunk_refinement.py); collection
orchestration/export is separate in `tools/refine_trunk_collection.py`. To apply
the independent step to an extracted collection stage:

```sh
python tools/refine_trunk_collection.py --stage /path/to/collection_stage
```
