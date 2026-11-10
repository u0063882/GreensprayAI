# Across-row registration quality

The existing ground-refined transforms were checked using low stem cross-sections, independently of mesh generation. The check reads either the decoded recordings plus transforms, or the 18 already registered PLYs. It applies no new pose correction.

## What a visible gap means

Side A and Side B often see opposite bark faces. Their point centroids therefore should be separated by approximately a trunk diameter. A gap between those returns is not itself a registration error; nearest-neighbour matching can wrongly shrink that diameter.

For each detected candidate in Z=0.12…0.38 m, this check fits a circular cross-section separately to each side and compares the fitted centres. Arc coverage, conditioning, radial residuals, radius agreement, and a 45-replicate acquisition-block bootstrap are used to flag unreliable estimates. The candidates are geometry clusters, not confirmed individual-tree labels; some can be stakes, weeds, or mixed/leaning stems.

## Earlier pooled-band result

Seven candidate sections were found. Only one passes the current usability gates; its fitted centre separation is about 4 cm across the row. Other fits have poorly constrained radii/centres or broad acquisition-bootstrap uncertainty. This is insufficient independent evidence to infer a reliable global translation or yaw adjustment. A circular-trunk model can itself be biased by non-circular bark, taper, lean and occlusion.

Positive deltas below mean B minus A in the **Python** frame (X along, Y across). In the MATLAB toolbox convention, along is +Y and across is −X, so the across sign reverses.

| Along position (m) | Centre Δalong (cm) | Centre Δacross (cm) | Across bootstrap 95% interval (cm) | Status |
| ---: | ---: | ---: | --- | --- |
| 0.06 | 1.1 | 4.0 | [1.9, 5.5] | usable with model assumptions |
| 1.05 | -0.9 | 3.7 | [-0.5, 8.2] | uncertain |
| 1.63 | 13.0 | -4.1 | [-6.7, 10.8] | uncertain |
| 3.01 | -0.4 | 8.5 | [5.0, 10.6] | uncertain |
| 3.69 | -0.4 | 7.2 | [2.8, 8.4] | uncertain |
| 4.55 | 2.4 | 6.6 | [2.5, 7.3] | uncertain |
| 5.95 | 3.9 | -0.2 | [-3.9, 1.4] | uncertain |

The bootstrap intervals quantify sensitivity to the acquisitions available here, conditional on the circular model; they are not a survey-accuracy guarantee. In particular, a usable fit is not enough to validate a whole-row correction.

[Cross-sections plot](https://github.com/u0063882/GreensprayAI/releases/download/current/cross_row_trunk_sections.png) · [Machine-readable report](../results/cross_row_quality.json) · [Candidate table](../results/cross_row_trunks.csv)

```sh
python processing/check_cross_row.py
# Or, with the extracted separate-cloud download:
python processing/check_cross_row.py --scan-dir /path/to/extracted/scans
```

A stronger correction should use several independent trunk sections at several heights, with consistent centre residuals, or common fixed targets/overlapping bark features visible from both sides. Current point-cloud transforms are retained while this remains unvalidated. Meshes inherit this registration uncertainty.

## Thin slices below 0.5 m

Only low trunk sections are used for alignment diagnostics. The full extent of **every** slice must remain below Z=0.5 m; even a slice centred below that limit is rejected if its upper edge reaches it. Default centres are Z=0.14, 0.18, 0.22, 0.26, 0.30, 0.34, 0.38, 0.42 and 0.46 m, with thickness 6 cm. Thus the highest measured band ends at 0.49 m. Hanging branches above this limit cannot contribute to fits or the shift proposal.

The 2 cm overlap between adjacent slices is intentional for inspecting continuity. Slices are correlated and **never count as independent trees**. This restriction applies to alignment diagnostics only: the full-height point cloud remains available to independent meshing.

Each side has an independent circle centre and radius at each matched height. There is no vertical-cylinder assumption or forced common centre. Blue points/circles are Side A; orange is Side B. A **+** marks a fitted centre, and a black segment connects the two centres. Solid circles pass the diagnostic gates; dashed circles are uncertain. Uncertain comparisons have hollow markers on the height plot. All measured points in the candidate window remain visible, including clutter.

Download from **Current results → Assets**:

- [All low cross-sections: seven candidate rows × nine height columns](https://github.com/u0063882/GreensprayAI/releases/download/current/trunk_slice_fits.png).
- [Detailed candidate pages, PDF](https://github.com/u0063882/GreensprayAI/releases/download/current/trunk_slice_fits.pdf).
- [Example near X=3.01 m](https://github.com/u0063882/GreensprayAI/releases/download/current/candidate_04_slices.png).
- [Centres and B-minus-A displacement versus height](https://github.com/u0063882/GreensprayAI/releases/download/current/trunk_slice_centres.png).
- [Trial common shift and held-out-candidate check](https://github.com/u0063882/GreensprayAI/releases/download/current/low_trunk_shift_check.png).
- [Complete bundle: PNGs, PDF, reports and CSV](https://github.com/u0063882/GreensprayAI/releases/download/current/trunk_slice_diagnostics.zip).

XY positions are centimetres from each candidate's **fixed low-stem reference**. Equal XY scales and unchanged references across heights preserve visible lean. Centre-versus-height connecting lines guide the eye; they are not fitted trunk axes. Heights are absolute Z in the shared registered frame, approximately above ground, rather than independently normalized per side. Fixed windows are not validated trunk tracking; inclined/non-circular stems, root flare, weeds and support hardware can bias partial-view circular fits.

### Current low-slice result

Of 63 candidate-height sections, three pass all strict gates, from two distinct candidates: candidate 1 at Z=0.18 m, and candidate 4 at Z=0.18/0.26 m. Their B-minus-A across-centre differences are approximately +3.6, +8.7 and +8.4 cm respectively. Other sections remain useful to inspect but are uncertain, often because acquisition-bootstrap intervals are wide. Multiple heights on candidate 4 still represent one candidate, not multiple independent trees.

Gates include at least 30 points and two acquisitions per side, arc coverage >100°, condition <150, radial RMSE <18 mm, at least 55% radial inliers within 22 mm, no parameter at its search bound, radius agreement within 35 mm, and an across-centre bootstrap interval narrower than 40 mm. The 45-replicate bootstrap resamples acquisitions, not individual returns. Its intervals describe sensitivity conditional on the circle model; passing does not establish survey accuracy or prove that a single trunk was isolated.

The [JSON](../results/trunk_slice_quality.json) and [CSV](../results/trunk_slice_centres.csv) retain absolute centres, radii, counts, conditioning, residuals, bootstrap intervals and each failure reason.

### Test of a common sideways correction

`assess_low_trunk_shift.py` tests a **pure across-row translation of Side B**, preserving along-row placement, height and rotation. Each candidate contributes one median across its geometry-qualified low slices. Wide bootstrap intervals can inform this exploratory proposal, but remain uncertain and cannot make it accepted. Candidates need at least two geometry-qualified slices; all geometry failures are retained as exclusions. This is not a fit of raw opposite bark surfaces onto each other.

The proposal is −6.53 cm across. It reduces in-sample candidate-centre RMS from 6.72 to 3.42 cm. Leaving each candidate out of estimation gives RMS 3.98 cm, but the held-out far-end candidate worsens by 7.22 cm. For the single shared proposal, that candidate's centre difference moves from −0.56 to −7.09 cm. In addition, only two distinct candidates pass the strict slice gates. **The common shift is rejected and has not been applied.** These results describe the fitted centres, not independent physical error measurements.

Acceptance requires at least three strictly usable candidates, at least 20% improvement in leave-one-candidate-out RMS, no held-out candidate worsening by over 1 cm, and translation within 15 cm. Passing would still require validation against the actual cloud and ground geometry before applying a correction. The present result does not pass; no points, poses or meshes have changed.

[Shift report](../results/low_trunk_shift_quality.json). The variation along the row supports investigating individual recording poses with joint low-stem/ground constraints, or verifying candidate identities and circular-model bias, rather than translating an entire side from a few selected sections.

```sh
python processing/check_cross_row.py --scan-dir /path/to/extracted/scans
python processing/plot_trunk_slices.py --scan-dir /path/to/extracted/scans --candidate-report output/cross_row_quality.json
# Custom low bands, still wholly below 0.5 m:
python processing/plot_trunk_slices.py --scan-dir /path/to/extracted/scans --candidate-report output/cross_row_quality.json --heights 0.15 0.25 0.35 0.45 --thickness 0.04
```

Outputs go to `output/trunk_slices/` and `output/trunk_slice_diagnostics.zip`. The registration/export command and release workflow also produce these diagnostics from registered scans, independently of meshing.

![Low-trunk common-shift test](https://github.com/u0063882/GreensprayAI/releases/download/current/low_trunk_shift_check.png)
