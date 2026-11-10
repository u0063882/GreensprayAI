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

## Thin slices at several heights

The new diagnostic fits **Side A and Side B separately in the same 6 cm-thick Z slices**, centred at Z=0.18, 0.35, 0.55, 0.75, 1.00 and 1.25 m. This avoids pooling 26 cm of a leaning stem into one section. Each side has an independent centre and radius at each height: there is no vertical-cylinder assumption, forced common centre or registration correction.

Finished plots and tables are in the same **Current results** release:

- [All cross-sections: seven candidate rows × six height columns](https://github.com/u0063882/GreensprayAI/releases/download/current/trunk_slice_fits.png).
- [Detailed candidate pages, PDF](https://github.com/u0063882/GreensprayAI/releases/download/current/trunk_slice_fits.pdf).
- [Example: candidate near X=3.01 m](https://github.com/u0063882/GreensprayAI/releases/download/current/candidate_04_slices.png).
- [Centre position and B-minus-A displacement versus height](https://github.com/u0063882/GreensprayAI/releases/download/current/trunk_slice_centres.png).
- [Complete bundle: per-candidate PNGs, PDF, plots, JSON and CSV](https://github.com/u0063882/GreensprayAI/releases/download/current/trunk_slice_diagnostics.zip).

Blue points/circles are Side A; orange is Side B. A **+** marks each fitted centre, and the black segment connects the two centres. Solid circles pass the diagnostic gates; dashed circles are uncertain. Uncertain centre comparisons are hollow on the height plot. The plots show all measured points in the selected candidate window, including clutter, so a poor model remains visible.

XY positions are centimetres from a **fixed low-stem reference for each candidate**. The reference does not move between heights, and each candidate's cross-section panels use the same limits and equal aspect ratio. A leaning centre therefore moves across successive panels. The centre-versus-height figure separately plots the along and across positions for each side, plus the B−A across displacement and its 95% acquisition-bootstrap interval. Connecting lines only guide the eye; they are not fitted trunk axes.

Heights are absolute Z in the shared registered frame, approximately above ground. They are not independently normalized to local ground on either side, which would accidentally compare different physical sections. The fixed candidate windows are not a validated trunk tracker: strong lean leaving the window, bifurcation, weeds and support hardware can invalidate the circle assumption. Horizontal cuts through inclined cylinders are generally elliptical, so partial-view circle fits can have model bias even with small residuals.

### Result of the thin-slice check

Of 42 candidate-height sections, three pass all gates, from only two distinct candidates:

| Candidate / base along X | Slice centre Z | Centre B−A along | Centre B−A across | Across bootstrap 95% interval |
| --- | ---: | ---: | ---: | --- |
| 1 / 0.06 m | 0.18 m | +2.5 cm | +3.6 cm | [+0.9, +4.7] cm |
| 4 / 3.01 m | 0.18 m | −0.6 cm | +8.7 cm | [+7.3, +10.4] cm |
| 4 / 3.01 m | 0.35 m | +0.5 cm | +11.4 cm | [+9.0, +12.7] cm |

These are fitted **centre differences**, not distances between opposite bark faces. Candidate 4 supports investigating a remaining across-row mismatch. Its two heights are still one candidate, not two independent trees. The higher circles frequently follow branches or mixed/cluttered sections and are marked uncertain. This does not establish a valid whole-row correction or a definitive lean angle.

Gates include at least 30 points and two contributing acquisitions per side, arc coverage >100°, normal-equation condition <150, radial RMSE <18 mm, at least 55% radial inliers within 22 mm, no fitted parameter at the search bound, radius agreement within 35 mm, and an across-centre bootstrap interval narrower than 40 mm. The 45-replicate bootstrap resamples acquisitions, not individual correlated returns. It measures acquisition sensitivity conditional on the model; neither the interval nor passing these gates proves a single circular trunk was isolated.

The [full JSON snapshot](../results/trunk_slice_quality.json) and [CSV](../results/trunk_slice_centres.csv) include absolute centres and radii, sample counts, arc coverage, conditioning, residuals and the individual uncertainty reasons. No point cloud, registration transform or mesh was modified.

```sh
python processing/check_cross_row.py --scan-dir /path/to/extracted/scans
python processing/plot_trunk_slices.py --scan-dir /path/to/extracted/scans --candidate-report output/cross_row_quality.json
# Optional different sampling:
python processing/plot_trunk_slices.py --scan-dir /path/to/extracted/scans --candidate-report output/cross_row_quality.json --heights 0.15 0.25 0.35 0.45 0.60 --thickness 0.04
```

Outputs go to `output/trunk_slices/` and `output/trunk_slice_diagnostics.zip`. These diagnostics also run after the capture-specific registration/export command, and the release workflow reproduces them directly from the exported registered scans.

![Independent circles and estimated centres at six heights, candidate 4](https://github.com/u0063882/GreensprayAI/releases/download/current/candidate_04_slices.png)
