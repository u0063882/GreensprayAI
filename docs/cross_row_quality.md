# Across-row registration quality

The existing ground-refined transforms were checked using low stem cross-sections, independently of mesh generation. The check reads either the decoded recordings plus transforms, or the 18 already registered PLYs. It applies no new pose correction.

## What a visible gap means

Side A and Side B often see opposite bark faces. Their point centroids therefore should be separated by approximately a trunk diameter. A gap between those returns is not itself a registration error; nearest-neighbour matching can wrongly shrink that diameter.

For each detected candidate in Z=0.12…0.38 m, this check fits a circular cross-section separately to each side and compares the fitted centres. Arc coverage, conditioning, radial residuals, radius agreement, and a 45-replicate acquisition-block bootstrap are used to flag unreliable estimates. The candidates are geometry clusters, not confirmed individual-tree labels; some can be stakes, weeds, or mixed/leaning stems.

## Current result

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
