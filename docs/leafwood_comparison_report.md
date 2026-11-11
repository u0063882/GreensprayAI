# Leaf/wood registration comparison — draft report

The delivered geometry-only registration remains the primary result. The alternative is an independent optional function that refines that baseline using shared NIR classes. Both methods retain exactly the same measured samples. No mesh is used or regenerated.

## Shared classification

One deterministic model was fitted to balanced paired SIGNAL/NEAR_IR samples from all 67 recordings. It uses Toolbox scheme-13 displayed Lab a*/b* features with the original fixed S/N normalization, nonlinear NIR hue mapping and SignalPhoton lightness. One pair of k-means centres is shared by every recording. The cluster with higher mean raw NIR is provisionally leaf. SignalPhoton controls final green/brown display brightness; no per-scan fitting or brightness rescaling is used.

This supersedes the earlier raw-NIR-only threshold, which labelled too much foliage as wood. It preserves the Toolbox feature definition while sharing a single model; it is not bit-identical to MATLAB’s per-scan stochastic labels. OS0 records NIR/signal/reflectivity, not camera RGB. Export RGB is derived once and then preserved unchanged through registration. Ground/grass can fall into either provisional class; confidence is heuristic and there is no botanically labelled validation set.

## Registration and safeguards

The first assembled side is fixed. Each opposite-side acquisition gets one tightly bounded rigid pose. Same-side overlaps below 0.5 m use distance/normal-gated point-to-plane correspondences with soft class compatibility and wood priority. Cross-side tree correspondences are never nearest-neighbour matches: fixed-radius leaning stem surfaces and scanner-facing constraints retain distinct bark hemispheres. Ground is constrained to local terrain. There is no scale, per-point displacement or deformation.

Unused low-height bands and spatial patches are withheld from the added refinement. Metrics use fixed, geometry-selected validation correspondences and identical measured samples; no rematching or label-dependent filtering improves the reported score. Reference stem models use fixed-side training heights. These are conditional internal consistency checks; the baseline already used the same acquisitions, so this is not independent survey accuracy or a from-scratch algorithm comparison.

A geometry-only control runs the identical extra refinement with semantic matching and weights disabled. This distinguishes the effect of additional optimisation from the effect of NIR information. It uses the same input points, reference models, bounds and validation samples. No hyperparameters were selected using the reported validation scores.

## Results

| Dataset | Check | Points | Delivered baseline median / capped RMS (mm) | Extra geometry-only control (mm) | Leaf/wood candidate (mm) |
| --- | --- | ---: | ---: | ---: | ---: |
| chandelier | withheld stem surface | 2,694 | 14.45 / 20.17 | 13.39 / 18.82 | 12.98 / 17.85 |
| chandelier | withheld ground | 1,094 | 8.04 / 14.73 | 8.10 / 14.75 | 8.17 / 14.78 |
| chandelier | withheld same face overlap | 1,605 | 7.34 / 10.51 | 6.59 / 9.75 | 6.61 / 9.84 |
| spindle1 | withheld stem surface | 1,351 | 23.91 / 26.19 | 22.43 / 25.36 | 21.42 / 25.57 |
| spindle1 | withheld ground | 929 | 12.53 / 19.76 | 12.40 / 19.67 | 11.96 / 19.64 |
| spindle1 | withheld same face overlap | 663 | 6.78 / 10.44 | 7.58 / 10.48 | 7.78 / 10.58 |
| spindle2 | withheld stem surface | 1,714 | 22.23 / 25.90 | 20.76 / 24.84 | 19.33 / 24.08 |
| spindle2 | withheld ground | 927 | 9.65 / 16.09 | 9.59 / 15.95 | 9.33 / 15.87 |
| spindle2 | withheld same face overlap | 766 | 6.73 / 10.33 | 6.53 / 10.34 | 6.21 / 9.85 |
| vhedge | withheld stem surface | 1,794 | 19.85 / 23.61 | 16.83 / 22.54 | 16.87 / 22.39 |
| vhedge | withheld ground | 662 | 8.18 / 16.12 | 8.46 / 16.21 | 8.55 / 16.09 |
| vhedge | withheld same face overlap | 1,016 | 7.21 / 10.50 | 6.81 / 10.59 | 6.36 / 9.60 |

Capped RMS limits each residual to 50 mm; uncapped RMS and sample counts remain in the JSON reports. Lower is better. A gate permits at most 1 mm worsening in median/capped RMS, rejects corrections at bounds, and flags low-stem across-width reductions over 3 mm. Passing means the candidate cleared these guards, not that it is more accurate.

| Dataset | Candidate checks | Largest low-stem width reduction (mm) | Status |
| --- | --- | ---: | --- |
| chandelier | Flagged | 9.28 | Baseline retained |
| spindle1 | Flagged | 9.48 | Baseline retained |
| spindle2 | Flagged | 17.38 | Baseline retained |
| vhedge | Flagged | 9.32 | Baseline retained |

Width reduction is a conservative warning, not proof of deformation: correcting an existing gap can also reduce measured width. Short arcs/clutter make the reference diameters uncertain. Flagged candidates are not suitable for automatic replacement.

chandelier: A rigid correction reaches its bound; Low-stem measured across width shrinks by over 3 mm: 1; Low-stem measured across width shrinks by over 3 mm: 3; Low-stem measured across width shrinks by over 3 mm: 4; Low-stem measured across width shrinks by over 3 mm: 5; Low-stem measured across width shrinks by over 3 mm: 6.
spindle1: A rigid correction reaches its bound; withheld_same_face_overlap: worsens by more than 1 mm; Low-stem measured across width shrinks by over 3 mm: 2; Low-stem measured across width shrinks by over 3 mm: 3.
spindle2: Low-stem measured across width shrinks by over 3 mm: 2; Low-stem measured across width shrinks by over 3 mm: 3; Low-stem measured across width shrinks by over 3 mm: 4.
vhedge: A rigid correction reaches its bound; Low-stem measured across width shrinks by over 3 mm: 0; Low-stem measured across width shrinks by over 3 mm: 1; Low-stem measured across width shrinks by over 3 mm: 3.

## Interpretation

The table, rather than the fitting objective, determines whether this experiment helped. Colour can favour stable wood but does not resolve missing opposite-face overlap. Improvement is not guaranteed. All reported capped RMS checks decreased or stayed equal for: spindle2, vhedge.
All published primary clouds keep their original baseline coordinates and ordering. Experimental clouds/matrices are retained under each dataset’s QC/leafwood_comparison folder and have not replaced the final results. An external reference or independent repeat acquisition is needed to quantify absolute accuracy.

## Output fields

`intensity` is an exact alias of raw first-return `signal` (MATLAB SignalPhoton); `near_ir` and `reflectivity` remain distinct raw sensor channels. `red/green/blue` show derived leafWoodSignal in primary/separate clouds. `leaf_red/leaf_green/leaf_blue` preserve that colour even in recording-coloured views. `scan_id` is an unsigned 32-bit original PCAP index, mapped to filenames/checksums in the scan manifest. `frame_id` and `pixel_index` identify the original sensor frame and row-major raw H×W pixel. `leafwood_class`, `wood_score`, and `leafwood_confidence` describe the shared classifier. The old `classification` height-band field is retained and is not leaf/wood truth.

Changing display colouring in CloudCompare by the scan_id scalar field does not alter the stored leaf/wood RGB. Some generic PLY readers ignore custom properties; use a reader that preserves scalar fields.
