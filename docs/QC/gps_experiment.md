# GPS refinement: first 15-second result

**GPS agreement improved; tree-cloud overlap stayed practically unchanged.**
This is an experiment on `experiment/gps-constrained-slam`. Main, Current
stationary outputs, the input and the original SDK result are unchanged.

| Reserved comparison | SDK baseline | GPS candidate |
| --- | ---: | ---: |
| GPS antenna position RMSE (27 fixes in held-out time blocks) | 6.90 cm | 5.18 cm |
| LiDAR capped nearest-neighbor RMS | 8.031 cm | 8.040 cm |
| LiDAR matches within 10 cm | 58.16% | 58.00% |

GPS final-test RMSE fell **24.9%**, or 1.71 cm. LiDAR overlap cost rose about
**0.09 mm**, a small practical change. Its block-bootstrap interval indicates a
small degradation, not an improvement. Do not describe this as demonstrated
improvement in tree-shape accuracy.

Maximum horizontal correction was **10.10 cm**, mostly along travel. Maximum
yaw correction was **0.0125 degrees**. All 818,820 selected points were retained.
Heights, raw signal/NIR, reflectivity, RGB/classes and source indices were
preserved; each column receives a rigid correction.

The candidate passed the stated internal guards: improved GPS test agreement,
less than 1 mm tolerated LiDAR degradation, no bound hits, and stability to a
doubled antenna-offset prior. Doubling that prior changed the estimated
correction at 8 m by at most **4.1 mm**.

## Interpretation

GPS genuinely influences the trajectory in this new postprocessing stage.
The unmeasured antenna lever arm is an uncertain nuisance parameter, **not a
surveyed calibration**. Shared software start aligns clocks; startup latency
remains uncalibrated and could contribute to centimetre-scale along-track
discrepancies. GPS agreement cannot be interpreted directly as absolute LiDAR
accuracy. The existing SDK map already agreed reasonably well with GPS.

Raw GPS fixes use whole-block training, selection and final-test splits.
Separate parts of the SDK-withheld pixels are used for selection and reserved
LiDAR checking. This symmetric, two-lag overlap metric differs from the earlier
pilot metric; compare baseline and candidate here rather than comparing its
absolute value with that earlier report.

**Recommendation:** inspect this trial alongside the original and test a longer
segment with independent checks before adopting GPS refinement as the default.
Passing these tolerances does not establish more accurate orchard geometry.
Main remains the baseline.

## Files

- [Comparison figure](../../results/gps_experiment/comparison.png)
- [Machine-readable report](../../results/gps_experiment/comparison.json)
- [Method, parameters and offline commands](../gps_refinement_experiment.md)
- [Trial ZIP, 119 MB](https://github.com/u0063882/GreensprayAI/releases/download/moving-scan-inputs/gps-15s-experiment.zip)
- [Successful Actions run](https://github.com/u0063882/GreensprayAI/actions/runs/44388714244)

The ZIP contains `candidate/registered.ply`, corrected OSF, column-pose NPZ,
frame-centre JSON/CSV, report and Rhino copies. The independent local command
is `refine_moving_gps.py`; it can also evaluate your longer existing SDK result.
Only the first 15 seconds were tested automatically.

All 116 repository tests passed. A real short-recording smoke check also
verified that zero corrections preserve every PLY field and OSF pose.
