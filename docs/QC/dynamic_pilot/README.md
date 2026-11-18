# First 15-second moving-sensor trial

The trial completed successfully on 18 November 2026. This is a pilot for
inspection, not an independently validated orchard reconstruction.

| Measure | Result |
| --- | ---: |
| Captured interval retained | 14.9902 s, within the requested first 15 s |
| Complete scans processed | 150; none dropped |
| Posed frame-centre time span | 14.9004 s |
| Selected measured first-return points | 818,820 |
| SLAM path length | 6.251 m |
| Start-to-end displacement | 5.058 m |
| Held-out matches within 10 cm, before → after | 40.2% → 54.0% |
| Held-out 10 cm capped RMS, before → after | 8.87 cm → 8.35 cm |
| Largest translation between scan centres | 15.5 cm |
| Largest orientation change between scan centres | 10.25 degrees |
| Sensor trajectory vertical range | 5.83 cm |
| Dominant low-surface plane inlier RMS | 2.25 cm |
| Plane tilt from approximate initial IMU vertical | 1.21 degrees |
| SLAM and basic diagnostic computation | 10.2 s on the Actions runner |

The held-out comparison uses 145 scan pairs roughly half a second apart.
These are internal overlap measures, affected by sampling and occlusion;
the plane RMS measures surface scatter, not registration accuracy. The
captured scene forms a coherent row in the preview, but isolated points and
other background remain. No final tree-only ROI has been selected.

**GPS validation could not run.** PCAP timestamps begin near 3271 seconds,
whereas H5 times are calendar/Unix timestamps. Their relationship is not
provided, so the pipeline does not invent a clock offset from the filename.
Resolving this offset would allow an independent trajectory comparison.
The maximum 10.25-degree single-frame orientation change also warrants
comparison against the recorded gyro before accepting a longer run. The
automatic `plausible_pilot_continue_with_QC` label is a screening result,
not an accuracy certificate or a recommendation to skip those checks.

Colours use the existing shared leaf/wood display model, provisionally. That
model has not been revalidated on this July capture, and colours were not
used in SLAM. Raw channels, source indices and per-column poses are retained.

- [Download the pilot ZIP (119 MB)](https://github.com/u0063882/GreensprayAI/releases/download/moving-scan-inputs/pilot-15s-results.zip)
- [Dense preview](preview.png)
- [Machine-readable report](../../../results/dynamic_pilot_15s.json)
- [Method and reproduction](../../moving_pilot.md)
- [Successful Actions run](https://github.com/u0063882/GreensprayAI/actions/runs/44281779031)

In the ZIP, `pilot_registered.ply` is the enriched cloud in metres. For Rhino,
open `rhino/final_registered.3dm` (model units: millimetres). The OSF retains
the selected complete dual-return scans with per-column SDK poses.

The full recording was not mapped. Input SHA-256 was verified, the original
input asset stayed unchanged, and all five Current stationary assets kept
their original asset IDs, sizes and SHA-256 digests.
