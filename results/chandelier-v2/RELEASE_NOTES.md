Ground-refined registration of 18 Ouster OS0-128 stationary recordings, nine on each side of the orchard row.

- `orchard_row_result.zip`: merged PLY (1,242,944 points), previews, transforms, processing notes and scripts.
- `separate_registered_scans_v2.zip`: 18 separate PLY files already registered in the same frame, each with one uniform RGB colour.
- `combined_by_recording_v2.zip`: one PLY containing all 18 contributions, coloured by recording (1,746,622 points, with overlaps preserved).
- `processing_code_and_notes.zip`: small archive of processing code and reports. The repository contains the current portable scripts.

Load all separate PLYs together and enable RGB colours. No additional registration is needed. Use version 2 clouds together; the original export used earlier transforms.

The crop covers 7.1 m along the row and a 1.4 m strip across it. Ground refinement reduced held-out ground-patch height-difference RMSE from 3.98 cm to 0.76 cm. This is an internal registration check, not independent survey accuracy. Coordinates are local metres with an approximate ground datum; the antenna offset and survey elevation are not calibrated. Individual tree identities are not assigned. Trellis hardware inside the spatial crop may remain.

Checksums, source URLs, byte counts, per-scan colours, transforms, and quality metrics are recorded under `results/chandelier-v2/` in the repository.
