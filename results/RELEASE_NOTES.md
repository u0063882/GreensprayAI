Current outputs for the 18-recording Ouster OS0-128 orchard capture, in one repository and one active result layout.

Downloads:
- `orchard_mesh.zip`: closed shrink-wrap-style canopy PLY/OBJ/STL, open observed surfaces, separate ground, previews and quality reports.
- `orchard_row_result.zip`: east-anchored merged PLY, current transforms and registration quality reports.
- `separate_registered_scans.zip`: all 18 registered clouds, one uniform RGB colour per recording.
- `combined_by_recording.zip`: all contributions in one PLY, with the same recording colours.
- `processing_code_and_notes.zip`: current repository code and documentation, including standalone meshing, registration, QA, MATLAB-format notes and the ShrinkWrap research report.
- `trunk_slice_diagnostics.zip`: independent A/B circle fits in nine thin matched-height slices below 0.5 m, per-candidate PNGs, PDF, centre-versus-height plots, JSON and CSV. Key plots and the candidate-4 example are also individual assets.
- Cross-row QA plot, CSV, JSON, mesh preview/quality JSON, and `SHA256SUMS.txt`.

The active repository metadata is consolidated under `results/`, with no separate versioned result directories. This `current` release is the active download location; the older tagged release remains historical provenance for the input archives.

Current registration: the completed east/Side A seed is fixed; each west/Side B recording has its own bounded rigid correction. All nine east cloud files are unchanged. Low stems, scanner-facing consistency, ground and west-to-west overlap constrain the 54 west pose variables. Held-out feature/height/ground/overlap checks pass. The three familiar cloud ZIPs contain the refined result; historical ZIPs are immutable seed inputs only.

New assets: `anchored_trunk_sections.png` (before/after circles), `anchored_registration_comparison.png`, detailed `anchored_registration_quality.json`, summary `registration_quality.json`, and `orchard_row_preview.png`. The merged cloud contains 1,219,008 measured points; separate contributions contain 1,739,386 points with overlap retained. The same crop is reapplied. Input seeds were already cropped, so omitted boundary returns are not recovered.

Held-out median radial residuals improve from 4.47 to 1.55 cm on excluded height bands and 5.74 to 2.21 cm on an excluded stem. Withheld west overlap RMS improves from 1.35 to 1.11 cm; held-out ground median changes by less than 1 mm. These are internal model/correspondence checks, not independent survey accuracy. Foliage motion, non-circular/root-flared stems and the fixed east anchor remain limitations. Current transforms and source hashes are in the reports.

Registration and meshing are independent programs. Meshes are rebuilt from the final cloud using the existing voxel/SDF prototype; it is not validated as Rhino-equivalent, biological volume or CFD geometry. Ground is a separate open sheet. The user's Rhino reference is ShrinkWrap with 15 mm target edges, −5 mm offset, smoothing 1 and optimization 50%, followed by Reduce Mesh by 75%. That reference is recorded; the prototype uses different controls.

Trunk QA uses nine overlapping 6 cm slices centred from Z=0.14 to 0.46 m, with every full slice below 0.5 m. Green/solid comparisons pass diagnostic gates; dashed/hollow fits remain uncertain. The common-shift check is an additional diagnostic on the current result and applies no further correction. MATLAB axis conversion and the final per-recording objective are documented under `docs/`.
