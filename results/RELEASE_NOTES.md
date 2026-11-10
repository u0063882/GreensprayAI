Current outputs for the 18-recording Ouster OS0-128 orchard capture, in one repository and one active result layout.

Downloads:
- `orchard_mesh.zip`: closed shrink-wrap-style canopy PLY/OBJ/STL, open observed surfaces, separate ground, previews and quality reports.
- `orchard_row_result.zip`: ground-refined merged PLY and the original registration/export notes.
- `separate_registered_scans.zip`: all 18 registered clouds, one uniform RGB colour per recording.
- `combined_by_recording.zip`: all contributions in one PLY, with the same recording colours.
- `processing_code_and_notes.zip`: current repository code and documentation, including standalone meshing, registration, QA, MATLAB-format notes and the ShrinkWrap research report.
- `trunk_slice_diagnostics.zip`: independent A/B circle fits in six thin matched-height slices, per-candidate PNGs, PDF, centre-versus-height plots, JSON and CSV. Key plots and the candidate-4 example are also individual assets.
- Cross-row QA plot, CSV, JSON, mesh preview/quality JSON, and `SHA256SUMS.txt`.

The active repository metadata is consolidated under `results/`, with no separate versioned result directories. This `current` release is the active download location; the older tagged release remains historical provenance for the input archives.

The meshing code provides OPEN observed surfaces and a CLOSED voxel/SDF shrink-wrap-style canopy proxy. The closed proxy is not validated as Rhino-equivalent, biological volume or a CFD-ready model. Its explicit outward offset, gap closing and caps infer unobserved geometry. Ground is a separate open sheet. Cross-row trunk-centre QA finds possible residual offsets but insufficient well-constrained independent trunks for an automatic global correction. Existing ground-refined point-cloud transforms are retained. All outputs use local metres, X along the row, Y across, Z up.

For MATLAB row axes X across / Y along, documentation and numerical examples are in `docs/registration_matlab.md` and `matlab/`. The axis conversion is a proper rotation with an across-axis sign reversal, not a simple X/Y exchange.

Registration and meshing are separate programs. `run_meshing.py` accepts a point cloud alone and optional metadata, without raw Ouster scans, registration caches, GPS or poses. See `docs/meshing.md` for the standalone interface and `docs/shrinkwrap_research.md` for the researched alternatives and experiments.

Thin-slice trunk QA: 6 cm slices at Z=0.18, 0.35, 0.55, 0.75, 1.00 and 1.25 m, with independent centres per side/per height. Three of 42 comparisons pass the diagnostic gates, from two candidates. The candidate at X≈3.01 m has across-centre differences about 8.7 and 11.4 cm at its two lower slices; higher mixed/branched sections are flagged uncertain. No alignment correction is applied. See `docs/cross_row_quality.md` for assumptions and acquisition-bootstrap intervals.
