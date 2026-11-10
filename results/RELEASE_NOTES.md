Current outputs for the 18-recording Ouster OS0-128 orchard capture, in one repository and one active result layout.

Downloads:
- `orchard_mesh.zip`: closed shrink-wrap-style canopy PLY/OBJ/STL, open observed surfaces, separate ground, previews and quality reports.
- `orchard_row_result.zip`: ground-refined merged PLY and the original registration/export notes.
- `separate_registered_scans.zip`: all 18 registered clouds, one uniform RGB colour per recording.
- `combined_by_recording.zip`: all contributions in one PLY, with the same recording colours.
- `processing_code_and_notes.zip`: the earlier registration-code snapshot; current registration, QA, MATLAB-format notes and meshing code are in the repository.
- Cross-row QA plot, CSV, JSON, mesh preview/quality JSON, and `SHA256SUMS.txt`.

The active repository metadata is consolidated under `results/`, with no separate versioned result directories. This `current` release is the active download location; the older tagged release remains historical provenance for the input archives.

The meshing code provides OPEN observed surfaces and a CLOSED voxel/SDF shrink-wrap-style canopy proxy. The closed proxy is not validated as Rhino-equivalent, biological volume or a CFD-ready model. Its explicit outward offset, gap closing and caps infer unobserved geometry. Ground is a separate open sheet. Cross-row trunk-centre QA finds possible residual offsets but insufficient well-constrained independent trunks for an automatic global correction. Existing ground-refined point-cloud transforms are retained. All outputs use local metres, X along the row, Y across, Z up.

For MATLAB row axes X across / Y along, documentation and numerical examples are in `docs/registration_matlab.md` and `matlab/`. The axis conversion is a proper rotation with an across-axis sign reversal, not a simple X/Y exchange.
