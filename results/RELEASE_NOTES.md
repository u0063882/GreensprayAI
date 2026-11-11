## Main downloads

1. **[Final aligned point cloud — PLY](https://github.com/u0063882/GreensprayAI/releases/download/current/01_final_aligned_pointcloud.ply)**
2. **[Final mesh — ZIP](https://github.com/u0063882/GreensprayAI/releases/download/current/02_final_mesh.zip)**
3. **[Transformation matrices — JSON](https://github.com/u0063882/GreensprayAI/releases/download/current/03_transformations.json)**

Coordinates and translations are **metres**. The final point cloud is already aligned; do not apply the matrices to it again. The mesh ZIP includes the closed canopy in PLY/OBJ/STL, the separate open ground sheet, and their combined scene.

<details>
<summary>Quality checks and supporting files</summary>

- **[90_QC.zip](https://github.com/u0063882/GreensprayAI/releases/download/current/90_QC.zip)** — figures, reports, tables, observed-surface alternatives and code snapshots in folders. Extract and open **QC/index.html** for the figure gallery.
- **[91_registered_contributions.zip](https://github.com/u0063882/GreensprayAI/releases/download/current/91_registered_contributions.zip)** — all 18 separate registered PLYs and the combined recording-coloured cloud, with manifests and notes.

Registration holds east/Side A fixed and gives each west/Side B recording its own bounded rigid correction. Fitting uses stems below 0.5 m, scanner-facing consistency, ground and same-face overlap. Held-out checks are internal model/correspondence diagnostics, not independently surveyed accuracy. Uncertain circle fits remain marked.

Meshing is independent and uses the existing voxel/SDF envelope prototype, not native Rhino ShrinkWrap. The user's Rhino reference remains recorded separately.

This reorganization preserves the validated point, mesh and matrix data. Packaging-only updates reuse those bytes; processing changes run the full build and validation. Original data and packaging commits are recorded in QC/provenance/data_origin.json. Historical seed releases are unchanged.

</details>
