## Final datasets

One release, with a folder per dataset inside the five downloads:

1. **[Final aligned point clouds](https://github.com/u0063882/GreensprayAI/releases/download/current/01_final_aligned_pointclouds.zip)** — open `spindle1/final_registered.ply` or `chandelier/final_registered.ply`.
2. **[Final meshes](https://github.com/u0063882/GreensprayAI/releases/download/current/02_final_mesh.zip)** — chandelier mesh/ground formats under `chandelier/`; spindle1 has no mesh yet.
3. **[Transformation matrices](https://github.com/u0063882/GreensprayAI/releases/download/current/03_transformations.zip)** — matching JSONs in each dataset folder.

Coordinates are metres. Each dataset uses an independent local frame. Final PLYs are already aligned; do not apply the matrices again. Chandelier geometry and matrix data are unchanged.

<details>
<summary>QC and individual scan contributions</summary>

- **[90_QC.zip](https://github.com/u0063882/GreensprayAI/releases/download/current/90_QC.zip)** — extract and open `QC/index.html`, then choose a dataset. Includes low-height circle figures, ground checks, reports, code and internal rebuild inputs.
- **[91_registered_contributions.zip](https://github.com/u0063882/GreensprayAI/releases/download/current/91_registered_contributions.zip)** — each dataset has `separate_scans/` and `combined/`, coloured by recording, with overlap retained.

Spindle1 was registered from 18 recordings using measured RTK track directions, gravity levelling, within-side ICP, relative ground refinement and bounded per-recording opposite-side poses. Low-stem fitting stays below 0.5 m. Validation uses unused height bands and spatial patches; entire-stem omission leaves some end recordings unsupported and is not used in the final configuration. Circle fits affected by clutter/short arcs remain marked uncertain. These checks are internal model diagnostics, not surveyed accuracy.

Registration and meshing remain independent. The existing chandelier mesh is a voxel/SDF envelope prototype, not native Rhino ShrinkWrap. Only the `current` release is active.

</details>
