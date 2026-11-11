## Final datasets

One release, with a folder per dataset inside the five downloads:

1. **[Final aligned point clouds](https://github.com/u0063882/GreensprayAI/releases/download/current/01_final_aligned_pointclouds.zip)** — open `final_registered.ply` in the `chandelier/`, `spindle1/`, `spindle2/` or `vhedge/` folder.
2. **[Final meshes](https://github.com/u0063882/GreensprayAI/releases/download/current/02_final_mesh.zip)** — chandelier mesh/ground formats under `chandelier/`; spindle1, spindle2 and vhedge have no mesh yet.
3. **[Transformation matrices](https://github.com/u0063882/GreensprayAI/releases/download/current/03_transformations.zip)** — matching JSONs in each dataset folder.

Coordinates are metres. Each dataset uses an independent local frame. Final PLYs are already aligned; do not apply the matrices again. All four primary clouds keep their baseline float coordinates and sample order. Matrices and existing mesh bytes are unchanged.

**Sensor attributes added:** original raw `intensity`/`signal`, ambient `near_ir`, calibrated `reflectivity`, derived shared leafWoodSignal RGB and unsigned 32-bit original `scan_id` (plus sensor frame/pixel indices). `source_scans.json` beside each PLY maps indices to original files/hashes. Display the `scan_id` scalar field to colour by recording. RGB is derived leaf/wood colour, not camera RGB.

**[Leaf/wood registration comparison](https://github.com/u0063882/GreensprayAI/blob/main/docs/leafwood_comparison_report.md):** independent optional refinement and geometry-only control evaluated on all four datasets. All experimental candidates trigger conservative shape/bound warnings and remain under QC. The existing registration remains primary.

<details>
<summary>QC and individual scan contributions</summary>

- **[90_QC.zip](https://github.com/u0063882/GreensprayAI/releases/download/current/90_QC.zip)** — extract and open `QC/index.html`, then choose a dataset. Includes low-height circle figures, ground checks, reports, code and internal rebuild inputs.
- **[91_registered_contributions.zip](https://github.com/u0063882/GreensprayAI/releases/download/current/91_registered_contributions.zip)** — each dataset has leaf/wood-coloured `separate_scans/` and recording-coloured `combined/`, with overlaps, raw channels and leaf/wood RGB fields retained.

Spindle1 was registered from 18 recordings using measured RTK track directions, gravity levelling, within-side ICP, relative ground refinement and bounded per-recording opposite-side poses. Low-stem fitting stays below 0.5 m. Validation uses unused height bands and spatial patches; entire-stem omission leaves some end recordings unsupported and is not used in the final configuration. Circle fits affected by clutter/short arcs remain marked uncertain. These checks are internal model diagnostics, not surveyed accuracy.

Vhedge uses 13 usable recordings (seven A, six B). One byte-identical cross-archive duplicate and one single-packet capture without IMU are excluded and documented in QC and its transformation JSON. Original archives are retained. Its wider crop preserves both V arms and their underlying ground. Side headings come from its own RTK tracks.

Spindle2 uses all 18 complete, distinct recordings. It has its own RTK-inferred headings, reviewed crop and low-stem windows, with the same bounded per-recording registration and held-out checks. Its QC includes ground alignment, low-height circles and the across-row profile.

Registration and meshing remain independent. The existing chandelier mesh is a voxel/SDF envelope prototype, not native Rhino ShrinkWrap. Only the `current` release is active.

</details>
