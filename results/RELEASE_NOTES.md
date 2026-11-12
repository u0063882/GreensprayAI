## Final datasets

One Current release, with dataset folders in the same five downloads:

1. **[Final aligned point clouds](https://github.com/u0063882/GreensprayAI/releases/download/current/01_final_aligned_pointclouds.zip)** — `final_registered.ply` under `chandelier/`, `spindle1/`, `spindle2/` and `vhedge/`.
2. **[Final mesh](https://github.com/u0063882/GreensprayAI/releases/download/current/02_final_mesh.zip)** — chandelier envelope/ground formats. The other datasets have no mesh yet.
3. **[Transformation matrices](https://github.com/u0063882/GreensprayAI/releases/download/current/03_transformations.zip)** — matching JSONs under each dataset folder.

**V-hedge:** joint rigid refinement now adjusts both sides and recovers a previously disconnected recording. Its 830,803 measured samples retain all non-coordinate fields and source identities. Terrain may slope and trunks may lean; no V symmetry or horizontal-bar target is imposed.

**Chandelier:** the measured X ROI expands from −0.5…6.6 to −1.25…7.35 m, adding 107,871 samples for a total of 1,326,879. All original points and accepted poses are retained. The independently rebuilt mesh hashes this exact expanded PLY.

**Spindle 1 / Spindle 2:** accepted trunk-refined primary outputs are retained. The same joint method was evaluated on them as QC.

Read the **[current report and figures](https://github.com/u0063882/GreensprayAI/blob/main/docs/joint_registration_report.md)** for excluded-return checks, reference-size uncertainty, anchor tests, the ambiguous object window and mesh limits. These are internal consistency checks, not independent survey accuracy or guaranteed biological symmetry.

Coordinates are metres; each dataset has its own local frame. Final PLYs are already registered: do not apply the matrices again. Original raw SIGNAL/intensity, ambient NIR, reflectivity, saved shared Lab leafWoodSignal RGB, `scan_id`, frame IDs and pixel indices are retained. `source_scans.json` maps scan indices to original PCAPs/hashes. RGB is derived leaf/wood colour, not camera RGB.

<details><summary>QC, individual contributions and original results</summary>

- **[90_QC.zip](https://github.com/u0063882/GreensprayAI/releases/download/current/90_QC.zip)** — extract and open `QC/index.html`. Current joint figures are under `QC/<dataset>/joint_registration/`; chandelier crop/mesh checks are under `QC/chandelier/roi_extension/`.
- **[91_registered_contributions.zip](https://github.com/u0063882/GreensprayAI/releases/download/current/91_registered_contributions.zip)** — dataset folders contain `separate_scans/` and recording-coloured `combined/`, with overlaps and leaf/wood RGB fields retained.

The pre-joint V-hedge result is archived at `QC/vhedge/provenance/joint_registration_baseline.zip`. The complete former chandelier crop, contributions, matrices and mesh are at `QC/chandelier/provenance/roi_extension_baseline.zip`. Earlier trunk/colour experiments remain historical QC. No separate v2 release or new top-level result list is introduced.

V-hedge still uses the same 13 usable recordings; its duplicate/incomplete capture exclusions are retained. Spindles use all 18 recordings each. Registration and meshing remain independent. The chandelier envelope uses the existing voxel/SDF approximation and is not native Rhino ShrinkWrap.

`git pull` updates code/docs, not release assets. Download the current ZIPs into a fresh folder.

</details>
