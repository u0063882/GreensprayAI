# Current downloads

Open [Current results → Assets](https://github.com/u0063882/GreensprayAI/releases/tag/current). There is one active release and five custom downloads:

1. **01_final_aligned_pointclouds.zip** — `final_registered.ply` under each of `chandelier/`, `spindle1/`, `spindle2/` and `vhedge/`.
2. **02_final_mesh.zip** — chandelier mesh formats and ground under `chandelier/`. Spindle1, spindle2 and vhedge have not been meshed.
3. **03_transformations.zip** — each dataset has its own `transformations.json`.
4. **90_QC.zip** — extract and open **QC/index.html** to choose a dataset and browse figures/reports.
5. **91_registered_contributions.zip** — each dataset has leaf/wood-coloured `separate_scans/` and recording-coloured `combined/`, with overlap retained. Both store raw sensor channels, leaf/wood RGB and original `scan_id`.

Every dataset uses its own independent local metric frame. PLY coordinates and matrix translations are metres. A final PLY is already registered; do not apply its matrices again.

- **V-hedge:** the final joint stage adjusts individual rigid poses on both sides and recovers a recording disconnected from its original pose graph. Its 830,803 primary samples retain their raw/display fields and source identities.
- **Chandelier:** the X ROI is extended from −0.5…6.6 to −1.25…7.35 m using original measured returns. It now contains 1,326,879 points. Every former point and accepted matrix is retained; the mesh is rebuilt from this exact expanded PLY.
- **Spindle 1 / Spindle 2:** their accepted trunk-refined primary outputs remain unchanged. The same joint method was evaluated as QC on both.

See the [current results and figures](joint_registration_report.md). V-hedge before/after figures and validation are under `QC/vhedge/joint_registration/`. Its pre-joint primary PLY, matrices and per-scan XYZ are archived in `QC/vhedge/provenance/joint_registration_baseline.zip`. Chandelier's complete former crop, contributions, matrices and mesh are in `QC/chandelier/provenance/roi_extension_baseline.zip`; current ROI/mesh checks are under `QC/chandelier/roi_extension/`.

The [earlier trunk-stage report](trunk_refinement_report.md) remains relevant to the spindle outputs. Before/after 18/30/40 cm figures are under `QC/<dataset>/trunk_refinement/`. Original trunk-stage baselines remain under provenance. Canonical final filenames and the five-download layout stay the same.

Each dataset's 01 folder also contains `source_scans.json` (scan index to original PCAP/hash) and `leafwood_model.json`. Choose the `scan_id` scalar field in CloudCompare to display recording colours. `intensity` is raw first-return SIGNAL; reflectivity and ambient NIR remain distinct channels. RGB is derived leafWoodSignal, not camera colour. See [fields and method](leafwood_registration.md).

The active classifier uses the original Toolbox scheme-13 Lab features, fitted once across all recordings. It supersedes the earlier NIR-only threshold. Spindle 1 and Spindle 2 have dimensioned final-baseline trunk slices under `QC/<dataset>/leafwood_comparison/trunk_slices_dimensioned.png` and `.pdf`, with fit details in `.json`. All slice axes and diameter labels use centimetres. Uncertain circle fits are marked and must not be read as measured physical diameters.

User field measurements give spindle trunk circumference 40–45 cm at 30 cm height, and an adjacent support-post circumference of 22 cm. Their circular-equivalent diameters are 12.7–14.3 cm and 7.0 cm. `QC/<dataset>/trunk_reference_30cm/raw_slices.png/.pdf` shows raw 2 cm slices at that height alongside size references, with measurements/provenance in `reference.json`. These checks do not modify the point clouds or force a fitted radius. Trunk and post are both woody, so they need separate geometric interpretation.

The [earlier colour-only comparison](leafwood_comparison_report.md) covers the pre-refinement baseline, an extra geometry-only control and optional leaf/wood refinement. Those historical candidates remain experimental under `QC/<dataset>/leafwood_comparison/`. They are separate from the final trunk/post step now applied to the spindle primary data in 01.

QC is grouped under `QC/chandelier/`, `QC/spindle1/`, `QC/spindle2/` and `QC/vhedge/`. Each has figures and reports. Required chandelier rebuild seeds are under `QC/chandelier/provenance/registration_seed/`; each named capture's original archives are under its `provenance/raw_inputs/` folder. Vhedge's duplicate and incomplete-file exclusions are recorded in its configuration and reports. The current source snapshot and collection metadata are under `QC/provenance/`. These internal inputs are not the final outputs.

GitHub adds its automatic source ZIP/tar links below these five custom files. The obsolete v2 release and tag have been removed. Previous flat download names were replaced by the shared ZIPs with dataset folders.

`git pull` updates repository code/docs, not downloads. Use a fresh download folder to avoid mixing old filenames. The three main ZIPs alone can be fetched with:

```sh
gh release download current --repo u0063882/GreensprayAI --dir final_downloads --pattern '0*' --clobber
```

## Publication

The collection publisher verifies source download hashes before extracting them. Datasets marked `frozen` in the collection manifest retain their verified outputs even when processing code changes. New captures run in separate workspaces, pass held-out geometry checks, and are validated against the measured points and transformation matrices before packaging. Unchanged capture outputs are reused using code/configuration/input fingerprints; a manual `rebuild` regenerates datasets marked `raw_capture`, while `repackage` refuses to process a changed capture.

All five replacement downloads must pass server checksum verification before obsolete flat attachments are removed. Current archive hashes are in the release notes; the four non-QC download hashes are also in `QC/SHA256SUMS.txt`. Individual dataset output hashes and build metadata are in QC provenance. The legacy chandelier raw/seed recipe remains documented separately.
