# GreensprayAI

Ouster OS0-128 orchard registration, cross-row quality checks, and observed-surface / closed shrink-wrap-style meshing. One repository contains the active code, MATLAB-format description and current result metadata.

**[Current downloads](https://github.com/u0063882/GreensprayAI/releases/tag/current)** · [Mesh](https://github.com/u0063882/GreensprayAI/releases/download/current/02_final_mesh.zip) · [Cross-row QA](docs/cross_row_quality.md) · [Registration in MATLAB terms](docs/registration_matlab.md)

**[Current joint registration and ROI report](docs/joint_registration_report.md).** V-hedge now uses a validated joint rigid refinement of both sides, including recovery of a disconnected recording. Chandelier has a measured X-ROI extension and an independently rebuilt matching mesh. Accepted spindle outputs remain unchanged. Original sensor channels, derived leaf/wood RGB and source IDs are retained; complete previous results are archived inside QC.

## Current results

**Finished data is under [Releases → Current results → Assets](https://github.com/u0063882/GreensprayAI/releases/tag/current).** Open that page and expand **Assets** if needed. The repository file list contains code and small report snapshots; the large point clouds and meshes are release downloads.

| Main download | Contents |
| --- | --- |
| [01_final_aligned_pointclouds.zip](https://github.com/u0063882/GreensprayAI/releases/download/current/01_final_aligned_pointclouds.zip) | One final PLY per dataset: `chandelier/`, `spindle1/`, `spindle2/`, `vhedge/` |
| [02_final_mesh.zip](https://github.com/u0063882/GreensprayAI/releases/download/current/02_final_mesh.zip) | Rebuilt chandelier canopy PLY/OBJ/STL, ground and combined scene, under `chandelier/` |
| [03_transformations.zip](https://github.com/u0063882/GreensprayAI/releases/download/current/03_transformations.zip) | One matching `transformations.json` per dataset folder |

Each dataset has its **own local frame, in metres**. Its final PLYs already have its matrices applied; do not apply them again. The dataset folders are separate capture results, not a common field-wide georeferenced scene.

| Dataset | Status |
| --- | --- |
| Chandelier | Accepted poses and original points retained; X ROI extended to −1.25…7.35 m; matching mesh rebuilt |
| Spindle1 | Final independent trunk-based refinement applied to the 18 registered recordings; no mesh yet |
| Spindle2 | Final independent trunk-based refinement applied to the 18 registered recordings; no mesh yet |
| Vhedge | Joint rigid poses for both sides; disconnected recording recovered; original samples and attributes retained |

<details>
<summary>Quality checks, individual scan contributions and code</summary>

- [90_QC.zip](https://github.com/u0063882/GreensprayAI/releases/download/current/90_QC.zip): extract and open **QC/index.html**, then choose a dataset. Figures/reports are grouped under `QC/chandelier/`, `QC/spindle1/`, `QC/spindle2/` and `QC/vhedge/`. Source inputs needed for rebuilding are tucked under provenance folders.
- [91_registered_contributions.zip](https://github.com/u0063882/GreensprayAI/releases/download/current/91_registered_contributions.zip): each dataset has `separate_scans/` with leaf/wood-coloured PLYs, plus `combined/` with recording colours and all overlaps retained. Both retain raw sensor channels, `scan_id` and separate leaf/wood RGB fields.
- Current code is in this repository and in `QC/provenance/current_code_and_notes.zip`. `git pull` updates code/docs; release downloads are separate.

</details>

See the [download guide](docs/downloads.md) and [named capture registration](docs/capture_registration.md). A/B names are labels; the new capture pipeline infers inward headings from the RTK tracks. Spindle1 uses a 7.1 m row section and a 1.8 m ground strip. Spindle2 uses a 7.45 m section and 2.0 m ground strip. Vhedge uses a 5.1 m section and a 3.1 m strip to preserve both V arms. The chandelier X crop is now wider; the other dataset input crops are unchanged. Supports and attached clutter may remain, and some short-arc trunk centre fits remain uncertain.

The [earlier colour-only experiment](docs/leafwood_comparison_report.md) remains a historical comparison on the pre-refinement baseline. Its candidate never became the primary result. The spindle primary data retain the independent trunk/post refinement. V-hedge now adds the joint stage documented above; its figures identify an ambiguous landmark excluded from fitting while retaining all measured points.

The following existing east-anchored description and historical report numbers refer to **chandelier**:

The underlying [east-anchored registration](docs/anchored_registration.md) held all nine original east/Side A files fixed during its pose fit; the subsequent ROI export adds original measured returns through those same accepted poses. The pose fit uses a separate bounded rigid pose for each west/Side B recording. Low stem observations stay below 0.5 m; ground and west-overlap constraints preserve scene consistency. [Before/after circles — `QC/chandelier/registration/anchored_trunk_sections.png`](https://github.com/u0063882/GreensprayAI/releases/download/current/90_QC.zip) show the improvement and remaining model uncertainty. The [mesh archive](docs/meshing.md) was computed from this refined cloud using the existing independent voxel/SDF mesher.

[Current canopy envelope preview — inside QC bundle: `QC/chandelier/meshing/shrinkwrap_preview.png`](https://github.com/u0063882/GreensprayAI/releases/download/current/90_QC.zip)

## Register a named capture

Extract that capture's files into its own `/path/to/spindle1/work/data/sideA/` and `sideB/` folders. Do not reuse another capture's workspace.

```sh
python -m pip install -r requirements-captures.txt
python run_capture.py --config presets/captures/spindle1.json --workspace /path/to/spindle1
```

Outputs go to that workspace's `final/`, with `final_registered.ply`, `transformations.json`, registered scan contributions and QC figures. Spindle2 has its own preset at `presets/captures/spindle2.json`. All 18 recordings are retained. Vhedge has its own preset at `presets/captures/vhedge.json`. That preset excludes one identical cross-archive duplicate and one single-packet capture without IMU; both original ZIPs remain in QC provenance. A fresh capture needs its own reviewed crop and low-stem windows, rather than copying chandelier coordinates. Cached stages are tied to source hashes, configuration and algorithm code. Registration does not import or run a mesher.

## Reproduce chandelier registration

Tested with Python 3.12:

```sh
python -m venv .venv
# Linux/macOS:
source .venv/bin/activate
# Windows PowerShell instead: .venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
```

Extract the original `sideA.zip` and `sideB.zip` into `work/data/sideA/` and `work/data/sideB/`. Each of the nine recordings per side needs its PCAP, JSON calibration and H5 GPS file with matching basenames.

```sh
python run_pipeline.py --check-inputs-only
python run_pipeline.py
# With existing decoded caches and final poses:
python run_pipeline.py --export-only
```

Generated final files go to `output/`; seed caches and the separate `anchored_poses.npy` go to `work/`. The full command includes the bounded east-anchored stage; only internally validated candidates are promoted. Raw recordings and generated binaries are excluded from Git history. Use a fresh `work/` directory for a new capture/full rerun. These capture-specific scripts assume the supplied station ordering, inward FOV, mounting, nine recordings per side and fixed row crop; review these before applying the pipeline elsewhere.

## Mesh any point cloud independently

Registration and meshing are separate programs. Meshing requires a point cloud in metres; it does not load raw recordings, cached scans, GPS, fitted poses or registration modules. It also accepts clouds exported from MATLAB or other software.

```sh
python -m pip install -r requirements-meshing.txt
python run_meshing.py --input /path/to/trees.ply --output-dir /path/to/mesh
```

Optional `--metadata metadata.json` records units and frame information without changing coordinates. Ground is included in the envelope unless explicitly excluded using an optional ground mesh; the general mesher does not assume that soil lies near Z=0. The defaults are the existing voxel/SDF prototype, not a validated Rhino equivalent.

See [standalone meshing and the orchard release recipe](docs/meshing.md), the [Rhino ShrinkWrap study, tested alternatives and proposed controls](docs/shrinkwrap_research.md), and the [user's Rhino 8 reference settings](docs/rhino_reference.md). The collection workflow preserves the validated chandelier mesh while new captures are registered independently.

## Registration and MATLAB

1. Official Ouster decoding, stationary frame combination and IMU gravity levelling.
2. RTK coarse placement and inward side-heading hypotheses.
3. Robust same-side point-to-plane ICP and a global pose graph with GPS/gravity priors.
4. Explicit opposite-side XY/yaw registration using low stems.
5. Joint relative ground roll/pitch/height refinement using overlapping patches and held-out validation.
6. Fixed east anchor; sequential west insertion and joint bounded per-recording refinement using low stems, scanner-facing consistency, ground and same-face overlap.
7. Validated point-cloud export and cross-row diagnostics. Meshing consumes the resulting cloud independently.

Python exports use **X along / Y across / Z up**. The MATLAB toolbox row convention uses **X across / Y along / Z up**, with the appropriate sign change to preserve handedness. The [MATLAB description](docs/registration_matlab.md) maps the objectives, parameters, transformation composition and actual solver differences. [`matlab/`](matlab/) includes a contribution viewer and an explanatory opposite-side SE(2) solver. These MATLAB functions have not been runtime-tested in MATLAB here.

The seed ground stage reduced held-out patch RMSE from 3.98 cm to 0.76 cm. The final bounded stage improves excluded-stem/height residuals while preserving ground and west overlap; see its [validation report](results/anchored_registration_quality.json). These are internal checks, not independent survey accuracy. The antenna offset and elevation datum are uncalibrated; foliage motion/occlusions and support hardware remain relevant. Individual tree identities are still unassigned.

## Repository layout

- `run_capture.py`, `captures/`: isolated named captures, label-independent RTK heading inference and per-recording refinement.
- `run_pipeline.py`: original chandelier-specific registration and point-cloud export.
- `run_meshing.py`: standalone point-cloud-to-envelope interface.
- `processing/register_anchored.py`, `export_anchored.py`: final bounded pose refinement and measured-point export.
- `processing/`: acquisition and geometry implementations; the mesher imports no registration modules.
- `docs/`, `matlab/`: method descriptions and MATLAB-format examples.
- `results/`: current transforms, scan mapping, diagnostic tables and report snapshots.
- `tools/`: mesh validation and current-release publishing.
- `.github/workflows/`: reproducible release generation, with GitHub's built-in token.
- `tests/`: analytic geometry, heading inference, per-recording pose and dataset packaging checks.

```sh
python -m unittest discover -s tests -v
```

No repository license has been imposed; the owner can choose one.

The [earlier structural investigation](docs/structural_alignment.md) records rejected trials on the former baseline. The [current report](docs/joint_registration_report.md) describes the accepted joint result and its remaining uncertainty. No tree-shape symmetry is imposed.

The [PCA orientation trial has been withdrawn](docs/structural_up_report.md): it introduced a large ground slope that is inconsistent with the field conditions. Current outputs retain the original gravity-referenced registration frame, the joint rigid refinements, and the wider chandelier across-row ROI with its matching mesh. Canopy PCA is diagnostic only; the remaining V-hedge slant is unresolved.
