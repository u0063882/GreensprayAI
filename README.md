# GreensprayAI

Ouster OS0-128 orchard registration, cross-row quality checks, and observed-surface / closed shrink-wrap-style meshing. One repository contains the active code, MATLAB-format description and current result metadata.

**[Current downloads](https://github.com/u0063882/GreensprayAI/releases/tag/current)** · [Mesh](https://github.com/u0063882/GreensprayAI/releases/download/current/02_final_mesh.zip) · [Cross-row QA](docs/cross_row_quality.md) · [Registration in MATLAB terms](docs/registration_matlab.md)

## Current results

**Finished data is under [Releases → Current results → Assets](https://github.com/u0063882/GreensprayAI/releases/tag/current).** Open that page and expand **Assets** if needed. The repository file list contains code and small report snapshots; the large point clouds and meshes are release downloads.

| Main download | Contents |
| --- | --- |
| [01_final_aligned_pointcloud.ply](https://github.com/u0063882/GreensprayAI/releases/download/current/01_final_aligned_pointcloud.ply) | Final merged aligned row; 1,219,008 measured samples, coordinates in metres |
| [02_final_mesh.zip](https://github.com/u0063882/GreensprayAI/releases/download/current/02_final_mesh.zip) | Final canopy in PLY/OBJ/STL, separate ground, and canopy-plus-ground scene |
| [03_transformations.json](https://github.com/u0063882/GreensprayAI/releases/download/current/03_transformations.json) | Final transformation matrices for all 18 recordings |

<details>
<summary>Quality checks, individual scan contributions and code</summary>

- [90_QC.zip](https://github.com/u0063882/GreensprayAI/releases/download/current/90_QC.zip): figures, reports, tables, observed-surface alternatives and code snapshots, grouped in folders. Extract and open **QC/index.html** to browse the figures.
- [91_registered_contributions.zip](https://github.com/u0063882/GreensprayAI/releases/download/current/91_registered_contributions.zip): **separate_scans/** contains the 18 coloured PLYs; **combined/** contains their combined coloured cloud with overlaps retained.
- Code is also available directly in this repository. `git pull` updates code/docs; release downloads are separate.

</details>

All final PLYs are already aligned. The JSON is for interpreting/applying poses to raw or levelled sensor data; **do not apply it again to the final PLY**. See the [download guide](docs/downloads.md).

All clouds and meshes share one local metric frame. Load the separate PLYs together and enable RGB colours. The crop covers 7.1 m along the row and a 1.4 m ground strip. Current metadata lives directly in [`results/`](results/); large downloads use this repository's single active **Current results** release. Required rebuild inputs are archived inside the QC ZIP; the obsolete v2 release has been retired.

The current [east-anchored registration](docs/anchored_registration.md) holds all nine east/Side A files fixed and fits a separate bounded rigid pose for each west/Side B recording. Low stem observations stay below 0.5 m; ground and west-overlap constraints preserve scene consistency. [Before/after circles — `QC/registration/anchored_trunk_sections.png`](https://github.com/u0063882/GreensprayAI/releases/download/current/90_QC.zip) show the improvement and remaining model uncertainty. The [mesh archive](docs/meshing.md) is rebuilt from this refined cloud using the existing independent voxel/SDF mesher.

[Current canopy envelope preview — inside QC bundle: `QC/meshing/shrinkwrap_preview.png`](https://github.com/u0063882/GreensprayAI/releases/download/current/90_QC.zip)

## Reproduce registration

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

See [standalone meshing and the orchard release recipe](docs/meshing.md), the [Rhino ShrinkWrap study, tested alternatives and proposed controls](docs/shrinkwrap_research.md), and the [user's Rhino 8 reference settings](docs/rhino_reference.md). GitHub Actions separately builds the orchard demonstration from checksummed exported clouds and publishes its downloads to the same release.

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

- `run_pipeline.py`: capture-specific registration and point-cloud export.
- `run_meshing.py`: standalone point-cloud-to-envelope interface.
- `processing/register_anchored.py`, `export_anchored.py`: final bounded pose refinement and measured-point export.
- `processing/`: acquisition and geometry implementations; the mesher imports no registration modules.
- `docs/`, `matlab/`: method descriptions and MATLAB-format examples.
- `results/`: current transforms, scan mapping, diagnostic tables and report snapshots.
- `tools/`: mesh validation and current-release publishing.
- `.github/workflows/`: reproducible release generation, with GitHub's built-in token.
- `tests/`: analytic cross-section checks that distinguish trunk diameter from centre error.

```sh
python -m unittest discover -s tests -v
```

No repository license has been imposed; the owner can choose one.
