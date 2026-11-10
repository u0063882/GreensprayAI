# GreensprayAI

Ouster OS0-128 orchard registration, cross-row quality checks, and observed-surface / closed shrink-wrap-style meshing. One repository contains the active code, MATLAB-format description and current result metadata.

**[Current downloads](https://github.com/u0063882/GreensprayAI/releases/tag/current)** · [Mesh](https://github.com/u0063882/GreensprayAI/releases/download/current/orchard_mesh.zip) · [Cross-row QA](docs/cross_row_quality.md) · [Registration in MATLAB terms](docs/registration_matlab.md)

## Current results

| Download | Purpose |
| --- | --- |
| `orchard_mesh.zip` | Closed canopy PLY/OBJ/STL, open observed surfaces, separate ground, previews and quality reports |
| `orchard_row_result.zip` | Merged, ground-refined row: 1,242,944 measured points |
| `separate_registered_scans.zip` | 18 separate registered PLYs, one uniform RGB colour per recording |
| `combined_by_recording.zip` | 1,746,622 points with overlaps preserved and recording colours |

All clouds and meshes share one local metric frame. Load the separate PLYs together and enable RGB colours. The crop covers 7.1 m along the row and a 1.4 m ground strip. Current metadata lives directly in [`results/`](results/); large downloads use this repository's single active **Current results** release. Older Git/release history is retained as provenance.

The [across-row check](docs/cross_row_quality.md) distinguishes opposite bark faces from fitted trunk-centre errors. It flags possible residual offsets, but the current partial sections do not provide enough independent evidence for a global correction. The existing ground-refined poses are retained. The [mesh archive](docs/meshing.md) contains both supported open surface patches and a closed, voxel/SDF shrink-wrap-style canopy proxy. The proxy deliberately adds offset and fills gaps; neither output is a validated biological or CFD model.

![Current observed-surface mesh](https://github.com/u0063882/GreensprayAI/releases/download/current/shrinkwrap_preview.png)

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
python run_pipeline.py --mesh
# With existing decoded caches and final poses:
python run_pipeline.py --export-only --mesh
```

Generated files go to `output/`; caches and poses go to `work/`. Raw recordings and generated binaries are excluded from Git history. Use a fresh `work/` directory for a new capture/full rerun. These capture-specific scripts assume the supplied station ordering, inward FOV, mounting, nine recordings per side and fixed row crop; review these before applying the pipeline elsewhere.

## Mesh an existing result

Extract `orchard_row_registered.ply` and `scan_transforms.json` from the merged ZIP into `output/`, then:

```sh
python processing/mesh.py
python processing/shrinkwrap.py
python tools/validate_mesh.py
```

See [meshing settings, geometry checks and limitations](docs/meshing.md). Mesh generation is also implemented in GitHub Actions; publication validates the source ZIP checksums, reconstructs the mesh and uploads all current downloads to the same release.

## Registration and MATLAB

1. Official Ouster decoding, stationary frame combination and IMU gravity levelling.
2. RTK coarse placement and inward side-heading hypotheses.
3. Robust same-side point-to-plane ICP and a global pose graph with GPS/gravity priors.
4. Explicit opposite-side XY/yaw registration using low stems.
5. Joint relative ground roll/pitch/height refinement using overlapping patches and held-out validation.
6. Row/ground crop, individual scan export, cross-row diagnostics and optional meshing.

Python exports use **X along / Y across / Z up**. The MATLAB toolbox row convention uses **X across / Y along / Z up**, with the appropriate sign change to preserve handedness. The [MATLAB description](docs/registration_matlab.md) maps the objectives, parameters, transformation composition and actual solver differences. [`matlab/`](matlab/) includes a contribution viewer and an explanatory opposite-side SE(2) solver. These MATLAB functions have not been runtime-tested in MATLAB here.

Ground refinement reduced held-out ground-patch height-difference RMSE from 3.98 cm to 0.76 cm. These and trunk/mesh checks are internal agreement diagnostics, not independent survey accuracy. The antenna offset and elevation datum are uncalibrated; foliage motion/occlusions and support hardware remain relevant. Individual tree identities are still unassigned.

## Repository layout

- `processing/`: decode, register, ground refinement, export, cross-row QA and meshing.
- `docs/`, `matlab/`: method descriptions and MATLAB-format examples.
- `results/`: current transforms, scan mapping, diagnostic tables and report snapshots.
- `tools/`: mesh validation and current-release publishing.
- `.github/workflows/`: reproducible release generation, with GitHub's built-in token.
- `tests/`: analytic cross-section checks that distinguish trunk diameter from centre error.

```sh
python -m unittest discover -s tests -v
```

No repository license has been imposed; the owner can choose one.
