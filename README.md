# GreensprayAI

Ouster OS0-128 orchard point-cloud registration and inspection. This repository preserves the processing code and the ground-refined **chandelier v2** result from 18 stationary recordings: nine from each side of the same tree row.

## Download the point clouds

[GitHub release: chandelier v2](https://github.com/u0063882/GreensprayAI/releases/tag/chandelier-v2)

The release contains the merged cloud, all 18 separate registered clouds, and a combined cloud coloured by recording. Load the separate PLYs together in CloudCompare or MeshLab and enable RGB colours. They already share one coordinate frame in metres.

While the release is being published, the [existing download page](https://orchard-scan-results-k7m4d.mebios.chatgpt.site) also supplies these files without sign-in.

| Export | Points | Purpose |
| --- | ---: | --- |
| Merged row | 1,242,944 | Deduplicated, filtered tree row and ground strip |
| Separate/colour-by-recording clouds | 1,746,622 total | Inspect every recording's contribution; overlapping measurements are retained |

Use version 2 files together. The earlier export used different registration transforms.

![Ground-refined orchard row](https://orchard-scan-results-k7m4d.mebios.chatgpt.site/preview.png)

## Reproduce this dataset

Tested with Python 3.12. Install the pinned numerical and Ouster dependencies:

```sh
python -m venv .venv
# Linux/macOS:
source .venv/bin/activate
# Windows PowerShell instead: .venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
```

Extract the original `sideA.zip` and `sideB.zip` into this layout. Every PCAP needs its matching calibration JSON and GPS H5 file:

```text
work/data/sideA/<recording>.pcap
work/data/sideA/<recording>.json
work/data/sideA/<recording>.h5
work/data/sideB/<recording>.pcap
work/data/sideB/<recording>.json
work/data/sideB/<recording>.h5
```

```sh
python run_pipeline.py --check-inputs-only
python run_pipeline.py
```

Generated results go to `output/`; decoded samples, baseline poses, and refined poses go to `work/`. These large local files are excluded from Git. For a fresh capture or full rerun, use a fresh `work/` directory; decoded caches are specific to the current input. With an existing decoded cache and final poses, regenerate just the exports using:

```sh
python run_pipeline.py --export-only
```

These scripts reproduce this particular orchard capture. They assume nine recordings per side, this sensor mounting/FOV, and the station ordering seen in the supplied files. The crop and registration thresholds are explicit in `processing/`; they need review before applying this pipeline to a different capture. They are not a general-purpose registration library.

## Processing and validation

1. Decode strongest returns using the Ouster calibration and combine the stationary frames. Account for the sideways sensor mounting using median IMU gravity.
2. Seed the local row frame with H5 RTK latitude/longitude. H5 supplies no survey elevation; initialize vertical placement from ground returns.
3. Register overlapping scans on each side with robust ICP and a pose graph, retaining GPS/gravity priors.
4. Align the opposite sides using low stems. A foliage-only local minimum was rejected during inspection.
5. Refine each recording's rigid roll, pitch, and height using overlapping ground patches. No per-point deformation or ground flattening is applied.
6. Crop the target row and the ground directly beneath it, then export the merged and per-recording clouds.

The crop is X −0.5…6.6 m, Y 1.2875…2.6875 m, and Z −0.12…4.5 m in the local registered frame. RGB in the merged file encodes height; RGB in contribution exports identifies the recording. PLY attributes also retain `scan_id`, reflectivity, and a simple near-ground/above-ground height-band classification.

Ground refinement reduced held-out patch height-difference RMSE from **3.98 cm to 0.76 cm**. Tree overlap was checked separately, with improved upper-canopy agreement. See [quality metrics](results/chandelier-v2/ground_refinement_quality.json), [tree checks](results/chandelier-v2/ground_refinement_tree_checks.json), [transforms](results/chandelier-v2/scan_transforms.json), and [scan colours](results/chandelier-v2/scan_colours.csv).

These are internal fit diagnostics, not independent survey accuracy. The antenna offset is unknown, the vertical datum is approximate, foliage can move or be occluded, and trellis or weeds inside the crop can remain. Individual tree identities are still unassigned.

## Code and result storage

- `processing/`: the capture-specific decoding, registration, refinement, and export stages.
- `run_pipeline.py`: input validation and stage runner.
- `results/chandelier-v2/`: transforms, scan mapping, metrics, and checksummed download manifest.
- `tools/publish_result_release.py`: copies the verified ZIPs to a GitHub release.
- `.github/workflows/publish-results.yml`: publishes those release assets on relevant pushes or a manual run, using GitHub's built-in token.

The release publisher checks archive byte counts, SHA-256 digests, and ZIP integrity before upload. Raw recordings and generated point-cloud binaries stay out of Git history. No license is imposed here; the repository owner can choose one.
