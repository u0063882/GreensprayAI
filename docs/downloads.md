# Current downloads

Open [Current results → Assets](https://github.com/u0063882/GreensprayAI/releases/tag/current). The numbered names put the three main downloads first:

1. **01_final_aligned_pointcloud.ply** — final merged point cloud, ready to open.
2. **02_final_mesh.zip** — final canopy PLY/OBJ/STL, ground sheet and combined scene.
3. **03_transformations.json** — the final matrices for all 18 acquisitions.

These are the existing validated outputs with cleaner packaging. Coordinates, mesh members and transformation values are preserved byte for byte. PLY coordinates and JSON translations are metres. The final PLY is already registered; do not apply the matrices to it again.

The two supplementary downloads are:

- **90_QC.zip**: extract it and open **QC/index.html** for an offline figure gallery. `registration/` holds pose/stem comparisons and reports; `trunk_slices/` holds circles, centre plots, PDF and tables; `meshing/` holds previews/reports and observed-surface alternatives; `provenance/` holds source hashes, data/build commits and code snapshots. Any other previous attachments are preserved under `additional_assets/`.
- **91_registered_contributions.zip**: `separate_scans/` has the 18 uniformly coloured scans; `combined/` has the same contributions together, with overlap preserved. Both retain their original manifests and notes.

GitHub automatically adds its own source-code ZIP/tar links below the custom assets. The custom list contains five downloads. Old flat attachment URLs have been replaced by this layout; the corresponding diagnostic files are now in the QC ZIP.

`git pull` updates repository code/docs, not these downloads. Download into a fresh folder to avoid mixing old filenames with the reorganized files. With GitHub CLI, the three main files alone can be fetched using:

```sh
gh release download current --repo u0063882/GreensprayAI --dir final_downloads --pattern '0*' --clobber
```

## Publication

The workflow compares geometry-affecting code/configuration with the current release tag. Processing changes rebuild and validate the dataset; documentation/packaging changes reuse the validated release bytes. A manual workflow run can force `rebuild` or `repackage`.

Repackaging first verifies every source asset against GitHub's SHA-256 digest, creates and checks the bundles, uploads and verifies all replacements, then removes the old flat attachments. Required rebuild inputs are archived under `QC/provenance/registration_seed/`; the obsolete v2 release and tag are removed after those inputs are verified. Primary-file hashes are inside the QC archive; all five archive/download hashes are listed in the release notes. The QC provenance records the original data commit separately from the current packaging commit.
