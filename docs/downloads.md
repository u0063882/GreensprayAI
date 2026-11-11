# Current downloads

Open [Current results → Assets](https://github.com/u0063882/GreensprayAI/releases/tag/current). There is one active release and five custom downloads:

1. **01_final_aligned_pointclouds.zip** — `chandelier/final_registered.ply` and `spindle1/final_registered.ply`.
2. **02_final_mesh.zip** — chandelier mesh formats and ground under `chandelier/`. Spindle1 has not been meshed.
3. **03_transformations.zip** — each dataset has its own `transformations.json`.
4. **90_QC.zip** — extract and open **QC/index.html** to choose a dataset and browse figures/reports.
5. **91_registered_contributions.zip** — each dataset has `separate_scans/` and `combined/` folders, in recording colours, with overlap retained.

Every dataset uses its own independent local metric frame. All PLY coordinates and matrix translations are metres. A final PLY is already registered; do not apply its matrices again. The chandelier point cloud, mesh members, matrices and scan contributions retain their original bytes.

QC is grouped under `QC/chandelier/` and `QC/spindle1/`. Each has figures and reports. Required chandelier rebuild seeds are under `QC/chandelier/provenance/registration_seed/`; spindle1's original archives are under `QC/spindle1/provenance/raw_inputs/`. The current source snapshot and collection metadata are under `QC/provenance/`. These internal inputs are not the final outputs.

GitHub adds its automatic source ZIP/tar links below these five custom files. The obsolete v2 release and tag have been removed. Previous flat download names were replaced by the shared ZIPs with dataset folders.

`git pull` updates repository code/docs, not downloads. Use a fresh download folder to avoid mixing old filenames. The three main ZIPs alone can be fetched with:

```sh
gh release download current --repo u0063882/GreensprayAI --dir final_downloads --pattern '0*' --clobber
```

## Publication

The collection publisher verifies source download hashes before extracting them. Chandelier's existing outputs stay frozen. New captures run in separate workspaces, pass held-out geometry checks, and are validated against the measured points and transformation matrices before packaging. Unchanged capture outputs are reused using code/configuration/input fingerprints; a manual `rebuild` regenerates the named raw capture datasets, while `repackage` refuses to process a changed capture.

All five replacement downloads must pass server checksum verification before obsolete flat attachments are removed. Current archive hashes are in the release notes; the four non-QC download hashes are also in `QC/SHA256SUMS.txt`. Individual dataset output hashes and build metadata are in QC provenance. The legacy chandelier raw/seed recipe remains documented separately.
