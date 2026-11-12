# Meshing and a shrink-wrap-style canopy proxy

The current chandelier mesh was rebuilt from the expanded measured X ROI. See the [current report](joint_registration_report.md) and `QC/chandelier/roi_extension/shrinkwrap_quality.json` for its matching input hash and counts; earlier example figures/statistics below describe the original crop.

[Download the current mesh archive](https://github.com/u0063882/GreensprayAI/releases/download/current/02_final_mesh.zip)

The main mesh ZIP contains only the final representations in the registered metric frame:

| File inside `chandelier/` | Representation |
| --- | --- |
| `canopy_wrap.ply`, `.obj`, `.stl` | Closed canopy envelope |
| `ground_surface.ply` | Separate open ground sheet |
| `canopy_wrap_with_ground.ply` | Canopy plus ground, an open combined scene |

PLY preserves colour. All coordinates are metres; STL has no unit metadata. The QC ZIP holds previews, mesh reports and the open observed-surface alternatives under **QC/chandelier/meshing/**. It is separate from the main mesh download to keep that download focused.

## Mesh a point cloud alone

Registration and meshing are independent. The public interface is:

```sh
python -m pip install -r requirements-meshing.txt
python run_meshing.py --input /path/to/cloud.ply --output-dir /path/to/mesh
```

Run it from any working directory using an absolute path to `run_meshing.py`. It reads only the specified cloud and optional explicitly supplied files. No `work/`, cached decoded scans, poses, RTK, sensor calibration or previous mesh is needed. Registration never starts a mesher. The same entry point works with a MATLAB export or a cloud from another registration system.

The input coordinates and frame are preserved. Lengths must be metres; STL has no units and uses float32 vertices, so PLY is preferable for clouds with large survey coordinates. The envelope wraps all supplied points, including any ground. There is no automatic row crop, tree identity assumption or ground-at-zero threshold. Filenames `canopy_wrap.*` are retained for compatibility but can represent any input cloud. Outputs include the mesh, a preview, a source-hashed quality report and a ZIP alongside the output directory. `--archive` changes the ZIP destination. Point formats supported by Open3D can be used; PLY is the tested interchange format.

Optional metadata is passive JSON, for example:

```json
{"units":"m","coordinate_frame":"MATLAB orchard frame", "up_axis":"Z"}
```

`--metadata info.json` records this information without applying a transform. Non-metre metadata is rejected, rather than silently scaled. Normals, sensor origins and confidence may be useful for future backends; the present envelope uses XYZ only and does not interpret those fields. Colour currently illustrates input Z rather than calibrated reflectance.

`--ground ground.ply` optionally excludes input points within `--ground-clearance 0.05` metres of/below a supplied Z-up terrain mesh, using nearest XY ground vertices. It then also exports the envelope with that ground mesh. This is an explicit, approximate 2.5-D terrain recipe; use it only where the terrain mesh covers the input footprint. It is not needed for the standalone mesher. To exclude ground without this approximation, supply an already segmented tree cloud.

Controls: `--voxel`, `--offset`, `--closing-radius`, and `--min-volume`; all lengths are metres and the last is cubic metres. This prototype requires offset at least one voxel, and cannot yet compensate inflation with a negative offset. Dense grids are limited to 35 million voxels. See the [research and next backend design](shrinkwrap_research.md) for the controls still needed.

## Reproduce the orchard demonstration

Extract the final PLY and matching JSON from the point-cloud and transformation ZIPs (choose the same dataset folder). To use the commands below, save them in `output/` as `orchard_row_registered.ply` and `scan_transforms.json` respectively. The observed-surface helper remains a capture-specific Z-up orchard recipe, separate from the general meshing entry point:

```sh
python processing/mesh.py --transforms output/scan_transforms.json
python run_meshing.py --input output/orchard_row_registered.ply --output-dir output/mesh --ground output/mesh/ground_surface.ply --include-observed --archive output/orchard_mesh.zip
python tools/validate_mesh.py
```

Omit `--transforms` to estimate normal consistency from the cloud alone. Scanner transforms are optional provenance used only to orient observed-surface normals; they are neither required nor discovered automatically. `--include-observed` explicitly adds the known observed-surface files to the orchard ZIP. The general command never picks up those files just because they happen to be in its output folder.

On minimal Ubuntu runners Open3D also needs `libegl1`, `libgl1` and `libgomp1`; the GitHub workflow installs these libraries. Meshing dependencies exclude the Ouster SDK and H5/GPS reader. The publication workflow consumes exported clouds, generates this explicit demonstration and publishes to **Current results**.

## Closed proxy: relation to Rhino 8 ShrinkWrap

This is a **shrink-wrap-style approximation**, not Rhino code, a reverse-engineered proprietary implementation, or a validated numerical equivalent. It uses openly specified primitives:

1. Rasterize the supplied observations on a 2.5 cm grid (above-ground only when explicitly requested).
2. Compute a Euclidean distance field and retain the 4 cm neighbourhood of the observations.
3. Close gaps with a 5 cm-radius spherical structuring element and fill fully enclosed voids.
4. Remove occupied components smaller than 0.0005 m³.
5. Build a smoothed signed-distance field (Gaussian sigma 0.65 voxel) and extract its zero isosurface with Lewiner marching cubes.
6. Orient triangles and validate closed edges, vertex manifoldness and orientability before publishing PLY/OBJ/STL. A full global triangle self-intersection search is not run; the surface is generated using topology-preserving Lewiner marching cubes.

The intuitive parameter mapping is:

| Shrink-wrap concept | This implementation | Important distinction |
| --- | --- | --- |
| Mesh/detail scale | `--voxel` | Grid resolution, not an exact target edge-length control |
| Outward offset | `--offset` | Distance from quantized point samples; quantization and smoothing affect the final surface |
| Gap/hole closure | `--closing-radius` plus enclosed-void fill | Explicit scale-dependent geometry inference |
| Smoothing | 0.65-voxel Gaussian signed-field smoothing | Not Rhino's smoothing iterations |
| Polygon optimization | Full marching-cubes output | Only preview displays are decimated |

The wrap adds inferred gap surfaces and end caps at the point-cloud crop ends. Nearby leaves, branches or trees can merge at this scale. Its area and volume are **parameter-dependent proxy measures**, not biological leaf area, wood/biomass volume or a validated flow boundary. Different voxel/offset/closing settings will change them. Cross-row registration uncertainty remains inherited by the proxy.

## Open observed surfaces

`mesh.py` uses a 2 cm voxel sample and local normals oriented toward each supplying scanner where provenance is available. Ball-pivoting radii are 3, 4.5 and 6.5 cm. Faces with edges longer than 10 cm, or centres/edge midpoints farther than 2.5 cm from sampled observations, are rejected. Small supported patches remain because visible leaf fragments can be disconnected at this resolution.

Ground uses 4 cm XY cells and robust local low-return heights, followed by Delaunay triangulation; triangles spanning more than 11 cm of unsupported ground are removed. It is not flattened onto Z=0.

The local observed-surface run has approximately 140,000 tree and 12,000 ground triangles. About 57% of 60,000 sampled tree returns are within 3 cm of that mesh (median 2.6 cm; 95th percentile 6.3 cm). Ground median distance is about 7 mm. The observed tree mesh is open, fragmented and not vertex-manifold/watertight. These statistics expose incomplete coverage rather than imply a complete leaf/branch model.

The QC download carries reports from the data build; repository report snapshots are under [`results/`](../results/). Reconstruction does not change the point-cloud poses or resolve the [cross-row QA](cross_row_quality.md).

[Closed canopy proxy — inside QC bundle: `QC/chandelier/meshing/shrinkwrap_preview.png`](https://github.com/u0063882/GreensprayAI/releases/download/current/90_QC.zip)

## User's Rhino baseline

The [reference settings](rhino_reference.md) are now recorded: 15 mm target edges, −5 mm offset, smoothing 1, optimization 50%, point inflation enabled, followed by Reduce Mesh by 75%, retaining approximately 25% of the ShrinkWrap output polygons. These are not interchangeable with the voxel prototype's parameters. The 0.5 m ceiling in the trunk alignment diagnostics does **not** crop meshing input; the standalone mesher still receives the full supplied tree cloud.
