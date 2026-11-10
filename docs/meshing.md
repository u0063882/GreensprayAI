# Meshing and a shrink-wrap-style canopy proxy

[Download the current mesh archive](https://github.com/u0063882/GreensprayAI/releases/download/current/orchard_mesh.zip)

The archive contains two deliberately different representations in the same registered metric frame, X along / Y across / Z up:

| File | Representation |
| --- | --- |
| `canopy_wrap.ply`, `.obj`, `.stl` | Closed voxel/SDF canopy proxy, topology validated before publication |
| `canopy_wrap_with_ground.ply` | Closed canopy plus the separate open observed ground sheet |
| `orchard_mesh.ply`, `.obj` | Open observed tree patches plus ground |
| `tree_observed_surface.ply`, `ground_surface.ply` | Separate observed components |
| `shrinkwrap_quality.json`, `mesh_quality.json` | Parameters, topology and coverage/proxy statistics |
| `shrinkwrap_preview.png`, `mesh_preview.png` | Display-only simplified previews |

PLY preserves colour. STL has no unit metadata: its coordinates here are **metres**, as are all other files. Use the closed canopy alone when inspecting closed-solid topology; adding the ground sheet produces an open scene.

## Reproduce

Extract `orchard_row_registered.ply` and `scan_transforms.json` into `output/`:

```sh
python processing/mesh.py
python processing/shrinkwrap.py --voxel 0.025 --offset 0.04 --closing-radius 0.05
python tools/validate_mesh.py
# Or include both mesh forms in a full/existing-registration pipeline:
python run_pipeline.py --mesh
python run_pipeline.py --export-only --mesh
```

On minimal Ubuntu runners Open3D also needs `libegl1`, `libgl1` and `libgomp1`; the GitHub workflow installs these system libraries. Neither mesher needs raw PCAPs. GitHub Actions builds these outputs from checksummed registered clouds and publishes them in this repository's single **Current results** release.

## Closed proxy: relation to Rhino 8 ShrinkWrap

This is a **shrink-wrap-style approximation**, not Rhino code, a reverse-engineered proprietary implementation, or a validated numerical equivalent. It uses openly specified primitives:

1. Rasterize above-ground observations on a 2.5 cm grid.
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

The release carries reports from its own build; repository report snapshots are under [`results/`](../results/). Reconstruction does not change the point-cloud poses or resolve the [cross-row QA](cross_row_quality.md).

![Closed canopy proxy](https://github.com/u0063882/GreensprayAI/releases/download/current/shrinkwrap_preview.png)
