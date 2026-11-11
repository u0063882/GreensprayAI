# Registration of named captures

`run_capture.py` runs registration for one dataset in one workspace. It neither imports a mesh reconstruction method nor depends on a mesh. Each new capture needs its own configuration under `presets/captures/`, containing a reviewed row crop, low-stem windows and bounded correction limits.

A/B names do not encode east/west or scanner heading. The pipeline projects RTK coordinates to a local horizontal frame with X following the first side's traverse and Y to its left. The signed displacement to the other track determines each side's inward sensor-forward heading. Swapping the side labels or traverse direction therefore does not reuse chandelier's compass assumptions.

The stages are:

1. Decode Ouster PCAPs using their matching calibration JSON. Median IMU acceleration defines up. The capture decoder retains measured first-return samples within its sensor-forward row window and 12 m range, with 10 mm cell deduplication. H5 RTK supplies a coarse position, not surveyed lidar coordinates or altitude.
2. Register overlapping recordings within each side with robust multiscale point-to-plane ICP and a pose graph regularized by RTK/gravity. This initialization can use above-ground tree geometry.
3. Find a low-return SE(2) coarse match between the sides using multistart initialization. This is an initializer, not a final common-offset restriction.
4. Refine relative rigid roll/pitch/height from overlapping 15 cm ground patches. Every fourth spatial cell is withheld. The terrain is retained rather than flattened.
5. Freeze the assembled reference side. Fit robust leaning/tapered low-stem reference models, flag poor/boundary fits, insert the other side's recordings separately, and jointly refine their bounded SE(3) poses. Residuals use radial bark distance, scanner-facing consistency, ground and same-face scan overlap.
6. Promote only candidates that pass the declared held-out checks; export original measured samples and complete sensor-to-final matrices. Apply one common final frame rotation/translation to put the configured row axis along X and its centre at Y=0.

Reference models use points below 0.48 m. Moving-side stem fitting uses disjoint bands 0.12–0.20, 0.26–0.34 and 0.40–0.46 m. The intervening height bands are excluded from the bounded pose residuals; the transformed fitting samples must stay below 0.5 m. Checkerboard ground and same-face matches are also withheld. The bounds, individual correction matrices, direct stem support and validation counts are recorded in the JSON report.

Spindle1 has four usable reference stem models. Omitting an entire end stem leaves some recordings without direct trunk support, so the final configuration uses all four and validates on the disjoint heights/spatial patches. There is no entire-stem holdout in that configuration. These checks share acquisitions, selected windows, coarse alignment and circular-model assumptions; they do not establish independent survey accuracy. Branch/support clutter and short arcs can invalidate diagnostic circle centres even below 0.5 m. Dashed circles denote uncertain opposite-side centre comparisons.

## Output and reproducibility

The workspace `final/` contains the merged PLY, transformation JSON, separate recording-coloured PLYs, combined contributions and `QC/index.html`. Only the configured row strip and its underlying ground are retained. A first measured sample is retained per 12 mm merged voxel, followed by radius-outlier removal; there is no per-point warping or synthesized bark. Separate contribution clouds retain overlaps.

`transformations.json` identifies the original PCAP and its SHA-256 for every recording. Column-vector convention:

```matlab
p_final_h = T_sensor_to_final * [p_ouster_sensor(:); 1];
T_sensor_to_final = F_row * T_registered_levelled * R_gravity;
% The final exported PLY already contains p_final: do not transform it again.
```

Each capture has independent caches and input hashes. Decoder caches also include the PCAP/JSON/H5 hashes, decoder source hash and SDK version, so changed decoding code cannot silently reuse old samples. A changed input manifest requires a fresh workspace. Changes to registration code/configuration invalidate the pose caches. Export checks confirm rigid matrices, original reflectivity and transformed float coordinates. The cloud and matrices use metres and a local ground datum; the antenna lever arm and absolute elevation remain uncalibrated.

For the collection release, original spindle1 archives are archived inside QC provenance. Small bootstrap Git blob IDs only transfer the first copy into the release; subsequent builds prefer the verified archived inputs. Raw files and point-cloud binaries are not added to the main branch's file tree. Chandelier remains a frozen validated dataset during this migration.

## Vhedge inputs and wider canopy

The two vhedge ZIPs contain 15 PCAP/JSON/H5 entries but 14 unique recordings. `084742` is byte-identical in both archives and belongs to the B track by RTK; only its B copy is selected. B `084751` contains one lidar packet, no IMU and zero elapsed recording time, so it cannot supply a stationary full-view cloud or its own gravity direction. The vhedge preset explicitly excludes those two archive entries, leaving seven A and six B recordings. Repeated captures at the same RTK station have different measured packets and are retained.

The original archives and all input hashes are preserved. Exclusions appear in the capture configuration, QC report and final transformation JSON. The decoder checks that excluded paths exist and requires at least three usable recordings on each side. It refuses to invent a gravity vector for missing IMU data.

The V canopy requires a wider crop than spindle1: a 5.1 m along-row section and 3.1 m strip, with its ground beneath. Six reviewed low-stem windows support the final bounded stage. Some opposing circle fits remain uncertain because supports, branches and partial bark arcs can mix even below 0.5 m. The across-row profile in QC shows the retained V geometry. These conditional model checks are not independent survey accuracy.

Published datasets can be marked `frozen` in the collection manifest. Their component hashes are checked, their original processing snapshot is retained, and their geometry is preserved while later datasets are added. Each new raw-capture build records its source commit and retains its own processing-code snapshot.
