# Raw geometry audit: central-plane claim withdrawn

**The 82.3° value is withdrawn as a measurement of the V's structural centre plane against the ground. The centre was not directly fitted.**

The calculation fitted height-dependent point averages in the two acquisition tracks, then fitted a plane to their midpoint. It used all selected returns rather than deliberately restricting each scan to one arm, but each view is strongly biased toward its near arm. It did not identify both arms of the **same tree**, their common trunk/junction, or their structural centre. Calling this a validated central-tree-plane angle was incorrect.

Equal track weights, height matching and frame repeatability do not resolve this problem. A regression example with a truly upright, symmetric V produces a tilted centroid proxy solely because the near/far visibility weights differ between tracks. A repeatable proxy can therefore be repeatably wrong about the structural centre.

## Current conclusion

- The central V-plane/ground angle **has not been established** by this audit.
- The former 82.3° and approximately 8° lean are visibility-biased centroid-proxy results, not validated tree-centre measurements.
- Individual side-wall inclinations are supporting descriptions of visible returns. They do not determine the centre plane or prove that the actual V is tilted.
- No rotation, registration correction, trunk straightening or biological-shape conclusion is justified by the withdrawn proxy.
- Current registered point clouds, transformation matrices and mesh are unchanged by this correction.

## What a valid centre measurement needs

For a per-tree raw diagnostic, identify the trunk/junction and **both arms in the same measurement frame**, separating nearby support posts and adjacent trees. Fit the observed arm geometry with free directions; do not make either arm upright or impose equal angles to gravity. Derive an explicitly defined structural centre, for example from paired arm centrelines at matched heights, and compare its plane with that measurement's own fitted ground.

If one arm is insufficiently visible, that recording cannot independently identify the centre. It must be reported as insufficient evidence, rather than completed by averaging separately occluded views. Verified correspondence to the same tree and branches would be required before combining different raw scans.

This is a diagnostic requirement, not a V-symmetry constraint for registration. Other growing systems need their actual observed structure and an explicit centre definition; the registration must continue to preserve free trunk lean and the original rigid shapes.

## Retained raw-source evidence

The raw ground detections, original-file hashes, IMU rotations, packet/frame selection, horizontal row detections and visible-return profiles remain useful records. They read the original PCAP/JSON/H5 triples and use factory XYZ plus original IMU gravity, without registered poses, terrain flattening, PCA-up correction or learned accelerometer-bias correction.

V-hedge's geographic tracks are inferred from the original RTK data: sideA is east and sideB west for this dataset. Absolute coordinates are omitted from the public audit. Duplicate A `084742` and incomplete B `084751` are excluded explicitly.

[Public diagnostic data](../results/raw_geometry_audit.json) retain the historical numerical values for reproducibility, with `requested_measurement_available=false`. Every legacy `central_tree_plane` result now has `structural_centre_identified=false` and `valid_for_tree_centre_ground_angle=false`. These legacy names are not evidence that a structural centre was fitted. [The proxy CSV](../results/vhedge_raw_central_plane.csv) is similarly status-labelled.

The plots and `90_QC.zip → QC/vhedge/raw_geometry/` are marked as withdrawn centroid-proxy diagnostics. The source remains available through:

```bash
python tools/audit_raw_capture_geometry.py \
  --data-root /path/to/original/vhedge/data \
  --config presets/captures/vhedge.json \
  --dataset vhedge \
  --output /path/to/raw_angle_checks \
  --repeat-last-frame
```

This reproduces the diagnostic proxy and its rejection status. It does **not** implement the missing per-tree dual-arm centre fit.
