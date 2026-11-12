Current orchard point clouds, matching matrices and the chandelier mesh. One release; one folder per dataset. Coordinates are metres and final PLYs already contain the exported poses.

**Orientation correction:** the PCA output rotations (about −10.33° for V-hedge and −8.39° for chandelier) are withdrawn. They introduced a steep displayed ground slope inconsistent with the field conditions. Current clouds, contributions, matrices and mesh are restored to the original gravity-referenced registration frame. The accepted joint registration and spindle trunk refinements are retained. Canopy PCA is diagnostic only; the remaining V-hedge slant is unresolved.

Chandelier coverage remains expanded to Y=0.65…3.35 m and X=−1.25…7.35 m. Its 1,527,905 measured points include 201,026 added samples from the across-row extension. The matching independently rebuilt canopy mesh has 609,606 triangles.

**Main downloads:** 01 final point clouds, 02 final chandelier mesh, 03 matching transformation matrices.
**Supporting downloads:** 90 quality checks/provenance, 91 separate registered contributions and a recording-colour view.

Raw SIGNAL/intensity, NIR, reflectivity, leaf/wood colours and scan/frame/pixel identifiers are retained. Original states are archived in QC. Terrain variation is retained; tree axes and V-shapes are not forced upright or symmetric.

[Orientation correction, limitations and figures](https://github.com/u0063882/GreensprayAI/blob/main/docs/structural_up_report.md). `git pull` updates code and documents; download fresh release ZIPs for data.

**Raw-audit correction:** the former 82.3° centroid-proxy value is withdrawn as a structural V-centre/ground measurement. Both arms and their common centre were not identified. The true centre-plane angle remains unmeasured; current clouds, matrices, contributions and mesh are unchanged. [Corrected report](https://github.com/u0063882/GreensprayAI/blob/main/docs/raw_geometry_audit.md).

The [across-row angle report](https://github.com/u0063882/GreensprayAI/blob/main/docs/tree_ground_angle_check.md) includes all four systems and every original usable scan (67 angles), with CSVs and fitted-line figures. It is a read-only observed-point diagnostic; it does not establish the structural V centre.
