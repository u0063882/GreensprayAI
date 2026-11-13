# Current orchard point clouds and mesh

[Current registration and orientation report](https://github.com/u0063882/GreensprayAI/blob/main/docs/balanced_registration.md). All four datasets have new ground-first local output frames. Validated joint scan-pose refinements are added for Spindle 2 and Chandelier. Spindle 1 and V-hedge retain their accepted relative scan geometry because further joint iterations did not pass the existing improvement threshold.

The final ground tilts are approximately 0.19°, 0.23°, 0.78° and 1.77° for Spindle 1, Spindle 2, Chandelier and V-hedge. The observed centre-direction tilts are approximately 0.75°, 0.91°, 3.11° and 7.10°. V-hedge retains a visible residual lean. These are chosen local frames and observed-envelope diagnostics, not calibrated Earth vertical or verified biological symmetry.

| Main asset | Contents |
| --- | --- |
| `01_final_aligned_pointclouds.zip` | New final PLY per dataset folder |
| `02_final_mesh.zip` | Chandelier canopy PLY/OBJ/STL and ground scene, rebuilt from its exact current cloud |
| `03_transformations.zip` | Matching composed original-sensor and gravity-levelled matrices per dataset |

Final matrices are already applied to PLY coordinates. Retained measured samples, point order, intensity/SIGNAL, NIR, reflectivity, derived leaf/wood RGB and source IDs are unchanged. No scaling, shear, per-point terrain flattening, trunk straightening or V-symmetry target is applied.

`90_QC.zip` contains current reports and previous accepted geometry/matrices/mesh under `QC/<dataset>/provenance/balanced_registration_baseline.zip`. Open `QC/index.html` for current results; earlier galleries refer to earlier working frames. `91_registered_contributions.zip` contains separate leaf/wood-coloured scans and combined recording-coloured PLYs in the same final frames. Current code is in the repo and `QC/provenance/current_code_and_notes.zip`.

Registration and meshing remain independent. Chandelier's rebuilt canopy is watertight; the optional ground sheet is open. Other systems have no mesh yet. Meshing is an envelope estimate that inherits point-cloud uncertainty.

The earlier 82.3° raw centroid-proxy claim remains withdrawn as a structural V-centre measurement. The ground-ROI and raw per-scan angle reports are historical diagnostics, not evidence of a surveyed centre plane.
