Current orchard point clouds, matching matrices and the chandelier mesh. One release; one folder per dataset. Coordinates are metres and final PLYs already contain the exported poses.

A shared wood-centre PCA frame addresses common lean: approximately −10.33° for V-hedge and −8.39° for chandelier. This is a structural-up coordinate convention, not independently surveyed gravity vertical. The prior gravity direction and inverse frame transform are retained in JSON. One common rigid rotation preserves every relative pose, branch angle and measured attribute. Spindle outputs remain unchanged.

Chandelier across-row coverage expands from Y=1.2875…2.6875 m to Y=0.65…3.35 m in the earlier registration frame. X remains −1.25…7.35 m. The cloud contains 1,527,905 measured points, adding 201,026 to the preceding X-expanded result. The matching independent mesh is rebuilt and transformed with its source cloud.

**Main downloads:** 01 final point clouds, 02 final chandelier mesh, 03 matching transformation matrices.
**Supporting downloads:** 90 quality checks/provenance, 91 separate registered contributions and a recording-colour view.

Raw SIGNAL/intensity, NIR, reflectivity, leaf/wood colours and scan/frame/pixel identifiers are retained. Original states are archived in QC. No V-symmetry target, per-point warp, trunk-straightening rule or flat-ground constraint is used.

[Method, limitations and figures](https://github.com/u0063882/GreensprayAI/blob/main/docs/structural_up_report.md). `git pull` updates code and documents; download fresh release ZIPs for data.
