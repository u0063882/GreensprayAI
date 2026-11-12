# Registration in MATLAB terms

This describes the registration actually used here and maps it to the row frame in `u0063882/GreenSprayToolbox`. That repository is referenced for comparison and has not been modified.

## Along-row and across-row axes

| Quantity | Python exports here | MATLAB toolbox row frame |
| --- | --- | --- |
| Along the tree row | X, approximately Side-A travel direction | +Y |
| Across the tree row | Y, to the left of +X | X, opposite the Python across sign |
| Vertical | +Z | +Z |

A proper rotation maps the current registered exports into the MATLAB convention:

```matlab
R = [0 -1 0; 1 0 0; 0 0 1];
t = [1.9875 0 0]; % current crop centre, not a surveyed trunk plane
pcMatlab = pctransform(pcPython, rigidtform3d(R,t));
% X_across_matlab = 1.9875 - Y_across_python
% Y_along_matlab  = X_along_python
% Z_matlab        = Z_python
```

The negative sign preserves a right-handed frame; exchanging X and Y alone would be a reflection. `matlab/viewRegisteredContributions.m` applies this conversion to the separate registered PLYs. The conversion changes display coordinates; it does not re-register the data.

## 1. Gravity and RTK initialisation

Ouster XYZ decoding uses the capture's JSON calibration. A median stationary IMU specific-force vector provides **up** in the sensor frame. A proper rotation levels the sensor; yaw remains undetermined by gravity. RTK latitude/longitude give a local east/north translation; the orchard row direction is estimated from Side-A station positions. The inward-looking sides provide coarse yaw hypotheses of opposite sign. Vertical offsets are initialized from local low ground because H5 contains no altitude.

In MATLAB matrix form, with points stored as Nx3:

```matlab
Plevel = P_sensor * Rlevel.';
Pcoarse = Plevel * Rcoarse.' + tcoarse.';
% Column-vector homogeneous equivalent:
Tcoarse = [Rcoarse, tcoarse; 0 0 0 1];
```

The current Python GPS conversion is a short-baseline spherical tangent approximation. `greenspray_localPose.m` in the MATLAB toolbox uses WGS84 ECEF-to-ENU instead. These are similar initializers for this short traverse, not identical implementations. An antenna-to-LiDAR lever arm and absolute boresight have not been calibrated.

## 2. Neighbours on each side

Here, same-side overlapping pairs up to three station steps apart are matched with robust **point-to-plane SE(3) ICP**, using successive correspondence limits 0.25, 0.13 and 0.07 m on 0.035 m voxel samples. Implausible pose changes and low-overlap pairs are rejected. A global pose graph then reconciles accepted pair transforms with position/gravity/yaw priors.

A conceptual MATLAB equivalent is:

```matlab
movingSmall = pcdownsample(moving,"gridAverage",0.035);
fixedSmall  = pcdownsample(fixed, "gridAverage",0.035);
% pcregistericp(...,Metric="pointToPlane",InitialTransform=...) is the
% nearest built-in stage, but reproducing the Tukey loss, correspondence
% schedule, rejection gates and pose graph requires the custom objective.
```

The residual for pair (i,j), where Ti maps scan i into the common frame, is based on `inv(Tij) * inv(Tj) * Ti`. Rotation and translation residuals are scaled separately. The toolbox's `greenspray_registerNeighbourScan.m` instead uses bounded XY/yaw refinement, with an optional phase-correlation seed; it accumulates neighbours separately per side. That is the same broad placement/refinement strategy, but a different solver.

## 3. Coarse opposite-side seed: along and across together

The toolbox currently keeps West and East independent. The seed pipeline adds an explicit inter-side initializer, followed by the new per-recording bounded stage below. Low stem points, Z=0.13…0.38 m, are downsampled to 0.025 m. Side B is moved onto Side A using only a planar rigid transform:

```matlab
Rz = [cos(theta) -sin(theta) 0;
      sin(theta)  cos(theta) 0;
      0           0          1];
T_B_to_A = [Rz, [dAcross; dAlong; 0]; 0 0 0 1];
```

In this MATLAB convention the optimized parameters are `[dAcross, dAlong, yaw]`; in Python they are `[dAlong, dAcross, yaw]` with the across sign reversed. Multi-start translations avoid matching a neighbouring tree or a repeated canopy pattern. Each iteration finds XY nearest neighbours, gates them at 0.25/0.13/0.07 m and computes a weighted proper Procrustes/SVD update. Weights are `1/(1+(distance/0.045)^2)`. Candidate selection uses the symmetric mean squared nearest-neighbour distance capped at 0.12 m. Vertical shift, roll and pitch are not optimized in this inter-side step.

`matlab/registerOppositeSidesSE2.m` is an explanatory numerical port of that solver in the MATLAB row convention. It has not been executed in MATLAB here; the Python seed implementation remains `processing/trunk_align.py`. This aggregate is an initializer, not the final restriction on west degrees of freedom.

Critically, **opposite-facing bark returns are not the same surface**. Blindly minimizing their separation can shrink a real trunk. Current QA fits independent circles and uncertainty per height. The final stage uses fixed east stem geometry, scanner-facing consistency and separate west poses; it does not match opposite bark returns directly onto one another.

## 4. Relative ground refinement

After the low-stem registration, each acquisition gets small rigid roll, pitch and height corrections. Overlapping 0.15 m XY ground cells supply height differences. Every fourth spatial cell is excluded from fitting and used as a holdout. Robust bounded IRLS minimizes those differences with a strong mean-correction anchor. This preserves relative terrain shape; it does not project all ground to Z=0.

Linearized, at pivot `p0`, in the Python frame:

```matlab
zCorrected = z + dZ + roll*(y-p0(2)) - pitch*(x-p0(1));
residual_ij = zCorrected_i - zCorrected_j;
% Apply the final fitted parameters as a full rigid rotation/translation,
% not as this per-point linearized height formula.
```

Under the MATLAB axis mapping, small rotation components become `roll_matlab=-pitch_python` and `pitch_matlab=roll_python`; `dZ` is unchanged. The toolbox's per-scan plane-to-Z=0 alignment is a useful initializer but differs from this joint relative-ground solve.

## Complete transform and MATLAB conventions

For raw **SDK sensor-frame** points:

```matlab
T_seed = Eground_i * Cside_i * Pwithin_i * Llevel_i;
T_sensor_to_python = Dwest_i * T_seed;
% Dwest_i is identity for east; each west recording has its own correction.
% Cside_i is identity for A and the fitted opposite-side transform for B.
C = [0 -1 0 1.9875; 1 0 0 0; 0 0 1 0; 0 0 0 1];
T_sensor_to_matlabRow = C * T_sensor_to_python;
R = T_sensor_to_matlabRow(1:3,1:3);
t = T_sensor_to_matlabRow(1:3,4).';
pcRegistered = pctransform(pcSensor,rigidtform3d(R,t));
% For legacy affine3d's POST-multiply convention: affine3d(T_sensor_to_matlabRow.').
```

`results/scan_transforms.json` stores `sensor_to_merged` as column-vector 4x4 matrices. Before applying these to MATLAB-decoded raw points, verify that `ousterFileReader/readOS0` returns the same SDK **sensor** frame; a LiDAR-frame-to-sensor calibration transform may also be needed. Already registered PLYs need only the display-axis conversion above.

## 5. Final bounded west poses with fixed east

See the [implemented method and validation](anchored_registration.md). East/A is MATLAB scan indices 1:9; west/B is 10:18 (Python IDs 0:8 and 9:17). The already completed east solution is fixed. There are nine separate six-component west pose updates, with soft relative-pose/overlap constraints rather than a single aggregate transform.

MATLAB-style expressions for each west correction:

```matlab
omega = x(1:3); % rotation vector, radians
S = [0 -omega(3) omega(2); omega(3) 0 -omega(1); -omega(2) omega(1) 0];
R = expm(S);
dt = x(4:6);   % [along; across; height], metres in the Python frame
Dwest_i = [R, pivot_i - R*pivot_i + dt; 0 0 0 1];
Tfinal_i = Dwest_i * Tseed_i;
```

The translation bounds are ±[0.04, 0.15, 0.025] m at the local row pivot. Rotation-vector component bounds are ±[0.5, 0.5, 2] degrees. Initializers are fitted one recording at a time and then jointly refined in a bounded least-squares objective. Physical relative-pose priors compare corrections at a common midpoint, so identical world transforms have zero relative residual even when pivot positions differ.

East low stems are leaning tapered circular-axis models. With row-vector points, their radial objective is:

```matlab
u = [slopeAlong; slopeAcross; 1]; u = u / norm(u);
Q = P - centreAtZ03.';
t = Q*u;
radial = Q - t*u.';
r = radiusAtZ03 + taper*t;
e = sqrt(sum(radial.^2,2)) - r;
% Also penalize model surface normals facing away from the supplying scanner.
```

This preserves opposite bark faces and supports lean/taper without forcing vertical stems. The objective adds local east-ground planes, same-face west overlap, and bounded pose priors. West candidate 5 and two low-height bands are excluded for validation. Ground/overlap checkerboards provide additional checks, conditional on model/correspondence assumptions. The expressions specify the Python method; a full MATLAB optimizer for this stage has not been runtime-tested or supplied.

## Subsequent joint stage (current V-hedge)

The [current report](joint_registration_report.md) supersedes the fixed-side assumption for V-hedge. The former registration supplies initial poses. Both sides then receive per-recording 6-DOF rigid corrections, with a coordinate-origin/yaw gauge at one recording and soft gravity constraints. Low-trunk axes/radii/taper are shared variables and remain free to lean; measured local terrain planes may slope.

With X along the row, Y across it and Z up, the important residuals can be written in MATLAB terms as:

```matlab
% Each observation and normal belongs to its original recording.
piWorld = Ri*pi + ti;
pjWorld = Rj*pj + tj;
niWorld = Ri*ni;
njWorld = Rj*nj;
rGround = [niWorld.'*(piWorld-pjWorld); ...
           njWorld.'*(piWorld-pjWorld)]; % no Z=0 or horizontal-plane target

axis = [leanX; leanY; 1]; axis = axis/norm(axis);
d = piWorld - [centreX; centreY; 0.30];
s = axis.'*d;
rTrunk = norm(d-axis*s) - (radius + taper*s);

g = Ri*upInCurrentFrame;
rGravity = g(1:2)/deg2rad(0.75); % weighted prior, not a soil normal

Tfinal = Cjoint * Crecovery * Tbaseline;
Pfinal = (Tfinal * [Psensor, ones(size(Psensor,1),1)].').';
Pfinal = Pfinal(:,1:3);
```

Actual fitting includes robust losses, source visibility, return-set exclusions, plausible pose bounds and same-facing wood overlap. Disconnected within-side recordings first receive a validated multistart recovery. No prescribed branch angle, symmetry, upright trunk or per-point warp is used. The saved final JSON matrices already include the composition above; loading a final PLY does not require another transformation.

## Current output frame

The PCA output-frame trial was withdrawn because it introduced a ground slope inconsistent with the field conditions. Final point clouds and saved sensor-to-output matrices again use the original gravity-referenced registration frame. Do not apply a canopy-PCA rotation to these files. Soil can have a slight real slope; neither a soil-plane normal nor a tree-growth PCA axis is an independent measurement of vertical.

```matlab
% T is scans(i).sensor_to_merged from the CURRENT transformations.json.
% Apply only when starting with that scan's decoded sensor-frame coordinates:
p_registered = p_sensor * T(1:3,1:3).' + T(1:3,4).'; % N-by-3
% Current final PLY coordinates are already registered; plot them directly.
```

The historical structural-frame estimator remains available for explicit research experiments. Release generation uses it only as a diagnostic, with zero applied PCA rotation. See [orientation correction](structural_up_report.md).
