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

## 3. Opposite-side registration: along and across together

The toolbox currently keeps West and East independent. This pipeline adds an explicit inter-side registration. Low stem points, Z=0.13…0.38 m, are downsampled to 0.025 m. Side B is moved onto Side A using only a planar rigid transform:

```matlab
Rz = [cos(theta) -sin(theta) 0;
      sin(theta)  cos(theta) 0;
      0           0          1];
T_B_to_A = [Rz, [dAcross; dAlong; 0]; 0 0 0 1];
```

In this MATLAB convention the optimized parameters are `[dAcross, dAlong, yaw]`; in Python they are `[dAlong, dAcross, yaw]` with the across sign reversed. Multi-start translations avoid matching a neighbouring tree or a repeated canopy pattern. Each iteration finds XY nearest neighbours, gates them at 0.25/0.13/0.07 m and computes a weighted proper Procrustes/SVD update. Weights are `1/(1+(distance/0.045)^2)`. Candidate selection uses the symmetric mean squared nearest-neighbour distance capped at 0.12 m. Vertical shift, roll and pitch are not optimized in this inter-side step.

`matlab/registerOppositeSidesSE2.m` is an explanatory numerical port of that solver in the MATLAB row convention. It has not been executed in MATLAB here; the production implementation remains `processing/trunk_align.py`.

Critically, **opposite-facing bark returns are not the same surface**. Blindly minimizing their separation can shrink a real trunk. Current cross-row QA fits candidate cross-sections separately by side, reports arc coverage and acquisition-block-bootstrap uncertainty, and declines to infer a global correction from too few usable trunks.

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
T_sensor_to_python = Eground_i * Cside_i * Pwithin_i * Llevel_i;
% Cside_i is identity for A and the fitted opposite-side transform for B.
C = [0 -1 0 1.9875; 1 0 0 0; 0 0 1 0; 0 0 0 1];
T_sensor_to_matlabRow = C * T_sensor_to_python;
R = T_sensor_to_matlabRow(1:3,1:3);
t = T_sensor_to_matlabRow(1:3,4).';
pcRegistered = pctransform(pcSensor,rigidtform3d(R,t));
% For legacy affine3d's POST-multiply convention: affine3d(T_sensor_to_matlabRow.').
```

`results/scan_transforms.json` stores `sensor_to_merged` as column-vector 4x4 matrices. Before applying these to MATLAB-decoded raw points, verify that `ousterFileReader/readOS0` returns the same SDK **sensor** frame; a LiDAR-frame-to-sensor calibration transform may also be needed. Already registered PLYs need only the display-axis conversion above.
