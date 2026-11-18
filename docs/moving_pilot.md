# Moving OS0 pilot: first 15 seconds

For a local run without GitHub Actions or release downloads, use
[the offline runner and installation guide](moving_offline.md).

The independent `dynamic/` pipeline opens only a capture-time-trimmed PCAP.
The full input ZIP is downloaded and SHA-256 checked on the Actions runner;
the remaining recording is not passed to SLAM. All packet types, including
legacy IMU packets, are retained within the selected interval.

Ouster SDK 1.0.1 uses its LIOSlamEngine with explicit constant-velocity
per-column deskew, a 15 cm internal voxel, 1–25 m returns and 50 iterations.
Firmware 3.1 legacy IMU acceleration supplies approximate initial gravity;
this is not a claim of supported fully coupled IMU deskew or surveyed vertical.
No ground leveling, trunk straightening, tree symmetry, GNSS fitting or
stationary registration is applied. The first map pose remains the gauge.

Twenty percent of pixel locations are withheld from SLAM in every frame,
including their second returns. Original ranges are restored before saving
the posed OSF and attributes. Held-out consistency compares scans roughly
0.5 seconds apart using the same measured points before and after mapping.
It is a capped nearest-neighbour diagnostic affected by sampling/occlusion,
not a surveyed accuracy score. GNSS checks use capture-host Unix timestamps
and a rigid horizontal fit without scale; antenna lever arm and timing are
uncalibrated. Absolute GNSS coordinates are omitted from reports.

The map retains selected original first returns, raw signal/intensity, NIR,
reflectivity, source frame/pixel IDs, times and provisional shared leaf/wood
RGB. This classifier was calibrated on other stationary acquisitions; colour
quality is unvalidated for this capture and is not used to fit SLAM poses.
The complete selected dual-return scans and per-column poses are in OSF.
`per_column_poses.npz` provides integer lidar timestamps and transformations;
`trajectory.csv` provides frame-centre poses. The enriched PLY uses metres.
Native Rhino files use millimetres and per-frame layers.

Pilot downloads are published on the **moving-scan-inputs** prerelease as
`pilot-15s-results.zip`, `pilot-15s-report.json`, `pilot-15s-preview.png`.
Reports and preview also appear in `results/dynamic_pilot_15s.json` and
`docs/QC/dynamic_pilot/preview.png`. This pilot never replaces Current assets.

Run locally after downloading Dynamic.zip:

```python
from dynamic.prepare import prepare_local
from dynamic.pilot import run
pcap, report = prepare_local('Dynamic.zip', 'work/pilot', seconds=15,
    expected_sha256='41a702f3a3818e555cbfdd2e8cad00d66eea94e2b70327216af33607a5d88bfa')
run(pcap, 'output/dynamic_pilot')
```

The GitHub Actions workflow is opt-in: dispatch it, or use `[dynamic-pilot]`
in a source commit message. That marker also skips stationary publishing.
