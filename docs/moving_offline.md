# Run moving-sensor registration on your own computer

Use **`run_moving.py`** from the repository root. It runs the same Ouster SDK
registration and quality checks used for the 15-second moving pilot. All
inputs and outputs stay on your computer. There are no GitHub, ChatGPT,
cloud-processing or paid-API calls in this command. No GitHub token is needed.

The first installation needs internet access to obtain Python packages.
After installation, local processing works offline. Source code is available
through `git pull` or GitHub **Code → Download ZIP**; extract the entire
repository, not just `run_moving.py`, because it imports the included modules
and shared colour model. `Dynamic.zip` is a separate input download; git pull
does not download release assets.

## 1. Install once

Use **64-bit Python 3.12**. These versions were exercised on Linux with Python
3.12; the Windows commands below avoid PowerShell activation-policy changes.
Windows execution has not been tested in this environment.

In Windows PowerShell, from the `GreensprayAI` folder:

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install --upgrade pip
.\.venv\Scripts\python.exe -m pip install -r requirements-moving.txt
```

On Linux/macOS:

```sh
python3.12 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements-moving.txt
```

On Ubuntu, if Open3D reports missing graphics/runtime libraries, install them
while online: `sudo apt-get install libegl1 libgl1 libgomp1`. Processing is
headless and does not require a graphical window or GPU. Package wheels must
be available for your OS/CPU/Python combination; the tested setup is Linux x86-64.

## 2. Run your Dynamic.zip: first 15 seconds

Change the input path to where you saved the existing `Dynamic.zip`.
The output directory must be new or empty; pick another name for another run.

Windows PowerShell:

```powershell
.\.venv\Scripts\python.exe run_moving.py --zip "D:\Lidar\Dynamic.zip" --seconds 15 --gps-clock shared-start --output "D:\Lidar\moving_15s"
```

Linux/macOS, with the virtual environment active:

```sh
python run_moving.py --zip "/path/to/Dynamic.zip" --seconds 15 --gps-clock shared-start --output "/path/to/moving_15s"
```

The runner reads a **local** ZIP containing exactly one classic PCAP and its
matching Ouster sensor JSON. A matching H5 is optional. Nested ZIP folders
are accepted. It computes the local archive SHA-256, decompresses only the
selected PCAP prefix and preserves all packet types inside that time interval.
It does not alter the original ZIP. Keep enough disk space for the prepared
PCAP plus several output formats.

## Alternative: already extracted PCAP, JSON and H5

```sh
python run_moving.py --pcap "/path/to/recording.pcap" --metadata "/path/to/sensor.json" --gps "/path/to/navigation.h5" --seconds 15 --gps-clock shared-start --output "/path/to/moving_15s"
```

On Windows, replace `python` with `.\.venv\Scripts\python.exe` as above.
If JSON and H5 share the PCAP basename and directory, omit `--metadata` and
`--gps`. Sensor JSON is required; GPS is optional. Explicit paths allow
metadata/navigation files with different names. ZIP mode reads its own
matching metadata and does not accept those separate path options.

Supported raw input here is **classic PCAP**, including microsecond and
nanosecond timestamps; PCAPNG is rejected with an error. This workflow was
developed for the supplied OS0-128 profile and its channel fields. It is not
a general multi-sensor calibration or arbitrary LiDAR importer.

## GPS timing: shared software start

`--gps-clock shared-start` implements the acquisition assumption you supplied:

```python
gps_query_time = first_valid_gps_time + (frame_capture_time - first_pcap_packet_time)
```

Positions are interpolated from the **actual H5 timestamps**, not matched by
sample index. Thus 614 frames and 573 GPS samples are supported, including
irregular GPS intervals. No time stretching is used to force endpoints to
agree. Startup latency remains an uncertainty. This option should only be
used when the start of this PCAP and the start of this H5 belong to the same
acquisition; an independently trimmed PCAP needs its original timing offset.

Default `--gps-clock absolute` compares timestamps directly, reproducing the
pilot's original clock policy; for your relative-PCAP/calendar-GPS pair it
reports insufficient overlap. GPS is a **diagnostic only** under either policy.
It never changes registration poses. The comparison fits horizontal rotation
and translation without scale; the antenna-to-LiDAR lever arm is uncalibrated.
Absolute latitude and longitude are omitted from derived reports.

## Outputs

```text
moving_15s/
  input/                       # prepared PCAP prefix, sensor JSON, optional H5
    preparation.json           # actual time interval and checksum
  results/
    registered.ply             # enriched point cloud, metres; poses already applied
    registered.osf             # selected dual-return scans with per-column poses
    transformations.json       # frame-centre body-to-map matrices and convention
    per_column_poses.npz       # full column matrices, integer times and frame IDs
    trajectory.csv             # scan-centre position and quaternion, qx qy qz qw
    report.json                 # configuration, GPS/overlap/ground checks and flags
    preview.png
    README.txt
    rhino/
      final_registered.3dm     # preferred Rhino import; units MILLIMETRES
      final_registered_xyzrgb_ascii.ply  # simple ASCII compatibility cloud, METRES
      export_quality.json
```

The enriched cloud preserves selected real measured first returns, raw
SIGNAL/intensity, NIR, reflectivity, frame/pixel IDs and derived leaf/wood RGB.
`scan_id` identifies the processed moving scan; `frame_id` is its sensor frame
ID. Registered coordinates already contain the transforms; do not apply them
again. Use per-column matrices rather than frame-centre summaries to reproduce
deskewed coordinates. The legacy names `host_unix`/`capture_host_unix` in the
NPZ/CSV hold the raw capture clock, which can be relative rather than Unix time.
The staged `input/` H5 remains an unchanged copy and contains your original GPS.

The processing matches the pilot: constant-velocity per-column deskew, median
initial accelerometer gravity, 15 cm internal voxel, 1–25 m range, 50 optimizer
iterations, and 20% held-out pixel locations for overlap checks. Those ranges
are restored before scan export. Map sampling keeps a real return per 4 cm
frame voxel and then 2.5 cm map voxel. Sampling spacing is not accuracy.

No tree-shape symmetry, ground flattening, GPS fitting, loop closure or pose
graph optimization is imposed. The shared leaf/wood model is provisional for
this recording and is not used to fit poses. The mesh program remains
independent: this command produces a point cloud, not a mesh.

## Longer recordings and options

The default is **15 seconds**. To process a different interval, change
`--seconds 15` to e.g. `--seconds 30`. To process the complete local input:

```sh
python run_moving.py --zip "/path/to/Dynamic.zip" --all --gps-clock shared-start --output "/path/to/moving_full"
```

`--all` is an explicit user choice. Only the 15-second moving pilot has been
evaluated so far. Longer runs use more RAM, disk and time; this implementation
keeps scan poses and sampled points in memory. Inspect `report.json`, including
orientation jumps, before trusting the output. A stopped/flagged run is not a
complete reconstruction merely because an output file exists.

`--voxel 0.15` and `--max-range 25` expose the existing SDK settings in metres.
`--no-rhino` skips the native Rhino and ASCII compatibility copies.
`python run_moving.py --help` lists the options. Running the command does not
consume ChatGPT tokens or publish anything online.

## Installing packages on a disconnected machine

Before disconnecting, on a computer with the **same OS, CPU architecture and
Python 3.12**, download all required wheels:

```sh
python -m pip download --only-binary=:all: -r requirements-moving.txt --dest wheelhouse
```

Copy the repository and `wheelhouse/` to the offline computer. In its Python
3.12 virtual environment, install without contacting a package server:

```sh
python -m pip install --no-index --find-links wheelhouse -r requirements-moving.txt
```

Linux system runtime libraries must also have been provisioned. Large wheel
files need not be committed to Git. Optional algorithm checks:

```sh
python -m unittest tests.test_dynamic_pilot tests.test_dynamic_offline -v
```
