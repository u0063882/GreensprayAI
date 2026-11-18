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

## Make a GIF of the accumulating cloud and moving sensor

After registration, run the independent **`make_moving_gif.py`** script from
the repository root. Its default layout is **one row with three synchronized
panels: external 3D view | sensor-eye view | top view**.
It reads only the enriched PLY and `trajectory.csv`.
It does not read the PCAP, run registration, change poses or edit source files.

In your existing Anaconda Prompt with `(greenspray)` active, update the code
and install the small set of animation dependencies (installation is online;
rendering is offline):

```bat
git pull
python -m pip install -r requirements-animation.txt
python make_moving_gif.py --results "C:\Users\u0063882\Documents\moving_15s"
```

The run folder or its `results/` subfolder is accepted. The default output is
**`moving_15s/results/accumulation_combined.gif`**, with a settings summary beside
it at `accumulation_combined.gif.json`. Published pilot folders containing `pilot_registered.ply`
and `trajectory.csv` are also supported. Do not use the simplified Rhino PLY:
it omits the scan indices needed for time alignment.

The left panel is a fixed external 3D view, the middle panel looks from the
moving sensor, and the right panel is a fixed top view. They share the same
accumulating point sample and frame time. The fixed map panels show a red
sensor/body-origin marker and its accumulating red path. Points appear using
their saved acquisition times; the sensor position is interpolated between
the saved trajectory samples. If per-point times are absent, the script joins
`scan_id` with the trajectory's `frame_index` and reveals whole scans instead.
The last frame contains the complete display sample, including returns just
after the final scan-centre time. The GIF loops and holds the finished map for
two seconds. It animates the selected measured returns in the fused output,
rather than replaying all overlapping returns in every raw scan.

Both GIF scripts show **estimated sensor speed in m/s and km/h** at the top.
This is the norm of a local linear-fit XYZ velocity from `trajectory.csv`,
using a one-second centered time window (clipped at recording ends). It is
derived from the SLAM trajectory, not GPS, and can reflect registration errors.
Use `--speed-window 0.5` for a more responsive estimate or `--speed-window 2`
for more smoothing. The estimate uses original capture seconds; `--speed`
changes playback rate only. Both versions omit the former bottom text line.

Defaults are **1800 x 650 pixels, 10 GIF frames/s, 2x playback and at most
200,000 displayed points** for the combined layout.
One deterministic sample is selected for the whole movie; points do not change
randomly between frames. Fixed view limits use the central 99% of cloud
coordinates and include the entire sensor path. This keeps isolated outliers
from shrinking the tree row on screen. `--full-bounds` includes the full cloud
extent. These are display settings only: the source PLY is unchanged.

Display defaults gently lift dark colours using one fixed brightness curve,
use larger opaque markers, and put all panels on a charcoal background.
The gamma exponent is 0.85, bounded brightness gain is 1.6, and the brightness
floor is zero. The gain uses `g*v / (1-v+g*v)` after gamma, instead of hard
highlight clipping. Distinct source brightnesses stay ordered and distinct
before GIF quantization, including dark returns and highlights. This replaces
the earlier aggressive floor/clipping that made points look equally bright.
Point marker area remains 3 squared points. The sensor-eye view has a bounded
distance-based size increase for nearby points. Original green/brown hues and
saturation are preserved. The mapping is computed once for the complete
display sample, so colour brightness does not pump as the map accumulates.
Exact black stays black.
Raw RGB, intensity, classification and geometry in the source files are not
changed. The same settings apply to both GIF scripts and all layouts.

Adjust the display with `--point-size`, `--gamma`, `--brightness`,
`--min-brightness` and `--background`. For example, `--point-size 4 --gamma 0.45`
makes points still larger and lifts shadows more. To use the saved colours
without a brightness adjustment, pass `--gamma 1 --brightness 1 --min-brightness 0`.
You can reduce `--max-points` for quicker rendering and smaller GIFs.

### Optional normal-based surface shading

Use **`--shading normals`** to add matte surface lighting from estimated local
normals. The default remains `--shading none`, so the current appearance stays
available. Update animation dependencies once; SciPy is used for neighborhoods
and is already installed by the moving-registration requirements:

```bat
git pull
python -m pip install -r requirements-animation.txt
python make_moving_gif.py --results "C:\Users\u0063882\Documents\moving_15s" --shading normals
```

Default output: **`results/accumulation_combined_normals.gif`**, leaving the
unshaded `accumulation_combined.gif` available for comparison. The option also
works with `make_moving_gif_follow.py` and the other view layouts.

Normals are estimated once for the displayed points using local PCA and the
**full saved cloud** as neighborhood support, rather than the thinned display sample. The
default neighborhood is at most 32 points within 0.10 m. Processing uses small
batches to limit memory. Planarity confidence reduces shading on sparse,
line-like or volumetric neighborhoods; unsupported points retain their
unshaded display colour rather than receiving an arbitrary dark patch.

A world-fixed virtual light at azimuth 315 degrees and elevation 50 degrees
gives the same lighting to each point throughout the movie and in all panels.
Two-sided diffuse lighting avoids flicker from arbitrary normal sign. The
default ambient fraction is 0.6, keeping even a confident surface at least
60% as bright as its unshaded display colour. The original signal-based
brightness differences and green/brown hues are still modulated, not replaced
by a new colour map. This is **surface shading, not cast shadows or occlusion**;
there is no mesh or continuous surface to block the virtual light.

Controls: `--ambient 0.75` makes lighting subtler; `--ambient 0.45` makes it
stronger. `--light-azimuth` and `--light-elevation` turn the virtual light.
`--normal-radius` (metres) and `--normal-neighbors` adjust normal support.
The `.gif.json` report records these settings and the number of supported
display points. This is offline shading from the complete final geometry,
including later returns; it does not imply normals were observed in real time.
No normals, colours, intensity values or coordinates are written back to the
source PLY, and registration is never rerun.

Examples (each command stays on one line in Anaconda Prompt):

```bat
python make_moving_gif.py --results "C:\Users\u0063882\Documents\moving_15s" --speed 1 --max-points 300000
python make_moving_gif.py --results "C:\Users\u0063882\Documents\moving_15s" --view top --speed 4 --max-points 30000 --output "C:\Users\u0063882\Documents\moving_fast.gif"
```

For another external 3D camera, use `--azimuth -120 --elevation 25`.
The middle panel accepts the sensor-eye options described below, including
`--fov 120`. To reproduce the older external + top layout, use `--view both`;
this writes `results/accumulation.gif`. `--view all` explicitly selects the
new combined layout. Other options are
`--fps`, `--width`, `--height` and `--hold`; see `python make_moving_gif.py --help`.
More points, pixels and frames increase rendering time and GIF size. GIF timing
is rounded to 10 ms and colours are quantized to a stable 256-colour palette.
Frames are encoded incrementally so the full uncompressed movie is not held
in memory. No ffmpeg, graphics window, GPU, SDK or online service is needed by
this postprocessing script; `requirements-animation.txt` is sufficient.

### Second version: view from inside the point cloud

**`make_moving_gif_follow.py`** places a virtual perspective camera at the
saved sensor/body origin. The left panel moves through the accumulated points
using the recorded body-to-map orientations; the right panel stays a fixed
top view with the red position marker and path. It does not orbit the cloud
or change registered points. Camera translation is interpolated linearly and
quaternion rotation uses shortest-arc SLERP. The camera is calibrated upright
at the first pose, then remains rigidly attached to the moving body.

```bat
python make_moving_gif_follow.py --results "C:\Users\u0063882\Documents\moving_15s"
```

Output: **`moving_15s/results/accumulation_follow.gif`**. This older two-panel
version remains available alongside the primary combined GIF. All versions
have settings and speed estimates in their corresponding `.gif.json` summaries.

The sensor-eye version requires the `qx,qy,qz,qw` orientation columns already
saved by `run_moving.py`. The forward viewing direction defaults to body +X
for the supplied mounting; use `--look-axis y` or e.g. `--look-axis minus_x`
for another mounting. Default horizontal field of view is 100 degrees, with
near/far display clipping at 0.15 m and 25 m. Example of a wider view:

```bat
python make_moving_gif_follow.py --results "C:\Users\u0063882\Documents\moving_15s" --fov 120 --speed 1 --max-points 300000
```

`--near` and `--far` change perspective clipping in metres. These sensor-eye
options also work in the combined layout. `--full-bounds` affects the fixed
map panels; `--azimuth`/`--elevation` control the external 3D panel. Other
animation settings are shared by both scripts.

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
