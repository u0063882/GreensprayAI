"""Local moving-registration command, called by run_moving.py.

Workflow: parse options -> prepare an isolated PCAP interval and metadata ->
call dynamic.pilot.run() -> leave results on disk. This module uses our input
preparation code; the actual Ouster SDK mapping calls live in pilot.py.

GPS options configure a diagnostic comparison only. They do not turn on GPS
pose constraints or change the SDK optimizer. See docs/moving_registration_workflow.md.
No local command here downloads data or publishes results.
"""

import argparse
import json
import math
import shutil
import zipfile
from pathlib import Path

from dynamic.prepare import prepare_local, trim_classic_pcap


def positive(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("must be a positive finite number")
    return number


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    source = p.add_mutually_exclusive_group(required=True)
    source.add_argument("--zip", type=Path, help="Local ZIP with one PCAP and matching sensor JSON")
    source.add_argument("--pcap", type=Path, help="Local classic PCAP file")
    p.add_argument(
        "--metadata", type=Path, help="Sensor JSON; otherwise use the PCAP basename plus .json"
    )
    p.add_argument(
        "--gps", type=Path, help="GPS H5 with time/lat/lon; otherwise use matching .h5 if present"
    )
    duration = p.add_mutually_exclusive_group()
    duration.add_argument(
        "--seconds", type=positive, default=15.0, help="First N capture seconds; default: 15"
    )
    duration.add_argument(
        "--all", action="store_true", help="Explicitly process the complete local recording"
    )
    p.add_argument("--output", type=Path, required=True, help="New or empty output directory")
    p.add_argument(
        "--gps-clock",
        choices=["absolute", "shared-start"],
        default="absolute",
        help="GPS comparison clock only (not a SLAM constraint); shared-start assumes both recordings started together",
    )
    p.add_argument(
        "--voxel",
        type=positive,
        default=0.15,
        help="Internal registration voxel in metres; default: 0.15",
    )
    p.add_argument(
        "--max-range",
        type=positive,
        default=25.0,
        help="Maximum fitting/export range in metres; default: 25",
    )
    p.add_argument(
        "--no-rhino",
        action="store_true",
        help="Skip native Rhino and simple ASCII PLY compatibility copies",
    )
    return p


def prepare_input(args):
    """[PROJECT] Stage a PCAP interval plus calibration and optional GPS H5.

    This copies metadata/navigation bytes and trims only the capture packet
    interval. It does not decode point geometry, estimate poses, resample GPS,
    or align trees. The original input files are never edited.
    """
    source = (args.zip or args.pcap).expanduser().resolve()
    if not source.is_file():
        raise FileNotFoundError(f"Input file not found: {source}")
    if args.zip and (args.metadata or args.gps):
        raise ValueError(
            "--metadata and --gps are for --pcap; ZIP mode reads matching files inside the archive"
        )
    metadata = gps = None
    if args.pcap:
        metadata = (args.metadata or source.with_suffix(".json")).expanduser().resolve()
        if not metadata.is_file():
            raise FileNotFoundError(f"Sensor metadata not found: {metadata}; supply --metadata")
        gps = (args.gps or source.with_suffix(".h5")).expanduser().resolve()
        if args.gps and not gps.is_file():
            raise FileNotFoundError(f"GPS file not found: {gps}")
    output = args.output.expanduser().resolve()
    if output.exists() and (not output.is_dir() or any(output.iterdir())):
        raise ValueError(f"Output must be a new or empty directory: {output}")
    output.mkdir(parents=True, exist_ok=True)
    staged = output / "input"
    staged.mkdir()
    seconds = None if args.all else args.seconds
    if args.zip:
        pcap, report = prepare_local(source, staged, seconds=seconds)
        report["source_archive_checksum_note"] = (
            "SHA-256 recorded locally; no external expected checksum supplied"
        )
    else:
        pcap = staged / source.name
        with source.open("rb") as stream:
            report = trim_classic_pcap(stream, pcap, seconds)
        shutil.copyfile(metadata, pcap.with_suffix(".json"))
        if gps.is_file():
            shutil.copyfile(gps, pcap.with_suffix(".h5"))
        report.update(
            source_member=source.name,
            source_pcap_bytes=source.stat().st_size,
            policy="Local input only; original PCAP, sensor JSON and GPS file are never edited.",
        )
    report["capture_clock_note"] = (
        "PCAP timestamps may be relative; legacy *_unix names do not imply calendar time"
    )
    report["offline_input"] = True
    report["gps_clock_policy"] = args.gps_clock
    (staged / "preparation.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return pcap, output


def main(argv=None):
    """Coordinate preparation and mapping; keep all heavy SLAM work in run()."""
    # 1. Parse physical parameters and the desired input duration.
    p = parser()
    args = p.parse_args(argv)
    if args.max_range <= 1.0:
        p.error("--max-range must exceed the fixed minimum range of 1 metre")
    try:
        # 2. Make output/input/ with the selected classic-PCAP prefix and
        # matching sensor JSON/H5. The full unselected PCAP is not mapped.
        pcap, output = prepare_input(args)
    except (ValueError, OSError, zipfile.BadZipFile) as error:
        p.error(str(error))
    # Heavy scientific packages are deliberately imported after --help and
    # argument validation. The preparation path has no HTTP/GitHub calls.
    from dynamic.pilot import run

    # 3. Mapping, our QC and exports. gps_clock is forwarded to a post-hoc
    # comparison inside run(), not to mapping.SlamEngine.create()/update().
    run(
        pcap,
        output / "results",
        voxel=args.voxel,
        max_range=args.max_range,
        gps_clock=args.gps_clock,
        rhino=not args.no_rhino,
        generic_outputs=True,
    )
    # 4. GIF creation and meshing are independent postprocessing commands.
    print(f'Finished. Open {output / "results" / "README.txt"}', flush=True)
