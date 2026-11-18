#!/usr/bin/env python3
"""Refine an existing moving SDK result experimentally; baseline files stay unchanged."""
import argparse
from pathlib import Path


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--results", type=Path, required=True)
    p.add_argument("--gps", type=Path, help="GPS H5; otherwise find it in the run input/ folder")
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--gps-clock", choices=["shared-start", "absolute"], default="shared-start")
    p.add_argument(
        "--gps-sigma",
        type=float,
        default=0.15,
        help="Nominal horizontal GPS uncertainty, metres; default .15",
    )
    p.add_argument(
        "--lever-sigma",
        type=float,
        default=1.0,
        help="Uncertain body-frame antenna offset prior, metres; default 1",
    )
    p.add_argument("--no-rhino", action="store_true")
    a = p.parse_args()
    baseline = a.results.resolve()
    if (baseline / "results").is_dir():
        baseline = baseline / "results"
    gps = a.gps
    if gps is None:
        files = list((baseline.parent / "input").glob("*.h5"))
        if len(files) != 1:
            p.error("Supply --gps with the matching acquisition H5")
        gps = files[0]
    if a.output.exists() and any(a.output.iterdir()):
        p.error("--output must be new or empty")
    if not (a.gps_sigma > 0 and a.lever_sigma > 0):
        p.error("Uncertainty scales must be positive")
    from dynamic.gps_refinement import run_experiment

    run_experiment(baseline, gps, a.output, a.gps_clock, a.gps_sigma, a.lever_sigma, not a.no_rhino)
    print("Review", a.output / "comparison.json", "before using the candidate.")


if __name__ == "__main__":
    main()
