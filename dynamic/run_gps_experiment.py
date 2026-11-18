"""GitHub runner for an isolated GPS experiment on the published 15 s baseline."""

from pathlib import Path
import argparse, hashlib, json, os, shutil, subprocess, zipfile
from dynamic.gps_refinement import load_baseline, load_gps, audit

REPO = "u0063882/GreensprayAI"
BRANCH = "experiment/gps-constrained-slam"
TAG = "moving-scan-inputs"
ROOT = Path(__file__).resolve().parent.parent
EXPECTED = {
    "Dynamic.zip": "41a702f3a3818e555cbfdd2e8cad00d66eea94e2b70327216af33607a5d88bfa",
    "pilot-15s-results.zip": "d90af28cdfa89a1d3e0a19826c26e4ed04cfe8c98757bf0cd6f51f3971c9a970",
}


def command(*args):
    return subprocess.check_output(args, text=True, cwd=ROOT).strip()


def release(tag):
    return json.loads(command("gh", "api", f"repos/{REPO}/releases/tags/{tag}"))


def fingerprint(assets):
    return {a["name"]: {k: a.get(k) for k in ["id", "size", "digest"]} for a in assets}


def extract_member(archive, name, target):
    matches = [p for p in archive.infolist() if Path(p.filename).name == name]
    if len(matches) != 1:
        raise ValueError(f"Expected exactly one {name}")
    with archive.open(matches[0]) as src, target.open("wb") as dst:
        shutil.copyfileobj(src, dst)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--audit-only", action="store_true")
    args = p.parse_args()
    work = Path("/tmp/greenspray-gps-experiment")
    work.mkdir(exist_ok=True)
    source_before, current_before = release(TAG), release("current")
    for name, expected in EXPECTED.items():
        asset = next(a for a in source_before["assets"] if a["name"] == name)
        if asset["digest"] != "sha256:" + expected:
            raise RuntimeError("Original input/baseline digest changed")
        command(
            "gh",
            "release",
            "download",
            TAG,
            "--repo",
            REPO,
            "--pattern",
            name,
            "--dir",
            str(work),
            "--clobber",
        )
        with (work / name).open("rb") as f:
            if hashlib.file_digest(f, "sha256").hexdigest() != expected:
                raise RuntimeError("Downloaded checksum differs")
    baseline = work / "baseline"
    baseline.mkdir(exist_ok=True)
    with zipfile.ZipFile(work / "pilot-15s-results.zip") as z:
        for name in ["pilot_report.json", "per_column_poses.npz", "trajectory.csv"]:
            extract_member(z, name, baseline / name)
        if not args.audit_only:
            for name in ["pilot_registered.ply", "pilot15s.osf"]:
                extract_member(z, name, baseline / name)
    with zipfile.ZipFile(work / "Dynamic.zip") as z:
        h5 = [p for p in z.infolist() if p.filename.endswith(".h5")]
        if len(h5) != 1:
            raise ValueError("One GPS H5 expected")
        extract_member(z, Path(h5[0].filename).name, work / "gps.h5")
    data = load_baseline(baseline)
    gps = load_gps(work / "gps.h5", data)
    report = audit(data, gps)
    if not args.audit_only:
        from dynamic.gps_refinement import run_experiment

        report = run_experiment(baseline, work / "gps.h5", work / "output")
    if fingerprint(current_before["assets"]) != fingerprint(release("current")["assets"]):
        raise RuntimeError("Current assets changed during experiment")
    after = release(TAG)
    for name in EXPECTED:
        a = [x for x in source_before["assets"] if x["name"] == name]
        b = [x for x in after["assets"] if x["name"] == name]
        if fingerprint(a) != fingerprint(b):
            raise RuntimeError("Input/baseline changed")
    report["publication"] = dict(
        source_code_commit=os.environ.get("GITHUB_SHA"),
        main_and_current_untouched=True,
        original_inputs_untouched=True,
    )
    if not args.audit_only:
        (work / "output/comparison.json").write_text(
            json.dumps(report, indent=2, allow_nan=False) + "\n"
        )
        archive = work / "gps-15s-experiment.zip"
        with zipfile.ZipFile(archive, "w", zipfile.ZIP_DEFLATED, compresslevel=6) as z:
            for member in sorted((work / "output").rglob("*")):
                if member.is_file():
                    z.write(
                        member, Path("gps_15s_experiment") / member.relative_to(work / "output")
                    )
        shutil.copyfile(work / "output/comparison.json", work / "gps-15s-comparison.json")
        shutil.copyfile(work / "output/comparison.png", work / "gps-15s-comparison.png")
        command(
            "gh",
            "release",
            "upload",
            TAG,
            "--repo",
            REPO,
            str(archive),
            str(work / "gps-15s-comparison.json"),
            str(work / "gps-15s-comparison.png"),
            "--clobber",
        )
    result = ROOT / "results/gps_experiment"
    result.mkdir(parents=True, exist_ok=True)
    name = "audit.json" if args.audit_only else "comparison.json"
    (result / name).write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    if not args.audit_only:
        shutil.copyfile(work / "output/comparison.png", result / "comparison.png")
    command("git", "config", "user.name", "github-actions[bot]")
    command("git", "config", "user.email", "41898282+github-actions[bot]@users.noreply.github.com")
    command("git", "add", str(result.relative_to(ROOT)))
    command("git", "commit", "-m", "Record isolated GPS experiment metrics [skip ci]")
    command("git", "push", "origin", f"HEAD:refs/heads/{BRANCH}")
    print(json.dumps(report, indent=2), flush=True)


if __name__ == "__main__":
    main()
