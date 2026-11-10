#!/usr/bin/env python3
"""Build the 18-recording seed graph, then anchor east and refine west poses."""
import argparse
import os
import shutil
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parent
STAGES = [
    'decode.py', 'register.py', 'trunk_align.py', 'refine_ground.py',
    'check_ground_refinement.py', 'finish.py', 'export.py', 'quality.py',
    'export_contributions.py',
]

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument('--check-inputs-only', action='store_true')
    modes.add_argument('--export-only', action='store_true', help='Use existing work/decoded and work/registered/final_poses.npy.')
    args = parser.parse_args()
    (ROOT / 'work').mkdir(exist_ok=True)
    (ROOT / 'output').mkdir(exist_ok=True)
    if args.export_only:
        if not (ROOT / 'work/registered/final_poses.npy').is_file():
            parser.error('Missing saved poses: work/registered/final_poses.npy')
        if len(list((ROOT / 'work/decoded').glob('*.npz'))) != 18:
            parser.error('Export requires all 18 cached, decoded recordings.')
        stages = STAGES[5:]
    else:
        for side in ['sideA', 'sideB']:
            files = sorted((ROOT / 'work/data' / side).glob('*.pcap'))
            if len(files) != 9:
                parser.error(f'Expected nine PCAP recordings in work/data/{side}; found {len(files)}.')
            for file in files:
                for suffix in ['.json', '.h5']:
                    if not file.with_suffix(suffix).is_file():
                        parser.error(f'Missing matching input: {file.with_suffix(suffix)}')
        print('Input check passed: 18 PCAP/JSON/H5 triples.', flush=True)
        if args.check_inputs_only:
            return
        # These stages cache decoded scans and preserve a baseline for ground checks.
        # Refuse a full rerun over an existing result so a previous capture cannot
        # accidentally supply cached points or baseline poses to a different run.
        if (ROOT / 'work/registered/final_poses.npy').exists():
            parser.error('A registration already exists. Use --export-only, or start with a fresh work directory for a full rerun.')
        stages = STAGES
    env = os.environ.copy()
    env.setdefault('MPLCONFIGDIR', str(ROOT / 'work/mpl'))
    for stage in stages:
        print(f'Running {stage}', flush=True)
        subprocess.run([sys.executable, str(ROOT / 'processing' / stage)], cwd=ROOT, env=env, check=True)
    anchored = ROOT / 'output/anchored_registration'
    subprocess.run([
        sys.executable, str(ROOT / 'processing/register_anchored.py'),
        '--scan-dir', str(ROOT / 'output/registered_scans'),
        '--transforms', str(ROOT / 'output/scan_transforms.json'),
        '--output-dir', str(anchored), '--export',
    ], cwd=ROOT, env=env, check=True)
    subprocess.run([
        sys.executable, str(ROOT / 'tools/validate_registered_result.py'),
        '--baseline-scan-dir', str(ROOT / 'output/registered_scans'),
        '--baseline-transforms', str(ROOT / 'output/scan_transforms.json'),
        '--result-dir', str(anchored),
    ], cwd=ROOT, env=env, check=True)
    for name in ['orchard_row_registered.ply', 'scan_transforms.json', 'orchard_row_result.zip',
                 'separate_registered_scans.zip', 'combined_by_recording.zip',
                 'registration_quality.json', 'anchored_registration_quality.json', 'anchored_registration_comparison.png',
                 'anchored_trunk_sections.png', 'orchard_row_preview.png']:
        shutil.copy2(anchored / name, ROOT / 'output' / name)
    shutil.copytree(anchored / 'registered_scans', ROOT / 'output/registered_scans', dirs_exist_ok=True)
    # Keep legacy seed caches intact, so export-only repeats one bounded refinement.
    # The authoritative current transforms are output/scan_transforms.json.
    import numpy as np
    seed = np.load(ROOT / 'work/registered/final_poses.npy')
    correction = np.load(anchored / 'corrections.npy')
    np.save(ROOT / 'work/registered/anchored_poses.npy', correction @ seed)
    for command in [
        [sys.executable, str(ROOT / 'processing/check_cross_row.py'), '--scan-dir', str(ROOT / 'output/registered_scans')],
        [sys.executable, str(ROOT / 'processing/plot_trunk_slices.py'), '--scan-dir', str(ROOT / 'output/registered_scans'),
         '--candidate-report', str(ROOT / 'presets/orchard_stem_candidates.json')],
    ]:
        subprocess.run(command, cwd=ROOT, env=env, check=True)
    print(f'Finished. Results: {ROOT / "output"}', flush=True)

if __name__ == '__main__':
    main()
