#!/usr/bin/env python3
"""Reproduce the current 18-recording chandelier dataset from the repository root."""
import argparse
import os
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
    print(f'Finished. Results: {ROOT / "output"}', flush=True)

if __name__ == '__main__':
    main()
