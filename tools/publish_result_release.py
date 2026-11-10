#!/usr/bin/env python3
"""Copy verified result archives to a GitHub release using the GitHub CLI."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import zipfile

ROOT = Path(__file__).resolve().parent.parent

def verify(path, asset):
    if path.stat().st_size != asset['bytes']:
        raise RuntimeError(f'Unexpected byte count: {path.name}')
    with path.open('rb') as f:
        digest = hashlib.file_digest(f, 'sha256').hexdigest()
    if digest != asset['sha256']:
        raise RuntimeError(f'Checksum mismatch: {path.name}')
    with zipfile.ZipFile(path) as archive:
        bad = archive.testzip()
        if bad:
            raise RuntimeError(f'Corrupt ZIP member: {bad}')

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--asset-dir', type=Path, default=ROOT / 'work/release-assets')
    parser.add_argument('--verify-only', action='store_true', help='Check existing local files without network or release writes.')
    args = parser.parse_args()
    manifest = json.loads((ROOT / 'results/chandelier-v2/download_manifest.json').read_text())
    args.asset_dir.mkdir(parents=True, exist_ok=True)
    paths = []
    for asset in manifest['assets']:
        name = asset['name']
        if Path(name).name != name or not asset['url'].startswith('https://'):
            raise RuntimeError('Invalid download manifest entry')
        path = args.asset_dir / name
        if not args.verify_only:
            subprocess.run(['curl', '--fail', '--location', '--retry', '3', '--max-time', '180', '--output', str(path), asset['url']], check=True)
        verify(path, asset)
        paths.append(str(path))
        print(f'Verified {name}', flush=True)
    if args.verify_only:
        return
    repo = os.environ.get('GITHUB_REPOSITORY', 'u0063882/GreensprayAI')
    tag = manifest['tag']
    exists = subprocess.run(['gh', 'release', 'view', tag, '--repo', repo], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0
    if exists:
        subprocess.run(['gh', 'release', 'upload', tag, '--repo', repo, *paths, '--clobber'], check=True)
    else:
        subprocess.run(['gh', 'release', 'create', tag, '--repo', repo, '--target', os.environ.get('GITHUB_SHA', 'main'), '--title', manifest['title'], '--notes-file', str(ROOT / 'results/chandelier-v2/RELEASE_NOTES.md'), *paths], check=True)

if __name__ == '__main__':
    main()
