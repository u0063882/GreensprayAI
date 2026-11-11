#!/usr/bin/env python3
"""Register a named capture in its own workspace, independently of meshing."""
import argparse
from pathlib import Path
from captures.pipeline import run_capture

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,required=True)
    parser.add_argument('--workspace',type=Path,required=True,help='Contains work/data/sideA and work/data/sideB; writes its own work caches and final outputs.')
    args=parser.parse_args();run_capture(args.workspace,args.config)
