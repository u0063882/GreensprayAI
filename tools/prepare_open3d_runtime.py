#!/usr/bin/env python3
"""Materialize and verify the small EGL loader for the Ubuntu x86-64 CI job."""
import argparse,base64,ctypes,hashlib,platform
from pathlib import Path
SHA256='f4b60a4618fbf760f395f64dfd53320446ba1011151a80e411742b3232b194e6'

def prepare(output):
    if platform.system()!='Linux' or platform.machine() not in ['x86_64','amd64']:
        raise RuntimeError('Bundled EGL loader supports Linux x86-64 only')
    libc,version=platform.libc_ver()
    if libc!='glibc' or tuple(map(int,version.split('.')[:2]))<(2,38):
        raise RuntimeError('Bundled EGL loader requires glibc 2.38 or later')
    payload=Path(__file__).resolve().parent/'runtime/libEGL.so.1.b64'
    content=base64.b64decode(b''.join(payload.read_bytes().split()),validate=True)
    if hashlib.sha256(content).hexdigest()!=SHA256:raise RuntimeError('EGL payload checksum mismatch')
    output=Path(output).resolve();output.mkdir(parents=True,exist_ok=True);target=output/'libEGL.so.1';target.write_bytes(content)
    ctypes.CDLL(str(target))
    return output

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output-dir',type=Path,required=True)
    print(prepare(parser.parse_args().output_dir))
