"""Standard-library integrity check; optional model checks need installed ML libs."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[2]
parser=argparse.ArgumentParser()
parser.add_argument('--models',action='store_true')
parser.add_argument('--models-only',action='store_true')
args=parser.parse_args()
if not args.models_only:
    expected=json.loads((ROOT/'FILE_MANIFEST.json').read_text(encoding='utf-8'))
    for name,value in expected.items():
        path=ROOT/name
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest()!=value:
            raise SystemExit(f'FAIL: {name}')
    print(f'PASS: {len(expected)} packaged file hashes')
if args.models or args.models_only:
    bundles=sorted((ROOT/'models').glob('*/deployment/verify_bundle.py'))
    if len(bundles)<3: raise SystemExit('Missing expected model bundles')
    for script in bundles:
        subprocess.run([sys.executable,str(script)],cwd=script.parents[2],check=True)
    print(f'PASS: {len(bundles)} model bundles')
