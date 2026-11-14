"""Compare VM/C++ observation JSON against Windows reference fixtures.

Usage: python scripts/check_fixture_outputs.py path/to/cpp_outputs
Each output must be <case_name>.json containing an observation float array.
"""
import argparse
import json
from pathlib import Path
import numpy as np
root=Path(__file__).resolve().parents[2]
p=argparse.ArgumentParser(); p.add_argument('outputs'); args=p.parse_args()
for item in json.loads((root/'training/tests/fixtures/manifest.json').read_text()):
    name=item['case']; expected=np.load(root/'training/tests/fixtures'/f'{name}.npz')['observation']
    actual=json.loads((Path(args.outputs)/f'{name}.json').read_text(encoding='utf-8-sig'))['observation']
    actual=np.asarray(actual,dtype=np.float32)
    if actual.shape!=expected.shape: raise AssertionError(f'{name}: wrong shape {actual.shape}')
    np.testing.assert_allclose(actual,expected,atol=1e-5,rtol=0,err_msg=name)
    print('PASS',name)
print('All supplied C++ outputs agree with the reference.')
