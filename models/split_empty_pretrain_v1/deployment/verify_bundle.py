"""Run on Windows or Ubuntu after extracting a model bundle."""
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
from stable_baselines3 import PPO

root=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root))
manifest=json.loads((root/'sha256.json').read_text(encoding='utf-8'))
for name,expected in manifest.items():
    path=root/name
    if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest()!=expected:
        raise RuntimeError(f'Hash mismatch: {name}')
model=PPO.load(root/'policy.zip',device='cpu')
data=np.load(root/'inference_reference.npz')
actual,_=model.predict(data['observations'],deterministic=True)
np.testing.assert_allclose(actual,data['actions'],rtol=1e-5,atol=1e-5)
print('PASS: manifest hashes and deterministic inference reference')
try:
    import rclpy
    print('PASS: rclpy and SB3 import in the same interpreter')
except ImportError:
    print('INFO: rclpy unavailable; ROS integration has not been verified here')
