#!/usr/bin/env python3
"""Compare the portable C++ filter with the frozen navrl v2 Python reference.

Set PYTHONPATH to training/envs before running. The executable is built from
safety_core_check.cpp; it does not need ROS or third-party C++ dependencies.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import subprocess

import numpy as np
from navrl.config import load_config
from navrl.control import filter_velocity


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--executable', required=True, type=Path)
    parser.add_argument('--config', required=True, type=Path)
    parser.add_argument('--report', required=True, type=Path)
    parser.add_argument('--cases', type=int, default=1200)
    parser.add_argument('--seed', type=int, default=20261115)
    args = parser.parse_args()
    base = load_config(args.config)
    rng = np.random.default_rng(args.seed)
    records, expected = [], []
    reasons = {}
    for index in range(args.cases):
        cfg = copy.deepcopy(base)
        sensor, robot, safety = cfg['sensor'], cfg['robot'], cfg['safety']
        count = int(rng.choice([72, 180, 360, 720]))
        sensor['range_min'] = float(rng.choice([0.02, 0.12, 0.24, 0.3]))
        sensor['range_max'] = float(rng.choice([3.5, 8.0, 10.0]))
        sensor['x'], sensor['y'] = rng.uniform(-0.12, 0.12, 2).tolist()
        sensor['yaw'] = float(rng.choice([0, np.pi / 2, -np.pi / 2, 0.15]))
        robot['radius'] = float(rng.uniform(0.15, 0.3))
        safety['margin'] = float(rng.uniform(0.01, 0.07))
        safety['reaction_time'] = float(rng.uniform(0.05, 0.4))
        safety['brake_deceleration'] = float(rng.uniform(0.3, 1.5))
        velocity = rng.uniform([-0.35, -1.2], [0.35, 1.2])
        if index % 5 == 0:
            velocity[1] = 0
        if index % 13 == 0:
            velocity[0] = 0
        if index % 37 == 0:
            velocity[:] = 0
        ranges = np.full(count, np.inf, dtype=np.float32)
        if index % 4 == 0:
            # Hits only exercise braking and footprint geometry without an
            # unknown ray masking the requested motion first.
            ranges[rng.choice(count, size=count // 4, replace=False)] = rng.uniform(
                sensor['range_min'], sensor['range_max'], count // 4)
        elif index % 4 == 1:
            # Invalid categories include finite above_max and -inf separately.
            values = [np.nan, -np.inf, sensor['range_min'] / 2,
                      sensor['range_max'] * 1.1]
            for _ in range(int(rng.integers(1, 8))):
                ranges[int(rng.integers(count))] = rng.choice(values)
        elif index % 4 == 2:
            # Float32 range boundaries and nearby hits test classification.
            rmin, rmax = np.float32(sensor['range_min']), np.float32(sensor['range_max'])
            values = [rmin, rmax, np.nextafter(rmin, np.float32(-np.inf)),
                      np.nextafter(rmax, np.float32(np.inf)), np.float32(0.25)]
            ranges[rng.choice(count, size=5, replace=False)] = values
        else:
            # A close hit may override a coverage reason, matching Python.
            ranges[int(rng.integers(count))] = rng.uniform(0.24, 0.7)
            if index % 3 == 0:
                ranges[int(rng.integers(count))] = np.nan
        if index % 101 == 0:
            ranges[:] = np.nan
        result, intervened, reason = filter_velocity(velocity, ranges, cfg)
        expected.append((result, intervened, reason))
        reasons[reason] = reasons.get(reason, 0) + 1
        header = [*velocity, sensor['range_min'], sensor['range_max'],
                  sensor['x'], sensor['y'], sensor['yaw'], robot['radius'],
                  safety['margin'], safety['reaction_time'],
                  safety['brake_deceleration'], 1, count]
        records.append(' '.join(map(str, [*header, *ranges.tolist()])))
    process = subprocess.run([str(args.executable), '--stdin'],
                             input='\n'.join(records) + '\n', text=True,
                             capture_output=True, check=True)
    lines = process.stdout.splitlines()
    mismatches = []
    if len(lines) != len(expected):
        raise AssertionError(f'C++ returned {len(lines)} records for {len(expected)} cases')
    for index, (line, (velocity, intervened, reason)) in enumerate(zip(lines, expected)):
        v, w, changed, cpp_reason = line.split('\t')
        if (not np.allclose([float(v), float(w)], velocity, rtol=0, atol=1e-12)
                or bool(int(changed)) != intervened or cpp_reason != reason):
            mismatches.append({'index': index, 'cpp': line,
                               'python': [*velocity.tolist(), bool(intervened), reason]})
    report = {'cases': len(expected), 'seed': args.seed, 'failures': len(mismatches),
              'reason_counts': reasons, 'mismatches': mismatches[:20],
              'config_sha256': hashlib.sha256(args.config.read_bytes()).hexdigest(),
              'reference': 'navrl.control.filter_velocity (bev_nav_v2)'}
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(report, indent=2))
    return int(bool(mismatches))


if __name__ == '__main__':
    raise SystemExit(main())
