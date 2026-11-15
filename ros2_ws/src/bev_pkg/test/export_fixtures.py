#!/usr/bin/env python3
"""Export training fixtures to the binary layout read by bev_fixture_check."""
import json
import struct
import sys
from pathlib import Path

TOKENS = {"inf": float("inf"), "-inf": float("-inf"), "nan": float("nan")}


def main() -> int:
    if len(sys.argv) != 3:
        print("usage: export_fixtures.py <fixture-json-dir> <output-dir>", file=sys.stderr)
        return 2
    source = Path(sys.argv[1])
    target = Path(sys.argv[2])
    target.mkdir(parents=True, exist_ok=True)
    count = 0
    for path in sorted(source.glob("*.json")):
        data = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(data, dict) or "config" not in data:
            continue
        cfg = data["config"]
        ranges = [TOKENS.get(v, v) if isinstance(v, str) else v for v in data["ranges"]]
        expected = [float(v) for v in data["expected_observation"]]
        header = struct.pack(
            "<11d",
            float(cfg["observation"]["size"]),
            float(cfg["observation"]["extent"]),
            float(cfg["sensor"]["range_min"]),
            float(cfg["sensor"]["range_max"]),
            float(cfg["sensor"]["x"]),
            float(cfg["sensor"]["y"]),
            float(cfg["sensor"]["yaw"]),
            float(cfg["robot"]["radius"]),
            float(cfg["observation"]["distance_clip"]),
            float(data["goal"][0]),
            float(data["goal"][1]),
        )
        blob = header
        blob += struct.pack("<i", len(ranges))
        blob += struct.pack(f"<{len(ranges)}f", *ranges)
        blob += struct.pack("<i", len(expected))
        blob += struct.pack(f"<{len(expected)}f", *expected)
        (target / f"{path.stem}.bin").write_bytes(blob)
        count += 1
    print(f"exported {count} fixtures to {target}")
    return 0 if count else 1


if __name__ == "__main__":
    raise SystemExit(main())
