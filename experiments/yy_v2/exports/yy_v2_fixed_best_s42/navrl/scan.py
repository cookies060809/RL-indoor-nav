"""Single source of truth for planar LaserScan return semantics.

A finite reading inside the configured range is a hit. Positive infinity means
the sensor saw no return along that ray (known free up to range_max). Every
other value (NaN, -inf, below range_min, above range_max) is unknown: it must
never be interpreted as free space.
"""
import numpy as np

UNKNOWN = 0
HIT = 1
FREE = 2


def classify_scan(ranges, sensor):
    values = np.asarray(ranges, dtype=np.float32)
    finite = np.isfinite(values)
    hit = finite & (values >= sensor['range_min']) & (values <= sensor['range_max'])
    free = np.isposinf(values)
    unknown = ~(hit | free)
    return values, hit, free, unknown


def scan_angles(beams, sensor):
    return -np.pi + np.arange(beams) * 2 * np.pi / beams + sensor['yaw']


def normalize_for_raster(ranges, sensor):
    """Map readings to the rasterizer contract: hit value or +inf; unknown -> NaN."""
    values, hit, free, unknown = classify_scan(ranges, sensor)
    output = np.full(values.shape, np.nan, dtype=np.float32)
    output[free] = np.inf
    output[hit] = values[hit]
    return output


def scan_summary(ranges, sensor):
    values, hit, free, unknown = classify_scan(ranges, sensor)
    return {
        'total': int(values.size),
        'hit': int(np.count_nonzero(hit)),
        'free': int(np.count_nonzero(free)),
        'unknown': int(np.count_nonzero(unknown)),
    }
