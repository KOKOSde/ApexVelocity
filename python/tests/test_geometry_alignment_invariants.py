import math
from typing import List, Dict

import pytest

from apexvelocity.geometry import smooth_path_geo, validate_path_alignment


def _synthetic_curve() -> List[Dict]:
    # Quarter circle arc centered at (0,0) with radius ~100 m, in lat/lon degrees
    # around a mid‑latitude to have non‑degenerate cos(lat)
    lat0, lon0 = 37.8020, -122.4190
    Rm = 100.0  # meters
    deg_per_m_lat = 1.0 / 111320.0
    deg_per_m_lon = 1.0 / (111320.0 * math.cos(math.radians(lat0)))

    pts: List[Dict] = []
    for t in range(0, 91, 10):  # 0..90 degrees
        rad = math.radians(t)
        dx = Rm * math.cos(rad)
        dy = Rm * math.sin(rad)
        lat = lat0 + dy * deg_per_m_lat
        lon = lon0 + dx * deg_per_m_lon
        pts.append(
            {
                "lat": lat,
                "lon": lon,
                "distance_along_m": 0.0,
                "curvature": 0.0,
            }
        )
    # distance_along_m (rough)
    cum = 0.0
    for i in range(1, len(pts)):
        dlat = (pts[i]["lat"] - pts[i - 1]["lat"]) * 111320
        dlon = (
            (pts[i]["lon"] - pts[i - 1]["lon"]) * 111320 * math.cos(math.radians(lat0))
        )
        cum += math.hypot(dlat, dlon)
        pts[i]["distance_along_m"] = cum
    return pts


def test_smoothing_preserves_alignment_and_length():
    raw = _synthetic_curve()
    smoothed = smooth_path_geo(raw, target_spacing_m=5.0, mode="catmull_rom")
    # Alignment: all smoothed points within 3 m of original polyline
    assert validate_path_alignment(raw, smoothed, max_offset_m=3.0)

    # Length within ~5%
    L_raw = raw[-1]["distance_along_m"]
    L_smooth = smoothed[-1]["distance_along_m"]
    if L_raw > 0:
        assert abs(L_smooth - L_raw) / L_raw < 0.05
