import pytest

from apexvelocity.geometry import smooth_path_geo, get_path_stats


def _try_fetch_ring_raw():
    try:
        from apexvelocity.osm_fetcher import (
            fetch_nurburgring_nordschleife,
            order_coordinates_as_path,
            build_path_points,
        )

        coords = fetch_nurburgring_nordschleife()
        if not coords:
            return None
        coords = order_coordinates_as_path(coords)
        pts = build_path_points(coords)
        return pts
    except Exception:
        return None


@pytest.mark.network
def test_nurburgring_length_plausible():
    raw = _try_fetch_ring_raw()
    if not raw or len(raw) < 10:
        pytest.skip("OSM fetch unavailable or returned no data; skipping ring test")

    # Smooth ~10 m spacing
    smoothed = smooth_path_geo(raw, target_spacing_m=10.0, mode="catmull_rom")
    s = get_path_stats(smoothed)

    # Known Nordschleife full lap length ≈ 20.83 km
    assert 19.8 <= s["total_distance_km"] <= 21.9
