import pytest

from apexvelocity.geometry import (
    smooth_path_geo,
    validate_path_alignment,
    get_path_stats,
)


def _try_fetch_lombard_raw():
    try:
        from apexvelocity.osm_fetcher import (
            fetch_lombard_street_sf,
            order_coordinates_as_path,
            build_path_points,
        )

        coords = fetch_lombard_street_sf()
        if not coords:
            return None
        coords = order_coordinates_as_path(coords)
        pts = build_path_points(coords)
        return pts
    except Exception:
        return None


@pytest.mark.network
def test_lombard_alignment_close_to_osm_centerline():
    raw = _try_fetch_lombard_raw()
    if not raw or len(raw) < 3:
        pytest.skip(
            "OSM fetch unavailable or returned no data; skipping alignment test"
        )

    smoothed = smooth_path_geo(raw, target_spacing_m=2.5, mode="catmull_rom")

    # Verify smoothed path stays near the raw centerline
    assert validate_path_alignment(raw, smoothed, max_offset_m=2.0)

    # Basic sanity: lengths are in plausible range
    s = get_path_stats(smoothed)
    assert 150 <= s["total_distance_m"] <= 250
