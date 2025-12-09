import math
import pytest

from apexvelocity.geometry import CRSTransformer


@pytest.mark.parametrize(
    "lat,lon",
    [
        (37.802017, -122.418625),   # Lombard (Hyde St end)
        (37.802050, -122.420140),   # Lombard (Leavenworth end)
        (50.3353, 6.9418),          # Nürburgring start/finish approx
        (50.3210, 6.9815),          # Another track point
    ],
)
def test_projection_roundtrip(lat, lon):
    tfm = CRSTransformer(lat, lon)
    x, y = tfm.latlon_to_meters(lat, lon)
    lat2, lon2 = tfm.meters_to_latlon(x, y)
    # < 1 meter ≈ 1e-5 deg at mid‑latitudes
    assert abs(lat - lat2) < 1e-5
    assert abs(lon - lon2) < 1e-5

