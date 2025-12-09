# Geometry Fix Notes

This note summarizes how paths are currently constructed and why previous visualizations were offset from the actual road centerline. It also records hypotheses and invariants to prevent regressions.

## Current Path Construction

- Lombard Street (python/examples/demo_san_francisco.py)
  - Previously used a short list of hand‑traced GPS coordinates (LOMBARD_CROOKED_SECTION) in WGS84 lat/lon degrees.
  - The list approximated the crooked block but did not strictly follow the OSM way centerline, causing a consistent lateral offset when rendered on satellite tiles.

- Nürburgring Nordschleife (python/examples/benchmark_nurburgring.py)
  - Previously used ~70 hard‑coded track corner coordinates traced from imagery and then linearly interpolated in lat/lon degrees.
  - Linear interpolation in degrees distorts scale; the resulting shape/length deviated from the true OSM geometry.

- OSM Fetchers (python/apexvelocity/osm_fetcher.py)
  - Provides Overpass queries for named ways/bounding boxes and helpers for Lombard and the Nürburgring relation.
  - Returned raw lat/lon coordinates of OSM ways/relations, but the examples weren’t using these functions by default.

## Smoothing/Projection Stack

- CRSTransformer (geometry.py)
  - Chooses a UTM zone from a path’s centroid.
  - Uses pyproj Transformer with `always_xy=True` for consistent lon/lat ordering in projection.
  - Forward: (lat, lon) → (x_m, y_m). Inverse: (x_m, y_m) → (lat, lon). Round‑trip now validated in tests.

- smooth_path_geo (geometry.py)
  - Projects input lat/lon into meters (UTM).
  - Resamples via Catmull‑Rom / Linestring / Cubic Spline in meter space at a target spacing.
  - Converts back to lat/lon and carries interpolated metadata.

## Root Cause of Offsets

1) Primary: hand‑traced points did not follow the actual OSM way centerline; the visual tube was parallel but laterally shifted by a few meters.
2) Secondary (guarded): axis swaps or non‑invertible transforms could cause shifts; verified not the case via round‑trip tests. Interpolation occurs in metric space (meters), so no degree‑space distortion during smoothing.

## Geometry Invariants (New)

- We store and visualize in WGS84 lat/lon.
- We project to a local metric CRS for smoothing/resampling, then invert back.
- Round‑trip projection error must be < ~1 m for representative points.
- Smoothing must not laterally shift the path away from the original centerline: every smoothed point should be within a small offset (≈ 2–3 m) of the raw polyline.

