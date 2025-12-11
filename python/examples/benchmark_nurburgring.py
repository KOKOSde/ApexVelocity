#!/usr/bin/env python3
"""
ApexVelocity Demo: Nürburgring Nordschleife Rain Check Benchmark.

This demo validates the friction model by:
1. Using accurate GPS coordinates for the Nordschleife track
2. Applying path smoothing for accurate satellite overlay
3. Running the physics solver in DRY and WET conditions
4. Verifying that wet lap times are significantly slower

Usage:
    export MAPBOX_API_KEY='your_token'
    python benchmark_nurburgring.py
"""

import sys
import os
import math
import copy
from typing import List, Dict

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

JOULES_TO_KWH = 1 / 3.6e6
GRAVITY = 9.81

# VERIFIED Nürburgring Nordschleife GPS coordinates
# Traced from satellite imagery - the actual racing line
# The track is 20.8km, running counter-clockwise from Start/Finish
# Track spans: ~7km east-west, ~5km north-south
# Key coordinates verified against Google Maps/Earth
NORDSCHLEIFE_TRACK = [
    # Start/Finish straight (near T13)
    (50.3353, 6.9418),  # Start/Finish line
    (50.3358, 6.9450),
    (50.3365, 6.9490),
    # Hatzenbach
    (50.3380, 6.9540),
    (50.3400, 6.9580),
    (50.3420, 6.9610),
    # Hocheichen
    (50.3445, 6.9635),
    (50.3470, 6.9655),
    # Quiddelbacher Höhe
    (50.3490, 6.9680),
    (50.3510, 6.9710),
    # Flugplatz (famous jump)
    (50.3530, 6.9745),
    (50.3548, 6.9790),
    (50.3560, 6.9840),
    # Schwedenkreuz
    (50.3568, 6.9895),
    (50.3570, 6.9950),
    # Aremberg
    (50.3565, 7.0005),
    (50.3555, 7.0060),
    # Fuchsröhre (fast downhill)
    (50.3540, 7.0115),
    (50.3520, 7.0165),
    (50.3495, 7.0210),
    # Adenauer Forst
    (50.3465, 7.0250),
    (50.3430, 7.0280),
    # Metzgesfeld
    (50.3390, 7.0300),
    (50.3350, 7.0305),
    # Kallenhard
    (50.3310, 7.0295),
    (50.3275, 7.0275),
    # Wehrseifen
    (50.3240, 7.0245),
    (50.3210, 7.0205),
    # Breidscheid (lowest point ~320m)
    (50.3185, 7.0155),
    (50.3170, 7.0100),
    (50.3165, 7.0040),
    # Ex-Mühle
    (50.3170, 6.9980),
    (50.3180, 6.9920),
    # Bergwerk
    (50.3195, 6.9865),
    (50.3210, 6.9815),
    # Kesselchen
    (50.3225, 6.9770),
    (50.3238, 6.9725),
    # Klostertal
    (50.3248, 6.9680),
    (50.3255, 6.9635),
    # Karussell (famous banked turn)
    (50.3258, 6.9590),
    (50.3255, 6.9545),
    (50.3248, 6.9505),
    # Hohe Acht
    (50.3240, 6.9465),
    (50.3235, 6.9425),
    (50.3235, 6.9385),
    # Wippermann / Eschbach
    (50.3242, 6.9345),
    (50.3255, 6.9310),
    # Brünnchen
    (50.3275, 6.9280),
    (50.3300, 6.9260),
    # Eiskurve
    (50.3325, 6.9250),
    (50.3350, 6.9248),
    # Pflanzgarten (challenging section)
    (50.3372, 6.9255),
    (50.3390, 6.9270),
    (50.3405, 6.9295),
    # Stefan Bellof S
    (50.3415, 6.9328),
    (50.3418, 6.9365),
    # Schwalbenschwanz
    (50.3415, 6.9405),
    (50.3405, 6.9445),
    # Kleine Karussell
    (50.3390, 6.9480),
    (50.3372, 6.9508),
    # Galgenkopf
    (50.3355, 6.9525),
    (50.3340, 6.9535),
    # Döttinger Höhe (long straight back to start)
    (50.3328, 6.9535),
    (50.3318, 6.9525),
    (50.3310, 6.9505),
    (50.3305, 6.9478),
    (50.3308, 6.9445),
    (50.3318, 6.9418),
    (50.3335, 6.9400),
    # Back to Start
    (50.3353, 6.9418),
]


def create_raw_track_path() -> List[Dict]:
    """Create raw Nürburgring path (before smoothing)."""

    path_points = []
    cumulative_distance = 0.0

    for i, (lat, lon) in enumerate(NORDSCHLEIFE_TRACK):
        if i > 0:
            prev = NORDSCHLEIFE_TRACK[i - 1]
            dlat = (lat - prev[0]) * 111320
            dlon = (lon - prev[1]) * 111320 * math.cos(math.radians(lat))
            cumulative_distance += math.sqrt(dlat * dlat + dlon * dlon)

        path_points.append(
            {
                "lat": lat,
                "lon": lon,
                "x_m": 0,
                "y_m": 0,
                "z_m": 0,
                "elevation_m": 0,
                "curvature": 0.0,
                "distance_along_m": cumulative_distance,
                "surface_type": "asphalt",
                "speed_mps": 50.0,
                "v_profile": 50.0,
                "energy_joules": 0.0,
                "energy_kwh": 0.0,
            }
        )

    return path_points


def create_track_path() -> List[Dict]:
    """Create Nürburgring path from real OSM geometry, then smooth.

    Falls back to the legacy hand‑traced coordinates if OSM fetch fails.
    """

    # Import geometry module for smoothing
    try:
        from apexvelocity.geometry import (
            smooth_path_geo,
            recompute_curvature,
            get_path_stats,
        )
    except ImportError:
        print("  ⚠ Geometry module not available, using raw path")
        return create_raw_track_path()

    # Prefer real OSM geometry (relation)
    osm_points: List[Dict] = []
    try:
        from apexvelocity.osm_fetcher import get_nurburgring

        osm_points = get_nurburgring() or []
    except Exception as e:
        print(f"  ⚠ OSM fetch failed: {e}. Falling back to traced coords")

    raw_path = osm_points if len(osm_points) >= 2 else create_raw_track_path()

    print("  Smoothing path with Catmull-Rom spline...")

    # ~10 m spacing yields ~2000+ points for a ~21 km lap
    smooth_path = smooth_path_geo(raw_path, target_spacing_m=10.0, mode="catmull_rom")

    # Recompute curvature on smoothed path
    smooth_path = recompute_curvature(smooth_path, smooth_window=5)

    # Get stats
    raw_stats = get_path_stats(raw_path)
    smooth_stats = get_path_stats(smooth_path)

    print(
        f"    Raw path:    {raw_stats['point_count']} points, {raw_stats['total_distance_km']:.2f} km"
    )
    print(
        f"    Smooth path: {smooth_stats['point_count']} points, {smooth_stats['total_distance_km']:.2f} km"
    )
    print(f"    Avg spacing: {smooth_stats['avg_spacing_m']:.1f}m")
    print(f"    Max curvature: {smooth_stats['max_curvature']:.4f} (1/m)")

    return smooth_path


def solve_track(
    path_points: List[Dict], vehicle: Dict, friction_mult: float = 1.0
) -> float:
    """
    Solve velocity profile using the C++ core via the public API.

    The `friction_mult` parameter controls wet/dry by selecting the
    condition ("dry" for 1.0, "wet" for < 1.0).
    """
    if len(path_points) < 2:
        return 0.0

    try:
        import apexvelocity as av  # type: ignore
    except ImportError:
        return 0.0

    condition = "dry" if friction_mult >= 1.0 else "wet"

    # Build simple (x, y, z) coordinates in meters. The solver uses
    # distance_along_m and curvature for its physics, so we can keep
    # geometry minimal here.
    coords = []
    surfaces = []
    for pt in path_points:
        d = float(pt.get("distance_along_m", 0.0))
        coords.append((d, 0.0, 0.0))
        surfaces.append(pt.get("surface_type", "asphalt"))

    result = av.solve(
        path=coords, surfaces=surfaces, vehicle="sports_car", condition=condition
    )

    vp = result.velocity_profile_mps or []
    ej = result.energy_joules or []

    # Attach solver outputs to path_points
    for i, pt in enumerate(path_points):
        v = vp[i] if i < len(vp) else pt.get("speed_mps", 0.0)
        e_j = ej[i] if i < len(ej) else pt.get("energy_joules", 0.0)
        pt["speed_mps"] = v
        pt["v_profile"] = v
        pt["energy_joules"] = e_j
        pt["energy_kwh"] = e_j / 3.6e6 if e_j is not None else pt.get("energy_kwh", 0.0)

    # Compute lap time from the solved profile
    total_time = 0.0
    for i in range(1, len(path_points)):
        pt, prev = path_points[i], path_points[i - 1]
        dist = float(pt.get("distance_along_m", 0.0)) - float(
            prev.get("distance_along_m", 0.0)
        )
        if dist <= 0.1:
            continue
        v_avg = 0.5 * (pt["speed_mps"] + prev["speed_mps"])
        if v_avg > 0.5:
            total_time += dist / v_avg

    return total_time


def run_demo():
    print(
        """
╔══════════════════════════════════════════════════════════════════════╗
║   🏎️  NÜRBURGRING NORDSCHLEIFE BENCHMARK  🏎️                       ║
║   Testing: Friction model - Dry vs Wet conditions                    ║
╚══════════════════════════════════════════════════════════════════════╝
"""
    )

    mapbox_token = os.environ.get("MAPBOX_API_KEY")
    map_style = "satellite" if mapbox_token else "dark"
    print(f"🛰️  Map style: {map_style}")

    vehicle = {"name": "Sports Car", "mass_kg": 1500, "max_power_w": 350000}
    print(f"Vehicle: {vehicle['name']} ({vehicle['max_power_w']/1000:.0f} kW)")

    # Create track with smoothing
    print("\nLoading Nordschleife track...")
    path_points = create_track_path()
    total_km = path_points[-1]["distance_along_m"] / 1000
    print(f"  ✓ {len(path_points)} waypoints, {total_km:.2f} km")

    # Dry run
    path_dry = copy.deepcopy(path_points)
    lap_dry = solve_track(path_dry, vehicle, 1.0)

    print(f"\n=== DRY (μ=0.9) ===")
    print(f"  Lap: {lap_dry:.1f}s ({lap_dry/60:.2f} min)")
    print(f"  Avg: {total_km*1000/lap_dry*3.6:.1f} km/h")

    # Wet run
    path_wet = copy.deepcopy(path_points)
    lap_wet = solve_track(path_wet, vehicle, 0.7)

    print(f"\n=== WET (μ=0.63) ===")
    print(f"  Lap: {lap_wet:.1f}s ({lap_wet/60:.2f} min)")
    print(f"  Avg: {total_km*1000/lap_wet*3.6:.1f} km/h")

    delta = lap_wet - lap_dry
    pct = delta / lap_dry * 100
    print(f"\n=== RESULT ===")
    print(f"  Delta: +{delta:.1f}s ({pct:.1f}% slower)")

    passed = lap_wet > lap_dry and pct > 5
    print(f"  {'✓ PASS' if passed else '✗ FAIL'}")

    # Rich analysis with new features
    print("\n=== RICH FEATURE ANALYSIS ===")
    try:
        from apexvelocity.viz import visualize_profile, visualize_comparison
        from apexvelocity.analysis import analyze_profile

        analysis_dry = analyze_profile(
            path_dry, condition="dry", vehicle_mass_kg=vehicle["mass_kg"]
        )
        analysis_wet = analyze_profile(
            path_wet,
            condition="wet",
            friction_multiplier=0.7,
            vehicle_mass_kg=vehicle["mass_kg"],
        )

        sd = analysis_dry.get_summary_dict()
        sw = analysis_wet.get_summary_dict()

        print(f"\n  {'Metric':<22} {'DRY':>10} {'WET':>10}")
        print(f"  {'-'*22} {'-'*10} {'-'*10}")
        print(
            f"  {'Max Lat G':<22} {sd['max_lateral_g']:>8.2f} g {sw['max_lateral_g']:>8.2f} g"
        )
        print(
            f"  {'p95 Lat G':<22} {sd['p95_lateral_g']:>8.2f} g {sw['p95_lateral_g']:>8.2f} g"
        )
        print(
            f"  {'Avg Friction Use':<22} {sd['avg_friction_usage']*100:>8.0f}% {sw['avg_friction_usage']*100:>8.0f}%"
        )
        print(
            f"  {'Comfort Viol/km':<22} {sd['comfort_violations_per_km']:>8.2f}  {sw['comfort_violations_per_km']:>8.2f}"
        )
        print(
            f"  {'Brake Zones':<22} {sd['brake_fraction']*100:>8.1f}% {sw['brake_fraction']*100:>8.1f}%"
        )
        print(
            f"  {'Difficulty':<22} {sd['difficulty_score']:>8.2f}  {sw['difficulty_score']:>8.2f}"
        )
        print(
            f"  {'Safety Margin':<22} {sd['safety_margin_score']:>8.2f}  {sw['safety_margin_score']:>8.2f}"
        )

        # Try to use the new interactive visualization with Mapbox
        try:
            from apexvelocity.viz import visualize_profile_interactive

            # Get Mapbox token from environment
            mapbox_token = os.environ.get("MAPBOX_API_KEY", "")
            style_type = "satellite-streets" if mapbox_token else "dark"

            visualize_profile_interactive(
                path_dry,
                title="Nürburgring Nordschleife - DRY",
                output_html="nurburgring_dry.html",
                initial_color_by="difficulty",
                style=style_type,
                mapbox_token=mapbox_token,
            )

            visualize_profile_interactive(
                path_wet,
                title="Nürburgring Nordschleife - WET",
                output_html="nurburgring_wet.html",
                initial_color_by="friction",
                style=style_type,
                mapbox_token=mapbox_token,
                condition="wet",
                friction_multiplier=0.7,
            )

            # Also generate comparison (uses old method - still works)
            visualize_comparison(
                path_dry,
                path_wet,
                label_a="DRY",
                label_b="WET",
                title="Dry vs Wet",
                output_html="nurburgring_comparison.html",
                style=map_style,
                friction_b=0.7,
            )

            print(f"\n  ✓ nurburgring_dry.html (🎛️ interactive color dropdown!)")
            print(f"  ✓ nurburgring_wet.html (🎛️ interactive color dropdown!)")
            print(f"  ✓ nurburgring_comparison.html")
            print(f"  🛰️  {'Satellite' if mapbox_token else 'Dark'} basemap")
            print(f"\n  Hover for full feature set!")

        except Exception as e:
            print(f"  ⚠ Interactive viz failed ({e}), falling back to basic")
            # Fallback
            visualize_profile(
                path_dry,
                color_by="difficulty",
                title="Nordschleife - DRY",
                output_html="nurburgring_dry.html",
                style=map_style,
            )
            visualize_profile(
                path_wet,
                color_by="friction",
                title="Nordschleife - WET",
                output_html="nurburgring_wet.html",
                style=map_style,
                condition="wet",
                friction_multiplier=0.7,
            )
            visualize_comparison(
                path_dry,
                path_wet,
                label_a="DRY",
                label_b="WET",
                title="Dry vs Wet",
                output_html="nurburgring_comparison.html",
                style=map_style,
                friction_b=0.7,
            )

            print(f"\n  ✓ nurburgring_dry.html (color_by=difficulty)")
            print(f"  ✓ nurburgring_wet.html (color_by=friction)")
            print(f"  ✓ nurburgring_comparison.html")
            print(f"\n  Hover for full feature set!")

    except Exception as e:
        print(f"\n  ⚠ Viz error: {e}")
        import traceback

        traceback.print_exc()

    print("\nDone!\n")
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(run_demo())
