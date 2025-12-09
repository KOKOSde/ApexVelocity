#!/usr/bin/env python3
"""
ApexVelocity Command-Line Interface.

Analyze any city's road network with physics-based velocity profiling
and rich AV/robotaxi training features.

Usage:
    python -m apexvelocity.cli --city "Manhattan, New York City, USA" --vehicle tesla_model_3
    python -m apexvelocity.cli --city "Tokyo, Japan" --color-metric comfort --condition wet
    python -m apexvelocity.cli --city "San Francisco, CA" --output sf_analysis.html --style satellite
"""

import argparse
import sys
import os
import re
import math
from typing import Optional, List, Dict

# Banner
BANNER = """
╔═══════════════════════════════════════════════════════════════════════╗
║                                                                       ║
║     █████╗ ██████╗ ███████╗██╗  ██╗                                  ║
║    ██╔══██╗██╔══██╗██╔════╝╚██╗██╔╝                                  ║
║    ███████║██████╔╝█████╗   ╚███╔╝                                   ║
║    ██╔══██║██╔═══╝ ██╔══╝   ██╔██╗                                   ║
║    ██║  ██║██║     ███████╗██╔╝ ██╗                                  ║
║    ╚═╝  ╚═╝╚═╝     ╚══════╝╚═╝  ╚═╝  VELOCITY                       ║
║                                                                       ║
║    Physics-Based Graph Annotation Engine                              ║
║    Research-Grade AV Training Features                                ║
║                                                                       ║
╚═══════════════════════════════════════════════════════════════════════╝
"""


def slugify(text: str) -> str:
    """Convert text to a filename-safe slug."""
    text = text.lower()
    text = re.sub(r'[^\w\s-]', '', text)
    text = re.sub(r'[\s_-]+', '_', text)
    return text.strip('_')[:50]


def print_header(text: str):
    """Print a formatted header."""
    print(f"\n{'─' * 64}")
    print(f"  {text}")
    print(f"{'─' * 64}")


def print_stat(label: str, value: str, unit: str = "", color: str = None):
    """Print a formatted statistic."""
    print(f"  {label:<26} {value:>14} {unit}")


def print_summary(analysis, city: str, vehicle: str, condition: str):
    """Print the comprehensive analysis summary."""
    s = analysis.get_summary_dict()
    
    print()
    print("=" * 64)
    print("  === ApexVelocity Summary ===")
    print("=" * 64)
    print()
    print(f"  City:      {city}")
    print(f"  Vehicle:   {vehicle}, Condition: {condition}")
    print()
    
    # Core metrics
    print(f"  Distance:    {s['total_distance_km']:.2f} km")
    print(f"  Lap time:    {s['lap_time_s']:.1f} s ({s['lap_time_min']:.2f} min)")
    print(f"  Avg speed:   {s['avg_speed_kmh']:.1f} km/h")
    print(f"  Max speed:   {s['max_speed_kmh']:.1f} km/h")
    print()
    
    # Energy
    print(f"  Energy:      {s['total_energy_kwh']:.3f} kWh ({s['energy_kwh_per_km']*1000:.0f} Wh/km)")
    print(f"  Regen pot.:  {s['regen_fraction']*100:.0f}%")
    print()
    
    # Dynamics
    print(f"  Max lat G:   {s['max_lateral_g']:.2f} g (p95: {s['p95_lateral_g']:.2f} g)")
    print(f"  Max long G:  {s['max_longitudinal_g']:.2f} g")
    print(f"  p95 jerk:    {s['p95_longitudinal_jerk']:.1f} m/s³")
    print()
    
    # Friction & Comfort
    print(f"  Friction:    avg {s['avg_friction_usage']*100:.0f}%, max {s['max_friction_usage']*100:.0f}%")
    print(f"  Comfort viol.: {s['comfort_violations_per_km']:.2f} /km")
    print(f"  Uncomfortable: {s['pct_uncomfortable']:.1f}%")
    print()
    
    # Action distribution
    print(f"  Brake zones: {s['brake_fraction']*100:.1f}%")
    print(f"  Coast zones: {s['coast_fraction']*100:.1f}%")
    print(f"  Accel zones: {s['accelerate_fraction']*100:.1f}%")
    print()
    
    # Composite scores
    print(f"  Difficulty:  {s['difficulty_score']:.2f}")
    print(f"  Safety margin: {s['safety_margin_score']:.2f}")
    print()
    
    # Geometry
    print(f"  Turns/km:    {s['turns_per_km']:.1f}")
    print(f"  Max grade:   {s['max_grade_percent']:.1f}%")
    print(f"  Intersections: {s['intersection_count']}")
    print()


def _solve_with_core_or_fallback(
    path_points: List[Dict],
    vehicle_name: str,
    condition: str,
    *,
    vehicle_params: Optional[Dict] = None,
    config_overrides: Optional[Dict] = None,
):
    """
    Try to solve using the C++ core via apexvelocity.api.solve.
    If unavailable, fall back to the simplified Python solver with a warning.

    This function mutates path_points in-place to add speed and energy fields
    so that downstream analysis and visualization paths do not need to fork.

    Args:
        path_points: List of dicts produced by loader/extraction
        vehicle_name: Vehicle preset name for core solver (e.g., "tesla_model_3")
        condition: "dry" or "wet"
        vehicle_params: Optional dict of parameters for fallback solver
        config_overrides: Placeholder for future configuration overrides

    Returns:
        None (path_points modified in-place)
    """
    # Build geometry + surface arrays from path_points
    geometry = []
    surfaces = []
    for pt in path_points:
        geometry.append([
            pt.get('x_m', 0.0),
            pt.get('y_m', 0.0),
            pt.get('z_m', 0.0),
            pt.get('curvature', 0.0),
            pt.get('distance_along_m', 0.0),
        ])
        surfaces.append(pt.get('surface_type', 'asphalt'))

    # Attempt core path first
    try:
        from .api import solve as api_solve
        result = api_solve(
            path=[(p[0], p[1], p[2]) for p in geometry],
            surfaces=surfaces,
            vehicle=vehicle_name,
            condition=condition,
        )

        # Map results onto path_points
        vp = result.velocity_profile_mps or []
        ej = result.energy_joules or []
        for i, pt in enumerate(path_points):
            v = vp[i] if i < len(vp) else pt.get('speed_mps', 20.0)
            e_j = ej[i] if i < len(ej) else pt.get('energy_joules', 0.0)
            pt['speed_mps'] = v
            pt['v_profile'] = v
            pt['energy_joules'] = e_j
            pt['energy_kwh'] = e_j / 3.6e6
        return

    except (ImportError, RuntimeError, AttributeError) as e:
        print(
            "[WARN] apexvelocity core extension not found or failed; "
            "using simplified Python solver. Results are approximate and "
            "NOT suitable for benchmarking.",
            file=sys.stderr,
        )
        # Fall through to Python fallback

    # Fallback simplified Python solver (mirrors previous inline logic)
    mu = 0.9 if condition == 'dry' else 0.63  # align with analysis defaults approximately
    g = 9.81
    vehicle = vehicle_params or {
        'mass_kg': 1500,
        'drag_coeff': 0.30,
        'frontal_area': 2.2,
        'rolling_res': 0.015,
        'name': vehicle_name,
    }
    mass = vehicle.get('mass_kg', 1500)
    crr = vehicle.get('rolling_res', 0.012)
    cd = vehicle.get('drag_coeff', 0.23)
    area = vehicle.get('frontal_area', 2.2)
    rho = 1.225

    total_energy = 0.0
    for i, pt in enumerate(path_points):
        curv = abs(pt.get('curvature', 0))
        if curv > 1e-6:
            v_max = math.sqrt(mu * g / curv)
        else:
            v_max = 35.0

        v = min(v_max, 35.0)
        v = max(v, 5.0)

        pt['speed_mps'] = v
        pt['v_profile'] = v

        if i > 0:
            prev = path_points[i - 1]
            dist = abs(pt.get('distance_along_m', 0) - prev.get('distance_along_m', 0))
            if dist < 0.1:
                dist = 10.0

            dz = pt.get('z_m', 0) - prev.get('z_m', 0)
            v_avg = (v + prev['speed_mps']) / 2

            f_aero = 0.5 * rho * cd * area * v_avg * v_avg
            f_roll = crr * mass * g
            grade = dz / dist if dist > 0.1 else 0
            f_grade = mass * g * grade

            segment_energy = (f_aero + f_roll + f_grade) * dist
            total_energy += max(segment_energy, 0)

        pt['energy_joules'] = total_energy
        pt['energy_kwh'] = total_energy / 3.6e6

    return

def main():
    parser = argparse.ArgumentParser(
        prog="apexvelocity",
        description="ApexVelocity - Physics-Based Road Network Analysis for AV Training",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s --city "San Francisco, CA"
  %(prog)s --city "Tokyo, Japan" --vehicle sports_car --condition wet
  %(prog)s --city "London, UK" --color-metric comfort --style satellite
  %(prog)s --city "Manhattan, NYC" --color-metric difficulty --output manhattan.html

Available Vehicles:
  tesla_model_3, compact_car, sports_car, suv, default

Color Features (--color-by):
  speed          - Color by velocity (red=slow, green=fast)
  energy         - Color by energy intensity (green=efficient, red=costly)
  lateral_g      - Color by lateral G-force (blue=low, red=high)
  longitudinal_g - Color by longitudinal acceleration
  friction       - Color by friction usage (green=safe, red=limit)
  comfort        - Color by comfort score (green=comfortable, red=uncomfortable)
  difficulty     - Color by local difficulty (blue=easy, red=challenging)
  safety_margin  - Color by safety margin (red=danger, green=safe)
  regen          - Color by regeneration potential
  action         - Color by recommended action (brake, coast, accelerate)

Map Styles (no API key needed):
  osm              - OpenStreetMap tiles (default)
  dark             - Dark CARTO tiles  
  light            - Light CARTO tiles

Map Styles (require MAPBOX_API_KEY):
  satellite        - Satellite imagery with streets
  satellite-plain  - Pure satellite imagery
  streets          - Mapbox street map
  outdoors         - Terrain/outdoor map
        """
    )
    
    # City-level analysis (legacy, optional if route args are provided)
    parser.add_argument(
        "--city", "-c",
        help="City/place name to analyze (e.g., 'Manhattan, NYC', 'Tokyo, Japan')"
    )
    
    parser.add_argument(
        "--vehicle", "-v",
        default="tesla_model_3",
        choices=["tesla_model_3", "compact_car", "sports_car", "suv", "default"],
        help="Vehicle preset to use (default: tesla_model_3)"
    )
    
    parser.add_argument(
        "--condition",
        default="dry",
        choices=["dry", "wet"],
        help="Road condition (default: dry)"
    )
    
    parser.add_argument(
        "--color-by", "--color-metric", "--mode", "-m",
        dest="color_by",
        default="speed",
        choices=["speed", "energy", "lateral_g", "longitudinal_g", "gforce", 
                 "friction", "comfort", "difficulty", "safety_margin", "regen", "action"],
        help="Feature to color paths by (default: speed)"
    )
    
    parser.add_argument(
        "--style", "-s",
        default="osm",
        choices=["osm", "dark", "light", "satellite", "satellite-plain", "streets", "outdoors"],
        help="Map style (default: osm - no API key needed; satellite requires MAPBOX_API_KEY)"
    )
    
    parser.add_argument(
        "--output", "-o",
        help="Output HTML file (default: <city_slug>_analysis.html)"
    )
    
    parser.add_argument(
        "--no-viz",
        action="store_true",
        help="Skip visualization, only show stats"
    )
    
    parser.add_argument(
        "--quiet", "-q",
        action="store_true",
        help="Minimal output"
    )

    # Route-based arguments (primary path)
    parser.add_argument(
        "--from",
        dest="addr_from",
        help="Start address or place name (e.g., 'Lombard St & Hyde St, San Francisco, CA')",
    )
    parser.add_argument(
        "--to",
        dest="addr_to",
        help="End address or place name (e.g., 'Lombard St & Leavenworth St, San Francisco, CA')",
    )
    parser.add_argument(
        "--from-lat",
        type=float,
        dest="from_lat",
        help="Start latitude (used with --from-lon)",
    )
    parser.add_argument(
        "--from-lon",
        type=float,
        dest="from_lon",
        help="Start longitude (used with --from-lat)",
    )
    parser.add_argument(
        "--to-lat",
        type=float,
        dest="to_lat",
        help="End latitude (used with --to-lon)",
    )
    parser.add_argument(
        "--to-lon",
        type=float,
        dest="to_lon",
        help="End longitude (used with --to-lat)",
    )
    parser.add_argument(
        "--place",
        dest="place_hint",
        help="Place hint for OSM graph loading (e.g., 'San Francisco, California, USA')",
    )
    
    args = parser.parse_args()
    
    if not args.quiet:
        print(BANNER)
    
    # Check Mapbox token only if satellite style requested
    mapbox_token = os.environ.get('MAPBOX_API_KEY') or os.environ.get('MAPBOX_ACCESS_TOKEN')
    if args.style in ['satellite', 'satellite-plain', 'streets', 'outdoors'] and not mapbox_token:
        print(f"⚠️  Note: --style {args.style} requires MAPBOX_API_KEY.")
        print("   Will fall back to 'osm' style instead.")
        print("   For satellite, set: export MAPBOX_API_KEY='your_token'")
        print("   Get a free token at: https://account.mapbox.com/access-tokens/")
        print()
    
    # Import analysis module
    try:
        from .analysis import analyze_profile
    except ImportError:
        from apexvelocity.analysis import analyze_profile

    # Core solver (via api.solve)
    try:
        from .api import solve as api_solve
        HAS_CORE_SOLVER = True
    except ImportError:
        HAS_CORE_SOLVER = False
    
    # Vehicle presets
    VEHICLES = {
        'tesla_model_3': {'mass_kg': 1847, 'drag_coeff': 0.23, 'frontal_area': 2.22, 'rolling_res': 0.012},
        'compact_car': {'mass_kg': 1200, 'drag_coeff': 0.30, 'frontal_area': 2.0, 'rolling_res': 0.015},
        'sports_car': {'mass_kg': 1500, 'drag_coeff': 0.32, 'frontal_area': 2.0, 'rolling_res': 0.010},
        'suv': {'mass_kg': 2200, 'drag_coeff': 0.38, 'frontal_area': 2.8, 'rolling_res': 0.018},
        'default': {'mass_kg': 1500, 'drag_coeff': 0.30, 'frontal_area': 2.2, 'rolling_res': 0.015},
    }
    
    vehicle = VEHICLES.get(args.vehicle, VEHICLES['default'])
    vehicle['name'] = args.vehicle.replace('_', ' ').title()
    
    friction_multiplier = 1.0 if args.condition == "dry" else 0.7
    
    # Decide between legacy city mode and new route mode
    use_route_mode = any(
        [
            args.addr_from,
            args.addr_to,
            args.from_lat is not None and args.from_lon is not None,
            args.to_lat is not None and args.to_lon is not None,
        ]
    )

    path_points: List[Dict] = []

    if use_route_mode:
        # Route-based analysis
        try:
            from .routing import RouteRequest, build_route
        except ImportError:
            print("  Error: routing module not available; falling back to --city mode.")
            use_route_mode = False

    if use_route_mode:
        # Resolve start/end coordinates
        def _geocode_address(addr: str, place: Optional[str]) -> Optional[tuple]:
            # Prefer Mapbox geocoding if key is present; otherwise osmnx.geocode
            token = os.environ.get("MAPBOX_API_KEY") or os.environ.get("MAPBOX_ACCESS_TOKEN")
            if token:
                try:
                    import urllib.request
                    import json
                    q = addr.replace(" ", "%20")
                    url = f"https://api.mapbox.com/geocoding/v5/mapbox.places/{q}.json?access_token={token}"
                    with urllib.request.urlopen(url, timeout=10) as resp:
                        data = json.loads(resp.read().decode("utf-8"))
                    feats = data.get("features") or []
                    if feats:
                        lon, lat = feats[0]["center"]
                        return (lat, lon)
                except Exception:
                    pass
            # Fallback: osmnx.geocode
            try:
                import osmnx as ox
                loc = ox.geocode(addr if not place else f"{addr}, {place}")
                return (loc[0], loc[1])
            except Exception:
                return None

        place_hint = args.place_hint or args.city

        # Start coordinates
        if args.from_lat is not None and args.from_lon is not None:
            start = (args.from_lat, args.from_lon)
        elif args.addr_from:
            s = _geocode_address(args.addr_from, place_hint)
            if not s:
                print(f"  Error: could not geocode --from address: {args.addr_from}")
                sys.exit(1)
            start = s
        else:
            print("  Error: route mode requires --from/--from-lat/--from-lon")
            sys.exit(1)

        # End coordinates
        if args.to_lat is not None and args.to_lon is not None:
            end = (args.to_lat, args.to_lon)
        elif args.addr_to:
            e = _geocode_address(args.addr_to, place_hint)
            if not e:
                print(f"  Error: could not geocode --to address: {args.addr_to}")
                sys.exit(1)
            end = e
        else:
            print("  Error: route mode requires --to/--to-lat/--to-lon")
            sys.exit(1)

        print_header("Building Route")
        print(f"  From:  {start}")
        print(f"  To:    {end}")
        if place_hint:
            print(f"  Place: {place_hint}")

        from .routing import RouteRequest, build_route  # type: ignore

        request = RouteRequest(
            start=start,
            end=end,
            place_hint=place_hint,
            vehicle=args.vehicle,
            condition=args.condition,
            target_spacing_m=5.0,
        )

        try:
            path_points = build_route(request)
        except Exception as e:
            print(f"\n  Error building route: {e}")
            sys.exit(1)

        print(f"  ✓ Route points: {len(path_points)}")
        if path_points:
            total_dist = path_points[-1].get("distance_along_m", 0.0)
            print(f"  ✓ Route distance: {total_dist/1000:.2f} km")

    else:
        # Legacy city-based analysis (kept for backwards compatibility)
        if not args.city:
            print("  Error: either --city or route args (--from/--to/lat/lon) must be provided")
            sys.exit(1)

        # Import loader only when needed
        try:
            from .loader import OSMLoader  # type: ignore
            HAS_LOADER = True
        except ImportError:
            HAS_LOADER = False

        print_header(f"Loading: {args.city}")

        if not HAS_LOADER:
            print("  Error: osmnx/loader not available, cannot run --city mode")
            sys.exit(1)

        try:
            loader = OSMLoader()

            print(f"  Fetching road network from OpenStreetMap...")
            G = loader.graph_from_place(args.city, network_type="drive")

            import random
            nodes = list(G.nodes())

            if len(nodes) < 10:
                print("  Error: Not enough road data found")
                sys.exit(1)

            print(f"  Found {len(nodes)} road intersections")

            # Find a good route
            random.seed(42)
            sample_nodes = random.sample(nodes, min(50, len(nodes)))

            origin = sample_nodes[0]
            dest = sample_nodes[0]
            max_dist = 0

            origin_data = G.nodes[origin]
            for n in sample_nodes:
                node_data = G.nodes[n]
                dist = ((node_data['y'] - origin_data['y'])**2 +
                        (node_data['x'] - origin_data['x'])**2) ** 0.5
                if dist > max_dist:
                    max_dist = dist
                    dest = n

            print(f"  Finding route through city...")

            route = loader.get_shortest_path(
                G,
                (G.nodes[origin]['y'], G.nodes[origin]['x']),
                (G.nodes[dest]['y'], G.nodes[dest]['x'])
            )

            path = loader.extract_path(G, route)

            # Convert to dict format with lat/lon and additional OSM attributes
            for i, pt in enumerate(path.points):
                path_points.append({
                    'lat': pt.lat,
                    'lon': pt.lon,
                    'x_m': pt.x_m,
                    'y_m': pt.y_m,
                    'z_m': pt.z_m,
                    'elevation_m': pt.z_m,
                    'curvature': pt.curvature,
                    'distance_along_m': pt.distance_along_m,
                    'surface_type': pt.surface_type,
                    'road_type': getattr(pt, 'road_type', None),
                    'speed_limit_kmh': getattr(pt, 'speed_limit_kmh', None),
                    'lane_count': getattr(pt, 'lane_count', None),
                    'is_intersection': getattr(pt, 'is_intersection', i == 0 or i == len(path.points) - 1),
                    'speed_mps': 0.0,
                    'v_profile': 0.0,
                    'energy_kwh': 0.0,
                })

            print(f"  ✓ Extracted {len(path_points)} path points")
            print(f"  ✓ Route distance: {path.total_distance_m/1000:.2f} km")

        except Exception as e:
            print(f"\n  Error loading city data: {e}")
            print("\n  Troubleshooting:")
            print("    1. Check city name spelling")
            print("    2. Try a more specific name (e.g., 'Manhattan, New York City, USA')")
            print("    3. Ensure you have internet connection")
            print("    4. Install required packages: pip install osmnx")
            sys.exit(1)
    
    # Solve using core solver if available; fallback only if core is missing
    print_header(f"Solving: {vehicle['name']} ({args.condition})")
    if HAS_CORE_SOLVER:
        try:
            # Use PathPointData so curvature and distance are preserved
            from .api import PathPointData  # type: ignore

            path_data = []
            for pt in path_points:
                path_data.append(
                    PathPointData(
                        x=pt.get("x_m", 0.0),
                        y=pt.get("y_m", 0.0),
                        z=pt.get("z_m", 0.0),
                        curvature=pt.get("curvature", 0.0),
                        distance_along=pt.get("distance_along_m", 0.0),
                        surface_type=pt.get("surface_type", "asphalt"),
                    )
                )

            result = api_solve(
                path=path_data,
                vehicle=args.vehicle,
                condition=args.condition,
            )

            vp = result.velocity_profile_mps or []
            ej = result.energy_joules or []
            for i, pt in enumerate(path_points):
                v = vp[i] if i < len(vp) else pt.get('speed_mps', 0.0)
                e_j = ej[i] if i < len(ej) else pt.get('energy_joules', 0.0)
                pt['speed_mps'] = v
                pt['v_profile'] = v
                pt['energy_joules'] = e_j
                pt['energy_kwh'] = e_j / 3.6e6
        except Exception as e:
            print(
                f"[WARN] Core solver failed ({e}); falling back to simplified Python solver.",
                file=sys.stderr,
            )
            _solve_with_core_or_fallback(
                path_points,
                vehicle_name=args.vehicle,
                condition=args.condition,
                vehicle_params=vehicle,
            )
    else:
        _solve_with_core_or_fallback(
            path_points,
            vehicle_name=args.vehicle,
            condition=args.condition,
            vehicle_params=vehicle,
        )

    print(f"  ✓ Solver complete")
    
    # Run comprehensive analysis
    print_header("Analysis")
    
    analysis = analyze_profile(
        path_points,
        condition=args.condition,
        friction_multiplier=friction_multiplier,
        vehicle_mass_kg=vehicle['mass_kg']
    )
    
    # Print comprehensive summary
    print_summary(analysis, args.city, vehicle['name'], args.condition)
    
    # Generate visualization
    if not args.no_viz:
        # Choose a sensible default filename if not provided
        if args.output:
            output_file = args.output
        else:
            base_label = args.city or args.place_hint or (args.addr_from or "route")
            output_file = f"{slugify(base_label)}_analysis.html"
        
        print_header(f"Visualization")
        print(f"  Style:       {args.style}")
        print(f"  Color by:    {args.color_by}")
        print(f"  Output:      {output_file}")
        print()
        
        # Prefer the fully interactive Mapbox/Deck.gl visualization, but keep
        # the original pydeck-based renderer as a fallback for older setups.
        use_interactive = False
        visualize_profile = None  # type: ignore
        visualize_profile_interactive = None  # type: ignore

        try:
            from .viz import visualize_profile_interactive as _viz_interactive  # type: ignore
            visualize_profile_interactive = _viz_interactive
            use_interactive = True
        except ImportError:
            use_interactive = False

        if not use_interactive:
            # Fallback: original pydeck visualization
            try:
                from .viz import visualize_profile as _viz  # type: ignore
            except ImportError:
                try:
                    from apexvelocity.viz import visualize_profile as _viz  # type: ignore
                except ImportError as e:
                    print(f"  ⚠ Visualization error: {e}")
                    print("    Install pydeck: pip install pydeck")
                    return 1
            visualize_profile = _viz  # type: ignore

        try:
            if use_interactive and visualize_profile_interactive:
                # Map CLI style names onto the interactive styles
                style_map = {
                    "osm": "streets",
                    "dark": "dark",
                    "light": "streets",
                    "satellite": "satellite",
                    "satellite-plain": "satellite",
                    "streets": "streets",
                    "outdoors": "streets",
                }
                interactive_style = style_map.get(args.style, "satellite")

                visualize_profile_interactive(
                    path_points,
                    title=f"{(args.city or args.place_hint or 'Route')} - {vehicle['name']}",
                    output_html=output_file,
                    initial_color_by=args.color_by,
                    style=interactive_style,
                    mapbox_token=mapbox_token,
                    condition=args.condition,
                    friction_multiplier=friction_multiplier,
                    path_width_m=8.0,
                )
            elif visualize_profile:
                visualize_profile(
                    path_points,
                    color_by=args.color_by,
                    title=f"{args.city} - {vehicle['name']}",
                    output_html=output_file,
                    style=args.style,
                    condition=args.condition,
                    friction_multiplier=friction_multiplier,
                )
            print(f"\n  🌐 Open {output_file} in your browser to view the map")
        except EnvironmentError as e:
            print(f"  ⚠ Visualization skipped: {e}")
        except Exception as e:
            print(f"  ⚠ Visualization error: {e}")
    
    print()
    print("=" * 64)
    print("  Analysis complete!")
    print("=" * 64)
    print()
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
