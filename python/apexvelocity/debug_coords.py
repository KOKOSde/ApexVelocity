"""
ApexVelocity Coordinate Debugging Module.

This module provides functions to diagnose coordinate and geometry issues
in the path visualization pipeline.
"""

import sys
from typing import List, Dict, Tuple, Optional


def print_path_debug(
    path_points: List[Dict],
    label: str = "PATH",
    n_samples: int = 5,
    file=sys.stdout
):
    """
    Print diagnostic information for a path.
    
    Args:
        path_points: List of path point dicts
        label: Label for this debug output
        n_samples: Number of sample points to print
        file: Output file (default: stdout)
    """
    if not path_points:
        print(f"\n[DEBUG {label}] Empty path!", file=file)
        return
    
    print(f"\n{'='*70}", file=file)
    print(f"[DEBUG {label}] Path Analysis", file=file)
    print(f"{'='*70}", file=file)
    
    n = len(path_points)
    print(f"\n  Total points: {n}", file=file)
    
    # Check for lat/lon vs x/y
    has_lat_lon = 'lat' in path_points[0] and 'lon' in path_points[0]
    has_xy = 'x_m' in path_points[0] or 'x' in path_points[0]
    
    print(f"  Has lat/lon: {has_lat_lon}", file=file)
    print(f"  Has x_m/y_m: {has_xy}", file=file)
    
    # Extract coordinate fields
    if has_lat_lon:
        lats = [p.get('lat', 0) for p in path_points]
        lons = [p.get('lon', 0) for p in path_points]
        
        print(f"\n  Coordinate System: WGS84 (lat/lon degrees)", file=file)
        print(f"  Bounding Box:", file=file)
        print(f"    Latitude:  {min(lats):.6f} to {max(lats):.6f} (range: {max(lats)-min(lats):.6f}°)", file=file)
        print(f"    Longitude: {min(lons):.6f} to {max(lons):.6f} (range: {max(lons)-min(lons):.6f}°)", file=file)
        
        # Approximate size in meters
        avg_lat = sum(lats) / len(lats)
        import math
        lat_m = (max(lats) - min(lats)) * 111320  # degrees to meters
        lon_m = (max(lons) - min(lons)) * 111320 * math.cos(math.radians(avg_lat))
        print(f"    Approx size: {lat_m:.0f}m (N-S) × {lon_m:.0f}m (E-W)", file=file)
    
    # Distance along path
    if 'distance_along_m' in path_points[-1]:
        total_dist = path_points[-1]['distance_along_m']
        print(f"\n  Total distance: {total_dist:.1f} m ({total_dist/1000:.2f} km)", file=file)
    
    # Sample points
    print(f"\n  First {min(n_samples, n)} points:", file=file)
    print(f"  {'idx':<5} {'lat':<12} {'lon':<14} {'dist_m':<10} {'curv':<10} {'surface'}", file=file)
    print(f"  {'-'*5} {'-'*12} {'-'*14} {'-'*10} {'-'*10} {'-'*10}", file=file)
    
    for i in range(min(n_samples, n)):
        p = path_points[i]
        lat = p.get('lat', p.get('y', 0))
        lon = p.get('lon', p.get('x', 0))
        dist = p.get('distance_along_m', 0)
        curv = p.get('curvature', 0)
        surf = p.get('surface_type', '?')
        print(f"  {i:<5} {lat:<12.6f} {lon:<14.6f} {dist:<10.1f} {curv:<10.6f} {surf}", file=file)
    
    if n > n_samples:
        print(f"  ... ({n - n_samples} more points)", file=file)
        print(f"\n  Last point:", file=file)
        p = path_points[-1]
        lat = p.get('lat', p.get('y', 0))
        lon = p.get('lon', p.get('x', 0))
        dist = p.get('distance_along_m', 0)
        curv = p.get('curvature', 0)
        surf = p.get('surface_type', '?')
        print(f"  {n-1:<5} {lat:<12.6f} {lon:<14.6f} {dist:<10.1f} {curv:<10.6f} {surf}", file=file)
    
    # Check for potential issues
    print(f"\n  Potential Issues:", file=file)
    issues = []
    
    # 1. Very few points
    if n < 50:
        issues.append(f"  ⚠️  Low point count ({n}) - may cause jagged/chunky appearance")
    
    # 2. Check spacing
    if n > 1 and has_lat_lon:
        import math
        spacings = []
        for i in range(1, n):
            dlat = (path_points[i].get('lat', 0) - path_points[i-1].get('lat', 0)) * 111320
            dlon = (path_points[i].get('lon', 0) - path_points[i-1].get('lon', 0)) * 111320 * math.cos(math.radians(avg_lat))
            spacings.append(math.sqrt(dlat**2 + dlon**2))
        
        min_spacing = min(spacings) if spacings else 0
        max_spacing = max(spacings) if spacings else 0
        avg_spacing = sum(spacings) / len(spacings) if spacings else 0
        
        print(f"  Point spacing: min={min_spacing:.1f}m, max={max_spacing:.1f}m, avg={avg_spacing:.1f}m", file=file)
        
        if max_spacing > 50:
            issues.append(f"  ⚠️  Large gaps between points (max {max_spacing:.0f}m) - interpolation may help")
    
    # 3. Check coordinate order for PyDeck
    if has_lat_lon:
        # PyDeck expects [lon, lat] but we store as {'lat': ..., 'lon': ...}
        # viz.py should convert correctly
        print(f"  Note: PyDeck PathLayer expects [lon, lat, elev] order", file=file)
    
    if not issues:
        print(f"  ✓ No obvious issues detected", file=file)
    else:
        for issue in issues:
            print(issue, file=file)
    
    print(f"\n{'='*70}\n", file=file)


def print_viz_segment_debug(
    path_data: List[Dict],
    label: str = "VIZ",
    n_samples: int = 3,
    file=sys.stdout
):
    """
    Print diagnostic information for visualization path segments.
    
    Args:
        path_data: List of path segment dicts (output from _build_path_data)
        label: Label for this debug output
        n_samples: Number of sample segments to print
        file: Output file (default: stdout)
    """
    if not path_data:
        print(f"\n[DEBUG {label}] No path segments!", file=file)
        return
    
    print(f"\n{'='*70}", file=file)
    print(f"[DEBUG {label}] Visualization Segments", file=file)
    print(f"{'='*70}", file=file)
    
    n = len(path_data)
    print(f"\n  Total segments: {n}", file=file)
    
    # Sample segment
    if n > 0:
        seg = path_data[0]
        print(f"\n  Sample segment [0] 'path' field:", file=file)
        if 'path' in seg:
            path_coords = seg['path']
            print(f"    {path_coords}", file=file)
            print(f"    Format: [[lon1, lat1, elev1], [lon2, lat2, elev2]]", file=file)
            
            # Check order
            if len(path_coords) >= 2:
                p1, p2 = path_coords[0], path_coords[1]
                lon1, lat1 = p1[0], p1[1]
                
                print(f"    First point: lon={lon1}, lat={lat1}", file=file)
                
                # San Francisco check
                if 37 < lat1 < 38 and -123 < lon1 < -122:
                    print(f"    ✓ Looks like San Francisco coordinates (correct order)", file=file)
                # Nürburgring check
                elif 50 < lat1 < 51 and 6 < lon1 < 8:
                    print(f"    ✓ Looks like Nürburgring coordinates (correct order)", file=file)
                # Check for swapped coords
                elif 37 < lon1 < 38 and -123 < lat1 < -122:
                    print(f"    ⚠️  SWAPPED! lat/lon appear to be in wrong order!", file=file)
                elif 50 < lon1 < 51 and 6 < lat1 < 8:
                    print(f"    ⚠️  SWAPPED! lat/lon appear to be in wrong order!", file=file)
    
    print(f"\n{'='*70}\n", file=file)


def check_nurburgring_scale():
    """
    Check if Nürburgring coordinates match real-world scale.
    
    Real Nordschleife:
    - Total length: ~20.832 km
    - Spans roughly 7km east-west, 5km north-south
    - Start/Finish: approximately 50.3320°N, 6.9428°E
    """
    print("\n" + "="*70)
    print("NÜRBURGRING NORDSCHLEIFE SCALE CHECK")
    print("="*70)
    
    # Real-world reference
    print("\nReal Nordschleife (from official sources):")
    print("  Total length: 20.832 km")
    print("  Start/Finish: ~50.3320°N, 6.9428°E")
    print("  Dimensions: ~7km (E-W) × ~5km (N-S)")
    
    # Our hardcoded coordinates (from benchmark_nurburgring.py)
    NORDSCHLEIFE_TRACK = [
        (50.3353, 6.9418),  # Start
        (50.3570, 7.0305),  # Approximate extremes from track
        (50.3165, 6.9248),
    ]
    
    # Compute our bounding box
    lats = [50.3353, 50.3358, 50.3365, 50.3380, 50.3400, 50.3420, 50.3445, 50.3470, 
            50.3490, 50.3510, 50.3530, 50.3548, 50.3560, 50.3568, 50.3570, 50.3565,
            50.3555, 50.3540, 50.3520, 50.3495, 50.3465, 50.3430, 50.3390, 50.3350,
            50.3310, 50.3275, 50.3240, 50.3210, 50.3185, 50.3170, 50.3165, 50.3170,
            50.3180, 50.3195, 50.3210, 50.3225, 50.3238, 50.3248, 50.3255, 50.3258,
            50.3255, 50.3248, 50.3240, 50.3235, 50.3235, 50.3242, 50.3255, 50.3275,
            50.3300, 50.3325, 50.3350, 50.3372, 50.3390, 50.3405, 50.3415, 50.3418,
            50.3415, 50.3405, 50.3390, 50.3372, 50.3355, 50.3340, 50.3328, 50.3318,
            50.3310, 50.3305, 50.3308, 50.3318, 50.3335, 50.3353]
    
    lons = [6.9418, 6.9450, 6.9490, 6.9540, 6.9580, 6.9610, 6.9635, 6.9655, 6.9680,
            6.9710, 6.9745, 6.9790, 6.9840, 6.9895, 6.9950, 7.0005, 7.0060, 7.0115,
            7.0165, 7.0210, 7.0250, 7.0280, 7.0300, 7.0305, 7.0295, 7.0275, 7.0245,
            7.0205, 7.0155, 7.0100, 7.0040, 6.9980, 6.9920, 6.9865, 6.9815, 6.9770,
            6.9725, 6.9680, 6.9635, 6.9590, 6.9545, 6.9505, 6.9465, 6.9425, 6.9385,
            6.9345, 6.9310, 6.9280, 6.9260, 6.9250, 6.9248, 6.9255, 6.9270, 6.9295,
            6.9328, 6.9365, 6.9405, 6.9445, 6.9480, 6.9508, 6.9525, 6.9535, 6.9535,
            6.9525, 6.9505, 6.9478, 6.9445, 6.9418, 6.9400, 6.9418]
    
    import math
    avg_lat = sum(lats) / len(lats)
    
    lat_range = max(lats) - min(lats)
    lon_range = max(lons) - min(lons)
    
    ns_km = lat_range * 111.32
    ew_km = lon_range * 111.32 * math.cos(math.radians(avg_lat))
    
    print(f"\nOur hardcoded coordinates:")
    print(f"  Start point: 50.3353°N, 6.9418°E")
    print(f"  Lat range: {min(lats):.4f} to {max(lats):.4f} (Δ = {lat_range:.4f}°)")
    print(f"  Lon range: {min(lons):.4f} to {max(lons):.4f} (Δ = {lon_range:.4f}°)")
    print(f"  Dimensions: {ew_km:.1f}km (E-W) × {ns_km:.1f}km (N-S)")
    
    # Compare
    print(f"\nComparison:")
    if 6.5 < ew_km < 8.5 and 4 < ns_km < 6:
        print(f"  ✓ Track dimensions match real Nordschleife approximately")
    else:
        print(f"  ⚠️  Track dimensions don't match!")
        print(f"      Expected: ~7km × ~5km, Got: {ew_km:.1f}km × {ns_km:.1f}km")
    
    print("\n" + "="*70)


def check_lombard_coordinates():
    """
    Check if Lombard Street coordinates match real-world.
    
    Real Lombard Street crooked section:
    - From Hyde St to Leavenworth St
    - About 180m long, 8 hairpin turns
    - Coordinates: ~37.8020°N, -122.4186°E to -122.4201°E
    """
    print("\n" + "="*70)
    print("LOMBARD STREET COORDINATE CHECK")
    print("="*70)
    
    # Real-world reference (from Google Maps)
    print("\nReal Lombard Street crooked section:")
    print("  Location: Russian Hill, San Francisco")
    print("  East end (Hyde St): ~37.8020°N, -122.4186°E")
    print("  West end (Leavenworth): ~37.8020°N, -122.4201°E")
    print("  Length: ~180m, 8 hairpin turns")
    
    # Our hardcoded coordinates
    LOMBARD_CROOKED_SECTION = [
        {'lat': 37.802017, 'lon': -122.418625},
        {'lat': 37.802035, 'lon': -122.418720},
        {'lat': 37.802115, 'lon': -122.418815},
        {'lat': 37.802028, 'lon': -122.418895},
        {'lat': 37.802108, 'lon': -122.418980},
        {'lat': 37.802025, 'lon': -122.419065},
        {'lat': 37.802105, 'lon': -122.419150},
        {'lat': 37.802022, 'lon': -122.419235},
        {'lat': 37.802102, 'lon': -122.419320},
        {'lat': 37.802019, 'lon': -122.419405},
        {'lat': 37.802099, 'lon': -122.419490},
        {'lat': 37.802016, 'lon': -122.419575},
        {'lat': 37.802096, 'lon': -122.419660},
        {'lat': 37.802013, 'lon': -122.419745},
        {'lat': 37.802093, 'lon': -122.419830},
        {'lat': 37.802010, 'lon': -122.419915},
        {'lat': 37.802090, 'lon': -122.420000},
        {'lat': 37.802050, 'lon': -122.420140},
    ]
    
    import math
    
    lats = [p['lat'] for p in LOMBARD_CROOKED_SECTION]
    lons = [p['lon'] for p in LOMBARD_CROOKED_SECTION]
    
    lat_range = max(lats) - min(lats)
    lon_range = max(lons) - min(lons)
    
    avg_lat = sum(lats) / len(lats)
    ns_m = lat_range * 111320
    ew_m = lon_range * 111320 * math.cos(math.radians(avg_lat))
    
    print(f"\nOur hardcoded coordinates:")
    print(f"  Points: {len(LOMBARD_CROOKED_SECTION)}")
    print(f"  Lat range: {min(lats):.6f} to {max(lats):.6f}")
    print(f"  Lon range: {min(lons):.6f} to {max(lons):.6f}")
    print(f"  Dimensions: {ew_m:.0f}m (E-W) × {ns_m:.0f}m (N-S)")
    
    # Compute total path length
    total_dist = 0
    for i in range(1, len(LOMBARD_CROOKED_SECTION)):
        dlat = (lats[i] - lats[i-1]) * 111320
        dlon = (lons[i] - lons[i-1]) * 111320 * math.cos(math.radians(avg_lat))
        total_dist += math.sqrt(dlat**2 + dlon**2)
    
    print(f"  Total path length: {total_dist:.0f}m")
    
    # Analysis
    print(f"\nAnalysis:")
    if abs(avg_lat - 37.802) < 0.001:
        print(f"  ✓ Latitude matches Lombard Street area")
    else:
        print(f"  ⚠️  Latitude doesn't match!")
    
    if -122.4202 < min(lons) and max(lons) < -122.4185:
        print(f"  ✓ Longitude range matches crooked section")
    else:
        print(f"  ⚠️  Longitude range off!")
    
    if len(LOMBARD_CROOKED_SECTION) < 50:
        print(f"  ⚠️  Only {len(LOMBARD_CROOKED_SECTION)} points for 8 hairpins!")
        print(f"      This will cause jagged/angular appearance.")
        print(f"      Recommend: Add intermediate points (10-15 per hairpin)")
    
    # Check spacing
    spacings = []
    for i in range(1, len(LOMBARD_CROOKED_SECTION)):
        dlat = (lats[i] - lats[i-1]) * 111320
        dlon = (lons[i] - lons[i-1]) * 111320 * math.cos(math.radians(avg_lat))
        spacings.append(math.sqrt(dlat**2 + dlon**2))
    
    print(f"\n  Point spacing:")
    print(f"    Min: {min(spacings):.1f}m")
    print(f"    Max: {max(spacings):.1f}m")
    print(f"    Avg: {sum(spacings)/len(spacings):.1f}m")
    
    print("\n" + "="*70)


if __name__ == "__main__":
    print("ApexVelocity Coordinate Debugging")
    print("="*70)
    
    check_lombard_coordinates()
    check_nurburgring_scale()





