#!/usr/bin/env python3
"""
Experimental / utility OSM fetch helpers.

NOTE:
- This module is not part of the stable public API for ApexVelocity.
- For production use, prefer `apexvelocity.loader.OSMLoader`.
- Functions here may change or be removed in future versions without a
  major version bump.

It provides Overpass/OSRM helpers used by bundled examples to fetch
street or track coordinates for demos and visualizations.
"""

import json
import math
import urllib.request
import urllib.parse
from typing import List, Dict, Optional, Tuple

# Overpass API endpoint
OVERPASS_URL = "https://overpass-api.de/api/interpreter"


def fetch_way_by_name(
    name: str,
    city: Optional[str] = None,
    country: Optional[str] = None,
    way_type: str = "highway",
) -> List[Dict]:
    """
    Fetch a street/road by name from OpenStreetMap.
    
    Args:
        name: Street name (e.g., "Lombard Street")
        city: City name (e.g., "San Francisco")
        country: Country (e.g., "USA")
        way_type: OSM way type (highway, building, etc.)
    
    Returns:
        List of {lat, lon} coordinate dicts
    """
    # Build search area
    area_filters = []
    if city:
        area_filters.append(f'area["name"="{city}"]')
    if country:
        area_filters.append(f'area["name"="{country}"]')
    
    if area_filters:
        area_query = " -> .searchArea;\n".join(area_filters) + " -> .searchArea;"
        area_ref = "(area.searchArea)"
    else:
        area_query = ""
        area_ref = ""
    
    query = f"""
    [out:json][timeout:30];
    {area_query}
    way["{way_type}"]["name"~"{name}",i]{area_ref};
    out geom;
    """
    
    return _execute_overpass_query(query)


def fetch_way_by_bbox(
    min_lat: float,
    min_lon: float, 
    max_lat: float,
    max_lon: float,
    way_type: str = "highway",
    name_filter: Optional[str] = None,
) -> List[Dict]:
    """
    Fetch ways within a bounding box.
    
    Args:
        min_lat, min_lon, max_lat, max_lon: Bounding box
        way_type: OSM way type
        name_filter: Optional regex to filter by name
    
    Returns:
        List of {lat, lon} coordinate dicts
    """
    name_clause = f'["name"~"{name_filter}",i]' if name_filter else ""
    
    query = f"""
    [out:json][timeout:30];
    way["{way_type}"]{name_clause}({min_lat},{min_lon},{max_lat},{max_lon});
    out geom;
    """
    
    return _execute_overpass_query(query)


def fetch_relation_by_id(relation_id: int) -> List[Dict]:
    """
    Fetch a relation (like a race track) by its OSM relation ID.
    
    Args:
        relation_id: OSM relation ID (e.g., 36916 for Nürburgring)
    
    Returns:
        List of {lat, lon} coordinate dicts forming the route
    """
    query = f"""
    [out:json][timeout:60];
    relation({relation_id});
    way(r);
    out geom;
    """
    
    return _execute_overpass_query(query)


def fetch_route_by_coordinates(
    start_lat: float,
    start_lon: float,
    end_lat: float,
    end_lon: float,
    profile: str = "car",
) -> List[Dict]:
    """
    Fetch a route between two points using OSRM.
    
    Args:
        start_lat, start_lon: Start coordinates
        end_lat, end_lon: End coordinates
        profile: Routing profile (car, bike, foot)
    
    Returns:
        List of {lat, lon} coordinate dicts
    """
    # Use OSRM demo server
    url = f"https://router.project-osrm.org/route/v1/{profile}/"
    url += f"{start_lon},{start_lat};{end_lon},{end_lat}"
    url += "?geometries=geojson&overview=full"
    
    try:
        with urllib.request.urlopen(url, timeout=30) as response:
            data = json.loads(response.read().decode('utf-8'))
            
            if data.get('code') != 'Ok' or not data.get('routes'):
                print(f"Warning: OSRM routing failed: {data.get('code')}")
                return []
            
            coords = data['routes'][0]['geometry']['coordinates']
            return [{'lat': c[1], 'lon': c[0]} for c in coords]
            
    except Exception as e:
        print(f"Warning: OSRM request failed: {e}")
        return []


def fetch_lombard_street_sf() -> List[Dict]:
    """
    Fetch the famous crooked section of Lombard Street, San Francisco.
    
    The crooked section is specifically on the 1000 block between Hyde St
    and Leavenworth St. This is a VERY small area: ~200m x 50m.
    
    Returns:
        List of {lat, lon} coordinate dicts that belong specifically to
        the Lombard Street carriageway, not the cross streets.
    """
    # Very tight bounding box around ONLY the crooked section.
    # Hyde St is at lon≈-122.4186, Leavenworth at lon≈-122.4201.
    # The crooked block is roughly:
    #   lat:  37.8018 – 37.8022
    #   lon: -122.4202 – -122.4185
    #
    # We keep a tiny margin but deliberately exclude the long straight
    # blocks west of Leavenworth so the path does not extend past the
    # famous hairpin segment.
    min_lat, min_lon = 37.8018, -122.4202
    max_lat, max_lon = 37.8022, -122.4185

    # IMPORTANT:
    # ---------
    # The original implementation queried *all* highways inside this bbox:
    #
    #   way["highway"](bbox);
    #
    # and then concatenated every node into a single list before running
    # nearest‑neighbour ordering. That meant we were stitching together
    # Hyde, Lombard, Leavenworth and even nearby driveways into one path,
    # which produced the “noodling” shape offset from the real street in
    # the visualization.
    #
    # To fix this, we *only* fetch ways whose name matches Lombard Street
    # inside the crooked‑section bbox. This keeps the geometry strictly on
    # the intended block while still being fully data‑driven from OSM.
    coords = fetch_way_by_bbox(
        min_lat=min_lat,
        min_lon=min_lon,
        max_lat=max_lat,
        max_lon=max_lon,
        way_type="highway",
        name_filter="Lombard",
    )

    if coords:
        # Extra defensive filter in case Overpass returns any neighbouring
        # geometry that happens to match the name but lies slightly out of
        # our tight crooked‑section envelope.
        filtered = [
            c
            for c in coords
            if min_lat <= c["lat"] <= max_lat
            and min_lon <= c["lon"] <= max_lon
        ]
        if len(filtered) >= 5:
            print(f"  Filtered to {len(filtered)} Lombard points in crooked section")
            return filtered

    print("  OSM query returned insufficient Lombard data, using fallback")
    return []


def fetch_nurburgring_nordschleife() -> List[Dict]:
    """
    Fetch the Nürburgring Nordschleife track geometry.
    
    The Nordschleife track road is named "Nordschleife" in OSM.
    We fetch all ways with this name within the track's bounding box.
    
    Returns:
        List of {lat, lon} coordinate dicts
    """
    print("Fetching Nürburgring Nordschleife from OSM...")
    
    # The Nordschleife track roads are named "Nordschleife" in OSM
    # Bounding box covers the full 20.8km track
    query = """
    [out:json][timeout:60];
    (
      way["name"="Nordschleife"](50.31,6.93,50.36,7.02);
      way["highway"]["name"~"Nordschleife"](50.31,6.93,50.36,7.02);
    );
    out geom;
    """
    
    coords = _execute_overpass_query(query)
    
    if coords and len(coords) >= 100:
        print(f"  Got {len(coords)} points for Nordschleife track")
        return coords
    
    # Fallback: try the race track relation
    print("  Trying race track relation...")
    query = """
    [out:json][timeout:60];
    relation["name"~"Nordschleife"];
    way(r);
    out geom;
    """
    
    coords = _execute_overpass_query(query)
    
    if coords and len(coords) >= 100:
        return coords
    
    # Final fallback: get any road in the area
    print("  Using general road approach...")
    query = """
    [out:json][timeout:60];
    way["highway"~"primary|secondary|tertiary"](50.32,6.94,50.36,7.01);
    out geom;
    """
    
    coords = _execute_overpass_query(query)
    
    return coords if coords else []


def _execute_overpass_query(query: str) -> List[Dict]:
    """Execute an Overpass API query and return coordinates + basic tags.

    We preserve key OSM tags (highway, surface, maxspeed, tracktype) on each
    coordinate so downstream callers (like Lombard routing) can infer more
    accurate materials and speed limits instead of hard-coding asphalt.
    """
    try:
        data = query.encode('utf-8')
        req = urllib.request.Request(
            OVERPASS_URL,
            data=urllib.parse.urlencode({'data': query}).encode('utf-8'),
            headers={'User-Agent': 'ApexVelocity/1.0'}
        )
        
        with urllib.request.urlopen(req, timeout=60) as response:
            result = json.loads(response.read().decode('utf-8'))
        
        # Extract coordinates from ways, carrying over relevant tags
        all_coords = []
        
        for element in result.get('elements', []):
            if element.get('type') == 'way' and 'geometry' in element:
                tags = element.get('tags', {}) or {}
                highway = tags.get('highway', '')
                surface = tags.get('surface', '')
                maxspeed = tags.get('maxspeed', None)
                tracktype = tags.get('tracktype', '')
                
                for node in element['geometry']:
                    all_coords.append({
                        'lat': node['lat'],
                        'lon': node['lon'],
                        'highway': highway,
                        'surface': surface,
                        'maxspeed': maxspeed,
                        'tracktype': tracktype,
                    })
        
        # Remove duplicates while preserving order
        seen = set()
        unique_coords = []
        for c in all_coords:
            key = (round(c['lat'], 6), round(c['lon'], 6))
            if key not in seen:
                seen.add(key)
                unique_coords.append(c)
        
        print(f"  Fetched {len(unique_coords)} coordinates from OSM")
        return unique_coords
        
    except Exception as e:
        print(f"Warning: Overpass query failed: {e}")
        return []


def order_coordinates_as_path(
    coords: List[Dict],
    start_point: Optional[Tuple[float, float]] = None,
) -> List[Dict]:
    """
    Order a set of coordinates into a continuous path using nearest-neighbor.
    
    Args:
        coords: List of {lat, lon} dicts
        start_point: Optional (lat, lon) to start from
    
    Returns:
        Ordered list of coordinates
    """
    if len(coords) < 2:
        return coords
    
    remaining = coords.copy()
    
    # Find starting point
    if start_point:
        start_lat, start_lon = start_point
        closest_idx = min(range(len(remaining)), 
                         key=lambda i: _haversine_dist(
                             remaining[i]['lat'], remaining[i]['lon'],
                             start_lat, start_lon))
        current = remaining.pop(closest_idx)
    else:
        current = remaining.pop(0)
    
    ordered = [current]
    
    while remaining:
        # Find nearest point
        nearest_idx = min(range(len(remaining)),
                         key=lambda i: _haversine_dist(
                             current['lat'], current['lon'],
                             remaining[i]['lat'], remaining[i]['lon']))
        current = remaining.pop(nearest_idx)
        ordered.append(current)
    
    return ordered


def _haversine_dist(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Calculate Haversine distance between two points in meters."""
    R = 6371000  # Earth radius in meters
    
    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    
    a = math.sin(dlat/2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    
    return R * c


def smooth_path(coords: List[Dict], window: int = 3) -> List[Dict]:
    """Apply moving average smoothing to path coordinates."""
    if len(coords) < window:
        return coords
    
    smoothed = []
    half_window = window // 2
    
    for i in range(len(coords)):
        start = max(0, i - half_window)
        end = min(len(coords), i + half_window + 1)
        
        avg_lat = sum(c['lat'] for c in coords[start:end]) / (end - start)
        avg_lon = sum(c['lon'] for c in coords[start:end]) / (end - start)
        
        smoothed.append({'lat': avg_lat, 'lon': avg_lon})
    
    return smoothed


def calculate_curvature(coords: List[Dict]) -> List[float]:
    """Calculate curvature at each point using 3-point circumcircle."""
    curvatures = [0.0]  # First point has no curvature
    
    for i in range(1, len(coords) - 1):
        p1, p2, p3 = coords[i-1], coords[i], coords[i+1]
        
        # Convert to local meters
        cos_lat = math.cos(math.radians(p2['lat']))
        x1 = (p1['lon'] - p2['lon']) * 111320 * cos_lat
        y1 = (p1['lat'] - p2['lat']) * 111320
        x3 = (p3['lon'] - p2['lon']) * 111320 * cos_lat
        y3 = (p3['lat'] - p2['lat']) * 111320
        
        # Menger curvature: 4*area / (d12 * d23 * d31)
        area = abs((-x1) * y3 - x3 * (-y1)) / 2
        d12 = math.sqrt(x1**2 + y1**2)
        d23 = math.sqrt(x3**2 + y3**2)
        d31 = math.sqrt((x1-x3)**2 + (y1-y3)**2)
        
        denom = d12 * d23 * d31
        if denom > 0.1:
            curvature = min(4 * area / denom, 0.1)  # Cap at 10m radius
        else:
            curvature = 0.0
        
        curvatures.append(curvature)
    
    curvatures.append(0.0)  # Last point
    return curvatures


def build_path_points(
    coords: List[Dict],
    surface_type: str = "asphalt",
    base_elevation: float = 0.0,
) -> List[Dict]:
    """
    Convert coordinates to full path points for the solver.
    
    Args:
        coords: List of {lat, lon} dicts
        surface_type: Road surface material
        base_elevation: Base elevation (set to 0 for flat display)
    
    Returns:
        List of path point dicts ready for the solver
    """
    if not coords:
        return []
    
    curvatures = calculate_curvature(coords)
    path_points = []
    cumulative_distance = 0.0
    
    for i, coord in enumerate(coords):
        if i > 0:
            dist = _haversine_dist(
                coords[i-1]['lat'], coords[i-1]['lon'],
                coord['lat'], coord['lon']
            )
            cumulative_distance += dist
        
        path_points.append({
            'lat': coord['lat'],
            'lon': coord['lon'],
            'x_m': (coord['lon'] - coords[0]['lon']) * 111320 * math.cos(math.radians(coord['lat'])),
            'y_m': (coord['lat'] - coords[0]['lat']) * 111320,
            'z_m': base_elevation,
            'elevation_m': base_elevation,
            'curvature': curvatures[i],
            'distance_along_m': cumulative_distance,
            'surface_type': surface_type,
            'speed_mps': 20.0,
            'v_profile': 20.0,
            'energy_joules': 0.0,
            'energy_kwh': 0.0,
        })
    
    return path_points


# Convenience functions for common locations
def get_lombard_street() -> List[Dict]:
    """Get Lombard Street path points ready for visualization."""
    coords = fetch_lombard_street_sf()
    if coords:
        coords = order_coordinates_as_path(coords)
        return build_path_points(coords, surface_type="asphalt", base_elevation=0)
    return []


def get_nurburgring() -> List[Dict]:
    """Get Nürburgring Nordschleife path points ready for visualization."""
    coords = fetch_nurburgring_nordschleife()
    if coords:
        coords = order_coordinates_as_path(coords)
        return build_path_points(coords, surface_type="asphalt", base_elevation=0)
    return []


if __name__ == "__main__":
    # Test the fetcher
    print("Testing OSM Fetcher...")
    
    print("\n=== Lombard Street ===")
    lombard = fetch_lombard_street_sf()
    if lombard:
        print(f"Got {len(lombard)} points")
        print(f"First: {lombard[0]}")
        print(f"Last: {lombard[-1]}")
    
    print("\n=== Nürburgring ===")
    nurburgring = fetch_nurburgring_nordschleife()
    if nurburgring:
        print(f"Got {len(nurburgring)} points")
        print(f"First: {nurburgring[0]}")
        print(f"Last: {nurburgring[-1]}")
