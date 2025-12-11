"""
Route building utilities for ApexVelocity.

This module provides a thin, stable layer around OSMLoader + geometry so that:
  - Demos (e.g., Lombard Street) can request routes by start/end coordinates
  - The CLI can build arbitrary routes in any city
  - Surfaces/materials are inferred from OSM tags using config/tag_mapping.json

Key API:
  - RouteRequest: simple dataclass describing a route query
  - build_route: returns a list[dict] path in the format expected by
                 analysis.analyze_profile and viz.visualize_profile_interactive

Note: We avoid Python 3.7+ typing features (e.g. PEP 563 annotations) to
remain compatible with the Python 3.6 runtime used in this environment.
"""

from dataclasses import dataclass
from typing import List, Dict, Tuple, Optional
import os
import math

from .loader import OSMLoader
from .geometry import smooth_path_geo, recompute_curvature, validate_path_alignment

try:
    # DEFAULT_CONFIG_DIR is provided by the core bindings and re-exported in __init__
    from . import DEFAULT_CONFIG_DIR as _DEFAULT_CONFIG_DIR
except Exception:
    _DEFAULT_CONFIG_DIR = None  # Fallback; OSMLoader will use its own defaults


@dataclass
class RouteRequest:
    """
    High-level route request description.

    Attributes:
        start: (lat, lon) of route start
        end: (lat, lon) of route end
        place_hint: Optional place name for OSM graph loading
        vehicle: Vehicle preset name (passed through to solver, not used here)
        condition: "dry" or "wet" (informational at this stage)
        network_type: OSMnx network type (default: "drive")
        target_spacing_m: Resampling spacing in meters for smoothing
    """

    start: Tuple[float, float]
    end: Tuple[float, float]
    place_hint: Optional[str] = None
    vehicle: str = "tesla_model_3"
    condition: str = "dry"
    network_type: str = "drive"
    target_spacing_m: float = 5.0


def _get_config_dir() -> Optional[str]:
    """
    Determine the config directory for tag mappings/materials.

    Preference order:
      1) APEXVELOCITY_CONFIG_DIR env var
      2) DEFAULT_CONFIG_DIR from core bindings (if available)
    """
    env_dir = os.environ.get("APEXVELOCITY_CONFIG_DIR")
    if env_dir and os.path.isdir(env_dir):
        return env_dir
    if _DEFAULT_CONFIG_DIR and os.path.isdir(_DEFAULT_CONFIG_DIR):
        return _DEFAULT_CONFIG_DIR
    return None


def build_route(request: RouteRequest) -> List[Dict]:
    """
    Build a route between two coordinates using OSMLoader + geometry smoothing.

    Returns:
        List[dict] path points with keys:
            lat, lon, x_m, y_m, z_m, elevation_m,
            curvature, distance_along_m,
            surface_type, grade_percent, grade_angle_rad,
            osm_highway, osm_surface, osm_speed_limit_kmh,
            speed_mps, v_profile, energy_joules, energy_kwh,
            surface_inferred (bool)
    """
    config_dir = _get_config_dir()
    loader = OSMLoader(config_dir=config_dir)

    # Special-case Lombard Street: use dedicated OSM fetcher for high-res geometry
    def _is_lombard_area(start, end, place_hint):
        lat1, lon1 = start
        lat2, lon2 = end
        if not place_hint or "san francisco" not in place_hint.lower():
            return False
        lat_min, lat_max = 37.8017, 37.8024
        lon_min, lon_max = -122.4208, -122.4183
        return (
            lat_min <= lat1 <= lat_max
            and lon_min <= lon1 <= lon_max
            and lat_min <= lat2 <= lat_max
            and lon_min <= lon2 <= lon_max
        )

    raw_points: List[Dict] = []

    if _is_lombard_area(request.start, request.end, request.place_hint or ""):
        try:
            from .osm_fetcher import (  # type: ignore
                fetch_lombard_street_sf,
                order_coordinates_as_path,
                build_path_points,
            )

            coords = fetch_lombard_street_sf()
            if coords:
                # Order all Lombard coordinates into a continuous path, then
                # extract only the sub‑segment between the requested start/end
                # coordinates so we don't wander onto neighbouring blocks.
                ordered = order_coordinates_as_path(coords)

                def _closest_index(target_lat: float, target_lon: float) -> int:
                    best_idx = 0
                    best_d2 = float("inf")
                    for i, c in enumerate(ordered):
                        dy = (c["lat"] - target_lat) * 111320.0
                        dx = (
                            (c["lon"] - target_lon)
                            * 111320.0
                            * math.cos(math.radians(target_lat))
                        )
                        d2 = dx * dx + dy * dy
                        if d2 < best_d2:
                            best_d2 = d2
                            best_idx = i
                    return best_idx

                s_idx = _closest_index(request.start[0], request.start[1])
                e_idx = _closest_index(request.end[0], request.end[1])

                if s_idx <= e_idx:
                    segment = ordered[s_idx : e_idx + 1]
                else:
                    segment = list(reversed(ordered[e_idx : s_idx + 1]))

                pts = build_path_points(
                    segment, surface_type="asphalt", base_elevation=0.0
                )

                # Build a tiny OSM graph over the crooked block to sample both
                # elevation (from DEM, if configured) and speed limits from
                # nearby edges. This keeps Lombard fully data-driven.
                node_elevations: Dict[Tuple[float, float], float] = {}
                node_speed_limits: Dict[Tuple[float, float], float] = {}

                if pts:
                    lats = [p["lat"] for p in pts]
                    lons = [p["lon"] for p in pts]
                    north = max(lats) + 0.001
                    south = min(lats) - 0.001
                    east = max(lons) + 0.001
                    west = min(lons) - 0.001

                    try:
                        G_meta = loader.graph_from_bbox(
                            north, south, east, west, network_type=request.network_type
                        )

                        # Optional: attach real elevation from DEM if configured
                        elev_method = os.environ.get(
                            "APEXVELOCITY_FAST_TESTS"
                        ) != "1" and os.environ.get("APEXVELOCITY_ELEVATION_METHOD")
                        if elev_method:
                            G_meta = loader.add_elevation(
                                G_meta, method=str(elev_method)
                            )

                        # Cache node elevations keyed by (lat, lon)
                        for nid, data in G_meta.nodes(data=True):
                            node_lat = data.get("y", None)
                            node_lon = data.get("x", None)
                            if node_lat is None or node_lon is None:
                                continue
                            key = (float(node_lat), float(node_lon))
                            z = data.get("elevation", None)
                            if z is not None:
                                node_elevations[key] = float(z)

                        # Derive a representative speed limit per node from
                        # incident edges' maxspeed tags.
                        for nid, data in G_meta.nodes(data=True):
                            node_lat = data.get("y", None)
                            node_lon = data.get("x", None)
                            if node_lat is None or node_lon is None:
                                continue
                            speeds: List[float] = []
                            for nbr, edge_dict in G_meta[nid].items():
                                if not edge_dict:
                                    continue
                                # Handle multi-edges (0, 1, ...)
                                edge_data = (
                                    edge_dict[0] if 0 in edge_dict else edge_dict
                                )
                                maxspeed = edge_data.get("maxspeed")
                                if not maxspeed:
                                    continue
                                if isinstance(maxspeed, list):
                                    maxspeed = maxspeed[0]
                                try:
                                    s_val = str(maxspeed)
                                    if "mph" in s_val:
                                        s_num = (
                                            float(s_val.replace(" mph", "").strip())
                                            * 1.60934
                                        )
                                    elif "km/h" in s_val:
                                        s_num = float(
                                            s_val.replace(" km/h", "").strip()
                                        )
                                    else:
                                        s_num = float(s_val.strip())
                                    speeds.append(s_num)
                                except Exception:
                                    continue
                            if speeds:
                                key = (float(node_lat), float(node_lon))
                                node_speed_limits[key] = sum(speeds) / len(speeds)

                    except Exception:
                        node_elevations = {}
                        node_speed_limits = {}

                def _nearest_elevation(
                    lat: float, lon: float, default: float = 0.0
                ) -> float:
                    if not node_elevations:
                        return default
                    best = None
                    best_d2 = float("inf")
                    for (nlat, nlon), z in node_elevations.items():
                        dy = (nlat - lat) * 111320.0
                        dx = (nlon - lon) * 111320.0 * math.cos(math.radians(lat))
                        d2 = dx * dx + dy * dy
                        if d2 < best_d2:
                            best_d2 = d2
                            best = z
                    return default if best is None else best

                def _nearest_speed_limit(
                    lat: float, lon: float, default: Optional[float] = None
                ) -> Optional[float]:
                    if not node_speed_limits:
                        return default
                    best = None
                    best_d2 = float("inf")
                    for (nlat, nlon), s in node_speed_limits.items():
                        dy = (nlat - lat) * 111320.0
                        dx = (nlon - lon) * 111320.0 * math.cos(math.radians(lat))
                        d2 = dx * dx + dy * dy
                        if d2 < best_d2:
                            best_d2 = d2
                            best = s
                    return default if best is None else best

                # Build raw_points with optional real elevation, inferred speed
                # limit, and inferred surface/material type.
                # Pre-compute a simple per-node inferred surface type using
                # the same TagMapper that OSMLoader uses for generic routes.
                node_surfaces: Dict[Tuple[float, float], str] = {}
                if "node_speed_limits" in locals() and node_speed_limits:
                    try:
                        for nid, data in G_meta.nodes(data=True):
                            node_lat = data.get("y", None)
                            node_lon = data.get("x", None)
                            if node_lat is None or node_lon is None:
                                continue
                            materials: List[str] = []
                            for nbr, edge_dict in G_meta[nid].items():
                                if not edge_dict:
                                    continue
                                edge_data = (
                                    edge_dict[0]
                                    if 0 in edge_dict
                                    else list(edge_dict.values())[0]
                                )
                                highway = edge_data.get("highway", "") or ""
                                surface = edge_data.get("surface", "") or ""
                                tracktype = edge_data.get("tracktype", "") or ""
                                smoothness = edge_data.get("smoothness", "") or ""
                                try:
                                    material = loader.tag_mapper.get_material(
                                        highway=highway,
                                        surface=surface,
                                        tracktype=tracktype,
                                        smoothness=smoothness,
                                    )
                                    if material:
                                        materials.append(material)
                                except Exception:
                                    continue
                            if materials:
                                key = (float(node_lat), float(node_lon))
                                # Simple majority/first material
                                node_surfaces[key] = materials[0]
                    except Exception:
                        node_surfaces = {}

                def _nearest_surface(
                    lat: float, lon: float, default: str = "asphalt"
                ) -> str:
                    if not node_surfaces:
                        return default
                    best = None
                    best_d2 = float("inf")
                    for (nlat, nlon), mat in node_surfaces.items():
                        dy = (nlat - lat) * 111320.0
                        dx = (nlon - lon) * 111320.0 * math.cos(math.radians(lat))
                        d2 = dx * dx + dy * dy
                        if d2 < best_d2:
                            best_d2 = d2
                            best = mat
                    return default if best is None else best

                for p in pts:
                    # Elevation and speed limit come from nearest sampled node.
                    base_z = p.get("z_m", 0.0)
                    z = _nearest_elevation(p["lat"], p["lon"], default=base_z)
                    s_lim = _nearest_speed_limit(
                        p["lat"], p["lon"], default=p.get("speed_limit_kmh")
                    )
                    surface_type = _nearest_surface(
                        p["lat"],
                        p["lon"],
                        default=p.get("surface_type", "asphalt"),
                    )

                    raw_points.append(
                        {
                            "lat": p["lat"],
                            "lon": p["lon"],
                            "x_m": p["x_m"],
                            "y_m": p["y_m"],
                            "z_m": z,
                            "elevation_m": z,
                            "curvature": p.get("curvature", 0.0),
                            "distance_along_m": p.get("distance_along_m", 0.0),
                            "surface_type": surface_type,
                            "surface_inferred": False,
                            "grade_angle_rad": 0.0,  # will be recomputed in analysis
                            "grade_percent": 0.0,  # derived later from z_m
                            "osm_highway": "residential",
                            "osm_surface": surface_type,
                            "osm_speed_limit_kmh": s_lim,
                            "speed_limit_kmh": s_lim,
                            "osm_node_id": None,
                            "speed_mps": 0.0,
                            "v_profile": 0.0,
                            "energy_joules": 0.0,
                            "energy_kwh": 0.0,
                        }
                    )
        except Exception:
            raw_points = []

    if not raw_points:
        # Generic OSMLoader-based route
        if request.place_hint:
            G = loader.graph_from_place(
                request.place_hint, network_type=request.network_type
            )
        else:
            # Small bbox around start/end if no place hint is provided
            lat1, lon1 = request.start
            lat2, lon2 = request.end
            north = max(lat1, lat2) + 0.01
            south = min(lat1, lat2) - 0.01
            east = max(lon1, lon2) + 0.01
            west = min(lon1, lon2) + 0.01
            G = loader.graph_from_bbox(
                north, south, east, west, network_type=request.network_type
            )

        # Optionally add real elevation/grade from configured DEM provider
        elev_method = os.environ.get(
            "APEXVELOCITY_FAST_TESTS"
        ) != "1" and os.environ.get("APEXVELOCITY_ELEVATION_METHOD")
        if elev_method:
            try:
                G = loader.add_elevation(G, method=str(elev_method))
            except Exception:
                # If elevation fails, continue with flat z_m
                pass

        # Find route using shortest path on the road network
        route = loader.get_shortest_path(
            G, origin=request.start, destination=request.end
        )
        loaded = loader.extract_path(G, route)

        for pt in loaded.points:
            inferred = not bool(pt.osm_surface or pt.osm_highway)
            raw_points.append(
                {
                    "lat": pt.lat,
                    "lon": pt.lon,
                    "x_m": pt.x_m,
                    "y_m": pt.y_m,
                    "z_m": pt.z_m,
                    "elevation_m": pt.z_m,
                    "curvature": pt.curvature,
                    "distance_along_m": pt.distance_along_m,
                    "surface_type": pt.surface_type,
                    "surface_inferred": inferred,
                    "grade_angle_rad": pt.grade_angle_rad,
                    "grade_percent": pt.grade_percent,
                    "osm_highway": pt.osm_highway,
                    "osm_surface": pt.osm_surface,
                    "osm_speed_limit_kmh": pt.osm_speed_limit_kmh,
                    # Standard key used by analysis / viz
                    "speed_limit_kmh": pt.speed_limit_kmh or pt.osm_speed_limit_kmh,
                    "osm_node_id": pt.osm_node_id,
                    "speed_mps": 0.0,
                    "v_profile": 0.0,
                    "energy_joules": 0.0,
                    "energy_kwh": 0.0,
                }
            )

    # Smooth & resample in geo space for better curvature / visualization.
    # For highly curved special cases (e.g., Lombard crooked block), we use
    # Catmull–Rom to produce a racing-line style smooth curve. For generic
    # city routes, we stick to LineString interpolation so that straight
    # segments and right-angle turns remain visually straight and aligned
    # with the underlying road grid.
    smooth_mode = (
        "catmull_rom"
        if raw_points
        and _is_lombard_area(request.start, request.end, request.place_hint or "")
        else "linestring"
    )

    smoothed = smooth_path_geo(
        raw_points,
        target_spacing_m=request.target_spacing_m,
        mode=smooth_mode,
    )
    smoothed = recompute_curvature(smoothed, smooth_window=5)

    # Safety check: if smoothing drifts too far from the original OSM
    # polyline (e.g., creates "noodling" artifacts), fall back to the
    # unsmoothed geometry while still recomputing curvature.
    try:
        if not validate_path_alignment(raw_points, smoothed, max_offset_m=5.0):
            smoothed = recompute_curvature(raw_points, smooth_window=5)
    except Exception:
        # On any validation error, just return smoothed as-is
        pass

    return smoothed
