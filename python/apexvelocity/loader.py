"""
Canonical OSM Ingestion for ApexVelocity.

This module is the public, stable API for loading road-network data.
It fetches from OpenStreetMap via OSMnx and constructs geometry suitable
for the C++ core solver (see `LoadedPath.to_geometry_list`).

For production use, prefer `apexvelocity.loader.OSMLoader`.
The `apexvelocity.osm_fetcher` module is experimental/demo-only.
"""

from typing import List, Dict, Tuple, Optional, Union, Any
from dataclasses import dataclass, field
import math
import json
import os

try:
    import osmnx as ox
    import networkx as nx
    _HAS_OSMNX = True
except ImportError:
    _HAS_OSMNX = False
    ox = None
    nx = None

try:
    import numpy as np
    _HAS_NUMPY = True
except ImportError:
    _HAS_NUMPY = False
    np = None

try:
    from pyproj import Transformer
    _HAS_PYPROJ = True
except ImportError:
    _HAS_PYPROJ = False
    Transformer = None

try:
    import requests
    _HAS_REQUESTS = True
except ImportError:
    _HAS_REQUESTS = False
    requests = None


@dataclass
class LoadedPathPoint:
    """A loaded path point with all attributes."""
    lat: float = 0.0
    lon: float = 0.0
    x_m: float = 0.0  # Local X coordinate (meters)
    y_m: float = 0.0  # Local Y coordinate (meters)
    z_m: float = 0.0  # Elevation (meters)
    curvature: float = 0.0  # 1/radius (1/m)
    distance_along_m: float = 0.0  # Cumulative distance
    surface_type: str = "asphalt"  # Inferred material
    grade_angle_rad: float = 0.0  # Slope angle
    grade_percent: float = 0.0  # Slope as percentage
    osm_highway: str = ""  # OSM highway tag
    osm_surface: str = ""  # OSM surface tag
    osm_speed_limit_kmh: Optional[float] = None  # Legal speed limit
    speed_limit_kmh: Optional[float] = None  # Alias used by analysis/visualization
    osm_node_id: Optional[int] = None  # OSM node ID


@dataclass
class LoadedPath:
    """A loaded path with metadata."""
    points: List[LoadedPathPoint] = field(default_factory=list)
    total_distance_m: float = 0.0
    total_climb_m: float = 0.0  # Total elevation gain
    total_descent_m: float = 0.0  # Total elevation loss
    name: str = ""
    source: str = ""  # "osmnx", "gpx", etc.
    
    def to_geometry_list(self) -> Tuple[List[List[float]], List[str]]:
        """Convert to format expected by solve_profile."""
        geometry = []
        surfaces = []
        for pt in self.points:
            geometry.append([
                pt.x_m, pt.y_m, pt.z_m,
                pt.curvature, pt.distance_along_m
            ])
            surfaces.append(pt.surface_type)
        return geometry, surfaces


class TagMapper:
    """Maps OSM tags to material types."""
    
    def __init__(self, mapping_file: Optional[str] = None):
        self.mapping = {}
        if mapping_file and os.path.exists(mapping_file):
            with open(mapping_file) as f:
                self.mapping = json.load(f)
        else:
            # Default mapping
            self.mapping = {
                "highway=motorway": "asphalt",
                "highway=motorway_link": "asphalt",
                "highway=trunk": "asphalt",
                "highway=primary": "asphalt",
                "highway=secondary": "asphalt",
                "highway=tertiary": "asphalt",
                "highway=residential": "asphalt",
                "highway=service": "asphalt",
                "highway=track": "dirt",
                "highway=path": "dirt",
                "highway=footway": "concrete",
                "highway=cycleway": "asphalt",
                "surface=asphalt": "asphalt",
                "surface=concrete": "concrete",
                "surface=paving_stones": "cobblestone",
                "surface=bricks": "cobblestone",
                "surface=brick": "cobblestone",
                "surface=cobblestone": "cobblestone",
                "surface=sett": "cobblestone",
                "surface=gravel": "gravel",
                "surface=dirt": "dirt",
                "surface=sand": "sand",
                "surface=grass": "dirt",
                "surface=mud": "dirt",
                "surface=ice": "ice",
                "surface=snow": "snow_packed",
            }
    
    def get_material(self, highway: str = "", surface: str = "", 
                     tracktype: str = "", smoothness: str = "") -> str:
        """Get material type from OSM tags."""
        # Surface tag takes priority
        if surface:
            key = f"surface={surface}"
            if key in self.mapping:
                return self.mapping[key]
        
        # Then highway tag
        if highway:
            key = f"highway={highway}"
            if key in self.mapping:
                return self.mapping[key]
        
        # Track type hints
        if tracktype:
            if tracktype in ["grade1", "grade2"]:
                return "gravel"
            elif tracktype in ["grade3", "grade4", "grade5"]:
                return "dirt"
        
        # Default
        return "asphalt"


class OSMLoader:
    """
    Loads road network data from OpenStreetMap.

    Provides helpers to fetch graphs by place/bbox, extract a concrete
    route into local-meter coordinates with curvature, grade, and inferred
    surface types, and export geometry/surface arrays for the core solver.
    """
    
    def __init__(self, config_dir: Optional[str] = None):
        if not _HAS_OSMNX:
            raise ImportError("osmnx is required for OSM data loading. Install with: pip install osmnx")
        if not _HAS_NUMPY:
            raise ImportError("numpy is required. Install with: pip install numpy")
        
        # Load tag mapping
        mapping_file = None
        if config_dir:
            mapping_file = os.path.join(config_dir, "tag_mapping.json")
        self.tag_mapper = TagMapper(mapping_file)
        
        # UTM transformer (will be set based on location)
        self._transformer = None
        self._utm_zone = None
    
    def _setup_utm(self, lat: float, lon: float):
        """Setup UTM transformer for lat/lon to meters conversion."""
        if not _HAS_PYPROJ:
            # Fallback to simple approximation
            self._transformer = None
            self._center_lat = lat
            self._center_lon = lon
            return
        
        # Determine UTM zone
        utm_zone = int((lon + 180) / 6) + 1
        hemisphere = "north" if lat >= 0 else "south"
        
        # Create transformer
        self._transformer = Transformer.from_crs(
            "EPSG:4326",  # WGS84
            f"+proj=utm +zone={utm_zone} +{hemisphere} +ellps=WGS84",
            always_xy=True
        )
        self._utm_zone = utm_zone
    
    def _latlon_to_xy(self, lat: float, lon: float) -> Tuple[float, float]:
        """Convert lat/lon to local x,y coordinates in meters."""
        if self._transformer:
            x, y = self._transformer.transform(lon, lat)
            return x, y
        else:
            # Simple approximation (good enough for small areas)
            # 1 degree latitude ≈ 111,320 meters
            # 1 degree longitude ≈ 111,320 * cos(lat) meters
            y = (lat - self._center_lat) * 111320
            x = (lon - self._center_lon) * 111320 * math.cos(math.radians(lat))
            return x, y
    
    def graph_from_place(self, place_name: str, 
                         network_type: str = "drive") -> Any:
        """Fetch road network graph from place name."""
        ox.settings.log_console = False
        ox.settings.use_cache = True
        
        G = ox.graph_from_place(place_name, network_type=network_type)
        return G
    
    def graph_from_bbox(self, north: float, south: float,
                        east: float, west: float,
                        network_type: str = "drive") -> Any:
        """Fetch road network graph from bounding box."""
        ox.settings.log_console = False
        ox.settings.use_cache = True
        
        G = ox.graph_from_bbox(north, south, east, west, network_type=network_type)
        return G
    
    def graph_from_relation(self, relation_id: int) -> Any:
        """Fetch graph from OSM relation ID (e.g., race track)."""
        ox.settings.log_console = False
        ox.settings.use_cache = True
        
        # For race tracks, we need to get the way data
        # This uses Overpass API
        query = f"""
        [out:json][timeout:120];
        rel({relation_id});
        way(r);
        (._;>;);
        out body;
        """
        
        try:
            # Try to get as graph
            G = ox.graph_from_place(
                {"relation": relation_id},
                network_type="all"
            )
            return G
        except Exception:
            # Fallback: get the relation boundary and create graph
            try:
                gdf = ox.geocode_to_gdf(f"R{relation_id}", by_osmid=True)
                G = ox.graph_from_polygon(gdf.geometry.iloc[0], network_type="all")
                return G
            except Exception as e:
                raise ValueError(f"Could not load relation {relation_id}: {e}")
    
    def add_elevation(self, G: Any, method: str = "google") -> Any:
        """Add elevation data to graph nodes using real DEM providers.
        
        Supported methods:
            - \"google\": use Google Elevation API via osmnx (requires GOOGLE_API_KEY)
            - \"srtm\": use a local SRTM/DEM raster (set APEXVELOCITY_SRTM_PATH)
            - \"openapi\" / \"opentopo\": use OpenTopoData public API (SRTM, etc.)
        
        The default behaviour can be overridden via the APEXVELOCITY_ELEVATION_METHOD
        environment variable.
        """
        try:
            method = (method or "").lower()

            if method == "srtm":
                # Use local SRTM/DEM raster if configured
                tif_path = os.path.expanduser(
                    os.path.expandvars(os.getenv("APEXVELOCITY_SRTM_PATH", ""))
                )
                if tif_path and os.path.exists(tif_path):
                    G = ox.elevation.add_node_elevations_raster(G, tif_path)
                else:
                    # No raster provided; skip elevation rather than inventing data
                    return G

            elif method == "google":
                # Requires GOOGLE_API_KEY in the environment
                G = ox.elevation.add_node_elevations_google(G)

            elif method in ("openapi", "opentopo", "opentopodata"):
                # Open, Google-Elevation-style API backed by open DEMs.
                # Docs: https://www.opentopodata.org/api/
                if not _HAS_REQUESTS:
                    # requests not installed; cannot call remote API
                    return G
                G = self._add_elevation_opentopo(G)

            else:
                # Fallback to environment-controlled default if set
                env_method = os.environ.get("APEXVELOCITY_ELEVATION_METHOD", "").lower()
                if env_method and env_method != method:
                    return self.add_elevation(G, method=env_method)
                # If nothing configured, do nothing (no synthetic elevation)
                return G
        
            # Add edge grades from the populated node elevations
            G = ox.elevation.add_edge_grades(G)
        except Exception:
            # If external elevation provider fails, continue without elevation
            return G
        
        return G

    def _add_elevation_opentopo(self, G: Any) -> Any:
        """Populate node elevations using the OpenTopoData public API.
        
        This uses the /v1/<dataset_name> endpoint with batched POST requests.
        Configuration (via environment variables):
            - APEXVELOCITY_OPENTOPO_BASE_URL (default: https://api.opentopodata.org/v1)
            - APEXVELOCITY_OPENTOPO_DATASET (default: srtm90m)
            - APEXVELOCITY_OPENTOPO_BATCH (default: 100 nodes per request)
            - APEXVELOCITY_OPENTOPO_INTERPOLATION (default: bilinear)
        See: https://www.opentopodata.org/api/
        """
        if not _HAS_REQUESTS:
            return G

        base_url = os.environ.get(
            "APEXVELOCITY_OPENTOPO_BASE_URL",
            "https://api.opentopodata.org/v1",
        ).rstrip("/")
        dataset = os.environ.get("APEXVELOCITY_OPENTOPO_DATASET", "srtm90m")
        interpolation = os.environ.get(
            "APEXVELOCITY_OPENTOPO_INTERPOLATION", "bilinear"
        )
        try:
            batch_size = int(os.environ.get("APEXVELOCITY_OPENTOPO_BATCH", "100"))
            if batch_size <= 0:
                batch_size = 100
        except ValueError:
            batch_size = 100

        url = f"{base_url}/{dataset}"

        # Collect node ids and their lat/lon (OSMnx stores y=lat, x=lon).
        node_ids = list(G.nodes())
        if not node_ids:
            return G

        coords: List[Tuple[float, float]] = []
        for nid in node_ids:
            data = G.nodes[nid]
            lat = data.get("y", None)
            lon = data.get("x", None)
            if lat is None or lon is None:
                coords.append((0.0, 0.0))
            else:
                coords.append((float(lat), float(lon)))

        # Query in batches to keep requests manageable.
        for start_idx in range(0, len(node_ids), batch_size):
            end_idx = min(start_idx + batch_size, len(node_ids))
            batch_coords = coords[start_idx:end_idx]

            # Build locations string: "lat,lon|lat,lon|..."
            loc_str = "|".join(
                f"{lat:.6f},{lon:.6f}" for (lat, lon) in batch_coords
            )
            if not loc_str:
                continue

            payload = {
                "locations": loc_str,
                "interpolation": interpolation,
            }

            try:
                resp = requests.post(url, json=payload, timeout=10)
                resp.raise_for_status()
                data = resp.json()
            except Exception:
                # On any HTTP / parsing error, abort elevation for remaining nodes.
                break

            if data.get("status") != "OK":
                # Server-side error or invalid request; stop trying further batches.
                break

            results = data.get("results", [])
            for offset, res in enumerate(results):
                elevation = res.get("elevation", None)
                if elevation is None:
                    continue
                nid = node_ids[start_idx + offset]
                try:
                    G.nodes[nid]["elevation"] = float(elevation)
                except (ValueError, TypeError):
                    # Skip invalid elevation values
                    continue

        return G
    
    def get_shortest_path(self, G: Any, 
                          origin: Tuple[float, float],
                          destination: Tuple[float, float]) -> List[int]:
        """Get shortest path between two points.

        This uses osmnx's nearest_nodes helper when available. If that
        requires optional dependencies (e.g., scikit-learn on unprojected
        graphs), we fall back to a simple linear nearest-neighbor search
        over graph nodes using lat/lon.
        """
        lat_o, lon_o = origin
        lat_d, lon_d = destination

        def _nearest_node_fallback(lat: float, lon: float) -> int:
            best_node = None
            best_dist2 = float("inf")
            for nid, data in G.nodes(data=True):
                y = data.get("y", 0.0)
                x = data.get("x", 0.0)
                dy = (y - lat) * 111320.0
                dx = (x - lon) * 111320.0 * math.cos(math.radians(lat))
                d2 = dx * dx + dy * dy
                if d2 < best_dist2:
                    best_dist2 = d2
                    best_node = nid
            return best_node  # type: ignore

        try:
            orig_node = ox.distance.nearest_nodes(G, lon_o, lat_o)
            dest_node = ox.distance.nearest_nodes(G, lon_d, lat_d)
        except Exception as e:
            msg = str(e)
            if "scikit-learn must be installed" in msg or "requires scipy" in msg:
                # Fallback to simple nearest-neighbor search
                orig_node = _nearest_node_fallback(lat_o, lon_o)
                dest_node = _nearest_node_fallback(lat_d, lon_d)
            else:
                raise

        route = ox.shortest_path(G, orig_node, dest_node)
        return route
    
    def extract_path(self, G: Any, node_ids: List[int],
                     add_elevation: bool = True) -> LoadedPath:
        """Extract path data from graph and node IDs."""
        if len(node_ids) < 2:
            raise ValueError("Need at least 2 nodes for a path")
        
        # Setup UTM for first node
        first_node = G.nodes[node_ids[0]]
        self._setup_utm(first_node['y'], first_node['x'])
        
        points = []
        total_distance = 0.0
        total_climb = 0.0
        total_descent = 0.0
        
        for i, node_id in enumerate(node_ids):
            node = G.nodes[node_id]
            lat = node['y']
            lon = node['x']
            
            # Convert to local coordinates
            x_m, y_m = self._latlon_to_xy(lat, lon)
            
            # Get elevation if available
            z_m = node.get('elevation', 0.0)
            if z_m is None:
                z_m = 0.0
            
            # Get edge data (for surface info)
            highway = ""
            surface = ""
            speed_limit = None
            
            if i < len(node_ids) - 1:
                next_node_id = node_ids[i + 1]
                if G.has_edge(node_id, next_node_id):
                    edge_data = G.get_edge_data(node_id, next_node_id)
                    if edge_data:
                        # Handle multi-edges
                        if isinstance(edge_data, dict) and 0 in edge_data:
                            edge_data = edge_data[0]
                        
                        highway = edge_data.get('highway', '')
                        if isinstance(highway, list):
                            highway = highway[0]
                        
                        surface = edge_data.get('surface', '')
                        if isinstance(surface, list):
                            surface = surface[0]
                        
                        maxspeed = edge_data.get('maxspeed', None)
                        if maxspeed:
                            try:
                                if isinstance(maxspeed, list):
                                    maxspeed = maxspeed[0]
                                speed_limit = float(maxspeed.replace(' km/h', '').replace(' mph', ''))
                            except (ValueError, AttributeError):
                                pass
            
            # Create point
            pt = LoadedPathPoint(
                lat=lat,
                lon=lon,
                x_m=x_m,
                y_m=y_m,
                z_m=z_m,
                distance_along_m=total_distance,
                osm_highway=highway,
                osm_surface=surface,
                osm_speed_limit_kmh=speed_limit,
                speed_limit_kmh=speed_limit,
                osm_node_id=node_id,
            )
            
            # Infer material
            pt.surface_type = self.tag_mapper.get_material(highway, surface)
            
            # Calculate distance to previous point
            if i > 0:
                prev = points[-1]
                dx = pt.x_m - prev.x_m
                dy = pt.y_m - prev.y_m
                dz = pt.z_m - prev.z_m
                segment_dist = math.sqrt(dx*dx + dy*dy + dz*dz)
                total_distance += segment_dist
                pt.distance_along_m = total_distance
                
                # Calculate grade
                horizontal_dist = math.sqrt(dx*dx + dy*dy)
                if horizontal_dist > 0.1:  # Avoid division by zero
                    pt.grade_angle_rad = math.atan2(dz, horizontal_dist)
                    pt.grade_percent = (dz / horizontal_dist) * 100
                
                # Track elevation change
                if dz > 0:
                    total_climb += dz
                else:
                    total_descent += abs(dz)
            
            points.append(pt)
        
        # Calculate curvature at each point (3-point method)
        self._calculate_curvatures(points)
        
        return LoadedPath(
            points=points,
            total_distance_m=total_distance,
            total_climb_m=total_climb,
            total_descent_m=total_descent,
            source="osmnx"
        )
    
    def _calculate_curvatures(self, points: List[LoadedPathPoint]):
        """Calculate curvature at each point using 3-point circumcircle."""
        n = len(points)
        
        for i in range(n):
            if i == 0 or i == n - 1:
                # Endpoints: use adjacent curvature or 0
                points[i].curvature = 0.0
            else:
                # 3-point Menger curvature
                p1 = (points[i-1].x_m, points[i-1].y_m)
                p2 = (points[i].x_m, points[i].y_m)
                p3 = (points[i+1].x_m, points[i+1].y_m)
                
                curvature = self._menger_curvature(p1, p2, p3)
                points[i].curvature = curvature
        
        # Smooth curvatures to avoid noise
        self._smooth_curvatures(points)
    
    def _menger_curvature(self, p1: Tuple[float, float],
                          p2: Tuple[float, float],
                          p3: Tuple[float, float]) -> float:
        """Calculate Menger curvature from 3 points.
        
        κ = 4 * Area / (|P1-P2| * |P2-P3| * |P3-P1|)
        """
        x1, y1 = p1
        x2, y2 = p2
        x3, y3 = p3
        
        # Signed area of triangle
        area = abs((x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1)) / 2
        
        # Side lengths
        d12 = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        d23 = math.sqrt((x3 - x2)**2 + (y3 - y2)**2)
        d31 = math.sqrt((x1 - x3)**2 + (y1 - y3)**2)
        
        denom = d12 * d23 * d31
        if denom < 1e-10:
            return 0.0
        
        curvature = 4 * area / denom
        return curvature
    
    def _smooth_curvatures(self, points: List[LoadedPathPoint], window: int = 3):
        """Apply simple moving average smoothing to curvatures."""
        n = len(points)
        if n < window:
            return
        
        curvatures = [pt.curvature for pt in points]
        smoothed = curvatures.copy()
        
        half = window // 2
        for i in range(half, n - half):
            total = sum(curvatures[i-half:i+half+1])
            smoothed[i] = total / window
        
        for i, pt in enumerate(points):
            pt.curvature = smoothed[i]


def load_path_from_place(place_name: str,
                         origin: Optional[Tuple[float, float]] = None,
                         destination: Optional[Tuple[float, float]] = None,
                         config_dir: Optional[str] = None) -> LoadedPath:
    """
    Convenience function to load a path from a place name.
    
    Args:
        place_name: OSM place name (e.g., "San Francisco, California")
        origin: (lat, lon) of start point, or None for random
        destination: (lat, lon) of end point, or None for random
        config_dir: Path to config directory for tag mapping
    
    Returns:
        LoadedPath with all points and metadata
    """
    loader = OSMLoader(config_dir)
    
    # Get graph
    G = loader.graph_from_place(place_name)
    
    # Get nodes
    nodes = list(G.nodes())
    
    if origin is None or destination is None:
        # Use first and last nodes
        origin_node = nodes[0]
        dest_node = nodes[min(len(nodes) - 1, 100)]  # Not too far
        route = loader.get_shortest_path(
            G,
            (G.nodes[origin_node]['y'], G.nodes[origin_node]['x']),
            (G.nodes[dest_node]['y'], G.nodes[dest_node]['x'])
        )
    else:
        route = loader.get_shortest_path(G, origin, destination)
    
    return loader.extract_path(G, route)


def load_path_from_bbox(north: float, south: float,
                        east: float, west: float,
                        origin: Tuple[float, float],
                        destination: Tuple[float, float],
                        config_dir: Optional[str] = None) -> LoadedPath:
    """Load a path from a bounding box."""
    loader = OSMLoader(config_dir)
    G = loader.graph_from_bbox(north, south, east, west)
    route = loader.get_shortest_path(G, origin, destination)
    return loader.extract_path(G, route)


def create_simple_path(coordinates: List[Tuple[float, float, float]],
                       surfaces: Optional[List[str]] = None) -> LoadedPath:
    """
    Create a path from simple coordinates without OSM.
    
    Args:
        coordinates: List of (x, y, z) tuples in meters
        surfaces: List of surface types (default: "asphalt")
    
    Returns:
        LoadedPath
    """
    points = []
    total_distance = 0.0
    total_climb = 0.0
    total_descent = 0.0
    
    for i, (x, y, z) in enumerate(coordinates):
        surface = surfaces[i] if surfaces and i < len(surfaces) else "asphalt"
        
        pt = LoadedPathPoint(
            x_m=x, y_m=y, z_m=z,
            distance_along_m=total_distance,
            surface_type=surface
        )
        
        if i > 0:
            prev = points[-1]
            dx = x - prev.x_m
            dy = y - prev.y_m
            dz = z - prev.z_m
            segment_dist = math.sqrt(dx*dx + dy*dy + dz*dz)
            total_distance += segment_dist
            pt.distance_along_m = total_distance
            
            # Grade
            horizontal_dist = math.sqrt(dx*dx + dy*dy)
            if horizontal_dist > 0.1:
                pt.grade_angle_rad = math.atan2(dz, horizontal_dist)
                pt.grade_percent = (dz / horizontal_dist) * 100
            
            if dz > 0:
                total_climb += dz
            else:
                total_descent += abs(dz)
        
        points.append(pt)
    
    # Calculate curvatures
    loader = OSMLoader.__new__(OSMLoader)
    loader._calculate_curvatures(points)
    
    return LoadedPath(
        points=points,
        total_distance_m=total_distance,
        total_climb_m=total_climb,
        total_descent_m=total_descent,
        source="manual"
    )
