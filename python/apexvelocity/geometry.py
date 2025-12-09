"""
ApexVelocity Geometry Processing Module.

Provides robust path smoothing, resampling, and curvature calculation
for accurate visualization overlays on satellite imagery.

Key functions:
- smooth_path_geo: Resample and smooth paths in meter space
- recompute_curvature: Calculate curvature using Menger formula
- project_to_utm / project_to_latlon: CRS conversion utilities
"""

from typing import List, Dict, Tuple, Optional, Union

# Python 3.6 compatibility: Literal was added in 3.8
try:
    from typing import Literal
except ImportError:
    # Fallback for Python 3.6/3.7
    Literal = None
import math
from dataclasses import dataclass

# Try to import optional dependencies
try:
    import numpy as np
    _HAS_NUMPY = True
except ImportError:
    _HAS_NUMPY = False
    np = None

try:
    from pyproj import Transformer, CRS
    _HAS_PYPROJ = True
except ImportError:
    _HAS_PYPROJ = False
    Transformer = None
    CRS = None

try:
    from shapely.geometry import LineString, Point
    from shapely.ops import transform as shapely_transform
    _HAS_SHAPELY = True
except ImportError:
    _HAS_SHAPELY = False
    LineString = None
    Point = None

try:
    from scipy.interpolate import CubicSpline
    _HAS_SCIPY = True
except ImportError:
    _HAS_SCIPY = False
    CubicSpline = None


# Type alias for smoothing mode
# Valid values: "catmull_rom", "linestring", "cubic_spline"
if Literal is not None:
    SmoothMode = Literal["catmull_rom", "linestring", "cubic_spline"]
else:
    SmoothMode = str  # Fallback for Python 3.6/3.7


@dataclass
class UTMZone:
    """UTM zone information for CRS conversion."""
    zone_number: int
    hemisphere: str  # "north" or "south"
    epsg_code: int
    
    @classmethod
    def from_latlon(cls, lat: float, lon: float) -> 'UTMZone':
        """Determine UTM zone from lat/lon."""
        zone_number = int((lon + 180) / 6) + 1
        hemisphere = "north" if lat >= 0 else "south"
        
        # Calculate EPSG code
        if hemisphere == "north":
            epsg_code = 32600 + zone_number
        else:
            epsg_code = 32700 + zone_number
        
        return cls(zone_number=zone_number, hemisphere=hemisphere, epsg_code=epsg_code)


class CRSTransformer:
    """Handles coordinate reference system transformations."""
    
    def __init__(self, center_lat: float, center_lon: float):
        """Initialize transformer for a given center point."""
        self.center_lat = center_lat
        self.center_lon = center_lon
        self.utm_zone = UTMZone.from_latlon(center_lat, center_lon)
        
        if _HAS_PYPROJ:
            self._to_utm = Transformer.from_crs(
                "EPSG:4326",  # WGS84
                f"EPSG:{self.utm_zone.epsg_code}",
                always_xy=True  # lon, lat order
            )
            self._to_latlon = Transformer.from_crs(
                f"EPSG:{self.utm_zone.epsg_code}",
                "EPSG:4326",
                always_xy=True
            )
        else:
            self._to_utm = None
            self._to_latlon = None
    
    def latlon_to_meters(self, lat: float, lon: float) -> Tuple[float, float]:
        """Convert lat/lon to local meters (UTM or approximate)."""
        if self._to_utm:
            x, y = self._to_utm.transform(lon, lat)
            return x, y
        else:
            # Approximate conversion (good for small areas)
            y = (lat - self.center_lat) * 111320
            x = (lon - self.center_lon) * 111320 * math.cos(math.radians(lat))
            return x, y
    
    def meters_to_latlon(self, x: float, y: float) -> Tuple[float, float]:
        """Convert local meters back to lat/lon."""
        if self._to_latlon:
            lon, lat = self._to_latlon.transform(x, y)
            return lat, lon
        else:
            # Approximate inverse
            lat = self.center_lat + y / 111320
            lon = self.center_lon + x / (111320 * math.cos(math.radians(lat)))
            return lat, lon


def _compute_arc_length(x_arr: List[float], y_arr: List[float]) -> List[float]:
    """Compute cumulative arc length along a path."""
    s = [0.0]
    for i in range(1, len(x_arr)):
        dx = x_arr[i] - x_arr[i-1]
        dy = y_arr[i] - y_arr[i-1]
        ds = math.sqrt(dx*dx + dy*dy)
        s.append(s[-1] + ds)
    return s


def _catmull_rom_point(p0: Tuple[float, float], p1: Tuple[float, float],
                       p2: Tuple[float, float], p3: Tuple[float, float],
                       t: float, alpha: float = 0.5) -> Tuple[float, float]:
    """
    Compute a point on a Catmull-Rom spline segment.
    
    Uses centripetal parameterization (alpha=0.5) by default for smooth curves.
    """
    def get_t(t_prev: float, p_prev: Tuple, p_curr: Tuple) -> float:
        dx = p_curr[0] - p_prev[0]
        dy = p_curr[1] - p_prev[1]
        dist = (dx*dx + dy*dy) ** (alpha / 2)
        return t_prev + dist
    
    t0 = 0.0
    t1 = get_t(t0, p0, p1)
    t2 = get_t(t1, p1, p2)
    t3 = get_t(t2, p2, p3)
    
    # Clamp to segment [t1, t2]
    t_actual = t1 + t * (t2 - t1)
    
    # Catmull-Rom formula
    def lerp(a: Tuple, b: Tuple, ta: float, tb: float, tc: float) -> Tuple[float, float]:
        if abs(tb - ta) < 1e-10:
            return a
        ratio = (tc - ta) / (tb - ta)
        return (a[0] + ratio * (b[0] - a[0]), a[1] + ratio * (b[1] - a[1]))
    
    A1 = lerp(p0, p1, t0, t1, t_actual)
    A2 = lerp(p1, p2, t1, t2, t_actual)
    A3 = lerp(p2, p3, t2, t3, t_actual)
    
    B1 = lerp(A1, A2, t0, t2, t_actual)
    B2 = lerp(A2, A3, t1, t3, t_actual)
    
    C = lerp(B1, B2, t1, t2, t_actual)
    
    return C


def _interpolate_catmull_rom(x_arr: List[float], y_arr: List[float],
                              target_spacing: float) -> Tuple[List[float], List[float]]:
    """
    Interpolate path using Catmull-Rom spline.
    
    Returns new x, y arrays with approximately target_spacing between points.
    """
    if len(x_arr) < 2:
        return x_arr.copy(), y_arr.copy()
    
    # Compute arc length
    s_arr = _compute_arc_length(x_arr, y_arr)
    total_length = s_arr[-1]
    
    if total_length < target_spacing:
        return x_arr.copy(), y_arr.copy()
    
    # Number of output points
    n_out = max(2, int(total_length / target_spacing) + 1)
    
    # Generate output arc length positions
    s_out = [i * target_spacing for i in range(n_out)]
    if s_out[-1] < total_length:
        s_out.append(total_length)
    
    # Build control points with duplicated endpoints for Catmull-Rom
    points = list(zip(x_arr, y_arr))
    
    # Duplicate first and last points
    pts_extended = [points[0]] + points + [points[-1]]
    s_extended = [s_arr[0] - (s_arr[1] - s_arr[0])] + s_arr + [s_arr[-1] + (s_arr[-1] - s_arr[-2])]
    
    x_new, y_new = [], []
    
    for s_target in s_out:
        # Find which segment we're in
        seg_idx = 0
        for i in range(1, len(s_arr)):
            if s_arr[i] >= s_target:
                seg_idx = i - 1
                break
            seg_idx = i - 1
        
        # Clamp to valid range
        seg_idx = max(0, min(seg_idx, len(points) - 2))
        
        # Get local t parameter [0, 1] within segment
        s_start = s_arr[seg_idx]
        s_end = s_arr[seg_idx + 1]
        if abs(s_end - s_start) < 1e-10:
            t_local = 0.0
        else:
            t_local = (s_target - s_start) / (s_end - s_start)
            t_local = max(0.0, min(1.0, t_local))
        
        # Get 4 control points for Catmull-Rom
        # pts_extended has extra point at start, so indices shift by 1
        p0 = pts_extended[seg_idx]      # seg_idx + 0 in extended
        p1 = pts_extended[seg_idx + 1]  # seg_idx + 1 in extended
        p2 = pts_extended[seg_idx + 2]  # seg_idx + 2 in extended
        p3 = pts_extended[seg_idx + 3] if seg_idx + 3 < len(pts_extended) else pts_extended[-1]
        
        x_pt, y_pt = _catmull_rom_point(p0, p1, p2, p3, t_local)
        x_new.append(x_pt)
        y_new.append(y_pt)
    
    return x_new, y_new


def _interpolate_linestring(x_arr: List[float], y_arr: List[float],
                            target_spacing: float) -> Tuple[List[float], List[float]]:
    """
    Interpolate path using Shapely LineString.
    
    Returns new x, y arrays with approximately target_spacing between points.
    """
    if not _HAS_SHAPELY:
        # Fallback to linear interpolation
        return _interpolate_linear(x_arr, y_arr, target_spacing)
    
    coords = list(zip(x_arr, y_arr))
    line = LineString(coords)
    
    total_length = line.length
    if total_length < target_spacing:
        return x_arr.copy(), y_arr.copy()
    
    n_out = max(2, int(total_length / target_spacing) + 1)
    
    x_new, y_new = [], []
    for i in range(n_out):
        s = i * target_spacing
        if s > total_length:
            s = total_length
        pt = line.interpolate(s)
        x_new.append(pt.x)
        y_new.append(pt.y)
    
    # Ensure we include the endpoint
    if len(x_new) > 0:
        end_pt = line.interpolate(total_length)
        if abs(x_new[-1] - end_pt.x) > 0.01 or abs(y_new[-1] - end_pt.y) > 0.01:
            x_new.append(end_pt.x)
            y_new.append(end_pt.y)
    
    return x_new, y_new


def _interpolate_cubic_spline(x_arr: List[float], y_arr: List[float],
                               target_spacing: float) -> Tuple[List[float], List[float]]:
    """
    Interpolate path using SciPy cubic spline.
    """
    if not _HAS_SCIPY or not _HAS_NUMPY:
        return _interpolate_catmull_rom(x_arr, y_arr, target_spacing)
    
    s_arr = np.array(_compute_arc_length(x_arr, y_arr))
    total_length = s_arr[-1]
    
    if total_length < target_spacing:
        return list(x_arr), list(y_arr)
    
    # Build cubic splines for x(s) and y(s)
    cs_x = CubicSpline(s_arr, x_arr, bc_type='natural')
    cs_y = CubicSpline(s_arr, y_arr, bc_type='natural')
    
    n_out = max(2, int(total_length / target_spacing) + 1)
    s_new = np.linspace(0, total_length, n_out)
    
    x_new = cs_x(s_new).tolist()
    y_new = cs_y(s_new).tolist()
    
    return x_new, y_new


def _interpolate_linear(x_arr: List[float], y_arr: List[float],
                        target_spacing: float) -> Tuple[List[float], List[float]]:
    """
    Simple linear interpolation fallback.
    """
    s_arr = _compute_arc_length(x_arr, y_arr)
    total_length = s_arr[-1]
    
    if total_length < target_spacing:
        return x_arr.copy(), y_arr.copy()
    
    n_out = max(2, int(total_length / target_spacing) + 1)
    
    x_new, y_new = [], []
    
    for i in range(n_out):
        s_target = i * target_spacing
        if s_target > total_length:
            s_target = total_length
        
        # Find segment
        seg_idx = 0
        for j in range(1, len(s_arr)):
            if s_arr[j] >= s_target:
                seg_idx = j - 1
                break
            seg_idx = j - 1
        
        seg_idx = max(0, min(seg_idx, len(x_arr) - 2))
        
        # Linear interpolation within segment
        s_start = s_arr[seg_idx]
        s_end = s_arr[seg_idx + 1]
        if abs(s_end - s_start) < 1e-10:
            t = 0.0
        else:
            t = (s_target - s_start) / (s_end - s_start)
        
        x_pt = x_arr[seg_idx] + t * (x_arr[seg_idx + 1] - x_arr[seg_idx])
        y_pt = y_arr[seg_idx] + t * (y_arr[seg_idx + 1] - y_arr[seg_idx])
        x_new.append(x_pt)
        y_new.append(y_pt)
    
    # Ensure endpoint
    if len(x_new) > 0 and (abs(x_new[-1] - x_arr[-1]) > 0.01 or abs(y_new[-1] - y_arr[-1]) > 0.01):
        x_new.append(x_arr[-1])
        y_new.append(y_arr[-1])
    
    return x_new, y_new


def _interpolate_metadata(old_points: List[Dict], old_s: List[float],
                          new_s: List[float], 
                          keys_to_interp: List[str] = None) -> List[Dict]:
    """
    Interpolate metadata from old points to new arc-length positions.
    
    Uses nearest-neighbor for string fields, linear interpolation for numeric.
    """
    if keys_to_interp is None:
        keys_to_interp = ['surface_type', 'elevation_m', 'z_m', 'grade_percent',
                          'road_type', 'speed_limit_kmh', 'lane_count']
    
    n_new = len(new_s)
    new_meta = [{} for _ in range(n_new)]
    
    for i, s_target in enumerate(new_s):
        # Find nearest old point
        min_dist = float('inf')
        nearest_idx = 0
        for j, s_old in enumerate(old_s):
            dist = abs(s_old - s_target)
            if dist < min_dist:
                min_dist = dist
                nearest_idx = j
        
        # Copy metadata from nearest point
        old_pt = old_points[nearest_idx]
        for key in keys_to_interp:
            if key in old_pt:
                new_meta[i][key] = old_pt[key]
    
    return new_meta


def smooth_path_geo(
    points: List[Dict],
    target_spacing_m: float = 5.0,
    mode: SmoothMode = "catmull_rom",
) -> List[Dict]:
    """
    Smooth and resample a path at regular intervals.
    
    Input: List of dicts with at least 'lat', 'lon', and optional metadata.
    Output: New list of dicts, resampled along a smooth curve.
    
    Algorithm:
    1. Project lat/lon to local meters (UTM or approximate)
    2. Interpolate using specified mode (catmull_rom, linestring, cubic_spline)
    3. Generate new sample points at target_spacing_m intervals
    4. Project back to lat/lon
    5. Interpolate/copy metadata from original points
    
    Args:
        points: List of dicts with 'lat', 'lon' keys (degrees)
        target_spacing_m: Target distance between output points (meters)
        mode: Interpolation method
        
    Returns:
        New list of dicts with smoothed path
    """
    if len(points) < 2:
        return points.copy() if points else []
    
    # Get center for CRS transformation
    lats = [p['lat'] for p in points]
    lons = [p['lon'] for p in points]
    center_lat = sum(lats) / len(lats)
    center_lon = sum(lons) / len(lons)
    
    # Create transformer
    transformer = CRSTransformer(center_lat, center_lon)
    
    # Project to meters
    x_m, y_m = [], []
    for p in points:
        x, y = transformer.latlon_to_meters(p['lat'], p['lon'])
        x_m.append(x)
        y_m.append(y)
    
    # Compute original arc length
    old_s = _compute_arc_length(x_m, y_m)
    
    # Interpolate based on mode
    if mode == "catmull_rom":
        x_new, y_new = _interpolate_catmull_rom(x_m, y_m, target_spacing_m)
    elif mode == "linestring":
        x_new, y_new = _interpolate_linestring(x_m, y_m, target_spacing_m)
    elif mode == "cubic_spline":
        x_new, y_new = _interpolate_cubic_spline(x_m, y_m, target_spacing_m)
    else:
        x_new, y_new = _interpolate_catmull_rom(x_m, y_m, target_spacing_m)
    
    # Compute new arc length for metadata interpolation
    new_s = _compute_arc_length(x_new, y_new)
    
    # Interpolate metadata
    new_meta = _interpolate_metadata(points, old_s, new_s)
    
    # Project back to lat/lon and build output
    result = []
    cumulative_dist = 0.0
    
    for i, (x, y) in enumerate(zip(x_new, y_new)):
        lat, lon = transformer.meters_to_latlon(x, y)
        
        # Compute distance from previous point
        if i > 0:
            dx = x - x_new[i-1]
            dy = y - y_new[i-1]
            cumulative_dist += math.sqrt(dx*dx + dy*dy)
        
        pt = {
            'lat': lat,
            'lon': lon,
            'x_m': x,
            'y_m': y,
            'distance_along_m': cumulative_dist,
            'curvature': 0.0,  # Will be computed by recompute_curvature
        }
        
        # Add interpolated metadata
        if i < len(new_meta):
            pt.update(new_meta[i])
        
        # Set defaults for required fields
        pt.setdefault('z_m', 0.0)
        pt.setdefault('elevation_m', pt.get('z_m', 0.0))
        pt.setdefault('surface_type', 'asphalt')
        pt.setdefault('speed_mps', 0.0)
        pt.setdefault('v_profile', 0.0)
        pt.setdefault('energy_joules', 0.0)
        pt.setdefault('energy_kwh', 0.0)
        
        result.append(pt)
    
    return result


def _point_to_segment_distance_m(px: float, py: float,
                                 x1: float, y1: float,
                                 x2: float, y2: float) -> float:
    """Compute shortest distance from a point to a line segment in meters."""
    vx, vy = x2 - x1, y2 - y1
    wx, wy = px - x1, py - y1
    seg_len2 = vx * vx + vy * vy
    if seg_len2 <= 1e-12:
        # Degenerate segment
        dx, dy = px - x1, py - y1
        return math.hypot(dx, dy)
    t = (wx * vx + wy * vy) / seg_len2
    t = max(0.0, min(1.0, t))
    projx, projy = x1 + t * vx, y1 + t * vy
    return math.hypot(px - projx, py - projy)


def validate_path_alignment(raw_points: List[Dict],
                            smoothed_points: List[Dict],
                            max_offset_m: float = 3.0) -> bool:
    """
    Returns True if every smoothed point lies within max_offset_m of the
    original polyline defined by raw_points.

    - Works entirely in a local metric CRS to avoid degree distortions.
    - Uses shapely if available; falls back to point-to-segment distances.
    """
    if not raw_points or not smoothed_points or len(raw_points) < 2:
        return True

    # Build transformer around combined centroid for stability
    lats = [p['lat'] for p in raw_points]
    lons = [p['lon'] for p in raw_points]
    center_lat = sum(lats) / len(lats)
    center_lon = sum(lons) / len(lons)
    tfm = CRSTransformer(center_lat, center_lon)

    raw_xy = [tfm.latlon_to_meters(p['lat'], p['lon']) for p in raw_points]
    smooth_xy = [tfm.latlon_to_meters(p['lat'], p['lon']) for p in smoothed_points]

    # If shapely available, use LineString distance
    if _HAS_SHAPELY:
        try:
            line = LineString(raw_xy)
            max_dist = 0.0
            for (x, y) in smooth_xy:
                d = line.distance(Point(x, y))
                if d > max_dist:
                    max_dist = d
                    if max_dist > max_offset_m:
                        return False
            return True
        except Exception:
            # Fall back to manual distance
            pass

    # Manual fallback: compute min distance to each segment
    max_dist = 0.0
    for (px, py) in smooth_xy:
        mind = float('inf')
        for i in range(1, len(raw_xy)):
            x1, y1 = raw_xy[i - 1]
            x2, y2 = raw_xy[i]
            d = _point_to_segment_distance_m(px, py, x1, y1, x2, y2)
            if d < mind:
                mind = d
                if mind <= 0:
                    break
        if mind > max_dist:
            max_dist = mind
            if max_dist > max_offset_m:
                return False
    return True


def _menger_curvature(p1: Tuple[float, float], p2: Tuple[float, float],
                      p3: Tuple[float, float]) -> float:
    """
    Compute Menger curvature from 3 points.
    
    κ = 4 * Area / (|P1-P2| * |P2-P3| * |P3-P1|)
    
    Returns curvature in 1/meters.
    """
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3
    
    # Signed area of triangle (using cross product)
    area = abs((x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1)) / 2
    
    # Side lengths
    d12 = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    d23 = math.sqrt((x3 - x2)**2 + (y3 - y2)**2)
    d31 = math.sqrt((x1 - x3)**2 + (y1 - y3)**2)
    
    denom = d12 * d23 * d31
    if denom < 1e-10:
        return 0.0
    
    return 4 * area / denom


def recompute_curvature(points: List[Dict], smooth_window: int = 3) -> List[Dict]:
    """
    Recompute curvature at each point using 3-point Menger formula.
    
    Works in meter space (x_m, y_m) if available, otherwise projects from lat/lon.
    
    Args:
        points: List of path point dicts
        smooth_window: Window size for moving average smoothing (odd number)
        
    Returns:
        Same list with updated 'curvature' field
    """
    if len(points) < 3:
        for p in points:
            p['curvature'] = 0.0
        return points
    
    # Get or compute meter coordinates
    has_meters = 'x_m' in points[0] and 'y_m' in points[0]
    
    if has_meters:
        x_arr = [p['x_m'] for p in points]
        y_arr = [p['y_m'] for p in points]
    else:
        # Project from lat/lon
        lats = [p['lat'] for p in points]
        lons = [p['lon'] for p in points]
        center_lat = sum(lats) / len(lats)
        center_lon = sum(lons) / len(lons)
        
        transformer = CRSTransformer(center_lat, center_lon)
        x_arr, y_arr = [], []
        for p in points:
            x, y = transformer.latlon_to_meters(p['lat'], p['lon'])
            x_arr.append(x)
            y_arr.append(y)
            p['x_m'] = x
            p['y_m'] = y
    
    n = len(points)
    curvatures = []
    
    # Compute raw curvature at each point
    for i in range(n):
        if i == 0:
            # Use curvature of second point
            if n > 2:
                k = _menger_curvature(
                    (x_arr[0], y_arr[0]),
                    (x_arr[1], y_arr[1]),
                    (x_arr[2], y_arr[2])
                )
            else:
                k = 0.0
        elif i == n - 1:
            # Use curvature of second-to-last point
            if n > 2:
                k = _menger_curvature(
                    (x_arr[n-3], y_arr[n-3]),
                    (x_arr[n-2], y_arr[n-2]),
                    (x_arr[n-1], y_arr[n-1])
                )
            else:
                k = 0.0
        else:
            k = _menger_curvature(
                (x_arr[i-1], y_arr[i-1]),
                (x_arr[i], y_arr[i]),
                (x_arr[i+1], y_arr[i+1])
            )
        curvatures.append(k)
    
    # Apply smoothing
    if smooth_window > 1 and n >= smooth_window:
        half = smooth_window // 2
        smoothed = curvatures.copy()
        
        for i in range(half, n - half):
            window = curvatures[i - half : i + half + 1]
            smoothed[i] = sum(window) / len(window)
        
        curvatures = smoothed
    
    # Update points
    for i, p in enumerate(points):
        p['curvature'] = curvatures[i]
        
        # Also compute curve radius for convenience
        if curvatures[i] > 1e-6:
            p['curve_radius_m'] = 1.0 / curvatures[i]
        else:
            p['curve_radius_m'] = float('inf')
    
    return points


def get_path_stats(points: List[Dict]) -> Dict:
    """
    Compute statistics about a path.
    
    Returns dict with:
    - total_distance_m
    - point_count
    - avg_spacing_m
    - min_spacing_m
    - max_spacing_m
    - avg_curvature
    - max_curvature
    - bounding_box
    """
    if not points:
        return {'total_distance_m': 0, 'point_count': 0}
    
    n = len(points)
    
    # Distance
    total_dist = points[-1].get('distance_along_m', 0)
    
    # Spacing
    spacings = []
    for i in range(1, n):
        d = points[i].get('distance_along_m', 0) - points[i-1].get('distance_along_m', 0)
        if d > 0:
            spacings.append(d)
    
    # Curvature
    curvatures = [p.get('curvature', 0) for p in points]
    
    # Bounding box
    lats = [p.get('lat', 0) for p in points]
    lons = [p.get('lon', 0) for p in points]
    
    return {
        'total_distance_m': total_dist,
        'total_distance_km': total_dist / 1000,
        'point_count': n,
        'avg_spacing_m': sum(spacings) / len(spacings) if spacings else 0,
        'min_spacing_m': min(spacings) if spacings else 0,
        'max_spacing_m': max(spacings) if spacings else 0,
        'avg_curvature': sum(curvatures) / n if n > 0 else 0,
        'max_curvature': max(curvatures) if curvatures else 0,
        'bounding_box': {
            'min_lat': min(lats), 'max_lat': max(lats),
            'min_lon': min(lons), 'max_lon': max(lons),
        }
    }
