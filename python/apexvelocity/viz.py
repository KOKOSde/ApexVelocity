"""
ApexVelocity Professional Visualization Module.

Creates interactive 3D map visualizations with:
- Optional satellite basemaps (requires Mapbox API key)
- Free OSM/dark basemaps (no API key needed)
- 3D extruded path tubes showing lateral G-force
- Rich tooltips with physics data for AV training
"""

from typing import List, Dict, Optional, Tuple, Any, Union
import math
import os
import sys
import warnings

try:
    import pydeck as pdk

    _HAS_PYDECK = True
except ImportError:
    _HAS_PYDECK = False
    pdk = None

try:
    from .analysis import analyze_profile, ProfileAnalysis, SegmentFeatures

    _HAS_ANALYSIS = True
except ImportError:
    _HAS_ANALYSIS = False


# Map style configurations
# "osm" and "dark" work without any API key
# "satellite" requires MAPBOX_API_KEY
MAP_STYLES = {
    # Mapbox styles (require API key)
    "satellite": "mapbox://styles/mapbox/satellite-streets-v12",
    "satellite-plain": "mapbox://styles/mapbox/satellite-v9",
    "mapbox-dark": "mapbox://styles/mapbox/dark-v11",
    "mapbox-light": "mapbox://styles/mapbox/light-v11",
    "streets": "mapbox://styles/mapbox/streets-v12",
    "outdoors": "mapbox://styles/mapbox/outdoors-v12",
    # Free styles (no API key needed)
    "osm": None,  # Will use TileLayer
    "dark": None,  # Will use dark TileLayer
    "light": None,  # Will use light TileLayer
}

# Free tile providers (no API key needed)
TILE_PROVIDERS = {
    "osm": "https://tile.openstreetmap.org/{z}/{x}/{y}.png",
    "dark": "https://cartodb-basemaps-a.global.ssl.fastly.net/dark_all/{z}/{x}/{y}.png",
    "light": "https://cartodb-basemaps-a.global.ssl.fastly.net/light_all/{z}/{x}/{y}.png",
}

# Color palettes (RGBA)
# Each palette defines low/mid/high colors for gradient interpolation
COLOR_PALETTES = {
    "speed": {
        "low": [220, 53, 69, 255],  # Red = slow
        "mid": [255, 193, 7, 255],  # Yellow = medium
        "high": [40, 167, 69, 255],  # Green = fast
    },
    "energy": {
        "low": [40, 167, 69, 255],  # Green = efficient
        "mid": [255, 193, 7, 255],  # Yellow = moderate
        "high": [220, 53, 69, 255],  # Red = high consumption
    },
    "gforce": {
        "low": [52, 152, 219, 255],  # Blue = low G
        "mid": [155, 89, 182, 255],  # Purple = medium G
        "high": [220, 53, 69, 255],  # Red = high G
    },
    "lateral_g": {
        "low": [52, 152, 219, 255],  # Blue = low lateral G
        "mid": [155, 89, 182, 255],  # Purple = medium
        "high": [220, 53, 69, 255],  # Red = high lateral G
    },
    "longitudinal_g": {
        "low": [40, 167, 69, 255],  # Green = acceleration
        "mid": [255, 193, 7, 255],  # Yellow = coast
        "high": [220, 53, 69, 255],  # Red = braking
    },
    "friction": {
        "low": [40, 167, 69, 255],  # Green = low usage
        "mid": [255, 193, 7, 255],  # Yellow = moderate
        "high": [220, 53, 69, 255],  # Red = near limit
    },
    "comfort": {
        "low": [40, 167, 69, 255],  # Green = comfortable
        "mid": [255, 193, 7, 255],  # Yellow = moderate
        "high": [220, 53, 69, 255],  # Red = uncomfortable
    },
    "difficulty": {
        "low": [52, 152, 219, 255],  # Blue = easy
        "mid": [155, 89, 182, 255],  # Purple = moderate
        "high": [220, 53, 69, 255],  # Red = difficult
    },
    "safety_margin": {
        "low": [220, 53, 69, 255],  # Red = low safety margin (dangerous)
        "mid": [255, 193, 7, 255],  # Yellow = moderate
        "high": [40, 167, 69, 255],  # Green = high safety margin (safe)
    },
    "regen": {
        "low": [128, 128, 128, 255],  # Gray = no regen
        "mid": [52, 152, 219, 255],  # Blue = some regen
        "high": [40, 167, 69, 255],  # Green = high regen potential
    },
    "action": {
        "BRAKE": [220, 53, 69, 255],
        "EASE_OFF": [255, 140, 0, 255],
        "EASE OFF": [255, 140, 0, 255],
        "COAST": [52, 152, 219, 255],
        "ACCELERATE": [40, 167, 69, 255],
        "HOLD": [128, 128, 128, 255],
        "SMOOTH": [155, 89, 182, 255],
    },
}

# Map color_by names to segment data fields
COLOR_BY_FIELDS = {
    "speed": "speed_kmh",
    "energy": "energy_kwh_per_km",
    "lateral_g": "lateral_g",
    "longitudinal_g": "longitudinal_g",
    "gforce": "lateral_g",  # Alias
    "friction": "friction_usage",
    "comfort": "comfort_score",
    "difficulty": "difficulty_score",
    "safety_margin": "safety_margin",
    "regen": "regen_potential_j",
    "action": "recommended_action",
}


def _check_pydeck():
    """Ensure pydeck is available."""
    if not _HAS_PYDECK:
        raise ImportError(
            "pydeck is required for visualization.\n" "Install with: pip install pydeck"
        )


def _get_mapbox_token() -> Optional[str]:
    """Get Mapbox API token from environment (optional)."""
    return os.environ.get("MAPBOX_API_KEY") or os.environ.get("MAPBOX_ACCESS_TOKEN")


def _get_basemap_layer(
    style: str,
) -> Tuple[Optional[str], Optional[Any], Optional[str]]:
    """
    Get basemap configuration based on style.

    Returns:
        (map_style, tile_layer, mapbox_token)
        - map_style: Mapbox style URL or None
        - tile_layer: TileLayer for OSM/CARTO or None
        - mapbox_token: Token if needed, else None
    """
    mapbox_token = _get_mapbox_token()

    # Check if this is a Mapbox style that needs a token
    mapbox_styles = [
        "satellite",
        "satellite-plain",
        "mapbox-dark",
        "mapbox-light",
        "streets",
        "outdoors",
    ]

    if style in mapbox_styles:
        if mapbox_token:
            # Use Mapbox style
            return MAP_STYLES[style], None, mapbox_token
        else:
            # Fallback to free alternative
            fallback = "dark" if "dark" in style else "osm"
            print(
                f"⚠️  MAPBOX_API_KEY not set. Using free '{fallback}' style instead of '{style}'.",
                file=sys.stderr,
            )
            print(
                "   For satellite imagery, set: export MAPBOX_API_KEY='your_token'",
                file=sys.stderr,
            )
            print(
                "   Get a free token at: https://account.mapbox.com/access-tokens/\n",
                file=sys.stderr,
            )
            style = fallback

    # Free tile-based styles
    if style in TILE_PROVIDERS:
        tile_url = TILE_PROVIDERS[style]
        tile_layer = pdk.Layer(
            "TileLayer",
            data=tile_url,
            min_zoom=0,
            max_zoom=19,
            tile_size=256,
            opacity=1.0,
        )
        return None, tile_layer, None

    # Default fallback
    tile_layer = pdk.Layer(
        "TileLayer",
        data=TILE_PROVIDERS["osm"],
        min_zoom=0,
        max_zoom=19,
        tile_size=256,
    )
    return None, tile_layer, None


def _interpolate_color(t: float, palette: Dict[str, List[int]]) -> List[int]:
    """Interpolate between colors based on t (0-1)."""
    t = max(0, min(1, t))

    if t < 0.5:
        t2 = t * 2
        c1, c2 = palette["low"], palette["mid"]
    else:
        t2 = (t - 0.5) * 2
        c1, c2 = palette["mid"], palette["high"]

    return [
        int(c1[0] + t2 * (c2[0] - c1[0])),
        int(c1[1] + t2 * (c2[1] - c1[1])),
        int(c1[2] + t2 * (c2[2] - c1[2])),
        int(c1[3] + t2 * (c2[3] - c1[3])),
    ]


def _get_color_for_segment(seg: Dict, mode: str, stats: Dict) -> List[int]:
    """Get color for a segment based on visualization mode (color_by)."""

    # Handle action mode separately (categorical, not gradient)
    if mode == "action":
        action = seg.get("recommended_action", "HOLD")
        return COLOR_PALETTES["action"].get(action, COLOR_PALETTES["action"]["HOLD"])

    # Get value and range based on mode
    if mode == "speed":
        val = seg.get("speed_kmh", 0)
        min_v, max_v = stats.get("min_speed", 0), stats.get("max_speed", 100)
    elif mode == "energy":
        val = seg.get("energy_kwh_per_km", 0)
        min_v, max_v = 0, max(stats.get("max_energy_intensity", 500), 0.001)
    elif mode in ("gforce", "lateral_g"):
        val = seg.get("lateral_g", 0)
        min_v, max_v = 0, max(stats.get("max_lateral_g", 1.0), 0.001)
    elif mode == "longitudinal_g":
        val = abs(seg.get("longitudinal_g", 0))
        min_v, max_v = 0, max(stats.get("max_longitudinal_g", 0.5), 0.001)
    elif mode == "friction":
        val = seg.get("friction_usage", 0)
        min_v, max_v = 0, 1.0
    elif mode == "comfort":
        # Use per-route comfort range so we don't end up solid red when the
        # whole route is merely "bad but not maxed out".
        val = seg.get("comfort_score", 0)
        min_v = stats.get("min_comfort", 0.0)
        max_v = stats.get("max_comfort", 1.0)
    elif mode == "difficulty":
        # Difficulty provided directly by analysis; scale to local min/max.
        val = seg.get("difficulty_score", 0)
        min_v = stats.get("min_difficulty", 0.0)
        max_v = stats.get("max_difficulty", 1.0)
    elif mode == "safety_margin":
        # Safety margin already encoded per segment.
        val = seg.get("safety_margin", 0)
        min_v = stats.get("min_safety_margin", 0.0)
        max_v = stats.get("max_safety_margin", 1.0)
    elif mode == "regen":
        val = seg.get("regen_potential_j", 0)
        min_v = stats.get("min_regen", 0.0)
        max_v = stats.get("max_regen", max(stats.get("max_regen", 0.0), 1.0))
    else:
        # Default to speed
        val = seg.get("speed_kmh", 0)
        min_v, max_v = stats.get("min_speed", 0), stats.get("max_speed", 100)

    # Normalize to [0, 1]
    if max_v > min_v:
        t = (val - min_v) / (max_v - min_v)
    else:
        t = 0.5
    t = max(0.0, min(1.0, t))  # Clamp

    palette = COLOR_PALETTES.get(mode, COLOR_PALETTES["speed"])
    return _interpolate_color(t, palette)


def _compute_view_state(
    lats: List[float], lons: List[float], pitch: float = 45, bearing: float = 0
) -> Any:
    """Compute optimal view state to fit all points."""
    if not lats or not lons:
        return pdk.ViewState(latitude=0, longitude=0, zoom=10)

    center_lat = sum(lats) / len(lats)
    center_lon = sum(lons) / len(lons)

    lat_range = max(lats) - min(lats)
    lon_range = max(lons) - min(lons)
    max_range = max(lat_range, lon_range)

    if max_range > 0.5:
        zoom = 9
    elif max_range > 0.2:
        zoom = 10
    elif max_range > 0.1:
        zoom = 11
    elif max_range > 0.05:
        zoom = 12
    elif max_range > 0.02:
        zoom = 13
    elif max_range > 0.01:
        zoom = 14
    elif max_range > 0.005:
        zoom = 15
    else:
        zoom = 16

    return pdk.ViewState(
        latitude=center_lat,
        longitude=center_lon,
        zoom=zoom,
        pitch=pitch,
        bearing=bearing,
    )


def _build_path_data(
    segments: List[Dict],
    mode: str = "speed",
    lateral_g_scale: float = 30.0,
    path_width_m: float = 8.0,
) -> Tuple[List[Dict], Dict]:
    """Build path data for visualization."""
    if len(segments) < 2:
        return [], {}

    speeds = [s.get("speed_kmh", 0) for s in segments if s.get("speed_kmh", 0) > 0]
    lateral_gs = [s.get("lateral_g", 0) for s in segments]
    friction_usages = [s.get("friction_usage", 0) for s in segments]
    energy_intensities = [s.get("energy_kwh_per_km", 0) for s in segments]
    comfort_scores = [
        s.get("comfort_score", 0)
        for s in segments
        if s.get("comfort_score") is not None
    ]
    difficulty_scores = [
        s.get("difficulty_score", 0)
        for s in segments
        if s.get("difficulty_score") is not None
    ]
    safety_margins = [
        s.get("safety_margin", 0)
        for s in segments
        if s.get("safety_margin") is not None
    ]
    regen_values = [
        s.get("regen_potential_j", 0) for s in segments if s.get("regen_potential_j")
    ]

    stats = {
        "min_speed": min(speeds) if speeds else 0,
        "max_speed": max(speeds) if speeds else 100,
        "avg_speed": sum(speeds) / len(speeds) if speeds else 0,
        "max_lateral_g": max(lateral_gs) if lateral_gs else 1.0,
        "avg_lateral_g": sum(lateral_gs) / len(lateral_gs) if lateral_gs else 0,
        "max_friction_usage": max(friction_usages) if friction_usages else 1.0,
        "max_energy_intensity": max(e for e in energy_intensities if e > 0)
        if any(e > 0 for e in energy_intensities)
        else 500,
        # Dynamic ranges for non-speed metrics so the legend/colormap uses
        # the full range of values present on this route instead of always
        # saturating to red.
        "min_comfort": min(comfort_scores) if comfort_scores else 0.0,
        "max_comfort": max(comfort_scores) if comfort_scores else 1.0,
        "min_difficulty": min(difficulty_scores) if difficulty_scores else 0.0,
        "max_difficulty": max(difficulty_scores) if difficulty_scores else 1.0,
        "min_safety_margin": min(safety_margins) if safety_margins else 0.0,
        "max_safety_margin": max(safety_margins) if safety_margins else 1.0,
        "min_regen": min(regen_values) if regen_values else 0.0,
        "max_regen": max(regen_values) if regen_values else 0.0,
    }

    path_data = []

    for i in range(len(segments) - 1):
        seg1 = segments[i]
        seg2 = segments[i + 1]

        lat1, lon1 = seg1.get("lat", 0), seg1.get("lon", 0)
        lat2, lon2 = seg2.get("lat", 0), seg2.get("lon", 0)

        if abs(lat1) < 0.001 and abs(lon1) < 0.001:
            continue
        if abs(lat2) < 0.001 and abs(lon2) < 0.001:
            continue

        avg_seg = {
            # Speed
            "speed_kmh": (seg1.get("speed_kmh", 0) + seg2.get("speed_kmh", 0)) / 2,
            "physics_limit_kmh": (
                seg1.get("physics_limit_kmh", 100) + seg2.get("physics_limit_kmh", 100)
            )
            / 2,
            "speed_ratio": (seg1.get("speed_ratio", 0) + seg2.get("speed_ratio", 0))
            / 2,
            "speed_limit_kmh": seg1.get("speed_limit_kmh"),
            "speed_limit_margin_kmh": seg1.get("speed_limit_margin_kmh"),
            "target_speed_kmh": seg1.get("target_speed_kmh", 0),
            # Dynamics
            "lateral_g": (seg1.get("lateral_g", 0) + seg2.get("lateral_g", 0)) / 2,
            "longitudinal_g": (
                seg1.get("longitudinal_g", 0) + seg2.get("longitudinal_g", 0)
            )
            / 2,
            "total_g": (seg1.get("total_g", 0) + seg2.get("total_g", 0)) / 2,
            "lateral_accel_mps2": seg1.get("lateral_accel_mps2", 0),
            "longitudinal_accel_mps2": seg1.get("longitudinal_accel_mps2", 0),
            "longitudinal_jerk_mps3": seg1.get("longitudinal_jerk_mps3", 0),
            "lateral_jerk_mps3": seg1.get("lateral_jerk_mps3", 0),
            "jerk_magnitude_mps3": seg1.get("jerk_magnitude_mps3", 0),
            # Friction
            "friction_usage": (
                seg1.get("friction_usage", 0) + seg2.get("friction_usage", 0)
            )
            / 2,
            "friction_mu": seg1.get("friction_mu", 0.9),
            # Geometry
            "curvature_1pm": seg1.get("curvature_1pm", 0),
            "curvature_radius_m": seg1.get(
                "curvature_radius_m", seg1.get("curve_radius_m", float("inf"))
            ),
            "dcurvature_ds": seg1.get("dcurvature_ds", 0),
            "grade_percent": seg1.get("grade_percent", 0),
            "turn_type": seg1.get("turn_type", "straight"),
            "turn_angle_deg": seg1.get("turn_angle_deg", 0),
            # Semantics
            "surface_type": seg1.get("surface_type", "asphalt"),
            "road_type": seg1.get("road_type"),
            "lane_count": seg1.get("lane_count"),
            "is_intersection": seg1.get("is_intersection", False),
            "distance_to_intersection_m": seg1.get("distance_to_intersection_m"),
            # Energy
            "energy_kwh_per_km": (
                seg1.get("energy_kwh_per_km", 0) + seg2.get("energy_kwh_per_km", 0)
            )
            / 2,
            "energy_per_meter_j": seg1.get("energy_per_meter_j", 0),
            "regen_potential_j": seg1.get("regen_potential_j", 0),
            "coasting_opportunity": seg1.get("coasting_opportunity", False),
            # Comfort
            "comfort_score": seg1.get("comfort_score", 0),
            "is_uncomfortable": seg1.get("is_uncomfortable", False),
            # Action
            "recommended_action": seg1.get("recommended_action", "HOLD"),
            "action_confidence": seg1.get("action_confidence", 1.0),
            "action_reason": seg1.get("action_reason", ""),
            "time_to_next_brake_s": seg1.get("time_to_next_brake_s"),
        }

        # Derive a per-segment safety margin if not explicitly provided.
        # High friction usage → low safety margin; low usage → high margin.
        if "safety_margin" not in avg_seg:
            fu = avg_seg.get("friction_usage", 0.0)
            # Clamp to [0, 1] and invert.
            fu = max(0.0, min(1.0, float(fu)))
            avg_seg["safety_margin"] = 1.0 - fu

        color = _get_color_for_segment(avg_seg, mode, stats)

        # Keep paths flat on the ground (elevation = 0) for accurate overlay
        # Use color intensity to show G-force instead of 3D extrusion
        elevation = 0

        # Format radius for display
        radius = avg_seg["curvature_radius_m"]
        radius_display = (
            f"{radius:.0f}"
            if isinstance(radius, (int, float)) and radius < 10000
            else "∞"
        )

        path_segment = {
            "path": [[lon1, lat1, elevation], [lon2, lat2, elevation]],
            "color": color,
            "width": path_width_m,
            "elevation": elevation,
            # Speed
            "speed_kmh": round(avg_seg["speed_kmh"], 1),
            "physics_limit_kmh": round(avg_seg["physics_limit_kmh"], 1),
            "speed_ratio": round(avg_seg["speed_ratio"] * 100, 0),
            "speed_limit_kmh": avg_seg.get("speed_limit_kmh") or "N/A",
            "speed_margin_kmh": f"{avg_seg['speed_limit_margin_kmh']:.1f}"
            if avg_seg.get("speed_limit_margin_kmh")
            else "N/A",
            "target_speed_kmh": round(avg_seg.get("target_speed_kmh", 0), 1),
            # Dynamics
            "lateral_g": round(avg_seg["lateral_g"], 3),
            "longitudinal_g": round(avg_seg["longitudinal_g"], 3),
            "total_g": round(avg_seg["total_g"], 3),
            "lateral_accel_mps2": round(avg_seg.get("lateral_accel_mps2", 0), 2),
            "longitudinal_accel_mps2": round(
                avg_seg.get("longitudinal_accel_mps2", 0), 2
            ),
            "longitudinal_jerk_mps3": round(
                avg_seg.get("longitudinal_jerk_mps3", 0), 2
            ),
            "lateral_jerk_mps3": round(avg_seg.get("lateral_jerk_mps3", 0), 2),
            "jerk_mps3": round(avg_seg.get("jerk_magnitude_mps3", 0), 2),
            # Friction
            "friction_usage": round(avg_seg["friction_usage"] * 100, 0),
            "friction_mu": round(avg_seg["friction_mu"], 2),
            # Geometry
            "curvature_1pm": f"{avg_seg.get('curvature_1pm', 0):.4f}",
            "curvature_radius_m": radius_display,
            "dcurvature_ds": f"{avg_seg.get('dcurvature_ds', 0):.5f}",
            "grade_percent": round(avg_seg["grade_percent"], 1),
            "turn_type": avg_seg.get("turn_type", "straight"),
            "turn_angle_deg": round(avg_seg.get("turn_angle_deg", 0), 1),
            # Semantics
            "surface_type": avg_seg["surface_type"],
            "road_type": avg_seg.get("road_type") or "unknown",
            "lane_count": avg_seg.get("lane_count") or "N/A",
            "is_intersection": avg_seg.get("is_intersection", False),
            "distance_to_intersection_m": f"{avg_seg['distance_to_intersection_m']:.0f}"
            if avg_seg.get("distance_to_intersection_m")
            else "N/A",
            # Energy
            "energy_kwh_per_km": round(
                avg_seg["energy_kwh_per_km"] * 1000, 0
            ),  # Wh/km for readability
            "energy_per_meter_j": round(avg_seg.get("energy_per_meter_j", 0), 1),
            "regen_potential_j": round(avg_seg.get("regen_potential_j", 0), 0),
            "coasting_opportunity": "✓" if avg_seg.get("coasting_opportunity") else "",
            # Comfort
            "comfort_score": round(avg_seg.get("comfort_score", 0), 2),
            "is_uncomfortable": "⚠️" if avg_seg.get("is_uncomfortable") else "✓",
            # Action
            "recommended_action": avg_seg["recommended_action"],
            "action_confidence": round(avg_seg.get("action_confidence", 1.0), 2),
            "action_reason": avg_seg["action_reason"],
            "time_to_brake_s": f"{avg_seg['time_to_next_brake_s']:.1f}"
            if avg_seg.get("time_to_next_brake_s")
            else "N/A",
        }

        path_data.append(path_segment)

    return path_data, stats


def _build_rich_tooltip() -> Dict:
    """Build rich HTML tooltip for path segments with comprehensive features."""
    return {
        "html": """
        <div style="font-family: 'SF Pro Display', -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
                    background: rgba(15, 20, 30, 0.96);
                    padding: 16px 20px;
                    border-radius: 12px;
                    border: 1px solid rgba(255,255,255,0.1);
                    box-shadow: 0 8px 32px rgba(0,0,0,0.5);
                    min-width: 320px;
                    max-width: 400px;
                    backdrop-filter: blur(12px);">
            
            <!-- Header: Action & Surface -->
            <div style="display: flex; justify-content: space-between; align-items: center;
                        margin-bottom: 12px; padding-bottom: 10px; border-bottom: 1px solid rgba(255,255,255,0.1);">
                <div>
                    <span style="font-size: 10px; text-transform: uppercase; letter-spacing: 1px; color: #666;">
                        {surface_type} • {road_type}
                    </span>
                    <span style="font-size: 9px; color: #555; margin-left: 8px;">{turn_type}</span>
                </div>
                <span style="font-size: 12px; font-weight: 600; padding: 4px 10px; border-radius: 4px;
                             background: rgba(255,255,255,0.1); color: #fff;">
                    {recommended_action}
                    <span style="font-size: 9px; color: #888; margin-left: 4px;">({action_confidence})</span>
                </span>
            </div>
            
            <!-- Speed Section -->
            <div style="margin-bottom: 14px;">
                <div style="font-size: 10px; color: #555; text-transform: uppercase; letter-spacing: 1px; margin-bottom: 4px;">
                    📊 SPEED
                </div>
                <div style="display: flex; justify-content: space-between; align-items: baseline;">
                    <span style="font-size: 24px; font-weight: 700; color: #fff;">{speed_kmh} <span style="font-size: 12px; color: #888;">km/h</span></span>
                    <span style="font-size: 12px; color: #888;">limit: {physics_limit_kmh} | posted: {speed_limit_kmh}</span>
                </div>
                <div style="height: 4px; background: rgba(255,255,255,0.1); border-radius: 2px; margin-top: 6px;">
                    <div style="height: 100%; width: {speed_ratio}%; background: linear-gradient(90deg, #22c55e, #eab308, #ef4444); border-radius: 2px;"></div>
                </div>
            </div>
            
            <!-- Dynamics Section -->
            <div style="margin-bottom: 14px;">
                <div style="font-size: 10px; color: #555; text-transform: uppercase; letter-spacing: 1px; margin-bottom: 6px;">
                    ⚡ DYNAMICS
                </div>
                <div style="display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 10px;">
                    <div>
                        <div style="font-size: 9px; color: #666;">Lat G</div>
                        <div style="font-size: 15px; font-weight: 600; color: #fff;">{lateral_g}g</div>
                    </div>
                    <div>
                        <div style="font-size: 9px; color: #666;">Long G</div>
                        <div style="font-size: 15px; font-weight: 600; color: #fff;">{longitudinal_g}g</div>
                    </div>
                    <div>
                        <div style="font-size: 9px; color: #666;">Total G</div>
                        <div style="font-size: 15px; font-weight: 600; color: #fff;">{total_g}g</div>
                    </div>
                    <div>
                        <div style="font-size: 9px; color: #666;">Lat Jerk</div>
                        <div style="font-size: 13px; color: #aaa;">{lateral_jerk_mps3} m/s³</div>
                    </div>
                    <div>
                        <div style="font-size: 9px; color: #666;">Long Jerk</div>
                        <div style="font-size: 13px; color: #aaa;">{longitudinal_jerk_mps3} m/s³</div>
                    </div>
                    <div>
                        <div style="font-size: 9px; color: #666;">Friction</div>
                        <div style="font-size: 15px; font-weight: 600;">{friction_usage}%</div>
                    </div>
                </div>
            </div>
            
            <!-- Geometry Section -->
            <div style="margin-bottom: 14px;">
                <div style="font-size: 10px; color: #555; text-transform: uppercase; letter-spacing: 1px; margin-bottom: 6px;">
                    📐 GEOMETRY
                </div>
                <div style="display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 10px;">
                    <div>
                        <div style="font-size: 9px; color: #666;">Radius</div>
                        <div style="font-size: 13px; color: #aaa;">{curvature_radius_m}m</div>
                    </div>
                    <div>
                        <div style="font-size: 9px; color: #666;">Grade</div>
                        <div style="font-size: 13px; color: #aaa;">{grade_percent}%</div>
                    </div>
                    <div>
                        <div style="font-size: 9px; color: #666;">Turn</div>
                        <div style="font-size: 13px; color: #aaa;">{turn_angle_deg}°</div>
                    </div>
                    <div>
                        <div style="font-size: 9px; color: #666;">κ (1/m)</div>
                        <div style="font-size: 11px; color: #777;">{curvature_1pm}</div>
                    </div>
                    <div>
                        <div style="font-size: 9px; color: #666;">dκ/ds</div>
                        <div style="font-size: 11px; color: #777;">{dcurvature_ds}</div>
                    </div>
                    <div>
                        <div style="font-size: 9px; color: #666;">Lanes</div>
                        <div style="font-size: 13px; color: #aaa;">{lane_count}</div>
                    </div>
                </div>
            </div>
            
            <!-- Comfort & Energy Section -->
            <div style="margin-bottom: 12px; padding-top: 10px; border-top: 1px solid rgba(255,255,255,0.1);">
                <div style="font-size: 10px; color: #555; text-transform: uppercase; letter-spacing: 1px; margin-bottom: 6px;">
                    💚 COMFORT & ENERGY
                </div>
                <div style="display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 10px;">
                    <div>
                        <div style="font-size: 9px; color: #666;">Comfort</div>
                        <div style="font-size: 14px; font-weight: 500; color: #fff;">{comfort_score} {is_uncomfortable}</div>
                    </div>
                    <div>
                        <div style="font-size: 9px; color: #666;">Energy</div>
                        <div style="font-size: 14px; font-weight: 500; color: #22c55e;">{energy_kwh_per_km} Wh/km</div>
                    </div>
                    <div>
                        <div style="font-size: 9px; color: #666;">Regen</div>
                        <div style="font-size: 13px; color: #aaa;">{regen_potential_j}J</div>
                    </div>
                    <div>
                        <div style="font-size: 9px; color: #666;">Coast?</div>
                        <div style="font-size: 13px; color: #3b82f6;">{coasting_opportunity}</div>
                    </div>
                    <div>
                        <div style="font-size: 9px; color: #666;">To Brake</div>
                        <div style="font-size: 13px; color: #ef4444;">{time_to_brake_s}s</div>
                    </div>
                    <div>
                        <div style="font-size: 9px; color: #666;">To Int.</div>
                        <div style="font-size: 13px; color: #aaa;">{distance_to_intersection_m}m</div>
                    </div>
                </div>
            </div>
            
            <!-- Action Reason -->
            <div style="font-size: 11px; color: #666; font-style: italic; padding-top: 8px; border-top: 1px solid rgba(255,255,255,0.06);">
                💡 {action_reason}
            </div>
        </div>
        """,
        "style": {
            "backgroundColor": "transparent",
            "border": "none",
        },
    }


def _build_hud_html(
    title: str,
    mode: str,
    stats: Dict,
    analysis: Optional["ProfileAnalysis"] = None,
    style: str = "osm",
) -> str:
    """Build the HUD overlay HTML."""

    mode_labels = {
        "speed": ("Speed (km/h)", "#ef4444", "#22c55e"),  # Red=slow, Green=fast
        "energy": ("Energy (Wh/km)", "#22c55e", "#ef4444"),  # Green=efficient, Red=high
        "gforce": ("Lateral G-Force", "#3b82f6", "#ef4444"),
        "lateral_g": ("Lateral G-Force", "#3b82f6", "#ef4444"),
        "longitudinal_g": ("Longitudinal G", "#22c55e", "#ef4444"),
        "friction": ("Friction Usage (%)", "#22c55e", "#ef4444"),
        "comfort": ("Comfort Score", "#22c55e", "#ef4444"),  # Green=comfortable
        "difficulty": ("Difficulty", "#3b82f6", "#ef4444"),  # Blue=easy, Red=hard
        "safety_margin": (
            "Safety Margin",
            "#ef4444",
            "#22c55e",
        ),  # Red=danger, Green=safe
        "regen": ("Regen Potential", "#888", "#22c55e"),
        "action": ("Recommended Action", "#22c55e", "#ef4444"),
    }

    mode_info = mode_labels.get(mode, mode_labels["speed"])

    if analysis:
        summary = analysis.get_summary_dict()
        distance_km = summary.get("total_distance_km", 0)
        time_s = summary.get("lap_time_s", 0)
        time_min = summary.get("lap_time_min", 0)
        energy_kwh = summary.get("total_energy_kwh", 0)
        total_regen_kwh = summary.get("total_regen_kwh", 0)
        avg_speed = summary.get("avg_speed_kmh", 0)
        max_speed = summary.get("max_speed_kmh", 0)
        max_lat_g = summary.get("max_lateral_g", 0)
        p95_lat_g = summary.get("p95_lateral_g", 0)
        avg_lat_g = summary.get("avg_lateral_g", 0)
        max_friction = summary.get("max_friction_usage", 0) * 100
        avg_friction = summary.get("avg_friction_usage", 0) * 100
        p95_jerk = summary.get("p95_longitudinal_jerk", 0)
        pct_brake = summary.get("brake_fraction", 0) * 100
        pct_coast = summary.get("coast_fraction", 0) * 100
        pct_accel = summary.get("accelerate_fraction", 0) * 100
        pct_uncomfortable = summary.get("pct_uncomfortable", 0)
        comfort_violations = summary.get("comfort_violations_per_km", 0)
        efficiency = summary.get("energy_kwh_per_km", 0)
        regen_fraction = summary.get("regen_fraction", 0)
        turns_per_km = summary.get("turns_per_km", 0)
        difficulty = summary.get("difficulty_score", 0)
        safety_margin = summary.get("safety_margin_score", 0)
        max_grade = summary.get("max_grade_percent", 0)
        avg_speed_limit = summary.get("avg_speed_limit_kmh", 0)
        p95_speed_limit = summary.get("p95_speed_limit_kmh", 0)
        avg_over_limit = summary.get("avg_speed_over_limit_kmh", 0)
        pct_distance_over_limit = summary.get("pct_distance_over_limit", 0)
    else:
        distance_km = stats.get("total_distance_km", 0)
        time_s = 0
        time_min = 0
        energy_kwh = 0
        avg_speed = stats.get("avg_speed", 0)
        max_speed = stats.get("max_speed", 0)
        max_lat_g = stats.get("max_lateral_g", 0)
        p95_lat_g = max_lat_g * 0.85
        avg_lat_g = stats.get("avg_lateral_g", 0)
        max_friction = stats.get("max_friction_usage", 0) * 100
        avg_friction = max_friction * 0.6
        p95_jerk = 0
        pct_brake = 0
        pct_coast = 0
        pct_accel = 0
        pct_uncomfortable = 0
        comfort_violations = 0
        efficiency = 0
        regen_fraction = 0
        turns_per_km = 0
        difficulty = 0
        safety_margin = 0
        max_grade = 0

    # Map style indicator
    style_label = style.upper()
    if style in ["satellite", "satellite-plain"]:
        style_badge = "🛰️ Satellite"
    elif style == "dark":
        style_badge = "🌙 Dark"
    else:
        style_badge = "🗺️ OSM"

    return f"""
    <style>
        @import url('https://fonts.googleapis.com/css2?family=Inter:wght@400;500;600;700&display=swap');
        .apex-hud {{ font-family: 'Inter', -apple-system, BlinkMacSystemFont, sans-serif; }}
        .apex-hud * {{ box-sizing: border-box; }}
    </style>
    
    <div class="apex-hud" style="position: absolute; top: 20px; left: 20px; z-index: 1000;">
        
        <div style="background: rgba(10, 15, 25, 0.92); padding: 20px 24px; 
                    border-radius: 16px; border: 1px solid rgba(255,255,255,0.08);
                    box-shadow: 0 8px 32px rgba(0,0,0,0.5); margin-bottom: 16px;
                    backdrop-filter: blur(20px);">
            <div style="display: flex; justify-content: space-between; align-items: flex-start;">
                <div>
                    <div style="font-size: 24px; font-weight: 700; color: #fff; margin-bottom: 6px;
                                background: linear-gradient(135deg, #fff, #a0a0a0);
                                -webkit-background-clip: text; -webkit-text-fill-color: transparent;">
                        🏎️ {title}
                    </div>
                    <div style="font-size: 12px; color: rgba(255,255,255,0.5); text-transform: uppercase; letter-spacing: 1.5px;">
                        {mode_info[0]} Analysis
                    </div>
                </div>
                <div style="font-size: 10px; padding: 4px 8px; background: rgba(255,255,255,0.1); 
                            border-radius: 6px; color: rgba(255,255,255,0.7);">
                    {style_badge}
                </div>
            </div>
        </div>
        
        <div style="background: rgba(10, 15, 25, 0.92); padding: 20px 24px;
                    border-radius: 16px; border: 1px solid rgba(255,255,255,0.08);
                    box-shadow: 0 8px 32px rgba(0,0,0,0.5); margin-bottom: 16px;
                    backdrop-filter: blur(20px); min-width: 280px;">
            
            <div style="font-size: 11px; color: rgba(255,255,255,0.4); text-transform: uppercase;
                        letter-spacing: 1px; margin-bottom: 16px;">Route Statistics</div>
            
            <div style="display: grid; grid-template-columns: 1fr 1fr; gap: 16px;">
                <div>
                    <div style="font-size: 10px; color: rgba(255,255,255,0.4); text-transform: uppercase;">Distance</div>
                    <div style="font-size: 20px; font-weight: 600; color: #fff;">{distance_km:.2f} <span style="font-size: 12px; color: rgba(255,255,255,0.5);">km</span></div>
                </div>
                <div>
                    <div style="font-size: 10px; color: rgba(255,255,255,0.4); text-transform: uppercase;">Time</div>
                    <div style="font-size: 20px; font-weight: 600; color: #fff;">{time_min:.1f} <span style="font-size: 12px; color: rgba(255,255,255,0.5);">min</span></div>
                </div>
                <div>
                    <div style="font-size: 10px; color: rgba(255,255,255,0.4); text-transform: uppercase;">Avg Speed</div>
                    <div style="font-size: 20px; font-weight: 600; color: #fff;">{avg_speed:.1f} <span style="font-size: 12px; color: rgba(255,255,255,0.5);">km/h</span></div>
                </div>
                <div>
                    <div style="font-size: 10px; color: rgba(255,255,255,0.4); text-transform: uppercase;">Max Speed</div>
                    <div style="font-size: 20px; font-weight: 600; color: #fff;">{max_speed:.1f} <span style="font-size: 12px; color: rgba(255,255,255,0.5);">km/h</span></div>
                </div>
            </div>
            
            <div style="height: 1px; background: rgba(255,255,255,0.08); margin: 16px 0;"></div>
            
            <div style="display: grid; grid-template-columns: 1fr 1fr; gap: 14px;">
                <div>
                    <div style="font-size: 10px; color: rgba(255,255,255,0.4); text-transform: uppercase;">Energy</div>
                    <div style="font-size: 18px; font-weight: 600; color: #22c55e;">{energy_kwh:.3f} <span style="font-size: 11px; color: rgba(255,255,255,0.5);">kWh</span></div>
                    <div style="font-size: 10px; color: rgba(255,255,255,0.35);">({efficiency*1000:.0f} Wh/km)</div>
                </div>
                <div>
                    <div style="font-size: 10px; color: rgba(255,255,255,0.4); text-transform: uppercase;">Regen Potential</div>
                    <div style="font-size: 18px; font-weight: 600; color: #3b82f6;">{regen_fraction*100:.0f}%</div>
                    <div style="font-size: 10px; color: rgba(255,255,255,0.35);">{total_regen_kwh:.3f} kWh</div>
                </div>
                <div>
                    <div style="font-size: 10px; color: rgba(255,255,255,0.4); text-transform: uppercase;">Max Lat G</div>
                    <div style="font-size: 18px; font-weight: 600; color: {'#ef4444' if max_lat_g > 0.6 else '#eab308' if max_lat_g > 0.3 else '#22c55e'};">{max_lat_g:.2f}g</div>
                    <div style="font-size: 10px; color: rgba(255,255,255,0.35);">(p95: {p95_lat_g:.2f}g)</div>
                </div>
                <div>
                    <div style="font-size: 10px; color: rgba(255,255,255,0.4); text-transform: uppercase;">Friction Use</div>
                    <div style="font-size: 18px; font-weight: 600; color: {'#ef4444' if max_friction > 80 else '#eab308' if max_friction > 50 else '#22c55e'};">{max_friction:.0f}%</div>
                    <div style="font-size: 10px; color: rgba(255,255,255,0.35);">(avg: {avg_friction:.0f}%)</div>
                </div>
            </div>
            
            <div style="height: 1px; background: rgba(255,255,255,0.08); margin: 14px 0;"></div>
            
            <div style="display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 12px; margin-bottom: 12px;">
                <div>
                    <div style="font-size: 10px; color: rgba(255,255,255,0.4); text-transform: uppercase;">p95 Jerk</div>
                    <div style="font-size: 14px; font-weight: 500; color: #fff;">{p95_jerk:.1f} <span style="font-size: 9px;">m/s³</span></div>
                </div>
                <div>
                    <div style="font-size: 10px; color: rgba(255,255,255,0.4); text-transform: uppercase;">Comfort Viol.</div>
                    <div style="font-size: 14px; font-weight: 500; color: {'#ef4444' if comfort_violations > 1 else '#eab308' if comfort_violations > 0.5 else '#22c55e'};">{comfort_violations:.2f}/km</div>
                </div>
                <div>
                    <div style="font-size: 10px; color: rgba(255,255,255,0.4); text-transform: uppercase;">Turns</div>
                    <div style="font-size: 14px; font-weight: 500; color: #fff;">{turns_per_km:.1f}/km</div>
                </div>
                <div>
                    <div style="font-size: 10px; color: rgba(255,255,255,0.4); text-transform: uppercase;">Max Grade</div>
                    <div style="font-size: 14px; font-weight: 500; color: #fff;">{max_grade:.1f}%</div>
                </div>
                <div>
                    <div style="font-size: 10px; color: rgba(255,255,255,0.4); text-transform: uppercase;">Difficulty</div>
                    <div style="font-size: 14px; font-weight: 600; color: {'#ef4444' if difficulty > 0.7 else '#eab308' if difficulty > 0.4 else '#22c55e'};">{difficulty:.2f}</div>
                </div>
                <div>
                    <div style="font-size: 10px; color: rgba(255,255,255,0.4); text-transform: uppercase;">Safety</div>
                    <div style="font-size: 14px; font-weight: 500; color: {'#22c55e' if safety_margin > 0.6 else '#eab308' if safety_margin > 0.3 else '#ef4444'};">{safety_margin:.2f}</div>
                </div>
                <div>
                    <div style="font-size: 10px; color: rgba(255,255,255,0.4); text-transform: uppercase;">P95 Speed Limit</div>
                    <div style="font-size: 14px; font-weight: 500; color: #fff;">{p95_speed_limit:.1f} km/h</div>
                </div>
                <div>
                    <div style="font-size: 10px; color: rgba(255,255,255,0.4); text-transform: uppercase;">Over Limit</div>
                    <div style="font-size: 14px; font-weight: 500; color: {'#ef4444' if pct_distance_over_limit > 20 else '#eab308' if pct_distance_over_limit > 5 else '#22c55e'};">{pct_distance_over_limit:.1f}% dist</div>
                </div>
                <div>
                    <div style="font-size: 10px; color: rgba(255,255,255,0.4); text-transform: uppercase;">Avg Overspeed</div>
                    <div style="font-size: 14px; font-weight: 500; color: #fff;">{avg_over_limit:.1f} km/h</div>
                </div>
            </div>
            
            <div style="height: 1px; background: rgba(255,255,255,0.08); margin: 14px 0;"></div>
            
            <div style="font-size: 10px; color: rgba(255,255,255,0.4); text-transform: uppercase; margin-bottom: 10px;">
                Action Distribution
            </div>
            <div style="display: flex; gap: 6px;">
                <div style="flex: 1; text-align: center; padding: 8px; background: rgba(239,68,68,0.2); border-radius: 8px;">
                    <div style="font-size: 14px; font-weight: 600; color: #ef4444;">{pct_brake:.0f}%</div>
                    <div style="font-size: 9px; color: rgba(255,255,255,0.5);">BRAKE</div>
                </div>
                <div style="flex: 1; text-align: center; padding: 8px; background: rgba(59,130,246,0.2); border-radius: 8px;">
                    <div style="font-size: 14px; font-weight: 600; color: #3b82f6;">{pct_coast:.0f}%</div>
                    <div style="font-size: 9px; color: rgba(255,255,255,0.5);">COAST</div>
                </div>
                <div style="flex: 1; text-align: center; padding: 8px; background: rgba(40,167,69,0.2); border-radius: 8px;">
                    <div style="font-size: 14px; font-weight: 600; color: #22c55e;">{pct_accel:.0f}%</div>
                    <div style="font-size: 9px; color: rgba(255,255,255,0.5);">ACCEL</div>
                </div>
                <div style="flex: 1; text-align: center; padding: 8px; background: rgba(234,179,8,0.2); border-radius: 8px;">
                    <div style="font-size: 14px; font-weight: 600; color: #eab308;">{pct_uncomfortable:.0f}%</div>
                    <div style="font-size: 9px; color: rgba(255,255,255,0.5);">UNCOMF</div>
                </div>
            </div>
        </div>
        
        <div style="background: rgba(10, 15, 25, 0.92); padding: 16px 20px;
                    border-radius: 12px; border: 1px solid rgba(255,255,255,0.08);
                    box-shadow: 0 8px 32px rgba(0,0,0,0.5);
                    backdrop-filter: blur(20px);">
            <div style="font-size: 10px; color: rgba(255,255,255,0.4); text-transform: uppercase;
                        letter-spacing: 1px; margin-bottom: 10px;">Legend</div>
            <div style="display: flex; align-items: center; gap: 8px;">
                <span style="font-size: 11px; color: rgba(255,255,255,0.6);">Low</span>
                <div style="flex: 1; height: 8px; border-radius: 4px;
                            background: linear-gradient(90deg, {mode_info[1]}, #eab308, {mode_info[2]});"></div>
                <span style="font-size: 11px; color: rgba(255,255,255,0.6);">High</span>
            </div>
            <div style="font-size: 10px; color: rgba(255,255,255,0.3); margin-top: 8px;">
                📐 Height = Lateral G-Force
            </div>
        </div>
    </div>
    """


def make_deck(
    view_state: Any,
    data_layers: List[Any],
    style: str = "osm",
    tooltip: Optional[Dict] = None,
) -> Any:
    """
    Create a pydeck Deck with the appropriate basemap.

    Args:
        view_state: pydeck ViewState
        data_layers: List of data layers (PathLayer, etc.)
        style: Map style - "osm", "dark", "satellite", etc.
        tooltip: Tooltip configuration

    Returns:
        pydeck.Deck object
    """
    _check_pydeck()

    map_style, tile_layer, mapbox_token = _get_basemap_layer(style)

    # Build layers list
    layers = []
    if tile_layer:
        layers.append(tile_layer)
    layers.extend(data_layers)

    # Build deck configuration
    deck_kwargs = {
        "layers": layers,
        "initial_view_state": view_state,
        "tooltip": tooltip,
    }

    if map_style and mapbox_token:
        # Use Mapbox provider for Mapbox styles
        deck_kwargs["map_style"] = map_style
        deck_kwargs["map_provider"] = "mapbox"
        deck_kwargs["api_keys"] = {"mapbox": mapbox_token}
    elif tile_layer:
        # Using TileLayer for basemap, no map_style needed
        deck_kwargs["map_style"] = None
        deck_kwargs["map_provider"] = None
    else:
        deck_kwargs["map_style"] = None

    return pdk.Deck(**deck_kwargs)


def visualize_profile(
    path_points: List[Dict],
    mode: str = None,  # Deprecated, use color_by
    color_by: str = "speed",
    title: Optional[str] = None,
    output_html: Optional[str] = None,
    show: bool = True,
    style: str = "osm",
    condition: str = "dry",
    friction_multiplier: float = 1.0,
    lateral_g_scale: float = 30.0,
    path_width_m: float = 8.0,
) -> Any:
    """
    Render an interactive 3D velocity profile visualization.

    Args:
        path_points: List of dicts with lat, lon, speed_mps, curvature, etc.
        color_by: Feature to color paths by. Options:
            - "speed": Color by velocity (km/h)
            - "energy": Color by energy intensity (Wh/km)
            - "lateral_g": Color by lateral G-force
            - "longitudinal_g": Color by longitudinal acceleration
            - "friction": Color by friction usage (% of limit)
            - "comfort": Color by comfort score
            - "difficulty": Color by local difficulty
            - "safety_margin": Color by safety margin
            - "action": Color by recommended action (brake/coast/accel)
        mode: (Deprecated) Alias for color_by, for backwards compatibility
        title: Title for the visualization
        output_html: If set, save to this HTML file
        show: If True and in notebook, display inline
        style: Map style - "osm" (default, no API key), "dark", "satellite" (needs MAPBOX_API_KEY)
        condition: "dry" or "wet" for friction calculation
        friction_multiplier: Additional friction scaling
        lateral_g_scale: Scale factor for G-force elevation
        path_width_m: Width of path tubes in meters

    Returns:
        pydeck.Deck object
    """
    # Handle backwards compatibility: mode -> color_by
    if mode is not None:
        color_by = mode
    _check_pydeck()

    # Analyze the profile
    if _HAS_ANALYSIS:
        analysis = analyze_profile(
            path_points, condition=condition, friction_multiplier=friction_multiplier
        )
        segments = [seg.to_dict() for seg in analysis.segments]
    else:
        analysis = None
        segments = path_points

    # Build path data
    path_data, stats = _build_path_data(
        segments,
        mode=color_by,
        lateral_g_scale=lateral_g_scale,
        path_width_m=path_width_m,
    )

    if not path_data:
        raise ValueError("No valid path data to visualize")

    # Extract coordinates
    lats = [s.get("lat", 0) for s in segments if abs(s.get("lat", 0)) > 0.001]
    lons = [s.get("lon", 0) for s in segments if abs(s.get("lon", 0)) > 0.001]

    if not lats or not lons:
        raise ValueError("No valid coordinates in path points. Ensure lat/lon are set.")

    if analysis:
        stats["total_distance_km"] = analysis.total_distance_km

    # Create path layer
    path_layer = pdk.Layer(
        "PathLayer",
        data=path_data,
        get_path="path",
        get_color="color",
        get_width="width",
        width_scale=1,
        width_units="meters",
        width_min_pixels=3,
        width_max_pixels=20,
        billboard=True,
        pickable=True,
        auto_highlight=True,
        highlight_color=[255, 255, 255, 80],
    )

    # View state
    view_state = _compute_view_state(lats, lons, pitch=45, bearing=15)

    # Tooltip
    tooltip = _build_rich_tooltip()

    # Create deck with appropriate basemap
    deck = make_deck(view_state, [path_layer], style=style, tooltip=tooltip)

    # Build HUD
    display_title = title or "ApexVelocity Analysis"
    hud_html = _build_hud_html(display_title, color_by, stats, analysis, style)

    # Save to HTML
    if output_html:
        html_content = deck.to_html(as_string=True)
        html_content = html_content.replace("<body>", f"<body>{hud_html}")

        with open(output_html, "w") as f:
            f.write(html_content)
        print(f"✓ Saved visualization: {output_html}")

    return deck


def visualize_comparison(
    path_points_a: List[Dict],
    path_points_b: List[Dict],
    label_a: str = "Condition A",
    label_b: str = "Condition B",
    title: Optional[str] = None,
    output_html: Optional[str] = None,
    style: str = "osm",
    condition_a: str = "dry",
    condition_b: str = "wet",
    friction_a: float = 1.0,
    friction_b: float = 0.7,
) -> Any:
    """
    Render two profiles on the same map for comparison.

    Args:
        path_points_a: First set of path points
        path_points_b: Second set of path points
        label_a: Label for first condition
        label_b: Label for second condition
        title: Optional title
        output_html: If set, save to this HTML file
        style: Map style - "osm" (default), "dark", "satellite"

    Returns:
        pydeck.Deck object
    """
    _check_pydeck()

    # Analyze both profiles
    if _HAS_ANALYSIS:
        analysis_a = analyze_profile(
            path_points_a, condition=condition_a, friction_multiplier=friction_a
        )
        analysis_b = analyze_profile(
            path_points_b, condition=condition_b, friction_multiplier=friction_b
        )
        segments_a = [seg.to_dict() for seg in analysis_a.segments]
        segments_b = [seg.to_dict() for seg in analysis_b.segments]
    else:
        analysis_a = analysis_b = None
        segments_a = path_points_a
        segments_b = path_points_b

    # Build path data
    path_data_a, stats_a = _build_path_data(
        segments_a, mode="speed", lateral_g_scale=25
    )
    path_data_b, stats_b = _build_path_data(
        segments_b, mode="speed", lateral_g_scale=25
    )

    # Color paths distinctly
    for seg in path_data_a:
        seg["color"] = [66, 165, 245, 220]
        for pt in seg["path"]:
            pt[2] += 5

    for seg in path_data_b:
        seg["color"] = [255, 152, 0, 220]
        for pt in seg["path"]:
            pt[2] += 35

    all_data = path_data_a + path_data_b

    lats = [s.get("lat", 0) for s in segments_a if abs(s.get("lat", 0)) > 0.001]
    lons = [s.get("lon", 0) for s in segments_a if abs(s.get("lon", 0)) > 0.001]

    path_layer = pdk.Layer(
        "PathLayer",
        data=all_data,
        get_path="path",
        get_color="color",
        get_width="width",
        width_scale=1,
        width_units="meters",
        width_min_pixels=3,
        billboard=True,
        pickable=True,
    )

    view_state = _compute_view_state(lats, lons, pitch=45, bearing=15)

    deck = make_deck(
        view_state, [path_layer], style=style, tooltip=_build_rich_tooltip()
    )

    # Comparison stats
    if analysis_a and analysis_b:
        time_a = analysis_a.total_time_s
        time_b = analysis_b.total_time_s
        energy_a = analysis_a.total_energy_kwh
        energy_b = analysis_b.total_energy_kwh
        max_g_a = analysis_a.max_lateral_g
        max_g_b = analysis_b.max_lateral_g
        avg_speed_a = analysis_a.avg_speed_kmh
        avg_speed_b = analysis_b.avg_speed_kmh
    else:
        time_a = time_b = 0
        energy_a = energy_b = 0
        max_g_a = max_g_b = 0
        avg_speed_a = stats_a.get("avg_speed", 0)
        avg_speed_b = stats_b.get("avg_speed", 0)

    time_delta = time_b - time_a
    time_pct = (time_delta / time_a * 100) if time_a > 0 else 0

    display_title = title or f"{label_a} vs {label_b}"

    # Style badge
    if style in ["satellite", "satellite-plain"]:
        style_badge = "🛰️ Satellite"
    elif style == "dark":
        style_badge = "🌙 Dark"
    else:
        style_badge = "🗺️ OSM"

    comparison_hud = f"""
    <style>
        @import url('https://fonts.googleapis.com/css2?family=Inter:wght@400;500;600;700&display=swap');
    </style>
    <div style="position: absolute; top: 20px; left: 20px; z-index: 1000;
                font-family: 'Inter', -apple-system, sans-serif;">
        
        <div style="background: rgba(10, 15, 25, 0.92); padding: 20px 24px;
                    border-radius: 16px; border: 1px solid rgba(255,255,255,0.08);
                    box-shadow: 0 8px 32px rgba(0,0,0,0.5); margin-bottom: 16px;
                    backdrop-filter: blur(20px);">
            <div style="display: flex; justify-content: space-between; align-items: flex-start;">
                <div>
                    <div style="font-size: 24px; font-weight: 700; color: #fff; margin-bottom: 6px;">
                        🏁 {display_title}
                    </div>
                    <div style="font-size: 12px; color: rgba(255,255,255,0.5); text-transform: uppercase; letter-spacing: 1.5px;">
                        Condition Comparison
                    </div>
                </div>
                <div style="font-size: 10px; padding: 4px 8px; background: rgba(255,255,255,0.1); 
                            border-radius: 6px; color: rgba(255,255,255,0.7);">
                    {style_badge}
                </div>
            </div>
        </div>
        
        <div style="background: rgba(10, 15, 25, 0.92); padding: 20px 24px;
                    border-radius: 16px; border: 1px solid rgba(255,255,255,0.08);
                    box-shadow: 0 8px 32px rgba(0,0,0,0.5);
                    backdrop-filter: blur(20px); min-width: 320px;">
            
            <table style="width: 100%; border-collapse: collapse; color: #fff; font-size: 14px;">
                <tr style="border-bottom: 1px solid rgba(255,255,255,0.1);">
                    <th style="text-align: left; padding: 12px 8px; color: rgba(255,255,255,0.5); font-weight: 500;"></th>
                    <th style="text-align: center; padding: 12px 8px;">
                        <div style="display: inline-block; width: 12px; height: 12px; background: #42a5f5; border-radius: 3px; margin-right: 6px;"></div>
                        {label_a}
                    </th>
                    <th style="text-align: center; padding: 12px 8px;">
                        <div style="display: inline-block; width: 12px; height: 12px; background: #ff9800; border-radius: 3px; margin-right: 6px;"></div>
                        {label_b}
                    </th>
                </tr>
                <tr>
                    <td style="padding: 10px 8px; color: rgba(255,255,255,0.6);">Lap Time</td>
                    <td style="text-align: center; padding: 10px 8px; font-weight: 600;">{time_a/60:.2f} min</td>
                    <td style="text-align: center; padding: 10px 8px; font-weight: 600;">{time_b/60:.2f} min</td>
                </tr>
                <tr>
                    <td style="padding: 10px 8px; color: rgba(255,255,255,0.6);">Avg Speed</td>
                    <td style="text-align: center; padding: 10px 8px; font-weight: 600;">{avg_speed_a:.1f} km/h</td>
                    <td style="text-align: center; padding: 10px 8px; font-weight: 600;">{avg_speed_b:.1f} km/h</td>
                </tr>
                <tr>
                    <td style="padding: 10px 8px; color: rgba(255,255,255,0.6);">Max G</td>
                    <td style="text-align: center; padding: 10px 8px; font-weight: 600;">{max_g_a:.2f} g</td>
                    <td style="text-align: center; padding: 10px 8px; font-weight: 600;">{max_g_b:.2f} g</td>
                </tr>
                <tr>
                    <td style="padding: 10px 8px; color: rgba(255,255,255,0.6);">Energy</td>
                    <td style="text-align: center; padding: 10px 8px; font-weight: 600;">{energy_a:.3f} kWh</td>
                    <td style="text-align: center; padding: 10px 8px; font-weight: 600;">{energy_b:.3f} kWh</td>
                </tr>
            </table>
            
            <div style="margin-top: 16px; padding-top: 16px; border-top: 1px solid rgba(255,255,255,0.1);">
                <div style="display: flex; justify-content: space-between; align-items: center;">
                    <span style="color: rgba(255,255,255,0.6);">Time Delta:</span>
                    <span style="font-size: 20px; font-weight: 700;
                                 color: {'#ef4444' if time_delta > 0 else '#22c55e'};">
                        {'+' if time_delta >= 0 else ''}{time_delta:.1f}s ({time_pct:+.1f}%)
                    </span>
                </div>
            </div>
        </div>
        
        <div style="background: rgba(10, 15, 25, 0.92); padding: 16px 20px; margin-top: 16px;
                    border-radius: 12px; border: 1px solid rgba(255,255,255,0.08);
                    box-shadow: 0 8px 32px rgba(0,0,0,0.5); backdrop-filter: blur(20px);">
            <div style="display: flex; gap: 20px;">
                <div style="display: flex; align-items: center; gap: 8px;">
                    <div style="width: 24px; height: 4px; background: #42a5f5; border-radius: 2px;"></div>
                    <span style="font-size: 12px; color: rgba(255,255,255,0.7);">{label_a}</span>
                </div>
                <div style="display: flex; align-items: center; gap: 8px;">
                    <div style="width: 24px; height: 4px; background: #ff9800; border-radius: 2px;"></div>
                    <span style="font-size: 12px; color: rgba(255,255,255,0.7);">{label_b}</span>
                </div>
            </div>
        </div>
    </div>
    """

    if output_html:
        html_content = deck.to_html(as_string=True)
        html_content = html_content.replace("<body>", f"<body>{comparison_hud}")

        with open(output_html, "w") as f:
            f.write(html_content)
        print(f"✓ Saved comparison: {output_html}")

    return deck


def visualize_profile_interactive(
    path_points: List[Dict],
    title: str = "ApexVelocity Analysis",
    output_html: str = "apexvelocity.html",
    initial_color_by: str = "speed",
    style: str = "satellite",
    mapbox_token: Optional[str] = None,
    condition: str = "dry",
    friction_multiplier: float = 1.0,
    path_width_m: float = 8.0,
) -> None:
    """
    Generate an interactive HTML visualization with metric switching dropdown.

    This creates a standalone HTML file that:
    1. Properly renders Mapbox satellite/street tiles
    2. Has an interactive dropdown to switch between color metrics
    3. Shows a dynamic legend that updates with the selected metric

    Args:
        path_points: List of dicts with lat, lon, speed_mps, curvature, etc.
        title: Title for the visualization
        output_html: Output HTML file path
        initial_color_by: Initial color metric to display
        style: Map style - "satellite", "satellite-streets", "dark", "streets"
        mapbox_token: Mapbox API token (or set MAPBOX_API_KEY env var)
        condition: "dry" or "wet" for friction calculation
        friction_multiplier: Additional friction scaling
        path_width_m: Width of path in meters
    """
    import json

    # Get Mapbox token
    token = mapbox_token or _get_mapbox_token()
    if not token:
        print(
            "⚠️  No Mapbox token found. Set MAPBOX_API_KEY or pass mapbox_token parameter."
        )
        print("   Falling back to OpenStreetMap tiles (no satellite).")

    # Analyze the profile
    if _HAS_ANALYSIS:
        analysis = analyze_profile(
            path_points, condition=condition, friction_multiplier=friction_multiplier
        )
        segments = [seg.to_dict() for seg in analysis.segments]
        summary = analysis.get_summary_dict()
    else:
        analysis = None
        segments = path_points
        summary = {}

    # Build path data for ALL color modes
    all_modes = [
        "speed",
        "energy",
        "lateral_g",
        "friction",
        "comfort",
        "difficulty",
        "safety_margin",
        "regen",
        "action",
    ]
    mode_data = {}

    for mode in all_modes:
        path_data, stats = _build_path_data(
            segments, mode=mode, path_width_m=path_width_m
        )
        mode_data[mode] = path_data

    # Extract coordinates for view
    lats = [s.get("lat", 0) for s in segments if abs(s.get("lat", 0)) > 0.001]
    lons = [s.get("lon", 0) for s in segments if abs(s.get("lon", 0)) > 0.001]

    if not lats or not lons:
        raise ValueError("No valid coordinates in path points")

    center_lat = sum(lats) / len(lats)
    center_lon = sum(lons) / len(lons)

    # Calculate zoom
    lat_range = max(lats) - min(lats)
    lon_range = max(lons) - min(lons)
    max_range = max(lat_range, lon_range)

    if max_range > 0.2:
        zoom = 10
    elif max_range > 0.05:
        zoom = 12
    elif max_range > 0.01:
        zoom = 14
    else:
        zoom = 16

    # Compute additional stats from segments
    max_grade = 0

    for seg in segments:
        grade = abs(seg.get("grade_percent", 0))
        if grade > max_grade:
            max_grade = grade

    # Build summary stats for HUD
    hud_stats = {
        "distance_km": summary.get("total_distance_km", 0),
        "time_min": summary.get("lap_time_min", 0),
        "avg_speed": summary.get("avg_speed_kmh", 0),
        "max_speed": summary.get("max_speed_kmh", 0),
        "energy_kwh": summary.get("total_energy_kwh", 0),
        "max_lat_g": summary.get("max_lateral_g", 0),
        "p95_lat_g": summary.get("p95_lateral_g", 0),
        "max_friction": summary.get("max_friction_usage", 0) * 100,
        "avg_friction": summary.get("avg_friction_usage", 0) * 100,
        "difficulty": summary.get("difficulty_score", 0),
        "safety_margin": summary.get("safety_margin_score", 0),
        "pct_brake": summary.get("brake_fraction", 0) * 100,
        "pct_coast": summary.get("coast_fraction", 0) * 100,
        "pct_accel": summary.get("accelerate_fraction", 0) * 100,
        "max_grade": max_grade,
        # Use analysis-level regen estimate so we always reflect the
        # physics-based regeneration model, even if core energy_joules
        # is strictly positive.
        "regen_kwh": summary.get("total_regen_kwh", 0),
        "regen_fraction": summary.get("regen_fraction", 0),
        # Speed-limit stats for UI (avg and p95).
        "avg_speed_limit_kmh": summary.get("avg_speed_limit_kmh", 0),
        "p95_speed_limit_kmh": summary.get("p95_speed_limit_kmh", 0),
    }
    # Add elevation metadata so the UI can indicate uphill/downhill.
    if path_points:
        start_elev = path_points[0].get("z_m")
        if start_elev is None:
            start_elev = path_points[0].get("elevation_m", 0.0)
        end_elev = path_points[-1].get("z_m")
        if end_elev is None:
            end_elev = path_points[-1].get("elevation_m", 0.0)
        hud_stats["start_elev_m"] = float(start_elev)
        hud_stats["end_elev_m"] = float(end_elev)
        hud_stats["net_elev_m"] = float(end_elev) - float(start_elev)
    else:
        hud_stats["start_elev_m"] = 0.0
        hud_stats["end_elev_m"] = 0.0
        hud_stats["net_elev_m"] = 0.0

    # Convert mode_data to JSON
    mode_data_json = json.dumps(mode_data)
    hud_stats_json = json.dumps(hud_stats)

    # Determine map style URL
    if token:
        if style == "satellite" or style == "satellite-streets":
            map_style_url = "mapbox://styles/mapbox/satellite-streets-v12"
        elif style == "dark":
            map_style_url = "mapbox://styles/mapbox/dark-v11"
        elif style == "streets":
            map_style_url = "mapbox://styles/mapbox/streets-v12"
        else:
            map_style_url = "mapbox://styles/mapbox/satellite-streets-v12"
        use_mapbox = True
    else:
        map_style_url = ""
        use_mapbox = False

    # Generate the HTML
    start_lat = path_points[0].get("lat", center_lat) if path_points else center_lat
    start_lon = path_points[0].get("lon", center_lon) if path_points else center_lon
    end_lat = path_points[-1].get("lat", center_lat) if path_points else center_lat
    end_lon = path_points[-1].get("lon", center_lon) if path_points else center_lon

    html_content = _generate_interactive_html(
        title=title,
        mode_data_json=mode_data_json,
        hud_stats_json=hud_stats_json,
        center_lat=center_lat,
        center_lon=center_lon,
        zoom=zoom,
        map_style_url=map_style_url,
        mapbox_token=token or "",
        use_mapbox=use_mapbox,
        initial_color_by=initial_color_by,
        path_width=path_width_m,
        start_lat=start_lat,
        start_lon=start_lon,
        end_lat=end_lat,
        end_lon=end_lon,
    )

    with open(output_html, "w") as f:
        f.write(html_content)

    print(f"✓ Saved interactive visualization: {output_html}")


def _generate_interactive_html(
    title: str,
    mode_data_json: str,
    hud_stats_json: str,
    center_lat: float,
    center_lon: float,
    zoom: int,
    map_style_url: str,
    mapbox_token: str,
    use_mapbox: bool,
    initial_color_by: str,
    path_width: float,
    start_lat: float,
    start_lon: float,
    end_lat: float,
    end_lon: float,
) -> str:
    """Generate the complete interactive HTML with Mapbox GL JS + deck.gl overlay."""

    # Use Mapbox GL JS as the basemap with deck.gl overlay for reliable satellite rendering
    return f"""<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>{title}</title>
    <script src="https://unpkg.com/deck.gl@^8.9.0/dist.min.js"></script>
    <script src="https://unpkg.com/@deck.gl/json@^8.9.0/dist.min.js"></script>
    <script src="https://api.mapbox.com/mapbox-gl-js/v2.15.0/mapbox-gl.js"></script>
    <link href="https://api.mapbox.com/mapbox-gl-js/v2.15.0/mapbox-gl.css" rel="stylesheet" />
    <link href="https://fonts.googleapis.com/css2?family=Inter:wght@400;500;600;700&family=JetBrains+Mono:wght@400;500&display=swap" rel="stylesheet">
    <style>
        * {{ margin: 0; padding: 0; box-sizing: border-box; }}
        body {{ font-family: 'Inter', -apple-system, BlinkMacSystemFont, sans-serif; background: #0a0f19; }}
        #map {{ width: 100vw; height: 100vh; position: absolute; top: 0; left: 0; }}
        #deck-canvas {{ width: 100vw; height: 100vh; position: absolute; top: 0; left: 0; pointer-events: none; }}
        
        /* Main Control Panel */
        .control-panel {{
            position: absolute;
            top: 20px;
            left: 20px;
            z-index: 1000;
            background: rgba(10, 15, 25, 0.96);
            padding: 20px;
            border-radius: 16px;
            border: 1px solid rgba(255,255,255,0.1);
            box-shadow: 0 8px 32px rgba(0,0,0,0.6);
            backdrop-filter: blur(20px);
            min-width: 320px;
            max-width: 360px;
            max-height: calc(100vh - 40px);
            overflow-y: auto;
        }}
        
        .control-panel::-webkit-scrollbar {{ width: 6px; }}
        .control-panel::-webkit-scrollbar-track {{ background: rgba(255,255,255,0.05); border-radius: 3px; }}
        .control-panel::-webkit-scrollbar-thumb {{ background: rgba(255,255,255,0.2); border-radius: 3px; }}
        
        .panel-header {{
            display: flex;
            align-items: center;
            gap: 10px;
            margin-bottom: 16px;
            padding-bottom: 16px;
            border-bottom: 1px solid rgba(255,255,255,0.1);
        }}
        
        .panel-header h2 {{
            margin: 0;
            font-size: 16px;
            font-weight: 700;
            color: #fff;
            flex: 1;
        }}
        
        .badge {{
            font-size: 10px;
            padding: 4px 8px;
            border-radius: 4px;
            font-weight: 600;
            text-transform: uppercase;
        }}
        
        .badge-satellite {{ background: rgba(59, 130, 246, 0.2); color: #3b82f6; }}
        .badge-dark {{ background: rgba(156, 163, 175, 0.2); color: #9ca3af; }}
        
        /* Section Styling */
        .section {{
            margin-bottom: 16px;
            padding-bottom: 16px;
            border-bottom: 1px solid rgba(255,255,255,0.06);
        }}
        
        .section:last-child {{
            margin-bottom: 0;
            padding-bottom: 0;
            border-bottom: none;
        }}
        
        .section-title {{
            font-size: 10px;
            color: rgba(255,255,255,0.4);
            text-transform: uppercase;
            letter-spacing: 1.2px;
            margin-bottom: 12px;
            display: flex;
            align-items: center;
            gap: 6px;
        }}
        
        .section-title::before {{
            content: '';
            width: 3px;
            height: 10px;
            background: #3b82f6;
            border-radius: 2px;
        }}
        
        /* Dropdown Styling */
        .control-group {{
            margin-bottom: 12px;
        }}
        
        .control-group label {{
            display: block;
            font-size: 11px;
            color: rgba(255,255,255,0.5);
            margin-bottom: 6px;
        }}
        
        .control-group select {{
            width: 100%;
            padding: 10px 12px;
            font-size: 13px;
            font-family: 'Inter', sans-serif;
            background: rgba(255,255,255,0.08);
            border: 1px solid rgba(255,255,255,0.15);
            border-radius: 8px;
            color: #fff;
            cursor: pointer;
            appearance: none;
            background-image: url("data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' width='12' height='12' viewBox='0 0 12 12'%3E%3Cpath fill='%23ffffff' d='M6 8L1 3h10z'/%3E%3C/svg%3E");
            background-repeat: no-repeat;
            background-position: right 12px center;
            transition: all 0.2s;
        }}
        
        .control-group select:hover {{
            background: rgba(255,255,255,0.12);
            border-color: rgba(255,255,255,0.25);
        }}
        
        .control-group select:focus {{
            outline: none;
            border-color: #3b82f6;
            box-shadow: 0 0 0 3px rgba(59, 130, 246, 0.2);
        }}
        
        /* Stats Grid */
        .stat-grid {{
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 10px;
        }}
        
        .stat-item {{
            background: rgba(255,255,255,0.04);
            padding: 10px 12px;
            border-radius: 8px;
            border: 1px solid rgba(255,255,255,0.06);
        }}
        
        .stat-label {{
            font-size: 9px;
            color: rgba(255,255,255,0.4);
            text-transform: uppercase;
            letter-spacing: 0.5px;
            margin-bottom: 4px;
        }}
        
        .stat-value {{
            font-size: 15px;
            font-weight: 600;
            color: #fff;
            font-family: 'JetBrains Mono', monospace;
        }}
        
        .stat-value.positive {{ color: #22c55e; }}
        .stat-value.negative {{ color: #ef4444; }}
        .stat-value.warning {{ color: #eab308; }}
        .stat-value.info {{ color: #3b82f6; }}
        
        /* Legend */
        .legend {{
            background: rgba(255,255,255,0.04);
            padding: 12px;
            border-radius: 8px;
            border: 1px solid rgba(255,255,255,0.06);
        }}
        
        .legend-title {{
            font-size: 11px;
            color: rgba(255,255,255,0.6);
            font-weight: 500;
            margin-bottom: 8px;
        }}
        
        .legend-bar {{
            height: 10px;
            border-radius: 5px;
            margin-bottom: 6px;
        }}
        
        .legend-labels {{
            display: flex;
            justify-content: space-between;
            font-size: 10px;
            color: rgba(255,255,255,0.5);
            font-family: 'JetBrains Mono', monospace;
        }}
        
        .legend-description {{
            font-size: 10px;
            color: rgba(255,255,255,0.4);
            margin-top: 8px;
            font-style: italic;
            line-height: 1.4;
        }}

        .endpoint-legend {{
            margin-top: 8px;
            display: flex;
            gap: 10px;
            align-items: center;
            font-size: 10px;
            color: rgba(156, 163, 175, 0.9);
        }}

        .endpoint-dot {{
            width: 10px;
            height: 10px;
            border-radius: 999px;
            border: 1px solid rgba(15,23,42,0.9);
        }}
        
        /* Action Distribution */
        .action-bar {{
            display: flex;
            height: 8px;
            border-radius: 4px;
            overflow: hidden;
            margin-top: 8px;
        }}
        
        .action-segment {{
            transition: width 0.3s;
        }}
        
        .action-labels {{
            display: flex;
            justify-content: space-between;
            margin-top: 6px;
            font-size: 9px;
        }}
        
        .action-label {{
            display: flex;
            align-items: center;
            gap: 4px;
            color: rgba(255,255,255,0.5);
        }}
        
        .action-dot {{
            width: 6px;
            height: 6px;
            border-radius: 50%;
        }}
        
        /* Tooltip container */
        #tooltip {{
            position: absolute;
            z-index: 2000;
            pointer-events: none;
            font-family: 'Inter', sans-serif;
        }}
    </style>
</head>
<body>
    <div id="map"></div>
    <canvas id="deck-canvas"></canvas>
    <div id="tooltip"></div>
    
    <div class="control-panel">
        <div class="panel-header">
            <h2>🏎️ {title}</h2>
            <span class="badge {'badge-satellite' if use_mapbox else 'badge-dark'}">{'🛰️ SAT' if use_mapbox else '🌙 DARK'}</span>
        </div>
        
        <!-- Color Mode Selection -->
        <div class="section">
            <div class="section-title">Visualization Mode</div>
            <div class="control-group">
                <select id="colorMode">
                    <option value="speed" {'selected' if initial_color_by == 'speed' else ''}>🚗 Speed</option>
                    <option value="energy" {'selected' if initial_color_by == 'energy' else ''}>⚡ Energy Intensity</option>
                    <option value="lateral_g" {'selected' if initial_color_by == 'lateral_g' else ''}>📐 Lateral G-Force</option>
                    <option value="friction" {'selected' if initial_color_by == 'friction' else ''}>🛞 Friction Usage (%)</option>
                    <option value="comfort" {'selected' if initial_color_by == 'comfort' else ''}>💚 Comfort Score</option>
                    <option value="difficulty" {'selected' if initial_color_by == 'difficulty' else ''}>⛰️ Difficulty</option>
                    <option value="safety_margin" {'selected' if initial_color_by == 'safety_margin' else ''}>🛡️ Safety Margin</option>
                    <option value="regen" {'selected' if initial_color_by == 'regen' else ''}>🔋 Regen Potential</option>
                    <option value="action" {'selected' if initial_color_by == 'action' else ''}>🎯 Recommended Action</option>
                </select>
            </div>

            <div class="control-group" style="margin-top: 8px;">
                <label for="unitMode" style="font-size: 10px; color: rgba(255,255,255,0.5); text-transform: uppercase; letter-spacing: 1px;">
                    Units
                </label>
                <select id="unitMode">
                    <option value="metric">Kilometers (km, km/h)</option>
                    <option value="imperial">Miles (mi, mph)</option>
                </select>
            </div>
            
            <div class="legend">
                <div class="legend-title" id="legendTitle">Speed (km/h)</div>
                <div class="legend-bar" id="legendBar" style="background: linear-gradient(90deg, #dc3545, #ffc107, #28a745);"></div>
                <div class="legend-labels">
                    <span id="legendLow">Slow</span>
                    <span id="legendHigh">Fast</span>
                </div>
                <div class="legend-description" id="legendDesc">Color intensity shows vehicle speed along the route</div>
                <div class="endpoint-legend">
                    <div style="display:flex;align-items:center;gap:4px;">
                        <span class="endpoint-dot" style="background:#38bdf8;"></span>
                        <span>Start</span>
                    </div>
                    <div style="display:flex;align-items:center;gap:4px;">
                        <span class="endpoint-dot" style="background:#ef4444;"></span>
                        <span>End</span>
                    </div>
                </div>
            </div>
        </div>
        
        <!-- Route Statistics -->
        <div class="section">
            <div class="section-title">Route Statistics</div>
            <div class="stat-grid" id="routeStats"></div>
        </div>
        
        <!-- Dynamics & Safety -->
        <div class="section">
            <div class="section-title">Dynamics & Safety</div>
            <div class="stat-grid" id="dynamicsStats"></div>
        </div>
        
        <!-- Energy Analysis -->
        <div class="section">
            <div class="section-title">Energy Analysis</div>
            <div class="stat-grid" id="energyStats"></div>
        </div>
        
        <!-- Action Distribution -->
        <div class="section">
            <div class="section-title">Driving Actions</div>
            <div class="action-bar" id="actionBar"></div>
            <div class="action-labels" id="actionLabels"></div>
        </div>
    </div>
    
    <script>
        // Data from Python
        const allModeData = {mode_data_json};
        const stats = {hud_stats_json};
        const MAPBOX_TOKEN = '{mapbox_token}';
        const USE_MAPBOX = {'true' if use_mapbox else 'false'};
        const ROUTE_START = {{ lat: {start_lat}, lon: {start_lon} }};
        const ROUTE_END = {{ lat: {end_lat}, lon: {end_lon} }};
        
        let currentMode = '{initial_color_by}';
        let map = null;
        let deckOverlay = null;
        let currentUnits = 'metric';  // 'metric' (km/kmh) or 'imperial' (mi/mph)
        
        // Legend configurations
        const legendConfigs = {{
            speed: {{
                title: 'Speed (km/h)',
                low: 'Slow', high: 'Fast',
                gradient: 'linear-gradient(90deg, #dc3545, #ffc107, #28a745)',
                desc: 'Color shows vehicle speed - red is slow/braking, green is fast'
            }},
            energy: {{
                title: 'Energy Intensity (Wh/km)',
                low: 'Efficient', high: 'High Cost',
                gradient: 'linear-gradient(90deg, #22c55e, #84cc16, #eab308, #f97316, #ef4444)',
                desc: 'Green = low energy use (regen possible), red = high energy cost'
            }},
            regen: {{
                title: 'Regen Potential',
                low: 'Low', high: 'High',
                gradient: 'linear-gradient(90deg, #9ca3af, #3b82f6, #22c55e)',
                desc: 'How much braking energy can be recovered along the route'
            }},
            lateral_g: {{
                title: 'Lateral G-Force',
                low: '0.0g', high: '1.0g+',
                gradient: 'linear-gradient(90deg, #3b82f6, #8b5cf6, #d946ef, #ef4444)',
                desc: 'Cornering intensity - blue is gentle, purple/red is aggressive'
            }},
            friction: {{
                title: 'Friction Usage (%)',
                low: '0% (Safe)', high: '100% (Limit)',
                gradient: 'linear-gradient(90deg, #22c55e, #84cc16, #eab308, #f97316, #ef4444)',
                desc: 'Tire grip utilization - green = headroom, red = at limit'
            }},
            comfort: {{
                title: 'Comfort Score',
                low: 'Comfortable', high: 'Uncomfortable',
                gradient: 'linear-gradient(90deg, #22c55e, #84cc16, #eab308, #f97316, #ef4444)',
                desc: 'Passenger comfort based on G-forces, jerk, and braking'
            }},
            difficulty: {{
                title: 'Segment Difficulty',
                low: 'Easy', high: 'Challenging',
                gradient: 'linear-gradient(90deg, #3b82f6, #8b5cf6, #d946ef, #ef4444)',
                desc: 'Driving difficulty combining curvature, grade, and grip demand'
            }},
            safety_margin: {{
                title: 'Safety Margin',
                low: 'Low (Danger)', high: 'High (Safe)',
                gradient: 'linear-gradient(90deg, #ef4444, #f97316, #eab308, #84cc16, #22c55e)',
                desc: 'Available grip headroom - red = near limit, green = safe'
            }},
            action: {{
                title: 'Recommended Action',
                low: 'BRAKE', high: 'ACCELERATE',
                gradient: 'linear-gradient(90deg, #ef4444, #3b82f6, #22c55e)',
                desc: 'Red = brake hard, Blue = coast/maintain, Green = accelerate'
            }},
        }};
        
        function updateLegend(mode) {{
            const config = legendConfigs[mode] || legendConfigs.speed;
            let title = config.title;
            let desc = config.desc;

            if (mode === 'speed') {{
                // Use string concatenation for compatibility with older JS engines.
                var unitLabel = currentUnits === 'imperial' ? 'mph' : 'km/h';
                title = 'Speed (' + unitLabel + ')';
                desc = 'Color shows vehicle speed along the route (' + unitLabel + ')';
            }}

            document.getElementById('legendTitle').textContent = title;
            document.getElementById('legendBar').style.background = config.gradient;
            document.getElementById('legendLow').textContent = config.low;
            document.getElementById('legendHigh').textContent = config.high;
            document.getElementById('legendDesc').textContent = desc;
        }}
        
        function formatNumber(val, decimals = 1) {{
            if (val === undefined || val === null) return '—';
            return typeof val === 'number' ? val.toFixed(decimals) : val;
        }}
        
        function updateAllStats() {{
            const useImperial = currentUnits === 'imperial';
            const distanceKm = stats.distance_km || 0;
            const avgSpeedKmh = stats.avg_speed || 0;
            const maxSpeedKmh = stats.max_speed || 0;
            const avgLimitKmh = stats.avg_speed_limit_kmh || 0;
            const p95LimitKmh = stats.p95_speed_limit_kmh || 0;

            const distanceVal = useImperial ? distanceKm * 0.621371 : distanceKm;
            const distanceUnit = useImperial ? 'mi' : 'km';

            const avgSpeedVal = useImperial ? avgSpeedKmh * 0.621371 : avgSpeedKmh;
            const maxSpeedVal = useImperial ? maxSpeedKmh * 0.621371 : maxSpeedKmh;
            const speedUnit = useImperial ? 'mph' : 'km/h';
            const avgLimitVal = useImperial ? avgLimitKmh * 0.621371 : avgLimitKmh;
            const p95LimitVal = useImperial ? p95LimitKmh * 0.621371 : p95LimitKmh;

            const energyKwh = stats.energy_kwh || 0;
            const distForEnergy = distanceKm > 0 ? distanceKm : 1;
            const whPerKm = (energyKwh * 1000) / distForEnergy;
            const whPerDist = useImperial ? whPerKm / 0.621371 : whPerKm;
            const whUnit = useImperial ? 'Wh/mi' : 'Wh/km';

            const startElev = stats.start_elev_m || 0;
            const endElev = stats.end_elev_m || 0;
            // Python provides net_elev_m = end_elev - start_elev. Positive
            // values mean we *climb* along the route (uphill), negative values
            // mean we *descend* (downhill).
            const rawNetElev = (typeof stats.net_elev_m === 'number') ? stats.net_elev_m : (endElev - startElev);
            const netElev = rawNetElev;
            const elevDeltaM = Math.abs(netElev || 0);
            const elevDelta = useImperial ? elevDeltaM * 3.28084 : elevDeltaM;
            const elevUnit = useImperial ? 'ft' : 'm';
            let direction = 'Mostly flat';
            if (elevDeltaM > 0.5) {{
                direction = netElev > 0 ? 'Downhill' : 'Uphill';
            }}
            // Route Statistics
            var distanceUnitLabel = useImperial ? 'mi' : 'km';
            var speedUnitLabel = useImperial ? 'mph' : 'km/h';
            document.getElementById('routeStats').innerHTML = ''
                + '<div class=\"stat-item\">'
                +   '<div class=\"stat-label\">Distance</div>'
                +   '<div class=\"stat-value\">' + formatNumber(distanceVal, 2) + ' ' + distanceUnitLabel + '</div>'
                + '</div>'
                + '<div class=\"stat-item\">'
                +   '<div class=\"stat-label\">Est. Time</div>'
                +   '<div class=\"stat-value\">' + formatNumber(stats.time_min, 1) + ' min</div>'
                + '</div>'
                + '<div class=\"stat-item\">'
                +   '<div class=\"stat-label\">Avg Speed</div>'
                +   '<div class=\"stat-value info\">' + formatNumber(avgSpeedVal, 0) + ' ' + speedUnitLabel + '</div>'
                + '</div>'
                + '<div class=\"stat-item\">'
                +   '<div class=\"stat-label\">Max Speed</div>'
                +   '<div class=\"stat-value info\">' + formatNumber(maxSpeedVal, 0) + ' ' + speedUnitLabel + '</div>'
                + '</div>'
                + '<div class=\"stat-item\">'
                +   '<div class=\"stat-label\">Avg Speed Limit</div>'
                +   '<div class=\"stat-value\">' + (avgLimitKmh > 0 ? formatNumber(avgLimitVal, 0) + ' ' + speedUnitLabel : '—') + '</div>'
                + '</div>'
                + '<div class=\"stat-item\">'
                +   '<div class=\"stat-label\">P95 Speed Limit</div>'
                +   '<div class=\"stat-value\">' + (p95LimitKmh > 0 ? formatNumber(p95LimitVal, 0) + ' ' + speedUnitLabel : '—') + '</div>'
                + '</div>'
                + '<div class=\"stat-item\">'
                +   '<div class=\"stat-label\">Direction</div>'
                +   '<div class=\"stat-value\">' + direction + ' (' + elevDelta.toFixed(1) + ' ' + elevUnit + ')</div>'
                + '</div>';
            
            // Dynamics & Safety
            const frictionClass = stats.max_friction > 90 ? 'negative' : stats.max_friction > 70 ? 'warning' : 'positive';
            const safetyClass = stats.safety_margin < 0.2 ? 'negative' : stats.safety_margin < 0.4 ? 'warning' : 'positive';
            const difficultyClass = stats.difficulty > 0.7 ? 'negative' : stats.difficulty > 0.4 ? 'warning' : 'positive';
            
            document.getElementById('dynamicsStats').innerHTML = `
                <div class="stat-item">
                    <div class="stat-label">Max Lateral G</div>
                    <div class="stat-value">${{formatNumber(stats.max_lat_g, 2)}}g</div>
                </div>
                <div class="stat-item">
                    <div class="stat-label">p95 Lateral G</div>
                    <div class="stat-value">${{formatNumber(stats.p95_lat_g, 2)}}g</div>
                </div>
                <div class="stat-item">
                    <div class="stat-label">Max Friction</div>
                    <div class="stat-value ${{frictionClass}}">${{formatNumber(stats.max_friction, 0)}}%</div>
                </div>
                <div class="stat-item">
                    <div class="stat-label">Avg Friction</div>
                    <div class="stat-value">${{formatNumber(stats.avg_friction, 0)}}%</div>
                </div>
                <div class="stat-item">
                    <div class="stat-label">Difficulty</div>
                    <div class="stat-value ${{difficultyClass}}">${{formatNumber(stats.difficulty, 2)}}</div>
                </div>
                <div class="stat-item">
                    <div class="stat-label">Safety Margin</div>
                    <div class="stat-value ${{safetyClass}}">${{formatNumber(stats.safety_margin, 2)}}</div>
                </div>
            `;
            
            // Energy Analysis
            const regenPotential = stats.regen_kwh || 0;
            
            document.getElementById('energyStats').innerHTML = ''
                + '<div class=\"stat-item\">'
                +   '<div class=\"stat-label\">Total Energy</div>'
                +   '<div class=\"stat-value\">' + formatNumber(energyKwh, 3) + ' kWh</div>'
                + '</div>'
                + '<div class=\"stat-item\">'
                +   '<div class=\"stat-label\">Energy per distance</div>'
                +   '<div class=\"stat-value\">' + formatNumber(whPerDist, 0) + ' ' + (useImperial ? 'Wh/mi' : 'Wh/km') + '</div>'
                + '</div>'
                + '<div class=\"stat-item\">'
                +   '<div class=\"stat-label\">Regen Potential</div>'
                +   '<div class=\"stat-value positive\">' + formatNumber(regenPotential, 3) + ' kWh</div>'
                + '</div>'
                + '<div class=\"stat-item\">'
                +   '<div class=\"stat-label\">Max Grade</div>'
                +   '<div class=\"stat-value\">' + formatNumber(stats.max_grade || 0, 1) + '%</div>'
                + '</div>';
            
            // Action Distribution Bar
            const brake = stats.pct_brake || 0;
            const coast = stats.pct_coast || 0;
            const accel = stats.pct_accel || 0;
            
            document.getElementById('actionBar').innerHTML = `
                <div class="action-segment" style="width: ${{brake}}%; background: #ef4444;"></div>
                <div class="action-segment" style="width: ${{coast}}%; background: #3b82f6;"></div>
                <div class="action-segment" style="width: ${{accel}}%; background: #22c55e;"></div>
            `;
            
            document.getElementById('actionLabels').innerHTML = `
                <div class="action-label">
                    <div class="action-dot" style="background: #ef4444;"></div>
                    Brake ${{brake.toFixed(0)}}%
                </div>
                <div class="action-label">
                    <div class="action-dot" style="background: #3b82f6;"></div>
                    Coast ${{coast.toFixed(0)}}%
                </div>
                <div class="action-label">
                    <div class="action-dot" style="background: #22c55e;"></div>
                    Accel ${{accel.toFixed(0)}}%
                </div>
            `;
        }}
        
        function createPathLayer(mode) {{
            const data = allModeData[mode] || allModeData['speed'] || [];
            
            return new deck.PathLayer({{
                id: 'path-layer-' + mode,
                data: data,
                getPath: d => d.path,
                getColor: d => d.color || [255, 0, 0, 255],
                getWidth: {path_width},
                widthUnits: 'meters',
                widthMinPixels: 4,
                widthMaxPixels: 25,
                capRounded: true,
                jointRounded: true,
                pickable: true,
                autoHighlight: true,
                highlightColor: [255, 255, 255, 120],
            }});
        }}
        
        function createEndpointLayers() {{
            const data = [
                {{ position: [ROUTE_START.lon, ROUTE_START.lat, 0], label: 'Start' }},
                {{ position: [ROUTE_END.lon, ROUTE_END.lat, 0], label: 'End' }},
            ];

            const dots = new deck.ScatterplotLayer({{
                id: 'route-endpoints',
                data,
                getPosition: d => d.position,
                getRadius: 14,
                radiusUnits: 'pixels',
                getFillColor: d => d.label === 'Start' ? [56, 189, 248, 240] : [239, 68, 68, 240],
                strokeWidth: 2,
                getLineColor: [17, 24, 39, 255],
                pickable: true,
            }});

            const labels = new deck.TextLayer({{
                id: 'route-endpoint-labels',
                data,
                getPosition: d => d.position,
                getText: d => d.label,
                getColor: [255, 255, 255, 255],
                getSize: 14,
                sizeUnits: 'pixels',
                getTextAnchor: 'middle',
                getAlignmentBaseline: 'center',
                fontFamily: 'Inter, system-ui, -apple-system, BlinkMacSystemFont, sans-serif',
                pickable: false,
            }});

            return [dots, labels];
        }}

        function updateColorMode() {{
            currentMode = document.getElementById('colorMode').value;
            updateLegend(currentMode);
            
            if (deckOverlay) {{
                const endpointLayers = createEndpointLayers();
                deckOverlay.setProps({{
                    layers: [createPathLayer(currentMode), ...endpointLayers]
                }});
            }}
        }}
        
        function showTooltip(info, event) {{
            const tooltip = document.getElementById('tooltip');
            if (!info.object) {{
                tooltip.style.display = 'none';
                return;
            }}
            
            const obj = info.object;

            // Special handling for route endpoints (Start/End markers)
            if (obj.label === 'Start' || obj.label === 'End') {{
                var label = obj.label === 'Start' ? 'Route Start' : 'Route End';
                tooltip.innerHTML = `
                    <div style="background: rgba(10,15,25,0.96); padding: 10px 14px; border-radius: 10px;
                                border: 1px solid rgba(255,255,255,0.15); box-shadow: 0 8px 24px rgba(0,0,0,0.45);
                                min-width: 160px; font-family: Inter, sans-serif;">
                        <div style="font-size: 13px; font-weight: 600; color: #fff; margin-bottom: 4px;">` + label + `</div>
                        <div style="font-size: 11px; color: #9ca3af;">` + (obj.label === 'Start' ? 'Blue marker' : 'Red marker') + ` on Lombard route</div>
                    </div>
                `;
                tooltip.style.display = 'block';
                tooltip.style.left = (event.clientX + 15) + 'px';
                tooltip.style.top = (event.clientY + 15) + 'px';
                return;
            }}

            let radiusDisplay = obj.curvature_radius_m;
            if (!radiusDisplay || radiusDisplay === 'Infinity' || radiusDisplay > 5000) {{
                radiusDisplay = 'straight';
            }} else {{
                radiusDisplay = Math.round(radiusDisplay) + 'm';
            }}

            const useImperial = currentUnits === 'imperial';
            let speedVal = obj.speed_kmh || 0;
            let speedUnit = 'km/h';
            if (useImperial) {{
                speedVal = speedVal * 0.621371;
                speedUnit = 'mph';
            }}

            // Posted legal speed limit (if available)
            let limitVal = null;
            let limitUnit = speedUnit;
            if (obj.speed_limit_kmh && obj.speed_limit_kmh !== 'N/A') {{
                limitVal = obj.speed_limit_kmh;
                if (useImperial) {{
                    limitVal = limitVal * 0.621371;
                }}
            }}

            let energyVal = obj.energy_kwh_per_km || 0;
            let energyUnit = 'Wh/km';
            if (useImperial) {{
                energyVal = energyVal / 0.621371;
                energyUnit = 'Wh/mi';
            }}
            
            tooltip.innerHTML = `
                <div style="background: rgba(10,15,25,0.96); padding: 14px 18px; border-radius: 12px;
                            border: 1px solid rgba(255,255,255,0.15); box-shadow: 0 8px 32px rgba(0,0,0,0.5);
                            min-width: 240px; font-family: Inter, sans-serif;">
                    <div style="font-size: 24px; font-weight: 700; color: #fff; margin-bottom: 8px;">
                        ${{speedVal.toFixed(0)}} <span style="font-size: 14px; color: #666;">${{speedUnit}}</span>
                    </div>
                    <div style="display: grid; grid-template-columns: 1fr 1fr; gap: 8px; font-size: 11px;">
                        <div><span style="color: #666;">Limit:</span> <span style="color: #fff; font-weight: 500;">${{limitVal !== null ? limitVal.toFixed(0) + ' ' + limitUnit : '—'}} </span></div>
                        <div><span style="color: #666;">Lateral G:</span> <span style="color: #fff; font-weight: 500;">${{obj.lateral_g || '0.00'}}g</span></div>
                        <div><span style="color: #666;">Friction:</span> <span style="color: #fff; font-weight: 500;">${{obj.friction_usage || 0}}%</span></div>
                        <div><span style="color: #666;">Comfort:</span> <span style="color: #fff; font-weight: 500;">${{obj.comfort_score || '0.00'}}</span></div>
                        <div><span style="color: #666;">Energy:</span> <span style="color: #fff; font-weight: 500;">${{energyVal.toFixed(0)}} ${{energyUnit}}</span></div>
                        <div><span style="color: #666;">Radius:</span> <span style="color: #fff; font-weight: 500;">${{radiusDisplay}}</span></div>
                        <div><span style="color: #666;">Grade:</span> <span style="color: #fff; font-weight: 500;">${{obj.grade_percent || 0}}%</span></div>
                    </div>
                    <div style="margin-top: 10px; padding-top: 8px; border-top: 1px solid rgba(255,255,255,0.1);
                                display: flex; justify-content: space-between; align-items: center;">
                        <span style="font-size: 12px; color: ${{
                            obj.recommended_action === 'BRAKE' ? '#ef4444' :
                            obj.recommended_action === 'ACCELERATE' ? '#22c55e' : '#3b82f6'
                        }}; font-weight: 600;">${{obj.recommended_action || 'COAST'}}</span>
                        <span style="font-size: 10px; color: #666;">${{obj.surface_type || 'asphalt'}}</span>
                    </div>
                </div>
            `;
            
            tooltip.style.display = 'block';
            tooltip.style.left = (event.clientX + 15) + 'px';
            tooltip.style.top = (event.clientY + 15) + 'px';
        }}
        
        function hideTooltip() {{
            document.getElementById('tooltip').style.display = 'none';
        }}
        
        function initMap() {{
            if (USE_MAPBOX && MAPBOX_TOKEN) {{
                // Use Mapbox GL JS for satellite imagery
                mapboxgl.accessToken = MAPBOX_TOKEN;
                
                map = new mapboxgl.Map({{
                    container: 'map',
                    style: 'mapbox://styles/mapbox/satellite-streets-v12',
                    center: [{center_lon}, {center_lat}],
                    zoom: {zoom},
                    pitch: 45,
                    bearing: 15,
                    antialias: true
                }});
                
                map.on('load', function() {{
                    // Create deck.gl overlay
                    const endpointLayers = createEndpointLayers();
                    deckOverlay = new deck.MapboxOverlay({{
                        layers: [createPathLayer(currentMode), ...endpointLayers],
                        getTooltip: null  // We handle tooltips manually
                    }});
                    
                    map.addControl(deckOverlay);
                    map.addControl(new mapboxgl.NavigationControl());
                    
                    // Handle hover for tooltips
                    map.on('mousemove', function(e) {{
                        const pickInfo = deckOverlay.pickObject({{
                            x: e.point.x,
                            y: e.point.y,
                            radius: 5
                        }});
                        
                        if (pickInfo && pickInfo.object) {{
                            showTooltip(pickInfo, e.originalEvent);
                            map.getCanvas().style.cursor = 'pointer';
                        }} else {{
                            hideTooltip();
                            map.getCanvas().style.cursor = '';
                        }}
                    }});
                    
                    map.on('mouseout', hideTooltip);
                }});
            }} else {{
                // Fallback to deck.gl standalone with OSM tiles
                const INITIAL_VIEW_STATE = {{
                    latitude: {center_lat},
                    longitude: {center_lon},
                    zoom: {zoom},
                    pitch: 45,
                    bearing: 15,
                }};
                
                // Create OSM tile layer
                const tileLayer = new deck.TileLayer({{
                    id: 'osm-tiles',
                    data: 'https://c.tile.openstreetmap.org/{{z}}/{{x}}/{{y}}.png',
                    minZoom: 0,
                    maxZoom: 19,
                    tileSize: 256,
                    renderSubLayers: props => {{
                        const {{boundingBox}} = props.tile;
                        return new deck.BitmapLayer(props, {{
                            data: null,
                            image: props.data,
                            bounds: [boundingBox[0][0], boundingBox[0][1], boundingBox[1][0], boundingBox[1][1]]
                        }});
                    }}
                }});
                
                const endpointLayers = createEndpointLayers();
                deckOverlay = new deck.DeckGL({{
                    canvas: 'deck-canvas',
                    initialViewState: INITIAL_VIEW_STATE,
                    controller: true,
                    layers: [tileLayer, createPathLayer(currentMode), ...endpointLayers],
                    onHover: (info, event) => {{
                        if (info.object) {{
                            showTooltip(info, event.srcEvent);
                        }} else {{
                            hideTooltip();
                        }}
                    }}
                }});
                
                // Hide the map div for non-Mapbox mode
                document.getElementById('map').style.display = 'none';
                document.getElementById('deck-canvas').style.pointerEvents = 'auto';
            }}
            
            // Initialize UI
            updateLegend(currentMode);
            updateAllStats();
            
            // Add event listeners
            document.getElementById('colorMode').addEventListener('change', updateColorMode);
            const unitSelect = document.getElementById('unitMode');
            if (unitSelect) {{
                unitSelect.addEventListener('change', (e) => {{
                    currentUnits = e.target.value === 'imperial' ? 'imperial' : 'metric';
                    updateLegend(currentMode);
                    updateAllStats();
                }});
            }}
        }}
        
        // Initialize when page loads
        window.onload = initMap;
    </script>
</body>
</html>"""
