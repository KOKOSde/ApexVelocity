"""
ApexVelocity - Physics-Based Graph Annotation Engine

This package provides Python bindings and high-level APIs for the ApexVelocity
core library, enabling physics-based velocity profiling and energy estimation
for road networks.

Stable 1.x public API (recommended entrypoints):
  - Routing & OSM:
      * RouteRequest
      * build_route
  - Core solver:
      * solve          (high-level wrapper around the C++ core)
      * solve_profile  (low-level geometry-based core binding; advanced use)
  - Analysis:
      * analyze_profile
      * ProfileAnalysis
      * SegmentFeatures
      * convert_segments_for_viz
  - Visualization:
      * visualize_profile
      * visualize_profile_interactive
      * visualize_comparison

Experimental modules (subject to change in 1.x):
  - apexvelocity.osm_fetcher
  - apexvelocity.envs (Gym/RL environments)
  - Debug/demo helpers under python/examples/
"""

import logging
import os

__version__ = "1.0.0"
__author__ = "ApexVelocity Team"

# Configure library logging early. Users can override this by configuring
# logging before importing apexvelocity or by adjusting APEXVELOCITY_LOG_LEVEL.
_log_level_name = os.environ.get("APEXVELOCITY_LOG_LEVEL", "INFO").upper()
_log_level = getattr(logging, _log_level_name, logging.INFO)

logging.basicConfig(
    level=_log_level,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)

logger = logging.getLogger("apexvelocity")

# Try to import the C++ extension module
try:
    from ._apexvelocity_core import (
        # Types
        MaterialProps,
        VehicleParams,
        PathPoint,
        SolverConfig,
        PhysicsConstants,
        VehicleLoader,
        Solver,
        # Config functions
        config_init,
        get_material,
        get_effective_mu,
        get_sim_param_double,
        get_sim_param_bool,
        # Callback functions
        set_friction_callback,
        clear_friction_callback,
        # Physics functions
        calc_friction_limited_speed,
        calc_rollover_limited_speed,
        curvature_to_radius,
        radius_to_curvature,
        mps_to_kmph,
        kmph_to_mps,
        # Solver
        solve_profile,
        # Constants
        DEFAULT_CONFIG_DIR,
    )
    _HAS_CORE = True
except ImportError as e:
    _HAS_CORE = False
    _IMPORT_ERROR = str(e)

# Re-export from api module (high-level public API)
from .api import (
    solve as _solve,
    create_path,
    load_vehicle,
    get_vehicle_presets,
)


def solve(*args, **kwargs):
    """
    High-level solver for velocity profiles (stable 1.x API).

    This wraps the C++ core solver via `apexvelocity.api.solve` and returns
    a `SolveResultData` dataclass with:
      - velocity_profile_mps
      - static_limits_mps
      - energy_joules / segment_energy_joules
      - total_distance_m / total_energy_kwh
      - max/min/avg speeds
    """
    return _solve(*args, **kwargs)

# Optional: loader module (requires osmnx)
try:
    from .loader import (
        OSMLoader,
        LoadedPath,
        LoadedPathPoint,
        load_path_from_place,
        load_path_from_bbox,
        create_simple_path,
    )
except ImportError:
    pass  # osmnx not installed

# OSM Fetcher module (uses Overpass API, no external deps).
# ⚠️ Experimental: this module is NOT part of the stable 1.x public API.
# It is intended for demos and utilities only and may change between
# minor releases.
from .osm_fetcher import (  # type: ignore
    fetch_way_by_name,
    fetch_way_by_bbox,
    fetch_relation_by_id,
    fetch_route_by_coordinates,
    fetch_lombard_street_sf,
    fetch_nurburgring_nordschleife,
    build_path_points,
    order_coordinates_as_path,
    calculate_curvature,
)

# Routing module (OSMLoader-based routing)
from .routing import (
    RouteRequest,
    build_route,
)

# Optional: visualization module (requires pydeck)
try:
    from .viz import (
        visualize_profile,
        visualize_profile_interactive,
        visualize_comparison,
    )
except ImportError:
    pass  # pydeck not installed

# Analysis module (always available)
from .analysis import (
    analyze_profile,
    ProfileAnalysis,
    SegmentFeatures,
    get_friction_coefficient,
    compute_physics_limit_speed,
    determine_recommended_action,
    convert_segments_for_viz,
)

# Geometry module (path smoothing, curvature)
from .geometry import (
    smooth_path_geo,
    recompute_curvature,
    get_path_stats,
    CRSTransformer,
)

def check_core_available():
    """Check if the C++ core library is available."""
    if not _HAS_CORE:
        raise ImportError(
            f"ApexVelocity C++ core not available: {_IMPORT_ERROR}\n"
            "Please build the Python bindings with:\n"
            "  cd core/build && cmake .. -DAPEXVELOCITY_BUILD_PYTHON=ON && make"
        )
    return True

__all__ = [
    # Version
    "__version__",
    # Types
    "MaterialProps",
    "VehicleParams", 
    "PathPoint",
    "SolverConfig",
    "PhysicsConstants",
    "VehicleLoader",
    "Solver",
    # Config
    "config_init",
    "get_material",
    "get_effective_mu",
    "get_sim_param_double",
    "get_sim_param_bool",
    # Callbacks
    "set_friction_callback",
    "clear_friction_callback",
    # Physics
    "calc_friction_limited_speed",
    "calc_rollover_limited_speed",
    "curvature_to_radius",
    "radius_to_curvature",
    "mps_to_kmph",
    "kmph_to_mps",
    # Solver
    "solve",
    "solve_profile",
    "solve_path",  # backwards-compatible alias for `solve`
    "create_path",
    "load_vehicle",
    "get_vehicle_presets",
    # Routing
    "RouteRequest",
    "build_route",
    # Utils
    "check_core_available",
    "DEFAULT_CONFIG_DIR",
]
