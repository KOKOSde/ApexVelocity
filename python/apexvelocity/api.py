"""
High-level Python API for ApexVelocity.

This module provides a simplified interface for common operations.
"""

from typing import List, Dict, Optional, Tuple, Union, Callable
from dataclasses import dataclass, field
import math

# Try to import core module
try:
    from . import _apexvelocity_core as core

    _HAS_CORE = True
except ImportError:
    _HAS_CORE = False
    core = None


@dataclass
class PathPointData:
    """Python representation of a path point."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    curvature: float = 0.0
    distance_along: float = 0.0
    surface_type: str = "asphalt"


@dataclass
class SolveResultData:
    """Python representation of solve results."""

    success: bool = False
    error_message: str = ""
    velocity_profile_mps: List[float] = field(default_factory=list)
    static_limits_mps: List[float] = field(default_factory=list)
    energy_joules: List[float] = field(default_factory=list)
    segment_energy_joules: List[float] = field(default_factory=list)
    total_distance_m: float = 0.0
    total_energy_joules: float = 0.0
    total_energy_kwh: float = 0.0
    max_speed_mps: float = 0.0
    min_speed_mps: float = 0.0
    avg_speed_mps: float = 0.0


def _check_core():
    """Raise ImportError if core is not available."""
    if not _HAS_CORE:
        raise ImportError(
            "ApexVelocity C++ core not available. "
            "Please build with -DAPEXVELOCITY_BUILD_PYTHON=ON"
        )


def create_path(
    coordinates: List[Tuple[float, float, float]],
    surfaces: Optional[List[str]] = None,
    curvatures: Optional[List[float]] = None,
) -> List[PathPointData]:
    """
    Create a path from coordinates.

    Args:
        coordinates: List of (x, y, z) tuples in meters
        surfaces: Optional list of surface types (default: "asphalt")
        curvatures: Optional list of curvatures (default: 0.0)

    Returns:
        List of PathPointData objects
    """
    if len(coordinates) < 2:
        raise ValueError("Need at least 2 coordinates")

    path = []
    distance = 0.0

    for i, (x, y, z) in enumerate(coordinates):
        if i > 0:
            dx = x - coordinates[i - 1][0]
            dy = y - coordinates[i - 1][1]
            dz = z - coordinates[i - 1][2]
            distance += math.sqrt(dx * dx + dy * dy + dz * dz)

        surface = surfaces[i] if surfaces and i < len(surfaces) else "asphalt"
        curvature = curvatures[i] if curvatures and i < len(curvatures) else 0.0

        path.append(
            PathPointData(
                x=x,
                y=y,
                z=z,
                curvature=curvature,
                distance_along=distance,
                surface_type=surface,
            )
        )

    # If curvatures were not provided, recompute them from geometry so that
    # the physics core sees realistic curvature values and can enforce
    # friction/rollover limits correctly.
    if curvatures is None:
        try:
            from apexvelocity.geometry import recompute_curvature  # type: ignore
        except ImportError:
            # Geometry module not available; fall back to zero curvature.
            return path

        # Convert to dict format expected by recompute_curvature
        path_dicts = []
        for p in path:
            path_dicts.append(
                {
                    "x_m": p.x,
                    "y_m": p.y,
                    "z_m": p.z,
                    "distance_along_m": p.distance_along,
                    "curvature": p.curvature,
                }
            )

        # Recompute curvature with a small smoothing window
        path_dicts = recompute_curvature(path_dicts, smooth_window=3)

        # Update path with calculated curvatures
        for i, p in enumerate(path):
            p.curvature = float(path_dicts[i].get("curvature", 0.0))

    return path


def load_vehicle(name: str = "default") -> Optional[Dict]:
    """
    Load a vehicle preset.

    Args:
        name: Vehicle preset name (e.g., "tesla_model_3", "compact_car")

    Returns:
        Dictionary of vehicle parameters, or None if not found
    """
    _check_core()

    if name == "default":
        v = core.VehicleLoader.get_default()
    else:
        loader = core.VehicleLoader(core.DEFAULT_CONFIG_DIR + "/vehicle_presets")
        result = loader.load_preset(name + ".json")
        if result is None:
            return None
        v = result

    return {
        "name": v.name,
        "mass_kg": v.mass_kg,
        "drag_coeff": v.drag_coeff,
        "frontal_area_m2": v.frontal_area_m2,
        "rolling_res_base": v.rolling_res_base,
        "max_lat_g": v.max_lat_g,
        "max_brake_g": v.max_brake_g,
        "max_power_w": v.max_power_w,
        "powertrain_efficiency": v.powertrain_efficiency,
        "track_width_m": v.track_width_m,
        "cog_height_m": v.cog_height_m,
    }


def get_vehicle_presets() -> List[str]:
    """
    Get list of available vehicle presets.

    Returns:
        List of preset names (without .json extension)
    """
    _check_core()

    loader = core.VehicleLoader(core.DEFAULT_CONFIG_DIR + "/vehicle_presets")
    presets = loader.list_presets()
    return [p.replace(".json", "") for p in presets]


def solve(
    path: Union[List[PathPointData], List[Tuple[float, float, float]]],
    surfaces: Optional[List[str]] = None,
    vehicle: str = "default",
    condition: str = "dry",
    initial_speed: float = 0.0,
    final_speed: float = 0.0,
    physics_model: str = "kinematic",
) -> SolveResultData:
    """
    Solve velocity profile for a path.

    Args:
        path: List of PathPointData or (x, y, z) coordinate tuples
        surfaces: Surface types (only needed if path is coordinates)
        vehicle: Vehicle preset name
        condition: "dry" or "wet"
        initial_speed: Initial speed constraint (m/s)
        final_speed: Final speed constraint (m/s)
        physics_model: "kinematic" (fast, 3-pass) or "dynamic" (tire model, experimental)

    Returns:
        SolveResultData with velocity profile and energy data
    """
    _check_core()

    # Convert to PathPointData if needed
    if path and isinstance(path[0], tuple):
        path = create_path(path, surfaces)

    # Build geometry array for core
    geometry = []
    surface_list = []

    for pt in path:
        geometry.append([pt.x, pt.y, pt.z, pt.curvature, pt.distance_along])
        surface_list.append(pt.surface_type)

    # Call core solver
    result = core.solve_profile(
        geometry=geometry,
        surfaces=surface_list,
        vehicle=vehicle,
        condition=condition,
        initial_speed=initial_speed,
        final_speed=final_speed,
        physics_model=physics_model,
    )

    # Convert to Python dataclass
    return SolveResultData(
        success=result.get("success", False),
        error_message=result.get("error_message", ""),
        velocity_profile_mps=result.get("velocity_profile_mps", []),
        static_limits_mps=result.get("static_limits_mps", []),
        energy_joules=result.get("energy_joules", []),
        segment_energy_joules=result.get("segment_energy_joules", []),
        total_distance_m=result.get("total_distance_m", 0.0),
        total_energy_joules=result.get("total_energy_joules", 0.0),
        total_energy_kwh=result.get("total_energy_joules", 0.0) / 3_600_000,
        max_speed_mps=result.get("max_speed_mps", 0.0),
        min_speed_mps=result.get("min_speed_mps", 0.0),
        avg_speed_mps=result.get("avg_speed_mps", 0.0),
    )


def set_friction_callback_py(callback: Callable[[str, str], float]) -> None:
    """
    Set a Python callback for friction coefficient lookup.

    The callback will be called whenever the solver needs to look up
    friction for a surface/condition combination.

    Args:
        callback: Function taking (surface_name: str, condition: str) -> float

    Example:
        >>> def my_friction(surface: str, condition: str) -> float:
        ...     if surface == "ice":
        ...         return 0.05  # Very slippery!
        ...     return 0.8  # Default
        >>>
        >>> av.set_friction_callback_py(my_friction)
    """
    _check_core()
    core.set_friction_callback(callback)


def clear_friction_callback_py() -> None:
    """Clear the friction callback and use default values."""
    _check_core()
    core.clear_friction_callback()
