"""
Dataset export utilities for ApexVelocity.

This module is part of the stable 1.x public API.

It provides small, high-leverage helpers to turn one or more routes into
tabular datasets suitable for ML / AV training and benchmarking.
"""

from __future__ import annotations

from typing import List, Dict, Optional

import math

try:
    import pandas as pd

    _HAS_PANDAS = True
except ImportError:  # pragma: no cover - optional dependency
    _HAS_PANDAS = False
    pd = None  # type: ignore

from .routing import RouteRequest, build_route
from .api import solve as _solve
from .analysis import analyze_profile, convert_segments_for_viz
from .api import load_vehicle


def _attach_solver_results(
    path_points: List[Dict], vehicle: str, condition: str
) -> None:
    """
    Run the core solver and attach speed/energy fields to path_points in-place.

    This mirrors the behaviour used by the CLI but goes through the public
    `apexvelocity.api.solve` wrapper.
    """
    if len(path_points) < 2:
        return

    # Build (x, y, z) coordinates in meters and surface types.
    coords = []
    surfaces = []
    for pt in path_points:
        coords.append(
            (
                float(pt.get("x_m", 0.0)),
                float(pt.get("y_m", 0.0)),
                float(pt.get("z_m", pt.get("elevation_m", 0.0))),
            )
        )
        surfaces.append(pt.get("surface_type", "asphalt"))

    result = _solve(
        path=coords, surfaces=surfaces, vehicle=vehicle, condition=condition
    )

    vp = result.velocity_profile_mps or []
    ej = result.energy_joules or []

    for i, pt in enumerate(path_points):
        v = vp[i] if i < len(vp) else pt.get("speed_mps", 0.0)
        e_j = ej[i] if i < len(ej) else pt.get("energy_joules", 0.0)

        pt["speed_mps"] = v
        pt["v_profile"] = v
        pt["energy_joules"] = e_j
        pt["energy_kwh"] = e_j / 3.6e6 if e_j is not None else pt.get("energy_kwh", 0.0)


def export_routes_to_parquet(
    routes: List[RouteRequest],
    output_path: str,
    *,
    vehicle: str = "tesla_model_3",
    condition: str = "dry",
    extras: Optional[Dict[str, str]] = None,
) -> None:
    """
    Export one or more routes to a Parquet dataset of segment-level features.

    For each RouteRequest this will:
      - build_route
      - solve the velocity profile via the C++ core
      - analyze the profile for rich AV / ML features
      - convert segments to dicts and append them to a single table

    Args:
        routes: List of RouteRequest objects describing the routes to export.
        output_path: Path to the Parquet file to write.
        vehicle: Vehicle preset name for the solver.
        condition: "dry" or "wet".
        extras: Optional dict of extra metadata columns to attach to every row.

    Notes:
        This helper requires `pandas` (and a Parquet engine such as `pyarrow`
        or `fastparquet`) to be installed in your environment.
    """
    if not _HAS_PANDAS:  # pragma: no cover - import guard
        raise ImportError(
            "pandas is required for export_routes_to_parquet.\n"
            "Install with: pip install pandas pyarrow"
        )

    rows: List[Dict] = []

    # Determine vehicle mass for energy/regen analysis
    vehicle_params = load_vehicle(vehicle)
    vehicle_mass_kg = float(vehicle_params["mass_kg"]) if vehicle_params else 1500.0

    for idx, req in enumerate(routes):
        path_points = build_route(req)
        if len(path_points) < 2:
            continue

        _attach_solver_results(path_points, vehicle=vehicle, condition=condition)

        analysis = analyze_profile(
            path_points,
            condition=condition,
            vehicle_mass_kg=vehicle_mass_kg,
        )
        seg_dicts = convert_segments_for_viz(analysis)

        # Attach route-level metadata
        for seg in seg_dicts:
            seg["route_index"] = idx
            seg["route_start_lat"] = req.start[0]
            seg["route_start_lon"] = req.start[1]
            seg["route_end_lat"] = req.end[0]
            seg["route_end_lon"] = req.end[1]
            seg["vehicle"] = vehicle
            seg["condition"] = condition
            if extras:
                for k, v in extras.items():
                    seg[k] = v
            rows.append(seg)

    if not rows:
        # Still create an empty file with a minimal schema so downstream
        # tooling can fail gracefully.
        df = pd.DataFrame([])  # type: ignore
    else:
        df = pd.DataFrame(rows)  # type: ignore

    df.to_parquet(output_path, index=False)  # type: ignore
