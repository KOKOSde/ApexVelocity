#!/usr/bin/env python3
"""
Minimal ApexVelocity benchmark suite.

This script demonstrates the canonical 1.x pipeline:

    RouteRequest -> build_route -> solve -> analyze_profile

and prints a compact table of key metrics for a few representative routes.
"""

import os
from typing import List, Tuple

import apexvelocity as av  # type: ignore


def _run_benchmark(
    name: str,
    start: Tuple[float, float],
    end: Tuple[float, float],
    place: str,
    vehicle: str = "tesla_model_3",
    condition: str = "dry",
) -> None:
    req = av.RouteRequest(
        start=start,
        end=end,
        place_hint=place,
        vehicle=vehicle,
        condition=condition,
        target_spacing_m=5.0,
    )

    print(f"\n=== {name} | {vehicle} | {condition} ===")
    path_points = av.build_route(req)
    if len(path_points) < 2:
        print("  ⚠ Could not build route (insufficient points)")
        return

    # Solve
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

    result = av.solve(
        path=coords, surfaces=surfaces, vehicle=vehicle, condition=condition
    )

    vp = result.velocity_profile_mps or []
    ej = result.energy_joules or []

    for i, pt in enumerate(path_points):
        v = vp[i] if i < len(vp) else pt.get("speed_mps", 0.0)
        e_j = ej[i] if i < len(e_j) else pt.get("energy_joules", 0.0)
        pt["speed_mps"] = v
        pt["v_profile"] = v
        pt["energy_joules"] = e_j
        pt["energy_kwh"] = e_j / 3.6e6 if e_j is not None else pt.get("energy_kwh", 0.0)

    # Analyze
    vehicle_params = av.load_vehicle(vehicle)
    mass = float(vehicle_params["mass_kg"]) if vehicle_params else 1500.0
    analysis = av.analyze_profile(
        path_points, condition=condition, vehicle_mass_kg=mass
    )
    s = analysis.get_summary_dict()

    print(
        f"  Distance:   {s['total_distance_km']:.2f} km  |  "
        f"Time: {s['lap_time_min']:.2f} min  |  "
        f"Energy: {s['total_energy_kwh']:.3f} kWh  "
        f"({s['energy_kwh_per_km']*1000:.0f} Wh/km)"
    )
    print(
        f"  Regen frac: {s['regen_fraction']*100:.0f}%  |  "
        f"Max/p95 lat G: {s['max_lateral_g']:.2f} / {s['p95_lateral_g']:.2f}  |  "
        f"Comfort viol/km: {s['comfort_violations_per_km']:.2f}"
    )
    print(
        f"  Difficulty: {s['difficulty_score']:.2f}  |  "
        f"Safety: {s['safety_margin_score']:.2f}"
    )


def main() -> int:
    print("\nApexVelocity Benchmark Suite\n" "============================\n")

    benchmarks: List[Tuple[str, Tuple[float, float], Tuple[float, float], str]] = [
        (
            "SF Lombard (uphill urban hill climb)",
            (37.802017, -122.418625),
            (37.802050, -122.420140),
            "San Francisco, California, USA",
        ),
        (
            "Manhattan corridor (flat grid)",
            (40.748817, -73.985428),  # Empire State Building
            (40.761432, -73.977622),  # Central Park South
            "Manhattan, New York, USA",
        ),
    ]

    for name, start, end, place in benchmarks:
        _run_benchmark(
            name, start, end, place, vehicle="tesla_model_3", condition="dry"
        )
        _run_benchmark(
            name, start, end, place, vehicle="tesla_model_3", condition="wet"
        )

    print("\nDone.\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
