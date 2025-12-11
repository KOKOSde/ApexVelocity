"""
Validate ApexVelocity against the nuScenes dataset and export residuals.

This script:
- Walks nuScenes scenes and extracts the ego-vehicle trajectory.
- Runs the ApexVelocity solver along each trajectory.
- Compares predicted speed vs. nuScenes ego speed.
- Writes a residuals CSV suitable for uncertainty training.
- Prints aggregate RMSE / MAE / R^2 metrics.

Usage (example):

    cd python
    python -m validation.validate_nuscenes \\
        --datarroot /data/sets/nuscenes \\
        --version v1.0-mini \\
        --split train \\
        --max-scenes 10 \\
        --vehicle tesla_model_3 \\
        --condition dry \\
        --output validation/nuscenes_residuals.csv

Notes:
- You must install the nuScenes devkit separately:

      pip install nuscenes-devkit

- You must point --dataroot at a valid nuScenes root (containing 'samples/',
  'sweeps/', 'maps/', etc.). This script does NOT download data.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np  # type: ignore[import-untyped]
import pandas as pd  # type: ignore[import-untyped]

from apexvelocity import api as av_api


GRAVITY = 9.81


def _import_nuscenes():
    """Import nuScenes devkit lazily with a clear error if missing."""
    try:
        from nuscenes.nuscenes import NuScenes  # type: ignore[import-untyped]
        from nuscenes.utils.splits import create_splits_scenes  # type: ignore[import-untyped]
    except ImportError as e:  # pragma: no cover - environment dependent
        raise SystemExit(
            "nuScenes devkit not found. Install it with:\n"
            "  pip install nuscenes-devkit\n"
            f"Original error: {e}"
        )
    return NuScenes, create_splits_scenes


def _compute_curvature(xs: np.ndarray, ys: np.ndarray) -> np.ndarray:
    """
    Approximate curvature κ [1/m] from discrete XY coordinates.

    Uses a simple angle-change over arc-length approximation.
    """
    n = len(xs)
    kappa = np.zeros(n, dtype=float)
    if n < 3:
        return kappa

    for i in range(1, n - 1):
        x0, y0 = xs[i - 1], ys[i - 1]
        x1, y1 = xs[i], ys[i]
        x2, y2 = xs[i + 1], ys[i + 1]

        v1x, v1y = x1 - x0, y1 - y0
        v2x, v2y = x2 - x1, y2 - y1
        len1 = math.hypot(v1x, v1y)
        len2 = math.hypot(v2x, v2y)
        if len1 < 1e-3 or len2 < 1e-3:
            continue

        dot = v1x * v2x + v1y * v2y
        cos_theta = max(-1.0, min(1.0, dot / (len1 * len2)))
        theta = math.acos(cos_theta)
        s = 0.5 * (len1 + len2)
        if s > 1e-3:
            kappa[i] = theta / s

    # Endpoints inherit nearest interior curvature
    if n >= 2:
        kappa[0] = kappa[1]
        kappa[-1] = kappa[-2]
    return kappa


def _compute_grade(zs: np.ndarray, distances: np.ndarray) -> np.ndarray:
    """Compute grade [%] per point using elevation and arc-length."""
    n = len(zs)
    grade = np.zeros(n, dtype=float)
    if n < 2:
        return grade

    for i in range(1, n):
        dz = zs[i] - zs[i - 1]
        ds = distances[i] - distances[i - 1]
        if ds > 0.1:
            grade[i] = (dz / ds) * 100.0

    # Clamp pathological grades to a reasonable band
    max_abs = 30.0
    grade = np.clip(grade, -max_abs, max_abs)
    return grade


def _extract_ego_trajectory(
    nusc,
    scene: Dict,
    channel: str = "LIDAR_TOP",
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Extract ego pose (x,y,z) and timestamps for a nuScenes scene.

    Returns:
        times_s: np.ndarray [N] timestamps [s]
        xs, ys, zs: np.ndarray [N] ego positions in meters (global frame)
    """
    times_s: List[float] = []
    xs: List[float] = []
    ys: List[float] = []
    zs: List[float] = []

    sample_token = scene["first_sample_token"]
    while sample_token:
        sample = nusc.get("sample", sample_token)
        data = sample["data"]
        if channel not in data:
            # Fallback to a common camera channel if lidar is missing
            channel_key = "CAM_FRONT" if "CAM_FRONT" in data else list(data.keys())[0]
        else:
            channel_key = channel

        sd = nusc.get("sample_data", data[channel_key])
        ego_pose = nusc.get("ego_pose", sd["ego_pose_token"])

        t_s = sd["timestamp"] * 1e-6  # microseconds → seconds
        x, y, z = ego_pose["translation"]

        times_s.append(float(t_s))
        xs.append(float(x))
        ys.append(float(y))
        zs.append(float(z))

        sample_token = sample["next"]
        if not sample_token:
            break

    return (
        np.asarray(times_s, dtype=float),
        np.asarray(xs, dtype=float),
        np.asarray(ys, dtype=float),
        np.asarray(zs, dtype=float),
    )


def _compute_true_speeds(
    times_s: np.ndarray, xs: np.ndarray, ys: np.ndarray
) -> np.ndarray:
    """Compute ego speed [m/s] from positions and timestamps."""
    n = len(times_s)
    speeds = np.zeros(n, dtype=float)
    if n < 2:
        return speeds

    for i in range(1, n):
        dt = times_s[i] - times_s[i - 1]
        dx = xs[i] - xs[i - 1]
        dy = ys[i] - ys[i - 1]
        if dt <= 1e-3:
            speeds[i] = speeds[i - 1]
        else:
            speeds[i] = math.hypot(dx, dy) / dt
    speeds[0] = speeds[1] if n > 1 else 0.0
    return speeds


def _build_path_for_solver(
    xs: np.ndarray,
    ys: np.ndarray,
    zs: np.ndarray,
    curvatures: np.ndarray,
    surface_type: str = "asphalt",
) -> List[av_api.PathPointData]:
    """Create PathPointData list for the ApexVelocity solver."""
    # Normalize coordinates to start at origin for better conditioning
    x0, y0, z0 = xs[0], ys[0], zs[0]
    coords = [
        (float(x - x0), float(y - y0), float(z - z0)) for x, y, z in zip(xs, ys, zs)
    ]
    surfaces = [surface_type] * len(coords)
    curv_list = [float(k) for k in curvatures]
    return av_api.create_path(coords, surfaces=surfaces, curvatures=curv_list)


def validate_nuscenes(
    dataroot: str,
    version: str,
    split: str,
    max_scenes: int,
    vehicle: str,
    condition: str,
    output_csv: str,
) -> Dict[str, float]:
    """
    Run nuScenes validation and write residuals CSV.

    Returns aggregate metrics (RMSE, MAE, R^2) in speed_kmh.
    """
    NuScenes, create_splits_scenes = _import_nuscenes()

    nusc = NuScenes(version=version, dataroot=dataroot, verbose=True)
    split_scenes = create_splits_scenes()
    if split not in split_scenes:
        raise SystemExit(
            f"Unknown split '{split}'. Available: {list(split_scenes.keys())}"
        )

    target_scene_names = set(split_scenes[split])
    scenes = [s for s in nusc.scene if s["name"] in target_scene_names]
    if max_scenes > 0:
        scenes = scenes[:max_scenes]

    rows: List[Dict[str, float]] = []

    for scene in scenes:
        scene_name = scene["name"]
        print(f"[nuScenes] Scene: {scene_name}")

        times_s, xs, ys, zs = _extract_ego_trajectory(nusc, scene)
        if len(times_s) < 5:
            print("  Skipping (too few samples).")
            continue

        # Geometry & ground-truth speed
        xs_arr = xs.astype(float)
        ys_arr = ys.astype(float)
        zs_arr = zs.astype(float)

        kappa = _compute_curvature(xs_arr, ys_arr)

        # Distances along path (Euclidean)
        distances = np.zeros_like(xs_arr)
        for i in range(1, len(xs_arr)):
            dx = xs_arr[i] - xs_arr[i - 1]
            dy = ys_arr[i] - ys_arr[i - 1]
            dz = zs_arr[i] - zs_arr[i - 1]
            distances[i] = distances[i - 1] + math.sqrt(dx * dx + dy * dy + dz * dz)

        grade = _compute_grade(zs_arr, distances)
        v_true_mps = _compute_true_speeds(times_s, xs_arr, ys_arr)

        # Build path and run ApexVelocity solver
        path_pts = _build_path_for_solver(xs_arr, ys_arr, zs_arr, kappa)
        solve_res = av_api.solve(
            path_pts,
            vehicle=vehicle,
            condition=condition,
            initial_speed=0.0,
            final_speed=0.0,
        )

        v_pred_mps = np.asarray(solve_res.velocity_profile_mps, dtype=float)
        v_static_mps = np.asarray(solve_res.static_limits_mps, dtype=float)
        energy_joules = np.asarray(solve_res.energy_joules, dtype=float)

        n = min(len(v_true_mps), len(v_pred_mps), len(v_static_mps), len(energy_joules))
        if n < 5:
            print("  Skipping (solver produced too few points).")
            continue

        v_true_mps = v_true_mps[:n]
        v_pred_mps = v_pred_mps[:n]
        v_static_mps = v_static_mps[:n]
        energy_joules = energy_joules[:n]
        distances_n = distances[:n]
        kappa_n = kappa[:n]
        grade_n = grade[:n]

        # Convert to km/h
        v_true_kmh = v_true_mps * 3.6
        v_pred_kmh = v_pred_mps * 3.6
        physics_limit_kmh = v_static_mps * 3.6

        # Approximate effective friction μ from static limits where curvature > 0
        mu_eff = np.zeros(n, dtype=float)
        for i in range(n):
            if kappa_n[i] > 1e-6 and v_static_mps[i] > 0.1:
                mu_eff[i] = (v_static_mps[i] ** 2) * kappa_n[i] / GRAVITY

        # Per-segment energy intensity [kWh/km]
        energy_kwh_per_km = np.zeros(n, dtype=float)
        for i in range(1, n):
            dE = energy_joules[i] - energy_joules[i - 1]
            ds = distances_n[i] - distances_n[i - 1]
            if ds > 0.1:
                j_per_m = dE / ds
                # Convert J/m → kWh/km: (J/m) * (1 kWh / 3.6e6 J) * 1000 m/km
                energy_kwh_per_km[i] = j_per_m * (1000.0 / 3.6e6)

        for i in range(n):
            rows.append(
                {
                    "scene": scene_name,
                    "index": float(i),
                    "distance_along_m": float(distances_n[i]),
                    "speed_kmh_true": float(v_true_kmh[i]),
                    "speed_kmh_pred": float(v_pred_kmh[i]),
                    "curvature_1pm": float(kappa_n[i]),
                    "grade_percent": float(grade_n[i]),
                    "physics_limit_kmh": float(physics_limit_kmh[i]),
                    "friction_mu_effective": float(mu_eff[i]),
                    "energy_kwh_per_km": float(energy_kwh_per_km[i]),
                }
            )

    if not rows:
        raise SystemExit(
            "No residuals collected. Check nuScenes dataroot/version/split."
        )

    out_path = Path(output_csv)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    df = pd.DataFrame(rows)
    df.to_csv(out_path, index=False)

    # Aggregate metrics
    v_true = df["speed_kmh_true"].to_numpy(dtype=float)
    v_pred = df["speed_kmh_pred"].to_numpy(dtype=float)
    err = v_true - v_pred

    rmse = float(math.sqrt(float(np.mean(err ** 2))))
    mae = float(np.mean(np.abs(err)))
    mean_true = float(np.mean(v_true))
    ss_res = float(np.sum((err) ** 2))
    ss_tot = float(np.sum((v_true - mean_true) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 1e-6 else 0.0

    print()
    print(f"nuScenes validation complete → {out_path}")
    print(f"  RMSE: {rmse:.3f} km/h")
    print(f"  MAE : {mae:.3f} km/h")
    print(f"  R^2 : {r2:.4f}")

    return {"rmse_kmh": rmse, "mae_kmh": mae, "r2": r2}


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(
        description="Validate ApexVelocity against nuScenes."
    )
    parser.add_argument(
        "--dataroot",
        type=str,
        required=True,
        help="Path to nuScenes dataroot (contains 'samples/', 'sweeps/', etc.).",
    )
    parser.add_argument(
        "--version",
        type=str,
        default="v1.0-mini",
        help="nuScenes version string (e.g., v1.0-mini, v1.0-trainval).",
    )
    parser.add_argument(
        "--split",
        type=str,
        default="train",
        help="Split name used by create_splits_scenes (e.g., train, val, mini_train).",
    )
    parser.add_argument(
        "--max-scenes",
        type=int,
        default=10,
        help="Maximum number of scenes to evaluate (0 = all).",
    )
    parser.add_argument(
        "--vehicle",
        type=str,
        default="tesla_model_3",
        help="ApexVelocity vehicle preset name (e.g., tesla_model_3, compact_car).",
    )
    parser.add_argument(
        "--condition",
        type=str,
        default="dry",
        help="Surface condition (dry / wet).",
    )
    parser.add_argument(
        "--output",
        type=str,
        default="validation/nuscenes_residuals.csv",
        help="Output CSV path for residuals and features.",
    )

    args = parser.parse_args(argv)
    validate_nuscenes(
        dataroot=args.dataroot,
        version=args.version,
        split=args.split,
        max_scenes=args.max_scenes,
        vehicle=args.vehicle,
        condition=args.condition,
        output_csv=args.output,
    )


if __name__ == "__main__":
    main()
