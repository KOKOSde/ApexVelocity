"""
Train an uncertainty model on ApexVelocity residuals.

This script expects a CSV file with at least the following columns:

    speed_kmh_true      # Ground-truth speed from dataset (e.g. nuScenes)
    speed_kmh_pred      # ApexVelocity-predicted speed
    curvature_1pm       # Path curvature [1/m]
    grade_percent       # Road grade [%]
    physics_limit_kmh   # Physics-based speed limit from solver [km/h]
    friction_mu_effective
    energy_kwh_per_km

Additional columns are ignored.

The script:
1. Computes residuals: error_kmh = speed_kmh_true - speed_kmh_pred
2. Trains a regressor to predict |error_kmh| from features.
3. Calibrates a global coverage_scale so that a symmetric interval
   [pred - scale*err_hat, pred + scale*err_hat] achieves ~90% coverage
   on a validation split.
4. Saves the trained model and metadata via joblib.

Usage:
    cd python
    python -m validation.uncertainty_training \\
        --csv residuals.csv \\
        --output models/uncertainty_model.joblib
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Dict, Tuple

import joblib  # type: ignore[import-untyped]
import numpy as np  # type: ignore[import-untyped]
import pandas as pd  # type: ignore[import-untyped]
from sklearn.ensemble import GradientBoostingRegressor  # type: ignore[import-untyped]
from sklearn.model_selection import train_test_split  # type: ignore[import-untyped]


FEATURE_COLUMNS = [
    "curvature_1pm",
    "grade_percent",
    "speed_kmh_pred",
    "physics_limit_kmh",
    "friction_mu_effective",
    "energy_kwh_per_km",
]


@dataclass
class UncertaintyMetadata:
    """Metadata stored alongside the trained model."""

    nominal_coverage: float
    coverage_scale: float
    achieved_coverage: float
    feature_columns: Tuple[str, ...]


def _build_features(df: pd.DataFrame) -> pd.DataFrame:
    # Ensure required columns exist; raise a clear error otherwise.
    missing = [c for c in FEATURE_COLUMNS if c not in df.columns]
    if missing:
        raise ValueError(f"Missing required feature columns: {missing}")

    feats = pd.DataFrame(index=df.index)
    feats["curvature_1pm"] = df["curvature_1pm"].astype(float)
    feats["grade_percent"] = df["grade_percent"].astype(float)
    feats["speed_kmh_pred"] = df["speed_kmh_pred"].astype(float)
    feats["physics_limit_kmh"] = df["physics_limit_kmh"].astype(float)
    feats["friction_mu_effective"] = df["friction_mu_effective"].astype(float)
    feats["energy_kwh_per_km"] = df["energy_kwh_per_km"].astype(float)
    return feats


def _compute_coverage(
    df_val: pd.DataFrame,
    err_hat: np.ndarray,
    scale: float,
) -> float:
    """Compute empirical coverage for a symmetric band."""
    v_pred = df_val["speed_kmh_pred"].to_numpy(dtype=float)
    v_true = df_val["speed_kmh_true"].to_numpy(dtype=float)
    band = np.abs(err_hat) * float(scale)
    low = v_pred - band
    high = v_pred + band
    inside = (v_true >= low) & (v_true <= high)
    return float(inside.mean())


def train_uncertainty_model(
    csv_path: str,
    output_path: str,
    nominal_coverage: float = 0.9,
) -> Dict[str, float]:
    """
    Train an uncertainty model from a residuals CSV and save it.

    Args:
        csv_path: Path to residual CSV.
        output_path: Where to save the joblib model + metadata.
        nominal_coverage: Target coverage for calibration (default 0.9).

    Returns:
        Dict with summary metrics (coverage, rmse, etc.) for logging.
    """
    df = pd.read_csv(csv_path)

    if "speed_kmh_true" not in df.columns or "speed_kmh_pred" not in df.columns:
        raise ValueError(
            "CSV must contain 'speed_kmh_true' and 'speed_kmh_pred' columns."
        )

    # Residual in km/h
    df["error_kmh"] = df["speed_kmh_true"].astype(float) - df["speed_kmh_pred"].astype(
        float
    )
    df["abs_error_kmh"] = df["error_kmh"].abs()

    X = _build_features(df)
    y = df["abs_error_kmh"].to_numpy(dtype=float)

    X_train, X_val, y_train, y_val, df_train, df_val = train_test_split(
        X, y, df, test_size=0.2, random_state=42
    )

    model = GradientBoostingRegressor(
        loss="squared_error",
        n_estimators=200,
        max_depth=3,
        learning_rate=0.05,
        subsample=0.8,
        random_state=42,
    )
    model.fit(X_train, y_train)

    # Basic error metrics on validation set
    y_val_hat = model.predict(X_val)
    rmse = float(np.sqrt(np.mean((y_val_hat - y_val) ** 2)))
    mae = float(np.mean(np.abs(y_val_hat - y_val)))

    # Coverage calibration: search for a global scale factor.
    scales = np.linspace(0.5, 2.0, 16)
    best_scale = 1.0
    best_diff = 1e9
    best_cov = 0.0

    for s in scales:
        cov = _compute_coverage(df_val, y_val_hat, s)
        diff = abs(cov - nominal_coverage)
        if diff < best_diff:
            best_diff = diff
            best_scale = float(s)
            best_cov = cov

    meta = UncertaintyMetadata(
        nominal_coverage=nominal_coverage,
        coverage_scale=best_scale,
        achieved_coverage=best_cov,
        feature_columns=tuple(FEATURE_COLUMNS),
    )

    out_dir = Path(output_path).resolve().parent
    out_dir.mkdir(parents=True, exist_ok=True)

    joblib.dump({"model": model, "metadata": asdict(meta)}, output_path)

    return {
        "rmse_val_abs_error_kmh": rmse,
        "mae_val_abs_error_kmh": mae,
        "coverage_val": best_cov,
        "coverage_scale": best_scale,
    }


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(
        description="Train an uncertainty model from residual CSV."
    )
    parser.add_argument(
        "--csv",
        type=str,
        required=True,
        help="Path to residuals CSV with speed_kmh_true/speed_kmh_pred and features.",
    )
    parser.add_argument(
        "--output",
        type=str,
        required=True,
        help="Output path for joblib model (e.g. models/uncertainty_model.joblib).",
    )
    parser.add_argument(
        "--coverage",
        type=float,
        default=0.9,
        help="Target nominal coverage for intervals (default: 0.9).",
    )

    args = parser.parse_args(argv)
    metrics = train_uncertainty_model(
        csv_path=args.csv,
        output_path=args.output,
        nominal_coverage=args.coverage,
    )

    print("Training complete.")
    for k, v in metrics.items():
        print(f"{k}: {v:.4f}")


if __name__ == "__main__":
    main()
