"""
Tests for the uncertainty training and prediction pipeline.

These tests use a synthetic dataset where the true residual magnitude
is a simple function of curvature, so we can verify that:
- The training script runs end-to-end on CSV input.
- The learned model + simple scaling produce coverage close to target.
"""

from __future__ import annotations

import os
from pathlib import Path

import numpy as np
import pandas as pd

from apexvelocity.analysis import SegmentFeatures
from apexvelocity.uncertainty import (
    extract_features_from_segments,
    predict_speed_intervals,
)
from validation.uncertainty_training import train_uncertainty_model


def _make_synthetic_residuals_csv(tmp_path: Path) -> str:
    rng = np.random.default_rng(123)
    n = 500

    curvature = rng.uniform(0.0, 0.02, size=n)
    grade = rng.normal(0.0, 3.0, size=n)
    mu = rng.uniform(0.6, 1.0, size=n)
    speed_pred = rng.uniform(30.0, 120.0, size=n)  # km/h
    physics_limit = speed_pred + rng.uniform(0.0, 20.0, size=n)
    energy_intensity = rng.uniform(0.1, 0.4, size=n)

    # True residual magnitude grows with curvature + grade noise
    base_err = 2.0 + 50.0 * curvature + 0.05 * np.abs(grade)
    noise = rng.normal(0.0, 0.5, size=n)
    err = base_err + noise

    sign = rng.choice([-1.0, 1.0], size=n)
    speed_true = speed_pred + sign * err

    df = pd.DataFrame(
        {
            "speed_kmh_true": speed_true,
            "speed_kmh_pred": speed_pred,
            "curvature_1pm": curvature,
            "grade_percent": grade,
            "physics_limit_kmh": physics_limit,
            "friction_mu_effective": mu,
            "energy_kwh_per_km": energy_intensity,
        }
    )

    out_path = tmp_path / "synthetic_residuals.csv"
    df.to_csv(out_path, index=False)
    return str(out_path)


def test_train_uncertainty_model_and_predict_intervals(tmp_path: Path) -> None:
    csv_path = _make_synthetic_residuals_csv(tmp_path)
    model_path = tmp_path / "uncertainty_model.joblib"

    metrics = train_uncertainty_model(
        csv_path=csv_path,
        output_path=str(model_path),
        nominal_coverage=0.9,
    )

    assert model_path.exists(), "Model file should be created"
    assert 0.7 <= metrics["coverage_val"] <= 1.0

    # Load trained model dict
    import joblib  # type: ignore[import-untyped]

    bundle = joblib.load(model_path)
    model = bundle["model"]
    meta = bundle["metadata"]

    # Build a small synthetic set of SegmentFeatures compatible with the model
    df = pd.read_csv(csv_path).head(100)
    segments = []
    for i, row in df.iterrows():
        seg = SegmentFeatures(index=i)
        seg.curvature_1pm = float(row["curvature_1pm"])
        seg.grade_percent = float(row["grade_percent"])
        seg.speed_kmh = float(row["speed_kmh_pred"])
        seg.physics_limit_speed_kmh = float(row["physics_limit_kmh"])
        seg.friction_mu_effective = float(row["friction_mu_effective"])
        seg.energy_kwh_per_km = float(row["energy_kwh_per_km"])
        segments.append(seg)

    X = extract_features_from_segments(segments)
    assert X.shape[0] == len(segments)

    # Predict intervals and measure empirical coverage on this small set
    intervals = predict_speed_intervals(
        segments,
        model=model,
        nominal_coverage=float(meta["nominal_coverage"]),
        coverage_scale=float(meta["coverage_scale"]),
    )

    assert len(intervals) == len(segments)

    v_true = df["speed_kmh_true"].to_numpy(dtype=float)
    inside = [
        low <= t <= high
        for t, (low, high) in zip(
            v_true, [(iv.speed_kmh_low, iv.speed_kmh_high) for iv in intervals]
        )
    ]
    coverage = float(np.mean(inside))

    # On synthetic data, coverage should be in a reasonable band around target.
    assert 0.8 <= coverage <= 0.98
