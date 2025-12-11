"""
Uncertainty quantification helpers for ApexVelocity.

This module provides utilities to:
- Extract ML-ready features from SegmentFeatures objects.
- Load a trained residual/uncertainty model.
- Predict per-segment confidence intervals around physics predictions.

The actual model training is done by the script:
    python/validation/uncertainty_training.py
which operates on residual CSVs generated from validation datasets
such as nuScenes.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import joblib  # type: ignore[import-untyped]
import numpy as np  # type: ignore[import-untyped]

from .analysis import SegmentFeatures


DEFAULT_MODEL_PATH = (
    Path(__file__).resolve().parent / "data" / "uncertainty_model.joblib"
)


@dataclass
class IntervalPrediction:
    """Uncertainty interval for a single segment."""

    speed_kmh_pred: float
    speed_kmh_low: float
    speed_kmh_high: float
    band_kmh: float
    nominal_coverage: float = 0.9


def _ensure_2d(x: np.ndarray) -> np.ndarray:
    if x.ndim == 1:
        return x.reshape(-1, 1)
    return x


def extract_features_from_segments(
    segments: Sequence[SegmentFeatures],
) -> np.ndarray:
    """
    Convert SegmentFeatures into a numeric feature matrix for ML.

    The exact feature set is intentionally simple and stable:
    - curvature_1pm
    - grade_percent
    - speed_kmh
    - physics_limit_speed_kmh
    - friction_mu_effective
    - energy_kwh_per_km

    Returns:
        np.ndarray of shape (n_segments, n_features)
    """

    n = len(segments)
    feats = np.zeros((n, 6), dtype=float)

    for i, seg in enumerate(segments):
        feats[i, 0] = float(seg.curvature_1pm)
        feats[i, 1] = float(seg.grade_percent)
        feats[i, 2] = float(seg.speed_kmh)
        feats[i, 3] = float(seg.physics_limit_speed_kmh)
        feats[i, 4] = float(seg.friction_mu_effective)
        feats[i, 5] = float(seg.energy_kwh_per_km)

    return feats


def load_uncertainty_model(
    model_path: Optional[str] = None,
):
    """
    Load a trained uncertainty model from disk.

    Args:
        model_path: Optional path to a joblib file. If None, uses the
            packaged default path under apexvelocity/data/.

    Returns:
        A scikit-learn compatible model object.

    Raises:
        FileNotFoundError if the model file is missing.
    """
    path = Path(model_path) if model_path is not None else DEFAULT_MODEL_PATH
    if not path.exists():
        raise FileNotFoundError(
            f"Uncertainty model not found at '{path}'. "
            "Train one with python/validation/uncertainty_training.py "
            "and point load_uncertainty_model() to it."
        )
    return joblib.load(path)


def predict_speed_intervals(
    segments: Sequence[SegmentFeatures],
    model,
    nominal_coverage: float = 0.9,
    coverage_scale: float = 1.0,
) -> List[IntervalPrediction]:
    """
    Predict per-segment speed intervals using a trained model.

    The model is assumed to predict a typical absolute error (e.g. 90th
    percentile |residual| in km/h). We then construct symmetric intervals:

        [v_pred - scale * err, v_pred + scale * err]

    Args:
        segments: List of SegmentFeatures from a solved/analysed route.
        model: Trained scikit-learn model with a .predict(X) method.
        nominal_coverage: Target nominal coverage (for metadata only).
        coverage_scale: Additional scaling for intervals. This is where
            calibration tuning is applied (see training script).

    Returns:
        List[IntervalPrediction] aligned with input segments.
    """
    if not segments:
        return []

    X = extract_features_from_segments(segments)
    # Model predicts a characteristic error magnitude in km/h.
    err_kmh = np.abs(_ensure_2d(model.predict(X)).ravel()) * float(coverage_scale)

    out: List[IntervalPrediction] = []
    for seg, e in zip(segments, err_kmh):
        v = float(seg.speed_kmh)
        low = v - e
        high = v + e
        out.append(
            IntervalPrediction(
                speed_kmh_pred=v,
                speed_kmh_low=low,
                speed_kmh_high=high,
                band_kmh=2.0 * e,
                nominal_coverage=nominal_coverage,
            )
        )
    return out
