"""
Tests for segment features on a flat, straight path.

This test verifies that the analysis module correctly computes
features for a simple case: flat terrain, no curvature, constant speed.
"""

import pytest
import math
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from apexvelocity.analysis import (
    analyze_profile,
    SegmentFeatures,
    ProfileAnalysis,
    GRAVITY,
)


def create_flat_straight_path(
    num_points: int = 50,
    segment_length_m: float = 10.0,
    speed_mps: float = 20.0,
) -> list:
    """
    Create a synthetic flat, straight path with constant speed.

    Args:
        num_points: Number of path points
        segment_length_m: Distance between consecutive points [m]
        speed_mps: Constant speed [m/s]

    Returns:
        List of path point dicts
    """
    path_points = []
    total_energy = 0.0

    for i in range(num_points):
        distance = i * segment_length_m

        # Simulate simple energy accumulation
        if i > 0:
            # Simple aero + rolling resistance
            f_total = 100 + 0.5 * 1.225 * 0.3 * 2.2 * speed_mps ** 2
            total_energy += f_total * segment_length_m

        path_points.append(
            {
                "lat": 37.8 + i * 0.00001,  # Slight movement for valid coords
                "lon": -122.4,
                "x_m": 0,
                "y_m": distance,
                "z_m": 0,  # Flat
                "elevation_m": 0,
                "curvature": 0,  # Straight
                "distance_along_m": distance,
                "surface_type": "asphalt",
                "speed_mps": speed_mps,
                "v_profile": speed_mps,
                "energy_joules": total_energy,
                "energy_kwh": total_energy / 3.6e6,
            }
        )

    return path_points


class TestFlatStraightPath:
    """Test segment features on a flat, straight path."""

    def test_curvature_is_zero(self):
        """Curvature should be approximately zero for straight path."""
        path = create_flat_straight_path(num_points=20)
        analysis = analyze_profile(path)

        for seg in analysis.segments:
            assert (
                seg.curvature_1pm < 1e-6
            ), f"Expected curvature ≈ 0, got {seg.curvature_1pm}"
            assert seg.curve_radius_m == float(
                "inf"
            ), "Expected infinite radius for straight"

    def test_grade_is_zero(self):
        """Grade should be zero for flat path."""
        path = create_flat_straight_path(num_points=20)
        analysis = analyze_profile(path)

        for seg in analysis.segments[1:]:  # Skip first (no grade calc)
            assert (
                abs(seg.grade_percent) < 0.1
            ), f"Expected grade ≈ 0%, got {seg.grade_percent}%"

    def test_lateral_accel_is_zero(self):
        """Lateral acceleration should be zero for straight path."""
        path = create_flat_straight_path(num_points=20)
        analysis = analyze_profile(path)

        for seg in analysis.segments:
            assert (
                abs(seg.lateral_accel_mps2) < 0.01
            ), f"Expected lat accel ≈ 0, got {seg.lateral_accel_mps2}"
            assert (
                abs(seg.lateral_g) < 0.001
            ), f"Expected lat G ≈ 0, got {seg.lateral_g}"

    def test_dcurvature_ds_is_zero(self):
        """Rate of curvature change should be zero."""
        path = create_flat_straight_path(num_points=20)
        analysis = analyze_profile(path)

        for seg in analysis.segments[1:]:
            assert (
                abs(seg.dcurvature_ds) < 1e-8
            ), f"Expected dκ/ds ≈ 0, got {seg.dcurvature_ds}"

    def test_jerk_is_low(self):
        """Jerk should be very low with constant speed."""
        path = create_flat_straight_path(num_points=30, speed_mps=20.0)
        analysis = analyze_profile(path)

        # Skip first few segments where derivatives are computed
        for seg in analysis.segments[3:]:
            # With constant speed, jerk should be minimal (numerical noise only)
            assert (
                abs(seg.longitudinal_jerk_mps3) < 0.5
            ), f"Expected low jerk, got {seg.longitudinal_jerk_mps3}"

    def test_comfort_score_is_low(self):
        """Comfort score should be low (comfortable) for gentle driving."""
        path = create_flat_straight_path(num_points=20)
        analysis = analyze_profile(path)

        for seg in analysis.segments:
            assert (
                seg.comfort_score < 0.3
            ), f"Expected comfortable (< 0.3), got {seg.comfort_score}"
            assert not seg.is_uncomfortable, "Should not be uncomfortable"

    def test_friction_usage_is_low(self):
        """Friction usage should be low on straight road."""
        path = create_flat_straight_path(num_points=20)
        analysis = analyze_profile(path)

        for seg in analysis.segments:
            # No lateral force, so friction usage is just longitudinal
            assert (
                seg.friction_usage < 0.3
            ), f"Expected low friction use, got {seg.friction_usage}"

    def test_recommended_action_not_brake(self):
        """Should not recommend brake on easy straight."""
        path = create_flat_straight_path(num_points=20)
        analysis = analyze_profile(path)

        brake_count = sum(
            1 for s in analysis.segments if s.recommended_action == "BRAKE"
        )
        assert brake_count == 0, f"Expected no BRAKE actions, got {brake_count}"

    def test_turn_type_is_straight(self):
        """Turn type should be 'straight' for all segments."""
        path = create_flat_straight_path(num_points=20)
        analysis = analyze_profile(path)

        for seg in analysis.segments:
            assert (
                seg.turn_type == "straight"
            ), f"Expected 'straight', got '{seg.turn_type}'"

    def test_route_level_aggregates(self):
        """Route-level metrics should be reasonable."""
        path = create_flat_straight_path(
            num_points=50, segment_length_m=10.0, speed_mps=20.0
        )
        analysis = analyze_profile(path)

        # Distance should match
        expected_dist = 49 * 10.0 / 1000  # km
        assert (
            abs(analysis.total_distance_km - expected_dist) < 0.01
        ), f"Expected ~{expected_dist:.3f} km, got {analysis.total_distance_km}"

        # Average speed should match input
        assert (
            abs(analysis.avg_speed_kmh - 20 * 3.6) < 1.0
        ), f"Expected ~72 km/h, got {analysis.avg_speed_kmh}"

        # No turns on straight path
        assert (
            analysis.turns_per_km < 0.1
        ), f"Expected ~0 turns/km, got {analysis.turns_per_km}"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
