"""
Tests for segment features on paths with turns and hills.

This test verifies that the analysis module correctly computes
features for complex scenarios: curves, elevation changes, varying speeds.
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
    classify_turn,
)


def create_90_degree_turn_path(
    approach_length: int = 10,
    turn_points: int = 10,
    exit_length: int = 10,
    turn_radius_m: float = 50.0,
    speed_mps: float = 15.0,
) -> list:
    """
    Create a path with a 90-degree right turn.

    The path goes straight, turns right 90 degrees, then goes straight.
    """
    path_points = []
    segment_length = 10.0
    distance = 0.0

    base_lat = 37.8
    base_lon = -122.4

    # Approach (going north)
    for i in range(approach_length):
        path_points.append(
            {
                "lat": base_lat + i * 0.0001,
                "lon": base_lon,
                "x_m": 0,
                "y_m": distance,
                "z_m": 0,
                "curvature": 0,
                "distance_along_m": distance,
                "surface_type": "asphalt",
                "speed_mps": speed_mps,
                "v_profile": speed_mps,
                "energy_joules": 0,
                "energy_kwh": 0,
            }
        )
        distance += segment_length

    # Turn (arc going from north to east)
    turn_curvature = 1.0 / turn_radius_m
    arc_length = (math.pi / 2) * turn_radius_m  # 90 degrees
    arc_step = arc_length / turn_points

    center_lat = base_lat + approach_length * 0.0001
    center_lon = base_lon + turn_radius_m / 111320

    for i in range(turn_points):
        angle = math.pi - (i / turn_points) * (math.pi / 2)  # From π to π/2
        lat_offset = turn_radius_m * math.sin(angle) / 111320
        lon_offset = turn_radius_m * math.cos(angle) / 111320

        # Reduce speed in turn due to friction limit
        turn_speed = min(speed_mps, math.sqrt(0.9 * GRAVITY / turn_curvature))

        path_points.append(
            {
                "lat": center_lat + lat_offset,
                "lon": center_lon + lon_offset,
                "x_m": 0,
                "y_m": distance,
                "z_m": 0,
                "curvature": turn_curvature,
                "distance_along_m": distance,
                "surface_type": "asphalt",
                "speed_mps": turn_speed,
                "v_profile": turn_speed,
                "energy_joules": 0,
                "energy_kwh": 0,
            }
        )
        distance += arc_step

    # Exit (going east)
    for i in range(exit_length):
        path_points.append(
            {
                "lat": center_lat,
                "lon": center_lon + turn_radius_m / 111320 + i * 0.0001,
                "x_m": 0,
                "y_m": distance,
                "z_m": 0,
                "curvature": 0,
                "distance_along_m": distance,
                "surface_type": "asphalt",
                "speed_mps": speed_mps,
                "v_profile": speed_mps,
                "energy_joules": 0,
                "energy_kwh": 0,
            }
        )
        distance += segment_length

    return path_points


def create_hill_path(
    flat_before: int = 10,
    uphill_length: int = 10,
    flat_top: int = 5,
    downhill_length: int = 10,
    flat_after: int = 10,
    grade_percent: float = 10.0,
    segment_length_m: float = 10.0,
) -> list:
    """
    Create a path that goes up a hill and back down.

    Profile: flat → uphill → flat → downhill → flat
    """
    path_points = []
    distance = 0.0
    elevation = 0.0

    base_lat = 37.8
    base_lon = -122.4

    # Flat approach
    for i in range(flat_before):
        path_points.append(
            {
                "lat": base_lat + i * 0.0001,
                "lon": base_lon,
                "z_m": elevation,
                "curvature": 0,
                "distance_along_m": distance,
                "surface_type": "asphalt",
                "speed_mps": 15.0,
                "v_profile": 15.0,
                "energy_joules": 0,
                "energy_kwh": 0,
            }
        )
        distance += segment_length_m

    # Uphill
    dz_per_segment = segment_length_m * (grade_percent / 100)
    for i in range(uphill_length):
        elevation += dz_per_segment
        path_points.append(
            {
                "lat": base_lat + (flat_before + i) * 0.0001,
                "lon": base_lon,
                "z_m": elevation,
                "curvature": 0,
                "distance_along_m": distance,
                "surface_type": "asphalt",
                "speed_mps": 12.0,  # Slower uphill
                "v_profile": 12.0,
                "energy_joules": 0,
                "energy_kwh": 0,
            }
        )
        distance += segment_length_m

    # Flat top
    for i in range(flat_top):
        path_points.append(
            {
                "lat": base_lat + (flat_before + uphill_length + i) * 0.0001,
                "lon": base_lon,
                "z_m": elevation,
                "curvature": 0,
                "distance_along_m": distance,
                "surface_type": "asphalt",
                "speed_mps": 15.0,
                "v_profile": 15.0,
                "energy_joules": 0,
                "energy_kwh": 0,
            }
        )
        distance += segment_length_m

    # Downhill
    for i in range(downhill_length):
        elevation -= dz_per_segment
        path_points.append(
            {
                "lat": base_lat + (flat_before + uphill_length + flat_top + i) * 0.0001,
                "lon": base_lon,
                "z_m": elevation,
                "curvature": 0,
                "distance_along_m": distance,
                "surface_type": "asphalt",
                "speed_mps": 18.0,  # Faster downhill
                "v_profile": 18.0,
                "energy_joules": 0,
                "energy_kwh": 0,
            }
        )
        distance += segment_length_m

    # Flat exit
    for i in range(flat_after):
        path_points.append(
            {
                "lat": base_lat
                + (flat_before + uphill_length + flat_top + downhill_length + i)
                * 0.0001,
                "lon": base_lon,
                "z_m": elevation,
                "curvature": 0,
                "distance_along_m": distance,
                "surface_type": "asphalt",
                "speed_mps": 15.0,
                "v_profile": 15.0,
                "energy_joules": 0,
                "energy_kwh": 0,
            }
        )
        distance += segment_length_m

    return path_points


class TestTurnPath:
    """Test segment features on a path with a 90-degree turn."""

    def test_curvature_nonzero_in_turn(self):
        """Curvature should be non-zero during the turn."""
        path = create_90_degree_turn_path(turn_radius_m=50.0)
        analysis = analyze_profile(path)

        # Find turn segments (middle of path)
        approach = 10
        turn_points = 10

        turn_segments = analysis.segments[approach : approach + turn_points]

        curvatures = [s.curvature_1pm for s in turn_segments]
        max_curvature = max(curvatures)

        # Expected curvature: 1/50 = 0.02
        assert max_curvature > 0.015, f"Expected curvature > 0.015, got {max_curvature}"

    def test_lateral_g_in_turn(self):
        """Lateral G should spike during the turn."""
        path = create_90_degree_turn_path(turn_radius_m=50.0)
        analysis = analyze_profile(path)

        # Check turn segments have higher lateral G
        approach = 10
        turn_points = 10

        approach_max_g = max(s.lateral_g for s in analysis.segments[:approach])
        turn_max_g = max(
            s.lateral_g for s in analysis.segments[approach : approach + turn_points]
        )

        assert (
            turn_max_g > approach_max_g * 5
        ), f"Expected turn lateral G >> approach, got turn={turn_max_g:.3f}, approach={approach_max_g:.3f}"

    def test_comfort_score_higher_in_turn(self):
        """Comfort score should be higher (worse) in the turn."""
        path = create_90_degree_turn_path(turn_radius_m=30.0)  # Tight turn
        analysis = analyze_profile(path)

        approach = 10
        turn_points = 10

        approach_comfort = (
            sum(s.comfort_score for s in analysis.segments[:approach]) / approach
        )
        turn_comfort = (
            sum(
                s.comfort_score
                for s in analysis.segments[approach : approach + turn_points]
            )
            / turn_points
        )

        assert (
            turn_comfort > approach_comfort
        ), f"Expected turn comfort > approach, got turn={turn_comfort:.3f}, approach={approach_comfort:.3f}"

    def test_friction_usage_higher_in_turn(self):
        """Friction usage should be higher in the turn."""
        path = create_90_degree_turn_path(turn_radius_m=40.0)
        analysis = analyze_profile(path)

        approach = 10
        turn_points = 10

        turn_friction = max(
            s.friction_usage
            for s in analysis.segments[approach : approach + turn_points]
        )
        approach_friction = max(s.friction_usage for s in analysis.segments[:approach])

        assert (
            turn_friction > approach_friction * 2
        ), f"Expected higher friction in turn, got turn={turn_friction:.3f}, approach={approach_friction:.3f}"

    def test_brake_recommended_in_tight_turn(self):
        """BRAKE should be recommended for a very tight turn."""
        path = create_90_degree_turn_path(turn_radius_m=20.0)  # Very tight
        analysis = analyze_profile(path)

        approach = 10
        turn_points = 10

        turn_segments = analysis.segments[approach : approach + turn_points]
        brake_or_ease = sum(
            1 for s in turn_segments if s.recommended_action in ["BRAKE", "EASE_OFF"]
        )

        # At least some segments should recommend braking
        assert brake_or_ease > 0, "Expected some BRAKE/EASE_OFF actions in tight turn"


class TestHillPath:
    """Test segment features on a path with hills."""

    def test_grade_positive_uphill(self):
        """Grade should be positive going uphill."""
        path = create_hill_path(grade_percent=10.0)
        analysis = analyze_profile(path)

        # Uphill starts after flat_before (10) segments
        uphill_start = 10
        uphill_end = 20

        uphill_grades = [
            s.grade_percent for s in analysis.segments[uphill_start + 1 : uphill_end]
        ]
        avg_uphill_grade = (
            sum(uphill_grades) / len(uphill_grades) if uphill_grades else 0
        )

        assert (
            avg_uphill_grade > 5.0
        ), f"Expected positive grade uphill, got {avg_uphill_grade}%"

    def test_grade_negative_downhill(self):
        """Grade should be negative going downhill."""
        path = create_hill_path(grade_percent=10.0)
        analysis = analyze_profile(path)

        # Downhill starts after flat_before + uphill + flat_top
        downhill_start = 10 + 10 + 5
        downhill_end = downhill_start + 10

        downhill_grades = [
            s.grade_percent
            for s in analysis.segments[downhill_start + 1 : downhill_end]
        ]
        avg_downhill_grade = (
            sum(downhill_grades) / len(downhill_grades) if downhill_grades else 0
        )

        assert (
            avg_downhill_grade < -5.0
        ), f"Expected negative grade downhill, got {avg_downhill_grade}%"

    def test_regen_potential_downhill(self):
        """Should have regen potential on downhill."""
        path = create_hill_path(grade_percent=15.0)  # Steeper for more regen
        analysis = analyze_profile(path)

        # Check total regen potential
        assert analysis.total_regen_kwh > 0, "Expected some regen potential"

    def test_elevation_gain_tracked(self):
        """Total elevation gain should match the hill."""
        path = create_hill_path(
            uphill_length=10, grade_percent=10.0, segment_length_m=10.0
        )
        analysis = analyze_profile(path)

        # Expected gain: 10 segments × 10m × 10% = 10m
        expected_gain = 10 * 10 * 0.10

        assert (
            abs(analysis.total_elevation_gain_m - expected_gain) < 2.0
        ), f"Expected ~{expected_gain}m gain, got {analysis.total_elevation_gain_m}m"


class TestTurnClassification:
    """Test turn classification logic."""

    def test_straight(self):
        """Small angle = straight."""
        assert classify_turn(2.0) == "straight"
        assert classify_turn(-3.0) == "straight"

    def test_slight_turns(self):
        """Medium angle = slight turn."""
        assert classify_turn(10.0) == "slight_left"
        assert classify_turn(-12.0) == "slight_right"

    def test_normal_turns(self):
        """Larger angle = normal turn."""
        assert classify_turn(30.0) == "left"
        assert classify_turn(-40.0) == "right"

    def test_sharp_turns(self):
        """Very large angle = sharp turn."""
        assert classify_turn(70.0) == "sharp_left"
        assert classify_turn(-80.0) == "sharp_right"

    def test_u_turn(self):
        """>90 degrees = U-turn."""
        assert classify_turn(120.0) == "u_turn"
        assert classify_turn(-150.0) == "u_turn"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
