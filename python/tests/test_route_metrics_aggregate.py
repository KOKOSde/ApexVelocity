"""
Tests for route-level aggregate metrics.

This test verifies that the analysis module correctly aggregates
segment-level features into route-level metrics.
"""

import pytest
import math
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from apexvelocity.analysis import (
    analyze_profile,
    ProfileAnalysis,
    GRAVITY,
)


def create_simple_path(
    num_points: int = 20,
    segment_length_m: float = 10.0,
    speed_mps: float = 15.0,
    curvature: float = 0.0,
    grade_percent: float = 0.0,
) -> list:
    """Create a simple path with uniform properties."""
    path_points = []
    distance = 0.0
    elevation = 0.0
    total_energy = 0.0
    
    for i in range(num_points):
        if i > 0:
            elevation += segment_length_m * (grade_percent / 100)
            # Simple energy model
            total_energy += (100 + 50 * speed_mps) * segment_length_m
        
        path_points.append({
            'lat': 37.8 + i * 0.0001,
            'lon': -122.4,
            'z_m': elevation,
            'curvature': curvature,
            'distance_along_m': distance,
            'surface_type': 'asphalt',
            'speed_mps': speed_mps,
            'v_profile': speed_mps,
            'energy_joules': total_energy,
            'energy_kwh': total_energy / 3.6e6,
        })
        distance += segment_length_m
    
    return path_points


def create_brake_heavy_path(num_points: int = 30) -> list:
    """Create a path that should trigger many BRAKE actions."""
    path_points = []
    distance = 0.0
    
    for i in range(num_points):
        # Alternating high and low curvature to trigger braking
        if i % 3 == 0:
            curv = 0.05  # Very tight, 20m radius
            speed = 8.0
        else:
            curv = 0.0
            speed = 20.0
        
        path_points.append({
            'lat': 37.8 + i * 0.0001,
            'lon': -122.4,
            'z_m': 0,
            'curvature': curv,
            'distance_along_m': distance,
            'surface_type': 'asphalt',
            'speed_mps': speed,
            'v_profile': speed,
            'energy_joules': 0,
            'energy_kwh': 0,
        })
        distance += 10.0
    
    return path_points


class TestEnergyMetrics:
    """Test energy-related aggregate metrics."""
    
    def test_energy_per_km_calculation(self):
        """energy_kwh_per_km = total_energy_kwh / total_distance_km."""
        path = create_simple_path(num_points=50, segment_length_m=20.0)
        analysis = analyze_profile(path)
        
        if analysis.total_distance_km > 0 and analysis.total_energy_kwh > 0:
            expected_efficiency = analysis.total_energy_kwh / analysis.total_distance_km
            assert abs(analysis.energy_kwh_per_km - expected_efficiency) < 0.001, \
                f"Energy efficiency mismatch: {analysis.energy_kwh_per_km} vs {expected_efficiency}"
    
    def test_total_distance_matches_segments(self):
        """Total distance should equal sum of segment lengths."""
        path = create_simple_path(num_points=25, segment_length_m=10.0)
        analysis = analyze_profile(path)
        
        # Expected: 24 segments × 10m = 240m = 0.24 km
        expected_km = 24 * 10.0 / 1000
        
        assert abs(analysis.total_distance_km - expected_km) < 0.01, \
            f"Expected {expected_km} km, got {analysis.total_distance_km} km"


class TestActionDistribution:
    """Test action distribution metrics."""
    
    def test_brake_fraction_calculation(self):
        """brake_fraction should match fraction of segments with BRAKE/EASE_OFF."""
        path = create_brake_heavy_path(num_points=30)
        analysis = analyze_profile(path)
        
        # Count brake actions manually
        total_distance = sum(s.segment_length_m for s in analysis.segments)
        brake_distance = sum(s.segment_length_m for s in analysis.segments 
                           if s.recommended_action in ["BRAKE", "EASE_OFF"])
        
        if total_distance > 0:
            expected_fraction = brake_distance / total_distance
            assert abs(analysis.brake_fraction - expected_fraction) < 0.01, \
                f"Brake fraction mismatch: {analysis.brake_fraction} vs {expected_fraction}"
    
    def test_action_fractions_sum_to_one(self):
        """All action fractions should sum to approximately 1.0."""
        path = create_simple_path(num_points=30)
        analysis = analyze_profile(path)
        
        total = (analysis.brake_fraction + analysis.coast_fraction + 
                analysis.accelerate_fraction + analysis.hold_fraction)
        
        # Allow small numerical error
        assert abs(total - 1.0) < 0.05, f"Action fractions sum to {total}, expected ~1.0"


class TestDifficultyScore:
    """Test difficulty score computation."""
    
    def test_flat_straight_is_easy(self):
        """Flat, straight path should have low difficulty."""
        path = create_simple_path(num_points=30, curvature=0, grade_percent=0)
        analysis = analyze_profile(path)
        
        assert analysis.difficulty_score < 0.3, \
            f"Expected low difficulty for flat/straight, got {analysis.difficulty_score}"
    
    def test_curvy_increases_difficulty(self):
        """Adding curvature should increase difficulty."""
        path_easy = create_simple_path(num_points=30, curvature=0)
        path_hard = create_simple_path(num_points=30, curvature=0.02)  # 50m radius
        
        analysis_easy = analyze_profile(path_easy)
        analysis_hard = analyze_profile(path_hard)
        
        assert analysis_hard.difficulty_score > analysis_easy.difficulty_score, \
            f"Expected curvy harder, got easy={analysis_easy.difficulty_score}, hard={analysis_hard.difficulty_score}"
    
    def test_steep_grade_increases_difficulty(self):
        """Steep grade should increase difficulty."""
        path_flat = create_simple_path(num_points=30, grade_percent=0)
        path_steep = create_simple_path(num_points=30, grade_percent=12)
        
        analysis_flat = analyze_profile(path_flat)
        analysis_steep = analyze_profile(path_steep)
        
        assert analysis_steep.difficulty_score > analysis_flat.difficulty_score, \
            f"Expected steep harder, got flat={analysis_flat.difficulty_score}, steep={analysis_steep.difficulty_score}"


class TestSafetyMarginScore:
    """Test safety margin score computation."""
    
    def test_low_friction_use_high_margin(self):
        """Low friction usage should give high safety margin."""
        path = create_simple_path(num_points=30, curvature=0, speed_mps=10)
        analysis = analyze_profile(path)
        
        # Low speed, no curves = low friction use = high safety
        assert analysis.safety_margin_score > 0.5, \
            f"Expected high safety margin for easy path, got {analysis.safety_margin_score}"


class TestSpeedStatistics:
    """Test speed-related aggregate metrics."""
    
    def test_avg_speed_matches_input(self):
        """Average speed should match input for constant-speed path."""
        speed_mps = 20.0
        path = create_simple_path(num_points=30, speed_mps=speed_mps)
        analysis = analyze_profile(path)
        
        expected_kmh = speed_mps * 3.6
        assert abs(analysis.avg_speed_kmh - expected_kmh) < 1.0, \
            f"Expected avg speed ~{expected_kmh} km/h, got {analysis.avg_speed_kmh} km/h"
    
    def test_max_speed_at_least_avg(self):
        """Max speed should be >= average speed."""
        path = create_simple_path(num_points=30)
        analysis = analyze_profile(path)
        
        assert analysis.max_speed_kmh >= analysis.avg_speed_kmh, \
            f"Max speed {analysis.max_speed_kmh} < avg speed {analysis.avg_speed_kmh}"
    
    def test_p95_between_avg_and_max(self):
        """p95 speed should be between average and max."""
        path = create_simple_path(num_points=50)
        analysis = analyze_profile(path)
        
        assert analysis.avg_speed_kmh <= analysis.p95_speed_kmh <= analysis.max_speed_kmh, \
            f"p95 ({analysis.p95_speed_kmh}) not between avg ({analysis.avg_speed_kmh}) and max ({analysis.max_speed_kmh})"


class TestComfortMetrics:
    """Test comfort-related aggregate metrics."""
    
    def test_easy_path_low_violations(self):
        """Easy path should have few comfort violations."""
        path = create_simple_path(num_points=30, curvature=0, speed_mps=10)
        analysis = analyze_profile(path)
        
        assert analysis.comfort_violations_per_km < 1.0, \
            f"Expected < 1 violation/km for easy path, got {analysis.comfort_violations_per_km}"
    
    def test_uncomfortable_percentage_reasonable(self):
        """Uncomfortable percentage should be between 0 and 1."""
        path = create_simple_path(num_points=30)
        analysis = analyze_profile(path)
        
        assert 0 <= analysis.pct_uncomfortable_distance <= 1.0, \
            f"Uncomfortable % out of range: {analysis.pct_uncomfortable_distance}"


class TestGeometryMetrics:
    """Test geometry-related aggregate metrics."""
    
    def test_max_grade_tracked(self):
        """Max grade should reflect the steepest segment."""
        path = create_simple_path(num_points=30, grade_percent=8.0)
        analysis = analyze_profile(path)
        
        assert analysis.max_grade_percent > 6.0, \
            f"Expected max grade > 6%, got {analysis.max_grade_percent}%"
    
    def test_elevation_gain_loss_consistent(self):
        """Elevation gain and loss should be consistent with grade."""
        # Uphill then flat
        path = create_simple_path(num_points=20, grade_percent=5.0)
        analysis = analyze_profile(path)
        
        # If going uphill, should have positive gain
        assert analysis.total_elevation_gain_m > 0 or analysis.total_elevation_loss_m > 0, \
            "Expected some elevation change"


class TestSummaryDict:
    """Test the get_summary_dict method."""
    
    def test_summary_contains_all_keys(self):
        """Summary dict should contain all expected keys."""
        path = create_simple_path(num_points=30)
        analysis = analyze_profile(path)
        summary = analysis.get_summary_dict()
        
        expected_keys = [
            'total_distance_km', 'lap_time_s', 'avg_speed_kmh', 'max_speed_kmh',
            'max_lateral_g', 'p95_lateral_g', 'max_friction_usage',
            'total_energy_kwh', 'energy_kwh_per_km', 'brake_fraction',
            'difficulty_score', 'safety_margin_score',
        ]
        
        for key in expected_keys:
            assert key in summary, f"Missing key in summary: {key}"
    
    def test_summary_values_are_rounded(self):
        """Summary values should be reasonably rounded."""
        path = create_simple_path(num_points=30)
        analysis = analyze_profile(path)
        summary = analysis.get_summary_dict()
        
        # Check that values aren't excessively precise
        # e.g., 0.123456789 should be rounded to something like 0.12
        assert len(str(summary['difficulty_score'])) < 10, \
            f"Difficulty score not rounded: {summary['difficulty_score']}"


class TestHUDDict:
    """Test the get_hud_dict method."""
    
    def test_hud_values_are_strings(self):
        """HUD dict should contain formatted strings."""
        path = create_simple_path(num_points=30)
        analysis = analyze_profile(path)
        hud = analysis.get_hud_dict()
        
        for key, value in hud.items():
            assert isinstance(value, str), f"HUD value for {key} should be string, got {type(value)}"
    
    def test_hud_contains_units(self):
        """HUD strings should include units."""
        path = create_simple_path(num_points=30)
        analysis = analyze_profile(path)
        hud = analysis.get_hud_dict()
        
        assert 'km' in hud['Distance'], f"Distance should have km: {hud['Distance']}"
        assert 'km/h' in hud['Avg Speed'], f"Speed should have km/h: {hud['Avg Speed']}"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])





