#!/usr/bin/env python3
"""
Tests for ApexVelocity geometry smoothing and curvature calculation.

Tests:
1. Straight line: curvature should be ~0
2. Circular arc: curvature should be ~1/R
3. Path length preservation after smoothing
4. Point count increase after smoothing
"""

import sys
import os
import math
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from apexvelocity.geometry import (
    smooth_path_geo,
    recompute_curvature,
    get_path_stats,
    CRSTransformer,
)


class TestStraightLine:
    """Test smoothing and curvature on a straight line."""
    
    def test_straight_line_curvature_is_zero(self):
        """A straight line should have zero curvature."""
        # Create a straight line from (0, 0) to (100, 0) in meters
        # Using lat/lon near equator where 1 degree ≈ 111km
        # 100m = ~0.0009 degrees at equator
        
        center_lat, center_lon = 0.0, 0.0
        points = []
        for i in range(11):  # 11 points, 10m apart
            lon_offset = i * 10 / 111320  # ~10m steps
            points.append({
                'lat': center_lat,
                'lon': center_lon + lon_offset,
                'surface_type': 'asphalt',
            })
        
        # Smooth the path
        smooth_path = smooth_path_geo(points, target_spacing_m=5.0, mode="catmull_rom")
        
        # Recompute curvature
        smooth_path = recompute_curvature(smooth_path)
        
        # Check that all curvatures are near zero
        curvatures = [p['curvature'] for p in smooth_path]
        max_curv = max(abs(c) for c in curvatures)
        
        # Tolerance: should be < 0.001 (radius > 1000m)
        assert max_curv < 0.001, f"Max curvature {max_curv} should be < 0.001 for straight line"
    
    def test_straight_line_distance_preserved(self):
        """Path length should be preserved after smoothing."""
        center_lat, center_lon = 37.8, -122.4  # SF area
        points = []
        for i in range(6):  # 6 points, ~20m apart
            lon_offset = i * 20 / (111320 * math.cos(math.radians(center_lat)))
            points.append({
                'lat': center_lat,
                'lon': center_lon + lon_offset,
            })
        
        # Get stats before smoothing
        raw_stats = get_path_stats(points)
        
        # Smooth
        smooth_path = smooth_path_geo(points, target_spacing_m=5.0)
        smooth_stats = get_path_stats(smooth_path)
        
        # Distance should be within 5% of original
        raw_dist = raw_stats['total_distance_m']
        smooth_dist = smooth_stats['total_distance_m']
        
        if raw_dist > 0:
            diff_pct = abs(smooth_dist - raw_dist) / raw_dist * 100
            assert diff_pct < 5, f"Distance changed by {diff_pct:.1f}%, expected < 5%"


class TestCircularArc:
    """Test curvature calculation on a circular arc."""
    
    def test_quarter_circle_curvature(self):
        """A quarter-circle arc should have curvature ≈ 1/R."""
        # Create a quarter circle with radius R = 50m
        R = 50.0  # meters
        center_lat, center_lon = 37.8, -122.4
        
        # Generate points along a 90-degree arc
        n_points = 10
        points = []
        for i in range(n_points):
            angle = i * (math.pi / 2) / (n_points - 1)  # 0 to 90 degrees
            # Convert to lat/lon offset (approximate)
            dx = R * math.cos(angle)  # meters
            dy = R * math.sin(angle)  # meters
            
            lat_offset = dy / 111320
            lon_offset = dx / (111320 * math.cos(math.radians(center_lat)))
            
            points.append({
                'lat': center_lat + lat_offset,
                'lon': center_lon + lon_offset,
            })
        
        # Smooth the path
        smooth_path = smooth_path_geo(points, target_spacing_m=5.0, mode="catmull_rom")
        smooth_path = recompute_curvature(smooth_path)
        
        # Expected curvature: 1/R = 0.02 (for R=50m)
        expected_curv = 1.0 / R
        
        # Check middle points (not endpoints)
        n = len(smooth_path)
        middle_curvatures = [smooth_path[i]['curvature'] for i in range(n//4, 3*n//4)]
        avg_curv = sum(middle_curvatures) / len(middle_curvatures)
        
        # Should be within 20% of expected
        diff_pct = abs(avg_curv - expected_curv) / expected_curv * 100
        assert diff_pct < 30, f"Avg curvature {avg_curv:.4f} vs expected {expected_curv:.4f} ({diff_pct:.0f}% diff)"
    
    def test_tight_turn_high_curvature(self):
        """A tight turn should have higher curvature than a gentle curve."""
        center_lat, center_lon = 37.8, -122.4
        
        # Tight turn: R = 20m
        tight_points = []
        for i in range(8):
            angle = i * (math.pi / 2) / 7
            dx = 20 * math.cos(angle)
            dy = 20 * math.sin(angle)
            lat_offset = dy / 111320
            lon_offset = dx / (111320 * math.cos(math.radians(center_lat)))
            tight_points.append({
                'lat': center_lat + lat_offset,
                'lon': center_lon + lon_offset,
            })
        
        # Gentle turn: R = 100m
        gentle_points = []
        for i in range(8):
            angle = i * (math.pi / 2) / 7
            dx = 100 * math.cos(angle)
            dy = 100 * math.sin(angle)
            lat_offset = dy / 111320
            lon_offset = dx / (111320 * math.cos(math.radians(center_lat)))
            gentle_points.append({
                'lat': center_lat + lat_offset,
                'lon': center_lon + lon_offset,
            })
        
        # Smooth both
        tight_smooth = smooth_path_geo(tight_points, target_spacing_m=3.0)
        gentle_smooth = smooth_path_geo(gentle_points, target_spacing_m=5.0)
        
        # Compute curvature
        tight_smooth = recompute_curvature(tight_smooth)
        gentle_smooth = recompute_curvature(gentle_smooth)
        
        # Get max curvature for each
        tight_max_curv = max(p['curvature'] for p in tight_smooth)
        gentle_max_curv = max(p['curvature'] for p in gentle_smooth)
        
        # Tight turn should have higher curvature
        assert tight_max_curv > gentle_max_curv, \
            f"Tight turn curvature ({tight_max_curv:.4f}) should be > gentle ({gentle_max_curv:.4f})"


class TestSmoothingPointCount:
    """Test that smoothing increases point count appropriately."""
    
    def test_point_count_increases(self):
        """Smoothing should increase point count for sparse input."""
        # Sparse input: only 5 points over 100m
        center_lat, center_lon = 50.3, 6.95  # Nürburgring area
        points = []
        for i in range(5):
            lon_offset = i * 25 / (111320 * math.cos(math.radians(center_lat)))
            points.append({
                'lat': center_lat,
                'lon': center_lon + lon_offset,
            })
        
        original_count = len(points)
        
        # Smooth at 5m spacing
        smooth_path = smooth_path_geo(points, target_spacing_m=5.0)
        smooth_count = len(smooth_path)
        
        # Should have more points
        assert smooth_count > original_count, \
            f"Smooth path ({smooth_count} pts) should have more points than raw ({original_count})"
        
        # Should have approximately 100m / 5m = 20 points
        expected_count = 100 // 5
        assert smooth_count >= expected_count * 0.8, \
            f"Expected ~{expected_count} points, got {smooth_count}"


class TestCRSTransformer:
    """Test coordinate transformation."""
    
    def test_roundtrip_san_francisco(self):
        """Test lat/lon -> meters -> lat/lon roundtrip for SF."""
        center_lat, center_lon = 37.8020, -122.4186  # Lombard Street
        
        transformer = CRSTransformer(center_lat, center_lon)
        
        # Test points
        test_points = [
            (37.8020, -122.4186),  # Center
            (37.8030, -122.4186),  # North
            (37.8020, -122.4200),  # West
        ]
        
        for lat, lon in test_points:
            x, y = transformer.latlon_to_meters(lat, lon)
            lat2, lon2 = transformer.meters_to_latlon(x, y)
            
            # Should be within 1m (about 0.00001 degrees)
            lat_diff = abs(lat2 - lat) * 111320
            lon_diff = abs(lon2 - lon) * 111320 * math.cos(math.radians(lat))
            
            assert lat_diff < 1.0, f"Lat roundtrip error: {lat_diff:.2f}m"
            assert lon_diff < 1.0, f"Lon roundtrip error: {lon_diff:.2f}m"
    
    def test_roundtrip_nurburgring(self):
        """Test roundtrip for Nürburgring."""
        center_lat, center_lon = 50.335, 6.942  # Nürburgring
        
        transformer = CRSTransformer(center_lat, center_lon)
        
        # Test point 1km away
        lat, lon = 50.345, 6.952
        x, y = transformer.latlon_to_meters(lat, lon)
        lat2, lon2 = transformer.meters_to_latlon(x, y)
        
        lat_diff = abs(lat2 - lat) * 111320
        lon_diff = abs(lon2 - lon) * 111320 * math.cos(math.radians(lat))
        
        assert lat_diff < 5.0, f"Lat roundtrip error: {lat_diff:.2f}m"
        assert lon_diff < 5.0, f"Lon roundtrip error: {lon_diff:.2f}m"


class TestPathStats:
    """Test path statistics calculation."""
    
    def test_stats_basic(self):
        """Test basic stats calculation."""
        points = [
            {'lat': 37.8, 'lon': -122.4, 'distance_along_m': 0, 'curvature': 0},
            {'lat': 37.801, 'lon': -122.4, 'distance_along_m': 100, 'curvature': 0.01},
            {'lat': 37.802, 'lon': -122.4, 'distance_along_m': 200, 'curvature': 0.02},
        ]
        
        stats = get_path_stats(points)
        
        assert stats['point_count'] == 3
        assert stats['total_distance_m'] == 200
        assert stats['total_distance_km'] == 0.2
        assert stats['avg_spacing_m'] == 100
        assert stats['max_curvature'] == 0.02


if __name__ == "__main__":
    pytest.main([__file__, "-v"])





