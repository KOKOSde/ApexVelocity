#!/usr/bin/env python3
"""
Tests for ApexVelocity demo path generation.

Validates that SF Lombard Street and Nürburgring paths:
1. Have reasonable path lengths
2. Have non-zero curvature at appropriate points
3. Coordinates are in expected bounding boxes
"""

import sys
import os
import math
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


class TestLombardStreet:
    """Test San Francisco Lombard Street path generation."""
    
    def test_path_length_in_range(self):
        """Lombard Street crooked section should be 150-250m."""
        from examples.demo_san_francisco import create_lombard_path
        
        path = create_lombard_path()
        total_dist = path[-1]['distance_along_m']
        
        assert 150 <= total_dist <= 250, \
            f"Path length {total_dist:.0f}m should be 150-250m"
    
    def test_path_has_enough_points(self):
        """Smoothed path should have at least 50 points."""
        from examples.demo_san_francisco import create_lombard_path
        
        path = create_lombard_path()
        n_points = len(path)
        
        assert n_points >= 50, \
            f"Path has {n_points} points, expected at least 50"
    
    def test_coordinates_in_san_francisco(self):
        """Coordinates should be in San Francisco."""
        from examples.demo_san_francisco import create_lombard_path
        
        path = create_lombard_path()
        
        lats = [p['lat'] for p in path]
        lons = [p['lon'] for p in path]
        
        # Lombard Street bounds
        assert 37.801 <= min(lats) <= max(lats) <= 37.803, \
            f"Latitudes {min(lats):.4f} to {max(lats):.4f} should be ~37.802"
        assert -122.421 <= min(lons) <= max(lons) <= -122.418, \
            f"Longitudes {min(lons):.4f} to {max(lons):.4f} should be ~-122.419"
    
    def test_curvature_nonzero_for_hairpins(self):
        """Path should have significant curvature for hairpin turns."""
        from examples.demo_san_francisco import create_lombard_path
        
        path = create_lombard_path()
        curvatures = [p.get('curvature', 0) for p in path]
        
        # Remove zeros and get stats
        nonzero_curvs = [c for c in curvatures if c > 0.001]
        
        assert len(nonzero_curvs) > 10, \
            f"Only {len(nonzero_curvs)} points with curvature > 0.001"
        
        max_curv = max(curvatures)
        # 8 hairpin turns should produce curvature > 0.01 (radius < 100m)
        assert max_curv > 0.01, \
            f"Max curvature {max_curv:.4f} should be > 0.01 for hairpins"
    
    def test_lat_lon_fields_present(self):
        """All points should have lat and lon fields."""
        from examples.demo_san_francisco import create_lombard_path
        
        path = create_lombard_path()
        
        for i, p in enumerate(path):
            assert 'lat' in p, f"Point {i} missing 'lat'"
            assert 'lon' in p, f"Point {i} missing 'lon'"
            assert abs(p['lat']) > 0.001, f"Point {i} has zero lat"
            assert abs(p['lon']) > 0.001, f"Point {i} has zero lon"


class TestNurburgring:
    """Test Nürburgring Nordschleife path generation."""
    
    def test_path_length_in_range(self):
        """Nordschleife should be 17-25km (we're approximate)."""
        from examples.benchmark_nurburgring import create_track_path
        
        path = create_track_path()
        total_km = path[-1]['distance_along_m'] / 1000
        
        assert 17 <= total_km <= 25, \
            f"Track length {total_km:.1f}km should be 17-25km"
    
    def test_path_has_enough_points(self):
        """Smoothed path should have at least 500 points."""
        from examples.benchmark_nurburgring import create_track_path
        
        path = create_track_path()
        n_points = len(path)
        
        # With 10m spacing over ~20km, expect ~2000 points
        assert n_points >= 500, \
            f"Track has {n_points} points, expected at least 500"
    
    def test_coordinates_in_germany(self):
        """Coordinates should be in Nürburgring area."""
        from examples.benchmark_nurburgring import create_track_path
        
        path = create_track_path()
        
        lats = [p['lat'] for p in path]
        lons = [p['lon'] for p in path]
        
        # Nürburgring bounds (approximate)
        assert 50.31 <= min(lats) <= max(lats) <= 50.36, \
            f"Latitudes {min(lats):.4f} to {max(lats):.4f} should be ~50.33"
        assert 6.90 <= min(lons) <= max(lons) <= 7.05, \
            f"Longitudes {min(lons):.4f} to {max(lons):.4f} should be ~6.95"
    
    def test_track_dimensions_reasonable(self):
        """Track should span ~7km E-W and ~5km N-S."""
        from examples.benchmark_nurburgring import create_track_path
        
        path = create_track_path()
        
        lats = [p['lat'] for p in path]
        lons = [p['lon'] for p in path]
        
        # Compute dimensions
        lat_range = max(lats) - min(lats)
        lon_range = max(lons) - min(lons)
        
        ns_km = lat_range * 111.32
        ew_km = lon_range * 111.32 * math.cos(math.radians(sum(lats) / len(lats)))
        
        assert 3 <= ns_km <= 7, f"N-S dimension {ns_km:.1f}km should be 3-7km"
        assert 5 <= ew_km <= 10, f"E-W dimension {ew_km:.1f}km should be 5-10km"
    
    def test_curvature_distribution_reasonable(self):
        """Track should have varied curvature (straights and corners)."""
        from examples.benchmark_nurburgring import create_track_path
        
        path = create_track_path()
        curvatures = [p.get('curvature', 0) for p in path]
        
        # Should have some straights (low curvature)
        n_straight = sum(1 for c in curvatures if c < 0.002)
        
        # Should have some medium/tight corners (curvature > 0.005 = radius < 200m)
        n_corners = sum(1 for c in curvatures if c > 0.005)
        
        total = len(curvatures)
        
        assert n_straight > total * 0.2, \
            f"Only {n_straight}/{total} straight segments, expected > 20%"
        assert n_corners > 10, \
            f"Only {n_corners} corner segments with curvature > 0.005"
        
        # Should have at least some non-zero curvature overall
        max_curv = max(curvatures)
        assert max_curv > 0.005, f"Max curvature {max_curv:.4f} should be > 0.005"
    
    def test_track_forms_loop(self):
        """Track should be a closed loop (start ≈ end)."""
        from examples.benchmark_nurburgring import create_track_path
        
        path = create_track_path()
        
        start = path[0]
        end = path[-1]
        
        # Start and end should be within 100m
        dlat = (end['lat'] - start['lat']) * 111320
        dlon = (end['lon'] - start['lon']) * 111320 * math.cos(math.radians(start['lat']))
        gap = math.sqrt(dlat**2 + dlon**2)
        
        assert gap < 100, \
            f"Track gap {gap:.0f}m should be < 100m for a closed loop"


class TestSmoothingEffect:
    """Test that smoothing actually improves the paths."""
    
    def test_sf_smooth_vs_raw(self):
        """Smoothed SF path should have more points than raw."""
        from examples.demo_san_francisco import create_raw_lombard_path, create_lombard_path
        
        raw = create_raw_lombard_path()
        smooth = create_lombard_path()
        
        assert len(smooth) > len(raw), \
            f"Smooth path ({len(smooth)} pts) should have more points than raw ({len(raw)})"
    
    def test_nurburgring_smooth_vs_raw(self):
        """Smoothed Nürburgring path should have more points than raw."""
        from examples.benchmark_nurburgring import create_raw_track_path, create_track_path
        
        raw = create_raw_track_path()
        smooth = create_track_path()
        
        assert len(smooth) > len(raw), \
            f"Smooth track ({len(smooth)} pts) should have more points than raw ({len(raw)})"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])

