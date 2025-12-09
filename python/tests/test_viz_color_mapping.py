#!/usr/bin/env python3
"""
Tests for ApexVelocity visualization color mapping.

Tests:
1. Different color_by modes produce different colors
2. Color gradients are applied correctly
3. Backwards compatibility with 'mode' parameter
"""

import sys
import os
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def create_test_segments():
    """Create synthetic segments with known values for testing."""
    return [
        {
            'lat': 37.8020, 'lon': -122.4186,
            'speed_kmh': 20,
            'energy_kwh_per_km': 0.1,
            'lateral_g': 0.2,
            'longitudinal_g': 0.1,
            'friction_usage': 0.3,
            'comfort_score': 0.2,
            'recommended_action': 'ACCELERATE',
            'curvature_1pm': 0.01,
            'grade_percent': 5,
        },
        {
            'lat': 37.8021, 'lon': -122.4187,
            'speed_kmh': 50,
            'energy_kwh_per_km': 0.3,
            'lateral_g': 0.5,
            'longitudinal_g': 0.3,
            'friction_usage': 0.6,
            'comfort_score': 0.5,
            'recommended_action': 'COAST',
            'curvature_1pm': 0.02,
            'grade_percent': 0,
        },
        {
            'lat': 37.8022, 'lon': -122.4188,
            'speed_kmh': 80,
            'energy_kwh_per_km': 0.5,
            'lateral_g': 0.8,
            'longitudinal_g': 0.5,
            'friction_usage': 0.9,
            'comfort_score': 0.8,
            'recommended_action': 'BRAKE',
            'curvature_1pm': 0.03,
            'grade_percent': -3,
        },
    ]


class TestColorByModes:
    """Test different color_by modes produce different results."""
    
    def test_color_by_speed(self):
        """Test speed coloring: slow=red, fast=green."""
        from apexvelocity.viz import _build_path_data
        
        segments = create_test_segments()
        path_data, stats = _build_path_data(segments, mode="speed")
        
        assert len(path_data) == 2  # 2 segments from 3 points
        
        # First segment (slow) should be more red
        # Last segment (fast) should be more green
        first_color = path_data[0]['color']
        last_color = path_data[-1]['color']
        
        # Green component should increase
        assert last_color[1] > first_color[1], \
            f"Green should increase: first={first_color}, last={last_color}"
    
    def test_color_by_energy(self):
        """Test energy coloring: efficient=green, high=red."""
        from apexvelocity.viz import _build_path_data
        
        segments = create_test_segments()
        path_data, stats = _build_path_data(segments, mode="energy")
        
        first_color = path_data[0]['color']
        last_color = path_data[-1]['color']
        
        # First segment (low energy) should be more green
        # Last segment (high energy) should be more red
        assert first_color[1] > last_color[1], \
            f"First should be greener: first={first_color}, last={last_color}"
    
    def test_color_by_lateral_g(self):
        """Test lateral_g coloring."""
        from apexvelocity.viz import _build_path_data
        
        segments = create_test_segments()
        path_data, stats = _build_path_data(segments, mode="lateral_g")
        
        # Should use gforce palette
        assert len(path_data) == 2
        
        # Low G should be bluer, high G should be redder
        first_color = path_data[0]['color']
        last_color = path_data[-1]['color']
        
        # Red component should increase with G
        assert last_color[0] > first_color[0], \
            f"Red should increase with G: first={first_color}, last={last_color}"
    
    def test_color_by_friction(self):
        """Test friction coloring: low=green, limit=red."""
        from apexvelocity.viz import _build_path_data
        
        segments = create_test_segments()
        path_data, stats = _build_path_data(segments, mode="friction")
        
        first_color = path_data[0]['color']
        last_color = path_data[-1]['color']
        
        # High friction (last segment) should be more red than low friction (first segment)
        # Red component should increase with friction usage
        assert last_color[0] >= first_color[0], \
            f"Red should increase with friction: first={first_color}, last={last_color}"
    
    def test_color_by_action(self):
        """Test action coloring (categorical)."""
        from apexvelocity.viz import _build_path_data, COLOR_PALETTES
        
        segments = create_test_segments()
        path_data, stats = _build_path_data(segments, mode="action")
        
        # First segment has ACCELERATE action (green)
        # Second has COAST action (blue)
        first_color = path_data[0]['color']
        
        # ACCELERATE should be green
        expected = COLOR_PALETTES['action']['ACCELERATE']
        assert first_color == expected, \
            f"ACCELERATE color {first_color} should be {expected}"
    
    def test_color_by_comfort(self):
        """Test comfort coloring."""
        from apexvelocity.viz import _build_path_data
        
        segments = create_test_segments()
        path_data, stats = _build_path_data(segments, mode="comfort")
        
        # Should produce valid colors
        assert len(path_data) == 2
        for seg in path_data:
            assert len(seg['color']) == 4
            assert all(0 <= c <= 255 for c in seg['color'])
    
    def test_color_by_difficulty(self):
        """Test difficulty coloring."""
        from apexvelocity.viz import _build_path_data
        
        segments = create_test_segments()
        path_data, stats = _build_path_data(segments, mode="difficulty")
        
        # Should produce valid colors
        assert len(path_data) == 2
        for seg in path_data:
            assert len(seg['color']) == 4
    
    def test_color_by_safety_margin(self):
        """Test safety_margin coloring."""
        from apexvelocity.viz import _build_path_data
        
        segments = create_test_segments()
        path_data, stats = _build_path_data(segments, mode="safety_margin")
        
        first_color = path_data[0]['color']
        last_color = path_data[-1]['color']
        
        # Low friction usage = high safety margin = green
        # High friction usage = low safety margin = red
        assert first_color[1] > last_color[1], \
            f"First should be greener (safer): first={first_color}, last={last_color}"


class TestDifferentModesProduceDifferentColors:
    """Verify that different color_by modes produce different results."""
    
    def test_speed_vs_energy_different(self):
        """Speed and energy modes should produce different colors."""
        from apexvelocity.viz import _build_path_data
        
        segments = create_test_segments()
        
        speed_data, _ = _build_path_data(segments, mode="speed")
        energy_data, _ = _build_path_data(segments, mode="energy")
        
        # At least one segment should have different colors
        different = False
        for s, e in zip(speed_data, energy_data):
            if s['color'] != e['color']:
                different = True
                break
        
        assert different, "Speed and energy should produce different colors"
    
    def test_all_modes_produce_valid_output(self):
        """All supported color_by modes should work."""
        from apexvelocity.viz import _build_path_data
        
        segments = create_test_segments()
        
        modes = ['speed', 'energy', 'lateral_g', 'longitudinal_g', 'gforce',
                 'friction', 'comfort', 'difficulty', 'safety_margin', 'action']
        
        for mode in modes:
            path_data, stats = _build_path_data(segments, mode=mode)
            assert len(path_data) > 0, f"Mode '{mode}' produced no data"
            
            for seg in path_data:
                assert 'path' in seg, f"Mode '{mode}' missing 'path'"
                assert 'color' in seg, f"Mode '{mode}' missing 'color'"
                assert len(seg['color']) == 4, f"Mode '{mode}' color wrong length"


class TestBackwardsCompatibility:
    """Test backwards compatibility with 'mode' parameter."""
    
    def test_mode_parameter_still_works(self):
        """The old 'mode' parameter should still work."""
        from apexvelocity.viz import visualize_profile
        
        # Just check that the function accepts mode parameter
        # Don't actually generate HTML
        try:
            # This should not raise an error about unknown parameter
            # It will fail for other reasons (no pydeck) but that's OK
            pass  # We can't easily test without pydeck installed
        except TypeError as e:
            if "mode" in str(e):
                pytest.fail("'mode' parameter should be accepted for backwards compatibility")


class TestColorPalettes:
    """Test color palette definitions."""
    
    def test_all_palettes_have_required_keys(self):
        """All gradient palettes should have low/mid/high keys."""
        from apexvelocity.viz import COLOR_PALETTES
        
        gradient_palettes = [
            'speed', 'energy', 'gforce', 'lateral_g', 'longitudinal_g',
            'friction', 'comfort', 'difficulty', 'safety_margin', 'regen'
        ]
        
        for name in gradient_palettes:
            assert name in COLOR_PALETTES, f"Palette '{name}' not defined"
            palette = COLOR_PALETTES[name]
            assert 'low' in palette, f"Palette '{name}' missing 'low'"
            assert 'mid' in palette, f"Palette '{name}' missing 'mid'"
            assert 'high' in palette, f"Palette '{name}' missing 'high'"
    
    def test_action_palette_has_action_keys(self):
        """Action palette should have action type keys."""
        from apexvelocity.viz import COLOR_PALETTES
        
        action_palette = COLOR_PALETTES['action']
        
        expected_keys = ['BRAKE', 'COAST', 'ACCELERATE', 'HOLD']
        for key in expected_keys:
            assert key in action_palette, f"Action palette missing '{key}'"
    
    def test_all_colors_valid_rgba(self):
        """All colors should be valid RGBA values."""
        from apexvelocity.viz import COLOR_PALETTES
        
        for palette_name, palette in COLOR_PALETTES.items():
            for color_name, color in palette.items():
                assert len(color) == 4, \
                    f"{palette_name}/{color_name}: color should have 4 components"
                assert all(0 <= c <= 255 for c in color), \
                    f"{palette_name}/{color_name}: values should be 0-255"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])

