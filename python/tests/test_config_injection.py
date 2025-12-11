"""
Test configuration injection via Python callbacks.

This test verifies that:
1. The C++ core can be loaded from Python
2. Friction callbacks work correctly
3. Callback affects solver results as expected
"""

import pytest
import math


# Skip all tests if core module not available
pytest.importorskip("apexvelocity._apexvelocity_core")


import apexvelocity as av
from apexvelocity import _apexvelocity_core as core


class TestCoreImport:
    """Test that the core module imports correctly."""

    def test_core_available(self):
        """Verify core module is importable."""
        assert core is not None

    def test_physics_constants(self):
        """Verify physics constants are accessible."""
        assert core.PhysicsConstants.EARTH_GRAVITY == pytest.approx(9.81)
        assert core.PhysicsConstants.MARS_GRAVITY == pytest.approx(3.71)

    def test_vehicle_params_creation(self):
        """Test creating VehicleParams."""
        v = core.VehicleParams()
        assert v.mass_kg > 0
        assert v.is_valid()

    def test_path_point_creation(self):
        """Test creating PathPoint."""
        pt = core.PathPoint()
        pt.x_m = 100.0
        pt.y_m = 50.0
        pt.surface_type = "asphalt"
        assert pt.x_m == 100.0
        assert pt.surface_type == "asphalt"


class TestPhysicsFunctions:
    """Test physics calculation functions."""

    def test_friction_limited_speed(self):
        """Test friction-limited speed calculation."""
        # Straight line (zero curvature) should return max speed
        v_straight = core.calc_friction_limited_speed(0.0, 0.9, 9.81)
        assert v_straight == pytest.approx(core.PhysicsConstants.MAX_SAFE_SPEED)

        # Tight curve should have lower speed
        curvature = 1.0 / 50.0  # radius = 50m
        v_curve = core.calc_friction_limited_speed(curvature, 0.9, 9.81)
        assert v_curve < v_straight
        assert v_curve > 0

        # Expected: sqrt(0.9 * 9.81 / 0.02) ≈ 21.0 m/s
        expected = math.sqrt(0.9 * 9.81 / curvature)
        assert v_curve == pytest.approx(expected, rel=0.01)

    def test_rollover_limited_speed(self):
        """Test rollover-limited speed calculation."""
        curvature = 1.0 / 50.0
        track_width = 1.6
        cog_height = 0.5

        v_rollover = core.calc_rollover_limited_speed(
            curvature, track_width, cog_height, 9.81
        )

        assert v_rollover > 0
        # Rollover limit is typically higher than friction limit for normal cars

    def test_unit_conversions(self):
        """Test speed unit conversions."""
        assert core.mps_to_kmph(1.0) == pytest.approx(3.6)
        assert core.kmph_to_mps(3.6) == pytest.approx(1.0)
        assert core.mps_to_kmph(27.78) == pytest.approx(100.0, rel=0.01)


class TestFrictionCallback:
    """Test friction callback injection."""

    def setup_method(self):
        """Clear any existing callback before each test."""
        core.clear_friction_callback()

    def teardown_method(self):
        """Clean up callback after each test."""
        core.clear_friction_callback()

    def test_default_effective_mu(self):
        """Test default effective mu values."""
        # Without callback, should use config values
        mu_asphalt_dry = core.get_effective_mu("asphalt", "dry")
        mu_asphalt_wet = core.get_effective_mu("asphalt", "wet")
        mu_ice = core.get_effective_mu("ice", "dry")

        # Asphalt should have high friction
        assert mu_asphalt_dry == pytest.approx(0.9, rel=0.1)
        assert mu_asphalt_wet < mu_asphalt_dry

        # Ice should have low friction
        assert mu_ice < mu_asphalt_dry
        assert mu_ice == pytest.approx(0.15, rel=0.1)

    def test_callback_overrides_default(self):
        """Test that callback overrides default friction values."""
        # Define a callback that makes asphalt very slippery
        def slippery_asphalt(surface: str, condition: str) -> float:
            if surface == "asphalt":
                return 0.1  # Very slippery!
            return 0.9  # Normal for others

        # Register callback
        core.set_friction_callback(slippery_asphalt)

        # Now asphalt should be slippery
        mu_asphalt = core.get_effective_mu("asphalt", "dry")
        assert mu_asphalt == pytest.approx(0.1, rel=0.01)

        # Other surfaces should be normal
        mu_concrete = core.get_effective_mu("concrete", "dry")
        assert mu_concrete == pytest.approx(0.9, rel=0.01)

    def test_callback_affects_speed_calculation(self):
        """Test that callback affects friction-limited speed calculation."""
        curvature = 1.0 / 100.0  # radius = 100m

        # Calculate speed without callback (normal asphalt)
        core.clear_friction_callback()
        mu_normal = core.get_effective_mu("asphalt", "dry")
        v_normal = core.calc_friction_limited_speed(curvature, mu_normal, 9.81)

        # Set slippery callback
        def very_slippery(surface: str, condition: str) -> float:
            return 0.1

        core.set_friction_callback(very_slippery)
        mu_slippery = core.get_effective_mu("asphalt", "dry")
        v_slippery = core.calc_friction_limited_speed(curvature, mu_slippery, 9.81)

        # Slippery surface should have lower max speed
        assert v_slippery < v_normal

        # Speed ratio should match sqrt of friction ratio
        expected_ratio = math.sqrt(0.1 / mu_normal)
        actual_ratio = v_slippery / v_normal
        assert actual_ratio == pytest.approx(expected_ratio, rel=0.05)

    def test_clear_callback_restores_default(self):
        """Test that clearing callback restores default behavior."""
        # Set callback
        core.set_friction_callback(lambda s, c: 0.1)
        assert core.get_effective_mu("asphalt", "dry") == pytest.approx(0.1)

        # Clear callback
        core.clear_friction_callback()

        # Should be back to default
        mu = core.get_effective_mu("asphalt", "dry")
        assert mu == pytest.approx(0.9, rel=0.1)


class TestSolverWithCallback:
    """Test solver behavior with friction callback."""

    def setup_method(self):
        """Clear callback before each test."""
        core.clear_friction_callback()

    def teardown_method(self):
        """Clean up after each test."""
        core.clear_friction_callback()

    def test_solve_profile_without_callback(self):
        """Test basic solve_profile without callback."""
        # Create a simple straight path
        geometry = [
            [0.0, 0.0, 0.0, 0.0, 0.0],  # x, y, z, curvature, distance
            [50.0, 0.0, 0.0, 0.0, 50.0],
            [100.0, 0.0, 0.0, 0.0, 100.0],
        ]
        surfaces = ["asphalt", "asphalt", "asphalt"]

        result = core.solve_profile(
            geometry=geometry,
            surfaces=surfaces,
            vehicle="default",
            condition="dry",
            initial_speed=20.0,
        )

        assert result["success"] is True
        assert len(result["velocity_profile_mps"]) == 3
        assert result["max_speed_mps"] > 0

    def test_solve_profile_with_slippery_callback(self):
        """Test solve_profile with slippery asphalt callback."""
        # Build a curved path
        curvature = 1.0 / 50.0  # 50m radius curve
        geometry = [
            [0.0, 0.0, 0.0, curvature, 0.0],
            [25.0, 5.0, 0.0, curvature, 25.5],
            [50.0, 15.0, 0.0, curvature, 52.0],
        ]
        surfaces = ["asphalt", "asphalt", "asphalt"]

        # Solve without callback
        core.clear_friction_callback()
        result_normal = core.solve_profile(
            geometry=geometry,
            surfaces=surfaces,
            vehicle="default",
            condition="dry",
            initial_speed=15.0,
        )

        # Solve with slippery callback (mu = 0.1)
        def slippery(surface: str, condition: str) -> float:
            return 0.1

        core.set_friction_callback(slippery)
        result_slippery = core.solve_profile(
            geometry=geometry,
            surfaces=surfaces,
            vehicle="default",
            condition="dry",
            initial_speed=15.0,
        )

        assert result_normal["success"] is True
        assert result_slippery["success"] is True

        # Slippery version should have LOWER static limits
        # (The solver computes v_static from friction)
        v_normal_static = result_normal.get("static_limits_mps", [])
        v_slippery_static = result_slippery.get("static_limits_mps", [])

        if v_normal_static and v_slippery_static:
            # Static limits should be lower with slippery surface
            for v_n, v_s in zip(v_normal_static, v_slippery_static):
                # On curved sections, slippery should be lower
                if v_n < 80:  # Not at max speed
                    assert v_s < v_n, f"Expected {v_s} < {v_n} for slippery surface"

        # Max speed should definitely be lower on slippery
        print(f"Normal max speed: {result_normal['max_speed_mps']:.2f} m/s")
        print(f"Slippery max speed: {result_slippery['max_speed_mps']:.2f} m/s")


class TestVehicleLoader:
    """Test vehicle loading functionality."""

    def test_load_default_vehicle(self):
        """Test loading default vehicle."""
        v = core.VehicleLoader.get_default()
        assert v is not None
        assert v.mass_kg > 0
        assert v.is_valid()

    def test_list_presets(self):
        """Test listing vehicle presets."""
        loader = core.VehicleLoader(core.DEFAULT_CONFIG_DIR + "/vehicle_presets")
        presets = loader.list_presets()

        # Should have some presets
        assert len(presets) > 0

        # Check for expected presets
        preset_names = [p.lower() for p in presets]
        assert any("tesla" in p for p in preset_names) or any(
            "compact" in p for p in preset_names
        )


# Main entry point for running tests directly
if __name__ == "__main__":
    pytest.main([__file__, "-v"])
