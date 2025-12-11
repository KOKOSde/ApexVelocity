#!/usr/bin/env python3
"""
Comprehensive physics validation test suite for ApexVelocity.

Tests all critical physics behaviors identified in the audit:
1. Hill climbing energy asymmetry
2. Wet vs dry friction
3. Friction-limited cornering
4. Energy consumption accuracy
5. Advanced physics model availability
"""

import sys
import os
import math
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from apexvelocity import api as av


def create_hill_path(length_m=1000, elevation_change_m=50, num_points=200):
    """Create an uphill path."""
    coords = []
    for i in range(num_points):
        s = i * (length_m / (num_points - 1))
        z = elevation_change_m * (s / length_m)
        coords.append((s, 0.0, z))
    return coords


def create_curved_path(radius_m=30, angle_deg=90, num_points=50):
    """Create a curved path (quarter circle)."""
    angle_rad = math.radians(angle_deg)
    coords = []
    for i in range(num_points):
        theta = angle_rad * i / (num_points - 1)
        x = radius_m * math.cos(theta)
        y = radius_m * math.sin(theta)
        coords.append((x, y, 0.0))
    return coords


def create_flat_straight(length_m=1000, num_points=100):
    """Create a flat straight path."""
    return [(i * (length_m / (num_points - 1)), 0.0, 0.0) for i in range(num_points)]


class TestHillClimbingEnergy:
    """Test 1: Hill climbing energy asymmetry."""

    def test_uphill_requires_more_energy_than_downhill(self):
        """Uphill should require more energy than downhill due to gravity."""
        coords_up = create_hill_path(length_m=1000, elevation_change_m=50)
        coords_down = list(reversed(coords_up))

        res_up = av.solve(coords_up, vehicle="tesla_model_3", condition="dry")
        res_down = av.solve(coords_down, vehicle="tesla_model_3", condition="dry")

        up_kwh = res_up.total_energy_joules / 3.6e6
        down_kwh = res_down.total_energy_joules / 3.6e6

        # Downhill can be 0 (regen scenario) or small positive
        # Uphill must be significantly higher
        assert up_kwh > 0.2, f"Uphill energy {up_kwh:.3f} kWh too low"

        # If downhill is non-zero, check ratio
        if down_kwh > 0.01:
            ratio = up_kwh / down_kwh
            assert ratio > 1.2, f"Up/down ratio {ratio:.2f} should be > 1.2"

        print(f"✅ Hill energy: Uphill={up_kwh:.3f} kWh, Downhill={down_kwh:.3f} kWh")


class TestWetVsDryFriction:
    """Test 2: Wet vs dry friction model."""

    def test_wet_slower_than_dry(self):
        """Wet conditions should reduce cornering speeds."""
        coords = create_curved_path(radius_m=30, angle_deg=90)

        res_dry = av.solve(coords, vehicle="compact_car", condition="dry")
        res_wet = av.solve(coords, vehicle="compact_car", condition="wet")

        max_dry = max(res_dry.velocity_profile_mps) * 3.6
        max_wet = max(res_wet.velocity_profile_mps) * 3.6

        slowdown_pct = (1 - max_wet / max_dry) * 100

        assert max_wet < max_dry, f"Wet {max_wet:.1f} should be < dry {max_dry:.1f}"
        assert (
            5 < slowdown_pct < 30
        ), f"Wet slowdown {slowdown_pct:.1f}% should be 5-30%"

        print(
            f"✅ Wet vs dry: Dry={max_dry:.1f} km/h, Wet={max_wet:.1f} km/h, Slowdown={slowdown_pct:.1f}%"
        )


class TestFrictionLimitedCornering:
    """Test 3: Friction-limited cornering speeds."""

    def test_curved_path_respects_friction_limit(self):
        """Speed on curves should respect friction limits."""
        radius_m = 30.0
        mu_dry = 0.9  # From materials.json
        g = 9.81

        coords = create_curved_path(radius_m=radius_m, angle_deg=90)
        result = av.solve(coords, vehicle="compact_car", condition="dry")

        max_speed_kmh = max(result.velocity_profile_mps) * 3.6
        theoretical_max = math.sqrt(mu_dry * g * radius_m) * 3.6

        ratio = max_speed_kmh / theoretical_max

        # Should be within 20% of theoretical (accounting for rollover, vehicle limits)
        assert 0.8 < ratio < 1.2, f"Speed ratio {ratio:.2f} should be 0.8-1.2"
        assert (
            max_speed_kmh < 70
        ), f"Speed {max_speed_kmh:.1f} km/h too high for R={radius_m}m"

        print(
            f"✅ Friction limit: Actual={max_speed_kmh:.1f} km/h, Theoretical={theoretical_max:.1f} km/h, Ratio={ratio:.2f}"
        )


class TestEnergyConsumption:
    """Test 4: Energy consumption accuracy."""

    def test_flat_straight_energy_realistic(self):
        """Energy consumption should match real-world Tesla Model 3 values."""
        coords = create_flat_straight(length_m=1000)
        result = av.solve(coords, vehicle="tesla_model_3", condition="dry")

        energy_kwh = result.total_energy_joules / 3.6e6
        wh_per_km = energy_kwh * 1000 / 1.0

        # Tesla Model 3 real-world: 150-220 Wh/km
        # Allow wider range for different speeds
        assert 80 < wh_per_km < 300, f"Energy {wh_per_km:.1f} Wh/km should be 80-300"

        print(f"✅ Energy: {wh_per_km:.1f} Wh/km (expected 100-250 Wh/km)")


class TestAdvancedPhysics:
    """Test 5: Advanced physics features."""

    def test_tire_model_files_present(self):
        """TireModel.h/cpp should exist."""
        import os

        repo_root = os.path.dirname(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        )
        tire_h = os.path.join(repo_root, "core/include/physics/TireModel.h")
        tire_cpp = os.path.join(repo_root, "core/src/physics/TireModel.cpp")

        assert os.path.exists(tire_h), "TireModel.h not found"
        assert os.path.exists(tire_cpp), "TireModel.cpp not found"

        print("✅ TireModel files present")

    def test_dynamic_vehicle_files_present(self):
        """DynamicVehicle.h/cpp should exist."""
        import os

        repo_root = os.path.dirname(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        )
        dyn_h = os.path.join(repo_root, "core/include/physics/DynamicVehicle.h")
        dyn_cpp = os.path.join(repo_root, "core/src/physics/DynamicVehicle.cpp")

        assert os.path.exists(dyn_h), "DynamicVehicle.h not found"
        assert os.path.exists(dyn_cpp), "DynamicVehicle.cpp not found"

        print("✅ DynamicVehicle files present")

    def test_physics_model_enum_exposed(self):
        """PhysicsModel enum should be accessible from Python."""
        try:
            from apexvelocity import _apexvelocity_core as core

            assert hasattr(core, "PhysicsModel"), "PhysicsModel not exposed"
            assert hasattr(
                core.PhysicsModel, "KINEMATIC"
            ), "KINEMATIC not in PhysicsModel"
            assert hasattr(core.PhysicsModel, "DYNAMIC"), "DYNAMIC not in PhysicsModel"
            print("✅ PhysicsModel enum exposed to Python")
        except ImportError:
            pytest.skip("Core module not available")

    def test_dynamic_model_callable(self):
        """Should be able to call solve with physics_model='dynamic'."""
        coords = create_curved_path(radius_m=50, angle_deg=45)

        # Should not raise an error
        result = av.solve(
            coords, vehicle="compact_car", condition="dry", physics_model="dynamic"
        )
        assert result.success, "Dynamic model solve failed"
        assert len(result.velocity_profile_mps) > 0, "No velocity profile returned"

        print("✅ Dynamic physics model callable from Python API")


def print_summary():
    """Print test summary."""
    print("\n" + "=" * 70)
    print(" APEXVELOCITY PHYSICS VALIDATION SUITE - SUMMARY")
    print("=" * 70)


if __name__ == "__main__":
    print_summary()
    pytest.main([__file__, "-v", "--tb=short"])
