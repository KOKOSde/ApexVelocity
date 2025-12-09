/**
 * @file test_config_physics.cpp
 * @brief GoogleTest suite for ConfigManager and PhysicsMath components.
 * 
 * Tests cover:
 * - Material library loading and lookup
 * - Simulation parameter loading
 * - Friction-limited speed calculations
 * - Rollover-limited speed calculations
 * - Earth vs Mars gravity comparisons
 */

#include <gtest/gtest.h>
#include <cmath>
#include <fstream>
#include <filesystem>

#include "utils/ConfigManager.h"
#include "physics/PhysicsMath.h"

namespace apex {
namespace testing {

// Tolerance for floating-point comparisons
constexpr double EPSILON = 1e-6;

// ============================================================================
// Test Fixture for ConfigManager Tests
// ============================================================================
class ConfigManagerTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Initialize ConfigManager with the test config directory
        ConfigManager::instance().initialize(TEST_CONFIG_DIR);
    }
};

// ============================================================================
// Test 1: Material Library
// ============================================================================
TEST_F(ConfigManagerTest, MaterialLibrary_GravelMuDry) {
    const auto& gravel = ConfigManager::instance().get_material("gravel");
    EXPECT_NEAR(gravel.mu_dry, 0.60, EPSILON);
}

TEST_F(ConfigManagerTest, MaterialLibrary_GravelMuWet) {
    const auto& gravel = ConfigManager::instance().get_material("gravel");
    EXPECT_NEAR(gravel.mu_wet, 0.45, EPSILON);
}

TEST_F(ConfigManagerTest, MaterialLibrary_GravelRollingResistance) {
    const auto& gravel = ConfigManager::instance().get_material("gravel");
    EXPECT_NEAR(gravel.rolling_resistance_coeff, 0.020, EPSILON);
}

TEST_F(ConfigManagerTest, MaterialLibrary_AsphaltRollingResistance) {
    const auto& asphalt = ConfigManager::instance().get_material("asphalt");
    EXPECT_NEAR(asphalt.rolling_resistance_coeff, 0.015, EPSILON);
}

TEST_F(ConfigManagerTest, MaterialLibrary_AsphaltMuDry) {
    const auto& asphalt = ConfigManager::instance().get_material("asphalt");
    EXPECT_NEAR(asphalt.mu_dry, 0.90, EPSILON);
}

TEST_F(ConfigManagerTest, MaterialLibrary_IceMuDry) {
    const auto& ice = ConfigManager::instance().get_material("ice");
    EXPECT_NEAR(ice.mu_dry, 0.15, EPSILON);
}

TEST_F(ConfigManagerTest, MaterialLibrary_UnknownMaterialReturnsDefault) {
    // Unknown material should return default (asphalt-like) values
    const auto& unknown = ConfigManager::instance().get_material("unknown_surface");
    EXPECT_NEAR(unknown.mu_dry, 0.9, EPSILON);
    EXPECT_NEAR(unknown.mu_wet, 0.6, EPSILON);
    EXPECT_NEAR(unknown.rolling_resistance_coeff, 0.015, EPSILON);
}

TEST_F(ConfigManagerTest, MaterialLibrary_AllMaterialsPresent) {
    // Verify all expected materials are loaded
    std::vector<std::string> expected_materials = {
        "asphalt", "concrete", "cobblestone", "gravel", 
        "dirt", "sand", "ice", "snow_packed"
    };
    
    for (const auto& mat_name : expected_materials) {
        const auto& mat = ConfigManager::instance().get_material(mat_name);
        EXPECT_GT(mat.mu_dry, 0.0) << "Material " << mat_name << " has invalid mu_dry";
        EXPECT_GT(mat.mu_wet, 0.0) << "Material " << mat_name << " has invalid mu_wet";
        EXPECT_GT(mat.rolling_resistance_coeff, 0.0) << "Material " << mat_name << " has invalid rolling resistance";
    }
}

// ============================================================================
// Test 2: Simulation Config Parameters
// ============================================================================
TEST_F(ConfigManagerTest, SimConfig_GravityDefault) {
    double gravity = ConfigManager::instance().get_sim_param<double>("gravity");
    EXPECT_NEAR(gravity, 9.81, EPSILON);
}

TEST_F(ConfigManagerTest, SimConfig_AirDensity) {
    double air_density = ConfigManager::instance().get_sim_param<double>("air_density");
    EXPECT_NEAR(air_density, 1.225, EPSILON);
}

TEST_F(ConfigManagerTest, SimConfig_StepSize) {
    double step_size = ConfigManager::instance().get_sim_param<double>("step_size_meters");
    EXPECT_NEAR(step_size, 1.0, EPSILON);
}

TEST_F(ConfigManagerTest, SimConfig_EnableRolloverChecks) {
    bool enable_rollover = ConfigManager::instance().get_sim_param<bool>("enable_rollover_checks");
    EXPECT_TRUE(enable_rollover);
}

TEST_F(ConfigManagerTest, SimConfig_GlobalFrictionMultiplier) {
    double multiplier = ConfigManager::instance().get_sim_param<double>("global_friction_multiplier");
    EXPECT_NEAR(multiplier, 1.0, EPSILON);
}

TEST_F(ConfigManagerTest, SimConfig_MissingParamThrows) {
    EXPECT_THROW(
        ConfigManager::instance().get_sim_param<double>("nonexistent_param"),
        std::runtime_error
    );
}

TEST_F(ConfigManagerTest, SimConfig_GetParamWithDefault) {
    // Test get_sim_param_or with default value
    double value = ConfigManager::instance().get_sim_param_or<double>("nonexistent", 42.0);
    EXPECT_NEAR(value, 42.0, EPSILON);
}

// ============================================================================
// Effective Mu Tests
// ============================================================================
TEST_F(ConfigManagerTest, EffectiveMu_DryCondition) {
    double mu = ConfigManager::instance().get_effective_mu("asphalt", "dry");
    // With global_friction_multiplier = 1.0, should equal mu_dry
    EXPECT_NEAR(mu, 0.90, EPSILON);
}

TEST_F(ConfigManagerTest, EffectiveMu_WetCondition) {
    double mu = ConfigManager::instance().get_effective_mu("asphalt", "wet");
    // With global_friction_multiplier = 1.0, should equal mu_wet
    EXPECT_NEAR(mu, 0.60, EPSILON);
}

// ============================================================================
// Tag Mapping Tests
// ============================================================================
TEST_F(ConfigManagerTest, TagMapping_HighwayMotorway) {
    std::string material = ConfigManager::instance().get_material_for_tag("highway=motorway");
    EXPECT_EQ(material, "asphalt");
}

TEST_F(ConfigManagerTest, TagMapping_HighwayTrack) {
    std::string material = ConfigManager::instance().get_material_for_tag("highway=track");
    EXPECT_EQ(material, "dirt");
}

TEST_F(ConfigManagerTest, TagMapping_SurfaceCobblestone) {
    std::string material = ConfigManager::instance().get_material_for_tag("surface=cobblestone");
    EXPECT_EQ(material, "cobblestone");
}

TEST_F(ConfigManagerTest, TagMapping_UnknownTagReturnsDefault) {
    std::string material = ConfigManager::instance().get_material_for_tag("unknown=tag");
    EXPECT_EQ(material, "asphalt");  // Default fallback
}

// ============================================================================
// Test 3: Physics Math - Friction Limited Speed
// ============================================================================
class PhysicsMathTest : public ::testing::Test {
protected:
    static constexpr double EARTH_G = 9.81;
    static constexpr double MARS_G = 3.71;
};

TEST_F(PhysicsMathTest, FrictionLimitedSpeed_BasicCalculation) {
    // curvature = 1/100 m^-1 (radius = 100m), mu = 0.9
    double curvature = 1.0 / 100.0;  // 0.01 m^-1
    double mu = 0.9;
    
    double v_max = physics::calc_friction_limited_speed_from_curvature(curvature, mu, EARTH_G);
    
    // Expected: sqrt(0.9 * 9.81 / 0.01) = sqrt(882.9) ≈ 29.71 m/s
    double expected = std::sqrt(mu * EARTH_G / curvature);
    EXPECT_NEAR(v_max, expected, EPSILON);
    EXPECT_NEAR(v_max, 29.714, 0.01);
}

TEST_F(PhysicsMathTest, FrictionLimitedSpeed_EarthVsMars) {
    // Test that Mars has lower max speed due to lower gravity
    double curvature = 1.0 / 100.0;  // radius = 100 m
    double mu = 0.9;
    
    double v_earth = physics::calc_friction_limited_speed_from_curvature(curvature, mu, EARTH_G);
    double v_mars = physics::calc_friction_limited_speed_from_curvature(curvature, mu, MARS_G);
    
    // Both should be positive
    EXPECT_GT(v_earth, 0.0);
    EXPECT_GT(v_mars, 0.0);
    
    // Mars should have lower max speed (less gravity = less friction force available)
    EXPECT_LT(v_mars, v_earth);
    
    // Verify the ratio roughly matches sqrt(gravity ratio)
    double expected_ratio = std::sqrt(MARS_G / EARTH_G);
    double actual_ratio = v_mars / v_earth;
    EXPECT_NEAR(actual_ratio, expected_ratio, EPSILON);
}

TEST_F(PhysicsMathTest, FrictionLimitedSpeed_StraightLine) {
    // Zero or very small curvature (straight line) should return max safe speed
    double curvature_zero = 0.0;
    double curvature_tiny = 1.0e-12;
    double mu = 0.9;
    
    double v_zero = physics::calc_friction_limited_speed_from_curvature(curvature_zero, mu, EARTH_G);
    double v_tiny = physics::calc_friction_limited_speed_from_curvature(curvature_tiny, mu, EARTH_G);
    
    EXPECT_NEAR(v_zero, physics::PhysicsConstants::MAX_SAFE_SPEED, EPSILON);
    EXPECT_NEAR(v_tiny, physics::PhysicsConstants::MAX_SAFE_SPEED, EPSILON);
}

TEST_F(PhysicsMathTest, FrictionLimitedSpeed_ZeroMu) {
    // Zero friction should return zero speed
    double curvature = 1.0 / 100.0;
    double v = physics::calc_friction_limited_speed_from_curvature(curvature, 0.0, EARTH_G);
    EXPECT_NEAR(v, 0.0, EPSILON);
}

TEST_F(PhysicsMathTest, FrictionLimitedSpeed_TightCurve) {
    // Very tight curve (small radius = high curvature)
    double curvature = 1.0 / 10.0;  // radius = 10 m
    double mu = 0.6;  // wet asphalt
    
    double v_max = physics::calc_friction_limited_speed_from_curvature(curvature, mu, EARTH_G);
    
    // Expected: sqrt(0.6 * 9.81 / 0.1) ≈ 7.67 m/s ≈ 27.6 km/h
    double expected = std::sqrt(mu * EARTH_G / curvature);
    EXPECT_NEAR(v_max, expected, EPSILON);
    EXPECT_GT(v_max, 0.0);
    EXPECT_LT(v_max, 10.0);  // Should be well under 10 m/s for this tight wet curve
}

// ============================================================================
// Rollover Limited Speed Tests
// ============================================================================
TEST_F(PhysicsMathTest, RolloverLimitedSpeed_BasicCalculation) {
    double curvature = 1.0 / 100.0;  // radius = 100 m
    double track_width = 1.8;        // typical car
    double cog_height = 0.5;         // typical sedan
    
    double v_rollover = physics::calc_rollover_limited_speed_from_curvature(
        curvature, track_width, cog_height, EARTH_G
    );
    
    // a_lat_roll = 9.81 * (1.8 / (2 * 0.5)) = 9.81 * 1.8 = 17.658 m/s^2
    // v_rollover = sqrt(17.658 / 0.01) = sqrt(1765.8) ≈ 42.02 m/s
    double a_lat_roll = EARTH_G * (track_width / (2.0 * cog_height));
    double expected = std::sqrt(a_lat_roll / curvature);
    
    EXPECT_NEAR(v_rollover, expected, EPSILON);
    EXPECT_GT(v_rollover, 0.0);
}

TEST_F(PhysicsMathTest, RolloverLimitedSpeed_HighCOG) {
    // Higher center of gravity = lower rollover speed
    double curvature = 1.0 / 100.0;
    double track_width = 1.8;
    
    double v_low_cog = physics::calc_rollover_limited_speed_from_curvature(
        curvature, track_width, 0.4, EARTH_G  // Low COG (sports car)
    );
    double v_high_cog = physics::calc_rollover_limited_speed_from_curvature(
        curvature, track_width, 0.8, EARTH_G  // High COG (SUV)
    );
    
    EXPECT_GT(v_low_cog, v_high_cog);  // Low COG can go faster before rollover
}

TEST_F(PhysicsMathTest, RolloverLimitedSpeed_StraightLine) {
    double curvature = 0.0;
    double v = physics::calc_rollover_limited_speed_from_curvature(
        curvature, 1.8, 0.5, EARTH_G
    );
    EXPECT_NEAR(v, physics::PhysicsConstants::MAX_SAFE_SPEED, EPSILON);
}

// ============================================================================
// Combined Limited Speed Tests
// ============================================================================
TEST_F(PhysicsMathTest, CombinedLimitedSpeed_FrictionLimited) {
    // For typical car on ice, friction limit should be lower than rollover
    double curvature = 1.0 / 100.0;
    double mu_ice = 0.15;
    double track_width = 1.8;
    double cog_height = 0.5;
    
    double v_combined = physics::calc_combined_limited_speed(
        curvature, mu_ice, track_width, cog_height, EARTH_G, true
    );
    
    double v_friction = physics::calc_friction_limited_speed_from_curvature(curvature, mu_ice, EARTH_G);
    double v_rollover = physics::calc_rollover_limited_speed_from_curvature(curvature, track_width, cog_height, EARTH_G);
    
    // On ice, friction limit should be the constraint
    EXPECT_LT(v_friction, v_rollover);
    EXPECT_NEAR(v_combined, v_friction, EPSILON);
}

TEST_F(PhysicsMathTest, CombinedLimitedSpeed_RolloverDisabled) {
    double curvature = 1.0 / 100.0;
    double mu = 0.9;
    
    double v_friction_only = physics::calc_combined_limited_speed(
        curvature, mu, 1.8, 0.5, EARTH_G, false  // Rollover check disabled
    );
    
    double v_friction = physics::calc_friction_limited_speed_from_curvature(curvature, mu, EARTH_G);
    
    EXPECT_NEAR(v_friction_only, v_friction, EPSILON);
}

// ============================================================================
// Utility Function Tests
// ============================================================================
TEST_F(PhysicsMathTest, CurvatureToRadius) {
    EXPECT_NEAR(physics::curvature_to_radius(0.01), 100.0, EPSILON);
    EXPECT_NEAR(physics::curvature_to_radius(0.1), 10.0, EPSILON);
    EXPECT_NEAR(physics::curvature_to_radius(1.0), 1.0, EPSILON);
    EXPECT_TRUE(std::isinf(physics::curvature_to_radius(0.0)));
}

TEST_F(PhysicsMathTest, RadiusToCurvature) {
    EXPECT_NEAR(physics::radius_to_curvature(100.0), 0.01, EPSILON);
    EXPECT_NEAR(physics::radius_to_curvature(10.0), 0.1, EPSILON);
    EXPECT_NEAR(physics::radius_to_curvature(1.0), 1.0, EPSILON);
    EXPECT_NEAR(physics::radius_to_curvature(std::numeric_limits<double>::infinity()), 0.0, EPSILON);
}

TEST_F(PhysicsMathTest, SpeedConversions) {
    EXPECT_NEAR(physics::mps_to_kmph(1.0), 3.6, EPSILON);
    EXPECT_NEAR(physics::mps_to_kmph(10.0), 36.0, EPSILON);
    EXPECT_NEAR(physics::kmph_to_mps(3.6), 1.0, EPSILON);
    EXPECT_NEAR(physics::kmph_to_mps(100.0), 27.777778, 0.0001);
}

TEST_F(PhysicsMathTest, RollingResistanceForce) {
    double mass = 1500.0;  // kg
    double rolling_coeff = 0.015;
    
    double force = physics::calc_rolling_resistance_force(mass, rolling_coeff, EARTH_G);
    
    // Expected: 1500 * 9.81 * 0.015 = 220.725 N
    EXPECT_NEAR(force, 220.725, 0.01);
}

TEST_F(PhysicsMathTest, DragForce) {
    double velocity = 30.0;       // m/s
    double drag_coeff = 0.3;      // typical car
    double frontal_area = 2.2;    // m^2
    double air_density = 1.225;   // kg/m^3
    
    double force = physics::calc_drag_force(velocity, drag_coeff, frontal_area, air_density);
    
    // Expected: 0.5 * 1.225 * 0.3 * 2.2 * 30^2 = 363.825 N
    EXPECT_NEAR(force, 363.825, 0.01);
}

TEST_F(PhysicsMathTest, GradeResistanceForce) {
    double mass = 1500.0;  // kg
    double grade_angle = 0.1;  // radians (~5.7 degrees)
    
    double force = physics::calc_grade_resistance_force(mass, grade_angle, EARTH_G);
    
    // Expected: 1500 * 9.81 * sin(0.1) ≈ 1468.0 N
    double expected = mass * EARTH_G * std::sin(grade_angle);
    EXPECT_NEAR(force, expected, EPSILON);
}

} // namespace testing
} // namespace apex

// Main function for GoogleTest
int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}





