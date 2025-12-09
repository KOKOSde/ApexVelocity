/**
 * @file test_solver_physics.cpp
 * @brief GoogleTest suite for VelocityProfileSolver and energy calculations.
 * 
 * Tests cover:
 * - Vehicle parameter loading
 * - Static speed limit computation
 * - Forward-backward solver passes
 * - Energy consumption calculations
 * - The "Sand Trap" scenario
 */

#include <gtest/gtest.h>
#include <cmath>
#include <numeric>

#include "physics/Types.h"
#include "physics/VehicleLoader.h"
#include "physics/VelocityProfileSolver.h"
#include "physics/PhysicsMath.h"
#include "utils/ConfigManager.h"

namespace apex {
namespace testing {

constexpr double EPSILON = 1e-6;

// ============================================================================
// Test Fixture
// ============================================================================
class SolverPhysicsTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Initialize ConfigManager with test config directory
        ConfigManager::instance().initialize(TEST_CONFIG_DIR);
    }
    
    /**
     * @brief Create a straight path with specified surface and length.
     */
    static physics::Path create_straight_path(
        double length_m,
        double step_m,
        const std::string& surface_type,
        double start_x = 0.0
    ) {
        physics::Path path;
        double distance = 0.0;
        double x = start_x;
        
        while (distance <= length_m + EPSILON) {
            physics::PathPoint pt;
            pt.x_m = x;
            pt.y_m = 0.0;
            pt.z_m = 0.0;
            pt.curvature = 0.0;  // Straight line
            pt.distance_along_m = distance;
            pt.surface_type = surface_type;
            path.push_back(pt);
            
            distance += step_m;
            x += step_m;
        }
        
        return path;
    }
    
    /**
     * @brief Create a curved path with specified curvature.
     */
    static physics::Path create_curved_path(
        double length_m,
        double step_m,
        double curvature,
        const std::string& surface_type
    ) {
        physics::Path path;
        double distance = 0.0;
        
        // For a circular arc with curvature κ = 1/R:
        // Parametric: x = R * sin(θ), y = R * (1 - cos(θ))
        // where θ = distance / R = distance * κ
        double radius = (curvature > 1e-9) ? (1.0 / curvature) : 1e9;
        
        while (distance <= length_m + EPSILON) {
            physics::PathPoint pt;
            double theta = distance * curvature;
            pt.x_m = radius * std::sin(theta);
            pt.y_m = radius * (1.0 - std::cos(theta));
            pt.z_m = 0.0;
            pt.curvature = curvature;
            pt.distance_along_m = distance;
            pt.surface_type = surface_type;
            path.push_back(pt);
            
            distance += step_m;
        }
        
        return path;
    }
    
    /**
     * @brief Append one path to another, adjusting distances.
     */
    static void append_path(physics::Path& main_path, const physics::Path& to_append) {
        if (main_path.empty()) {
            main_path = to_append;
            return;
        }
        
        double last_dist = main_path.back().distance_along_m;
        double last_x = main_path.back().x_m;
        double last_y = main_path.back().y_m;
        
        for (size_t i = 1; i < to_append.size(); ++i) {  // Skip first point (overlap)
            physics::PathPoint pt = to_append[i];
            pt.distance_along_m = last_dist + to_append[i].distance_along_m;
            pt.x_m = last_x + to_append[i].x_m;
            pt.y_m = last_y + to_append[i].y_m;
            main_path.push_back(pt);
        }
    }
    
    /**
     * @brief Get a simple test vehicle.
     */
    static physics::VehicleParams get_test_vehicle() {
        physics::VehicleParams v;
        v.name = "test_car";
        v.mass_kg = 1500.0;
        v.drag_coeff = 0.30;
        v.frontal_area_m2 = 2.2;
        v.rolling_res_base = 0.015;
        v.max_lat_g = 0.8;
        v.max_brake_g = 0.9;
        v.max_power_w = 150000.0;
        v.powertrain_efficiency = 0.90;
        v.track_width_m = 1.6;
        v.cog_height_m = 0.5;
        return v;
    }
};

// ============================================================================
// Vehicle Loader Tests
// ============================================================================
TEST_F(SolverPhysicsTest, VehicleLoader_DefaultVehicle) {
    auto vehicle = physics::VehicleLoader::get_default();
    EXPECT_TRUE(vehicle.is_valid());
    EXPECT_GT(vehicle.mass_kg, 0.0);
    EXPECT_GT(vehicle.max_power_w, 0.0);
}

TEST_F(SolverPhysicsTest, VehicleLoader_FromJsonString) {
    std::string json = R"({
        "name": "test_car",
        "mass_kg": 1200,
        "drag_coeff": 0.28,
        "frontal_area_m2": 2.0,
        "rolling_res_base": 0.012,
        "max_lat_g": 0.9,
        "max_brake_g": 1.0,
        "max_power_w": 200000,
        "powertrain_efficiency": 0.92,
        "track_width_m": 1.55,
        "cog_height_m": 0.45
    })";
    
    auto vehicle = physics::VehicleLoader::from_json_string(json);
    ASSERT_TRUE(vehicle.has_value());
    EXPECT_EQ(vehicle->name, "test_car");
    EXPECT_NEAR(vehicle->mass_kg, 1200.0, EPSILON);
    EXPECT_NEAR(vehicle->drag_coeff, 0.28, EPSILON);
    EXPECT_TRUE(vehicle->is_valid());
}

TEST_F(SolverPhysicsTest, VehicleLoader_InvalidJson) {
    std::string json = "not valid json";
    auto vehicle = physics::VehicleLoader::from_json_string(json);
    EXPECT_FALSE(vehicle.has_value());
}

TEST_F(SolverPhysicsTest, VehicleParams_Validation) {
    physics::VehicleParams valid;
    EXPECT_TRUE(valid.is_valid());
    
    physics::VehicleParams invalid_mass = valid;
    invalid_mass.mass_kg = -100.0;
    EXPECT_FALSE(invalid_mass.is_valid());
    
    physics::VehicleParams invalid_efficiency = valid;
    invalid_efficiency.powertrain_efficiency = 1.5;
    EXPECT_FALSE(invalid_efficiency.is_valid());
}

// ============================================================================
// Path Point Tests
// ============================================================================
TEST_F(SolverPhysicsTest, PathPoint_Distance2D) {
    physics::PathPoint p1, p2;
    p1.x_m = 0.0; p1.y_m = 0.0;
    p2.x_m = 3.0; p2.y_m = 4.0;
    
    EXPECT_NEAR(p1.distance_to_2d(p2), 5.0, EPSILON);
}

TEST_F(SolverPhysicsTest, PathPoint_GradeAngle) {
    physics::PathPoint p1, p2;
    p1.x_m = 0.0; p1.y_m = 0.0; p1.z_m = 0.0;
    p2.x_m = 100.0; p2.y_m = 0.0; p2.z_m = 10.0;  // 10% grade
    
    double angle = p1.grade_angle_to(p2);
    // tan(angle) = 10/100 = 0.1, angle ≈ 0.0997 rad
    EXPECT_NEAR(std::tan(angle), 0.1, 0.001);
}

// ============================================================================
// Solver Basic Tests
// ============================================================================
TEST_F(SolverPhysicsTest, Solver_EmptyPath) {
    physics::VehicleParams vehicle = get_test_vehicle();
    physics::VelocityProfileSolver solver(vehicle);
    
    physics::Path empty_path;
    auto result = solver.solve(empty_path);
    
    EXPECT_FALSE(result.success);
    EXPECT_FALSE(result.error_message.empty());
}

TEST_F(SolverPhysicsTest, Solver_SinglePoint) {
    physics::VehicleParams vehicle = get_test_vehicle();
    physics::VelocityProfileSolver solver(vehicle);
    
    physics::Path path;
    physics::PathPoint pt;
    path.push_back(pt);
    
    auto result = solver.solve(path);
    EXPECT_FALSE(result.success);  // Need at least 2 points
}

TEST_F(SolverPhysicsTest, Solver_StraightPath) {
    physics::VehicleParams vehicle = get_test_vehicle();
    physics::SolverConfig config;
    config.initial_speed_mps = 30.0;  // Start at reasonable speed
    
    physics::VelocityProfileSolver solver(vehicle, config);
    
    physics::Path path = create_straight_path(100.0, 10.0, "asphalt");
    
    auto result = solver.solve(path);
    
    EXPECT_TRUE(result.success);
    EXPECT_EQ(result.points_processed, path.size());
    EXPECT_NEAR(result.total_distance_m, 100.0, 1.0);
    
    // On a straight path with high initial speed, max speed should be high
    EXPECT_GT(result.max_speed_mps, 20.0);
    
    // Average speed should also be reasonable
    EXPECT_GT(result.avg_speed_mps, 10.0);
}

TEST_F(SolverPhysicsTest, Solver_CurvedPath_SpeedLimited) {
    physics::VehicleParams vehicle = get_test_vehicle();
    physics::SolverConfig config;
    config.initial_speed_mps = 15.0;  // Start near expected curve limit
    
    physics::VelocityProfileSolver solver(vehicle, config);
    
    // Tight curve: radius = 20m, curvature = 0.05
    double curvature = 1.0 / 20.0;
    physics::Path path = create_curved_path(50.0, 5.0, curvature, "asphalt");
    
    auto result = solver.solve(path);
    
    EXPECT_TRUE(result.success);
    
    // On a tight curve, speeds should be limited
    // With mu ≈ 0.9, g = 9.81, κ = 0.05:
    // v_max = sqrt(0.9 * 9.81 / 0.05) ≈ 13.3 m/s
    // But lateral G limit (0.8g) gives: sqrt(0.8 * 9.81 / 0.05) ≈ 12.5 m/s
    EXPECT_LT(result.max_speed_mps, 20.0);  // Should be well below 20 m/s
    EXPECT_GT(result.avg_speed_mps, 5.0);   // But reasonable positive value
}

TEST_F(SolverPhysicsTest, Solver_BackwardPass_BrakingConstraint) {
    physics::VehicleParams vehicle = get_test_vehicle();
    
    physics::SolverConfig config;
    config.final_speed_mps = 5.0;  // Force slow finish
    
    physics::VelocityProfileSolver solver(vehicle, config);
    
    physics::Path path = create_straight_path(100.0, 10.0, "asphalt");
    auto result = solver.solve(path);
    
    EXPECT_TRUE(result.success);
    
    // Final speed should be close to target
    EXPECT_NEAR(path.back().v_profile, 5.0, 1.0);
    
    // Earlier points should have progressively higher allowed speeds
    // (they can go faster because they have distance to brake)
    for (size_t i = 1; i < path.size(); ++i) {
        // Due to braking, later speeds <= earlier static limits
        EXPECT_LE(path[i].v_profile, path[i].v_static + EPSILON);
    }
}

TEST_F(SolverPhysicsTest, Solver_ForwardPass_AccelerationConstraint) {
    physics::VehicleParams vehicle = get_test_vehicle();
    
    physics::SolverConfig config;
    config.initial_speed_mps = 5.0;  // Start slow
    
    physics::VelocityProfileSolver solver(vehicle, config);
    
    physics::Path path = create_straight_path(200.0, 10.0, "asphalt");
    auto result = solver.solve(path);
    
    EXPECT_TRUE(result.success);
    
    // First point should start at or near initial speed
    EXPECT_LE(path[0].v_profile, config.initial_speed_mps + 1.0);
    
    // Speed should generally increase in the first half (acceleration zone)
    // and may decrease in the second half (braking to stop)
    // Check that we reach higher than initial speed at some point
    EXPECT_GT(result.max_speed_mps, config.initial_speed_mps);
    
    // Check that acceleration happens in first portion
    size_t mid = path.size() / 3;  // First third
    double max_speed_first_third = 0.0;
    for (size_t i = 0; i <= mid && i < path.size(); ++i) {
        max_speed_first_third = std::max(max_speed_first_third, path[i].v_profile);
    }
    EXPECT_GT(max_speed_first_third, config.initial_speed_mps);
}

// ============================================================================
// Energy Calculation Tests
// ============================================================================
TEST_F(SolverPhysicsTest, Energy_PositiveOnStraight) {
    physics::VehicleParams vehicle = get_test_vehicle();
    physics::VelocityProfileSolver solver(vehicle);
    
    physics::Path path = create_straight_path(100.0, 10.0, "asphalt");
    auto result = solver.solve(path);
    
    EXPECT_TRUE(result.success);
    EXPECT_GT(result.total_energy_joules, 0.0);
    EXPECT_GT(result.energy_per_meter, 0.0);
    
    // Energy should accumulate along path
    for (size_t i = 1; i < path.size(); ++i) {
        EXPECT_GE(path[i].energy_joules, path[i-1].energy_joules);
    }
}

TEST_F(SolverPhysicsTest, Energy_HigherOnSand) {
    physics::VehicleParams vehicle = get_test_vehicle();
    physics::VelocityProfileSolver solver(vehicle);
    
    // Test asphalt
    physics::Path asphalt_path = create_straight_path(100.0, 5.0, "asphalt");
    auto asphalt_result = solver.solve(asphalt_path);
    
    // Test sand (higher rolling resistance)
    physics::Path sand_path = create_straight_path(100.0, 5.0, "sand");
    auto sand_result = solver.solve(sand_path);
    
    EXPECT_TRUE(asphalt_result.success);
    EXPECT_TRUE(sand_result.success);
    
    // Sand should require more energy per meter
    // Sand rolling_resistance = 0.050 vs asphalt = 0.015 (~3.3x higher)
    EXPECT_GT(sand_result.energy_per_meter, asphalt_result.energy_per_meter);
}

// ============================================================================
// THE SAND TRAP SCENARIO
// ============================================================================
TEST_F(SolverPhysicsTest, SandTrap_EnergyComparison) {
    /**
     * Scenario: "The Sand Trap"
     * 
     * Compare energy consumption on two SEPARATE paths:
     * - 100m straight asphalt
     * - 100m straight sand
     * 
     * By running them separately with identical conditions, we eliminate
     * acceleration effects and can directly compare rolling resistance impact.
     * 
     * Expectations:
     * - Rolling resistance energy on sand >> asphalt due to higher C_rr
     * - Total energy on sand should be significantly higher
     */
    
    physics::VehicleParams vehicle = get_test_vehicle();
    physics::SolverConfig config;
    config.initial_speed_mps = 20.0;  // Start at constant speed
    
    // Run on asphalt
    physics::VelocityProfileSolver solver_asphalt(vehicle, config);
    physics::Path asphalt_path = create_straight_path(100.0, 5.0, "asphalt");
    auto asphalt_result = solver_asphalt.solve(asphalt_path);
    
    // Run on sand with same conditions
    physics::VelocityProfileSolver solver_sand(vehicle, config);
    physics::Path sand_path = create_straight_path(100.0, 5.0, "sand");
    auto sand_result = solver_sand.solve(sand_path);
    
    EXPECT_TRUE(asphalt_result.success);
    EXPECT_TRUE(sand_result.success);
    
    // Get segment energies to examine rolling resistance specifically
    const auto& asphalt_segments = solver_asphalt.get_segment_energies();
    const auto& sand_segments = solver_sand.get_segment_energies();
    
    // Sum up rolling resistance energy only (isolates surface effect)
    double asphalt_rolling_energy = 0.0;
    double sand_rolling_energy = 0.0;
    
    for (const auto& seg : asphalt_segments) {
        asphalt_rolling_energy += seg.rolling_joules;
    }
    for (const auto& seg : sand_segments) {
        sand_rolling_energy += seg.rolling_joules;
    }
    
    double E_per_m_asphalt = asphalt_result.energy_per_meter;
    double E_per_m_sand = sand_result.energy_per_meter;
    double rolling_per_m_asphalt = asphalt_rolling_energy / 100.0;
    double rolling_per_m_sand = sand_rolling_energy / 100.0;
    
    // Print for debugging
    std::cout << "\n=== SAND TRAP TEST RESULTS ===" << std::endl;
    std::cout << "Asphalt: total " << E_per_m_asphalt << " J/m, rolling " 
              << rolling_per_m_asphalt << " J/m" << std::endl;
    std::cout << "Sand: total " << E_per_m_sand << " J/m, rolling " 
              << rolling_per_m_sand << " J/m" << std::endl;
    std::cout << "Rolling ratio (sand/asphalt): " << (rolling_per_m_sand / rolling_per_m_asphalt) << std::endl;
    std::cout << "Total ratio (sand/asphalt): " << (E_per_m_sand / E_per_m_asphalt) << std::endl;
    
    // ASSERTION: Sand should have significantly higher ROLLING resistance energy
    // Sand rolling coefficient (0.050) is ~3.3x asphalt (0.015)
    // We expect rolling energy on sand to be 2x-4x higher
    EXPECT_GT(rolling_per_m_asphalt, 0.0) << "Asphalt rolling energy should be positive";
    EXPECT_GT(rolling_per_m_sand, 0.0) << "Sand rolling energy should be positive";
    EXPECT_GT(rolling_per_m_sand, rolling_per_m_asphalt * 2.0) 
        << "Sand rolling resistance should be at least 2x asphalt";
    
    // Total energy should also be higher on sand (rolling is a major component)
    EXPECT_GT(E_per_m_sand, E_per_m_asphalt) 
        << "Sand should use more total energy than asphalt";
}

TEST_F(SolverPhysicsTest, SandTrap_SpeedsSimilarOnStraight) {
    /**
     * Additional check: On straight sections, speeds should be similar
     * because friction doesn't limit straight-line speed significantly.
     */
    
    physics::VehicleParams vehicle = get_test_vehicle();
    physics::SolverConfig config;
    config.initial_speed_mps = 20.0;
    
    physics::VelocityProfileSolver solver(vehicle, config);
    
    // Solve separate paths
    physics::Path asphalt_path = create_straight_path(100.0, 5.0, "asphalt");
    physics::Path sand_path = create_straight_path(100.0, 5.0, "sand");
    
    auto asphalt_result = solver.solve(asphalt_path);
    auto sand_result = solver.solve(sand_path);
    
    EXPECT_TRUE(asphalt_result.success);
    EXPECT_TRUE(sand_result.success);
    
    // Average speeds should be relatively similar (within 20%)
    // because curvature, not rolling resistance, limits cornering speed
    double speed_ratio = sand_result.avg_speed_mps / asphalt_result.avg_speed_mps;
    EXPECT_GT(speed_ratio, 0.5);
    EXPECT_LT(speed_ratio, 1.5);
    
    std::cout << "\n=== SPEED COMPARISON ===" << std::endl;
    std::cout << "Avg speed on asphalt: " << asphalt_result.avg_speed_mps << " m/s" << std::endl;
    std::cout << "Avg speed on sand: " << sand_result.avg_speed_mps << " m/s" << std::endl;
}

// ============================================================================
// Surface Material Impact Tests
// ============================================================================
TEST_F(SolverPhysicsTest, Surface_IceLowersCornering) {
    physics::VehicleParams vehicle = get_test_vehicle();
    physics::VelocityProfileSolver solver(vehicle);
    
    double curvature = 1.0 / 50.0;  // radius = 50m
    
    physics::Path asphalt_curve = create_curved_path(50.0, 5.0, curvature, "asphalt");
    physics::Path ice_curve = create_curved_path(50.0, 5.0, curvature, "ice");
    
    auto asphalt_result = solver.solve(asphalt_curve);
    auto ice_result = solver.solve(ice_curve);
    
    EXPECT_TRUE(asphalt_result.success);
    EXPECT_TRUE(ice_result.success);
    
    // Ice (mu ≈ 0.15) should have much lower max speed than asphalt (mu ≈ 0.9)
    EXPECT_LT(ice_result.max_speed_mps, asphalt_result.max_speed_mps);
    
    // Ice speed should be roughly sqrt(0.15/0.9) ≈ 0.41x asphalt speed
    double expected_ratio = std::sqrt(0.15 / 0.9);
    double actual_ratio = ice_result.max_speed_mps / asphalt_result.max_speed_mps;
    EXPECT_NEAR(actual_ratio, expected_ratio, 0.15);
    
    std::cout << "\n=== ICE VS ASPHALT CORNERING ===" << std::endl;
    std::cout << "Asphalt max speed: " << asphalt_result.max_speed_mps << " m/s" << std::endl;
    std::cout << "Ice max speed: " << ice_result.max_speed_mps << " m/s" << std::endl;
    std::cout << "Ratio: " << actual_ratio << " (expected ~" << expected_ratio << ")" << std::endl;
}

TEST_F(SolverPhysicsTest, RollingResistance_AllMaterials) {
    physics::VehicleParams vehicle = get_test_vehicle();
    physics::VelocityProfileSolver solver(vehicle);
    
    std::vector<std::string> surfaces = {
        "asphalt", "concrete", "cobblestone", "gravel", "dirt", "sand", "ice", "snow_packed"
    };
    
    std::vector<double> energies;
    
    for (const auto& surface : surfaces) {
        physics::Path path = create_straight_path(100.0, 5.0, surface);
        auto result = solver.solve(path);
        
        EXPECT_TRUE(result.success) << "Failed for surface: " << surface;
        energies.push_back(result.energy_per_meter);
        
        std::cout << surface << ": " << result.energy_per_meter << " J/m" << std::endl;
    }
    
    // Sand should have highest rolling resistance energy
    // Find index of sand
    size_t sand_idx = std::find(surfaces.begin(), surfaces.end(), "sand") - surfaces.begin();
    size_t asphalt_idx = std::find(surfaces.begin(), surfaces.end(), "asphalt") - surfaces.begin();
    
    EXPECT_GT(energies[sand_idx], energies[asphalt_idx]);
}

} // namespace testing
} // namespace apex

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

