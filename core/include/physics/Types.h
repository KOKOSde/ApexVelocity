#ifndef APEXVELOCITY_PHYSICS_TYPES_H
#define APEXVELOCITY_PHYSICS_TYPES_H

#include <string>
#include <vector>
#include <cmath>

namespace apex {
namespace physics {

/**
 * @brief Vehicle physical parameters for dynamics calculations.
 * 
 * These parameters define the kinematic and dynamic properties of a vehicle
 * used for velocity profiling and energy consumption calculations.
 */
struct VehicleParams {
    double mass_kg = 1500.0;              ///< Vehicle mass in kg
    double drag_coeff = 0.30;             ///< Aerodynamic drag coefficient Cd
    double frontal_area_m2 = 2.2;         ///< Frontal area in m^2
    double rolling_res_base = 0.015;      ///< Base rolling resistance coeff (asphalt reference)
    double max_lat_g = 0.8;               ///< Max lateral acceleration in g (comfort/tire limit)
    double max_brake_g = 0.9;             ///< Max braking deceleration in g
    double max_power_w = 150000.0;        ///< Max power in Watts (150 kW default)
    double powertrain_efficiency = 0.90;  ///< Powertrain efficiency 0-1
    double track_width_m = 1.6;           ///< Track width (wheel to wheel) in m
    double cog_height_m = 0.5;            ///< Center of gravity height in m
    
    std::string name = "default";         ///< Vehicle name/identifier
    
    /**
     * @brief Validate that all parameters are within reasonable bounds.
     * @return true if valid, false otherwise.
     */
    bool is_valid() const {
        return mass_kg > 0.0 &&
               drag_coeff >= 0.0 && drag_coeff <= 2.0 &&
               frontal_area_m2 > 0.0 &&
               rolling_res_base >= 0.0 && rolling_res_base <= 0.5 &&
               max_lat_g > 0.0 && max_lat_g <= 3.0 &&
               max_brake_g > 0.0 && max_brake_g <= 3.0 &&
               max_power_w > 0.0 &&
               powertrain_efficiency > 0.0 && powertrain_efficiency <= 1.0 &&
               track_width_m > 0.0 &&
               cog_height_m > 0.0;
    }
};

/**
 * @brief A point along a path with geometry, physics, and solver results.
 * 
 * PathPoint holds both input geometry data and computed velocity/energy results.
 */
struct PathPoint {
    // Geometry
    double x_m = 0.0;                     ///< X coordinate in meters
    double y_m = 0.0;                     ///< Y coordinate in meters
    double z_m = 0.0;                     ///< Z coordinate (elevation) in meters
    double curvature = 0.0;               ///< Path curvature κ = 1/radius (1/m)
    double distance_along_m = 0.0;        ///< Cumulative distance along path in meters
    
    // Surface
    std::string surface_type = "asphalt"; ///< Surface material type
    
    // Solver results
    double v_static = 0.0;                ///< Static speed limit from curvature/friction (m/s)
    double v_profile = 0.0;               ///< Final speed after solver (m/s)
    double energy_joules = 0.0;           ///< Cumulative energy to reach this point (J)
    
    // Optional: heading/tangent angle (radians)
    double heading_rad = 0.0;             ///< Heading angle in radians
    
    /**
     * @brief Calculate distance to another point (2D).
     */
    double distance_to_2d(const PathPoint& other) const {
        double dx = other.x_m - x_m;
        double dy = other.y_m - y_m;
        return std::sqrt(dx * dx + dy * dy);
    }
    
    /**
     * @brief Calculate distance to another point (3D).
     */
    double distance_to_3d(const PathPoint& other) const {
        double dx = other.x_m - x_m;
        double dy = other.y_m - y_m;
        double dz = other.z_m - z_m;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    
    /**
     * @brief Calculate grade angle to next point (radians).
     * @param next The next point along the path.
     * @return Grade angle in radians (positive = uphill).
     */
    double grade_angle_to(const PathPoint& next) const {
        double horizontal_dist = distance_to_2d(next);
        if (horizontal_dist < 1e-9) return 0.0;
        double dz = next.z_m - z_m;
        return std::atan2(dz, horizontal_dist);
    }
};

/**
 * @brief A path is a sequence of PathPoints.
 */
using Path = std::vector<PathPoint>;

/**
 * @brief Solver configuration options.
 */
struct SolverConfig {
    std::string condition = "dry";        ///< Surface condition ("dry" or "wet")
    bool enable_rollover_checks = true;   ///< Enable rollover speed limiting
    bool enable_power_limit = true;       ///< Enable power-limited acceleration
    double min_speed_mps = 1.0;           ///< Minimum speed for calculations (avoid div by zero)
    double initial_speed_mps = 0.0;       ///< Initial speed at path start
    double final_speed_mps = 0.0;         ///< Target final speed at path end
};

/**
 * @brief Energy breakdown for a path segment.
 */
struct SegmentEnergy {
    double aero_joules = 0.0;             ///< Aerodynamic drag energy
    double rolling_joules = 0.0;          ///< Rolling resistance energy
    double grade_joules = 0.0;            ///< Grade/elevation energy
    double kinetic_joules = 0.0;          ///< Kinetic energy change
    double total_joules = 0.0;            ///< Total energy for segment
    double distance_m = 0.0;              ///< Segment distance
    
    double energy_per_meter() const {
        return (distance_m > 0) ? total_joules / distance_m : 0.0;
    }
};

} // namespace physics
} // namespace apex

#endif // APEXVELOCITY_PHYSICS_TYPES_H





