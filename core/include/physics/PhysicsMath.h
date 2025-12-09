#ifndef APEXVELOCITY_PHYSICS_MATH_H
#define APEXVELOCITY_PHYSICS_MATH_H

#include <cmath>
#include <limits>

namespace apex {
namespace physics {

/**
 * @brief Default constants for physics calculations.
 */
struct PhysicsConstants {
    static constexpr double EARTH_GRAVITY = 9.81;           ///< m/s^2
    static constexpr double MARS_GRAVITY = 3.71;            ///< m/s^2
    static constexpr double MIN_CURVATURE = 1.0e-9;         ///< 1/m - below this, treat as straight
    static constexpr double MAX_SAFE_SPEED = 83.33;         ///< m/s (~300 km/h)
    static constexpr double AIR_DENSITY_SEA_LEVEL = 1.225;  ///< kg/m^3
};

/**
 * @brief Calculate friction-limited maximum speed for a given curvature.
 * 
 * Uses the relationship: a_lat = v^2 * κ, where lateral acceleration
 * must not exceed μ * g for tire grip.
 * 
 * @param curvature Path curvature κ = 1/radius (1/m). If <= MIN_CURVATURE, treated as straight.
 * @param mu Coefficient of friction (dimensionless, typically 0.1 to 1.0).
 * @param gravity Gravitational acceleration (m/s^2), default is Earth's 9.81.
 * @return Maximum safe speed in m/s.
 * 
 * Formula: v_max = sqrt(μ * g / κ)
 * 
 * For very small curvature (straight line), returns MAX_SAFE_SPEED.
 */
double calc_friction_limited_speed_from_curvature(
    double curvature,
    double mu,
    double gravity = PhysicsConstants::EARTH_GRAVITY
);

/**
 * @brief Calculate rollover-limited maximum speed for a given curvature.
 * 
 * Uses a simplified rollover model based on track width and center of gravity height.
 * 
 * @param curvature Path curvature κ = 1/radius (1/m). If <= MIN_CURVATURE, treated as straight.
 * @param track_width_m Vehicle track width (m), typically 1.5 to 2.0 for cars.
 * @param cog_height_m Height of center of gravity above ground (m), typically 0.4 to 0.8 for cars.
 * @param gravity Gravitational acceleration (m/s^2), default is Earth's 9.81.
 * @return Maximum speed before rollover risk (m/s).
 * 
 * Formula: a_lat_roll = g * (track_width / (2 * cog_height))
 *          v_rollover = sqrt(a_lat_roll / κ)
 * 
 * For very small curvature (straight line), returns MAX_SAFE_SPEED.
 */
double calc_rollover_limited_speed_from_curvature(
    double curvature,
    double track_width_m,
    double cog_height_m,
    double gravity = PhysicsConstants::EARTH_GRAVITY
);

/**
 * @brief Calculate the combined safe speed considering both friction and rollover limits.
 * 
 * @param curvature Path curvature (1/m).
 * @param mu Coefficient of friction.
 * @param track_width_m Vehicle track width (m).
 * @param cog_height_m Center of gravity height (m).
 * @param gravity Gravitational acceleration (m/s^2).
 * @param enable_rollover_check If false, only friction limit is considered.
 * @return Minimum of friction-limited and rollover-limited speeds (m/s).
 */
double calc_combined_limited_speed(
    double curvature,
    double mu,
    double track_width_m,
    double cog_height_m,
    double gravity = PhysicsConstants::EARTH_GRAVITY,
    bool enable_rollover_check = true
);

/**
 * @brief Convert curvature to radius.
 * @param curvature Curvature in 1/m.
 * @return Radius in meters. Returns infinity for zero curvature.
 */
inline double curvature_to_radius(double curvature) {
    if (std::abs(curvature) < PhysicsConstants::MIN_CURVATURE) {
        return std::numeric_limits<double>::infinity();
    }
    return 1.0 / curvature;
}

/**
 * @brief Convert radius to curvature.
 * @param radius Radius in meters.
 * @return Curvature in 1/m. Returns 0 for infinite radius.
 */
inline double radius_to_curvature(double radius) {
    if (radius <= 0 || std::isinf(radius)) {
        return 0.0;
    }
    return 1.0 / radius;
}

/**
 * @brief Calculate rolling resistance force.
 * @param mass Vehicle mass (kg).
 * @param rolling_coeff Rolling resistance coefficient (dimensionless).
 * @param gravity Gravitational acceleration (m/s^2).
 * @return Rolling resistance force (N).
 */
inline double calc_rolling_resistance_force(
    double mass,
    double rolling_coeff,
    double gravity = PhysicsConstants::EARTH_GRAVITY
) {
    return mass * gravity * rolling_coeff;
}

/**
 * @brief Calculate aerodynamic drag force.
 * @param velocity Vehicle velocity (m/s).
 * @param drag_coeff Drag coefficient (dimensionless, typically 0.25-0.45 for cars).
 * @param frontal_area Frontal area (m^2, typically 2.0-2.5 for cars).
 * @param air_density Air density (kg/m^3).
 * @return Drag force (N).
 * 
 * Formula: F_drag = 0.5 * ρ * Cd * A * v^2
 */
inline double calc_drag_force(
    double velocity,
    double drag_coeff,
    double frontal_area,
    double air_density = PhysicsConstants::AIR_DENSITY_SEA_LEVEL
) {
    return 0.5 * air_density * drag_coeff * frontal_area * velocity * velocity;
}

/**
 * @brief Calculate grade/slope resistance force.
 * @param mass Vehicle mass (kg).
 * @param grade_angle Slope angle in radians (positive = uphill).
 * @param gravity Gravitational acceleration (m/s^2).
 * @return Grade resistance force (N), positive when going uphill.
 */
inline double calc_grade_resistance_force(
    double mass,
    double grade_angle,
    double gravity = PhysicsConstants::EARTH_GRAVITY
) {
    return mass * gravity * std::sin(grade_angle);
}

/**
 * @brief Convert speed from m/s to km/h.
 */
inline double mps_to_kmph(double mps) {
    return mps * 3.6;
}

/**
 * @brief Convert speed from km/h to m/s.
 */
inline double kmph_to_mps(double kmph) {
    return kmph / 3.6;
}

} // namespace physics
} // namespace apex

#endif // APEXVELOCITY_PHYSICS_MATH_H





