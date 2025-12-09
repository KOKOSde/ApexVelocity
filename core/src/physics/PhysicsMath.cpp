#include "physics/PhysicsMath.h"

#include <algorithm>
#include <cmath>

namespace apex {
namespace physics {

double calc_friction_limited_speed_from_curvature(
    double curvature,
    double mu,
    double gravity
) {
    // For straight lines or very small curvature, return max safe speed
    if (curvature <= PhysicsConstants::MIN_CURVATURE) {
        return PhysicsConstants::MAX_SAFE_SPEED;
    }
    
    // Ensure positive friction coefficient
    if (mu <= 0.0) {
        return 0.0;
    }
    
    // Ensure positive gravity
    if (gravity <= 0.0) {
        return 0.0;
    }
    
    // Physics: a_lat = v^2 * κ, and a_lat_max = μ * g
    // Therefore: v_max = sqrt(μ * g / κ)
    double v_max = std::sqrt(mu * gravity / curvature);
    
    // Cap at maximum safe speed
    return std::min(v_max, PhysicsConstants::MAX_SAFE_SPEED);
}

double calc_rollover_limited_speed_from_curvature(
    double curvature,
    double track_width_m,
    double cog_height_m,
    double gravity
) {
    // For straight lines or very small curvature, no rollover constraint
    if (curvature <= PhysicsConstants::MIN_CURVATURE) {
        return PhysicsConstants::MAX_SAFE_SPEED;
    }
    
    // Validate inputs
    if (track_width_m <= 0.0 || cog_height_m <= 0.0 || gravity <= 0.0) {
        return 0.0;
    }
    
    // Rollover lateral acceleration limit:
    // a_lat_roll = g * (track_width / (2 * cog_height))
    // 
    // This is derived from the static rollover threshold where the
    // lateral force at the CoG would cause the inside wheels to lift.
    double a_lat_rollover = gravity * (track_width_m / (2.0 * cog_height_m));
    
    // From a_lat = v^2 * κ:
    // v_rollover = sqrt(a_lat_roll / κ)
    double v_rollover = std::sqrt(a_lat_rollover / curvature);
    
    // Cap at maximum safe speed
    return std::min(v_rollover, PhysicsConstants::MAX_SAFE_SPEED);
}

double calc_combined_limited_speed(
    double curvature,
    double mu,
    double track_width_m,
    double cog_height_m,
    double gravity,
    bool enable_rollover_check
) {
    double v_friction = calc_friction_limited_speed_from_curvature(curvature, mu, gravity);
    
    if (!enable_rollover_check) {
        return v_friction;
    }
    
    double v_rollover = calc_rollover_limited_speed_from_curvature(
        curvature, track_width_m, cog_height_m, gravity
    );
    
    // The actual limit is the more restrictive of the two
    return std::min(v_friction, v_rollover);
}

} // namespace physics
} // namespace apex





