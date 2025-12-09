#include "physics/VelocityProfileSolver.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace apex {
namespace physics {

VelocityProfileSolver::VelocityProfileSolver(const VehicleParams& vehicle, const SolverConfig& config)
    : vehicle_(vehicle)
    , config_(config)
    , progress_callback_(nullptr) {
}

SolverResult VelocityProfileSolver::solve(Path& path) {
    return solve(path, ConfigManager::instance());
}

SolverResult VelocityProfileSolver::solve(Path& path, const ConfigManager& config_mgr) {
    SolverResult result;
    
    // Validate input
    if (path.size() < 2) {
        result.success = false;
        result.error_message = "Path must have at least 2 points";
        return result;
    }
    
    if (!vehicle_.is_valid()) {
        result.success = false;
        result.error_message = "Invalid vehicle parameters";
        return result;
    }
    
    // Get gravity from config
    double gravity = config_mgr.get_sim_param_or<double>("gravity", PhysicsConstants::EARTH_GRAVITY);
    
    // Pass 0: Static limits
    compute_static_limits(path, config_mgr);
    if (progress_callback_) progress_callback_(0, 1.0);
    
    // Pass 1: Backward pass (braking)
    backward_pass(path, gravity);
    if (progress_callback_) progress_callback_(1, 1.0);
    
    // Pass 2: Forward pass (acceleration)
    forward_pass(path, gravity);
    if (progress_callback_) progress_callback_(2, 1.0);
    
    // Compute energy
    compute_energy(path, config_mgr);
    if (progress_callback_) progress_callback_(3, 1.0);
    
    // Compute statistics
    result.success = true;
    result.points_processed = path.size();
    result.total_distance_m = path.back().distance_along_m - path.front().distance_along_m;
    
    if (!path.empty()) {
        result.total_energy_joules = path.back().energy_joules;
        
        double sum_speed = 0.0;
        result.max_speed_mps = 0.0;
        result.min_speed_mps = std::numeric_limits<double>::max();
        
        for (const auto& pt : path) {
            sum_speed += pt.v_profile;
            result.max_speed_mps = std::max(result.max_speed_mps, pt.v_profile);
            result.min_speed_mps = std::min(result.min_speed_mps, pt.v_profile);
        }
        
        result.avg_speed_mps = sum_speed / path.size();
        
        if (result.total_distance_m > 0) {
            result.energy_per_meter = result.total_energy_joules / result.total_distance_m;
        }
    }
    
    return result;
}

void VelocityProfileSolver::compute_static_limits(Path& path, const ConfigManager& config_mgr) {
    double gravity = config_mgr.get_sim_param_or<double>("gravity", PhysicsConstants::EARTH_GRAVITY);
    bool enable_rollover = config_mgr.get_sim_param_or<bool>("enable_rollover_checks", true);
    
    // Override with solver config if specified
    if (!config_.enable_rollover_checks) {
        enable_rollover = false;
    }
    
    for (auto& pt : path) {
        // If v_static has already been populated (e.g. by a higher-level
        // caller such as the Python bindings with a friction callback),
        // respect it as the starting point. Otherwise, compute the
        // friction-limited speed from the configured materials.
        double v_static = pt.v_static;
        if (v_static <= 0.0) {
            // Get effective friction coefficient for this surface
            double mu_effective = config_mgr.get_effective_mu(pt.surface_type, config_.condition);
            
            // Friction-limited speed
            double v_friction = calc_friction_limited_speed_from_curvature(
                pt.curvature, mu_effective, gravity
            );
            
            // Start with friction limit
            v_static = v_friction;
        }
        
        // Apply rollover limit if enabled
        if (enable_rollover && pt.curvature > PhysicsConstants::MIN_CURVATURE) {
            double v_rollover = calc_rollover_limited_speed_from_curvature(
                pt.curvature, vehicle_.track_width_m, vehicle_.cog_height_m, gravity
            );
            v_static = std::min(v_static, v_rollover);
        }
        
        // Apply lateral G limit from vehicle
        if (pt.curvature > PhysicsConstants::MIN_CURVATURE) {
            double a_lat_max = vehicle_.max_lat_g * gravity;
            double v_lat_g = std::sqrt(a_lat_max / pt.curvature);
            v_static = std::min(v_static, v_lat_g);
        }
        
        // Apply absolute speed cap
        v_static = std::min(v_static, PhysicsConstants::MAX_SAFE_SPEED);
        
        pt.v_static = v_static;
        pt.v_profile = v_static;  // Initialize profile with static limit
    }
}

void VelocityProfileSolver::backward_pass(Path& path, double gravity) {
    if (path.size() < 2) return;
    
    // Maximum braking deceleration
    double a_brake_max = vehicle_.max_brake_g * gravity;
    
    // Set final speed constraint only if explicitly specified (> 0)
    if (config_.final_speed_mps > 0.0) {
        path.back().v_profile = std::min(path.back().v_static, config_.final_speed_mps);
    }
    // Otherwise, v_profile already has v_static from compute_static_limits
    
    // Traverse backward: i from (n-2) down to 0
    for (size_t i = path.size() - 1; i > 0; --i) {
        size_t prev_idx = i - 1;
        
        // Distance between points
        double d = path[i].distance_along_m - path[prev_idx].distance_along_m;
        if (d <= 0) continue;  // Skip invalid distances
        
        // Speed at next point (already computed)
        double v_next = path[i].v_profile;
        
        // Maximum speed at current point to be able to brake to v_next
        // v_i^2 = v_{i+1}^2 + 2 * a_brake * d
        // => v_i = sqrt(v_{i+1}^2 + 2 * a_brake * d)
        double v_brake_limited = std::sqrt(v_next * v_next + 2.0 * a_brake_max * d);
        
        // Take minimum of static limit and brake-limited speed
        path[prev_idx].v_profile = std::min(path[prev_idx].v_static, v_brake_limited);
    }
}

void VelocityProfileSolver::forward_pass(Path& path, double gravity) {
    if (path.size() < 2) return;
    
    // Set initial speed only if explicitly specified (> 0)
    if (config_.initial_speed_mps > 0.0) {
        path[0].v_profile = std::min(path[0].v_profile, config_.initial_speed_mps);
    }
    // Otherwise, v_profile already has the value from backward pass
    
    // Ensure minimum speed at start
    path[0].v_profile = std::max(path[0].v_profile, config_.min_speed_mps);
    
    // Traverse forward
    for (size_t i = 1; i < path.size(); ++i) {
        size_t prev_idx = i - 1;
        
        // Distance between points
        double d = path[i].distance_along_m - path[prev_idx].distance_along_m;
        if (d <= 0) continue;
        
        double v_prev = path[prev_idx].v_profile;
        
        // Calculate acceleration limits
        // Tire-limited acceleration (longitudinal)
        double a_tire = vehicle_.max_lat_g * gravity;  // Using lateral G as proxy
        
        // Power-limited acceleration: a = P / (m * v)
        double a_power = std::numeric_limits<double>::max();
        if (config_.enable_power_limit && v_prev > config_.min_speed_mps) {
            a_power = (vehicle_.max_power_w * vehicle_.powertrain_efficiency) / 
                      (vehicle_.mass_kg * v_prev);
        }
        
        // Use minimum of tire and power limits
        double a_accel_max = std::min(a_tire, a_power);
        
        // Maximum speed at current point given acceleration from previous
        // v_i = sqrt(v_{i-1}^2 + 2 * a_accel * d)
        double v_accel_limited = std::sqrt(v_prev * v_prev + 2.0 * a_accel_max * d);
        
        // Take minimum of backward pass result and acceleration-limited speed
        path[i].v_profile = std::min(path[i].v_profile, v_accel_limited);
        
        // Ensure minimum speed
        path[i].v_profile = std::max(path[i].v_profile, config_.min_speed_mps);
    }
}

void VelocityProfileSolver::compute_energy(Path& path, const ConfigManager& config_mgr) {
    segment_energies_.clear();
    
    if (path.size() < 2) return;
    
    double air_density = config_mgr.get_sim_param_or<double>("air_density", 
                                                             PhysicsConstants::AIR_DENSITY_SEA_LEVEL);
    
    double cumulative_energy = 0.0;
    path[0].energy_joules = 0.0;
    
    for (size_t i = 1; i < path.size(); ++i) {
        SegmentEnergy seg = compute_segment_energy(path[i-1], path[i], config_mgr, air_density);
        segment_energies_.push_back(seg);
        
        cumulative_energy += seg.total_joules;
        path[i].energy_joules = cumulative_energy;
    }
}

SegmentEnergy VelocityProfileSolver::compute_segment_energy(
    const PathPoint& p1, 
    const PathPoint& p2,
    const ConfigManager& config_mgr,
    double air_density
) {
    SegmentEnergy seg;
    
    // Segment distance
    seg.distance_m = p2.distance_along_m - p1.distance_along_m;
    if (seg.distance_m <= 0) return seg;
    
    // Average speed for the segment
    double v_avg = (p1.v_profile + p2.v_profile) / 2.0;
    v_avg = std::max(v_avg, config_.min_speed_mps);
    
    // Grade angle
    double theta = p1.grade_angle_to(p2);
    
    // Get gravity
    double gravity = config_mgr.get_sim_param_or<double>("gravity", PhysicsConstants::EARTH_GRAVITY);
    
    // Effective rolling resistance (scaled by surface type)
    double C_rr_eff = get_effective_rolling_resistance(p2.surface_type, config_mgr);
    
    // Forces
    // Aerodynamic drag: F_aero = 0.5 * rho * Cd * A * v^2
    double F_aero = calc_drag_force(v_avg, vehicle_.drag_coeff, vehicle_.frontal_area_m2, air_density);
    
    // Rolling resistance: F_roll = C_rr * m * g * cos(theta)
    double F_roll = C_rr_eff * vehicle_.mass_kg * gravity * std::cos(theta);
    
    // Grade resistance: F_grade = m * g * sin(theta)
    double F_grade = vehicle_.mass_kg * gravity * std::sin(theta);
    
    // Kinetic energy change
    double delta_KE = 0.5 * vehicle_.mass_kg * (p2.v_profile * p2.v_profile - p1.v_profile * p1.v_profile);
    
    // Total force
    double F_total = F_aero + F_roll + F_grade;
    
    // Energy = Force * distance + kinetic energy change
    // For v1, we don't account for regeneration (all positive)
    seg.aero_joules = F_aero * seg.distance_m;
    seg.rolling_joules = F_roll * seg.distance_m;
    seg.grade_joules = F_grade * seg.distance_m;
    seg.kinetic_joules = std::max(0.0, delta_KE);  // Only positive (acceleration)
    
    // Total energy (at the wheel)
    double mechanical_energy = F_total * seg.distance_m + std::max(0.0, delta_KE);
    
    // Account for powertrain efficiency (input energy > mechanical energy)
    if (mechanical_energy > 0 && vehicle_.powertrain_efficiency > 0) {
        seg.total_joules = mechanical_energy / vehicle_.powertrain_efficiency;
    } else {
        // Coasting or regen - for v1, just set to 0 (no regen credit)
        seg.total_joules = std::max(0.0, mechanical_energy);
    }
    
    return seg;
}

double VelocityProfileSolver::get_effective_rolling_resistance(
    const std::string& surface_type,
    const ConfigManager& config_mgr
) const {
    // Get material rolling resistance
    const MaterialProps& mat = config_mgr.get_material(surface_type);
    
    // Scale vehicle's base rolling resistance by surface ratio
    // C_rr_eff = vehicle.rolling_res_base * (material.rolling_resistance / asphalt_ref)
    double surface_ratio = mat.rolling_resistance_coeff / ASPHALT_ROLLING_REF;
    
    return vehicle_.rolling_res_base * surface_ratio;
}

} // namespace physics
} // namespace apex

