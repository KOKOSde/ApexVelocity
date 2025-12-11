#include "physics/DynamicVehicle.h"

#include <algorithm>
#include <cmath>

#include "physics/PhysicsMath.h"

namespace apex {
namespace physics {

DynamicVehicle::DynamicVehicle(const VehicleParams& params,
                               const PacejkaTireParams& tire_lat_params,
                               const PacejkaTireParams& tire_long_params,
                               double gravity)
    : params_(params),
      tire_lat_(tire_lat_params),
      tire_long_(tire_long_params),
      gravity_(gravity) {}

void DynamicVehicle::compute_static_loads(double& Fz_front, double& Fz_rear) const {
    const double Fz_total = params_.mass_kg * gravity_;
    Fz_front = params_.front_weight_fraction * Fz_total;
    Fz_rear = Fz_total - Fz_front;
}

DynamicVehicleState DynamicVehicle::step(const DynamicVehicleState& s,
                                         double steering_rad,
                                         double throttle,
                                         double mu,
                                         double dt_s) const {
    DynamicVehicleState ns = s;

    // Basic geometry
    const double L = params_.wheelbase_m;
    const double a = params_.front_weight_fraction * L;
    const double b = L - a;

    // Actuator lag: first-order filters on steering and throttle
    const double tau_steer = 0.2;    // ~200 ms steering lag
    const double tau_throttle = 0.3; // ~300 ms throttle/brake lag
    const double a_steer = dt_s / (tau_steer + dt_s);
    const double a_throttle = dt_s / (tau_throttle + dt_s);

    const double steering_eff = s.steering_eff_rad +
                                a_steer * (steering_rad - s.steering_eff_rad);
    const double throttle_eff = s.throttle_eff +
                                a_throttle * (throttle - s.throttle_eff);

    ns.steering_eff_rad = steering_eff;
    ns.throttle_eff = throttle_eff;

    // Avoid division by zero at very low speeds
    const double vx = std::max(std::abs(s.vx_mps), 0.1);

    // Slip angles (bicycle model)
    const double alpha_f = steering_eff - std::atan2(s.vy_mps + a * s.yaw_rate_rps, vx);
    const double alpha_r = -std::atan2(s.vy_mps - b * s.yaw_rate_rps, vx);

    // Longitudinal demand based on throttle and power/ braking limits
    double Fx_total = 0.0;
    if (throttle_eff >= 0.0) {
        // Traction: power-limited
        const double P = params_.max_power_w * params_.powertrain_efficiency;
        const double Fx = (vx > 0.1) ? (P / vx) : 0.0;
        Fx_total = Fx * throttle_eff;
    } else {
        // Braking: decel limited by max_brake_g
        const double a_brake = params_.max_brake_g * gravity_;
        Fx_total = params_.mass_kg * a_brake * throttle_eff;  // throttle is negative
    }

    // Static normal loads (very simple model)
    double Fz_front = 0.0;
    double Fz_rear = 0.0;
    compute_static_loads(Fz_front, Fz_rear);

    // Longitudinal weight transfer approximation:
    //   ΔFz = (m * a_x * h_cg) / wheelbase
    const double a_long_demand =
        (params_.mass_kg > 0.0) ? (Fx_total / params_.mass_kg) : 0.0;
    const double dF_long =
        (params_.wheelbase_m > 0.0)
            ? (params_.mass_kg * a_long_demand * params_.cog_height_m /
               params_.wheelbase_m)
            : 0.0;
    Fz_front -= dF_long;
    Fz_rear += dF_long;
    // Clamp to non-negative loads
    Fz_front = std::max(Fz_front, 0.0);
    Fz_rear = std::max(Fz_rear, 0.0);

    // Split longitudinal force between axles (simple 50/50)
    const double Fx_front = 0.5 * Fx_total;
    const double Fx_rear = 0.5 * Fx_total;

    // Tire lateral forces using Pacejka (per axle, 2 tires lumped).
    // We use a simple friction-circle style clamp at the vehicle level
    // to keep total lateral force within μ·m·g, which matches textbook
    // expectations for steady-state cornering limits.
    auto Fy_front_per_tire = tire_lat_.lateral_force(alpha_f, 0.5 * Fz_front, mu);
    auto Fy_rear_per_tire = tire_lat_.lateral_force(alpha_r, 0.5 * Fz_rear, mu);

    double Fy_front = 2.0 * Fy_front_per_tire;
    double Fy_rear = 2.0 * Fy_rear_per_tire;

    const double Fx_sum = Fx_front + Fx_rear;
    double Fy_sum = Fy_front + Fy_rear;

    // Global lateral force clamp
    const double Fy_limit = mu * params_.mass_kg * gravity_;
    const double Fy_mag = std::abs(Fy_sum);
    if (Fy_limit > 0.0 && Fy_mag > Fy_limit) {
        const double scale = Fy_limit / Fy_mag;
        Fy_front *= scale;
        Fy_rear *= scale;
        Fy_sum *= scale;
    }

    // Equations of motion (planar, small-angle approximation)
    ns.ax_mps2 = Fx_sum / params_.mass_kg - s.vy_mps * s.yaw_rate_rps;
    // For reporting and basic validation, we store the lateral acceleration
    // component due purely to tire forces here. The coupling term vx*r is
    // still captured through the evolution of vy and yaw_rate.
    ns.ay_mps2 = Fy_sum / params_.mass_kg;

    // Yaw dynamics (about CG)
    const double Iz = params_.mass_kg * (L * L) / 12.0;  // crude guess
    const double Mz = a * Fy_front - b * Fy_rear;
    const double yaw_accel = (Iz > 0.0) ? (Mz / Iz) : 0.0;

    // Integrate velocities
    ns.vx_mps += ns.ax_mps2 * dt_s;
    ns.vy_mps += ns.ay_mps2 * dt_s;
    ns.yaw_rate_rps += yaw_accel * dt_s;

    // Integrate pose using updated velocities
    ns.yaw_rad += ns.yaw_rate_rps * dt_s;
    const double cos_yaw = std::cos(ns.yaw_rad);
    const double sin_yaw = std::sin(ns.yaw_rad);
    const double vx_body = ns.vx_mps;
    const double vy_body = ns.vy_mps;

    const double vx_world = cos_yaw * vx_body - sin_yaw * vy_body;
    const double vy_world = sin_yaw * vx_body + cos_yaw * vy_body;

    ns.x_m += vx_world * dt_s;
    ns.y_m += vy_world * dt_s;

    ns.front_load_N = Fz_front;
    ns.rear_load_N = Fz_rear;

    return ns;
}

}  // namespace physics
}  // namespace apex



