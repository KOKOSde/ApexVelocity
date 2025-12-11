#ifndef APEXVELOCITY_DYNAMIC_VEHICLE_H
#define APEXVELOCITY_DYNAMIC_VEHICLE_H

#include "physics/Types.h"
#include "physics/TireModel.h"

namespace apex {
namespace physics {

/**
 * @brief Simple single-track (bicycle) dynamic vehicle model.
 *
 * This model is intended for research and validation use, not production
 * control. It captures:
 *  - Longitudinal and lateral velocities at the center of gravity.
 *  - Yaw rate and planar position.
 *  - Approximate weight transfer between front and rear axles.
 *
 * It uses a Pacejka tire model for lateral/longitudinal forces and combines
 * them with aerodynamic drag and rolling resistance.
 */
struct DynamicVehicleState {
    double x_m{0.0};
    double y_m{0.0};
    double yaw_rad{0.0};

    double vx_mps{0.0};
    double vy_mps{0.0};
    double yaw_rate_rps{0.0};

    double ax_mps2{0.0};
    double ay_mps2{0.0};

    double front_load_N{0.0};
    double rear_load_N{0.0};

    // Effective, filtered actuator commands (for lag modeling)
    double steering_eff_rad{0.0};
    double throttle_eff{0.0};
};

class DynamicVehicle {
public:
    DynamicVehicle(const VehicleParams& params,
                   const PacejkaTireParams& tire_lat_params,
                   const PacejkaTireParams& tire_long_params,
                   double gravity = 9.81);

    /**
     * @brief Simulate one timestep.
     *
     * @param state Current vehicle state.
     * @param steering_rad Front wheel steering angle (radians).
     * @param throttle Normalized traction/brake command in [-1, 1]
     *        (positive = drive, negative = brake).
     * @param mu Friction coefficient for current surface/condition.
     * @param dt_s Timestep in seconds.
     * @return Updated state after dt_s.
     */
    DynamicVehicleState step(const DynamicVehicleState& state,
                             double steering_rad,
                             double throttle,
                             double mu,
                             double dt_s) const;

    const VehicleParams& params() const { return params_; }

private:
    VehicleParams params_;
    PacejkaTireModel tire_lat_;
    PacejkaTireModel tire_long_;
    double gravity_;

    void compute_static_loads(double& Fz_front, double& Fz_rear) const;
};

}  // namespace physics
}  // namespace apex

#endif  // APEXVELOCITY_DYNAMIC_VEHICLE_H



