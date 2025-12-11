#ifndef APEXVELOCITY_TIRE_MODEL_H
#define APEXVELOCITY_TIRE_MODEL_H

#include <utility>

namespace apex {
namespace physics {

/**
 * @brief Parameters for the Pacejka "Magic Formula" tire model.
 *
 * These parameters shape the normalized force–slip curve. In practice, the
 * peak factor D is often proportional to μ·Fz, where μ is the friction
 * coefficient and Fz is the normal load. Here we keep D as a unit peak
 * and scale by μ and Fz at call time.
 *
 * Typical passenger car values (from commonly cited examples in tire
 * dynamics literature) are in the range:
 *  - B ≈ 10 (stiffness)
 *  - C ≈ 1.9 (shape)
 *  - D = 1.0 (normalized peak)
 *  - E ≈ -1.0 (curvature)
 */
struct PacejkaTireParams {
    double B{10.0};   ///< Stiffness factor
    double C{1.9};    ///< Shape factor
    double D{1.0};    ///< Peak factor (normalized)
    double E{-1.0};   ///< Curvature factor
};

/**
 * @brief Simple Pacejka Magic Formula tire model.
 *
 * This class implements a symmetric Magic Formula curve for both lateral
 * (slip angle) and longitudinal (slip ratio) behavior. It is not meant to
 * be a full tire library, but provides a realistic S-shaped force curve
 * that can be calibrated via config.
 */
class PacejkaTireModel {
public:
    explicit PacejkaTireModel(const PacejkaTireParams& params) : params_(params) {}

    /**
     * @brief Compute lateral force from slip angle.
     *
     * @param slip_angle_rad Tire slip angle in radians.
     * @param normal_force_N Normal load on the tire (N).
     * @param mu Friction coefficient (dimensionless).
     * @return Lateral force Fy (N).
     */
    double lateral_force(double slip_angle_rad,
                         double normal_force_N,
                         double mu) const;

    /**
     * @brief Compute longitudinal force from slip ratio.
     *
     * @param slip_ratio Longitudinal slip ratio (dimensionless, typically -1..1).
     * @param normal_force_N Normal load on the tire (N).
     * @param mu Friction coefficient.
     * @return Longitudinal force Fx (N).
     */
    double longitudinal_force(double slip_ratio,
                              double normal_force_N,
                              double mu) const;

    /**
     * @brief Combined-slip forces (friction circle style).
     *
     * This first computes pure lateral and pure longitudinal forces, then
     * rescales them so that sqrt(Fx^2 + Fy^2) <= mu * normal_force_N.
     *
     * @param slip_angle_rad Lateral slip angle in radians.
     * @param slip_ratio Longitudinal slip ratio.
     * @param normal_force_N Normal load on the tire (N).
     * @param mu Friction coefficient.
     * @return Pair (Fx, Fy) in Newtons.
     */
    std::pair<double, double> combined_forces(double slip_angle_rad,
                                              double slip_ratio,
                                              double normal_force_N,
                                              double mu) const;

private:
    PacejkaTireParams params_;

    double magic_formula(double x, double peak_value) const;
};

}  // namespace physics
}  // namespace apex

#endif  // APEXVELOCITY_TIRE_MODEL_H



