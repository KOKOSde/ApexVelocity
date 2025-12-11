#include "physics/TireModel.h"

#include <algorithm>
#include <cmath>

namespace apex {
namespace physics {

double PacejkaTireModel::magic_formula(double x, double peak_value) const {
    // Standard Magic Formula:
    //   y = D * sin(C * atan(B*x - E * (B*x - atan(B*x))))
    // We treat params_.D as a normalized peak and scale by peak_value.
    const double Bx = params_.B * x;
    const double inner = Bx - params_.E * (Bx - std::atan(Bx));
    const double angle = params_.C * std::atan(inner);
    return peak_value * params_.D * std::sin(angle);
}

double PacejkaTireModel::lateral_force(double slip_angle_rad,
                                       double normal_force_N,
                                       double mu) const {
    if (normal_force_N <= 0.0 || mu <= 0.0) {
        return 0.0;
    }
    const double peak = mu * normal_force_N;
    return magic_formula(slip_angle_rad, peak);
}

double PacejkaTireModel::longitudinal_force(double slip_ratio,
                                            double normal_force_N,
                                            double mu) const {
    if (normal_force_N <= 0.0 || mu <= 0.0) {
        return 0.0;
    }
    const double peak = mu * normal_force_N;
    return magic_formula(slip_ratio, peak);
}

std::pair<double, double> PacejkaTireModel::combined_forces(
    double slip_angle_rad,
    double slip_ratio,
    double normal_force_N,
    double mu) const {
    const double Fx_pure = longitudinal_force(slip_ratio, normal_force_N, mu);
    const double Fy_pure = lateral_force(slip_angle_rad, normal_force_N, mu);

    const double F_total = std::sqrt(Fx_pure * Fx_pure + Fy_pure * Fy_pure);
    const double F_max = mu * normal_force_N;

    if (F_total <= F_max || F_total <= 0.0 || F_max <= 0.0) {
        return {Fx_pure, Fy_pure};
    }

    const double scale = F_max / F_total;
    return {Fx_pure * scale, Fy_pure * scale};
}

}  // namespace physics
}  // namespace apex



