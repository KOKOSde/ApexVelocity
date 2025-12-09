#ifndef APEXVELOCITY_VELOCITY_PROFILE_SOLVER_H
#define APEXVELOCITY_VELOCITY_PROFILE_SOLVER_H

#include "physics/Types.h"
#include "physics/PhysicsMath.h"
#include "utils/ConfigManager.h"

#include <vector>
#include <functional>

namespace apex {
namespace physics {

/**
 * @brief Solver statistics and results summary.
 */
struct SolverResult {
    bool success = false;
    size_t points_processed = 0;
    double total_distance_m = 0.0;
    double total_energy_joules = 0.0;
    double max_speed_mps = 0.0;
    double min_speed_mps = 0.0;
    double avg_speed_mps = 0.0;
    double energy_per_meter = 0.0;
    std::string error_message;
};

/**
 * @brief Forward-Backward velocity profile solver for path traversal.
 * 
 * Implements a three-pass algorithm:
 * - Pass 0 (Static): Compute curvature/friction limited speeds
 * - Pass 1 (Backward): Apply braking constraints from end to start
 * - Pass 2 (Forward): Apply acceleration constraints from start to end
 * 
 * Also computes per-segment energy consumption.
 */
class VelocityProfileSolver {
public:
    /**
     * @brief Construct solver with vehicle params and configuration.
     * @param vehicle Vehicle parameters for dynamics calculations.
     * @param config Solver configuration options.
     */
    VelocityProfileSolver(const VehicleParams& vehicle, const SolverConfig& config = SolverConfig());
    
    /**
     * @brief Solve the velocity profile for a path.
     * @param path Vector of PathPoints (modified in place with v_profile and energy_joules).
     * @return SolverResult with statistics and success status.
     * 
     * The path must have at least 2 points with valid distance_along_m values.
     */
    SolverResult solve(Path& path);
    
    /**
     * @brief Solve using a specific ConfigManager instance.
     * @param path The path to solve.
     * @param config_mgr The configuration manager for material properties.
     * @return SolverResult with statistics.
     */
    SolverResult solve(Path& path, const ConfigManager& config_mgr);
    
    /**
     * @brief Get energy breakdown for each segment.
     * @return Vector of SegmentEnergy for each segment (size = path.size() - 1).
     */
    const std::vector<SegmentEnergy>& get_segment_energies() const { return segment_energies_; }
    
    /**
     * @brief Get the vehicle parameters.
     */
    const VehicleParams& get_vehicle() const { return vehicle_; }
    
    /**
     * @brief Get the solver configuration.
     */
    const SolverConfig& get_config() const { return config_; }
    
    /**
     * @brief Set a progress callback (called after each pass).
     * @param callback Function taking (pass_number, progress_0_to_1).
     */
    void set_progress_callback(std::function<void(int, double)> callback) {
        progress_callback_ = callback;
    }

private:
    /**
     * @brief Pass 0: Compute static speed limits from curvature and friction.
     */
    void compute_static_limits(Path& path, const ConfigManager& config_mgr);
    
    /**
     * @brief Pass 1: Backward pass applying braking constraints.
     */
    void backward_pass(Path& path, double gravity);
    
    /**
     * @brief Pass 2: Forward pass applying acceleration constraints.
     */
    void forward_pass(Path& path, double gravity);
    
    /**
     * @brief Compute energy for all segments.
     */
    void compute_energy(Path& path, const ConfigManager& config_mgr);
    
    /**
     * @brief Compute energy for a single segment.
     */
    SegmentEnergy compute_segment_energy(
        const PathPoint& p1, 
        const PathPoint& p2,
        const ConfigManager& config_mgr,
        double air_density
    );
    
    /**
     * @brief Get effective rolling resistance for a surface type.
     */
    double get_effective_rolling_resistance(
        const std::string& surface_type,
        const ConfigManager& config_mgr
    ) const;
    
    VehicleParams vehicle_;
    SolverConfig config_;
    std::vector<SegmentEnergy> segment_energies_;
    std::function<void(int, double)> progress_callback_;
    
    // Reference values for rolling resistance scaling
    static constexpr double ASPHALT_ROLLING_REF = 0.015;
};

} // namespace physics
} // namespace apex

#endif // APEXVELOCITY_VELOCITY_PROFILE_SOLVER_H





