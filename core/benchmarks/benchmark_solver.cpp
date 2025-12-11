#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "physics/Types.h"
#include "physics/PhysicsMath.h"
#include "utils/ConfigManager.h"
#include "utils/ThreadPool.h"

using apex::ConfigManager;
using apex::ThreadPool;
using apex::physics::Path;
using apex::physics::PathPoint;
using apex::physics::SolverConfig;
using apex::physics::VehicleParams;

namespace {

Path create_straight_path(double length_m, double step_m) {
    Path path;
    double distance = 0.0;
    double x = 0.0;

    while (distance <= length_m + 1e-6) {
        PathPoint pt;
        pt.x_m = x;
        pt.y_m = 0.0;
        pt.z_m = 0.0;
        pt.curvature = 0.0;
        pt.distance_along_m = distance;
        pt.surface_type = "asphalt";
        path.push_back(pt);

        distance += step_m;
        x += step_m;
    }
    return path;
}

std::vector<Path> generate_paths(std::size_t num_paths, double length_m, double step_m) {
    std::vector<Path> paths;
    paths.reserve(num_paths);
    for (std::size_t i = 0; i < num_paths; ++i) {
        paths.emplace_back(create_straight_path(length_m, step_m));
    }
    return paths;
}

}  // namespace

int main(int argc, char** argv) {
    std::size_t num_paths = 1000;
    double length_m = 200.0;
    double step_m = 2.0;
    std::size_t num_threads = std::thread::hardware_concurrency();

    if (argc > 1) {
        num_paths = static_cast<std::size_t>(std::strtoul(argv[1], nullptr, 10));
    }
    if (argc > 2) {
        num_threads = static_cast<std::size_t>(std::strtoul(argv[2], nullptr, 10));
    }

    std::cout << "ApexVelocity simple batch benchmark\n";
    std::cout << "  paths       : " << num_paths << "\n";
    std::cout << "  path length : " << length_m << " m\n";
    std::cout << "  step        : " << step_m << " m\n";
    std::cout << "  threads     : " << num_threads << "\n";

    auto& cfg = ConfigManager::instance();
    if (!cfg.is_initialized()) {
        cfg.initialize(APEXVELOCITY_DEFAULT_CONFIG_DIR);
    }

    VehicleParams vehicle;
    vehicle.name = "benchmark_vehicle";

    SolverConfig config;
    config.initial_speed_mps = 20.0;
    config.condition = "dry";

    auto paths = generate_paths(num_paths, length_m, step_m);

    ThreadPool pool(num_threads);

    auto t_start = std::chrono::steady_clock::now();

    std::vector<std::future<void>> futures;
    futures.reserve(paths.size());

    // For now, this benchmark only exercises the physics math and config
    // layers by computing static friction-limited speeds for each point.
    for (std::size_t i = 0; i < paths.size(); ++i) {
        futures.emplace_back(pool.enqueue([&, i]() {
            auto& path = paths[i];
            const double gravity = cfg.get_sim_param_or<double>("gravity", apex::physics::PhysicsConstants::EARTH_GRAVITY);
            for (auto& pt : path) {
                double mu = cfg.get_effective_mu(pt.surface_type, config.condition);
                pt.v_static = apex::physics::calc_combined_limited_speed(
                    pt.curvature,
                    mu,
                    vehicle.track_width_m,
                    vehicle.cog_height_m,
                    gravity,
                    /*enable_rollover_check=*/true
                );
            }
        }));
    }

    for (auto& f : futures) {
        f.get();
    }

    auto t_end = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = t_end - t_start;

    double elapsed_s = elapsed.count();
    double routes_per_second = (elapsed_s > 0.0)
                                   ? static_cast<double>(paths.size()) / elapsed_s
                                   : 0.0;

    std::cout << "Benchmark results:\n";
    std::cout << "  elapsed       : " << elapsed_s << " s\n";
    std::cout << "  routes/second : " << routes_per_second << "\n";

    return 0;
}


