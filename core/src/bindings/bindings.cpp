/**
 * @file bindings.cpp
 * @brief Pybind11 bindings for ApexVelocity core library.
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>

#include "utils/ConfigManager.h"
#include "physics/PhysicsMath.h"
#include "physics/Types.h"
#include "physics/VehicleLoader.h"
#include "physics/VelocityProfileSolver.h"

#include <mutex>
#include <functional>

namespace py = pybind11;

namespace {

// Global Python friction callback
std::mutex g_py_callback_mutex;
std::function<double(const std::string&, const std::string&)> g_py_friction_callback;
bool g_has_py_callback = false;

// Wrapper class for ConfigManager with Python callback support
class PyConfigManager {
public:
    static void set_friction_callback(std::function<double(const std::string&, const std::string&)> callback) {
        std::lock_guard<std::mutex> lock(g_py_callback_mutex);
        g_py_friction_callback = callback;
        g_has_py_callback = true;
    }
    
    static void clear_friction_callback() {
        std::lock_guard<std::mutex> lock(g_py_callback_mutex);
        g_py_friction_callback = nullptr;
        g_has_py_callback = false;
    }
    
    static double get_effective_mu(const std::string& material, const std::string& condition) {
        {
            std::lock_guard<std::mutex> lock(g_py_callback_mutex);
            if (g_has_py_callback && g_py_friction_callback) {
                return g_py_friction_callback(material, condition);
            }
        }
        return apex::ConfigManager::instance().get_effective_mu(material, condition);
    }
    
    static bool has_callback() {
        std::lock_guard<std::mutex> lock(g_py_callback_mutex);
        return g_has_py_callback;
    }
};

// Custom solver that uses Python callback
class PySolver {
public:
    PySolver(const apex::physics::VehicleParams& vehicle, 
             const apex::physics::SolverConfig& config = apex::physics::SolverConfig())
        : vehicle_(vehicle), config_(config) {}
    
    py::dict solve(const std::vector<apex::physics::PathPoint>& input_points) {
        apex::physics::Path path = input_points;
        
        // If we have a Python callback, we need to modify the solver behavior
        // For simplicity, we'll compute static limits ourselves using the callback
        if (PyConfigManager::has_callback()) {
            double gravity = apex::ConfigManager::instance().get_sim_param_or<double>("gravity", 9.81);
            
            for (auto& pt : path) {
                double mu = PyConfigManager::get_effective_mu(pt.surface_type, config_.condition);
                double v_friction = apex::physics::calc_friction_limited_speed_from_curvature(
                    pt.curvature, mu, gravity
                );
                pt.v_static = v_friction;
                pt.v_profile = v_friction;
                
                // Apply vehicle lateral G limit
                if (pt.curvature > apex::physics::PhysicsConstants::MIN_CURVATURE) {
                    double v_lat = std::sqrt(vehicle_.max_lat_g * gravity / pt.curvature);
                    pt.v_static = std::min(pt.v_static, v_lat);
                    pt.v_profile = pt.v_static;
                }
            }
        }
        
        apex::physics::VelocityProfileSolver solver(vehicle_, config_);
        auto result = solver.solve(path);
        
        py::dict output;
        output["success"] = result.success;
        output["error_message"] = result.error_message;
        output["total_distance_m"] = result.total_distance_m;
        output["total_energy_joules"] = result.total_energy_joules;
        output["max_speed_mps"] = result.max_speed_mps;
        output["min_speed_mps"] = result.min_speed_mps;
        output["avg_speed_mps"] = result.avg_speed_mps;
        output["energy_per_meter"] = result.energy_per_meter;
        
        // Extract arrays
        std::vector<double> velocities, energies, static_limits;
        for (const auto& pt : path) {
            velocities.push_back(pt.v_profile);
            energies.push_back(pt.energy_joules);
            static_limits.push_back(pt.v_static);
        }
        
        output["velocity_profile_mps"] = velocities;
        output["energy_joules"] = energies;
        output["static_limits_mps"] = static_limits;
        
        // Segment energies
        const auto& segments = solver.get_segment_energies();
        std::vector<double> seg_energies;
        for (const auto& seg : segments) {
            seg_energies.push_back(seg.total_joules);
        }
        output["segment_energy_joules"] = seg_energies;
        
        return output;
    }

private:
    apex::physics::VehicleParams vehicle_;
    apex::physics::SolverConfig config_;
};

// Convenience function for simple solving
py::dict solve_profile(
    const std::vector<std::vector<double>>& geometry,  // [[x, y, z, curvature, distance], ...]
    const std::vector<std::string>& surfaces,
    const std::string& vehicle_name = "default",
    const std::string& condition = "dry",
    double initial_speed = 0.0,
    double final_speed = 0.0,
    const std::string& physics_model = "kinematic"
) {
    // Build path
    apex::physics::Path path;
    for (size_t i = 0; i < geometry.size(); ++i) {
        apex::physics::PathPoint pt;
        if (geometry[i].size() >= 5) {
            pt.x_m = geometry[i][0];
            pt.y_m = geometry[i][1];
            pt.z_m = geometry[i][2];
            pt.curvature = geometry[i][3];
            pt.distance_along_m = geometry[i][4];
        }
        if (i < surfaces.size()) {
            pt.surface_type = surfaces[i];
        }
        path.push_back(pt);
    }
    
    // Load vehicle
    apex::physics::VehicleParams vehicle;
    if (vehicle_name != "default") {
        std::string presets_dir = std::string(APEXVELOCITY_DEFAULT_CONFIG_DIR) + "/vehicle_presets";
        apex::physics::VehicleLoader loader(presets_dir);
        auto loaded = loader.load_preset(vehicle_name + ".json");
        if (loaded) {
            vehicle = *loaded;
        } else {
            vehicle = apex::physics::VehicleLoader::get_default();
        }
    } else {
        vehicle = apex::physics::VehicleLoader::get_default();
    }
    
    // Config
    apex::physics::SolverConfig config;
    config.condition = condition;
    config.initial_speed_mps = initial_speed;
    config.final_speed_mps = final_speed;
    
    // Set physics model
    if (physics_model == "dynamic") {
        config.model = apex::physics::PhysicsModel::DYNAMIC;
    } else {
        config.model = apex::physics::PhysicsModel::KINEMATIC;
    }
    
    PySolver solver(vehicle, config);
    return solver.solve(path);
}

} // anonymous namespace

PYBIND11_MODULE(_apexvelocity_core, m) {
    m.doc() = "ApexVelocity Core - Physics-Based Graph Annotation Engine";
    
    // ========================================================================
    // Physics Constants
    // ========================================================================
    py::class_<apex::physics::PhysicsConstants>(m, "PhysicsConstants")
        .def_readonly_static("EARTH_GRAVITY", &apex::physics::PhysicsConstants::EARTH_GRAVITY)
        .def_readonly_static("MARS_GRAVITY", &apex::physics::PhysicsConstants::MARS_GRAVITY)
        .def_readonly_static("MAX_SAFE_SPEED", &apex::physics::PhysicsConstants::MAX_SAFE_SPEED)
        .def_readonly_static("MIN_CURVATURE", &apex::physics::PhysicsConstants::MIN_CURVATURE);
    
    // ========================================================================
    // MaterialProps
    // ========================================================================
    py::class_<apex::MaterialProps>(m, "MaterialProps")
        .def(py::init<>())
        .def(py::init<double, double, double>())
        .def_readwrite("mu_dry", &apex::MaterialProps::mu_dry)
        .def_readwrite("mu_wet", &apex::MaterialProps::mu_wet)
        .def_readwrite("rolling_resistance_coeff", &apex::MaterialProps::rolling_resistance_coeff);
    
    // ========================================================================
    // VehicleParams
    // ========================================================================
    py::class_<apex::physics::VehicleParams>(m, "VehicleParams")
        .def(py::init<>())
        .def_readwrite("mass_kg", &apex::physics::VehicleParams::mass_kg)
        .def_readwrite("drag_coeff", &apex::physics::VehicleParams::drag_coeff)
        .def_readwrite("frontal_area_m2", &apex::physics::VehicleParams::frontal_area_m2)
        .def_readwrite("rolling_res_base", &apex::physics::VehicleParams::rolling_res_base)
        .def_readwrite("max_lat_g", &apex::physics::VehicleParams::max_lat_g)
        .def_readwrite("max_brake_g", &apex::physics::VehicleParams::max_brake_g)
        .def_readwrite("max_power_w", &apex::physics::VehicleParams::max_power_w)
        .def_readwrite("powertrain_efficiency", &apex::physics::VehicleParams::powertrain_efficiency)
        .def_readwrite("track_width_m", &apex::physics::VehicleParams::track_width_m)
        .def_readwrite("cog_height_m", &apex::physics::VehicleParams::cog_height_m)
        .def_readwrite("name", &apex::physics::VehicleParams::name)
        .def("is_valid", &apex::physics::VehicleParams::is_valid);
    
    // ========================================================================
    // PathPoint
    // ========================================================================
    py::class_<apex::physics::PathPoint>(m, "PathPoint")
        .def(py::init<>())
        .def_readwrite("x_m", &apex::physics::PathPoint::x_m)
        .def_readwrite("y_m", &apex::physics::PathPoint::y_m)
        .def_readwrite("z_m", &apex::physics::PathPoint::z_m)
        .def_readwrite("curvature", &apex::physics::PathPoint::curvature)
        .def_readwrite("distance_along_m", &apex::physics::PathPoint::distance_along_m)
        .def_readwrite("surface_type", &apex::physics::PathPoint::surface_type)
        .def_readwrite("v_static", &apex::physics::PathPoint::v_static)
        .def_readwrite("v_profile", &apex::physics::PathPoint::v_profile)
        .def_readwrite("energy_joules", &apex::physics::PathPoint::energy_joules)
        .def("distance_to_2d", &apex::physics::PathPoint::distance_to_2d)
        .def("distance_to_3d", &apex::physics::PathPoint::distance_to_3d)
        .def("grade_angle_to", &apex::physics::PathPoint::grade_angle_to);
    
    // ========================================================================
    // PhysicsModel enum
    // ========================================================================
    py::enum_<apex::physics::PhysicsModel>(m, "PhysicsModel")
        .value("KINEMATIC", apex::physics::PhysicsModel::KINEMATIC)
        .value("DYNAMIC", apex::physics::PhysicsModel::DYNAMIC)
        .export_values();
    
    // ========================================================================
    // SolverConfig
    // ========================================================================
    py::class_<apex::physics::SolverConfig>(m, "SolverConfig")
        .def(py::init<>())
        .def_readwrite("condition", &apex::physics::SolverConfig::condition)
        .def_readwrite("enable_rollover_checks", &apex::physics::SolverConfig::enable_rollover_checks)
        .def_readwrite("enable_power_limit", &apex::physics::SolverConfig::enable_power_limit)
        .def_readwrite("min_speed_mps", &apex::physics::SolverConfig::min_speed_mps)
        .def_readwrite("initial_speed_mps", &apex::physics::SolverConfig::initial_speed_mps)
        .def_readwrite("final_speed_mps", &apex::physics::SolverConfig::final_speed_mps)
        .def_readwrite("model", &apex::physics::SolverConfig::model)
        .def_readwrite("enable_tire_slip", &apex::physics::SolverConfig::enable_tire_slip)
        .def_readwrite("enable_weight_transfer", &apex::physics::SolverConfig::enable_weight_transfer);
    
    // ========================================================================
    // VehicleLoader
    // ========================================================================
    py::class_<apex::physics::VehicleLoader>(m, "VehicleLoader")
        .def(py::init<const std::string&>())
        .def("load_preset", &apex::physics::VehicleLoader::load_preset)
        .def("load_all_presets", &apex::physics::VehicleLoader::load_all_presets)
        .def("list_presets", &apex::physics::VehicleLoader::list_presets)
        .def_static("from_json_string", &apex::physics::VehicleLoader::from_json_string)
        .def_static("get_default", &apex::physics::VehicleLoader::get_default);
    
    // ========================================================================
    // ConfigManager access
    // ========================================================================
    m.def("config_init", [](const std::string& path) {
        apex::ConfigManager::instance().initialize(path);
    }, py::arg("config_dir"), "Initialize configuration from directory");
    
    m.def("get_material", [](const std::string& name) {
        return apex::ConfigManager::instance().get_material(name);
    }, py::arg("material_name"), "Get material properties");
    
    m.def("get_effective_mu", &PyConfigManager::get_effective_mu,
        py::arg("material_name"), py::arg("condition") = "dry",
        "Get effective friction coefficient (uses callback if set)");
    
    m.def("get_sim_param_double", [](const std::string& key) {
        return apex::ConfigManager::instance().get_sim_param<double>(key);
    }, py::arg("key"), "Get simulation parameter as double");
    
    m.def("get_sim_param_bool", [](const std::string& key) {
        return apex::ConfigManager::instance().get_sim_param<bool>(key);
    }, py::arg("key"), "Get simulation parameter as bool");
    
    // ========================================================================
    // Friction callback
    // ========================================================================
    m.def("set_friction_callback", &PyConfigManager::set_friction_callback,
        py::arg("callback"),
        "Set a Python callback for friction coefficient lookup.\n"
        "Callback signature: (surface_name: str, condition: str) -> float");
    
    m.def("clear_friction_callback", &PyConfigManager::clear_friction_callback,
        "Clear the friction callback");
    
    // ========================================================================
    // Physics math functions
    // ========================================================================
    m.def("calc_friction_limited_speed", &apex::physics::calc_friction_limited_speed_from_curvature,
        py::arg("curvature"), py::arg("mu"), 
        py::arg("gravity") = apex::physics::PhysicsConstants::EARTH_GRAVITY,
        "Calculate friction-limited speed for a curve");
    
    m.def("calc_rollover_limited_speed", &apex::physics::calc_rollover_limited_speed_from_curvature,
        py::arg("curvature"), py::arg("track_width_m"), py::arg("cog_height_m"),
        py::arg("gravity") = apex::physics::PhysicsConstants::EARTH_GRAVITY,
        "Calculate rollover-limited speed for a curve");
    
    m.def("curvature_to_radius", &apex::physics::curvature_to_radius,
        py::arg("curvature"), "Convert curvature to radius");
    
    m.def("radius_to_curvature", &apex::physics::radius_to_curvature,
        py::arg("radius"), "Convert radius to curvature");
    
    m.def("mps_to_kmph", &apex::physics::mps_to_kmph, py::arg("mps"));
    m.def("kmph_to_mps", &apex::physics::kmph_to_mps, py::arg("kmph"));
    
    // ========================================================================
    // Solver
    // ========================================================================
    py::class_<PySolver>(m, "Solver")
        .def(py::init<const apex::physics::VehicleParams&, const apex::physics::SolverConfig&>(),
             py::arg("vehicle"), py::arg("config") = apex::physics::SolverConfig())
        .def("solve", &PySolver::solve, py::arg("path"),
             "Solve velocity profile for a path");
    
    // ========================================================================
    // Convenience function
    // ========================================================================
    m.def("solve_profile", &solve_profile,
        py::arg("geometry"),
        py::arg("surfaces"),
        py::arg("vehicle") = "default",
        py::arg("condition") = "dry",
        py::arg("initial_speed") = 0.0,
        py::arg("final_speed") = 0.0,
        py::arg("physics_model") = "kinematic",
        "Convenience function to solve a velocity profile.\n"
        "geometry: list of [x, y, z, curvature, distance_along]\n"
        "surfaces: list of surface type strings\n"
        "vehicle: vehicle preset name or 'default'\n"
        "condition: 'dry' or 'wet'\n"
        "physics_model: 'kinematic' (fast, 3-pass) or 'dynamic' (tire model)");
    
    // ========================================================================
    // Default config directory
    // ========================================================================
    m.attr("DEFAULT_CONFIG_DIR") = APEXVELOCITY_DEFAULT_CONFIG_DIR;
}





