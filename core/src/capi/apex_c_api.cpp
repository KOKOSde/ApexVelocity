/**
 * @file apex_c_api.cpp
 * @brief Implementation of the C API for ApexVelocity.
 */

#include "capi/apex_c_api.h"
#include "utils/ConfigManager.h"
#include "physics/PhysicsMath.h"
#include "physics/Types.h"
#include "physics/VehicleLoader.h"
#include "physics/VelocityProfileSolver.h"

#include <string>
#include <vector>
#include <cstring>
#include <mutex>
#include <unordered_map>

namespace {

// Global state for overrides and callbacks
std::mutex g_mutex;
double g_friction_multiplier_override = 1.0;
bool g_has_friction_multiplier_override = false;
std::unordered_map<std::string, apex::MaterialProps> g_material_overrides;

ApexFrictionCallback g_friction_callback = nullptr;
void* g_friction_callback_user_data = nullptr;

// Helper to duplicate a C string
char* strdup_safe(const char* s) {
    if (!s) return nullptr;
    size_t len = strlen(s) + 1;
    char* copy = new char[len];
    memcpy(copy, s, len);
    return copy;
}

// Convert C vehicle params to C++ VehicleParams
apex::physics::VehicleParams to_cpp_vehicle(const ApexVehicleParams* params) {
    apex::physics::VehicleParams v;
    if (params) {
        v.mass_kg = params->mass_kg;
        v.drag_coeff = params->drag_coeff;
        v.frontal_area_m2 = params->frontal_area_m2;
        v.rolling_res_base = params->rolling_res_base;
        v.max_lat_g = params->max_lat_g;
        v.max_brake_g = params->max_brake_g;
        v.max_power_w = params->max_power_w;
        v.powertrain_efficiency = params->powertrain_efficiency;
        v.track_width_m = params->track_width_m;
        v.cog_height_m = params->cog_height_m;
        if (params->name) v.name = params->name;
    }
    return v;
}

// Convert C solver config to C++ SolverConfig
apex::physics::SolverConfig to_cpp_config(const ApexSolverConfig* config) {
    apex::physics::SolverConfig c;
    if (config) {
        if (config->condition) c.condition = config->condition;
        c.enable_rollover_checks = config->enable_rollover_checks;
        c.enable_power_limit = config->enable_power_limit;
        c.min_speed_mps = config->min_speed_mps;
        c.initial_speed_mps = config->initial_speed_mps;
        c.final_speed_mps = config->final_speed_mps;
    }
    return c;
}

// Convert C path points to C++ Path
apex::physics::Path to_cpp_path(const ApexPathPoint* points, size_t num_points) {
    apex::physics::Path path;
    path.reserve(num_points);
    for (size_t i = 0; i < num_points; ++i) {
        apex::physics::PathPoint pt;
        pt.x_m = points[i].x_m;
        pt.y_m = points[i].y_m;
        pt.z_m = points[i].z_m;
        pt.curvature = points[i].curvature;
        pt.distance_along_m = points[i].distance_along_m;
        if (points[i].surface_type) {
            pt.surface_type = points[i].surface_type;
        }
        path.push_back(pt);
    }
    return path;
}

// Wrapper solver class that can use callbacks
class CallbackConfigManager {
public:
    double get_effective_mu(const std::string& material, const std::string& condition) {
        std::lock_guard<std::mutex> lock(g_mutex);
        
        // Check for callback first
        if (g_friction_callback) {
            return g_friction_callback(material.c_str(), condition.c_str(), g_friction_callback_user_data);
        }
        
        // Check for material override
        auto it = g_material_overrides.find(material);
        if (it != g_material_overrides.end()) {
            double base_mu = (condition == "wet") ? it->second.mu_wet : it->second.mu_dry;
            double multiplier = g_has_friction_multiplier_override ? g_friction_multiplier_override : 1.0;
            return base_mu * multiplier;
        }
        
        // Fall back to ConfigManager
        double mu = apex::ConfigManager::instance().get_effective_mu(material, condition);
        
        // Apply friction multiplier override if set
        if (g_has_friction_multiplier_override) {
            // The ConfigManager already applies its own multiplier, so we need to adjust
            double base_multiplier = apex::ConfigManager::instance().get_sim_param_or<double>("global_friction_multiplier", 1.0);
            mu = (mu / base_multiplier) * g_friction_multiplier_override;
        }
        
        return mu;
    }
};

} // anonymous namespace

extern "C" {

/* ============================================================================
 * Configuration functions
 * ============================================================================ */

int apex_config_init(const char* config_dir) {
    try {
        apex::ConfigManager::instance().initialize(config_dir ? config_dir : APEXVELOCITY_DEFAULT_CONFIG_DIR);
        return 0;
    } catch (...) {
        return -1;
    }
}

int apex_config_reload(void) {
    try {
        // Re-initialize to reload
        std::string config_dir = APEXVELOCITY_DEFAULT_CONFIG_DIR;
        apex::ConfigManager::instance().initialize(config_dir);
        return 0;
    } catch (...) {
        return -1;
    }
}

int apex_config_get_double(const char* key, double* out_value) {
    if (!key || !out_value) return -1;
    try {
        *out_value = apex::ConfigManager::instance().get_sim_param<double>(key);
        return 0;
    } catch (...) {
        return -1;
    }
}

int apex_config_get_bool(const char* key, bool* out_value) {
    if (!key || !out_value) return -1;
    try {
        *out_value = apex::ConfigManager::instance().get_sim_param<bool>(key);
        return 0;
    } catch (...) {
        return -1;
    }
}

void apex_config_set_friction_multiplier(double multiplier) {
    std::lock_guard<std::mutex> lock(g_mutex);
    g_friction_multiplier_override = multiplier;
    g_has_friction_multiplier_override = true;
}

void apex_config_set_material_overrides(const ApexMaterialOverride* overrides, size_t num_overrides) {
    std::lock_guard<std::mutex> lock(g_mutex);
    g_material_overrides.clear();
    for (size_t i = 0; i < num_overrides; ++i) {
        if (overrides[i].material_name) {
            apex::MaterialProps props;
            props.mu_dry = overrides[i].mu_dry;
            props.mu_wet = overrides[i].mu_wet;
            props.rolling_resistance_coeff = overrides[i].rolling_resistance_coeff;
            g_material_overrides[overrides[i].material_name] = props;
        }
    }
}

void apex_config_clear_overrides(void) {
    std::lock_guard<std::mutex> lock(g_mutex);
    g_friction_multiplier_override = 1.0;
    g_has_friction_multiplier_override = false;
    g_material_overrides.clear();
}

/* ============================================================================
 * Vehicle functions
 * ============================================================================ */

int apex_vehicle_load_preset(const char* preset_name, ApexVehicleParams* out_params) {
    if (!preset_name || !out_params) return -1;
    
    try {
        std::string presets_dir = std::string(APEXVELOCITY_DEFAULT_CONFIG_DIR) + "/vehicle_presets";
        apex::physics::VehicleLoader loader(presets_dir);
        
        std::string filename = std::string(preset_name) + ".json";
        auto params = loader.load_preset(filename);
        
        if (!params) return -1;
        
        out_params->mass_kg = params->mass_kg;
        out_params->drag_coeff = params->drag_coeff;
        out_params->frontal_area_m2 = params->frontal_area_m2;
        out_params->rolling_res_base = params->rolling_res_base;
        out_params->max_lat_g = params->max_lat_g;
        out_params->max_brake_g = params->max_brake_g;
        out_params->max_power_w = params->max_power_w;
        out_params->powertrain_efficiency = params->powertrain_efficiency;
        out_params->track_width_m = params->track_width_m;
        out_params->cog_height_m = params->cog_height_m;
        out_params->name = nullptr;  // Caller should not rely on this
        
        return 0;
    } catch (...) {
        return -1;
    }
}

void apex_vehicle_get_default(ApexVehicleParams* out_params) {
    if (!out_params) return;
    
    auto def = apex::physics::VehicleLoader::get_default();
    out_params->mass_kg = def.mass_kg;
    out_params->drag_coeff = def.drag_coeff;
    out_params->frontal_area_m2 = def.frontal_area_m2;
    out_params->rolling_res_base = def.rolling_res_base;
    out_params->max_lat_g = def.max_lat_g;
    out_params->max_brake_g = def.max_brake_g;
    out_params->max_power_w = def.max_power_w;
    out_params->powertrain_efficiency = def.powertrain_efficiency;
    out_params->track_width_m = def.track_width_m;
    out_params->cog_height_m = def.cog_height_m;
    out_params->name = nullptr;
}

/* ============================================================================
 * Solver functions
 * ============================================================================ */

struct ApexSolverImpl {
    apex::physics::VehicleParams vehicle;
    apex::physics::SolverConfig config;
};

ApexSolverHandle apex_solver_create(const ApexVehicleParams* vehicle, const ApexSolverConfig* config) {
    try {
        auto* impl = new ApexSolverImpl();
        impl->vehicle = to_cpp_vehicle(vehicle);
        impl->config = to_cpp_config(config);
        return static_cast<ApexSolverHandle>(impl);
    } catch (...) {
        return nullptr;
    }
}

void apex_solver_destroy(ApexSolverHandle solver) {
    if (solver) {
        delete static_cast<ApexSolverImpl*>(solver);
    }
}

ApexSolveResult apex_solver_solve(ApexSolverHandle solver, const ApexPathPoint* points, size_t num_points) {
    ApexSolveResult result = {};
    
    if (!solver || !points || num_points < 2) {
        result.success = false;
        result.error_message = strdup_safe("Invalid input parameters");
        return result;
    }
    
    try {
        auto* impl = static_cast<ApexSolverImpl*>(solver);
        
        // Convert path
        apex::physics::Path path = to_cpp_path(points, num_points);
        
        // Create solver and solve
        apex::physics::VelocityProfileSolver cpp_solver(impl->vehicle, impl->config);
        auto solve_result = cpp_solver.solve(path);
        
        if (!solve_result.success) {
            result.success = false;
            result.error_message = strdup_safe(solve_result.error_message.c_str());
            return result;
        }
        
        // Populate result
        result.success = true;
        result.num_points = path.size();
        result.velocity_profile_mps = new double[path.size()];
        result.energy_joules = new double[path.size()];
        
        for (size_t i = 0; i < path.size(); ++i) {
            result.velocity_profile_mps[i] = path[i].v_profile;
            result.energy_joules[i] = path[i].energy_joules;
        }
        
        // Segment energies
        const auto& segments = cpp_solver.get_segment_energies();
        result.segment_energy_joules = new double[segments.size()];
        for (size_t i = 0; i < segments.size(); ++i) {
            result.segment_energy_joules[i] = segments[i].total_joules;
        }
        
        result.total_energy_joules = solve_result.total_energy_joules;
        result.max_speed_mps = solve_result.max_speed_mps;
        result.min_speed_mps = solve_result.min_speed_mps;
        result.avg_speed_mps = solve_result.avg_speed_mps;
        result.error_message = nullptr;
        
        return result;
        
    } catch (const std::exception& e) {
        result.success = false;
        result.error_message = strdup_safe(e.what());
        return result;
    } catch (...) {
        result.success = false;
        result.error_message = strdup_safe("Unknown error");
        return result;
    }
}

ApexSolveResult apex_solve(
    const ApexVehicleParams* vehicle,
    const ApexSolverConfig* config,
    const ApexPathPoint* points,
    size_t num_points
) {
    ApexSolverHandle solver = apex_solver_create(vehicle, config);
    if (!solver) {
        ApexSolveResult result = {};
        result.success = false;
        result.error_message = strdup_safe("Failed to create solver");
        return result;
    }
    
    ApexSolveResult result = apex_solver_solve(solver, points, num_points);
    apex_solver_destroy(solver);
    return result;
}

void apex_result_free(ApexSolveResult* result) {
    if (!result) return;
    
    delete[] result->velocity_profile_mps;
    delete[] result->energy_joules;
    delete[] result->segment_energy_joules;
    delete[] result->error_message;
    
    result->velocity_profile_mps = nullptr;
    result->energy_joules = nullptr;
    result->segment_energy_joules = nullptr;
    result->error_message = nullptr;
}

/* ============================================================================
 * Physics utility functions
 * ============================================================================ */

double apex_calc_friction_speed(double curvature, double mu, double gravity) {
    return apex::physics::calc_friction_limited_speed_from_curvature(curvature, mu, gravity);
}

double apex_calc_rollover_speed(double curvature, double track_width, double cog_height, double gravity) {
    return apex::physics::calc_rollover_limited_speed_from_curvature(curvature, track_width, cog_height, gravity);
}

double apex_get_effective_mu(const char* material_name, const char* condition) {
    if (!material_name) return 0.9;  // Default
    
    std::string mat = material_name;
    std::string cond = condition ? condition : "dry";
    
    // Check callback first
    {
        std::lock_guard<std::mutex> lock(g_mutex);
        if (g_friction_callback) {
            return g_friction_callback(material_name, condition, g_friction_callback_user_data);
        }
    }
    
    return apex::ConfigManager::instance().get_effective_mu(mat, cond);
}

/* ============================================================================
 * Callback support
 * ============================================================================ */

void apex_set_friction_callback(ApexFrictionCallback callback, void* user_data) {
    std::lock_guard<std::mutex> lock(g_mutex);
    g_friction_callback = callback;
    g_friction_callback_user_data = user_data;
}

void apex_clear_friction_callback(void) {
    std::lock_guard<std::mutex> lock(g_mutex);
    g_friction_callback = nullptr;
    g_friction_callback_user_data = nullptr;
}

} // extern "C"





