/**
 * @file apex_c_api.h
 * @brief C API for ApexVelocity core library.
 * 
 * This header provides a C-compatible interface for use with CGO (Go)
 * and other FFI systems. All functions use C linkage.
 */

#ifndef APEXVELOCITY_C_API_H
#define APEXVELOCITY_C_API_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Opaque handles
 * ============================================================================ */

typedef void* ApexSolverHandle;
typedef void* ApexConfigHandle;

/* ============================================================================
 * Data structures (C-compatible)
 * ============================================================================ */

/**
 * @brief Vehicle parameters for C API.
 */
typedef struct {
    double mass_kg;
    double drag_coeff;
    double frontal_area_m2;
    double rolling_res_base;
    double max_lat_g;
    double max_brake_g;
    double max_power_w;
    double powertrain_efficiency;
    double track_width_m;
    double cog_height_m;
    const char* name;
} ApexVehicleParams;

/**
 * @brief Path point for C API.
 */
typedef struct {
    double x_m;
    double y_m;
    double z_m;
    double curvature;
    double distance_along_m;
    const char* surface_type;
} ApexPathPoint;

/**
 * @brief Solver configuration.
 */
typedef struct {
    const char* condition;         // "dry" or "wet"
    bool enable_rollover_checks;
    bool enable_power_limit;
    double min_speed_mps;
    double initial_speed_mps;
    double final_speed_mps;
} ApexSolverConfig;

/**
 * @brief Result from solve operation.
 */
typedef struct {
    bool success;
    size_t num_points;
    double* velocity_profile_mps;   // Array of velocities
    double* energy_joules;          // Array of cumulative energy
    double* segment_energy_joules;  // Array of per-segment energy (size = num_points - 1)
    double total_energy_joules;
    double max_speed_mps;
    double min_speed_mps;
    double avg_speed_mps;
    char* error_message;            // NULL if success, otherwise error string
} ApexSolveResult;

/**
 * @brief Material property override.
 */
typedef struct {
    const char* material_name;
    double mu_dry;
    double mu_wet;
    double rolling_resistance_coeff;
} ApexMaterialOverride;

/* ============================================================================
 * Configuration functions
 * ============================================================================ */

/**
 * @brief Initialize the configuration manager with a config directory.
 * @param config_dir Path to the configuration directory.
 * @return 0 on success, non-zero on error.
 */
int apex_config_init(const char* config_dir);

/**
 * @brief Reload configuration files.
 * @return 0 on success, non-zero on error.
 */
int apex_config_reload(void);

/**
 * @brief Get a simulation parameter as double.
 * @param key Parameter name.
 * @param out_value Pointer to store the value.
 * @return 0 on success, non-zero if not found.
 */
int apex_config_get_double(const char* key, double* out_value);

/**
 * @brief Get a simulation parameter as bool.
 * @param key Parameter name.
 * @param out_value Pointer to store the value.
 * @return 0 on success, non-zero if not found.
 */
int apex_config_get_bool(const char* key, bool* out_value);

/**
 * @brief Set a global friction multiplier override.
 * @param multiplier The friction multiplier (1.0 = no change).
 */
void apex_config_set_friction_multiplier(double multiplier);

/**
 * @brief Apply temporary material overrides (for a single solve operation).
 * @param overrides Array of material overrides.
 * @param num_overrides Number of overrides.
 */
void apex_config_set_material_overrides(const ApexMaterialOverride* overrides, size_t num_overrides);

/**
 * @brief Clear all temporary overrides.
 */
void apex_config_clear_overrides(void);

/* ============================================================================
 * Vehicle functions
 * ============================================================================ */

/**
 * @brief Load a vehicle preset by name.
 * @param preset_name Name of the preset (e.g., "tesla_model_3").
 * @param out_params Pointer to store the vehicle parameters.
 * @return 0 on success, non-zero if preset not found.
 */
int apex_vehicle_load_preset(const char* preset_name, ApexVehicleParams* out_params);

/**
 * @brief Get default vehicle parameters.
 * @param out_params Pointer to store the vehicle parameters.
 */
void apex_vehicle_get_default(ApexVehicleParams* out_params);

/* ============================================================================
 * Solver functions
 * ============================================================================ */

/**
 * @brief Create a new solver instance.
 * @param vehicle Vehicle parameters.
 * @param config Solver configuration (can be NULL for defaults).
 * @return Solver handle, or NULL on error.
 */
ApexSolverHandle apex_solver_create(const ApexVehicleParams* vehicle, const ApexSolverConfig* config);

/**
 * @brief Destroy a solver instance.
 * @param solver Solver handle.
 */
void apex_solver_destroy(ApexSolverHandle solver);

/**
 * @brief Solve a velocity profile.
 * @param solver Solver handle.
 * @param points Array of path points.
 * @param num_points Number of points.
 * @return Solve result (caller must free with apex_result_free).
 */
ApexSolveResult apex_solver_solve(ApexSolverHandle solver, const ApexPathPoint* points, size_t num_points);

/**
 * @brief Convenience function to solve without creating a persistent solver.
 * @param vehicle Vehicle parameters.
 * @param config Solver configuration (can be NULL for defaults).
 * @param points Array of path points.
 * @param num_points Number of points.
 * @return Solve result (caller must free with apex_result_free).
 */
ApexSolveResult apex_solve(
    const ApexVehicleParams* vehicle,
    const ApexSolverConfig* config,
    const ApexPathPoint* points,
    size_t num_points
);

/**
 * @brief Free a solve result.
 * @param result Pointer to the result to free.
 */
void apex_result_free(ApexSolveResult* result);

/* ============================================================================
 * Physics utility functions
 * ============================================================================ */

/**
 * @brief Calculate friction-limited speed.
 * @param curvature Path curvature (1/m).
 * @param mu Friction coefficient.
 * @param gravity Gravitational acceleration (m/s^2).
 * @return Maximum speed (m/s).
 */
double apex_calc_friction_speed(double curvature, double mu, double gravity);

/**
 * @brief Calculate rollover-limited speed.
 * @param curvature Path curvature (1/m).
 * @param track_width Track width (m).
 * @param cog_height Center of gravity height (m).
 * @param gravity Gravitational acceleration (m/s^2).
 * @return Maximum speed (m/s).
 */
double apex_calc_rollover_speed(double curvature, double track_width, double cog_height, double gravity);

/**
 * @brief Get effective friction coefficient for a material.
 * @param material_name Material name (e.g., "asphalt").
 * @param condition "dry" or "wet".
 * @return Effective friction coefficient.
 */
double apex_get_effective_mu(const char* material_name, const char* condition);

/* ============================================================================
 * Callback support (for Python integration)
 * ============================================================================ */

/**
 * @brief Function pointer type for friction callback.
 * @param surface_name The surface material name.
 * @param condition "dry" or "wet".
 * @param user_data User-provided context.
 * @return Friction coefficient to use.
 */
typedef double (*ApexFrictionCallback)(const char* surface_name, const char* condition, void* user_data);

/**
 * @brief Register a friction callback.
 * @param callback The callback function.
 * @param user_data User context passed to callback.
 */
void apex_set_friction_callback(ApexFrictionCallback callback, void* user_data);

/**
 * @brief Clear the friction callback.
 */
void apex_clear_friction_callback(void);

#ifdef __cplusplus
}
#endif

#endif /* APEXVELOCITY_C_API_H */





