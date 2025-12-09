#ifndef APEXVELOCITY_CONFIG_MANAGER_H
#define APEXVELOCITY_CONFIG_MANAGER_H

#include <string>
#include <unordered_map>
#include <stdexcept>
#include <memory>

#include <yaml-cpp/yaml.h>
#include <nlohmann/json.hpp>

namespace apex {

/**
 * @brief Material physical properties for friction and rolling resistance calculations.
 */
struct MaterialProps {
    double mu_dry;                    ///< Coefficient of friction (dry conditions)
    double mu_wet;                    ///< Coefficient of friction (wet conditions)
    double rolling_resistance_coeff;  ///< Rolling resistance coefficient
    
    MaterialProps() : mu_dry(0.9), mu_wet(0.6), rolling_resistance_coeff(0.015) {}
    
    MaterialProps(double dry, double wet, double rolling)
        : mu_dry(dry), mu_wet(wet), rolling_resistance_coeff(rolling) {}
};

/**
 * @brief Singleton configuration manager for ApexVelocity.
 * 
 * Loads and manages simulation parameters from YAML and material/tag data from JSON.
 * Thread-safe initialization with lazy loading support.
 */
class ConfigManager {
public:
    /**
     * @brief Get the singleton instance.
     * @return Reference to the ConfigManager instance.
     */
    static ConfigManager& instance();
    
    /**
     * @brief Initialize with a specific config directory path.
     * @param config_dir Path to the configuration directory.
     * 
     * This must be called before any other methods if you want to use
     * a non-default config directory. Can only be called once.
     */
    void initialize(const std::string& config_dir);
    
    /**
     * @brief Check if the manager has been initialized.
     */
    bool is_initialized() const { return initialized_; }
    
    /**
     * @brief Get material properties by name.
     * @param material_name The material identifier (e.g., "asphalt", "gravel").
     * @return Const reference to MaterialProps. Returns default asphalt-like values if not found.
     */
    const MaterialProps& get_material(const std::string& material_name) const;
    
    /**
     * @brief Get the material name for an OSM tag.
     * @param osm_tag The OSM tag string (e.g., "highway=motorway").
     * @return Material name string. Returns "asphalt" as default if tag not found.
     */
    std::string get_material_for_tag(const std::string& osm_tag) const;
    
    /**
     * @brief Calculate effective friction coefficient.
     * @param material_name The material identifier.
     * @param condition Either "dry" or "wet".
     * @return Effective mu value, adjusted by global_friction_multiplier.
     */
    double get_effective_mu(const std::string& material_name,
                           const std::string& condition = "dry") const;
    
    /**
     * @brief Get a simulation parameter by key.
     * @tparam T The expected type (double, bool, int, std::string).
     * @param key The parameter name from simulation.yaml.
     * @return The parameter value.
     * @throws std::runtime_error if key not found or type mismatch.
     */
    template <typename T>
    T get_sim_param(const std::string& key) const;
    
    /**
     * @brief Get a simulation parameter with default value.
     * @tparam T The expected type.
     * @param key The parameter name.
     * @param default_value Value to return if key not found.
     * @return The parameter value or default.
     */
    template <typename T>
    T get_sim_param_or(const std::string& key, const T& default_value) const;
    
    // Delete copy/move operations for singleton
    ConfigManager(const ConfigManager&) = delete;
    ConfigManager& operator=(const ConfigManager&) = delete;
    ConfigManager(ConfigManager&&) = delete;
    ConfigManager& operator=(ConfigManager&&) = delete;

private:
    ConfigManager();
    ~ConfigManager() = default;
    
    void load_simulation_config(const std::string& path);
    void load_materials(const std::string& path);
    void load_tag_mapping(const std::string& path);
    void ensure_initialized() const;
    
    bool initialized_ = false;
    std::string config_dir_;
    
    YAML::Node sim_config_;
    std::unordered_map<std::string, MaterialProps> materials_;
    std::unordered_map<std::string, std::string> tag_to_material_;
    
    MaterialProps default_material_;  // Fallback for unknown materials
};

// ============================================================================
// Template implementations
// ============================================================================

template <typename T>
T ConfigManager::get_sim_param(const std::string& key) const {
    ensure_initialized();
    
    if (!sim_config_[key]) {
        throw std::runtime_error("Simulation parameter not found: " + key);
    }
    
    try {
        return sim_config_[key].as<T>();
    } catch (const YAML::Exception& e) {
        throw std::runtime_error("Type mismatch for parameter '" + key + "': " + e.what());
    }
}

template <typename T>
T ConfigManager::get_sim_param_or(const std::string& key, const T& default_value) const {
    ensure_initialized();
    
    if (!sim_config_[key]) {
        return default_value;
    }
    
    try {
        return sim_config_[key].as<T>();
    } catch (const YAML::Exception&) {
        return default_value;
    }
}

} // namespace apex

#endif // APEXVELOCITY_CONFIG_MANAGER_H





