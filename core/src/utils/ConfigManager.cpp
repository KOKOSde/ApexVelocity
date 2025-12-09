#include "utils/ConfigManager.h"

#include <fstream>
#include <filesystem>
#include <iostream>

namespace apex {

ConfigManager& ConfigManager::instance() {
    static ConfigManager instance;
    return instance;
}

ConfigManager::ConfigManager() 
    : initialized_(false)
    , default_material_(0.9, 0.6, 0.015)  // Asphalt-like defaults
{
}

void ConfigManager::initialize(const std::string& config_dir) {
    if (initialized_) {
        // Allow re-initialization for testing purposes
        materials_.clear();
        tag_to_material_.clear();
    }
    
    config_dir_ = config_dir;
    
    // Build paths
    std::filesystem::path base_path(config_dir_);
    std::string sim_path = (base_path / "simulation.yaml").string();
    std::string mat_path = (base_path / "materials.json").string();
    std::string tag_path = (base_path / "tag_mapping.json").string();
    
    // Load all config files
    load_simulation_config(sim_path);
    load_materials(mat_path);
    load_tag_mapping(tag_path);
    
    initialized_ = true;
}

void ConfigManager::ensure_initialized() const {
    if (!initialized_) {
        // Auto-initialize with default config directory
        const_cast<ConfigManager*>(this)->initialize(APEXVELOCITY_DEFAULT_CONFIG_DIR);
    }
}

void ConfigManager::load_simulation_config(const std::string& path) {
    try {
        sim_config_ = YAML::LoadFile(path);
    } catch (const YAML::Exception& e) {
        throw std::runtime_error("Failed to load simulation config from '" + path + "': " + e.what());
    }
}

void ConfigManager::load_materials(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open materials file: " + path);
    }
    
    try {
        nlohmann::json j;
        file >> j;
        
        for (auto& [name, props] : j.items()) {
            MaterialProps mat;
            mat.mu_dry = props.value("mu_dry", 0.9);
            mat.mu_wet = props.value("mu_wet", 0.6);
            mat.rolling_resistance_coeff = props.value("rolling_resistance_coeff", 0.015);
            materials_[name] = mat;
        }
    } catch (const nlohmann::json::exception& e) {
        throw std::runtime_error("Failed to parse materials JSON from '" + path + "': " + e.what());
    }
}

void ConfigManager::load_tag_mapping(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open tag mapping file: " + path);
    }
    
    try {
        nlohmann::json j;
        file >> j;
        
        for (auto& [tag, material] : j.items()) {
            tag_to_material_[tag] = material.get<std::string>();
        }
    } catch (const nlohmann::json::exception& e) {
        throw std::runtime_error("Failed to parse tag mapping JSON from '" + path + "': " + e.what());
    }
}

const MaterialProps& ConfigManager::get_material(const std::string& material_name) const {
    ensure_initialized();
    
    auto it = materials_.find(material_name);
    if (it != materials_.end()) {
        return it->second;
    }
    
    // Return default material for unknown surfaces
    return default_material_;
}

std::string ConfigManager::get_material_for_tag(const std::string& osm_tag) const {
    ensure_initialized();
    
    auto it = tag_to_material_.find(osm_tag);
    if (it != tag_to_material_.end()) {
        return it->second;
    }
    
    // Default to asphalt for unknown tags
    return "asphalt";
}

double ConfigManager::get_effective_mu(const std::string& material_name,
                                       const std::string& condition) const {
    ensure_initialized();
    
    const MaterialProps& mat = get_material(material_name);
    
    // Get the global friction multiplier from sim config
    double multiplier = 1.0;
    try {
        multiplier = get_sim_param<double>("global_friction_multiplier");
    } catch (...) {
        // Use default multiplier if not found
    }
    
    double base_mu = (condition == "wet") ? mat.mu_wet : mat.mu_dry;
    return base_mu * multiplier;
}

} // namespace apex





