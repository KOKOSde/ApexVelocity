#include "physics/VehicleLoader.h"

#include <nlohmann/json.hpp>
#include <fstream>
#include <filesystem>
#include <iostream>

namespace apex {
namespace physics {

VehicleLoader::VehicleLoader(const std::string& presets_dir)
    : presets_dir_(presets_dir) {
}

std::optional<VehicleParams> VehicleLoader::load_preset(const std::string& filename) const {
    std::filesystem::path filepath = std::filesystem::path(presets_dir_) / filename;
    
    std::ifstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "VehicleLoader: Could not open " << filepath << std::endl;
        return std::nullopt;
    }
    
    try {
        nlohmann::json j;
        file >> j;
        
        VehicleParams params;
        params.name = j.value("name", filename);
        params.mass_kg = j.value("mass_kg", 1500.0);
        params.drag_coeff = j.value("drag_coeff", 0.30);
        params.frontal_area_m2 = j.value("frontal_area_m2", 2.2);
        params.rolling_res_base = j.value("rolling_res_base", 0.015);
        params.max_lat_g = j.value("max_lat_g", 0.8);
        params.max_brake_g = j.value("max_brake_g", 0.9);
        params.max_power_w = j.value("max_power_w", 150000.0);
        params.powertrain_efficiency = j.value("powertrain_efficiency", 0.90);
        params.track_width_m = j.value("track_width_m", 1.6);
        params.cog_height_m = j.value("cog_height_m", 0.5);
        
        if (!params.is_valid()) {
            std::cerr << "VehicleLoader: Invalid parameters in " << filepath << std::endl;
            return std::nullopt;
        }
        
        return params;
        
    } catch (const nlohmann::json::exception& e) {
        std::cerr << "VehicleLoader: JSON parse error in " << filepath << ": " << e.what() << std::endl;
        return std::nullopt;
    }
}

std::unordered_map<std::string, VehicleParams> VehicleLoader::load_all_presets() const {
    std::unordered_map<std::string, VehicleParams> presets;
    
    for (const auto& filename : list_presets()) {
        auto params = load_preset(filename);
        if (params) {
            presets[params->name] = *params;
        }
    }
    
    return presets;
}

std::vector<std::string> VehicleLoader::list_presets() const {
    std::vector<std::string> presets;
    
    try {
        for (const auto& entry : std::filesystem::directory_iterator(presets_dir_)) {
            if (entry.path().extension() == ".json") {
                presets.push_back(entry.path().filename().string());
            }
        }
    } catch (const std::filesystem::filesystem_error& e) {
        std::cerr << "VehicleLoader: Could not list presets directory: " << e.what() << std::endl;
    }
    
    return presets;
}

std::optional<VehicleParams> VehicleLoader::from_json_string(const std::string& json_str) {
    try {
        nlohmann::json j = nlohmann::json::parse(json_str);
        
        VehicleParams params;
        params.name = j.value("name", "custom");
        params.mass_kg = j.value("mass_kg", 1500.0);
        params.drag_coeff = j.value("drag_coeff", 0.30);
        params.frontal_area_m2 = j.value("frontal_area_m2", 2.2);
        params.rolling_res_base = j.value("rolling_res_base", 0.015);
        params.max_lat_g = j.value("max_lat_g", 0.8);
        params.max_brake_g = j.value("max_brake_g", 0.9);
        params.max_power_w = j.value("max_power_w", 150000.0);
        params.powertrain_efficiency = j.value("powertrain_efficiency", 0.90);
        params.track_width_m = j.value("track_width_m", 1.6);
        params.cog_height_m = j.value("cog_height_m", 0.5);
        
        if (!params.is_valid()) {
            return std::nullopt;
        }
        
        return params;
        
    } catch (const nlohmann::json::exception&) {
        return std::nullopt;
    }
}

VehicleParams VehicleLoader::get_default() {
    VehicleParams params;
    params.name = "default_sedan";
    params.mass_kg = 1500.0;
    params.drag_coeff = 0.30;
    params.frontal_area_m2 = 2.2;
    params.rolling_res_base = 0.015;
    params.max_lat_g = 0.8;
    params.max_brake_g = 0.9;
    params.max_power_w = 150000.0;
    params.powertrain_efficiency = 0.90;
    params.track_width_m = 1.6;
    params.cog_height_m = 0.5;
    return params;
}

} // namespace physics
} // namespace apex





