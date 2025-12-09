#ifndef APEXVELOCITY_VEHICLE_LOADER_H
#define APEXVELOCITY_VEHICLE_LOADER_H

#include "physics/Types.h"

#include <string>
#include <unordered_map>
#include <vector>
#include <optional>

namespace apex {
namespace physics {

/**
 * @brief Loads and manages vehicle preset configurations from JSON files.
 * 
 * Reads vehicle parameters from config/vehicle_presets JSON files
 * and provides access to predefined vehicle configurations.
 */
class VehicleLoader {
public:
    /**
     * @brief Construct a VehicleLoader with the path to presets directory.
     * @param presets_dir Path to the vehicle_presets directory.
     */
    explicit VehicleLoader(const std::string& presets_dir);
    
    /**
     * @brief Load a specific vehicle preset from a JSON file.
     * @param filename The JSON filename (e.g., "tesla_model_3.json").
     * @return VehicleParams if successful, nullopt on failure.
     */
    std::optional<VehicleParams> load_preset(const std::string& filename) const;
    
    /**
     * @brief Load all presets from the presets directory.
     * @return Map of vehicle name -> VehicleParams.
     */
    std::unordered_map<std::string, VehicleParams> load_all_presets() const;
    
    /**
     * @brief Get list of available preset filenames.
     * @return Vector of JSON filenames in the presets directory.
     */
    std::vector<std::string> list_presets() const;
    
    /**
     * @brief Load vehicle params from a JSON string.
     * @param json_str The JSON string containing vehicle params.
     * @return VehicleParams if successful, nullopt on failure.
     */
    static std::optional<VehicleParams> from_json_string(const std::string& json_str);
    
    /**
     * @brief Get a default vehicle configuration.
     * @return Default VehicleParams (small sedan).
     */
    static VehicleParams get_default();

private:
    std::string presets_dir_;
};

} // namespace physics
} // namespace apex

#endif // APEXVELOCITY_VEHICLE_LOADER_H

