// Package solver provides CGO bindings to the ApexVelocity C++ core library.
package solver

/*
#cgo CFLAGS: -I${SRCDIR}/../../../core/include
#cgo LDFLAGS: -L${SRCDIR}/../../../build/core -lapexvelocity -lstdc++ -lstdc++fs -lm -lyaml-cpp
#cgo LDFLAGS: -L${SRCDIR}/../../../build/_deps/yaml-cpp-build
#cgo LDFLAGS: -Wl,-rpath,${SRCDIR}/../../../build/core
#cgo LDFLAGS: -Wl,-rpath,${SRCDIR}/../../../build/_deps/yaml-cpp-build

#include <stdlib.h>
#include "capi/apex_c_api.h"
*/
import "C"
import (
	"errors"
	"unsafe"
)

// VehicleParams represents vehicle physical parameters.
type VehicleParams struct {
	MassKg              float64 `json:"mass_kg"`
	DragCoeff           float64 `json:"drag_coeff"`
	FrontalAreaM2       float64 `json:"frontal_area_m2"`
	RollingResBase      float64 `json:"rolling_res_base"`
	MaxLatG             float64 `json:"max_lat_g"`
	MaxBrakeG           float64 `json:"max_brake_g"`
	MaxPowerW           float64 `json:"max_power_w"`
	PowertrainEfficiency float64 `json:"powertrain_efficiency"`
	TrackWidthM         float64 `json:"track_width_m"`
	CogHeightM          float64 `json:"cog_height_m"`
	Name                string  `json:"name"`
}

// PathPoint represents a point along a path.
type PathPoint struct {
	X           float64 `json:"x"`
	Y           float64 `json:"y"`
	Z           float64 `json:"z"`
	Curvature   float64 `json:"curvature"`
	DistanceM   float64 `json:"distance_m"`
	SurfaceType string  `json:"surface_type"`
}

// SolverConfig contains solver configuration options.
type SolverConfig struct {
	Condition           string  `json:"condition"`
	EnableRolloverChecks bool    `json:"enable_rollover_checks"`
	EnablePowerLimit    bool    `json:"enable_power_limit"`
	MinSpeedMPS         float64 `json:"min_speed_mps"`
	InitialSpeedMPS     float64 `json:"initial_speed_mps"`
	FinalSpeedMPS       float64 `json:"final_speed_mps"`
}

// SolveResult contains the results of a solve operation.
type SolveResult struct {
	Success            bool      `json:"success"`
	ErrorMessage       string    `json:"error_message,omitempty"`
	VelocityProfileMPS []float64 `json:"velocity_profile_mps"`
	EnergyJoules       []float64 `json:"energy_joules"`
	SegmentEnergyJ     []float64 `json:"segment_energy_joules"`
	TotalEnergyJ       float64   `json:"total_energy_joules"`
	TotalEnergyKWH     float64   `json:"total_energy_kwh"`
	MaxSpeedMPS        float64   `json:"max_speed_mps"`
	MinSpeedMPS        float64   `json:"min_speed_mps"`
	AvgSpeedMPS        float64   `json:"avg_speed_mps"`
}

// MaterialOverride represents an override for material properties.
type MaterialOverride struct {
	MaterialName           string  `json:"material_name"`
	MuDry                  float64 `json:"mu_dry"`
	MuWet                  float64 `json:"mu_wet"`
	RollingResistanceCoeff float64 `json:"rolling_resistance_coeff"`
}

// InitConfig initializes the configuration manager.
func InitConfig(configDir string) error {
	cDir := C.CString(configDir)
	defer C.free(unsafe.Pointer(cDir))
	
	ret := C.apex_config_init(cDir)
	if ret != 0 {
		return errors.New("failed to initialize config")
	}
	return nil
}

// ReloadConfig reloads configuration files.
func ReloadConfig() error {
	ret := C.apex_config_reload()
	if ret != 0 {
		return errors.New("failed to reload config")
	}
	return nil
}

// SetFrictionMultiplier sets a global friction multiplier.
func SetFrictionMultiplier(multiplier float64) {
	C.apex_config_set_friction_multiplier(C.double(multiplier))
}

// SetMaterialOverrides applies temporary material overrides.
func SetMaterialOverrides(overrides []MaterialOverride) {
	if len(overrides) == 0 {
		return
	}
	
	cOverrides := make([]C.ApexMaterialOverride, len(overrides))
	cStrings := make([]*C.char, len(overrides))
	
	for i, o := range overrides {
		cStrings[i] = C.CString(o.MaterialName)
		cOverrides[i].material_name = cStrings[i]
		cOverrides[i].mu_dry = C.double(o.MuDry)
		cOverrides[i].mu_wet = C.double(o.MuWet)
		cOverrides[i].rolling_resistance_coeff = C.double(o.RollingResistanceCoeff)
	}
	
	C.apex_config_set_material_overrides(&cOverrides[0], C.size_t(len(overrides)))
	
	for _, s := range cStrings {
		C.free(unsafe.Pointer(s))
	}
}

// ClearOverrides clears all temporary overrides.
func ClearOverrides() {
	C.apex_config_clear_overrides()
}

// LoadVehiclePreset loads a vehicle preset by name.
func LoadVehiclePreset(name string) (*VehicleParams, error) {
	cName := C.CString(name)
	defer C.free(unsafe.Pointer(cName))
	
	var params C.ApexVehicleParams
	ret := C.apex_vehicle_load_preset(cName, &params)
	if ret != 0 {
		return nil, errors.New("vehicle preset not found: " + name)
	}
	
	return &VehicleParams{
		MassKg:              float64(params.mass_kg),
		DragCoeff:           float64(params.drag_coeff),
		FrontalAreaM2:       float64(params.frontal_area_m2),
		RollingResBase:      float64(params.rolling_res_base),
		MaxLatG:             float64(params.max_lat_g),
		MaxBrakeG:           float64(params.max_brake_g),
		MaxPowerW:           float64(params.max_power_w),
		PowertrainEfficiency: float64(params.powertrain_efficiency),
		TrackWidthM:         float64(params.track_width_m),
		CogHeightM:          float64(params.cog_height_m),
		Name:                name,
	}, nil
}

// GetDefaultVehicle returns default vehicle parameters.
func GetDefaultVehicle() *VehicleParams {
	var params C.ApexVehicleParams
	C.apex_vehicle_get_default(&params)
	
	return &VehicleParams{
		MassKg:              float64(params.mass_kg),
		DragCoeff:           float64(params.drag_coeff),
		FrontalAreaM2:       float64(params.frontal_area_m2),
		RollingResBase:      float64(params.rolling_res_base),
		MaxLatG:             float64(params.max_lat_g),
		MaxBrakeG:           float64(params.max_brake_g),
		MaxPowerW:           float64(params.max_power_w),
		PowertrainEfficiency: float64(params.powertrain_efficiency),
		TrackWidthM:         float64(params.track_width_m),
		CogHeightM:          float64(params.cog_height_m),
		Name:                "default",
	}
}

// Solve performs velocity profile solving.
func Solve(vehicle *VehicleParams, config *SolverConfig, points []PathPoint) (*SolveResult, error) {
	if len(points) < 2 {
		return nil, errors.New("need at least 2 points")
	}
	
	// Convert vehicle params
	var cVehicle C.ApexVehicleParams
	cVehicle.mass_kg = C.double(vehicle.MassKg)
	cVehicle.drag_coeff = C.double(vehicle.DragCoeff)
	cVehicle.frontal_area_m2 = C.double(vehicle.FrontalAreaM2)
	cVehicle.rolling_res_base = C.double(vehicle.RollingResBase)
	cVehicle.max_lat_g = C.double(vehicle.MaxLatG)
	cVehicle.max_brake_g = C.double(vehicle.MaxBrakeG)
	cVehicle.max_power_w = C.double(vehicle.MaxPowerW)
	cVehicle.powertrain_efficiency = C.double(vehicle.PowertrainEfficiency)
	cVehicle.track_width_m = C.double(vehicle.TrackWidthM)
	cVehicle.cog_height_m = C.double(vehicle.CogHeightM)
	
	// Convert config
	var cConfig C.ApexSolverConfig
	if config != nil {
		cCondition := C.CString(config.Condition)
		defer C.free(unsafe.Pointer(cCondition))
		cConfig.condition = cCondition
		cConfig.enable_rollover_checks = C.bool(config.EnableRolloverChecks)
		cConfig.enable_power_limit = C.bool(config.EnablePowerLimit)
		cConfig.min_speed_mps = C.double(config.MinSpeedMPS)
		cConfig.initial_speed_mps = C.double(config.InitialSpeedMPS)
		cConfig.final_speed_mps = C.double(config.FinalSpeedMPS)
	} else {
		cConfig.condition = C.CString("dry")
		cConfig.enable_rollover_checks = C.bool(true)
		cConfig.enable_power_limit = C.bool(true)
		cConfig.min_speed_mps = C.double(1.0)
	}
	
	// Convert path points
	cPoints := make([]C.ApexPathPoint, len(points))
	surfaceStrings := make([]*C.char, len(points))
	
	for i, pt := range points {
		surfaceStrings[i] = C.CString(pt.SurfaceType)
		cPoints[i].x_m = C.double(pt.X)
		cPoints[i].y_m = C.double(pt.Y)
		cPoints[i].z_m = C.double(pt.Z)
		cPoints[i].curvature = C.double(pt.Curvature)
		cPoints[i].distance_along_m = C.double(pt.DistanceM)
		cPoints[i].surface_type = surfaceStrings[i]
	}
	
	// Call solver
	result := C.apex_solve(&cVehicle, &cConfig, &cPoints[0], C.size_t(len(points)))
	
	// Clean up strings
	for _, s := range surfaceStrings {
		C.free(unsafe.Pointer(s))
	}
	
	// Convert result
	solveResult := &SolveResult{
		Success:       bool(result.success),
		TotalEnergyJ:  float64(result.total_energy_joules),
		TotalEnergyKWH: float64(result.total_energy_joules) / 3600000.0,
		MaxSpeedMPS:   float64(result.max_speed_mps),
		MinSpeedMPS:   float64(result.min_speed_mps),
		AvgSpeedMPS:   float64(result.avg_speed_mps),
	}
	
	if result.error_message != nil {
		solveResult.ErrorMessage = C.GoString(result.error_message)
	}
	
	if result.success {
		numPoints := int(result.num_points)
		solveResult.VelocityProfileMPS = make([]float64, numPoints)
		solveResult.EnergyJoules = make([]float64, numPoints)
		
		velocities := (*[1 << 30]C.double)(unsafe.Pointer(result.velocity_profile_mps))[:numPoints:numPoints]
		energies := (*[1 << 30]C.double)(unsafe.Pointer(result.energy_joules))[:numPoints:numPoints]
		
		for i := 0; i < numPoints; i++ {
			solveResult.VelocityProfileMPS[i] = float64(velocities[i])
			solveResult.EnergyJoules[i] = float64(energies[i])
		}
		
		if numPoints > 1 {
			numSegments := numPoints - 1
			solveResult.SegmentEnergyJ = make([]float64, numSegments)
			segments := (*[1 << 30]C.double)(unsafe.Pointer(result.segment_energy_joules))[:numSegments:numSegments]
			for i := 0; i < numSegments; i++ {
				solveResult.SegmentEnergyJ[i] = float64(segments[i])
			}
		}
	}
	
	// Free C result
	C.apex_result_free(&result)
	
	if !solveResult.Success {
		return nil, errors.New(solveResult.ErrorMessage)
	}
	
	return solveResult, nil
}

// GetEffectiveMu returns the effective friction coefficient.
func GetEffectiveMu(materialName, condition string) float64 {
	cMat := C.CString(materialName)
	cCond := C.CString(condition)
	defer C.free(unsafe.Pointer(cMat))
	defer C.free(unsafe.Pointer(cCond))
	
	return float64(C.apex_get_effective_mu(cMat, cCond))
}





