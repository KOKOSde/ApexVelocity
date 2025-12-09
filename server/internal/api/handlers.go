// Package api provides HTTP handlers for the ApexVelocity REST API.
package api

import (
	"encoding/json"
	"log"
	"net/http"
	"sync"

	"github.com/apexvelocity/server/internal/solver"
)

// AnalyzeRequest represents a request to analyze a path.
type AnalyzeRequest struct {
	Geometry  [][]float64      `json:"geometry"`  // [[lat, lon, elevation], ...]
	Surface   []string         `json:"surface"`   // surface types per point
	Vehicle   string           `json:"vehicle"`   // vehicle preset name
	Condition string           `json:"condition"` // "dry" or "wet"
	Overrides *OverrideConfig  `json:"overrides,omitempty"`
}

// OverrideConfig contains optional configuration overrides.
type OverrideConfig struct {
	GlobalFrictionMultiplier *float64                   `json:"global_friction_multiplier,omitempty"`
	Materials                map[string]MaterialOverride `json:"materials,omitempty"`
}

// MaterialOverride for API.
type MaterialOverride struct {
	MuDry                  *float64 `json:"mu_dry,omitempty"`
	MuWet                  *float64 `json:"mu_wet,omitempty"`
	RollingResistanceCoeff *float64 `json:"rolling_resistance_coeff,omitempty"`
}

// AnalyzeResponse represents the response from analyze.
type AnalyzeResponse struct {
	VelocityProfileMPS  []float64 `json:"velocity_profile_mps"`
	SegmentEnergyJoules []float64 `json:"segment_energy_joules"`
	TotalEnergyKWH      float64   `json:"total_energy_kwh"`
	MaxSpeedMPS         float64   `json:"max_speed_mps"`
	MinSpeedMPS         float64   `json:"min_speed_mps"`
	AvgSpeedMPS         float64   `json:"avg_speed_mps"`
}

// ErrorResponse represents an error response.
type ErrorResponse struct {
	Error string `json:"error"`
}

// Handler holds the API handlers and worker pool.
type Handler struct {
	workerPool chan struct{}
	configMu   sync.RWMutex
}

// NewHandler creates a new API handler with a worker pool.
func NewHandler(maxWorkers int) *Handler {
	return &Handler{
		workerPool: make(chan struct{}, maxWorkers),
	}
}

// acquireWorker blocks until a worker slot is available.
func (h *Handler) acquireWorker() {
	h.workerPool <- struct{}{}
}

// releaseWorker releases a worker slot.
func (h *Handler) releaseWorker() {
	<-h.workerPool
}

// writeJSON writes a JSON response.
func writeJSON(w http.ResponseWriter, status int, v interface{}) {
	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(status)
	if err := json.NewEncoder(w).Encode(v); err != nil {
		log.Printf("Error encoding JSON response: %v", err)
	}
}

// writeError writes an error response.
func writeError(w http.ResponseWriter, status int, message string) {
	writeJSON(w, status, ErrorResponse{Error: message})
}

// HandleConfigReload handles POST /v1/config/reload
func (h *Handler) HandleConfigReload(w http.ResponseWriter, r *http.Request) {
	h.configMu.Lock()
	defer h.configMu.Unlock()

	if err := solver.ReloadConfig(); err != nil {
		writeError(w, http.StatusInternalServerError, "Failed to reload config: "+err.Error())
		return
	}

	writeJSON(w, http.StatusOK, map[string]string{"status": "config reloaded"})
}

// HandleAnalyze handles POST /v1/analyze
func (h *Handler) HandleAnalyze(w http.ResponseWriter, r *http.Request) {
	// Parse request
	var req AnalyzeRequest
	if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
		writeError(w, http.StatusBadRequest, "Invalid JSON: "+err.Error())
		return
	}

	// Validate request
	if len(req.Geometry) < 2 {
		writeError(w, http.StatusBadRequest, "Need at least 2 geometry points")
		return
	}

	if len(req.Surface) == 0 {
		// Default all to asphalt
		req.Surface = make([]string, len(req.Geometry))
		for i := range req.Surface {
			req.Surface[i] = "asphalt"
		}
	}

	if len(req.Surface) != len(req.Geometry) {
		writeError(w, http.StatusBadRequest, "Surface array must match geometry array length")
		return
	}

	if req.Condition == "" {
		req.Condition = "dry"
	}

	// Acquire worker slot
	h.acquireWorker()
	defer h.releaseWorker()

	// Apply overrides if present
	h.configMu.Lock()
	if req.Overrides != nil {
		if req.Overrides.GlobalFrictionMultiplier != nil {
			solver.SetFrictionMultiplier(*req.Overrides.GlobalFrictionMultiplier)
		}
		if len(req.Overrides.Materials) > 0 {
			overrides := make([]solver.MaterialOverride, 0, len(req.Overrides.Materials))
			for name, mat := range req.Overrides.Materials {
				override := solver.MaterialOverride{MaterialName: name}
				if mat.MuDry != nil {
					override.MuDry = *mat.MuDry
				} else {
					override.MuDry = 0.9
				}
				if mat.MuWet != nil {
					override.MuWet = *mat.MuWet
				} else {
					override.MuWet = 0.6
				}
				if mat.RollingResistanceCoeff != nil {
					override.RollingResistanceCoeff = *mat.RollingResistanceCoeff
				} else {
					override.RollingResistanceCoeff = 0.015
				}
				overrides = append(overrides, override)
			}
			solver.SetMaterialOverrides(overrides)
		}
	}
	h.configMu.Unlock()

	// Ensure overrides are cleared after solve
	defer func() {
		h.configMu.Lock()
		solver.ClearOverrides()
		h.configMu.Unlock()
	}()

	// Load vehicle
	var vehicle *solver.VehicleParams
	if req.Vehicle == "" || req.Vehicle == "default" {
		vehicle = solver.GetDefaultVehicle()
	} else {
		var err error
		vehicle, err = solver.LoadVehiclePreset(req.Vehicle)
		if err != nil {
			writeError(w, http.StatusBadRequest, "Unknown vehicle: "+req.Vehicle)
			return
		}
	}

	// Convert geometry to path points
	points := make([]solver.PathPoint, len(req.Geometry))
	var totalDistance float64 = 0

	for i, geo := range req.Geometry {
		if len(geo) < 3 {
			writeError(w, http.StatusBadRequest, "Each geometry point needs [lat, lon, elevation]")
			return
		}

		// Convert lat/lon to local coordinates (simplified: use as x,y directly)
		// In production, you'd use proper projection
		points[i] = solver.PathPoint{
			X:           geo[1], // lon as x
			Y:           geo[0], // lat as y
			Z:           geo[2], // elevation
			Curvature:   0,      // Calculate from geometry
			DistanceM:   totalDistance,
			SurfaceType: req.Surface[i],
		}

		if i > 0 {
			// Simple distance calculation (would use haversine in production)
			dx := points[i].X - points[i-1].X
			dy := points[i].Y - points[i-1].Y
			dz := points[i].Z - points[i-1].Z
			// Scale lat/lon to approximate meters (very rough)
			dx *= 111320 // degrees to meters at equator
			dy *= 110540
			dist := (dx*dx + dy*dy + dz*dz)
			if dist > 0 {
				dist = sqrtFloat64(dist)
			}
			totalDistance += dist
			points[i].DistanceM = totalDistance
		}
	}

	// Calculate curvature (simplified: 3-point estimation)
	for i := 1; i < len(points)-1; i++ {
		points[i].Curvature = estimateCurvature(
			points[i-1].X, points[i-1].Y,
			points[i].X, points[i].Y,
			points[i+1].X, points[i+1].Y,
		)
	}

	// Solve
	config := &solver.SolverConfig{
		Condition:            req.Condition,
		EnableRolloverChecks: true,
		EnablePowerLimit:     true,
		MinSpeedMPS:          1.0,
	}

	result, err := solver.Solve(vehicle, config, points)
	if err != nil {
		writeError(w, http.StatusInternalServerError, "Solver error: "+err.Error())
		return
	}

	// Build response
	resp := AnalyzeResponse{
		VelocityProfileMPS:  result.VelocityProfileMPS,
		SegmentEnergyJoules: result.SegmentEnergyJ,
		TotalEnergyKWH:      result.TotalEnergyKWH,
		MaxSpeedMPS:         result.MaxSpeedMPS,
		MinSpeedMPS:         result.MinSpeedMPS,
		AvgSpeedMPS:         result.AvgSpeedMPS,
	}

	writeJSON(w, http.StatusOK, resp)
}

// HandleHealth handles GET /health
func (h *Handler) HandleHealth(w http.ResponseWriter, r *http.Request) {
	writeJSON(w, http.StatusOK, map[string]string{"status": "healthy"})
}

// sqrtFloat64 calculates square root.
func sqrtFloat64(x float64) float64 {
	if x <= 0 {
		return 0
	}
	z := x / 2
	for i := 0; i < 10; i++ {
		z = z - (z*z-x)/(2*z)
	}
	return z
}

// estimateCurvature estimates curvature at point 2 given 3 consecutive points.
func estimateCurvature(x1, y1, x2, y2, x3, y3 float64) float64 {
	// Using Menger curvature formula
	// κ = 4 * area / (|P1-P2| * |P2-P3| * |P3-P1|)
	
	// Area using cross product
	area := ((x2-x1)*(y3-y1) - (x3-x1)*(y2-y1)) / 2
	if area < 0 {
		area = -area
	}
	
	// Side lengths
	d12 := sqrtFloat64((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1))
	d23 := sqrtFloat64((x3-x2)*(x3-x2) + (y3-y2)*(y3-y2))
	d31 := sqrtFloat64((x1-x3)*(x1-x3) + (y1-y3)*(y1-y3))
	
	denom := d12 * d23 * d31
	if denom < 1e-10 {
		return 0
	}
	
	return 4 * area / denom
}





