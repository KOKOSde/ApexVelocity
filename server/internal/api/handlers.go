// Package api provides HTTP handlers for the ApexVelocity REST API.
package api

import (
	"encoding/json"
	"fmt"
	"net/http"
	"os"
	"strings"
	"sync"
	"time"

	"github.com/apexvelocity/server/internal/solver"
	"github.com/google/uuid"
	"github.com/prometheus/client_golang/prometheus"
	"github.com/prometheus/client_golang/prometheus/promauto"
	"github.com/prometheus/client_golang/prometheus/promhttp"
	"go.uber.org/zap"
	"go.uber.org/zap/zapcore"
	"golang.org/x/crypto/bcrypt"
	"golang.org/x/time/rate"
	"gopkg.in/yaml.v3"
)

var logger *zap.Logger

var (
	httpRequestsTotal = promauto.NewCounterVec(
		prometheus.CounterOpts{
			Name: "apexvelocity_http_requests_total",
			Help: "Total HTTP requests",
		},
		[]string{"method", "endpoint", "status"},
	)

	httpRequestDuration = promauto.NewHistogramVec(
		prometheus.HistogramOpts{
			Name:    "apexvelocity_http_request_duration_seconds",
			Help:    "HTTP request duration",
			Buckets: prometheus.DefBuckets,
		},
		[]string{"method", "endpoint"},
	)

	solverDuration = promauto.NewHistogram(
		prometheus.HistogramOpts{
			Name:    "apexvelocity_solver_duration_seconds",
			Help:    "Solver execution duration",
			Buckets: []float64{0.1, 0.5, 1, 2, 5, 10},
		},
	)

	solverPointsProcessed = promauto.NewHistogram(
		prometheus.HistogramOpts{
			Name:    "apexvelocity_solver_points_processed",
			Help:    "Number of points processed by solver",
			Buckets: []float64{100, 500, 1000, 5000, 10000},
		},
	)
)

func init() {
	config := zap.NewProductionConfig()
	config.EncoderConfig.TimeKey = "timestamp"
	config.EncoderConfig.EncodeTime = zapcore.ISO8601TimeEncoder

	l, err := config.Build()
	if err != nil {
		logger = zap.NewNop()
		return
	}
	logger = l
}

// APIKey represents a single API key configuration entry.
type APIKey struct {
	Name      string `yaml:"name"`
	Hash      string `yaml:"hash"`
	RateLimit int    `yaml:"rate_limit"` // requests per minute
}

// AuthConfig is loaded from config/auth.yaml.
type AuthConfig struct {
	APIKeys      []APIKey `yaml:"api_keys"`
	AdminKeys    []APIKey `yaml:"admin_keys"`
	AuthDisabled bool     `yaml:"auth_disabled"`
}

// AuthMiddleware holds API keys and rate limiters.
type AuthMiddleware struct {
	keys     []*APIKey
	limiters map[string]*rate.Limiter
	disabled bool
}

// statusRecorder wraps an http.ResponseWriter and records the final status code.
type statusRecorder struct {
	http.ResponseWriter
	status int
}

func (r *statusRecorder) WriteHeader(code int) {
	r.status = code
	r.ResponseWriter.WriteHeader(code)
}

// NewAuthMiddleware loads API key configuration from the given path.
// If the file is missing or invalid, auth is effectively disabled but
// the server will still start.
func NewAuthMiddleware(configPath string) *AuthMiddleware {
	mw := &AuthMiddleware{
		keys:     []*APIKey{},
		limiters: map[string]*rate.Limiter{},
		disabled: true,
	}

	data, err := os.ReadFile(configPath)
	if err != nil {
		if logger != nil {
			logger.Warn("auth_config_not_found",
				zap.String("path", configPath),
				zap.Error(err),
			)
		}
		return mw
	}

	var cfg AuthConfig
	if err := yaml.Unmarshal(data, &cfg); err != nil {
		if logger != nil {
			logger.Error("auth_config_unmarshal_error",
				zap.String("path", configPath),
				zap.Error(err),
			)
		}
		return mw
	}

	// Populate keys and limiters
	for i := range cfg.APIKeys {
		key := cfg.APIKeys[i]
		mw.keys = append(mw.keys, &key)
		if key.RateLimit > 0 {
			// Convert "requests per minute" into a rate limiter.
			limit := rate.Every(time.Minute / time.Duration(key.RateLimit))
			mw.limiters[key.Name] = rate.NewLimiter(limit, key.RateLimit)
		}
	}

	for i := range cfg.AdminKeys {
		key := cfg.AdminKeys[i]
		mw.keys = append(mw.keys, &key)
		// Admin keys are not rate-limited by default.
	}

	mw.disabled = cfg.AuthDisabled

	if logger != nil {
		logger.Info("auth_config_loaded",
			zap.Int("api_keys", len(cfg.APIKeys)),
			zap.Int("admin_keys", len(cfg.AdminKeys)),
			zap.Bool("auth_disabled", cfg.AuthDisabled),
		)
	}

	return mw
}

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
	auth       *AuthMiddleware
}

// NewHandler creates a new API handler with a worker pool and optional auth.
func NewHandler(maxWorkers int, configDir string) *Handler {
	h := &Handler{
		workerPool: make(chan struct{}, maxWorkers),
	}

	// Initialize auth middleware from config/auth.yaml, but do not fail hard
	// if the file is missing or invalid.
	authConfigPath := configDir + "/auth.yaml"
	h.auth = NewAuthMiddleware(authConfigPath)

	return h
}

// AuthMiddleware returns the auth middleware function. If auth is disabled
// or not configured, this returns a no-op middleware.
func (h *Handler) AuthMiddleware() func(http.Handler) http.Handler {
	if h.auth == nil {
		return func(next http.Handler) http.Handler { return next }
	}
	return h.auth.Middleware
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
		if logger != nil {
			logger.Error("encode_response_error", zap.Error(err))
		}
	}
}

// writeError writes an error response.
func writeError(w http.ResponseWriter, status int, message string) {
	if logger != nil {
		logger.Warn("request_error",
			zap.Int("status", status),
			zap.String("message", message),
		)
	}
	writeJSON(w, status, ErrorResponse{Error: message})
}

// LoggingMiddleware logs structured request information for each HTTP call.
func LoggingMiddleware(next http.Handler) http.Handler {
	return http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		start := time.Now()
		requestID := uuid.New().String()

		// Wrap ResponseWriter to capture status code for metrics.
		rec := &statusRecorder{
			ResponseWriter: w,
			status:         http.StatusOK,
		}

		if logger != nil {
			logger.Info("incoming_request",
				zap.String("request_id", requestID),
				zap.String("method", r.Method),
				zap.String("path", r.URL.Path),
				zap.String("remote_addr", r.RemoteAddr),
			)
		}

		next.ServeHTTP(rec, r)

		duration := time.Since(start)

		if logger != nil {
			logger.Info("request_completed",
				zap.String("request_id", requestID),
				zap.Duration("duration_ms", duration),
				zap.Int("status", rec.status),
			)
		}

		// Record Prometheus metrics
		method := r.Method
		path := r.URL.Path
		statusLabel := fmt.Sprintf("%d", rec.status)

		httpRequestsTotal.WithLabelValues(method, path, statusLabel).Inc()
		httpRequestDuration.WithLabelValues(method, path).Observe(duration.Seconds())
	})
}

// SecurityHeadersMiddleware adds common security headers to HTTP responses.
func SecurityHeadersMiddleware(next http.Handler) http.Handler {
	return http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		w.Header().Set("X-Content-Type-Options", "nosniff")
		w.Header().Set("X-Frame-Options", "DENY")
		w.Header().Set("X-XSS-Protection", "1; mode=block")
		w.Header().Set("Strict-Transport-Security", "max-age=31536000; includeSubDomains")
		w.Header().Set("Content-Security-Policy", "default-src 'self'")
		next.ServeHTTP(w, r)
	})
}

// MetricsHandler exposes Prometheus metrics on /metrics.
func MetricsHandler() http.Handler {
	return promhttp.Handler()
}

// Middleware enforces API key authentication and per-key rate limiting.
func (auth *AuthMiddleware) Middleware(next http.Handler) http.Handler {
	return http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		// Allow disabling auth via environment or config.
		if auth == nil || auth.disabled || os.Getenv("AUTH_DISABLED") == "true" {
			next.ServeHTTP(w, r)
			return
		}

		authHeader := r.Header.Get("Authorization")
		if authHeader == "" {
			writeError(w, http.StatusUnauthorized, "Missing Authorization header")
			return
		}

		token := strings.TrimSpace(strings.TrimPrefix(authHeader, "Bearer "))
		if token == "" {
			writeError(w, http.StatusUnauthorized, "Missing API token")
			return
		}

		var matchedKey *APIKey
		for _, key := range auth.keys {
			if bcrypt.CompareHashAndPassword([]byte(key.Hash), []byte(token)) == nil {
				matchedKey = key
				break
			}
		}

		if matchedKey == nil {
			writeError(w, http.StatusForbidden, "Invalid API key")
			return
		}

		// Enforce rate limit for this key, if configured.
		if limiter, ok := auth.limiters[matchedKey.Name]; ok {
			if !limiter.Allow() {
				w.Header().Set("Retry-After", "60")
				writeError(w, http.StatusTooManyRequests, "Rate limit exceeded")
				return
			}
		}

		next.ServeHTTP(w, r)
	})
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

// HandleRateLimit returns basic rate limit information for the current API key.
// It uses the same Authorization header as other authenticated endpoints.
func (h *Handler) HandleRateLimit(w http.ResponseWriter, r *http.Request) {
	if h.auth == nil || h.auth.disabled || os.Getenv("AUTH_DISABLED") == "true" {
		writeJSON(w, http.StatusOK, map[string]string{"status": "auth_disabled"})
		return
	}

	authHeader := r.Header.Get("Authorization")
	if authHeader == "" {
		writeError(w, http.StatusUnauthorized, "Missing Authorization header")
		return
	}

	token := strings.TrimSpace(strings.TrimPrefix(authHeader, "Bearer "))
	if token == "" {
		writeError(w, http.StatusUnauthorized, "Missing API token")
		return
	}

	var matchedKey *APIKey
	for _, key := range h.auth.keys {
		if bcrypt.CompareHashAndPassword([]byte(key.Hash), []byte(token)) == nil {
			matchedKey = key
			break
		}
	}

	if matchedKey == nil {
		writeError(w, http.StatusForbidden, "Invalid API key")
		return
	}

	resp := map[string]interface{}{
		"key":                  matchedKey.Name,
		"rate_limit_per_minute": matchedKey.RateLimit,
	}
	writeJSON(w, http.StatusOK, resp)
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





