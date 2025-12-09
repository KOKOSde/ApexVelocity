# ApexVelocity Metrics Reference

This document provides a comprehensive reference for all metrics computed by ApexVelocity, designed for AV/robotaxi engineers, researchers, and benchmarking.

## Per-Segment Features

These features are computed for each path segment and are available in the `SegmentFeatures` dataclass.

### Geometry & Semantics

| Name | Units | Category | Use | Description |
|------|-------|----------|-----|-------------|
| `curvature_1pm` | 1/m | Geometry | Both | Path curvature (κ). Higher values = tighter turns. |
| `curve_radius_m` | m | Geometry | Both | Curve radius (1/κ). Infinity for straight segments. |
| `dcurvature_ds` | 1/m² | Geometry | Training | Rate of change of curvature along path. High values indicate clothoid transitions. |
| `grade_percent` | % | Geometry | Both | Road gradient. Positive = uphill, negative = downhill. |
| `elevation_m` | m | Geometry | Both | Elevation above sea level. |
| `turn_type` | string | Geometry | Training | Turn classification: straight, left, right, sharp_left, sharp_right, u_turn. |
| `turn_angle_deg` | degrees | Geometry | Training | Heading change over segment. Positive = left, negative = right. |
| `road_type` | string | Semantics | Both | OSM highway type: motorway, primary, secondary, residential, etc. |
| `speed_limit_kmh` | km/h | Semantics | Both | Posted speed limit from OSM (if available). |
| `lane_count` | int | Semantics | Both | Number of lanes from OSM (if available). |
| `surface_type` | string | Semantics | Both | Road surface: asphalt, concrete, gravel, cobblestone, etc. |
| `is_intersection_segment` | bool | Semantics | Training | True if segment is within an intersection. |
| `distance_to_next_intersection_m` | m | Semantics | Training | Distance to next intersection (lookahead). |

### Speed & Velocity

| Name | Units | Category | Use | Description |
|------|-------|----------|-----|-------------|
| `speed_mps` | m/s | Speed | Both | Current speed in meters per second. |
| `speed_kmh` | km/h | Speed | Both | Current speed in kilometers per hour. |
| `physics_limit_speed_kmh` | km/h | Speed | Both | Maximum friction-limited speed based on curvature and μ. |
| `speed_ratio` | [0-1] | Speed | Training | Fraction of physics limit used (speed / limit). |
| `speed_limit_margin_kmh` | km/h | Speed | Training | Posted limit minus current speed. Positive = under limit. |
| `target_speed_kmh` | km/h | Speed | Training | Comfort-based recommended speed (accounts for jerk limits). |

### Dynamics & Acceleration

| Name | Units | Category | Use | Description |
|------|-------|----------|-----|-------------|
| `lateral_accel_mps2` | m/s² | Dynamics | Both | Lateral (centripetal) acceleration. |
| `lateral_g` | g | Dynamics | Both | Lateral acceleration in g-units (common metric). |
| `longitudinal_accel_mps2` | m/s² | Dynamics | Both | Longitudinal acceleration. Positive = accelerating. |
| `longitudinal_g` | g | Dynamics | Both | Longitudinal acceleration in g-units. |
| `total_accel_g` | g | Dynamics | Both | Total acceleration magnitude: √(lat² + long²). |
| `longitudinal_jerk_mps3` | m/s³ | Dynamics | Training | Rate of change of longitudinal acceleration. High values = jerky ride. |
| `lateral_jerk_mps3` | m/s³ | Dynamics | Training | Rate of change of lateral acceleration. High values = uncomfortable swaying. |
| `jerk_magnitude_mps3` | m/s³ | Dynamics | Training | Total jerk magnitude: √(lat_jerk² + long_jerk²). |

### Friction & Tire Usage

| Name | Units | Category | Use | Description |
|------|-------|----------|-----|-------------|
| `friction_mu_effective` | [0-1] | Friction | Both | Effective friction coefficient (surface × condition × multiplier). |
| `friction_usage` | [0-1] | Friction | Both | Fraction of available friction used: √(a_lat² + a_long²) / (μg). Values > 0.8 = near limit. |

### Energy & Efficiency

| Name | Units | Category | Use | Description |
|------|-------|----------|-----|-------------|
| `energy_joules` | J | Energy | Benchmark | Cumulative energy from start of path. |
| `energy_kwh` | kWh | Energy | Benchmark | Cumulative energy in kilowatt-hours. |
| `energy_per_meter_j` | J/m | Energy | Both | Energy consumption rate for this segment. |
| `energy_kwh_per_km` | kWh/km | Energy | Both | Energy intensity (efficiency metric). |
| `regen_potential_j` | J | Energy | Training | Potential regenerative energy (nonzero if decelerating + downhill). |
| `coasting_opportunity` | bool | Energy | Training | True if coasting would save energy with minimal time impact. |

### Comfort Metrics

| Name | Units | Category | Use | Description |
|------|-------|----------|-----|-------------|
| `comfort_score` | [0-1] | Comfort | Training | Composite comfort score. 0 = comfortable, 1 = very uncomfortable. Based on lateral G, longitudinal G, and jerk. |
| `is_uncomfortable` | bool | Comfort | Training | True if comfort_score exceeds threshold (0.3). |
| `comfort_violations` | int | Comfort | Benchmark | Count of comfort violations (G or jerk exceeded) in this segment. |

### Maneuver & Action

| Name | Units | Category | Use | Description |
|------|-------|----------|-----|-------------|
| `recommended_action` | string | Action | Training | Recommended driving action: BRAKE, EASE_OFF, COAST, HOLD, ACCELERATE, SMOOTH. |
| `action_confidence` | [0-1] | Action | Training | Confidence in recommended action. Lower values = near decision boundaries. |
| `action_reason` | string | Action | Training | Human-readable explanation for the recommended action. |
| `time_to_next_brake_s` | s | Action | Training | Estimated time until next BRAKE action at current speed. |

---

## Route-Level Metrics

These metrics are computed for the entire route and are available in the `ProfileAnalysis` dataclass.

### Core Metrics

| Name | Units | Category | Use | Description |
|------|-------|----------|-----|-------------|
| `total_distance_km` | km | Core | Both | Total route distance. |
| `total_time_s` | s | Core | Both | Total travel time (lap time). |
| `avg_speed_kmh` | km/h | Speed | Both | Average speed: distance / time. |
| `max_speed_kmh` | km/h | Speed | Both | Maximum speed achieved. |
| `min_speed_kmh` | km/h | Speed | Both | Minimum speed (excluding stops). |
| `p95_speed_kmh` | km/h | Speed | Benchmark | 95th percentile speed. |
| `speed_std_kmh` | km/h | Speed | Benchmark | Speed standard deviation (variability). |

### Dynamics Statistics

| Name | Units | Category | Use | Description |
|------|-------|----------|-----|-------------|
| `max_lateral_g` | g | Dynamics | Both | Maximum lateral acceleration. |
| `avg_lateral_g` | g | Dynamics | Benchmark | Average lateral acceleration. |
| `p95_lateral_g` | g | Dynamics | Both | 95th percentile lateral acceleration. |
| `max_longitudinal_g` | g | Dynamics | Both | Maximum longitudinal acceleration magnitude. |
| `max_total_g` | g | Dynamics | Benchmark | Maximum total acceleration. |
| `p95_longitudinal_jerk` | m/s³ | Dynamics | Benchmark | 95th percentile longitudinal jerk. |
| `p95_lateral_jerk` | m/s³ | Dynamics | Benchmark | 95th percentile lateral jerk. |
| `max_jerk_mps3` | m/s³ | Dynamics | Both | Maximum jerk magnitude. |

### Friction Statistics

| Name | Units | Category | Use | Description |
|------|-------|----------|-----|-------------|
| `max_friction_usage` | [0-1] | Friction | Both | Maximum friction utilization on route. |
| `avg_friction_usage` | [0-1] | Friction | Both | Average friction utilization. |
| `p95_friction_usage` | [0-1] | Friction | Benchmark | 95th percentile friction usage. |

### Energy Statistics

| Name | Units | Category | Use | Description |
|------|-------|----------|-----|-------------|
| `total_energy_kwh` | kWh | Energy | Both | Total energy consumed. |
| `energy_kwh_per_km` | kWh/km | Energy | Both | Energy efficiency (primary efficiency metric). |
| `total_regen_kwh` | kWh | Energy | Benchmark | Total regenerative energy potential. |
| `regen_fraction` | [0-1] | Energy | Benchmark | Regenerated / total energy magnitude. |

### Comfort Statistics

| Name | Units | Category | Use | Description |
|------|-------|----------|-----|-------------|
| `avg_comfort_score` | [0-1] | Comfort | Benchmark | Average comfort score across route. |
| `max_comfort_score` | [0-1] | Comfort | Benchmark | Maximum comfort score (worst comfort). |
| `comfort_violations_per_km` | /km | Comfort | Benchmark | Count of comfort violations per kilometer. Primary comfort metric. |
| `pct_uncomfortable_distance` | [0-1] | Comfort | Benchmark | Fraction of distance with comfort_score > threshold. |

### Action Distribution

| Name | Units | Category | Use | Description |
|------|-------|----------|-----|-------------|
| `brake_fraction` | [0-1] | Action | Both | Fraction of distance with BRAKE/EASE_OFF action. |
| `coast_fraction` | [0-1] | Action | Both | Fraction of distance with COAST action. |
| `accelerate_fraction` | [0-1] | Action | Both | Fraction of distance with ACCELERATE action. |
| `hold_fraction` | [0-1] | Action | Both | Fraction of distance with HOLD action. |

### Geometry Statistics

| Name | Units | Category | Use | Description |
|------|-------|----------|-----|-------------|
| `turns_per_km` | /km | Geometry | Benchmark | Number of significant turns per kilometer. |
| `avg_curvature` | 1/m | Geometry | Benchmark | Average curvature. |
| `max_curvature` | 1/m | Geometry | Benchmark | Maximum curvature. |
| `avg_grade_percent` | % | Geometry | Benchmark | Average absolute grade. |
| `max_grade_percent` | % | Geometry | Benchmark | Maximum absolute grade. |
| `total_elevation_gain_m` | m | Geometry | Benchmark | Total elevation gain. |
| `total_elevation_loss_m` | m | Geometry | Benchmark | Total elevation loss. |
| `intersection_count` | int | Geometry | Benchmark | Number of intersections on route. |

### Composite Scores

| Name | Units | Category | Use | Description |
|------|-------|----------|-----|-------------|
| `difficulty_score` | [0-1] | Composite | Benchmark | Composite difficulty score based on curvature variance, grade, intersections, and comfort violations. Higher = more challenging route. |
| `safety_margin_score` | [0-1] | Composite | Benchmark | Safety margin based on friction usage and lateral G. Higher = more safety margin available. |

---

## Color Modes for Visualization

ApexVelocity supports multiple color modes for path visualization:

| Mode | Metric Used | Color Scale | Description |
|------|-------------|-------------|-------------|
| `speed` | `speed_kmh` | Red→Yellow→Green | Velocity profile (red=slow, green=fast) |
| `energy` | `energy_kwh_per_km` | Green→Yellow→Red | Energy intensity (green=efficient, red=costly) |
| `gforce` | `lateral_g` | Blue→Purple→Red | Lateral G-force (blue=low, red=high) |
| `friction` | `friction_usage` | Green→Yellow→Red | Friction utilization (green=safe, red=limit) |
| `comfort` | `comfort_score` | Green→Yellow→Red | Comfort score (green=comfortable, red=uncomfortable) |
| `difficulty` | Composite | Blue→Purple→Red | Local difficulty (blue=easy, red=challenging) |
| `action` | `recommended_action` | Discrete colors | Recommended action (red=brake, blue=coast, green=accelerate) |

---

## Usage Categories

- **Training**: Features designed for AV training, reinforcement learning, and behavior cloning.
- **Benchmark**: Features designed for fair comparison and benchmarking of different routes or vehicles.
- **Both**: Features useful for both training and benchmarking.

---

## Example: Accessing Features in Python

```python
from apexvelocity.analysis import analyze_profile

# Analyze a path
analysis = analyze_profile(path_points, condition="dry")

# Access per-segment features
for seg in analysis.segments:
    print(f"Speed: {seg.speed_kmh:.1f} km/h")
    print(f"Lateral G: {seg.lateral_g:.3f} g")
    print(f"Comfort: {seg.comfort_score:.2f}")
    print(f"Action: {seg.recommended_action}")

# Access route-level metrics
print(f"Distance: {analysis.total_distance_km:.2f} km")
print(f"Lap time: {analysis.total_time_s:.1f} s")
print(f"Difficulty: {analysis.difficulty_score:.2f}")

# Get summary as dict
summary = analysis.get_summary_dict()
hud = analysis.get_hud_dict()
```

---

## Notes

- All features are computed in SI units internally (meters, seconds, etc.)
- Percentile calculations (p95) use standard interpolation
- Comfort thresholds are based on ISO 2631 ride quality guidelines
- Friction coefficients are based on tire-road interaction research
- Jerk thresholds are based on passenger comfort studies

---

*See the [README.md](../README.md) for installation and quick start instructions.*





