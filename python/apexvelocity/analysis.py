"""
ApexVelocity Professional Analysis Module.

Computes rich physics features for autonomous vehicle training and benchmarking.
This module provides research-grade metrics including:
- Geometry: curvature, grade, turn types, intersections
- Dynamics: acceleration, jerk, lateral/longitudinal forces
- Comfort: composite scores, target speeds, violation tracking
- Energy: consumption, regeneration potential, efficiency

All features are documented with units and intended uses.
"""

from dataclasses import dataclass, field
from typing import List, Dict, Optional, Tuple, Any, Union
import math

try:
    import numpy as np

    _HAS_NUMPY = True
except ImportError:
    _HAS_NUMPY = False
    np = None


# ==============================================================================
# CONSTANTS
# ==============================================================================

GRAVITY = 9.81  # m/s²

# Comfort thresholds (based on ISO 2631 and ride quality research)
COMFORT_LATERAL_G_THRESHOLD = 0.2  # g - passenger comfort limit
COMFORT_LONGITUDINAL_G_THRESHOLD = 0.25  # g - comfortable braking/accel
COMFORT_JERK_THRESHOLD = 0.5  # m/s³ - comfortable jerk

# Friction utilization thresholds
HIGH_FRICTION_USAGE = 0.7  # 70% of friction limit - caution zone
CRITICAL_FRICTION_USAGE = 0.85  # 85% of friction limit - near tire limit

# Turn classification thresholds (degrees per 100m)
TURN_STRAIGHT_THRESHOLD = 5.0
TURN_SLIGHT_THRESHOLD = 15.0
TURN_NORMAL_THRESHOLD = 45.0
TURN_SHARP_THRESHOLD = 90.0


# ==============================================================================
# SEGMENT FEATURES DATACLASS
# ==============================================================================


@dataclass
class SegmentFeatures:
    """
    Rich physics features for a single path segment.

    All features are documented with units and typical ranges.
    These are designed for AV training and fair benchmarking.
    """

    # -------------------------------------------------------------------------
    # Position & Geometry
    # -------------------------------------------------------------------------
    index: int = 0
    """Segment index (0-based)"""

    lat: float = 0.0
    """Latitude [degrees]"""

    lon: float = 0.0
    """Longitude [degrees]"""

    elevation_m: float = 0.0
    """Elevation above sea level [meters]"""

    distance_along_m: float = 0.0
    """Cumulative distance from start [meters]"""

    segment_length_m: float = 0.0
    """Length of this segment [meters]"""

    curvature_1pm: float = 0.0
    """Curvature [1/meter]. κ = 1/radius"""

    curve_radius_m: float = float("inf")
    """Curve radius [meters]. Inf for straight segments."""

    dcurvature_ds: float = 0.0
    """Rate of change of curvature [1/m²]. High values = clothoid transition."""

    grade_percent: float = 0.0
    """Road grade [%]. Positive = uphill, negative = downhill."""

    # -------------------------------------------------------------------------
    # Semantics (from OSM or inferred)
    # -------------------------------------------------------------------------
    road_type: Optional[str] = None
    """OSM highway type: motorway, primary, secondary, residential, etc."""

    speed_limit_kmh: Optional[float] = None
    """Posted speed limit [km/h]. None if unknown."""

    lane_count: Optional[int] = None
    """Number of lanes. None if unknown."""

    surface_type: str = "asphalt"
    """Road surface: asphalt, concrete, gravel, etc."""

    is_intersection_segment: bool = False
    """True if this segment is within an intersection."""

    distance_to_next_intersection_m: Optional[float] = None
    """Distance to next intersection [meters]. None if unknown."""

    turn_type: str = "straight"
    """Turn classification: straight, left, right, sharp_left, sharp_right, u_turn"""

    turn_angle_deg: float = 0.0
    """Heading change over this segment [degrees]. Signed: + = left, - = right."""

    # -------------------------------------------------------------------------
    # Speed & Velocity
    # -------------------------------------------------------------------------
    speed_mps: float = 0.0
    """Current speed [m/s]"""

    speed_kmh: float = 0.0
    """Current speed [km/h]"""

    speed_limit_margin_kmh: Optional[float] = None
    """Speed limit minus current speed [km/h]. Positive = under limit."""

    physics_limit_speed_mps: float = 0.0
    """Maximum friction-limited speed [m/s]"""

    physics_limit_speed_kmh: float = 0.0
    """Maximum friction-limited speed [km/h]"""

    speed_ratio: float = 0.0
    """Fraction of physics limit used [0-1]. speed / physics_limit"""

    target_speed_kmh: float = 0.0
    """Comfort-based recommended speed [km/h]. Accounts for jerk limits."""

    # -------------------------------------------------------------------------
    # Dynamics & Acceleration
    # -------------------------------------------------------------------------
    lateral_accel_mps2: float = 0.0
    """Lateral (centripetal) acceleration [m/s²]"""

    lateral_g: float = 0.0
    """Lateral acceleration in g-units"""

    longitudinal_accel_mps2: float = 0.0
    """Longitudinal acceleration [m/s²]. Positive = accelerating."""

    longitudinal_g: float = 0.0
    """Longitudinal acceleration in g-units"""

    total_accel_g: float = 0.0
    """Total acceleration magnitude [g]. sqrt(lat² + long²)"""

    longitudinal_jerk_mps3: float = 0.0
    """Rate of change of longitudinal acceleration [m/s³]"""

    lateral_jerk_mps3: float = 0.0
    """Rate of change of lateral acceleration [m/s³]"""

    jerk_magnitude_mps3: float = 0.0
    """Total jerk magnitude [m/s³]"""

    # -------------------------------------------------------------------------
    # Friction & Tire Usage
    # -------------------------------------------------------------------------
    friction_mu_effective: float = 0.9
    """Effective friction coefficient (surface × condition × multiplier)"""

    friction_usage: float = 0.0
    """Fraction of available friction used [0-1]. sqrt(a_lat² + a_long²) / (μg)"""

    # -------------------------------------------------------------------------
    # Energy & Efficiency
    # -------------------------------------------------------------------------
    energy_joules: float = 0.0
    """Cumulative energy from start [J]"""

    energy_kwh: float = 0.0
    """Cumulative energy from start [kWh]"""

    energy_per_meter_j: float = 0.0
    """Energy consumption rate [J/m] for this segment"""

    energy_kwh_per_km: float = 0.0
    """Energy intensity [kWh/km] for this segment"""

    regen_potential_j: float = 0.0
    """Potential regenerative energy [J]. Nonzero if decelerating + downhill."""

    coasting_opportunity: bool = False
    """True if coasting would save energy with minimal time impact."""

    # -------------------------------------------------------------------------
    # Comfort Metrics
    # -------------------------------------------------------------------------
    comfort_score: float = 0.0
    """Composite comfort score [0-1]. 0 = comfortable, 1 = very uncomfortable."""

    is_uncomfortable: bool = False
    """True if comfort_score exceeds threshold."""

    comfort_violations: int = 0
    """Count of comfort violations in this segment (jerk, G, etc.)"""

    # -------------------------------------------------------------------------
    # Maneuver & Action
    # -------------------------------------------------------------------------
    recommended_action: str = "HOLD"
    """Recommended action: BRAKE, EASE_OFF, COAST, HOLD, ACCELERATE"""

    action_confidence: float = 1.0
    """Confidence in recommended action [0-1]. Lower near decision boundaries."""

    action_reason: str = ""
    """Human-readable explanation for recommended action."""

    time_to_next_brake_s: Optional[float] = None
    """Time until next BRAKE action at current speed [s]. None if no brake ahead."""

    # -------------------------------------------------------------------------
    # Methods
    # -------------------------------------------------------------------------

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for visualization and export."""
        return {
            # Geometry
            "lat": self.lat,
            "lon": self.lon,
            "elevation_m": self.elevation_m,
            "distance_along_m": round(self.distance_along_m, 1),
            "curvature_1pm": round(self.curvature_1pm, 6),
            "curve_radius_m": round(self.curve_radius_m, 1)
            if self.curve_radius_m < 1e6
            else "∞",
            "dcurvature_ds": round(self.dcurvature_ds, 6),
            "grade_percent": round(self.grade_percent, 1),
            "turn_type": self.turn_type,
            "turn_angle_deg": round(self.turn_angle_deg, 1),
            # Semantics
            "road_type": self.road_type,
            "speed_limit_kmh": self.speed_limit_kmh,
            "lane_count": self.lane_count,
            "surface_type": self.surface_type,
            "is_intersection": self.is_intersection_segment,
            "distance_to_intersection_m": self.distance_to_next_intersection_m,
            # Speed
            "speed_kmh": round(self.speed_kmh, 1),
            "speed_limit_margin_kmh": round(self.speed_limit_margin_kmh, 1)
            if self.speed_limit_margin_kmh
            else None,
            "physics_limit_kmh": round(self.physics_limit_speed_kmh, 1),
            "speed_ratio": round(self.speed_ratio, 2),
            "target_speed_kmh": round(self.target_speed_kmh, 1),
            # Dynamics
            "lateral_accel_mps2": round(self.lateral_accel_mps2, 2),
            "lateral_g": round(self.lateral_g, 3),
            "longitudinal_accel_mps2": round(self.longitudinal_accel_mps2, 2),
            "longitudinal_g": round(self.longitudinal_g, 3),
            "total_g": round(self.total_accel_g, 3),
            "longitudinal_jerk_mps3": round(self.longitudinal_jerk_mps3, 2),
            "lateral_jerk_mps3": round(self.lateral_jerk_mps3, 2),
            "jerk_magnitude_mps3": round(self.jerk_magnitude_mps3, 2),
            # Friction
            "friction_mu": round(self.friction_mu_effective, 2),
            "friction_usage": round(self.friction_usage, 2),
            "friction_usage_pct": round(self.friction_usage * 100, 0),
            # Energy
            "energy_kwh_per_km": round(self.energy_kwh_per_km, 3),
            "energy_per_meter_j": round(self.energy_per_meter_j, 1),
            "regen_potential_j": round(self.regen_potential_j, 1),
            "coasting_opportunity": self.coasting_opportunity,
            # Comfort
            "comfort_score": round(self.comfort_score, 2),
            "is_uncomfortable": self.is_uncomfortable,
            # Action
            "recommended_action": self.recommended_action,
            "action_confidence": round(self.action_confidence, 2),
            "action_reason": self.action_reason,
            "time_to_next_brake_s": round(self.time_to_next_brake_s, 1)
            if self.time_to_next_brake_s
            else None,
        }


# ==============================================================================
# PROFILE ANALYSIS DATACLASS (ROUTE-LEVEL METRICS)
# ==============================================================================


@dataclass
class ProfileAnalysis:
    """
    Complete analysis of a velocity profile with route-level metrics.

    All aggregate metrics are documented with units and computation methods.
    Designed for fair benchmarking across different routes and vehicles.
    """

    segments: List[SegmentFeatures] = field(default_factory=list)
    """List of per-segment features"""

    # -------------------------------------------------------------------------
    # Distance & Time
    # -------------------------------------------------------------------------
    total_distance_km: float = 0.0
    """Total route distance [km]"""

    total_time_s: float = 0.0
    """Total travel time [seconds] (lap time)"""

    # -------------------------------------------------------------------------
    # Speed Statistics
    # -------------------------------------------------------------------------
    avg_speed_kmh: float = 0.0
    """Average speed [km/h]. distance / time."""

    max_speed_kmh: float = 0.0
    """Maximum speed achieved [km/h]"""

    min_speed_kmh: float = 0.0
    """Minimum speed (excluding stops) [km/h]"""

    p95_speed_kmh: float = 0.0
    """95th percentile speed [km/h]"""

    speed_std_kmh: float = 0.0
    """Speed standard deviation [km/h]. Higher = more variable."""

    # -------------------------------------------------------------------------
    # Dynamics Statistics
    # -------------------------------------------------------------------------
    max_lateral_g: float = 0.0
    """Maximum lateral acceleration [g]"""

    avg_lateral_g: float = 0.0
    """Average lateral acceleration [g]"""

    p95_lateral_g: float = 0.0
    """95th percentile lateral acceleration [g]"""

    max_longitudinal_g: float = 0.0
    """Maximum longitudinal acceleration magnitude [g]"""

    max_total_g: float = 0.0
    """Maximum total acceleration [g]"""

    p95_longitudinal_jerk: float = 0.0
    """95th percentile longitudinal jerk magnitude [m/s³]"""

    p95_lateral_jerk: float = 0.0
    """95th percentile lateral jerk magnitude [m/s³]"""

    max_jerk_mps3: float = 0.0
    """Maximum jerk magnitude [m/s³]"""

    # -------------------------------------------------------------------------
    # Speed Limit / Compliance
    # -------------------------------------------------------------------------
    avg_speed_limit_kmh: float = 0.0
    """Average posted speed limit across segments [km/h]"""

    p95_speed_limit_kmh: float = 0.0
    """95th percentile posted speed limit [km/h]"""

    avg_speed_over_limit_kmh: float = 0.0
    """Average amount by which speed exceeds limit on overspeed segments [km/h]"""

    pct_distance_over_limit: float = 0.0
    """Fraction of distance driven above the posted speed limit [0-1]"""

    # -------------------------------------------------------------------------
    # Friction Usage
    # -------------------------------------------------------------------------
    max_friction_usage: float = 0.0
    """Maximum friction utilization [0-1]"""

    avg_friction_usage: float = 0.0
    """Average friction utilization [0-1]"""

    p95_friction_usage: float = 0.0
    """95th percentile friction usage [0-1]"""

    # -------------------------------------------------------------------------
    # Energy Statistics
    # -------------------------------------------------------------------------
    total_energy_kwh: float = 0.0
    """Total energy consumed [kWh]"""

    energy_kwh_per_km: float = 0.0
    """Energy efficiency [kWh/km]"""

    total_regen_kwh: float = 0.0
    """Total regenerative energy potential [kWh]"""

    regen_fraction: float = 0.0
    """Regenerated / total energy magnitude [0-1]"""

    # -------------------------------------------------------------------------
    # Comfort Statistics
    # -------------------------------------------------------------------------
    avg_comfort_score: float = 0.0
    """Average comfort score [0-1]"""

    max_comfort_score: float = 0.0
    """Maximum comfort score [0-1]"""

    comfort_violations_per_km: float = 0.0
    """Count of comfort violations per kilometer"""

    pct_uncomfortable_distance: float = 0.0
    """Fraction of distance with comfort_score > threshold"""

    # -------------------------------------------------------------------------
    # Action Distribution (fraction of distance)
    # -------------------------------------------------------------------------
    brake_fraction: float = 0.0
    """Fraction of distance with BRAKE/EASE_OFF action [0-1]"""

    coast_fraction: float = 0.0
    """Fraction of distance with COAST action [0-1]"""

    accelerate_fraction: float = 0.0
    """Fraction of distance with ACCELERATE action [0-1]"""

    hold_fraction: float = 0.0
    """Fraction of distance with HOLD action [0-1]"""

    # -------------------------------------------------------------------------
    # Geometry Statistics
    # -------------------------------------------------------------------------
    turns_per_km: float = 0.0
    """Number of significant turns per kilometer"""

    avg_curvature: float = 0.0
    """Average curvature [1/m]"""

    max_curvature: float = 0.0
    """Maximum curvature [1/m]"""

    avg_grade_percent: float = 0.0
    """Average absolute grade [%]"""

    max_grade_percent: float = 0.0
    """Maximum absolute grade [%]"""

    total_elevation_gain_m: float = 0.0
    """Total elevation gain [m]"""

    total_elevation_loss_m: float = 0.0
    """Total elevation loss [m]"""

    intersection_count: int = 0
    """Number of intersections on route"""

    # -------------------------------------------------------------------------
    # Composite Scores
    # -------------------------------------------------------------------------
    difficulty_score: float = 0.0
    """
    Composite difficulty score [0-1].
    Based on: curvature variance, grade, intersections, comfort violations.
    Higher = more challenging route.
    """

    safety_margin_score: float = 0.0
    """
    Safety margin score [0-1].
    Based on: avg friction usage, p95 lateral G.
    Higher = more safety margin.
    """

    # -------------------------------------------------------------------------
    # Methods
    # -------------------------------------------------------------------------

    def get_summary_dict(self) -> Dict[str, Any]:
        """Get summary statistics as dictionary for display/export."""
        return {
            # Core
            "total_distance_km": round(self.total_distance_km, 2),
            "lap_time_s": round(self.total_time_s, 1),
            "lap_time_min": round(self.total_time_s / 60, 2),
            # Speed
            "avg_speed_kmh": round(self.avg_speed_kmh, 1),
            "max_speed_kmh": round(self.max_speed_kmh, 1),
            "min_speed_kmh": round(self.min_speed_kmh, 1),
            "p95_speed_kmh": round(self.p95_speed_kmh, 1),
            # Dynamics
            "max_lateral_g": round(self.max_lateral_g, 2),
            "avg_lateral_g": round(self.avg_lateral_g, 2),
            "p95_lateral_g": round(self.p95_lateral_g, 2),
            "max_longitudinal_g": round(self.max_longitudinal_g, 2),
            "p95_longitudinal_jerk": round(self.p95_longitudinal_jerk, 1),
            "p95_lateral_jerk": round(self.p95_lateral_jerk, 1),
            # Friction
            "max_friction_usage": round(self.max_friction_usage, 2),
            "avg_friction_usage": round(self.avg_friction_usage, 2),
            "p95_friction_usage": round(self.p95_friction_usage, 2),
            # Energy
            "total_energy_kwh": round(self.total_energy_kwh, 3),
            "energy_kwh_per_km": round(self.energy_kwh_per_km, 3),
            "total_regen_kwh": round(self.total_regen_kwh, 3),
            "regen_fraction": round(self.regen_fraction, 2),
            # Comfort
            "avg_comfort_score": round(self.avg_comfort_score, 2),
            "comfort_violations_per_km": round(self.comfort_violations_per_km, 2),
            "pct_uncomfortable": round(self.pct_uncomfortable_distance * 100, 1),
            # Actions
            "brake_fraction": round(self.brake_fraction, 3),
            "coast_fraction": round(self.coast_fraction, 3),
            "accelerate_fraction": round(self.accelerate_fraction, 3),
            # Geometry
            "turns_per_km": round(self.turns_per_km, 1),
            "max_grade_percent": round(self.max_grade_percent, 1),
            "total_elevation_gain_m": round(self.total_elevation_gain_m, 1),
            "total_elevation_loss_m": round(self.total_elevation_loss_m, 1),
            "intersection_count": self.intersection_count,
            # Speed-limit compliance
            "avg_speed_limit_kmh": round(self.avg_speed_limit_kmh, 1),
            "p95_speed_limit_kmh": round(self.p95_speed_limit_kmh, 1),
            "avg_speed_over_limit_kmh": round(self.avg_speed_over_limit_kmh, 1),
            "pct_distance_over_limit": round(self.pct_distance_over_limit * 100, 1),
            # Composite
            "difficulty_score": round(self.difficulty_score, 2),
            "safety_margin_score": round(self.safety_margin_score, 2),
        }

    def get_hud_dict(self) -> Dict[str, str]:
        """Get formatted strings for HUD display."""
        return {
            "Distance": f"{self.total_distance_km:.2f} km",
            "Lap Time": f"{self.total_time_s:.1f}s ({self.total_time_s/60:.2f} min)",
            "Avg Speed": f"{self.avg_speed_kmh:.1f} km/h",
            "Max Speed": f"{self.max_speed_kmh:.1f} km/h",
            "Energy": f"{self.total_energy_kwh:.3f} kWh ({self.energy_kwh_per_km*1000:.0f} Wh/km)",
            "Max Lat G": f"{self.max_lateral_g:.2f} g (p95: {self.p95_lateral_g:.2f} g)",
            "Max Jerk": f"{self.max_jerk_mps3:.1f} m/s³",
            "Friction Usage": f"avg {self.avg_friction_usage*100:.0f}%, max {self.max_friction_usage*100:.0f}%",
            "Comfort Violations": f"{self.comfort_violations_per_km:.2f} /km",
            "Brake Zones": f"{self.brake_fraction*100:.1f}%",
            "Turns": f"{self.turns_per_km:.1f} /km",
            "Difficulty": f"{self.difficulty_score:.2f}",
        }


# ==============================================================================
# HELPER FUNCTIONS
# ==============================================================================


def get_friction_coefficient(surface_type: str, condition: str = "dry") -> float:
    """
    Get effective friction coefficient for a surface type and condition.

    Args:
        surface_type: Road surface (asphalt, concrete, gravel, etc.)
        condition: "dry" or "wet"

    Returns:
        Friction coefficient μ (typically 0.1 - 1.0)
    """
    materials = {
        "asphalt": {"mu_dry": 0.90, "mu_wet": 0.60},
        "concrete": {"mu_dry": 0.85, "mu_wet": 0.65},
        "cobblestone": {"mu_dry": 0.55, "mu_wet": 0.40},
        "gravel": {"mu_dry": 0.60, "mu_wet": 0.45},
        "dirt": {"mu_dry": 0.55, "mu_wet": 0.35},
        "sand": {"mu_dry": 0.40, "mu_wet": 0.30},
        "ice": {"mu_dry": 0.15, "mu_wet": 0.10},
        "snow_packed": {"mu_dry": 0.25, "mu_wet": 0.20},
    }

    props = materials.get(surface_type.lower(), materials["asphalt"])
    return props["mu_wet"] if condition == "wet" else props["mu_dry"]


def compute_physics_limit_speed(
    curvature: float, mu: float, gravity: float = GRAVITY
) -> float:
    """
    Compute friction-limited speed from curvature.

    Args:
        curvature: Path curvature [1/m]
        mu: Friction coefficient
        gravity: Gravitational acceleration [m/s²]

    Returns:
        Maximum speed [m/s] before tire slip
    """
    if curvature < 1e-9:
        return 100.0  # ~360 km/h for straight
    return math.sqrt(mu * gravity / curvature)


def classify_turn(heading_change_deg: float) -> str:
    """
    Classify a turn based on heading change.

    Args:
        heading_change_deg: Signed heading change [degrees]. + = left, - = right.

    Returns:
        Turn type string
    """
    abs_change = abs(heading_change_deg)

    if abs_change < TURN_STRAIGHT_THRESHOLD:
        return "straight"
    elif abs_change < TURN_SLIGHT_THRESHOLD:
        return "slight_left" if heading_change_deg > 0 else "slight_right"
    elif abs_change < TURN_NORMAL_THRESHOLD:
        return "left" if heading_change_deg > 0 else "right"
    elif abs_change < TURN_SHARP_THRESHOLD:
        return "sharp_left" if heading_change_deg > 0 else "sharp_right"
    else:
        return "u_turn"


def determine_recommended_action(
    speed_ratio: float,
    friction_usage: float,
    lateral_g: float,
    longitudinal_accel: float,
    grade_percent: float,
    comfort_score: float,
    curvature: float,
) -> Tuple[str, float, str]:
    """
    Determine recommended action based on physics state.

    Returns:
        (action, confidence, reason) tuple
    """
    # Priority 1: Critical safety - near tire limit
    if friction_usage > CRITICAL_FRICTION_USAGE:
        conf = 1.0 - (friction_usage - CRITICAL_FRICTION_USAGE) / (
            1.0 - CRITICAL_FRICTION_USAGE
        )
        return (
            "BRAKE",
            min(1.0, 1.0 - conf * 0.3),
            f"Critical friction ({friction_usage:.0%})",
        )

    if lateral_g > 0.7:
        return "BRAKE", 0.95, f"High lateral G ({lateral_g:.2f}g)"

    # Priority 2: High friction - ease off
    if friction_usage > HIGH_FRICTION_USAGE:
        margin = (CRITICAL_FRICTION_USAGE - friction_usage) / (
            CRITICAL_FRICTION_USAGE - HIGH_FRICTION_USAGE
        )
        return (
            "EASE_OFF",
            0.7 + 0.2 * margin,
            f"High friction usage ({friction_usage:.0%})",
        )

    if lateral_g > 0.5 and speed_ratio > 0.85:
        return "EASE_OFF", 0.65, "Near limit in corner"

    # Priority 3: Comfort optimization
    if comfort_score > 0.6:
        return "SMOOTH", 0.6, "Comfort optimization"

    # Priority 4: Efficiency - coasting opportunities
    if speed_ratio < 0.6 and friction_usage < 0.3:
        if grade_percent < -2:  # Downhill
            return "COAST", 0.8, "Downhill regen opportunity"
        elif abs(longitudinal_accel) < 0.5:
            return "COAST", 0.7, "Steady state coasting"

    # Priority 5: Acceleration opportunity
    if speed_ratio < 0.7 and friction_usage < 0.4 and curvature < 0.01:
        if grade_percent > 2:
            return "ACCELERATE", 0.75, "Uphill, margin available"
        elif grade_percent < -1:
            return "COAST", 0.7, "Downhill regen"
        else:
            return "ACCELERATE", 0.7, "Speed headroom"

    # Default - maintain
    return "HOLD", 0.8, "Optimal operating point"


def _percentile(values: List[float], p: float) -> float:
    """Compute percentile of a list of values."""
    if not values:
        return 0.0
    sorted_vals = sorted(values)
    k = (len(sorted_vals) - 1) * p / 100
    f = math.floor(k)
    c = math.ceil(k)
    if f == c:
        return sorted_vals[int(k)]
    return sorted_vals[int(f)] * (c - k) + sorted_vals[int(c)] * (k - f)


def _compute_heading(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Compute heading from point 1 to point 2 [degrees, 0=N, 90=E]."""
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    heading = math.degrees(math.atan2(dlon, dlat))
    return heading % 360


# ==============================================================================
# MAIN ANALYSIS FUNCTION
# ==============================================================================


def analyze_profile(
    path_points: List[Dict],
    condition: str = "dry",
    friction_multiplier: float = 1.0,
    comfort_g_threshold: float = COMFORT_LATERAL_G_THRESHOLD,
    comfort_jerk_threshold: float = COMFORT_JERK_THRESHOLD,
    vehicle_mass_kg: float = 1500.0,
) -> ProfileAnalysis:
    """
    Analyze a velocity profile and compute rich features.

    This is the main entry point for computing AV training features.
    All features are computed per-segment and aggregated to route-level.

    Args:
        path_points: List of path point dicts with lat, lon, speed_mps, curvature, etc.
        condition: "dry" or "wet" for friction calculation
        friction_multiplier: Additional friction scaling (e.g., 0.7 for rain)
        comfort_g_threshold: Lateral G threshold for comfort [g]
        comfort_jerk_threshold: Jerk threshold for comfort [m/s³]
        vehicle_mass_kg: Vehicle mass for energy calculations [kg]

    Returns:
        ProfileAnalysis with all per-segment and route-level features
    """
    if len(path_points) < 2:
        return ProfileAnalysis()

    segments: List[SegmentFeatures] = []

    # -------------------------------------------------------------------------
    # First pass: Compute basic features
    # -------------------------------------------------------------------------

    total_distance = 0.0
    total_time = 0.0
    total_energy_consumed = 0.0
    total_regen = 0.0
    elevation_gain = 0.0
    elevation_loss = 0.0
    overspeed_distance = 0.0
    overspeed_deltas: List[float] = []

    # Action tracking
    action_distances = {
        "BRAKE": 0,
        "EASE_OFF": 0,
        "COAST": 0,
        "HOLD": 0,
        "ACCELERATE": 0,
        "SMOOTH": 0,
    }

    # Comfort tracking
    uncomfortable_distance = 0.0
    total_comfort_violations = 0

    # Turn tracking
    turn_count = 0
    prev_heading = None

    for i, pt in enumerate(path_points):
        seg = SegmentFeatures(index=i)

        # ----- Position & Geometry -----
        seg.lat = pt.get("lat", 0)
        seg.lon = pt.get("lon", 0)
        seg.elevation_m = pt.get("z_m", pt.get("elevation_m", 0))
        seg.distance_along_m = pt.get("distance_along_m", 0)
        seg.curvature_1pm = abs(pt.get("curvature", 0))

        if seg.curvature_1pm > 1e-9:
            seg.curve_radius_m = 1.0 / seg.curvature_1pm
        else:
            seg.curve_radius_m = float("inf")

        # Segment length
        if i > 0:
            seg.segment_length_m = max(
                0.1,
                seg.distance_along_m - path_points[i - 1].get("distance_along_m", 0),
            )

        # Grade
        if i > 0 and seg.segment_length_m > 0.1:
            prev_elev = path_points[i - 1].get(
                "z_m", path_points[i - 1].get("elevation_m", 0)
            )
            dz = seg.elevation_m - prev_elev
            seg.grade_percent = (dz / seg.segment_length_m) * 100

            # Clamp pathological grades (noise / DEM artifacts)
            max_grade_abs = 30.0  # [%] typical drivable max; prevents 600% spikes
            if seg.grade_percent > max_grade_abs:
                seg.grade_percent = max_grade_abs
            elif seg.grade_percent < -max_grade_abs:
                seg.grade_percent = -max_grade_abs

            if dz > 0:
                elevation_gain += dz
            else:
                elevation_loss += abs(dz)

        # Curvature rate of change
        if i > 0 and seg.segment_length_m > 0.1:
            prev_curv = abs(path_points[i - 1].get("curvature", 0))
            seg.dcurvature_ds = (seg.curvature_1pm - prev_curv) / seg.segment_length_m

        # ----- Heading & Turns -----
        if i < len(path_points) - 1:
            next_pt = path_points[i + 1]
            heading = _compute_heading(
                seg.lat,
                seg.lon,
                next_pt.get("lat", seg.lat),
                next_pt.get("lon", seg.lon),
            )
            if prev_heading is not None:
                heading_change = heading - prev_heading
                # Normalize to [-180, 180]
                while heading_change > 180:
                    heading_change -= 360
                while heading_change < -180:
                    heading_change += 360
                seg.turn_angle_deg = heading_change
                seg.turn_type = classify_turn(heading_change)

                if abs(heading_change) > TURN_NORMAL_THRESHOLD:
                    turn_count += 1
            prev_heading = heading

        # ----- Semantics -----
        seg.surface_type = pt.get("surface_type", "asphalt")
        seg.road_type = pt.get("road_type", pt.get("highway", None))
        seg.speed_limit_kmh = pt.get(
            "speed_limit_kmh", pt.get("osm_speed_limit_kmh", pt.get("maxspeed", None))
        )
        seg.lane_count = pt.get("lane_count", pt.get("lanes", None))
        seg.is_intersection_segment = pt.get("is_intersection", False)

        # ----- Friction -----
        seg.friction_mu_effective = (
            get_friction_coefficient(seg.surface_type, condition) * friction_multiplier
        )

        # ----- Speed -----
        seg.speed_mps = pt.get("speed_mps", pt.get("v_profile", 20))
        seg.speed_kmh = seg.speed_mps * 3.6

        seg.physics_limit_speed_mps = compute_physics_limit_speed(
            seg.curvature_1pm, seg.friction_mu_effective
        )
        seg.physics_limit_speed_kmh = seg.physics_limit_speed_mps * 3.6

        # Enforce display speed cap using posted limit + physics limit.
        # This does NOT change the underlying solver energy_joules, but
        # makes HUD speeds/lap time more realistic for city routes.
        effective_cap_kmh = seg.physics_limit_speed_kmh
        if seg.speed_limit_kmh:
            effective_cap_kmh = min(effective_cap_kmh, seg.speed_limit_kmh)
        # Global sanity cap (e.g., 160 km/h)
        try:
            global_cap = float(
                os.environ.get("APEXVELOCITY_MAX_DISPLAY_SPEED_KMH", "160")
            )
        except Exception:
            global_cap = 160.0
        effective_cap_kmh = min(effective_cap_kmh, global_cap)

        if effective_cap_kmh > 0 and seg.speed_kmh > effective_cap_kmh:
            seg.speed_kmh = effective_cap_kmh
            seg.speed_mps = seg.speed_kmh / 3.6

        if seg.physics_limit_speed_mps > 0:
            seg.speed_ratio = min(1.0, seg.speed_mps / seg.physics_limit_speed_mps)

        if seg.speed_limit_kmh:
            seg.speed_limit_margin_kmh = seg.speed_limit_kmh - seg.speed_kmh
            if seg.speed_limit_margin_kmh < 0 and seg.segment_length_m > 0:
                overspeed_distance += seg.segment_length_m
                overspeed_deltas.append(-seg.speed_limit_margin_kmh)

        # ----- Dynamics -----
        if seg.curvature_1pm > 1e-9:
            seg.lateral_accel_mps2 = seg.speed_mps ** 2 * seg.curvature_1pm
            seg.lateral_g = seg.lateral_accel_mps2 / GRAVITY

        if i > 0 and seg.segment_length_m > 0:
            prev_speed = path_points[i - 1].get(
                "speed_mps", path_points[i - 1].get("v_profile", seg.speed_mps)
            )
            avg_speed = (seg.speed_mps + prev_speed) / 2
            if avg_speed > 0.1:
                dt = seg.segment_length_m / avg_speed
                seg.longitudinal_accel_mps2 = (seg.speed_mps - prev_speed) / dt
                seg.longitudinal_g = seg.longitudinal_accel_mps2 / GRAVITY

                # Longitudinal jerk
                if len(segments) > 0 and dt > 0:
                    prev_seg = segments[-1]
                    seg.longitudinal_jerk_mps3 = (
                        seg.longitudinal_accel_mps2 - prev_seg.longitudinal_accel_mps2
                    ) / dt
                    seg.lateral_jerk_mps3 = (
                        seg.lateral_accel_mps2 - prev_seg.lateral_accel_mps2
                    ) / dt
                    seg.jerk_magnitude_mps3 = math.sqrt(
                        seg.longitudinal_jerk_mps3 ** 2 + seg.lateral_jerk_mps3 ** 2
                    )

        seg.total_accel_g = math.sqrt(seg.lateral_g ** 2 + seg.longitudinal_g ** 2)

        # ----- Friction Usage -----
        max_accel = seg.friction_mu_effective * GRAVITY
        actual_accel = math.sqrt(
            seg.lateral_accel_mps2 ** 2 + seg.longitudinal_accel_mps2 ** 2
        )
        if max_accel > 0:
            seg.friction_usage = min(1.0, actual_accel / max_accel)

        # ----- Energy -----
        seg.energy_joules = pt.get("energy_joules", 0)
        seg.energy_kwh = pt.get("energy_kwh", seg.energy_joules / 3.6e6)

        if i > 0 and seg.segment_length_m > 0:
            prev_energy = path_points[i - 1].get("energy_kwh", 0)
            delta_energy_kwh = seg.energy_kwh - prev_energy
            seg.energy_per_meter_j = (delta_energy_kwh * 3.6e6) / seg.segment_length_m
            seg.energy_kwh_per_km = delta_energy_kwh / (seg.segment_length_m / 1000)

            # Regen potential (decelerating OR downhill)
            if seg.longitudinal_accel_mps2 < -0.5 or seg.grade_percent < -2:
                # Simplified regen: kinetic energy change + grade potential
                dv = seg.speed_mps - path_points[i - 1].get("speed_mps", seg.speed_mps)
                if dv < 0:
                    kinetic_regen = (
                        0.5
                        * vehicle_mass_kg
                        * (
                            path_points[i - 1].get("speed_mps", seg.speed_mps) ** 2
                            - seg.speed_mps ** 2
                        )
                    )
                    seg.regen_potential_j = kinetic_regen * 0.7  # 70% regen efficiency

            if seg.grade_percent < -1:
                grade_regen = (
                    vehicle_mass_kg
                    * GRAVITY
                    * abs(seg.grade_percent / 100)
                    * seg.segment_length_m
                    * 0.7
                )
                seg.regen_potential_j += grade_regen

            total_regen += seg.regen_potential_j / 3.6e6  # Convert to kWh

        # Coasting opportunity: small speed loss for big energy save
        if (
            seg.speed_ratio < 0.75
            and seg.friction_usage < 0.4
            and seg.grade_percent < 1
        ):
            seg.coasting_opportunity = True

        # ----- Comfort -----
        lat_discomfort = (
            max(0, seg.lateral_g - comfort_g_threshold) / comfort_g_threshold
        )
        long_discomfort = (
            max(0, abs(seg.longitudinal_g) - COMFORT_LONGITUDINAL_G_THRESHOLD)
            / COMFORT_LONGITUDINAL_G_THRESHOLD
        )
        jerk_discomfort = (
            max(0, seg.jerk_magnitude_mps3 - comfort_jerk_threshold)
            / comfort_jerk_threshold
        )

        seg.comfort_score = min(
            1.0, (lat_discomfort + long_discomfort + jerk_discomfort) / 3
        )
        seg.is_uncomfortable = seg.comfort_score > 0.3

        if seg.lateral_g > comfort_g_threshold:
            seg.comfort_violations += 1
        if abs(seg.longitudinal_g) > COMFORT_LONGITUDINAL_G_THRESHOLD:
            seg.comfort_violations += 1
        if seg.jerk_magnitude_mps3 > comfort_jerk_threshold:
            seg.comfort_violations += 1

        total_comfort_violations += seg.comfort_violations

        # Target speed (comfort-limited)
        comfort_limit_factor = 1.0 - 0.3 * seg.comfort_score
        seg.target_speed_kmh = min(
            seg.physics_limit_speed_kmh * 0.85, seg.speed_kmh * comfort_limit_factor
        )
        if seg.speed_limit_kmh:
            seg.target_speed_kmh = min(seg.target_speed_kmh, seg.speed_limit_kmh)

        # ----- Recommended Action -----
        (
            seg.recommended_action,
            seg.action_confidence,
            seg.action_reason,
        ) = determine_recommended_action(
            seg.speed_ratio,
            seg.friction_usage,
            seg.lateral_g,
            seg.longitudinal_accel_mps2,
            seg.grade_percent,
            seg.comfort_score,
            seg.curvature_1pm,
        )

        # ----- Accumulate -----
        total_distance += seg.segment_length_m
        if seg.speed_mps > 0.1:
            total_time += seg.segment_length_m / seg.speed_mps

        action_key = seg.recommended_action
        if action_key in action_distances:
            action_distances[action_key] += seg.segment_length_m
        else:
            action_distances["HOLD"] += seg.segment_length_m

        if seg.is_uncomfortable:
            uncomfortable_distance += seg.segment_length_m

        segments.append(seg)

    # -------------------------------------------------------------------------
    # Second pass: Compute lookahead features (time to next brake)
    # -------------------------------------------------------------------------
    for i in range(len(segments)):
        seg = segments[i]

        # Find next brake zone
        dist_to_brake = 0.0
        for j in range(i + 1, len(segments)):
            dist_to_brake += segments[j].segment_length_m
            if segments[j].recommended_action in ["BRAKE", "EASE_OFF"]:
                if seg.speed_mps > 0.1:
                    seg.time_to_next_brake_s = dist_to_brake / seg.speed_mps
                break

        # Find distance to next intersection
        dist_to_int = 0.0
        for j in range(i + 1, len(segments)):
            dist_to_int += segments[j].segment_length_m
            if segments[j].is_intersection_segment:
                seg.distance_to_next_intersection_m = dist_to_int
                break

    # -------------------------------------------------------------------------
    # Aggregate to route-level metrics
    # -------------------------------------------------------------------------
    analysis = ProfileAnalysis(segments=segments)

    analysis.total_distance_km = total_distance / 1000
    analysis.total_time_s = total_time
    analysis.total_energy_kwh = segments[-1].energy_kwh if segments else 0
    analysis.total_regen_kwh = total_regen
    analysis.total_elevation_gain_m = elevation_gain
    analysis.total_elevation_loss_m = elevation_loss
    analysis.intersection_count = sum(1 for s in segments if s.is_intersection_segment)

    # Speed stats
    speeds = [s.speed_kmh for s in segments if s.speed_kmh > 0]
    if speeds:
        analysis.avg_speed_kmh = sum(speeds) / len(speeds)
        analysis.max_speed_kmh = max(speeds)
        analysis.min_speed_kmh = min(speeds)
        analysis.p95_speed_kmh = _percentile(speeds, 95)
        mean_speed = analysis.avg_speed_kmh
        analysis.speed_std_kmh = math.sqrt(
            sum((s - mean_speed) ** 2 for s in speeds) / len(speeds)
        )

    # Lateral G stats
    lateral_gs = [s.lateral_g for s in segments]
    if lateral_gs:
        analysis.max_lateral_g = max(lateral_gs)
        analysis.avg_lateral_g = sum(lateral_gs) / len(lateral_gs)
        analysis.p95_lateral_g = _percentile(lateral_gs, 95)

    # Longitudinal stats
    long_gs = [abs(s.longitudinal_g) for s in segments]
    analysis.max_longitudinal_g = max(long_gs) if long_gs else 0
    analysis.max_total_g = max((s.total_accel_g for s in segments), default=0)

    # Jerk stats
    long_jerks = [abs(s.longitudinal_jerk_mps3) for s in segments]
    lat_jerks = [abs(s.lateral_jerk_mps3) for s in segments]
    analysis.p95_longitudinal_jerk = _percentile(long_jerks, 95)
    analysis.p95_lateral_jerk = _percentile(lat_jerks, 95)
    analysis.max_jerk_mps3 = max((s.jerk_magnitude_mps3 for s in segments), default=0)

    # Friction stats
    frictions = [s.friction_usage for s in segments]
    if frictions:
        analysis.max_friction_usage = max(frictions)
        analysis.avg_friction_usage = sum(frictions) / len(frictions)
        analysis.p95_friction_usage = _percentile(frictions, 95)

    # Energy stats
    if analysis.total_distance_km > 0:
        analysis.energy_kwh_per_km = (
            analysis.total_energy_kwh / analysis.total_distance_km
        )

    total_energy_magnitude = analysis.total_energy_kwh + analysis.total_regen_kwh
    if total_energy_magnitude > 0:
        analysis.regen_fraction = analysis.total_regen_kwh / total_energy_magnitude

    # Comfort stats
    comfort_scores = [s.comfort_score for s in segments]
    if comfort_scores:
        analysis.avg_comfort_score = sum(comfort_scores) / len(comfort_scores)
        analysis.max_comfort_score = max(comfort_scores)

    if analysis.total_distance_km > 0:
        analysis.comfort_violations_per_km = (
            total_comfort_violations / analysis.total_distance_km
        )
        analysis.pct_uncomfortable_distance = uncomfortable_distance / (
            total_distance if total_distance > 0 else 1
        )

    # Action distribution
    if total_distance > 0:
        analysis.brake_fraction = (
            action_distances["BRAKE"] + action_distances["EASE_OFF"]
        ) / total_distance
        analysis.coast_fraction = action_distances["COAST"] / total_distance
        analysis.accelerate_fraction = action_distances["ACCELERATE"] / total_distance
        analysis.hold_fraction = (
            action_distances["HOLD"] + action_distances["SMOOTH"]
        ) / total_distance

    # Geometry stats
    if analysis.total_distance_km > 0:
        analysis.turns_per_km = turn_count / analysis.total_distance_km

    curvatures = [s.curvature_1pm for s in segments]
    if curvatures:
        analysis.avg_curvature = sum(curvatures) / len(curvatures)
        analysis.max_curvature = max(curvatures)

    grades = [abs(s.grade_percent) for s in segments]
    if grades:
        analysis.avg_grade_percent = sum(grades) / len(grades)
        analysis.max_grade_percent = max(grades)

    # Speed limit statistics
    limits = [s.speed_limit_kmh for s in segments if s.speed_limit_kmh]
    if limits:
        analysis.avg_speed_limit_kmh = sum(limits) / len(limits)
        analysis.p95_speed_limit_kmh = _percentile(limits, 95)
    if overspeed_deltas:
        analysis.avg_speed_over_limit_kmh = sum(overspeed_deltas) / len(
            overspeed_deltas
        )
    if total_distance > 0:
        analysis.pct_distance_over_limit = overspeed_distance / total_distance

    # Composite scores
    # Difficulty: combines curvature variance, grade, intersections, comfort issues
    curv_var = (
        sum((c - analysis.avg_curvature) ** 2 for c in curvatures) / len(curvatures)
        if curvatures
        else 0
    )
    curv_factor = min(1.0, curv_var * 10000)  # Scale variance
    grade_factor = min(1.0, analysis.max_grade_percent / 15)  # 15% = max difficulty
    int_factor = min(
        1.0, (analysis.intersection_count / max(1, analysis.total_distance_km)) / 10
    )  # 10/km = max
    comfort_factor = min(
        1.0, analysis.comfort_violations_per_km / 5
    )  # 5/km = max difficulty

    analysis.difficulty_score = (
        curv_factor + grade_factor + int_factor + comfort_factor
    ) / 4

    # Safety margin: inverse of friction usage and lateral G
    analysis.safety_margin_score = max(
        0, 1.0 - (analysis.avg_friction_usage + analysis.p95_lateral_g) / 2
    )

    return analysis


def convert_segments_for_viz(analysis: ProfileAnalysis) -> List[Dict]:
    """Convert analysis segments to visualization-ready format."""
    return [seg.to_dict() for seg in analysis.segments]
