# ApexVelocity Coordinate & Alignment Diagnosis

## STEP 0 – HIGH-LEVEL REPO BREAKDOWN

### Top-Level Directory Structure

| Directory | Description |
|-----------|-------------|
| `core/` | C++20 physics engine, solver, config loading, pybind11 bindings |
| `server/` | Go 1.21+ HTTP API with CGO bridge to C++ core |
| `python/apexvelocity/` | Python high-level API, analysis, visualization, CLI |
| `python/examples/` | Demo scripts (SF Lombard Street, Nürburgring benchmarks) |
| `config/` | YAML/JSON config files: materials, vehicles, tag mappings |
| `docs/` | Documentation (metrics table, this diagnosis) |

### Core C++ Module (`core/`)

**Key Files:**
- `include/physics/Types.h` - Defines `VehicleParams`, `PathPoint`, `SolverConfig`
- `include/physics/VelocityProfileSolver.h` - 3-pass kinematic solver
- `include/physics/PhysicsMath.h` - Friction/rollover speed limits
- `include/utils/ConfigManager.h` - Singleton config loader
- `src/bindings/bindings.cpp` - pybind11 Python bindings

**Solver Input:**
```cpp
struct PathPoint {
    double x_m, y_m, z_m;      // Local coordinates in METERS
    double curvature;           // 1/radius (1/m)
    double distance_along_m;    // Cumulative distance
    std::string surface_type;   // "asphalt", "gravel", etc.
};
```

**Solver Output:**
- `v_static` - Static speed limit from curvature/friction
- `v_profile` - Final speed after forward/backward passes
- `energy_joules` - Cumulative energy

**Key Insight:** The C++ solver works in **meters (local coordinates)**, not lat/lon.

### Python Module (`python/apexvelocity/`)

| File | Purpose |
|------|---------|
| `api.py` | High-level `solve()`, `create_path()`, `load_vehicle()` |
| `analysis.py` | Rich feature extraction: `SegmentFeatures`, `ProfileAnalysis`, `analyze_profile()` |
| `loader.py` | OSM data loading via osmnx, `OSMLoader`, `LoadedPath`, curvature calculation |
| `viz.py` | PyDeck visualization: `visualize_profile()`, `visualize_comparison()`, `_build_path_data()` |
| `cli.py` | CLI: `--city`, `--vehicle`, `--color-metric`, `--style` |
| `osm_fetcher.py` | Alternative OSM fetcher (partially used) |

**Data Flow:**
```
Input (lat/lon) → analysis.py (compute features) → viz.py (build PyDeck paths) → HTML
```

### Example Scripts (`python/examples/`)

| Script | Purpose |
|--------|---------|
| `demo_san_francisco.py` | Energy benchmark on Lombard Street (18 hardcoded GPS points) |
| `benchmark_nurburgring.py` | Dry vs Wet friction benchmark (73 key corners → 346 interpolated) |

### Server (`server/`)

**Endpoints:**
- `POST /v1/analyze` - Accepts `geometry` (lat/lon/elev arrays), returns velocity/energy
- `POST /v1/config/reload` - Hot-reload configuration

**Note:** Server expects `[lat, lon, elevation]` but internally converts to local meters.

---

## STEP 1 – COORDINATE PIPELINE TRACE

### San Francisco (demo_san_francisco.py)

**Source:** Hardcoded `LOMBARD_CROOKED_SECTION` (lines 33-63)
```python
LOMBARD_CROOKED_SECTION = [
    {'lat': 37.802017, 'lon': -122.418625},  # Start at Hyde St
    ...
    {'lat': 37.802050, 'lon': -122.420140},  # End at Leavenworth
]
```

**Coordinate System:** WGS84 (lat/lon degrees) - ✓ Correct

**Conversion:** `create_lombard_path()` converts to dict with:
- `lat`, `lon` - preserved as degrees
- `x_m`, `y_m` - set to 0 (NOT used for local coords)
- `curvature` - computed from Menger curvature formula

**Key Finding:** 
- Only **18 points** for 8 hairpin turns
- Average spacing: 11.7m
- No CRS projection applied - stays in lat/lon degrees

### Nürburgring (benchmark_nurburgring.py)

**Source:** Hardcoded `NORDSCHLEIFE_TRACK` (lines 31-131)
```python
NORDSCHLEIFE_TRACK = [
    (50.3353, 6.9418),  # Start/Finish (lat, lon)
    ...
]
```

**Coordinate System:** WGS84 (lat/lon degrees) - ✓ Correct

**Interpolation:** `create_track_path()`:
- 73 key corners → 346 points (5 intermediate per segment)
- Linear interpolation in lat/lon space
- Curvature computed from Menger formula

**Key Finding:**
- Track dimensions correct: 7.5km × 4.5km (real: ~7km × 5km)
- Total distance: 22.66 km (real: 20.83 km) - close but slightly long
- Linear interpolation creates **straight segments**, not smooth curves

### Visualization (viz.py)

**`_build_path_data()` (line 370):**
```python
path_segment = {
    'path': [[lon1, lat1, elevation], [lon2, lat2, elevation]],
    ...
}
```

**PyDeck PathLayer expects:** `[longitude, latitude, altitude]` - ✓ CORRECT ORDER

**Verified Output:**
```
path: [[-122.418625, 37.802017, 0], [-122.41872, 37.802035, 0]]
Point 1: lon=-122.418625, lat=37.802017
✓ Coordinates in correct [lon, lat] order for SF
```

---

## STEP 2 – DEBUG LOGGING RESULTS

### San Francisco Path Analysis
```
Total points: 18
Bounding Box:
  Latitude:  37.802010 to 37.802115 (range: 0.000105°)
  Longitude: -122.420140 to -122.418625 (range: 0.001515°)
  Approx size: 12m (N-S) × 133m (E-W)
Total distance: 198.6 m

⚠️ Low point count (18) - may cause jagged/chunky appearance
Point spacing: min=8.6m, max=13.1m, avg=11.7m
```

### Nürburgring Path Analysis
```
Total points: 346
Bounding Box:
  Latitude:  50.316500 to 50.357000 (range: 0.040500°)
  Longitude: 6.924800 to 7.030500 (range: 0.105700°)
  Approx size: 4508m (N-S) × 7511m (E-W)
Total distance: 22658.3 m (22.66 km)

⚠️ Large gaps between points (max 93m) - interpolation may help
Curvature values: ALL ZEROS in first 5 points!
```

---

## STEP 3 – ROOT CAUSE ANALYSIS

### 1. Lat/lon swapped?
**NO** - We consistently use `[lon, lat, elevation]` in viz.py (line 370):
```python
'path': [[lon1, lat1, elevation], [lon2, lat2, elevation]]
```
Verified: SF coordinates are `-122.x, 37.x` which is correct `[lon, lat]`.

### 2. Degrees vs meters?
**NO** - Path data stays in degrees throughout. We never convert to UTM/meters for visualization.
The `x_m`, `y_m` fields are set to 0 and not used by viz.py.

### 3. Wrong CRS or base coordinates for Nürburgring?
**PARTIALLY** - The coordinates are approximately correct:
- Start point: 50.3353°N, 6.9418°E (matches real start/finish)
- Dimensions: 7.5km × 4.5km (close to real 7km × 5km)
- **BUT:** The hand-traced racing line doesn't perfectly follow the actual road geometry visible in satellite imagery.

### 4. Broken path construction for Lombard Street?
**YES - LOW POINT DENSITY:**
- Only 18 points for 8 hairpin turns (2.25 points per hairpin)
- Each hairpin rendered as 2-3 straight line segments
- Creates angular/jagged appearance instead of smooth curves

**NOT A COORDINATE SYSTEM ISSUE** - The coordinates are correct, but insufficient density creates the visual misalignment illusion.

---

## STEP 4 – SUMMARY

### Repo Overview (Recap)

The data flows through ApexVelocity as follows:

1. **Input Sources:**
   - Hardcoded GPS coordinates (SF, Nürburgring demos)
   - OSM via osmnx (CLI loader.py path)
   
2. **Analysis Pipeline:**
   ```
   path_points (lat/lon) → analyze_profile() → SegmentFeatures[] → ProfileAnalysis
   ```

3. **Visualization Pipeline:**
   ```
   SegmentFeatures → _build_path_data() → PathLayer segments → PyDeck HTML
   ```

4. **Coordinate Flow:**
   - All coordinates stay in WGS84 (lat/lon degrees)
   - viz.py correctly uses `[lon, lat, elev]` order for PyDeck
   - No CRS conversion bugs found

### SF Lombard Street – Findings

| Aspect | Status |
|--------|--------|
| Coordinate system | ✓ WGS84 lat/lon degrees |
| Lat/lon order | ✓ Correct [lon, lat] for PyDeck |
| Location accuracy | ✓ Matches real Lombard Street |
| Path continuity | ⚠️ 17 separate 2-point segments |

**Primary Cause of Misalignment:**
The path appears "slightly off" because:
1. **Low point density:** Only 18 points for 8 hairpin turns
2. **Angular segments:** Each hairpin is rendered as 2-3 straight lines instead of smooth curves
3. **Visual illusion:** Straight segments cutting across curved road geometry make the path look offset

**Why it's not a coordinate bug:** The first/last coordinates exactly match the real Hyde St / Leavenworth intersections.

### Nürburgring – Findings

| Aspect | Status |
|--------|--------|
| Coordinate system | ✓ WGS84 lat/lon degrees |
| Scale | ✓ 7.5km × 4.5km (close to real 7km × 5km) |
| Track length | ⚠️ 22.66 km (real: 20.83 km, ~9% too long) |
| Path smoothness | ⚠️ Linear interpolation creates straight segments |

**Primary Causes of Mismatch:**
1. **Hand-traced coordinates don't follow actual road geometry:** The 73 key corners are approximate racing line positions, not GPS traces of the actual road centerline.
2. **Linear interpolation:** Creates straight segments between corners instead of following road curves.
3. **Missing intermediate points:** 65m average spacing is too large for smooth rendering.
4. **9% length error:** Suggests the racing line traced doesn't match actual track geometry.

### Next Fix Steps (not implemented yet)

1. **SF Lombard Street:**
   - Increase point density to 80-120 points (10-15 per hairpin)
   - Add cubic spline interpolation in lat/lon space
   - OR: Fetch actual road geometry from OSM (way nodes) instead of hand-tracing

2. **Nürburgring:**
   - Replace hand-traced coordinates with actual OSM road geometry
   - Use osmnx to fetch the actual "Nürburgring" relation geometry
   - Apply curve-aware interpolation (Catmull-Rom spline) instead of linear

3. **General Improvements:**
   - Add `smooth_path()` function that interpolates using Catmull-Rom splines
   - Increase default point density to 1 point per 5-10m
   - Add debug overlay to show individual path points in visualization

4. **Visualization Enhancements:**
   - Add point markers at original coordinates for debugging
   - Add option to show "raw" vs "smoothed" path
   - Add coordinate validation warnings in viz output

5. **Automated QA:**
   - Add unit test that computes path length and compares to expected
   - Add test that verifies bounding box matches expected location
   - Add visualization regression tests using screenshot comparison

---

## Debug Code Location

Debug utilities added in `python/apexvelocity/debug_coords.py`:
- `print_path_debug(path_points, label)` - Analyze any path
- `print_viz_segment_debug(path_data, label)` - Analyze viz segments
- `check_lombard_coordinates()` - SF coordinate validation
- `check_nurburgring_scale()` - Ring scale validation

Run diagnostics:
```bash
cd python
python -m apexvelocity.debug_coords
```





