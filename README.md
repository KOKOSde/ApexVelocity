<p align="center">
  <h1 align="center">🏎️ ApexVelocity</h1>
  <p align="center">
    <strong>Physics-Based Graph Annotation Engine</strong>
  </p>
  <p align="center">
    Transform static road networks into dynamic kinematic graphs with physics-accurate velocity profiles and energy predictions.
  </p>
</p>

<p align="center">
  <a href="#features">Features</a> •
  <a href="#quick-start">Quick Start</a> •
  <a href="#installation">Installation</a> •
  <a href="#usage">Usage</a> •
  <a href="#architecture">Architecture</a> •
  <a href="#benchmarks">Benchmarks</a>
</p>

---

![Demo](assets/demo.png)

## Overview

ApexVelocity is a high-performance physics engine that annotates road networks with realistic velocity profiles and energy consumption estimates. It combines:

- **Real Physics**: Friction-limited cornering, rollover prevention, power/braking constraints
- **Material Awareness**: Different surfaces (asphalt, gravel, cobblestone) affect grip and rolling resistance
- **Energy Modeling**: Accurate energy predictions including aerodynamic drag, grade resistance, and regenerative braking potential
- **Configurable Universe**: Swap between Earth, Mars, or custom physics constants

## Features

✅ **Physics-Based Velocity Profiling**
- 3-pass kinematic solver (static limits → backward braking → forward acceleration)
- Friction-limited cornering speeds from road curvature
- Vehicle-specific power and braking constraints

✅ **Energy Consumption Modeling**
- Aerodynamic drag: `F = ½ρCdAv²`
- Rolling resistance with surface-specific coefficients
- Grade resistance for hills and elevation changes

✅ **Interactive 3D Visualization**
- Dark-themed maps with glowing velocity tubes
- Height extrusion shows lateral G-force danger zones
- Rich tooltips with physics data

✅ **Plug-and-Play CLI**
- Analyze any city with a single command
- Automatic OSM data fetching and processing

## Quick Start

**No API keys required!** ApexVelocity works out of the box with free OpenStreetMap tiles.

```bash
# Analyze any city (uses free OSM basemap)
python -m apexvelocity.cli --city "Tokyo, Japan"

# Run the San Francisco benchmark
python examples/demo_san_francisco.py

# Run the Nürburgring wet/dry comparison
python examples/benchmark_nurburgring.py
```

### Optional: Satellite Imagery

For high-quality satellite maps, get a free Mapbox token:

1. Sign up at [mapbox.com](https://account.mapbox.com/auth/signup/)
2. Copy your public token from [account.mapbox.com/access-tokens/](https://account.mapbox.com/access-tokens/)
3. Set the environment variable:
   ```bash
   export MAPBOX_API_KEY='pk.your_token_here'
   ```
4. Run with `--style satellite`:
   ```bash
   python -m apexvelocity.cli --city "Manhattan, NYC" --style satellite
   ```

If you do **not** set `MAPBOX_API_KEY`:

- The CLI and visualization APIs will still run and will automatically fall back
  to free styles (e.g., `osm`, `dark`, or `light`).
- You will see a **warning** in the console explaining that satellite imagery
  is unavailable and that a free Mapbox token can be added later.

## Installation

### Prerequisites

- Python 3.8+
- CMake 3.16+ (for C++ core)
- C++20 compiler (GCC 10+, Clang 12+)

### Python Package (Recommended)

```bash
cd python
pip install -e .
```

### Build C++ Core

```bash
cd core
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
ctest --output-on-failure
```

### Full Installation

```bash
# Clone the repository
git clone https://github.com/your-org/ApexVelocity.git
cd ApexVelocity

# Install Python dependencies
pip install -r requirements.txt

# Build C++ core and Python bindings
cd core && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

# Install Python package
cd ../../python && pip install -e .
```

## Usage

### Command-Line Interface

```bash
# Basic usage
python -m apexvelocity.cli --city "San Francisco, CA"

# With options
python -m apexvelocity.cli \
    --city "Manhattan, New York City, USA" \
    --vehicle tesla_model_3 \
    --condition wet \
    --mode energy \
    --output nyc_analysis.html

# Available vehicles: tesla_model_3, compact_car, sports_car, suv
# Visualization modes: speed, energy, gforce
# Conditions: dry, wet
```

CLI solver path:
- Uses the high-performance C++ core solver by default (via Python bindings).
- If the native extension is unavailable, it falls back to a simplified Python
  solver and emits a warning. Results may differ slightly from the core.

### Python API (Stable 1.x Surface)

```python
from apexvelocity import solve_profile, VehicleLoader
from apexvelocity.loader import load_path_from_place
from apexvelocity.viz import visualize_profile

# Load a route
path = load_path_from_place("San Francisco, California")

# Get vehicle parameters
vehicle = VehicleLoader.load_vehicle_params("tesla_model_3")

# Solve velocity profile
geometry, surfaces = path.to_geometry_list()
result = solve_profile(geometry, surfaces, vehicle, condition="dry")

# Visualize
visualize_profile(path.points, mode="speed", output_html="analysis.html")
```

Data loading:
- For programmatic ingestion, prefer the stable 1.x APIs:
  - `RouteRequest` + `build_route` (canonical start→end routing on top of OSM)
  - `solve` / `solve_profile` (C++ core velocity/energy solver)
  - `analyze_profile` (rich segment‑level AV features)
  - `convert_segments_for_viz` (dicts for visualization/ML)
- `apexvelocity.osm_fetcher` is **experimental** and primarily used by bundled
  examples. Its API may change in minor 1.x releases.

### Configuration

ApexVelocity uses YAML/JSON configuration files in the `config/` directory:

```yaml
# config/simulation.yaml
gravity: 9.81           # m/s² (use 3.71 for Mars!)
air_density: 1.225      # kg/m³
step_size_meters: 1.0
enable_rollover_checks: true
global_friction_multiplier: 1.0  # Reduce for wet/icy conditions
```

```json
{
  "asphalt": { "mu_dry": 0.90, "mu_wet": 0.60, "rolling_resistance_coeff": 0.015 },
  "cobblestone": { "mu_dry": 0.55, "mu_wet": 0.40, "rolling_resistance_coeff": 0.025 },
  "gravel": { "mu_dry": 0.60, "mu_wet": 0.45, "rolling_resistance_coeff": 0.020 }
}
```

## Versioning & Stability

ApexVelocity follows **semantic versioning**:

- **Stable in 1.x (Python)**:
  - `RouteRequest`, `build_route`
  - `solve`, `solve_profile`
  - `analyze_profile`, `ProfileAnalysis`, `SegmentFeatures`, `convert_segments_for_viz`
  - `visualize_profile`, `visualize_profile_interactive`, `visualize_comparison`
  - `export_routes_to_parquet` (dataset exporter)
- **Experimental (subject to change in 1.x)**:
  - `apexvelocity.osm_fetcher` (Overpass helpers and special‑case tracks)
  - `apexvelocity.envs` (Gymnasium / RL environments)
  - Example scripts under `python/examples/`

Python support:
- The published package targets **Python 3.10+** (see `pyproject.toml`).

## Architecture

```
ApexVelocity/
├── core/                    # C++20 Physics Engine
│   ├── include/
│   │   ├── physics/         # PhysicsMath, VelocityProfileSolver
│   │   ├── utils/           # ConfigManager
│   │   └── capi/            # C API for FFI
│   ├── src/
│   └── tests/               # GoogleTest suite
│
├── python/                  # Python Interface
│   ├── apexvelocity/
│   │   ├── api.py           # High-level Python API
│   │   ├── loader.py        # OSM data loading
│   │   ├── viz.py           # PyDeck visualization
│   │   ├── cli.py           # Command-line interface
│   │   └── envs/            # Gymnasium RL environment
│   └── examples/
│       ├── demo_san_francisco.py
│       └── benchmark_nurburgring.py
│
├── server/                  # Go REST API (optional)
│   └── cmd/apex-server/
│
└── config/                  # Configuration files
    ├── simulation.yaml
    ├── materials.json
    ├── tag_mapping.json
    └── vehicle_presets/
```

## Benchmarks

### San Francisco Hill Climb

Tests the energy model on steep gradients (Lombard Street area):

```
Vehicle: Tesla Model 3 (1847 kg)

UPHILL:
  Energy consumed: 1.36 kWh
  Hairpin corners: Speed drops to 24 km/h

DOWNHILL:
  Energy consumed: 0.97 kWh
  Regen potential: 0.25 kWh

✓ PASS: Uphill energy > Downhill energy (ratio: 1.4x)
```

### Nürburgring Wet vs Dry

Tests the friction model under varying conditions:

```
Vehicle: High-Performance Sports Car (350 kW)

DRY:  Lap time 14.2 min, Max speed 214 km/h
WET:  Lap time 15.9 min, Max speed 202 km/h

✓ PASS: Wet 11.9% slower (friction model validated)
```

## Physics Model

### Friction-Limited Cornering Speed

```
v_max = √(μ · g / κ)

where:
  μ = friction coefficient (surface + condition)
  g = gravitational acceleration
  κ = curvature (1/radius)
```

### Energy Per Segment

```
E = (F_aero + F_roll + F_grade) · Δs

F_aero = ½ρ · Cd · A · v²
F_roll = Crr · m · g · cos(θ)
F_grade = m · g · sin(θ)
```

### Velocity Profile Solver (3-Pass)

1. **Static Limits**: Compute max speed at each point from curvature + friction
2. **Backward Pass**: Propagate braking constraints from end to start
3. **Forward Pass**: Propagate acceleration constraints from start to end

## Contributing

Contributions are welcome! Please read our contributing guidelines and submit PRs.

### Publishing to Your Own GitHub Repository

To publish ApexVelocity under your own GitHub account:

1. **Create a new empty repository** on GitHub (e.g., `USERNAME/ApexVelocity`).
2. **Add the remote** in your local clone:
   ```bash
   git remote add origin git@github.com:USERNAME/ApexVelocity.git
   # or, using HTTPS:
   # git remote add origin https://github.com/USERNAME/ApexVelocity.git
   ```
3. **Create a Personal Access Token (PAT)** if you use HTTPS:
   - Go to GitHub → **Settings → Developer settings → Personal access tokens**.
   - Generate a token with `repo` scope.
   - When pushing over HTTPS, use your **GitHub username** and the PAT as the
     **password** when prompted.
4. **Push the code and tags**:
   ```bash
   git push -u origin master        # or main, depending on your branch name
   git push origin v1.0.0           # push the 1.0.0 tag
   ```

Tokens and credentials are **never** stored in this repository; you manage them
locally via your own Git configuration or credential helper.

## License

MIT License - see [LICENSE](LICENSE) for details.

## Acknowledgments

- OpenStreetMap contributors for road network data
- OSMnx for Python OSM interface
- PyDeck/Mapbox for visualization

---

<p align="center">
  Made with ⚡ for realistic road physics simulation
</p>
