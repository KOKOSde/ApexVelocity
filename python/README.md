# ApexVelocity Python Package

Python bindings for the ApexVelocity physics-based graph annotation engine.

## Installation

### From Source (Development)

1. Build the C++ core with Python bindings:

```bash
cd ApexVelocity/core
mkdir build && cd build
cmake .. -DAPEXVELOCITY_BUILD_PYTHON=ON
cmake --build . -j$(nproc)
```

2. Copy the built module to the Python package:

```bash
cp _apexvelocity_core*.so ../python/apexvelocity/
```

3. Install in editable mode:

```bash
cd ../python
pip install -e .
```

## Quick Start

```python
import apexvelocity as av

# Create a simple path
geometry = [
    [0.0, 0.0, 0.0, 0.0, 0.0],      # [x, y, z, curvature, distance]
    [50.0, 0.0, 0.0, 0.01, 50.0],   # slight curve
    [100.0, 0.0, 0.0, 0.0, 100.0],  # straight
]
surfaces = ["asphalt", "asphalt", "asphalt"]

# Solve velocity profile
result = av.solve_profile(
    geometry=geometry,
    surfaces=surfaces,
    vehicle="tesla_model_3",
    condition="dry",
    initial_speed=20.0,
)

print(f"Max speed: {result['max_speed_mps']:.1f} m/s")
print(f"Total energy: {result['total_energy_joules']/3600000:.2f} kWh")
```

## Custom Friction Callback

Override friction coefficients dynamically:

```python
import apexvelocity as av

def my_friction(surface: str, condition: str) -> float:
    """Custom friction for wet conditions."""
    if surface == "asphalt" and condition == "wet":
        return 0.4  # Very wet
    return 0.8  # Default

av.set_friction_callback(my_friction)

# Now solve_profile will use your callback
result = av.solve_profile(...)

# Clear when done
av.clear_friction_callback()
```

## Gymnasium Environment (Experimental)

```python
import gymnasium as gym
from apexvelocity.envs import ApexVelocityEnv

# Create environment
env = ApexVelocityEnv(
    path_length=1000.0,
    vehicle="compact_car",
    condition="dry",
)

# Reset and run
obs, info = env.reset()
for _ in range(100):
    action = env.action_space.sample()  # Random action
    obs, reward, terminated, truncated, info = env.step(action)
    if terminated or truncated:
        break

env.close()
```

⚠️ The RL environment is a **simplified demo**. It uses synthetic paths and a
lightweight internal physics model, and does **not** yet integrate with the
OSM loader or the C++ core. API and behavior may change in future 1.x releases.

## API Reference

### Core Functions

- `solve_profile(geometry, surfaces, vehicle, condition, initial_speed, final_speed)` - Solve velocity profile
- `set_friction_callback(fn)` - Set custom friction callback
- `clear_friction_callback()` - Clear friction callback
- `get_effective_mu(surface, condition)` - Get friction coefficient
- `calc_friction_limited_speed(curvature, mu, gravity)` - Calculate max cornering speed
- `calc_rollover_limited_speed(curvature, track_width, cog_height, gravity)` - Calculate rollover limit

### Types

- `VehicleParams` - Vehicle physical parameters
- `PathPoint` - Point along a path
- `SolverConfig` - Solver configuration
- `MaterialProps` - Material friction properties

## Running Tests

```bash
cd python
pytest tests/ -v
```





