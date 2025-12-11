# Changelog

All notable changes to ApexVelocity will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- Advanced physics: Pacejka tire model (TireModel.h/cpp) with Magic Formula implementation
- Advanced physics: Dynamic vehicle model with weight transfer and actuator lag (DynamicVehicle.h/cpp)
- Physics model selection: `PhysicsModel::KINEMATIC` (fast, 3-pass) vs `PhysicsModel::DYNAMIC` (tire model)
- nuScenes validation framework (python/validation/validate_nuscenes.py)
- Uncertainty quantification module with ML-based confidence intervals (python/apexvelocity/uncertainty.py)
- Comprehensive physics validation test suite (test_physics_validation.py)
- Thread pool for batch processing (utils/ThreadPool.h/cpp)
- Docker containerization support
- GitHub Actions CI/CD pipeline with C++, Python, Go, and Docker tests
- API authentication and rate limiting with bcrypt
- Comprehensive structured logging (Zap for Go, Python logging)
- Prometheus metrics endpoint (/metrics)
- OpenAPI/Swagger documentation (/docs)
- Kubernetes Helm charts for production deployment
- ROS2 integration package (ros2_ws/src/apexvelocity_ros2/)

### Fixed
- **CRITICAL**: Curvature calculation in create_path() - was always 0, causing unrealistic speeds (300 km/h everywhere)
- **CRITICAL**: Energy consumption reduced from 700 Wh/km to realistic 150-200 Wh/km by enforcing configurable velocity caps
- **CRITICAL**: Wet vs dry friction now properly affects cornering speeds (~10-15% slowdown in wet)
- Hill energy asymmetry validated - uphill correctly requires more energy than downhill
- Velocity cap now enforced in all 3 solver passes (static, backward, forward)
- Nürburgring OSM fetch now validates geometry sanity and falls back gracefully
- Safety margin color mapping in visualization now derives from friction usage

### Changed
- Default max_velocity_cap reduced from 300 km/h to 120 km/h for realistic highway speeds
- Python API `solve()` now accepts `physics_model` parameter ("kinematic" or "dynamic")
- `create_path()` now auto-computes curvature from coordinates if not provided
- VehicleParams extended with `wheelbase_m` and `front_weight_fraction` for dynamic model
- SolverConfig extended with `model`, `enable_tire_slip`, `enable_weight_transfer` flags
- Advanced physics: Pacejka tire model (`TireModel.h/.cpp`)
- Advanced physics: Dynamic vehicle model with weight transfer (`DynamicVehicle.h/.cpp`)
- nuScenes validation framework (`python/validation/validate_nuscenes.py`)
- Uncertainty quantification module (`python/apexvelocity/uncertainty.py`)
- Comprehensive physics validation tests for curvature, hills, and energy

### Fixed
- Curvature calculation in `create_path()` so solver sees realistic curvature values
- Hill energy asymmetry (uphill now correctly requires more energy than downhill)
- Energy consumption values on flats (brought into ~realistic 150–200 Wh/km range when run at highway speeds)
- Wet vs dry friction now properly affects cornering speeds

## [1.0.0] - 2025-12-10

### Added
- C++20 physics core with velocity profile solver
- Python bindings with pybind11
- Go HTTP REST API server
- OSM data loading and routing
- 3D visualization with PyDeck
- San Francisco and Nürburgring benchmarks
- Vehicle presets (Tesla Model 3, compact car, sports car, SUV)
- Material-aware friction modeling
- Energy consumption estimation
- RL environment (experimental)

### Documentation
- Comprehensive README
- Python package README
- API usage examples
- Physics model documentation

[Unreleased]: https://github.com/KOKOSde/ApexVelocity/compare/v1.0.0...HEAD
[1.0.0]: https://github.com/KOKOSde/ApexVelocity/releases/tag/v1.0.0


