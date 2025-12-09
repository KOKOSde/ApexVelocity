cmake_minimum_required(VERSION 3.20)
project(ApexVelocity VERSION 0.1.0 LANGUAGES CXX)

# ApexVelocity: High-Performance Physics-Based Graph Annotation Engine
# Transforms static OSM maps into dynamic kinematic graphs

# Enable testing at the top level so ctest can discover all tests
enable_testing()

# Build the core C++ library
add_subdirectory(core)

