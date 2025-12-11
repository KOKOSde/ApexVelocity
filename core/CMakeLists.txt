cmake_minimum_required(VERSION 3.20)
project(ApexVelocityCore VERSION 0.1.0 LANGUAGES CXX)

# ============================================================================
# C++20 Standard
# ============================================================================
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compiler warnings
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============================================================================
# Options
# ============================================================================
option(BUILD_SHARED_LIBS "Build shared libraries" OFF)
option(APEXVELOCITY_BUILD_TESTS "Build tests" ON)
option(APEXVELOCITY_BUILD_PYTHON "Build Python bindings" OFF)
option(APEXVELOCITY_BUILD_CAPI "Build C API shared library" OFF)

# ============================================================================
# Dependencies - FetchContent for yaml-cpp, nlohmann_json, and GoogleTest
# ============================================================================
include(FetchContent)

# yaml-cpp for YAML parsing
FetchContent_Declare(
    yaml-cpp
    GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
    GIT_TAG 0.8.0
)
set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_CONTRIB OFF CACHE BOOL "" FORCE)
set(YAML_CPP_INSTALL OFF CACHE BOOL "" FORCE)

# Enable PIC globally for Python module compatibility
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# nlohmann_json for JSON parsing
FetchContent_Declare(
    nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG v3.11.3
)

FetchContent_MakeAvailable(yaml-cpp nlohmann_json)

# ============================================================================
# ApexVelocity Core Library
# ============================================================================
set(APEXVELOCITY_SOURCES
    src/utils/ConfigManager.cpp
    src/physics/PhysicsMath.cpp
    src/physics/VehicleLoader.cpp
    src/physics/TireModel.cpp
    src/physics/DynamicVehicle.cpp
    src/physics/VelocityProfileSolver.cpp
    src/capi/apex_c_api.cpp
)

set(APEXVELOCITY_HEADERS
    include/utils/ConfigManager.h
    include/physics/PhysicsMath.h
    include/physics/Types.h
    include/physics/VehicleLoader.h
    include/physics/TireModel.h
    include/physics/DynamicVehicle.h
    include/physics/VelocityProfileSolver.h
    include/capi/apex_c_api.h
)

add_library(apexvelocity_core ${APEXVELOCITY_SOURCES} ${APEXVELOCITY_HEADERS})

# Enable position-independent code for shared library linking
set_target_properties(apexvelocity_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(apexvelocity_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(apexvelocity_core
    PUBLIC
        yaml-cpp::yaml-cpp
        nlohmann_json::nlohmann_json
)

# For GCC < 9, we need to explicitly link stdc++fs for std::filesystem
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9)
    target_link_libraries(apexvelocity_core PUBLIC stdc++fs)
endif()

# Define config directory path for the library
target_compile_definitions(apexvelocity_core
    PUBLIC
        APEXVELOCITY_DEFAULT_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../config"
)

# ============================================================================
# Tests
# ============================================================================
if(APEXVELOCITY_BUILD_TESTS)
    enable_testing()
    
    # GoogleTest
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG v1.14.0
    )
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
    
    include(GoogleTest)
    
    # Test executable
    add_executable(test_config_physics
        tests/test_config_physics.cpp
    )
    
    target_link_libraries(test_config_physics
        PRIVATE
            apexvelocity_core
            GTest::gtest
            GTest::gtest_main
    )
    
    # Define test config directory
    target_compile_definitions(test_config_physics
        PRIVATE
            TEST_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../config"
    )
    
    # Register tests with CTest - using add_test for reliability
    add_test(NAME ConfigPhysicsTests COMMAND test_config_physics)
    
    # Phase 2: Solver Physics Tests
    add_executable(test_solver_physics
        tests/test_solver_physics.cpp
    )
    
    target_link_libraries(test_solver_physics
        PRIVATE
            apexvelocity_core
            GTest::gtest
            GTest::gtest_main
    )
    
    target_compile_definitions(test_solver_physics
        PRIVATE
            TEST_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../config"
    )
    
    add_test(NAME SolverPhysicsTests COMMAND test_solver_physics)
endif()

# ============================================================================
# Installation (optional)
# ============================================================================
install(TARGETS apexvelocity_core
    EXPORT ApexVelocityCoreTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(DIRECTORY include/
    DESTINATION include
    FILES_MATCHING PATTERN "*.h"
)

# ============================================================================
# C API Shared Library (for Go CGO)
# ============================================================================
if(APEXVELOCITY_BUILD_CAPI)
    add_library(apexvelocity_capi SHARED ${APEXVELOCITY_SOURCES})
    
    target_include_directories(apexvelocity_capi
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
    )
    
    target_link_libraries(apexvelocity_capi
        PUBLIC
            yaml-cpp::yaml-cpp
            nlohmann_json::nlohmann_json
    )
    
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9)
        target_link_libraries(apexvelocity_capi PUBLIC stdc++fs)
    endif()
    
    target_compile_definitions(apexvelocity_capi
        PUBLIC
            APEXVELOCITY_DEFAULT_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../config"
    )
    
    set_target_properties(apexvelocity_capi PROPERTIES
        OUTPUT_NAME "apexvelocity"
        VERSION ${PROJECT_VERSION}
        SOVERSION 0
    )
    
    install(TARGETS apexvelocity_capi
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
    )
endif()

# ============================================================================
# Python Bindings (pybind11)
# ============================================================================
if(APEXVELOCITY_BUILD_PYTHON)
    # Find or fetch pybind11
    find_package(pybind11 QUIET)
    if(NOT pybind11_FOUND)
        FetchContent_Declare(
            pybind11
            GIT_REPOSITORY https://github.com/pybind/pybind11.git
            GIT_TAG v2.11.1
        )
        FetchContent_MakeAvailable(pybind11)
    endif()
    
    pybind11_add_module(_apexvelocity_core
        src/bindings/bindings.cpp
    )
    
    target_link_libraries(_apexvelocity_core
        PRIVATE
            apexvelocity_core
    )
    
    target_compile_definitions(_apexvelocity_core
        PRIVATE
            APEXVELOCITY_DEFAULT_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../config"
    )
    
    # Install to Python package directory
    install(TARGETS _apexvelocity_core
        LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/python/apexvelocity
    )
endif()

