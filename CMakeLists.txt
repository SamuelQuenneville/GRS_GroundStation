cmake_minimum_required(VERSION 3.5)
project(GRS_GroundStation)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_C_STANDARD 99)

# Default to Release build
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Enable Link Time Optimization
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)

# Global optimization flags
set(CMAKE_CXX_FLAGS_RELEASE
        "${CMAKE_CXX_FLAGS_RELEASE} \
    -O3 \
    -march=native \
    -mtune=native \
    -ffast-math \
    -fno-math-errno \
    -funsafe-math-optimizations \
    -DNDEBUG"
)

set(CMAKE_C_FLAGS_RELEASE
        "${CMAKE_C_FLAGS_RELEASE} \
    -O3 \
    -march=native \
    -mtune=native \
    -ffast-math \
    -fno-math-errno \
    -funsafe-math-optimizations \
    -DNDEBUG"
)

include(FetchContent)

# Required for mavsdk and casadi to exist together
add_compile_definitions(_GLIBCXX_USE_CXX11_ABI=1)

set(CASADI_DIR "${CMAKE_CURRENT_SOURCE_DIR}/casadi")
set(CMAKE_PREFIX_PATH "${CASADI_DIR}" ${CMAKE_PREFIX_PATH})

find_package(MAVSDK REQUIRED)
find_package(casadi CONFIG REQUIRED)

# RTK support (u-blox F9P base station) via PX4-GPSDrivers.
# Populated the same way as yaml-cpp below, but its own CMakeLists.txt only
# builds a standalone test executable we don't want, so we pull in the
# specific source files we need (gps_helper, ubx, rtcm) directly instead of
# add_subdirectory().
FetchContent_Declare(
        px4_gpsdrivers
        GIT_REPOSITORY https://github.com/PX4/PX4-GPSDrivers.git
        GIT_TAG 1b279865192dd61af62095744366466a0d39ffce            # Full SHA of commit on main on 2026-08-24 (no tag available)
)
FetchContent_GetProperties(px4_gpsdrivers)

if(NOT px4_gpsdrivers_POPULATED)
    message(STATUS "Fetching PX4-GPSDrivers...")
    FetchContent_MakeAvailable(px4_gpsdrivers)
endif()

set(GRS_GPSDRIVERS
        ${px4_gpsdrivers_SOURCE_DIR}/src/gps_helper.cpp
        ${px4_gpsdrivers_SOURCE_DIR}/src/gps_helper.h
        ${px4_gpsdrivers_SOURCE_DIR}/src/ubx.cpp
        ${px4_gpsdrivers_SOURCE_DIR}/src/ubx.h
        ${px4_gpsdrivers_SOURCE_DIR}/src/rtcm.cpp
        ${px4_gpsdrivers_SOURCE_DIR}/src/rtcm.h
)

set(GRS_SOURCE
        source/main.cpp
        source/gcs.cpp
        source/gcs.h
        source/gcsConfig.h
)

set(GRS_COMMUNICATION
        source/Communication/communicationManager.cpp
        source/Communication/communicationManager.h
        source/Communication/statesAggregator.cpp
        source/Communication/statesAggregator.h
        source/Communication/mavlinkMessageBuilder.cpp
        source/Communication/mavlinkMessageBuilder.h
        source/Communication/rtkBaseStation.cpp
        source/Communication/rtkBaseStation.h
        source/Communication/catapultLauncher.cpp
        source/Communication/catapultLauncher.h
)

set(GRS_CONFIGURATION
        source/Configuration/configurationParser.cpp
        source/Configuration/configurationParser.h

)

set(GRS_CONTROL
        source/Control/controlInterface.cpp
        source/Control/controlInterface.h
        source/Control/controlDispatcher.cpp
        source/Control/controlDispatcher.h
        source/Control/controller.h
        source/Control/mpcController.cpp
        source/Control/mpcController.h
        source/Control/SolverBackend/solverBackend.h
        source/Control/SolverBackend/solverBackendFactory.cpp
        source/Control/SolverBackend/solverBackendFactory.h
        source/Control/SolverBackend/oneUavNmpcBackend.cpp
        source/Control/SolverBackend/oneUavNmpcBackend.h
        source/Control/SolverBackend/twoUavPayloadNmpcBackend.cpp
        source/Control/SolverBackend/twoUavPayloadNmpcBackend.h
        source/Control/navigationFrameManager.cpp
        source/Control/navigationFrameManager.h
)

set(GRS_DASHBOARD
        source/Dashboard/httplib.h
        source/Dashboard/dashboardServer.cpp
        source/Dashboard/dashboardServer.h
        source/Dashboard/dashboardTypes.h
        source/Dashboard/jsonWriter.h
        source/Dashboard/jsonReader.h
        source/Dashboard/browserLauncher.h

)

set(GRS_DEFINITION
        source/Definitions/communicationStructures.h
        source/Definitions/logDefinitions.h
        source/Definitions/yamlCustomStructure.h
        source/Definitions/controllerStructures.h
        source/Definitions/gpsDefinitions.h
        source/Definitions/catapultProtocol.h
)

set(GRS_GEO
        source/Geo/geodeticConverter.cpp
        source/Geo/geodeticConverter.h

)

set(GRS_LOG
        source/Log/programLogger.cpp
        source/Log/programLogger.h
        source/Log/threadSafeQueue.h
        source/Log/logger.cpp
        source/Log/logger.h
)

set(GRS_MATHEMATICS
        source/Mathematics/vector.h
        source/Mathematics/matrix.h
        source/Mathematics/math.h
        source/Mathematics/mathUtils.h

)

set(GRS_POWERTRAIN
        source/Powertrain/powertrain.cpp
        source/Powertrain/powertrain.h
)

# Phase 0 of ADR-001: C++ port of trajectory_generation (MATLAB) -- payload
# path, aircraft path/takeoff, tether forces, inverse dynamics. Standalone
# (only depends on Mathematics/), so it links into the main binary without
# pulling in MAVSDK/CasADi headers.
set(GRS_TRAJECTORY
        source/Trajectory/trajectoryConfig.h
        source/Trajectory/trajectoryGenerator.h
        source/Trajectory/trajectoryGenerator.cpp
)

set(GRS_UTIL
        source/Util/profilingTimer.h
        source/Util/parseUtils.h
)

set(GRS_USERINPUTS
        source/Console/consoleInterface.cpp
        source/Console/consoleInterface.h
        source/Definitions/gpsDefinitions.h
)

# Copy input files
file(COPY "inputFilesExamples/" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/inputFilesExamples/")

# Copy dashboard web files
file(COPY "dashboard/" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/dashboard/")


add_executable(GRS_GroundStation
        ${GRS_SOURCE}
        ${GRS_COMMUNICATION}
        ${GRS_CONFIGURATION}
        ${GRS_CONTROL}
        ${GRS_DASHBOARD}
        ${GRS_DEFINITION}
        ${GRS_GEO}
        ${GRS_LOG}
        ${GRS_MATHEMATICS}
        ${GRS_POWERTRAIN}
        ${GRS_TRAJECTORY}
        ${GRS_UTIL}
        ${GRS_USERINPUTS}
        ${GRS_GPSDRIVERS}
)

# Standalone regression harness comparing TrajectoryGenerator's output
# against Octave-run MATLAB golden CSVs (source/Trajectory/tests/golden/).
# Only needs Mathematics/ + Trajectory/, so it builds without MAVSDK/CasADi.
add_executable(trajectory_generator_tests EXCLUDE_FROM_ALL
        source/Trajectory/tests/validate_trajectory.cpp
        ${GRS_TRAJECTORY}
        ${GRS_MATHEMATICS}
)
target_include_directories(trajectory_generator_tests PRIVATE source)

find_library(BLASFEO_LIB blasfeo PATHS ${CASADI_DIR} REQUIRED)
find_library(FATROP_LIB fatrop PATHS ${CASADI_DIR} REQUIRED)

# Fixed here, not in the generated files themselves (each is marked
# "do not hand-edit" and gets overwritten on every MATLAB re-export):
# per-source COMPILE_DEFINITIONS give each solver's copy of these symbols
# a solver-specific name via plain preprocessor token substitution, which
# safely renames both the definition site and every internal call site
# within that one translation unit -- purely mechanical, zero behavior
# change. Applied to every solver's .c file (including the first,
# solver_oneGround_nmpc.c) rather than leaving one "bare", so the scheme is
# symmetric and doesn't depend on link order or which solver happened to be
# added first.
#
# Add a new entry here (and a new NMPC_RUNTIME_SYMBOLS foreach block) for
# every future solver .c file added to this target -- re-derive the symbol
# list per-pair with the same nm diff if a CasADi/Fatrop version bump ever
# changes which helper functions are emitted this way.
set(NMPC_RUNTIME_SYMBOLS
        casadi_detect_bounds_after
        casadi_detect_bounds_before
        casadi_f0_checkout
        casadi_f0_free_mem
        casadi_f0_release
        casadi_fatrop_full_eval_lag_hess
        casadi_fatrop_solve
        casadi_max
        casadi_oracle_call
        fatrop_free_mem
        fatrop_init_mem
)

function(nmpc_solver_isolate_runtime_symbols source_file prefix)
    set(defs "")
    foreach(sym ${NMPC_RUNTIME_SYMBOLS})
        list(APPEND defs "${sym}=${prefix}_${sym}")
    endforeach()
    set_source_files_properties(${source_file} PROPERTIES COMPILE_DEFINITIONS "${defs}")
endfunction()

# Each solver gets its own prefix, including across NMPC/NMHE for the same
# vehicle config (oneGroundNmpc vs oneGroundNmhe, not both "oneGround") --
# reusing a prefix across two files would just recreate this exact
# collision one level up, between the two files' now-identically-renamed
# copies.
nmpc_solver_isolate_runtime_symbols(source/Control/CasadiSolver/solver_oneGround_nmpc.c oneGroundNmpc)
nmpc_solver_isolate_runtime_symbols(source/Control/CasadiSolver/solver_twoUavPayload_nmpc.c twoUavPayloadNmpc)
nmpc_solver_isolate_runtime_symbols(source/Control/CasadiSolver/nmhe_oneGround.c oneGroundNmhe)
nmpc_solver_isolate_runtime_symbols(source/Control/CasadiSolver/nmhe_twoUavPayload.c twoUavPayloadNmhe)

add_library(nmpc_solver STATIC
        source/Control/CasadiSolver/solver_oneGround_nmpc.c
        source/Control/CasadiSolver/solver_oneGround_nmpc.h
        source/Control/CasadiSolver/solver_twoUavPayload_nmpc.c
        source/Control/CasadiSolver/solver_twoUavPayload_nmpc.h
        source/Control/CasadiSolver/nmhe_oneGround.c
        source/Control/CasadiSolver/nmhe_oneGround.h
        source/Control/CasadiSolver/nmhe_twoUavPayload.c
        source/Control/CasadiSolver/nmhe_twoUavPayload.h
)

target_compile_options(nmpc_solver PRIVATE
        -O3
        -march=native
        -mtune=native
        -ffast-math
        -funroll-loops
)

target_link_libraries(nmpc_solver PRIVATE
        ${FATROP_LIB}
        ${BLASFEO_LIB}
        casadi::casadi
)

# Include libraries
# YAML
FetchContent_Declare(
        yaml-cpp
        GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
        GIT_TAG yaml-cpp-0.7.0 # Can be a tag (yaml-cpp-x.x.x), a commit hash, or a branch name (master)
)
FetchContent_GetProperties(yaml-cpp)

if(NOT yaml-cpp_POPULATED)
    message(STATUS "Fetching yaml-cpp...")
    FetchContent_MakeAvailable(yaml-cpp)
endif()

target_include_directories(GRS_GroundStation PUBLIC source)
target_include_directories(GRS_GroundStation PRIVATE ${px4_gpsdrivers_SOURCE_DIR}/src)

# PX4-GPSDrivers requires the consuming project to supply its own definitions.h
# (sensor_gps_s, satellite_info_s, sensor_gnss_relative_s, and
# the GPS_INFO/GPS_WARN/GPS_ERR logging macros). Resolved relative to the "source" include dir added above,
# so this points at source/Definitions/gpsDefinitions.h.
target_compile_definitions(GRS_GroundStation PRIVATE GPS_DEFINITIONS_HEADER="Definitions/gpsDefinitions.h")

target_link_libraries(GRS_GroundStation
        nmpc_solver
        yaml-cpp::yaml-cpp
        MAVSDK::mavsdk
)
