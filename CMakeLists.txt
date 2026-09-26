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

# GRS_BUILD_GCS=OFF builds only the MAVSDK-free parts (grs_core,
# grs_batchsim, tests), e.g. on a machine without MAVSDK or on a compute
# server running Monte Carlo campaigns.
option(GRS_BUILD_GCS "Build the GRS_GroundStation executable (needs MAVSDK)" ON)
option(GRS_BUILD_BATCHSIM "Build grs_batchsim, the closed-loop batch simulator" ON)

find_package(casadi CONFIG REQUIRED)
if(GRS_BUILD_GCS)
    find_package(MAVSDK REQUIRED)
endif()

set(GRS_SOURCE
        source/main.cpp
        source/gcs.cpp
        source/gcs.h
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

# Controller core: everything a control tick needs, no MAVSDK. Shared by
# the GCS and grs_batchsim through the grs_core library below.
set(GRS_CONTROL_CORE
        source/Control/controller.h
        source/Control/controlStep.cpp
        source/Control/controlStep.h
        source/Control/estimatorRunner.cpp
        source/Control/estimatorRunner.h
        source/Control/stateVector.cpp
        source/Control/stateVector.h
        source/Control/mpcController.cpp
        source/Control/mpcController.h
        source/Control/estimator.h
        source/Control/nmheEstimator.cpp
        source/Control/nmheEstimator.h
        source/Control/nlpsol.cpp
        source/Control/nlpsol.h
)

set(GRS_CONTROL
        source/Control/controlInterface.cpp
        source/Control/controlInterface.h
        source/Control/controlDispatcher.cpp
        source/Control/controlDispatcher.h
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

set(GRS_DEFINITION_CORE
        source/Definitions/vehicleStructures.h
        source/Definitions/logDefinitions.h
        source/Definitions/yamlCustomStructure.h
        source/Definitions/controllerStructures.h
        source/Definitions/catapultProtocol.h
        source/gcsConfig.h
)

set(GRS_DEFINITION
        source/Definitions/communicationStructures.h
        source/Definitions/gpsDefinitions.h
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

# grs_batchsim: truth plant (C++ port of the MATLAB models), Monte Carlo
# sample mapping, closed-loop runner. See source/Simulation/batchSimMain.cpp.
set(GRS_SIMULATION
        source/Simulation/plantModel.cpp
        source/Simulation/plantModel.h
        source/Simulation/truthSample.cpp
        source/Simulation/truthSample.h
        source/Simulation/closedLoopRunner.cpp
        source/Simulation/closedLoopRunner.h
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

# ---------------------------------------------------------------------------
# grs_core: controller core + configuration + logging + trajectory generator,
# no MAVSDK. Linked by the GCS and by grs_batchsim, so both run the exact
# same controller/estimator code.
# ---------------------------------------------------------------------------
add_library(grs_core STATIC
        ${GRS_CONFIGURATION}
        ${GRS_CONTROL_CORE}
        ${GRS_DEFINITION_CORE}
        ${GRS_LOG}
        ${GRS_MATHEMATICS}
        ${GRS_TRAJECTORY}
        ${GRS_UTIL}
)
target_include_directories(grs_core PUBLIC source)
# The global -ffast-math implies -ffinite-math-only, which lets GCC fold
# std::isnan()/std::isfinite() to constants: MpcController/NmheEstimator's
# m_solutionIsValid() NaN checks and grs_batchsim's NaN-aware metrics were
# silently compiled out. Keep the rest of fast-math, restore NaN/Inf
# semantics for the controller core and the batch sim (PUBLIC so it
# propagates to both executables). The generated solvers keep full
# fast-math (their own target_compile_options).
target_compile_options(grs_core PUBLIC -fno-finite-math-only)
find_package(Threads REQUIRED)  # ThreadedEstimatorRunner (NMHE thread)
target_link_libraries(grs_core PUBLIC
        nmpc_solver
        yaml-cpp::yaml-cpp
        Threads::Threads
)

if(GRS_BUILD_GCS)
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

    add_executable(GRS_GroundStation
            ${GRS_SOURCE}
            ${GRS_COMMUNICATION}
            ${GRS_CONTROL}
            ${GRS_DASHBOARD}
            ${GRS_DEFINITION}
            ${GRS_GEO}
            ${GRS_POWERTRAIN}
            ${GRS_USERINPUTS}
            ${GRS_GPSDRIVERS}
    )

    target_include_directories(GRS_GroundStation PUBLIC source)
    target_include_directories(GRS_GroundStation PRIVATE ${px4_gpsdrivers_SOURCE_DIR}/src)

    # PX4-GPSDrivers requires the consuming project to supply its own definitions.h
    # (sensor_gps_s, satellite_info_s, sensor_gnss_relative_s, and
    # the GPS_INFO/GPS_WARN/GPS_ERR logging macros). Resolved relative to the "source" include dir added above,
    # so this points at source/Definitions/gpsDefinitions.h.
    target_compile_definitions(GRS_GroundStation PRIVATE GPS_DEFINITIONS_HEADER="Definitions/gpsDefinitions.h")

    target_link_libraries(GRS_GroundStation
            grs_core
            MAVSDK::mavsdk
    )
endif()

if(GRS_BUILD_BATCHSIM)
    add_executable(grs_batchsim
            source/Simulation/batchSimMain.cpp
            ${GRS_SIMULATION}
    )
    target_link_libraries(grs_batchsim grs_core)
endif()

# Truth-plant port vs golden data evaluated by the MATLAB models
# (GRS_Controller: 06_comparison/cpp_batchsim/export_plant_golden.m).
# Usage: plant_model_tests <folder with plant_golden_*.csv>
add_executable(plant_model_tests EXCLUDE_FROM_ALL
        source/Simulation/plantModel.cpp
        source/Simulation/plantModel.h
)
target_include_directories(plant_model_tests PRIVATE source)
