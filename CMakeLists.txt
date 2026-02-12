cmake_minimum_required(VERSION 3.5)
project(GRS_GroundStation)

set(CMAKE_CXX_STANDARD 20)

include(FetchContent)

# Required for mavsdk and casadi to exist together
add_compile_definitions(_GLIBCXX_USE_CXX11_ABI=1)

set(CASADI_DIR "${CMAKE_CURRENT_SOURCE_DIR}/casadi")
set(CMAKE_PREFIX_PATH "${CASADI_DIR}" ${CMAKE_PREFIX_PATH})

find_package(MAVSDK REQUIRED)
find_package(casadi CONFIG REQUIRED)

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
        source/Control/NMPCController.cpp
        source/Control/NMPCController.h
        #source/Control/solver.c
        #source/Control/solver.h
        #source/Control/solver_one_grounded.c
        #source/Control/solver_one_grounded.h
        source/Control/solver_param_weight.c
        source/Control/solver_param_weight.h
        source/Control/navigationFrameManager.cpp
        source/Control/navigationFrameManager.h
)

set(GRS_DEFINITION
        source/Definitions/communicationStructures.h
        source/Definitions/logDefinitions.h
        source/Definitions/yamlCustomStructure.h
        source/Definitions/controllerStructures.h
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

set(GRS_UTIL
        source/Util/profilingTimer.h
)

set(GRS_USERINPUTS
        source/Console/consoleInterface.cpp
        source/Console/consoleInterface.h
)

# Copy the Meshes used for the demo into the build directory
file(COPY "inputFilesExamples/" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/inputFilesExamples/")

add_executable(GRS_GroundStation
        ${GRS_SOURCE}
        ${GRS_COMMUNICATION}
        ${GRS_CONFIGURATION}
        ${GRS_CONTROL}
        ${GRS_DEFINITION}
        ${GRS_GEO}
        ${GRS_LOG}
        ${GRS_MATHEMATICS}
        ${GRS_POWERTRAIN}
        ${GRS_UTIL}
        ${GRS_USERINPUTS}
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
    FetchContent_Populate(yaml-cpp)
    add_subdirectory(${yaml-cpp_SOURCE_DIR} ${yaml-cpp_BINARY_DIR})
endif()

target_include_directories(GRS_GroundStation PUBLIC source)

find_library(BLASFEO_LIB blasfeo PATHS ${CASADI_DIR} REQUIRED)
find_library(FATROP_LIB fatrop PATHS ${CASADI_DIR} REQUIRED)

target_link_libraries(GRS_GroundStation
        yaml-cpp::yaml-cpp
        MAVSDK::mavsdk
        ${BLASFEO_LIB}
        ${FATROP_LIB}
        casadi::casadi
)
