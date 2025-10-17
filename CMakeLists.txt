cmake_minimum_required(VERSION 3.30)
project(GRS_GroundStation)

set(CMAKE_CXX_STANDARD 20)

# Required for mavsdk and casadi to exist together
add_compile_definitions(_GLIBCXX_USE_CXX11_ABI=1)

set(CASADI_DIR "${CMAKE_CURRENT_SOURCE_DIR}/casadi")
set(CMAKE_PREFIX_PATH "${CASADI_DIR}" ${CMAKE_PREFIX_PATH})

find_package(MAVSDK REQUIRED)
find_package(casadi CONFIG REQUIRED)

set(GRS_SOURCE
        source/main.cpp
        source/groundStationApp.cpp
        source/groundStationApp.h
)

set(GRS_COMMUNICATION
        source/Communication/communicationManager.cpp
        source/Communication/communicationManager.h
        source/Communication/guided.cpp
        source/Communication/guided.h
)

set(GRS_CONTROL
        source/Control/controlInterface.cpp
        source/Control/controlInterface.h
)

set(GRS_DEFINITION
        source/Definitions/communicationStructures.h
)

set(GRS_LOG
        source/Log/programLogger.cpp
        source/Log/programLogger.h
)

set(GRS_USERINPUTS
        source/UserInputs/commandHandler.cpp
        source/UserInputs/commandHandler.h
        source/UserInputs/userInputListener.cpp
        source/UserInputs/userInputListener.h
)

add_executable(GRS_GroundStation
        ${GRS_SOURCE}
        ${GRS_COMMUNICATION}
        ${GRS_CONTROL}
        ${GRS_DEFINITION}
        ${GRS_LOG}
        ${GRS_USERINPUTS}
)

target_include_directories(GRS_GroundStation PUBLIC source)

find_library(BLASFEO_LIB blasfeo PATHS ${CASADI_DIR} REQUIRED)
find_library(FATROP_LIB fatrop PATHS ${CASADI_DIR} REQUIRED)

target_link_libraries(GRS_GroundStation
        MAVSDK::mavsdk
        ${BLASFEO_LIB}
        ${FATROP_LIB}
        casadi::casadi
)
