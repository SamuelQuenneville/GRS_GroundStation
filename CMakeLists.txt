cmake_minimum_required(VERSION 3.30)
project(GRS_GroundStation)

set(CMAKE_CXX_STANDARD 20)

find_package(MAVSDK REQUIRED)

set(GRS_SOURCE
        source/main.cpp
        source/groundStationApp.cpp
        source/groundStationApp.h
        source/communicationManager.cpp
        source/communicationManager.h
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
        ${GRS_CONTROL}
        ${GRS_DEFINITIONS}
        ${GRS_LOG}
        ${GRS_USERINPUTS}
)

target_include_directories(GRS_GroundStation PUBLIC source)

target_link_libraries(GRS_GroundStation MAVSDK::mavsdk)
