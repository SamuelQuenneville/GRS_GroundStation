cmake_minimum_required(VERSION 3.30)
project(GRS_GroundStation)

set(CMAKE_CXX_STANDARD 20)

find_package(MAVSDK REQUIRED)

set(GRS_SOURCE
        source/main.cpp
        source/userInputListener.cpp
        source/userInputListener.h
        source/groundStationApp.cpp
        source/groundStationApp.h
        source/commandHandler.cpp
        source/commandHandler.h
)

set(GRS_LOG
        source/Log/programLogger.cpp
        source/Log/programLogger.h
)

add_executable(GRS_GroundStation
        ${GRS_SOURCE}
        ${GRS_LOG}
)

target_include_directories(GRS_GroundStation PUBLIC source)

target_link_libraries(GRS_GroundStation MAVSDK::mavsdk)
