cmake_minimum_required(VERSION 3.16)
project(StewartSim_V2 VERSION 2.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
    add_compile_options(/W4)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(SOURCES
    main.cpp
    actuator/MotorDriver.cpp
    actuator/ActuatorState.cpp
    geometry/Geometry.cpp
    geometry/Kinematics.cpp
    platform/StewartPlatform.cpp
    platform/SafetyMonitor.cpp
    threading/ThreadedFOCDriver.cpp
    threading/ThreadedController.cpp
    threading/ThreadedSafetyMonitor.cpp
    threading/Logger.cpp
)

add_executable(StewartSim_V2 ${SOURCES})
target_include_directories(StewartSim_V2 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

if(MSVC)
    target_compile_options(StewartSim_V2 PRIVATE /W4)
endif()

# Link threading library on Linux
if(UNIX)
    target_link_libraries(StewartSim_V2 PRIVATE pthread m)
endif()