cmake_minimum_required(VERSION 3.16)
project(StewartSim_V3 VERSION 3.0 LANGUAGES CXX)

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
    control/TaskSpaceController.cpp
)

add_executable(StewartSim_V3 ${SOURCES})
target_include_directories(StewartSim_V3 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    /usr/include/eigen3)

if(UNIX)
    target_link_libraries(StewartSim_V3 PRIVATE pthread m)
endif()