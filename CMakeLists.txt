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
<<<<<<< HEAD
    threading/Logger.cpp
)

=======
    threading/Logger.cppcmake_minimum_required(VERSION 3.16)
project(StewartSim_V3 VERSION 3.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
    add_compile_options(/W4)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Eigen
find_package(Eigen3 REQUIRED NO_MODULE)

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

add_executable(StewartSim_V3 ${SOURCES})
target_include_directories(StewartSim_V3 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    C:/libs/eigen)

if(UNIX)
    target_link_libraries(StewartSim_V3 PRIVATE pthread m)
endif()
)

>>>>>>> af8a86cf56cab7fd0dee41797364c627ff15043a
add_executable(StewartSim_V2 ${SOURCES})
target_include_directories(StewartSim_V2 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

if(MSVC)
    target_compile_options(StewartSim_V2 PRIVATE /W4)
endif()

# Link threading library on Linux
if(UNIX)
    target_link_libraries(StewartSim_V2 PRIVATE pthread m)
endif()