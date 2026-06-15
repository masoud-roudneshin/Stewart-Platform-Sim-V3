<<<<<<< HEAD
# Stewart Platform Simulation V2 — Multithreaded C++ Robotics Control

Extends V1 with a full multithreaded architecture — FOC at 10kHz per actuator,
position control at 1kHz, lock-free logging, and a dedicated safety monitor thread.

---

## What's New in V2

| Feature | V1 | V2 |
|---|---|---|
| Execution model | Sequential | Multithreaded |
| FOC loop | Sequential, 10kHz simulated | Dedicated thread per actuator, 10kHz |
| Position loop | Sequential, 1kHz simulated | Dedicated thread, 1kHz |
| Safety monitor | Wrapper class | Dedicated monitoring thread |
| Data transfer | Direct function calls | mutex + atomic + lock-free ring buffer |
| Logging | std::cout in main loop | Lock-free ring buffer to logger thread |

---

## Threading Architecture

```
main()
├── ThreadedSafetyMonitor  (1kHz) — monitors limits, state machine
├── ThreadedFOCDriver × 6  (10kHz each) — one per actuator
├── ThreadedController     (1kHz) — PD control, reads state, writes iq_ref
└── Logger                 (drains) — lock-free ring buffer consumer
```

## Data Flow

```
FOC thread    →  mutex      →  latest_state  →  Controller thread
Controller    →  atomic     →  iq_ref        →  FOC thread
FOC thread    →  RingBuffer →  log data      →  Logger thread
Safety thread →  atomic     →  estop flag    →  FOC + Controller threads
=======
# Stewart Platform Simulation V3 — Eigen-Based Linear Algebra

Extends V2 by replacing all custom math with Eigen — the industry standard
linear algebra library used by ROS2, MoveIt, Drake, and Pinocchio.

---

## What's New in V3

| Feature | V2 | V3 |
|---|---|---|
| Math library | Custom std::array functions | Eigen |
| Vector type | std::array<double,3> | Eigen::Vector3d |
| Matrix type | std::array<std::array<double,3>,3> | Eigen::Matrix3d |
| Joint coordinates | std::array<std::array<double,3>,6> | Eigen::Matrix<double,3,6> |
| IK computation | Manual loop | Vectorized column-wise operations |
| Rotation matrix | Manual trig computation | Eigen::AngleAxisd |
| Jacobian | Manual loop | Eigen block operations |

---

## Key Eigen Types Used

```cpp
using Vec3   = Eigen::Vector3d;
using Mat3   = Eigen::Matrix3d;
using Vec6   = Eigen::Matrix<double, 6, 1>;
using Mat6   = Eigen::Matrix<double, 6, 6>;
using Mat3x6 = Eigen::Matrix<double, 3, 6>;
>>>>>>> af8a86cf56cab7fd0dee41797364c627ff15043a
```

---

<<<<<<< HEAD
## Synchronization Design

| Data | Mechanism | Reason |
|---|---|---|
| FOCState (stroke, velocity) | std::mutex | Multi-field consistent snapshot |
| iq_ref | std::atomic | Single value, never block RT thread |
| Log stream | Lock-free RingBuffer | High-frequency SPSC, no blocking |
| estop flag | std::atomic<bool> | Single flag, fastest response |
=======
## Vectorized Inverse Kinematics

```cpp
Mat3x6 platform_world = R * geom.platform_joints;
platform_world.colwise() += p;
Mat3x6 leg_vectors  = platform_world - geom.base_joints;
Vec6   leg_lengths  = leg_vectors.colwise().norm();
Mat3x6 unit_vectors = leg_vectors.colwise().normalized();
```

No explicit loop — all 6 legs computed simultaneously using SIMD.
>>>>>>> af8a86cf56cab7fd0dee41797364c627ff15043a

---

## Building

### Requirements
- C++17 compiler (MSVC, GCC, Clang)
- CMake 3.16+
<<<<<<< HEAD
=======
- Eigen 3.4+ (header-only)
>>>>>>> af8a86cf56cab7fd0dee41797364c627ff15043a

### Build

```bash
mkdir build && cd build
cmake ..
cmake --build .
<<<<<<< HEAD
./StewartSim_V2
```

### Visual Studio
Open `Stewart_Platform_V2.sln` directly.
=======
./StewartSim_V3
```

### Visual Studio
Open `Stewart_Platform_V3.sln` directly.
>>>>>>> af8a86cf56cab7fd0dee41797364c627ff15043a

---

## Simulation Results

<<<<<<< HEAD
Same convergence as V1 — verified identical results:

- Pure heave (z=+5cm): all 6 legs converge identically ✅
- Heave + roll (z=+5cm, roll=0.1rad): 6 different targets, all converge ✅
- ESTOP: all iq_refs zeroed immediately, state transitions to ESTOP ✅
=======
Identical to V1 and V2 — verified:

- Pure heave (z=+5cm): all 6 legs converge identically ✅
- Heave + roll (z=+5cm, roll=0.1rad): 6 different targets, all converge ✅
- ESTOP: all iq_refs zeroed immediately ✅
>>>>>>> af8a86cf56cab7fd0dee41797364c627ff15043a
- Clean shutdown: all threads join in reverse order ✅

---

## Project Structure

```
<<<<<<< HEAD
Stewart_Platform_V2/
├── main.cpp
├── CMakeLists.txt
├── math/
│   └── Math.h
├── geometry/
│   ├── Geometry.h/.cpp
│   └── Kinematics.h/.cpp
├── actuator/
│   ├── MotorDriver.h/.cpp
│   └── ActuatorState.h/.cpp
├── control/
│   └── PDController.h
=======
Stewart_Platform_V3/
├── main.cpp
├── CMakeLists.txt
├── math/Math.h              — Eigen types and rotation matrix
├── geometry/
│   ├── Geometry.h/.cpp      — Mat3x6 joint coordinates
│   └── Kinematics.h/.cpp    — Vectorized IK and Jacobian
├── actuator/
│   ├── MotorDriver.h/.cpp
│   └── ActuatorState.h/.cpp
├── control/PDController.h
>>>>>>> af8a86cf56cab7fd0dee41797364c627ff15043a
├── platform/
│   ├── StewartPlatform.h/.cpp
│   └── SafetyMonitor.h/.cpp
├── threading/
│   ├── SharedData.h
│   ├── ThreadedFOCDriver.h/.cpp
│   ├── ThreadedController.h/.cpp
│   ├── ThreadedSafetyMonitor.h/.cpp
│   └── Logger.h/.cpp
└── utils/
    ├── LoopTimer.h
    └── RingBuffer.h
```

---

## Roadmap

<<<<<<< HEAD
- [ ] Trajectory tracking with dynamic set_targets()
- [ ] Forward kinematics (Newton-Raphson, requires Eigen)
=======
- [ ] Forward kinematics (Newton-Raphson solver)
>>>>>>> af8a86cf56cab7fd0dee41797364c627ff15043a
- [ ] Task-space impedance control via Jacobian inverse
- [ ] ROS2 node wrapper (rclcpp)
- [ ] Real hardware interface (CAN bus, ODrive)

---

## Author

<<<<<<< HEAD
Masoud Roudneshin — PhD Electrical Engineering  
=======
Masoud Roudneshin — PhD Electrical & Computer Engineering  
>>>>>>> af8a86cf56cab7fd0dee41797364c627ff15043a
Specialization: Robotics, Optimization and Control