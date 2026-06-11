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
```

---

## Vectorized Inverse Kinematics

```cpp
Mat3x6 platform_world = R * geom.platform_joints;
platform_world.colwise() += p;
Mat3x6 leg_vectors  = platform_world - geom.base_joints;
Vec6   leg_lengths  = leg_vectors.colwise().norm();
Mat3x6 unit_vectors = leg_vectors.colwise().normalized();
```

No explicit loop — all 6 legs computed simultaneously using SIMD.

---

## Building

### Requirements
- C++17 compiler (MSVC, GCC, Clang)
- CMake 3.16+
- Eigen 3.4+ (header-only)

### Build

```bash
mkdir build && cd build
cmake ..
cmake --build .
./StewartSim_V3
```

### Visual Studio
Open `Stewart_Platform_V3.sln` directly.

---

## Simulation Results

Identical to V1 and V2 — verified:

- Pure heave (z=+5cm): all 6 legs converge identically ✅
- Heave + roll (z=+5cm, roll=0.1rad): 6 different targets, all converge ✅
- ESTOP: all iq_refs zeroed immediately ✅
- Clean shutdown: all threads join in reverse order ✅

---

## Project Structure

```
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

- [ ] Forward kinematics (Newton-Raphson solver)
- [ ] Task-space impedance control via Jacobian inverse
- [ ] ROS2 node wrapper (rclcpp)
- [ ] Real hardware interface (CAN bus, ODrive)

---

## Author

Masoud Roudneshin — PhD Electrical & Computer Engineering  
Specialization: Robotics, Optimization and Control