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
```

---

## Synchronization Design

| Data | Mechanism | Reason |
|---|---|---|
| FOCState (stroke, velocity) | std::mutex | Multi-field consistent snapshot |
| iq_ref | std::atomic | Single value, never block RT thread |
| Log stream | Lock-free RingBuffer | High-frequency SPSC, no blocking |
| estop flag | std::atomic<bool> | Single flag, fastest response |

---

## Building

### Requirements
- C++17 compiler (MSVC, GCC, Clang)
- CMake 3.16+

### Build

```bash
mkdir build && cd build
cmake ..
cmake --build .
./StewartSim_V2
```

### Visual Studio
Open `Stewart_Platform_V2.sln` directly.

---

## Simulation Results

Same convergence as V1 — verified identical results:

- Pure heave (z=+5cm): all 6 legs converge identically ✅
- Heave + roll (z=+5cm, roll=0.1rad): 6 different targets, all converge ✅
- ESTOP: all iq_refs zeroed immediately, state transitions to ESTOP ✅
- Clean shutdown: all threads join in reverse order ✅

---

## Project Structure

```
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

- [ ] Trajectory tracking with dynamic set_targets()
- [ ] Forward kinematics (Newton-Raphson, requires Eigen)
- [ ] Task-space impedance control via Jacobian inverse
- [ ] ROS2 node wrapper (rclcpp)
- [ ] Real hardware interface (CAN bus, ODrive)

---

## Author

Masoud Roudneshin — PhD Electrical Engineering  
Specialization: Robotics, Optimization and Control