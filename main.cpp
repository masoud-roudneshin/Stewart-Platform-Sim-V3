<<<<<<< HEAD
#include <Eigen/Dense>
#include <iostream>
#include <iomanip>
#include "math/Math.h"
#include "geometry/Geometry.h"
#include "geometry/Kinematics.h"
#include "platform/StewartPlatform.h"

int main()
{
    std::cout << "=== Stewart Platform V3 — IK Test ===\n\n";

    // 1. Create Geometry
    PlatformGeometry geom(1.0, 0.5, 5.0 * PI / 180.0, 5.0 * PI / 180.0);
    real_t body_length = 0.8;
=======
// placeholder
#include<array>
#include <memory>
#include <thread>
#include <iostream>
#include "geometry/Geometry.h"
#include "threading/SharedData.h"
#include "threading/ThreadedFOCDriver.h"
#include "threading/ThreadedController.h"
#include "threading/Logger.h"
#include "actuator/MotorDriver.h"
#include "geometry/Kinematics.h"
#include "platform/StewartPlatform.h"
#include "threading/ThreadedSafetyMonitor.h"
#include "math/Math.h"

int main()
{

    // Make Platform Geometry

    PlatformGeometry geom(1.0, 0.5, 5.0 * PI / 180.0, 5.0 * PI / 180.0);
    double body_length = 0.8;
    MotorParameters motor;
    LeadScrewParameters screw;
    real_t force_to_iq_gain = screw.lead / (2.0 * PI * motor.Kt * screw.efficiency);

    // Make Actuators Shared Data between the threads (and also their pointers)

    std::array<std::unique_ptr<ActuatorSharedData>, 6> shared_data;

    for (size_t i = 0; i < 6; i++)
    {
        shared_data[i] = std::make_unique<ActuatorSharedData>();
    }

    std::array<ActuatorSharedData*, 6> shared_ptrs;

    for (size_t i = 0; i < 6; i++)
    {
        shared_ptrs[i] = shared_data[i].get();
    }

    // Making the threaded objects

    // Make FOC Threaded Driver
    ThreadedSafetyMonitor safety(shared_ptrs, 2.0, 500.0, 0.6);

    std::array<std::unique_ptr<ThreadedFOCDriver>, 6> foc_drivers;

    for (size_t i = 0; i < 6; i++)
    {
        foc_drivers[i] = std::make_unique<ThreadedFOCDriver>(*shared_data[i], safety);
    }

    ThreadedController controller(shared_ptrs,
        safety,
        10.0,              // wn
        0.707,             // zeta
        0.001,             // dt   1kHz control loop
        force_to_iq_gain); // computed above);
    Logger logger(shared_ptrs, safety);

    // IK and target Pose

    Pose6DoF target_pos;
    target_pos.z = 0.05;
    target_pos.roll = 0.1;

>>>>>>> af8a86cf56cab7fd0dee41797364c627ff15043a
    // Create temporary platform just to get geometry info
    StewartPlatform temp_platform(geom, body_length, 0.6, 500.0, 10.0, 0.707, 0.0001);
    real_t mid_heave = temp_platform.get_mid_heave();

<<<<<<< HEAD
    // 2. Create Test Pose
    Pose6DoF true_pose;
    true_pose.z = 0.2 + mid_heave;
    true_pose.roll = 0.3;

    Vec6 leg_lengths;
    Mat3x6 actuators_unit_vector = Mat3x6::Zero();
    Mat3x6 jointCoordinates_Platform_World = Mat3x6::Zero();

    Kinematics::compute_InverseKinematics(
        geom,
        true_pose,
        leg_lengths,
        actuators_unit_vector,
        jointCoordinates_Platform_World);

    // 3. Run FK starting from neutral pose (deliberate wrong initial guess)
    Pose6DoF estimated_pose;  // default zero — wrong initial guess
    estimated_pose.z = mid_heave;  // only z hint


    

    bool converged = Kinematics::compute_forward_kinematics(
        geom, leg_lengths, estimated_pose);

    // 4. Compare true pose vs estimated pose
    std::cout << "Converged: " << converged << "\n";
    std::cout << "True    z=" << true_pose.z << " roll=" << true_pose.roll << "\n";
    std::cout << "Est     z=" << estimated_pose.z << " roll=" << estimated_pose.roll << "\n";
    

    /*

    Kinematics::compute_InverseKinematics(geom, pose,
        leg_lengths, unit_vectors, platform_joints_world);

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "Leg lengths:\n";
    for (int i = 0; i < 6; i++)
        std::cout << "  L" << i << " = " << leg_lengths(i) << " m\n";
    */
=======
    // Inverse Kinematics 

    target_pos.z += mid_heave;   // offset by mid_heave

    Vec6 leg_lengths;
    Vec6 target_strokes = Vec6::Zero();
    Mat3x6 unit_vectors;
    Mat3x6 platform_joints_world;

    Kinematics::compute_InverseKinematics(geom, target_pos,
        leg_lengths, unit_vectors, platform_joints_world);

    target_strokes = leg_lengths.array() - body_length;
    

    controller.set_target(target_strokes);


    // Starting the threads
    safety.start();
    for (int i = 0; i < 6; i++) { foc_drivers[i]->start(); }

    controller.start();

    logger.start();

    // Wait for the user's input

    /*
    std::cout << "Press Enter to stop...\n";
    std::cin.get();
    safety.estop();
    */

    // Wait 2 seconds then trigger ESTOP test
    std::this_thread::sleep_for(std::chrono::seconds(2));
    std::cout << "\nTriggering ESTOP...\n" << std::flush;
    safety.estop();
    std::this_thread::sleep_for(std::chrono::milliseconds(500)); // longer wait
    std::cout << "\n\n=== ESTOP TRIGGERED ===\n" << std::flush;
    std::cout << "State: " << static_cast<int>(safety.get_state()) << "\n" << std::flush;
    std::cout << "Press Enter to exit...\n" << std::flush;

    std::this_thread::sleep_for(std::chrono::seconds(3));

    std::cout << "Stopping logger...\n" << std::flush;
    logger.stop();
    std::cout << "Stopping controller...\n" << std::flush;
    controller.stop();
    std::cout << "Stopping FOC...\n" << std::flush;
    for (int i = 0; i < 6; i++) foc_drivers[i]->stop();
    std::cout << "Stopping safety...\n" << std::flush;
    safety.stop();
    std::cout << "All stopped.\n" << std::flush;

>>>>>>> af8a86cf56cab7fd0dee41797364c627ff15043a
    return 0;
}