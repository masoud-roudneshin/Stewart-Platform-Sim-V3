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
    // Create temporary platform just to get geometry info
    StewartPlatform temp_platform(geom, body_length, 0.6, 500.0, 10.0, 0.707, 0.0001);
    real_t mid_heave = temp_platform.get_mid_heave();

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
    return 0;
}