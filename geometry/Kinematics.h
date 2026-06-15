#pragma once
#include <array>
#include "../math/Math.h"
#include "../geometry/Geometry.h"

<<<<<<< HEAD
using Pose6 = Eigen::Matrix<double, 6, 1>;

=======
>>>>>>> af8a86cf56cab7fd0dee41797364c627ff15043a
namespace Kinematics
{

    void compute_InverseKinematics(
        const PlatformGeometry& geom,
        const Pose6DoF& pose,
        Vec6& leg_lengths,
        Mat3x6& actuators_unit_vector,
        Mat3x6& jointCoordinates_Platform_World);

    /// Jacobian
    // This gives the 6x6 Jacobian for Stewart Platform
<<<<<<< HEAD
    Pose6 pose_to_vec(const Pose6DoF& p);
    Pose6DoF vec_to_pose(const Pose6& x);
=======
>>>>>>> af8a86cf56cab7fd0dee41797364c627ff15043a

    void compute_Jacobian(  const Pose6DoF& pose,
                            Mat3x6& actuators_unit_vector,
                            Mat3x6& jointCoordinates_Platform_World,
                            Mat6& Jacobian);

<<<<<<< HEAD
    bool compute_forward_kinematics(const PlatformGeometry& geom,
                                    const Vec6& leg_lengths_measured,
                                    Pose6DoF& pose_estimate,
                                    int max_iterations = 20,
                                    real_t tolerance = 1e-6);

=======
>>>>>>> af8a86cf56cab7fd0dee41797364c627ff15043a
    // Forward Kinematics
}
