#pragma once
#include <array>
#include "../math/Math.h"
#include "../geometry/Geometry.h"

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

    void compute_Jacobian(  const Pose6DoF& pose,
                            Mat3x6& actuators_unit_vector,
                            Mat3x6& jointCoordinates_Platform_World,
                            Mat6& Jacobian);

    // Forward Kinematics
}
