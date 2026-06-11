#include "../geometry/Kinematics.h"

void Kinematics::compute_InverseKinematics(
    const PlatformGeometry& geom, // input
    const Pose6DoF& pose,         // input
    Vec6& leg_lengths,//output
    Mat3x6& actuators_unit_vector,//output
    Mat3x6& jointCoordinates_Platform_World) //output

{

    // Compute once — reused for all 6 legs
    Mat3 rotation_matrix;
    rotation_matrix = compute_rotation_matrix(pose.roll, pose.pitch, pose.yaw);

    Vec3 p(pose.x, pose.y, pose.z);
    Mat3x6 leg_vectors;
    jointCoordinates_Platform_World = rotation_matrix * geom.jointCoordinates_Platform;
    leg_vectors = jointCoordinates_Platform_World.colwise() + p - geom.jointCoordinates_Base;

    leg_lengths = leg_vectors.colwise().norm();
    
    actuators_unit_vector = leg_vectors.colwise().normalized();

}

///

void Kinematics::compute_Jacobian(const Pose6DoF& pose,
    Mat3x6& actuators_unit_vector,
    Mat3x6& jointCoordinates_Platform_World,
    Mat6& Jacobian)
{

    for (size_t i = 0; i < 6; i++)
    {
        Vec3 temp = actuators_unit_vector.col(i).cross(jointCoordinates_Platform_World.col(i));
            
        Jacobian.row(i).head<3>() = actuators_unit_vector.col(i).transpose();
        Jacobian.row(i).tail<3>() = temp.transpose();

    }
}