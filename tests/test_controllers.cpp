#include <gtest/gtest.h>
#include "math/Math.h"
#include "geometry/Geometry.h"
#include "geometry/Kinematics.h"
#include "control/PDController.h"
#include "platform/StewartPlatform.h"
#include "control/TaskSpaceController.h"

// ─── PDController Tests ──────────────────────────────────────
class PDControllerTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        // Same parameters as V1/V2/V3
        real_t M = 2.0;    // moving mass kg
        real_t b = 5.0;   // viscous friction
        real_t wn = 10.0;   // natural frequency
        real_t zeta = 0.707;  // damping ratio
        real_t dt = 0.001;  // timestep

        pd = PDController(M, b, wn, zeta, dt);
    }

    PDController pd;
};

TEST_F(PDControllerTest, ZeroErrorGivesZeroForce)
{
    // At target — no force needed
    real_t F = pd.compute(0.3, 0.3, 0.0);
    EXPECT_NEAR(F, 0.0, 1e-10);
}

TEST_F(PDControllerTest, PositiveErrorGivesPositiveForce)
{
    // Target ahead — push forward
    real_t F = pd.compute(0.3, 0.0, 0.0);
    EXPECT_GT(F, 0.0);
}

TEST_F(PDControllerTest, NegativeErrorGivesNegativeForce)
{
    // Overshot target — push back
    real_t F = pd.compute(0.0, 0.3, 0.0);
    EXPECT_LT(F, 0.0);
}

TEST_F(PDControllerTest, PositiveVelocityReducesForce)
{
    // Moving toward target — damping reduces force
    real_t F_no_vel = pd.compute(0.3, 0.0, 0.0);
    real_t F_with_vel = pd.compute(0.3, 0.0, 1.0);
    EXPECT_LT(F_with_vel, F_no_vel);
}

TEST_F(PDControllerTest, GravityFeedforwardAddsToForce)
{
    real_t F_no_gravity = pd.compute(0.3, 0.3, 0.0, 0.0);
    real_t F_with_gravity = pd.compute(0.3, 0.3, 0.0, 10.0);
    EXPECT_NEAR(F_with_gravity, 10.0, 1e-10);
    EXPECT_GT(F_with_gravity, F_no_gravity);
}

// ─── TaskSpaceController Tests ───────────────────────────────
class TaskSpaceControllerTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        geom = PlatformGeometry(1.0, 0.5,
            5.0 * PI / 180.0,
            5.0 * PI / 180.0);

        real_t body_length = 0.8;
        StewartPlatform platform(geom, body_length, 0.6, 500.0, 10.0, 0.707, 0.0001);
        mid_heave = platform.get_mid_heave();

        Vec6 kp, kd;
        kp << 3000, 3000, 3000, 300, 300, 300;
        kd << 150, 150, 150, 15, 15, 15;
        controller = std::make_unique<TaskSpaceController>(kp, kd);
    }

    std::unique_ptr<TaskSpaceController> controller; 
    PlatformGeometry geom;
    real_t           mid_heave;

    Mat6 get_jacobian(const Pose6DoF& pose)
    {
        Vec6   leg_lengths;
        Mat3x6 unit_vectors, joints_world;
        Kinematics::compute_InverseKinematics(geom, pose,
            leg_lengths, unit_vectors, joints_world);
        Mat6 J;
        Kinematics::compute_Jacobian(pose, unit_vectors, joints_world, J);
        return J;
    }
};

TEST_F(TaskSpaceControllerTest, ZeroErrorGivesZeroForces)
{
    // At target with zero velocity — no force needed
    Pose6DoF pose;
    pose.z = mid_heave;

    Mat6 J = get_jacobian(pose);
    Vec6 velocity = Vec6::Zero();

    Vec6 forces = controller->compute(pose, pose, velocity, J);

    for (int i = 0; i < 6; i++)
        EXPECT_NEAR(forces(i), 0.0, 1e-6)
        << "Force " << i << " should be zero at target";
}

TEST_F(TaskSpaceControllerTest, PositiveZErrorGivesPositiveForces)
{
    // Platform below target — forces should push up
    Pose6DoF actual, desired;
    actual.z = mid_heave;
    desired.z = mid_heave + 0.05;  // target is above

    Mat6 J = get_jacobian(actual);
    Vec6 velocity = Vec6::Zero();

    Vec6 forces = controller->compute(actual, desired, velocity, J);

    // All forces should be positive for pure heave up
    for (int i = 0; i < 6; i++)
        EXPECT_GT(forces(i), 0.0)
        << "Force " << i << " should be positive for upward heave";
}

TEST_F(TaskSpaceControllerTest, ForcesSymmetricForPureHeave)
{
    // Pure heave — symmetric geometry → all forces equal
    Pose6DoF actual, desired;
    actual.z = mid_heave;
    desired.z = mid_heave + 0.05;

    Mat6 J = get_jacobian(actual);
    Vec6 velocity = Vec6::Zero();

    Vec6 forces = controller->compute(actual, desired, velocity, J);

    for (int i = 1; i < 6; i++)
        EXPECT_NEAR(forces(i), forces(0), 1.0)
        << "Force " << i << " differs too much from force 0";
}