/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <iostream>
#include <memory>
#include <vector>

#include <units/units.h>
#include <wpi/MathExtras.h>

#include "frc/geometry/Pose2d.h"
#include "frc/system/LinearSystem.h"
#include "frc/system/plant/LinearSystemId.h"
#include "frc/trajectory/TrajectoryGenerator.h"
#include "frc/trajectory/constraint/DifferentialDriveVelocitySystemConstraint.h"
#include "gtest/gtest.h"
#include "trajectory/TestTrajectory.h"

using namespace frc;

TEST(DifferentialDriveVelocitySystemTest, Constraint) {
  const auto maxVoltage = 10_V;

  // Pick an unreasonably large kA to ensure the constraint has to do some work
  const LinearSystem<2, 2, 2> system =
      IdentifyDrivetrainSystem(1.0, 3.0, 1.0, 3.0, maxVoltage);
  const DifferentialDriveKinematics kinematics{0.5_m};
  auto config = TrajectoryConfig(12_mps, 12_mps_sq);
  config.AddConstraint(DifferentialDriveVelocitySystemConstraint(
      system, kinematics, maxVoltage));

  auto trajectory = TestTrajectory::GetTrajectory(config);

  units::second_t time = 0_s;
  units::second_t dt = 20_ms;
  units::second_t duration = trajectory.TotalTime();

  while (time < duration) {
    const Trajectory::State point = trajectory.Sample(time);
    time += dt;

    const ChassisSpeeds chassisSpeeds{point.velocity, 0_mps,
                                      point.velocity * point.curvature};

    auto [left, right] = kinematics.ToWheelSpeeds(chassisSpeeds);

    auto x = frc::MakeMatrix<2, 1>(left.to<double>(), right.to<double>());

    // Not really a strictly-correct test as we're using the chassis accel
    // instead of the wheel accel, but much easier than doing it "properly" and
    // a reasonable check anyway
    auto xDot = frc::MakeMatrix<2, 1>(point.acceleration.to<double>(),
                                      point.acceleration.to<double>());

    auto u = system.B().inverse() * (xDot - (system.A() * x));

    EXPECT_TRUE(((-maxVoltage.to<double>() - 0.5) <= u(0)) &&
                (u(0) <= (maxVoltage.to<double>() + 0.5)));
    EXPECT_TRUE(((-maxVoltage.to<double>() - 0.5) <= u(1)) &&
                (u(1) <= (maxVoltage.to<double>() + 0.5)));
  }
}
