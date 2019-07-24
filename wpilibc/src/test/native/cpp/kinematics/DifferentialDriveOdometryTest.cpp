/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <wpi/math>

#include "frc/kinematics/DifferentialDriveKinematics.h"
#include "frc/kinematics/DifferentialDriveOdometry.h"
#include "gtest/gtest.h"

static constexpr double kEpsilon = 1E-9;

using namespace frc;

TEST(DifferentialDriveOdometry, OneIteration) {
  DifferentialDriveKinematics kinematics{0.381};
  DifferentialDriveOdometry odometry{kinematics};

  odometry.ResetPosition(Pose2d());
  const auto& pose = odometry.Update(0.02, 0.02, Rotation2d());

  EXPECT_NEAR(pose.Translation().X(), 0.02, kEpsilon);
  EXPECT_NEAR(pose.Translation().Y(), 0.0, kEpsilon);
  EXPECT_NEAR(pose.Rotation().Radians(), 0.0, kEpsilon);
}

TEST(DifferentialDriveOdometry, QuarterCircle) {
  DifferentialDriveKinematics kinematics{0.381};
  DifferentialDriveOdometry odometry{kinematics};

  odometry.ResetPosition(Pose2d());
  const auto& pose =
      odometry.Update(0.0, 5 * wpi::math::pi, Rotation2d::FromDegrees(90.0));

  EXPECT_NEAR(pose.Translation().X(), 5.0, kEpsilon);
  EXPECT_NEAR(pose.Translation().Y(), 5.0, kEpsilon);
  EXPECT_NEAR(pose.Rotation().Degrees(), 90.0, kEpsilon);
}
