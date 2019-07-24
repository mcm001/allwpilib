/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "frc/kinematics/SwerveDriveKinematics.h"
#include "frc/kinematics/SwerveDriveOdometry.h"
#include "gtest/gtest.h"

using namespace frc;

TEST(SwerveDriveOdometry, StraightLine) {
  SwerveDriveKinematics kinematics{
      Translation2d{12, 12}, Translation2d{12, -12}, Translation2d{-12, 12},
      Translation2d{-12, -12}};

  SwerveModuleState zero{0.0, Rotation2d()};
  SwerveModuleState state{5.0, Rotation2d()};
  SwerveDriveOdometry odometry{kinematics};

  Pose2d pose;

  pose = odometry.UpdateWithTime(0_s, zero, zero, zero, zero);
  pose = odometry.UpdateWithTime(0.02_s, state, state, state, state);

  EXPECT_NEAR(pose.Translation().X(), 5 * 0.02, 1E-9);
}
