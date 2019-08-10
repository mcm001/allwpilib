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
  DifferentialDriveKinematics kinematics{0.381_m};
  DifferentialDriveOdometry odometry{kinematics};

  odometry.ResetPosition(Pose2d());
  const auto& pose = odometry.Update(0.02_m, 0.02_m, Rotation2d());

  EXPECT_NEAR(pose.Translation().X().to<double>(), 0.02, kEpsilon);
  EXPECT_NEAR(pose.Translation().Y().to<double>(), 0.0, kEpsilon);
  EXPECT_NEAR(pose.Rotation().Radians().to<double>(), 0.0, kEpsilon);
}

TEST(DifferentialDriveOdometry, QuarterCircle) {
  DifferentialDriveKinematics kinematics{0.381_m};
  DifferentialDriveOdometry odometry{kinematics};

  odometry.ResetPosition(Pose2d());
  const auto& pose = odometry.Update(0.0_m, units::meter_t(5 * wpi::math::pi),
                                     Rotation2d(90.0_deg));

  EXPECT_NEAR(pose.Translation().X().to<double>(), 5.0, kEpsilon);
  EXPECT_NEAR(pose.Translation().Y().to<double>(), 5.0, kEpsilon);
  EXPECT_NEAR(pose.Rotation().Degrees().to<double>(), 90.0, kEpsilon);
}
