/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "frc/kinematics/SwerveDriveKinematics.h"
#include "gtest/gtest.h"

using namespace frc;

static constexpr double kEpsilon = 1E-9;

TEST(SwerveDriveKinematics, StraightLineInverseKinematics) {
  ChassisSpeeds speeds{5.0, 0.0, 0.0};

  SwerveDriveKinematics kinematics{
      Translation2d{12, 12}, Translation2d{12, -12}, Translation2d{-12, 12},
      Translation2d{-12, -12}};

  auto [fl, fr, bl, br] = kinematics.ToSwerveModuleStates(speeds);

  EXPECT_NEAR(fl.speed, 5.0, kEpsilon);
  EXPECT_NEAR(fr.speed, 5.0, kEpsilon);
  EXPECT_NEAR(bl.speed, 5.0, kEpsilon);
  EXPECT_NEAR(br.speed, 5.0, kEpsilon);

  EXPECT_NEAR(fl.angle.Radians(), 0.0, kEpsilon);
  EXPECT_NEAR(fr.angle.Radians(), 0.0, kEpsilon);
  EXPECT_NEAR(bl.angle.Radians(), 0.0, kEpsilon);
  EXPECT_NEAR(br.angle.Radians(), 0.0, kEpsilon);
}

TEST(SwerveDriveKinematics, StraightLineForwardKinematics) {
  SwerveDriveKinematics kinematics{
      Translation2d{12, 12}, Translation2d{12, -12}, Translation2d{-12, 12},
      Translation2d{-12, -12}};

  SwerveModuleState state{5.0, Rotation2d()};

  auto chassisSpeeds = kinematics.ToChassisSpeeds(state, state, state, state);

  EXPECT_NEAR(chassisSpeeds.dx, 5.0, kEpsilon);
  EXPECT_NEAR(chassisSpeeds.dy, 0.0, kEpsilon);
  EXPECT_NEAR(chassisSpeeds.dtheta, 0.0, kEpsilon);
}

TEST(SwerveDriveKinematics, NormalizeTest) {
  SwerveModuleState state1{5.0, Rotation2d()};
  SwerveModuleState state2{6.0, Rotation2d()};
  SwerveModuleState state3{4.0, Rotation2d()};
  SwerveModuleState state4{7.0, Rotation2d()};

  std::array<SwerveModuleState, 4> arr{state1, state2, state3, state4};
  SwerveDriveKinematics<4>::NormalizeWheelSpeeds(&arr, 5.5);

  double kFactor = 5.5 / 7.0;

  EXPECT_NEAR(arr[0].speed, 5.0 * kFactor, kEpsilon);
  EXPECT_NEAR(arr[1].speed, 6.0 * kFactor, kEpsilon);
  EXPECT_NEAR(arr[2].speed, 4.0 * kFactor, kEpsilon);
  EXPECT_NEAR(arr[3].speed, 7.0 * kFactor, kEpsilon);
}
