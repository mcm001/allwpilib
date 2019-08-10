/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <units/units.h>

#include "frc/kinematics/SwerveDriveKinematics.h"
#include "gtest/gtest.h"

using namespace frc;

static constexpr double kEpsilon = 1E-9;

TEST(SwerveDriveKinematics, StraightLineInverseKinematics) {
  ChassisSpeeds speeds{5.0_mps, 0.0_mps, 0.0_rad_per_s};

  SwerveDriveKinematics kinematics{
      Translation2d{12_m, 12_m}, Translation2d{12_m, -12_m},
      Translation2d{-12_m, 12_m}, Translation2d{-12_m, -12_m}};

  auto [fl, fr, bl, br] = kinematics.ToSwerveModuleStates(speeds);

  EXPECT_NEAR(fl.speed.to<double>(), 5.0, kEpsilon);
  EXPECT_NEAR(fr.speed.to<double>(), 5.0, kEpsilon);
  EXPECT_NEAR(bl.speed.to<double>(), 5.0, kEpsilon);
  EXPECT_NEAR(br.speed.to<double>(), 5.0, kEpsilon);

  EXPECT_NEAR(fl.angle.Radians().to<double>(), 0.0, kEpsilon);
  EXPECT_NEAR(fr.angle.Radians().to<double>(), 0.0, kEpsilon);
  EXPECT_NEAR(bl.angle.Radians().to<double>(), 0.0, kEpsilon);
  EXPECT_NEAR(br.angle.Radians().to<double>(), 0.0, kEpsilon);
}

TEST(SwerveDriveKinematics, StraightLineForwardKinematics) {
  SwerveDriveKinematics kinematics{
      Translation2d{12_m, 12_m}, Translation2d{12_m, -12_m},
      Translation2d{-12_m, 12_m}, Translation2d{-12_m, -12_m}};

  SwerveModuleState state{5.0_mps, Rotation2d()};

  auto chassisSpeeds = kinematics.ToChassisSpeeds(state, state, state, state);

  EXPECT_NEAR(chassisSpeeds.vx.to<double>(), 5.0, kEpsilon);
  EXPECT_NEAR(chassisSpeeds.vy.to<double>(), 0.0, kEpsilon);
  EXPECT_NEAR(chassisSpeeds.omega.to<double>(), 0.0, kEpsilon);
}

TEST(SwerveDriveKinematics, NormalizeTest) {
  SwerveModuleState state1{5.0_mps, Rotation2d()};
  SwerveModuleState state2{6.0_mps, Rotation2d()};
  SwerveModuleState state3{4.0_mps, Rotation2d()};
  SwerveModuleState state4{7.0_mps, Rotation2d()};

  std::array<SwerveModuleState, 4> arr{state1, state2, state3, state4};
  SwerveDriveKinematics<4>::NormalizeWheelSpeeds(&arr, 5.5_mps);

  double kFactor = 5.5 / 7.0;

  EXPECT_NEAR(arr[0].speed.to<double>(), 5.0 * kFactor, kEpsilon);
  EXPECT_NEAR(arr[1].speed.to<double>(), 6.0 * kFactor, kEpsilon);
  EXPECT_NEAR(arr[2].speed.to<double>(), 4.0 * kFactor, kEpsilon);
  EXPECT_NEAR(arr[3].speed.to<double>(), 7.0 * kFactor, kEpsilon);
}
