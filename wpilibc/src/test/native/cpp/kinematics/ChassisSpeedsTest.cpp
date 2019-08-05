/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "frc/kinematics/ChassisSpeeds.h"
#include "gtest/gtest.h"

static constexpr double kEpsilon = 1E-9;

TEST(ChassisSpeeds, FieldRelativeConstruction) {
  const auto chassisSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
      1.0, 0.0, 0.5, frc::Rotation2d::FromDegrees(-90.0));

  EXPECT_NEAR(0.0, chassisSpeeds.vx, kEpsilon);
  EXPECT_NEAR(1.0, chassisSpeeds.vy, kEpsilon);
  EXPECT_NEAR(0.5, chassisSpeeds.omega, kEpsilon);
}
