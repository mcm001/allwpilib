/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <wpi/math>

#include "frc/kinematics/ChassisSpeeds.h"
#include "frc/kinematics/DifferentialDriveKinematics.h"
#include "gtest/gtest.h"

using namespace frc;

static constexpr double kEpsilon = 1E-9;

TEST(DifferentialDriveKinematics, InverseKinematicsFromZero) {
  const DifferentialDriveKinematics kinematics{0.381};
  const ChassisSpeeds chassisSpeeds;
  const auto wheelSpeeds = kinematics.ToWheelSpeeds(chassisSpeeds);

  EXPECT_NEAR(wheelSpeeds.left, 0, kEpsilon);
  EXPECT_NEAR(wheelSpeeds.right, 0, kEpsilon);
}

TEST(DifferentialDriveKinematics, ForwardKinematicsFromZero) {
  const DifferentialDriveKinematics kinematics{0.381};
  const DifferentialDriveWheelSpeeds wheelSpeeds;
  const auto chassisSpeeds = kinematics.ToChassisSpeeds(wheelSpeeds);

  EXPECT_NEAR(chassisSpeeds.dx, 0, kEpsilon);
  EXPECT_NEAR(chassisSpeeds.dy, 0, kEpsilon);
  EXPECT_NEAR(chassisSpeeds.dtheta, 0, kEpsilon);
}

TEST(DifferentialDriveKinematics, InverseKinematicsForStraightLine) {
  const DifferentialDriveKinematics kinematics{0.381};
  const ChassisSpeeds chassisSpeeds{3.0, 0, 0};
  const auto wheelSpeeds = kinematics.ToWheelSpeeds(chassisSpeeds);

  EXPECT_NEAR(wheelSpeeds.left, 3, kEpsilon);
  EXPECT_NEAR(wheelSpeeds.right, 3, kEpsilon);
}

TEST(DifferentialDriveKinematics, ForwardKinematicsForStraightLine) {
  const DifferentialDriveKinematics kinematics{0.381};
  const DifferentialDriveWheelSpeeds wheelSpeeds{3.0, 3.0};
  const auto chassisSpeeds = kinematics.ToChassisSpeeds(wheelSpeeds);

  EXPECT_NEAR(chassisSpeeds.dx, 3, kEpsilon);
  EXPECT_NEAR(chassisSpeeds.dy, 0, kEpsilon);
  EXPECT_NEAR(chassisSpeeds.dtheta, 0, kEpsilon);
}

TEST(DifferentialDriveKinematics, InverseKinematicsForRotateInPlace) {
  const DifferentialDriveKinematics kinematics{0.381};
  const ChassisSpeeds chassisSpeeds{0.0, 0.0, wpi::math::pi};
  const auto wheelSpeeds = kinematics.ToWheelSpeeds(chassisSpeeds);

  EXPECT_NEAR(wheelSpeeds.left, -0.381 * wpi::math::pi, kEpsilon);
  EXPECT_NEAR(wheelSpeeds.right, +0.381 * wpi::math::pi, kEpsilon);
}

TEST(DifferentialDriveKinematics, ForwardKinematicsForRotateInPlace) {
  const DifferentialDriveKinematics kinematics{0.381};
  const DifferentialDriveWheelSpeeds wheelSpeeds{+0.381 * wpi::math::pi,
                                                 -0.381 * wpi::math::pi};
  const auto chassisSpeeds = kinematics.ToChassisSpeeds(wheelSpeeds);

  EXPECT_NEAR(chassisSpeeds.dx, 0, kEpsilon);
  EXPECT_NEAR(chassisSpeeds.dy, 0, kEpsilon);
  EXPECT_NEAR(chassisSpeeds.dtheta, -wpi::math::pi, kEpsilon);
}
