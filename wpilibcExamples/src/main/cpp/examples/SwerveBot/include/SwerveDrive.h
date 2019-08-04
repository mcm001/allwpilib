/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>

#include "SwerveModule.h"

class SwerveDrive {
 public:
  void Drive(double xSpeed, double ySpeed, double rot);
  void UpdateOdometry();

  static constexpr double kMaxSpeed = 3.0;  // 3 meters per second
  static constexpr double kMaxAngularSpeed =
      wpi::math::pi;  // 1/2 rotation per second

 private:
  frc::Translation2d m_frontLeftLocation{+0.381, +0.831};
  frc::Translation2d m_frontRightLocation{+0.381, -0.831};
  frc::Translation2d m_backLeftLocation{-0.381, +0.831};
  frc::Translation2d m_backRightLocation{-0.381, -0.831};

  SwerveModule m_frontLeft{1, 2};
  SwerveModule m_frontRight{2, 3};
  SwerveModule m_backLeft{5, 6};
  SwerveModule m_backRight{7, 8};

  frc::SwerveDriveKinematics<4> m_kinematics{
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation,
      m_backRightLocation};

  frc::SwerveDriveOdometry<4> m_odometry{m_kinematics};
};
