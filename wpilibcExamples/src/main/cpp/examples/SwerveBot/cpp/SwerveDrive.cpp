/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "SwerveDrive.h"

void SwerveDrive::Drive(units::meters_per_second_t xSpeed,
                        units::meters_per_second_t ySpeed,
                        units::radians_per_second_t rot, bool fieldRelative) {
  auto states = m_kinematics.ToSwerveModuleStates(
      fieldRelative
          ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                xSpeed, ySpeed, rot,
                // Negating the angle because WPILib Gyros are CW positive.
                frc::Rotation2d(units::degree_t(-m_gyro.GetAngle())))
          : frc::ChassisSpeeds{xSpeed, ySpeed, rot});

  m_kinematics.NormalizeWheelSpeeds(&states, kMaxSpeed);

  auto [fl, fr, bl, br] = states;

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_backLeft.SetDesiredState(bl);
  m_backRight.SetDesiredState(br);
}

void SwerveDrive::UpdateOdometry() {
  m_odometry.Update(m_frontLeft.GetState(), m_frontRight.GetState(),
                    m_backLeft.GetState(), m_backRight.GetState());
}
