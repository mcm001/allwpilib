/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "MecanumDrive.h"

frc::MecanumDriveWheelSpeeds MecanumDrive::GetCurrentState() const {
  return {units::meters_per_second_t(m_frontLeftEncoder.GetRate()),
          units::meters_per_second_t(m_frontRightEncoder.GetRate()),
          units::meters_per_second_t(m_backLeftEncoder.GetRate()),
          units::meters_per_second_t(m_backRightEncoder.GetRate())};
}

void MecanumDrive::SetSpeeds(const frc::MecanumDriveWheelSpeeds& wheelSpeeds) {
  const auto frontLeftOutput = m_frontLeftPIDController.Calculate(
      m_frontLeftEncoder.GetRate(), wheelSpeeds.frontLeft.to<double>());
  const auto frontRightOutput = m_frontRightPIDController.Calculate(
      m_frontRightEncoder.GetRate(), wheelSpeeds.frontRight.to<double>());
  const auto backLeftOutput = m_backLeftPIDController.Calculate(
      m_backLeftEncoder.GetRate(), wheelSpeeds.rearLeft.to<double>());
  const auto backRightOutput = m_backRightPIDController.Calculate(
      m_backRightEncoder.GetRate(), wheelSpeeds.rearRight.to<double>());

  m_frontLeftMotor.Set(frontLeftOutput);
  m_frontRightMotor.Set(frontRightOutput);
  m_backLeftMotor.Set(backLeftOutput);
  m_backRightMotor.Set(backRightOutput);
}

void MecanumDrive::Drive(units::meters_per_second_t xSpeed,
                         units::meters_per_second_t ySpeed,
                         units::radians_per_second_t rot, bool fieldRelative) {
  auto wheelSpeeds = m_kinematics.ToWheelSpeeds(
      fieldRelative
          ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                xSpeed, ySpeed, rot,
                // Negating the angle because WPILib Gyros are CW positive.
                frc::Rotation2d(units::degree_t(-m_gyro.GetAngle())))
          : frc::ChassisSpeeds{xSpeed, ySpeed, rot});
  wheelSpeeds.Normalize(kMaxSpeed);
  SetSpeeds(wheelSpeeds);
}

void MecanumDrive::UpdateOdometry() { m_odometry.Update(GetCurrentState()); }
