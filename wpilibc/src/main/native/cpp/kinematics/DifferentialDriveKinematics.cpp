/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "frc/kinematics/DifferentialDriveKinematics.h"

using namespace frc;

DifferentialDriveKinematics::DifferentialDriveKinematics(double driveRadius)
    : m_driveRadius(driveRadius) {}

ChassisSpeeds DifferentialDriveKinematics::ToChassisSpeeds(
    const DifferentialDriveWheelSpeeds& wheelSpeeds) const {
  return {(wheelSpeeds.left + wheelSpeeds.right) / 2, 0,
          (wheelSpeeds.right - wheelSpeeds.left) / (m_driveRadius * 2)};
}

DifferentialDriveWheelSpeeds DifferentialDriveKinematics::ToWheelSpeeds(
    const ChassisSpeeds& chassisSpeeds) const {
  return {chassisSpeeds.dx - m_driveRadius * chassisSpeeds.dtheta,
          chassisSpeeds.dx + m_driveRadius * chassisSpeeds.dtheta};
}
