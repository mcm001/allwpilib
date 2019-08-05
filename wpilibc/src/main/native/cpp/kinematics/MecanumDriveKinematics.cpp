/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "frc/kinematics/MecanumDriveKinematics.h"

using namespace frc;

MecanumDriveWheelSpeeds MecanumDriveKinematics::ToWheelSpeeds(
    const ChassisSpeeds& chassisSpeeds, const Translation2d& centerOfRotation) {
  // We have a new center of rotation. We need to compute the matrix again.
  if (centerOfRotation.X() != m_previousCoR.X() ||
      centerOfRotation.Y() != m_previousCoR.Y()) {
    auto fl = m_frontLeftWheel - centerOfRotation;
    auto fr = m_frontRightWheel - centerOfRotation;
    auto rl = m_rearLeftWheel - centerOfRotation;
    auto rr = m_rearRightWheel - centerOfRotation;

    SetInverseKinematics(fl, fr, rl, rr);

    m_previousCoR = centerOfRotation;
  }

  Eigen::Vector3d chassisSpeedsVector;
  chassisSpeedsVector << chassisSpeeds.vx, chassisSpeeds.vy,
      chassisSpeeds.omega;

  Eigen::Matrix<double, 4, 1> wheelsMatrix =
      m_inverseKinematics * chassisSpeedsVector;

  MecanumDriveWheelSpeeds wheelSpeeds;
  wheelSpeeds.frontLeft = wheelsMatrix(0, 0);
  wheelSpeeds.frontRight = wheelsMatrix(1, 0);
  wheelSpeeds.rearLeft = wheelsMatrix(2, 0);
  wheelSpeeds.rearRight = wheelsMatrix(3, 0);
  return wheelSpeeds;
}

ChassisSpeeds MecanumDriveKinematics::ToChassisSpeeds(
    const MecanumDriveWheelSpeeds& wheelSpeeds) {
  Eigen::Matrix<double, 4, 1> wheelSpeedsMatrix;
  // clang-format off
  wheelSpeedsMatrix << wheelSpeeds.frontLeft, wheelSpeeds.frontRight,
                       wheelSpeeds.rearLeft, wheelSpeeds.rearRight;
  // clang-format on

  Eigen::Vector3d chassisSpeedsVector =
      m_forwardKinematics.solve(wheelSpeedsMatrix);

  return {chassisSpeedsVector(0), chassisSpeedsVector(1),
          chassisSpeedsVector(2)};
}

void MecanumDriveKinematics::SetInverseKinematics(Translation2d fl,
                                                  Translation2d fr,
                                                  Translation2d rl,
                                                  Translation2d rr) {
  // clang-format off
  m_inverseKinematics << 1, -1, -(fl.X() + fl.Y()),
                         1,  1, fr.X() - fr.Y(),
                         1,  1, rl.X() - rl.Y(),
                         1, -1, -(rr.X() + rr.Y());
  // clang-format on
  m_inverseKinematics /= std::sqrt(2);
}
