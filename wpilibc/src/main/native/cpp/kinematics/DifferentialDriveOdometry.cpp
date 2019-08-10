/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "frc/kinematics/DifferentialDriveOdometry.h"

using namespace frc;

DifferentialDriveOdometry::DifferentialDriveOdometry(
    DifferentialDriveKinematics kinematics, const Pose2d& initialPose)
    : m_kinematics(kinematics), m_pose(initialPose) {
  previousAngle = m_pose.Rotation();
}

const Pose2d& DifferentialDriveOdometry::Update(units::meter_t leftEncoder,
                                                units::meter_t rightEncoder) {
  std::scoped_lock lock(m_mutex);

  const auto deltaLeft = leftEncoder - prevLeftEncoder;
  const auto deltaRight = rightEncoder - prevRightEncoder;
  const auto [dx, dy, dtheta] =
      m_kinematics.ToChassisSpeeds({deltaLeft / 1_s, deltaRight / 1_s});

  m_pose = m_pose.Exp({dx * 1_s, dy * 1_s, dtheta * 1_s});

  prevLeftEncoder = leftEncoder;
  prevRightEncoder = rightEncoder;
  previousAngle += Rotation2d(dtheta * 1_s);

  return m_pose;
}

const Pose2d& DifferentialDriveOdometry::Update(units::meter_t leftEncoder,
                                                units::meter_t rightEncoder,
                                                const Rotation2d& gyro) {
  std::scoped_lock lock(m_mutex);
  const auto deltaLeft = leftEncoder - prevLeftEncoder;
  const auto deltaRight = rightEncoder - prevRightEncoder;

  auto dx = (deltaLeft + deltaRight) / 2;
  auto dtheta = gyro - previousAngle;

  m_pose = m_pose.Exp({dx, 0_m, dtheta.Radians()});

  prevLeftEncoder = leftEncoder;
  prevRightEncoder = rightEncoder;
  previousAngle = gyro;

  return m_pose;
}
