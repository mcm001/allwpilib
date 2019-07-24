/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "frc/kinematics/MecanumDriveOdometry.h"

using namespace frc;

MecanumDriveOdometry::MecanumDriveOdometry(MecanumDriveKinematics kinematics,
                                           const Pose2d& initialPose)
    : m_kinematics(kinematics), m_pose(initialPose) {}

const Pose2d& MecanumDriveOdometry::UpdateWithTime(
    units::second_t currentTime, MecanumDriveWheelSpeeds wheelSpeeds) {
  std::scoped_lock lock(m_mutex);
  units::second_t deltaTime =
      (m_previousTime >= 0_s) ? currentTime - m_previousTime : 0_s;
  m_previousTime = currentTime;

  double period = units::unit_cast<double>(deltaTime);
  auto [dx, dy, dtheta] = m_kinematics.ToChassisSpeeds(wheelSpeeds);

  m_pose = m_pose.Exp({dx * period, dy * period, dtheta * period});

  return m_pose;
}

const Pose2d& MecanumDriveOdometry::UpdateWithTime(
    units::second_t currentTime, units::radians_per_second_t angularVelocity,
    MecanumDriveWheelSpeeds wheelSpeeds) {
  std::scoped_lock lock(m_mutex);
  units::second_t deltaTime =
      (m_previousTime >= 0_s) ? currentTime - m_previousTime : 0_s;
  m_previousTime = currentTime;

  double period = units::unit_cast<double>(deltaTime);
  auto [dx, dy, dtheta] = m_kinematics.ToChassisSpeeds(wheelSpeeds);
  static_cast<void>(dtheta);

  m_pose = m_pose.Exp({dx * period, dy * period,
                       units::unit_cast<double>(angularVelocity) * period});

  return m_pose;
}
