/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

namespace frc {
template <size_t NumModules>
SwerveDriveOdometry<NumModules>::SwerveDriveOdometry(
    SwerveDriveKinematics<NumModules> kinematics, const Pose2d& initialPose)
    : m_kinematics(kinematics), m_pose(initialPose) {}

template <size_t NumModules>
template <typename... ModuleStates>
const Pose2d& frc::SwerveDriveOdometry<NumModules>::UpdateWithTime(
    units::second_t currentTime, ModuleStates&&... moduleStates) {
  std::scoped_lock lock(m_mutex);
  units::second_t deltaTime =
      (m_previousTime >= 0_s) ? currentTime - m_previousTime : 0_s;
  m_previousTime = currentTime;

  double period = units::unit_cast<double>(deltaTime);
  auto [dx, dy, dtheta] = m_kinematics.ToChassisSpeeds(moduleStates...);

  m_pose = m_pose.Exp({dx * period, dy * period, dtheta * period});

  return m_pose;
}

template <size_t NumModules>
template <typename... ModuleStates>
const Pose2d& frc::SwerveDriveOdometry<NumModules>::UpdateWithTime(
    units::second_t currentTime, units::radians_per_second_t angularVelocity,
    ModuleStates&&... moduleStates) {
  std::scoped_lock lock(m_mutex);
  units::second_t deltaTime =
      (m_previousTime >= 0_s) ? currentTime - m_previousTime : 0_s;
  m_previousTime = currentTime;

  double period = units::unit_cast<double>(deltaTime);
  auto [dx, dy, dtheta] = m_kinematics.ToChassisSpeeds(moduleStates...);
  static_cast<void>(dtheta);

  m_pose = m_pose.Exp({dx * period, dy * period,
                       units::unit_cast<double>(angularVelocity) * period});

  return m_pose;
}
}  // namespace frc
