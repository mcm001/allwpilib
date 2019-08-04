/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <algorithm>

namespace frc {

template <class... Wheels>
SwerveDriveKinematics(Translation2d, Wheels...)
    ->SwerveDriveKinematics<1 + sizeof...(Wheels)>;

template <size_t NumModules>
std::array<SwerveModuleState, NumModules>
SwerveDriveKinematics<NumModules>::ToSwerveModuleStates(
    const ChassisSpeeds& chassisSpeeds, const Translation2d& centerOfRotation) {
  // We have a new center of rotation. We need to compute the matrix again.
  if (centerOfRotation.X() != m_previousCoR.X() ||
      centerOfRotation.Y() != m_previousCoR.Y()) {
    for (size_t i = 0; i < NumModules; i++) {
      // clang-format off
      m_inverseKinematics.template block<2, 3>(i * 2, 0) <<
        1, 0, -m_modules[i].Y() + centerOfRotation.Y(),
        0, 1, +m_modules[i].X() - centerOfRotation.X();
      // clang-format on
    }
    m_previousCoR = centerOfRotation;
  }

  Eigen::Vector3d chassisSpeedsVector;
  chassisSpeedsVector << chassisSpeeds.dx, chassisSpeeds.dy,
      chassisSpeeds.dtheta;

  Eigen::Matrix<double, NumModules * 2, 1> moduleStatesMatrix =
      m_inverseKinematics * chassisSpeedsVector;
  std::array<SwerveModuleState, NumModules> moduleStates;

  for (size_t i = 0; i < NumModules; i++) {
    double x = moduleStatesMatrix(i * 2, 0);
    double y = moduleStatesMatrix(i * 2 + 1, 0);

    double speed = std::hypot(x, y);
    Rotation2d rotation{x, y};

    moduleStates[i] = {speed, rotation};
  }

  return moduleStates;
}

template <size_t NumModules>
template <typename... ModuleStates>
ChassisSpeeds SwerveDriveKinematics<NumModules>::ToChassisSpeeds(
    ModuleStates&&... wheelStates) {
  static_assert(sizeof...(wheelStates) == NumModules,
                "Number of modules is not consistent with number of wheel "
                "locations provided in constructor.");

  std::array<SwerveModuleState, NumModules> moduleStates{wheelStates...};
  Eigen::Matrix<double, NumModules * 2, 1> moduleStatesMatrix;

  for (size_t i = 0; i < NumModules; i++) {
    SwerveModuleState module = moduleStates[i];
    moduleStatesMatrix.row(i * 2) << module.speed * module.angle.Cos();
    moduleStatesMatrix.row(i * 2 + 1) << module.speed * module.angle.Sin();
  }

  Eigen::Vector3d chassisSpeedsVector =
      m_forwardKinematics.solve(moduleStatesMatrix);

  return {chassisSpeedsVector(0), chassisSpeedsVector(1),
          chassisSpeedsVector(2)};
}

template <size_t NumModules>
void SwerveDriveKinematics<NumModules>::NormalizeWheelSpeeds(
    std::array<SwerveModuleState, NumModules>* moduleStates,
    double attainableMaxSpeed) {
  auto& states = *moduleStates;
  double realMaxSpeed =
      std::max_element(states.begin(), states.end(),
                       [](const auto& a, const auto& b) {
                         return std::abs(a.speed) < std::abs(b.speed);
                       })
          ->speed;

  if (realMaxSpeed > attainableMaxSpeed) {
    for (auto& module : states) {
      module.speed = module.speed / realMaxSpeed * attainableMaxSpeed;
    }
  }
}

}  // namespace frc
