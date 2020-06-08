/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "frc/trajectory/constraint/DifferentialDriveVelocitySystemConstraint.h"

#include <algorithm>
#include <limits>

#include <units/units.h>
#include <wpi/MathExtras.h>

using namespace frc;

DifferentialDriveVelocitySystemConstraint::
    DifferentialDriveVelocitySystemConstraint(
        LinearSystem<2, 2, 2> system, DifferentialDriveKinematics kinematics,
        units::volt_t maxVoltage)
    : m_system(system), m_kinematics(kinematics), m_maxVoltage(maxVoltage) {}

units::meters_per_second_t
DifferentialDriveVelocitySystemConstraint::MaxVelocity(
    const Pose2d& pose, units::curvature_t curvature,
    units::meters_per_second_t velocity) const {
  return units::meters_per_second_t(std::numeric_limits<double>::max());
}

TrajectoryConstraint::MinMax
DifferentialDriveVelocitySystemConstraint::MinMaxAcceleration(
    const Pose2d& pose, units::curvature_t curvature,
    units::meters_per_second_t speed) const {
  auto wheelSpeeds =
      m_kinematics.ToWheelSpeeds({speed, 0_mps, speed * curvature});

  Eigen::Vector2d x;
  x << wheelSpeeds.left.to<double>(), wheelSpeeds.right.to<double>();

  Eigen::Vector2d xDot;
  Eigen::Vector2d u;

  // dx/dt for minimum u
  u << -m_maxVoltage.to<double>(), -m_maxVoltage.to<double>();
  xDot = m_system.A() * x + m_system.B() * u;
  units::meters_per_second_squared_t minChassisAcceleration;
  minChassisAcceleration =
      units::meters_per_second_squared_t((xDot(0, 0) + xDot(1, 0)) / 2.0);

  // dx/dt for maximum u
  u << m_maxVoltage.to<double>(), m_maxVoltage.to<double>();
  xDot = m_system.A() * x + m_system.B() * u;
  units::meters_per_second_squared_t maxChassisAcceleration;
  maxChassisAcceleration =
      units::meters_per_second_squared_t((xDot(0, 0) + xDot(1, 0)) / 2.0);

  return {minChassisAcceleration, maxChassisAcceleration};
}