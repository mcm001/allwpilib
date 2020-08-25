/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <array>

#include <units/angle.h>
#include <units/length.h>
#include <units/mass.h>
#include <units/moment_of_inertia.h>

#include "frc/simulation/LinearSystemSim.h"
#include "frc/system/plant/DCMotor.h"

namespace frc {
namespace sim {
class SingleJointedArmSim : public LinearSystemSim<2, 1, 1> {
 public:
  SingleJointedArmSim(const LinearSystem<2, 1, 1>& system,
                      units::kilogram_t mass, units::meter_t armLength,
                      units::radian_t minAngle, units::radian_t maxAngle,
                      bool addNoise,
                      const std::array<double, 1>& measurementStdDevs);

  SingleJointedArmSim(const DCMotor& motor, units::kilogram_square_meter_t j,
                      double G, units::kilogram_t mass,
                      units::meter_t armLength, units::radian_t minAngle,
                      units::radian_t maxAngle, bool addNoise,
                      const std::array<double, 1>& measurementStdDevs);

  SingleJointedArmSim(const DCMotor& motor, double G, units::kilogram_t mass,
                      units::meter_t armLength, units::radian_t minAngle,
                      units::radian_t maxAngle, bool addNoise,
                      const std::array<double, 1>& measurementStdDevs);

  bool HasHitLowerLimit(const Eigen::Matrix<double, 2, 1>& x) const;
  bool HasHitUpperLimit(const Eigen::Matrix<double, 2, 1>& x) const;

  units::radian_t GetAngle() const;
  units::radians_per_second_t GetVelocity() const;

  Eigen::Matrix<double, 2, 1> UpdateXhat(
      const Eigen::Matrix<double, 2, 1>& currentXhat,
      const Eigen::Matrix<double, 1, 1>& u, units::second_t dt) override;

 private:
  units::meter_t m_r;
  units::radian_t m_minAngle;
  units::radian_t m_maxAngle;
  units::kilogram_t m_mass;
};
}  // namespace sim
}  // namespace frc
