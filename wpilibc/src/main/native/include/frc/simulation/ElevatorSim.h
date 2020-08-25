/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <array>

#include <units/length.h>
#include <units/mass.h>
#include <units/velocity.h>

#include "frc/simulation/LinearSystemSim.h"
#include "frc/system/plant/DCMotor.h"

namespace frc {
namespace sim {
class ElevatorSim : public LinearSystemSim<2, 1, 1> {
 public:
  ElevatorSim(const DCMotor& gearbox, units::kilogram_t carriageMass,
              double gearing, units::meter_t drumRadius,
              units::meter_t minHeight, units::meter_t maxHeight,
              bool addNoise = false,
              const std::array<double, 1>& m_measurementStdDevs = {});

  bool HasHitLowerLimit(const Eigen::Matrix<double, 2, 1>& x) const;
  bool HasHitUpperLimit(const Eigen::Matrix<double, 2, 1>& x) const;

  units::meter_t GetElevatorPosition() const;
  units::meters_per_second_t GetElevatorVelocity() const;

  units::ampere_t GetCurrentDraw() const override;

 protected:
  Eigen::Matrix<double, 2, 1> UpdateXhat(
      const Eigen::Matrix<double, 2, 1>& currentXhat,
      const Eigen::Matrix<double, 1, 1>& u, units::second_t dt) override;

 private:
  DCMotor m_motor;
  units::meter_t m_drumRadius;
  units::meter_t m_minHeight;
  units::meter_t m_maxHeight;
  double m_gearing;
};
}  // namespace sim
}  // namespace frc
