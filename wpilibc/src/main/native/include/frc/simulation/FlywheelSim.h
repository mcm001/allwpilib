/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <units/angular_velocity.h>

#include "frc/simulation/LinearSystemSim.h"
#include "frc/system/LinearSystem.h"
#include "frc/system/plant/DCMotor.h"

namespace frc {
namespace sim {
class FlywheelSim : public LinearSystemSim<1, 1, 1> {
 public:
  FlywheelSim(const LinearSystem<1, 1, 1>& plant, bool addNoise,
              const std::array<double, 1>& measurementStdDevs,
              const DCMotor& gearbox, double gearing);

  units::radians_per_second_t GetVelocity() const;
  units::ampere_t GetCurrentDraw() const override;

 private:
  DCMotor m_motor;
  double m_gearing;
};
}  // namespace sim
}  // namespace frc
