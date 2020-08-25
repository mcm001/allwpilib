/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <Eigen/Core>
#include <units/angular_velocity.h>
#include <units/current.h>

#include "SimLinearSystem.h"
#include "frc/StateSpaceUtil.h"
#include "frc/system/LinearSystem.h"
#include "frc/system/plant/DCMotor.h"

namespace frc {

namespace sim {

class SimFlywheel : public frc::sim::SimLinearSystem<1, 1, 1> {
 public:
  SimFlywheel(frc::LinearSystem<1, 1, 1>& flywheelPlant, const bool addNoise,
              const std::array<double, 1>& measurementStdDevs,
              frc::DCMotor flywheelGearbox, double gearing)
      : m_motor(flywheelGearbox),
        m_gearing(gearing),
        frc::sim::SimLinearSystem<1, 1, 1>(flywheelPlant, addNoise,
                                           measurementStdDevs) {}

  units::radians_per_second_t Velocity() {
    return units::radians_per_second_t(Output(0));
  }

  units::ampere_t DrawnCurrent() override {
    // I = V / R - omega / (Kv * R)
    // Reductions are output over input, so a reduction of 2:1 means the motor
    // is spinning 2x faster than the flywheel
    return units::volt_t(m_u(0)) / m_motor.R -
           Velocity() * m_gearing / (m_motor.Kv * m_motor.R);
  }

 private:
  frc::DCMotor m_motor;
  double m_gearing;
};

}  // namespace sim
}  // namespace frc
