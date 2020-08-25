/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "frc/simulation/FlywheelSim.h"

using namespace frc;
using namespace frc::sim;

FlywheelSim::FlywheelSim(const LinearSystem<1, 1, 1>& plant, bool addNoise,
                         const std::array<double, 1>& measurementStdDevs,
                         const DCMotor& gearbox, double gearing)
    : LinearSystemSim<1, 1, 1>(plant, addNoise, measurementStdDevs),
      m_motor(gearbox),
      m_gearing(gearing) {}

units::radians_per_second_t FlywheelSim::GetVelocity() const {
  return units::radians_per_second_t(Y()(0, 0));
}

units::ampere_t FlywheelSim::GetCurrentDraw() const {
  // I = V / R - omega / (Kv * R)
  // Reductions are greater than 1, so a reduction of 10:1 would mean the motor
  // is spinning 10x faster than the output.
  return units::volt_t(m_u(0, 0)) / m_motor.R -
         GetVelocity() * m_gearing / (m_motor.Kv * m_motor.R);
}
