/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <functional>

#include <Eigen/Core>
#include <units/current.h>
#include <units/length.h>
#include <units/velocity.h>

#include "SimLinearSystem.h"
#include "frc/StateSpaceUtil.h"
#include "frc/system/LinearSystem.h"
#include "frc/system/RungeKutta.h"
#include "frc/system/plant/DCMotor.h"

namespace frc {

namespace sim {

class SimElevator : public frc::sim::SimLinearSystem<2, 1, 1> {
 public:
  SimElevator(frc::LinearSystem<2, 1, 1>& elevatorPlant, const bool addNoise,
              const std::array<double, 1>& measurementStdDevs,
              frc::DCMotor flywheelGearbox, double gearing,
              units::meter_t drumRadius)
      : m_motor(flywheelGearbox),
        m_gearing(gearing),
        m_drumRadius(drumRadius),
        frc::sim::SimLinearSystem<2, 1, 1>(elevatorPlant, addNoise,
                                           measurementStdDevs) {}

  units::meters_per_second_squared_t ElevatorVelocity() {
    return units::meters_per_second_squared_t(Output(0));
  }

  units::meter_t ElevatorPosition() { return units::meter_t(Output(0)); }

  units::ampere_t DrawnCurrent() override {
    // I = V / R - omega / (Kv * R)
    // Reductions are greater than 1, so a reduction of 10:1 would mean the
    // motor is spinning 10x faster than the output v = r w, so w = v/r
    units::meters_per_second_t velocity =
        units::meters_per_second_t(m_trueXhat(1));
    units::radians_per_second_t motorVelocityRadPerSec =
        velocity / m_drumRadius * m_gearing * 1_rad;
    return units::volt_t(m_u(0)) / m_motor.R -
           motorVelocityRadPerSec / (m_motor.Kv * m_motor.R);
  }

 protected:
  Eigen::Matrix<double, 2, 1> UpdateXhat(
      Eigen::Matrix<double, 2, 1>& currentXhat, Eigen::Matrix<double, 1, 1>& u,
      units::second_t dt) override {
    return frc::RungeKutta(
        [&](Eigen::Matrix<double, 2, 1> x, Eigen::Matrix<double, 1, 1> u_) {
          return m_plant.A() * x + m_plant.B() * u_ +
                 frc::MakeMatrix<2, 1>(0.0, -9.8);
        },
        currentXhat, u, dt);
  }

 private:
  frc::DCMotor m_motor;
  double m_gearing;
  units::meter_t m_drumRadius;
};

}  // namespace sim
}  // namespace frc
