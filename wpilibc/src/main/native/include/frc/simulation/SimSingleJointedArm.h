/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <functional>

#include <Eigen/Core>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/mass.h>

#include "SimLinearSystem.h"
#include "frc/StateSpaceUtil.h"
#include "frc/system/LinearSystem.h"
#include "frc/system/RungeKutta.h"
#include "frc/system/plant/DCMotor.h"

namespace frc {

namespace sim {

class SimSingleJointedArm : public frc::sim::SimLinearSystem<2, 1, 1> {
 public:
  /**
   * Create a LinearSystemSimulator. This simulator uses an {@link LinearSystem}
   * to simulate the state of the system. In simulationPeriodic, users should
   * first set inputs from motors, update the simulation and write simulated
   * outputs to sensors.
   *
   * @param armPlant           The arm system being controlled.
   * @param addNoise           If we should add noise to the measurement
   *                           vector.
   * @param measurementStdDevs Standard deviations of measurements. Can be null
   *                           if addNoise is false.
   * @param armMass          The mass of the arm.
   * @param armLength    The distance from the pivot of the arm to its center
   *                           of rotation. This number is not the same as the
   * overall length of the arm.
   * @param motor              The DCMotor used to drive the arm.
   * @param gearing            The gearing between the motor and the output,
   * written as output over input. In most cases this should be greater than
   * one.
   */
  SimSingleJointedArm(frc::LinearSystem<2, 1, 1>& armPlant, const bool addNoise,
                      const std::array<double, 1>& measurementStdDevs,
                      units::kilogram_t armMass, units::meter_t armLength,
                      frc::DCMotor motor, double gearing)
      : m_armLength(armLength),
        m_armMass(armMass),
        m_motor(motor),
        m_gearing(gearing),
        frc::sim::SimLinearSystem<2, 1, 1>(armPlant, addNoise,
                                           measurementStdDevs) {}

  units::radian_t Position() { return units::radian_t(Output(0)); }

  units::radians_per_second_t Velocity() {
    return units::radians_per_second_t(Output(1));
  }

  units::ampere_t DrawnCurrent() override {
    // I = V / R - omega / (Kv * R)
    // Reductions are output over input, so a reduction of 2:1 means the motor
    // is spinning 2x faster than the output
    return units::volt_t(m_u(0)) / m_motor.R -
           Velocity() * m_gearing / (m_motor.Kv * m_motor.R);
  }

 protected:
  Eigen::Matrix<double, 2, 1> UpdateXhat(
      Eigen::Matrix<double, 2, 1>& currentXhat, Eigen::Matrix<double, 1, 1>& u,
      units::second_t dt) override {
    /*
    Horizontal case:
    Torque = f * r = I * alpha
    alpha = f * r / I
    since f=mg,
    alpha = m g r / I

    Multiply RHS by cos(theta) to account for the arm angle
    */
    return frc::RungeKutta(
        [&](Eigen::Matrix<double, 2, 1> x, Eigen::Matrix<double, 1, 1> u_) {
          return m_plant.A() * x + m_plant.B() * u_ +
                 frc::MakeMatrix<2, 1>(0.0,
                                       (m_armMass * m_armLength * -9.8 /
                                        (m_armMass * m_armLength * m_armLength))
                                           .template to<double>());
        },
        currentXhat, u, dt);
  }

 private:
  units::kilogram_t m_armMass;
  units::meter_t m_armLength;
  frc::DCMotor m_motor;
  double m_gearing;
};

}  // namespace sim
}  // namespace frc
