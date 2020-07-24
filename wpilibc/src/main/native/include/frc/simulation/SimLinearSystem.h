/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <array>

#include <Eigen/Core>
#include <units/current.h>
#include <units/time.h>

#include "frc/StateSpaceUtil.h"
#include "frc/system/LinearSystem.h"

namespace frc {

namespace sim {

template <int States, int Inputs, int Outputs>
class SimLinearSystem {
 public:
  /**
   * Create a SimLinearSystem. This simulator uses an {@link LinearSystem} to
   * simulate the state of the system. In simulationPeriodic, users should first
   * set inputs from motors, update the simulation and write simulated outputs
   * to sensors.
   *
   * @param system             The system being controlled.
   * @param addNoise           If we should add noise to the measurement vector
   * @param measurementStdDevs Standard deviations of measurements. Can be null
   * if addNoise is false.
   */
  SimLinearSystem(frc::LinearSystem<States, Inputs, Outputs>& plant,
                  const bool addNoise,
                  const std::array<double, Outputs>& measurementStdDevs)
      : m_plant(plant),
        m_shouldAddNoise(addNoise),
        m_measurementStdDevs(measurementStdDevs) {
    m_trueXhat.setZero();
    m_u.setZero();
    m_y.setZero();
  }

  void Update(units::second_t dt) {
    m_trueXhat = UpdateXhat(m_trueXhat, m_u, dt);
    m_y = m_plant.CalculateY(m_trueXhat, m_u);

    if (m_shouldAddNoise) {
      m_y += frc::MakeWhiteNoiseVector<Outputs>(m_measurementStdDevs);
    }
  }

  Eigen::Matrix<double, Outputs, 1>& Getputs() { return m_y; }

  double Output(int row) const { return m_y(row); }

  void SetInput(const Eigen::Matrix<double, Inputs, 1>& u) { m_u = u; }

  /**
   * Get the current drawn by this simulated system. Override this method to add
   * current calculation.
   *
   * @return The currently drawn current.
   */
  virtual units::ampere_t DrawnCurrent() { return units::ampere_t(0.0); }

 protected:
  LinearSystem<States, Inputs, Outputs> m_plant;
  bool m_shouldAddNoise;
  Eigen::Matrix<double, States, 1> m_trueXhat;
  Eigen::Matrix<double, Outputs, 1> m_y;
  Eigen::Matrix<double, Inputs, 1> m_u;
  std::array<double, Outputs> m_measurementStdDevs;

  virtual Eigen::Matrix<double, States, 1> UpdateXhat(
      Eigen::Matrix<double, States, 1>& currentXhat,
      Eigen::Matrix<double, Inputs, 1>& u, units::second_t dt) {
    return m_plant.CalculateX(currentXhat, u, dt);
  }
};

}  // namespace sim
}  // namespace frc
