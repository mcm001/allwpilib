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
class LinearSystemSim {
 public:
  LinearSystemSim(const LinearSystem<States, Inputs, Outputs>& system,
                  bool addNoise,
                  const std::array<double, Outputs>& measurementStdDevs)
      : m_plant(system),
        m_shouldAddNoise(addNoise),
        m_measurementStdDevs(measurementStdDevs) {
    m_trueXhat = Eigen::Matrix<double, States, 1>::Zero();
    m_y = Eigen::Matrix<double, Outputs, 1>::Zero();
    m_u = Eigen::Matrix<double, Inputs, 1>::Zero();
  }

  bool ShouldAddNoise() const { return m_shouldAddNoise; }
  const Eigen::Matrix<double, Outputs, 1>& Y() const { return m_y; }

  void SetShouldAddNoise(bool shouldAddNoise) {
    m_shouldAddNoise = shouldAddNoise;
  }

  void Update(units::second_t dt) {
    // x = Ax + Bu
    m_trueXhat = UpdateXhat(m_trueXhat, m_u, dt);

    // y = Cx + Du
    m_y = m_plant.CalculateY(m_trueXhat, m_u);

    // Add noise if needed.
    if (m_shouldAddNoise) {
      m_y += frc::MakeWhiteNoiseVector<Outputs>(m_measurementStdDevs);
    }
  }

  void SetInput(const Eigen::Matrix<double, Inputs, 1>& u) { m_u = u; }

  void SetInput(size_t row, double value) { m_u(row, 0) = value; }

  void ResetState(const Eigen::Matrix<double, States, 1>& state) {
    m_trueXhat = state;
  }

 protected:
  virtual Eigen::Matrix<double, States, 1> UpdateXhat(
      const Eigen::Matrix<double, States, 1>& currentXhat,
      const Eigen::Matrix<double, Inputs, 1>& u, units::second_t dt) {
    return m_plant.CalculateX(currentXhat, u, dt);
  }

  virtual units::ampere_t GetCurrentDraw() const {
    return units::ampere_t(0.0);
  }

  LinearSystem<States, Inputs, Outputs> m_plant;
  bool m_shouldAddNoise;

  Eigen::Matrix<double, States, 1> m_trueXhat;
  Eigen::Matrix<double, Outputs, 1> m_y;
  Eigen::Matrix<double, Inputs, 1> m_u;
  std::array<double, Outputs> m_measurementStdDevs;
};
}  // namespace sim
}  // namespace frc
