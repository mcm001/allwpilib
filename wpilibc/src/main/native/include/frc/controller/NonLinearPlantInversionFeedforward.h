/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <array>
#include <functional>

#include <Eigen/Core>
#include <Eigen/QR>
#include <drake/math/discrete_algebraic_riccati_equation.h>
#include <units/units.h>

#include "frc/StateSpaceUtil.h"
#include "frc/system/Discretization.h"
#include "frc/system/LinearSystem.h"
#include "frc/system/NumericalJacobian.h"

namespace frc {

template <int N>
using Vector = Eigen::Matrix<double, N, 1>;

/**
 * Constructs a plant inversion model-based feedforward from given model
 * dynamics.
 *
 * <p>B is calculated through a {@link
 * edu.wpi.first.wpilibj.system.NumericalJacobian}.
 *
 * <p>The feedforward is calculated as
 * u_ff = B<sup>+</sup> (rDot - f(x)), were B<sup>+</sup> is the pseudoinverse
 * of B.
 *
 * <p>For more on the underlying math, read
 * https://file.tavsys.net/control/controls-engineering-in-frc.pdf.
 */
template <int States, int Inputs>
class NonLinearPlantInversionFeedforward {
 public:
  /**
   * Constructs a feedforward with given model dynamics.
   *
   * @param f         A vector-valued function of x, the state, and
   *                  u, the input, that returns the derivative of
   *                  the state vector.
   * @param dtSeconds The timestep between calls of calculate().
   */
  NonLinearPlantInversionFeedforward(
      std::function<Vector<States>(const Vector<States>&,
                                   const Vector<Inputs>&)>
          f,
      units::second_t dt)
      : m_dt(dt), m_f(f) {
    m_B = NumericalJacobianU<States, States, Inputs>(f, Vector<States>::Zero(),
                                                     Vector<Inputs>::Zero());

    m_r.setZero();
    Reset(m_r);
  }

  NonLinearPlantInversionFeedforward(NonLinearPlantInversionFeedforward&&) =
      default;
  NonLinearPlantInversionFeedforward& operator=(
      NonLinearPlantInversionFeedforward&&) = default;

  /**
   * Returns the previously calculated feedforward as an input vector.
   *
   * @return The calculated feedforward.
   */
  const Eigen::Matrix<double, Inputs, 1>& Uff() const { return m_uff; }

  /**
   * Returns an element of the previously calculated feedforward.
   *
   * @param row Row of uff.
   *
   * @return The row of the calculated feedforward.
   */
  double Uff(int i) const { return m_uff(i, 0); }

  /**
   * Returns the current reference vector r.
   *
   * @return The current reference vector.
   */
  const Eigen::Matrix<double, States, 1>& R() const { return m_r; }

  /**
   * Returns an element of the reference vector r.
   *
   * @param i Row of r.
   *
   * @return The row of the current reference vector.
   */
  double R(int i) const { return m_r(i, 0); }

  /**
   * Resets the feedforward with a specified initial state vector.
   *
   * @param initialState The initial state vector.
   */
  void Reset(const Eigen::Matrix<double, States, 1>& initalState) {
    m_r = initalState;
    m_uff.setZero();
  }

  /**
   * Calculate the feedforward with only the future reference. This
   * uses the internally stored previous reference.
   *
   * @param nextR The future reference state of time k + dt.
   *
   * @return The calculated feedforward.
   */
  Eigen::Matrix<double, Inputs, 1> Calculate(
      const Eigen::Matrix<double, States, 1>& nextR) {
    return Calculate(m_r, nextR);
  }

  /**
   * Calculate the feedforward with current anf future reference vectors.
   *
   * @param r     The current reference state of time k.
   * @param nextR The future reference state of time k + dt.
   *
   * @return The calculated feedforward.
   */
  Eigen::Matrix<double, Inputs, 1> Calculate(
      const Eigen::Matrix<double, States, 1>& r,
      const Eigen::Matrix<double, States, 1>& nextR) {
    Vector<States> rDot = (nextR - r) / m_dt.to<double>();

    m_uff = m_B.householderQr().solve(rDot - m_f(r, Vector<Inputs>::Zero()));

    m_r = nextR;
    return m_uff;
  }

 private:
  Eigen::Matrix<double, States, Inputs> m_B;

  units::second_t m_dt;

  /**
   * The model dynamics, if the overload is used.
   */
  std::function<Vector<States>(const Vector<States>&, const Vector<Inputs>&)>
      m_f;

  // Current reference
  Vector<States> m_r;

  // Computed feedforward
  Vector<Inputs> m_uff;
};

}  // namespace frc
