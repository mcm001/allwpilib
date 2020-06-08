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
#include <units/units.h>

#include "frc/system/NumericalJacobian.h"

namespace frc {

template <int N>
using Vector = Eigen::Matrix<double, N, 1>;

/**
 * Constructs a control-affine plant inversion model-based feedforward from
 * given model dynamics.
 *
 * If given the vector valued function as f(x, u) where x is the state
 * vector and u is the input vector, the B matrix(continuous input matrix)
 * is calculated through a NumericalJacobian. In this case f has to be
 * control-affine (of the form f(x) + Bu).
 *
 * The feedforward is calculated as
 * u_ff = B<sup>+</sup> (rDot - f(x)), were B<sup>+</sup> is the pseudoinverse
 * of B.
 *
 * This feedforward does not account for a dynamic B matrix, B is either
 * determined or supplied when the feedforward is created and remains constant.
 *
 * <p>For more on the underlying math, read
 * https://file.tavsys.net/control/controls-engineering-in-frc.pdf.
 */
template <int States, int Inputs>
class ControlAffinePlantInversionFeedforward {
 public:
  /**
   * Constructs a feedforward with given model dynamics as a function
   * of state and input.
   *
   * @param f  A vector-valued function of x, the state, and
   *           u, the input, that returns the derivative of
   *           the state vector. HAS to be control-affine
   *           (of the form f(x) + Bu).
   * @param dt The timestep between calls of calculate().
   */
  ControlAffinePlantInversionFeedforward(
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

  /**
   * Constructs a feedforward with given model dynamics as a function of state,
   * and the plant's B matrix(continuous input matrix).
   *
   * @param f  A vector-valued function of x, the state,
   *           that returns the derivative of the state vector.
   * @param B  Continuous input matrix of the plant being controlled.
   * @param dt The timestep between calls of calculate().
   */
  ControlAffinePlantInversionFeedforward(
      std::function<Vector<States>(const Vector<States>&)> f,
      const Eigen::Matrix<double, States, Inputs>& B, units::second_t dt)
      : m_dt(dt), m_B(B) {
    m_f = [=](const Vector<States>& x,
              const Vector<Inputs>& u) -> Vector<States> { return f(x); };

    m_r.setZero();
    Reset(m_r);
  }

  ControlAffinePlantInversionFeedforward(
      ControlAffinePlantInversionFeedforward&&) = default;
  ControlAffinePlantInversionFeedforward& operator=(
      ControlAffinePlantInversionFeedforward&&) = default;

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
   * uses the internally stored current reference.
   *
   * @param nextR The reference state of the future timestep(k + dt).
   *
   * @return The calculated feedforward.
   */
  Eigen::Matrix<double, Inputs, 1> Calculate(
      const Eigen::Matrix<double, States, 1>& nextR) {
    return Calculate(m_r, nextR);
  }

  /**
   * Calculate the feedforward with current and future reference vectors.
   *
   * @param r     The reference state of the current timestep(k).
   * @param nextR The reference state of the future timestep(k + dt).
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
   * The model dynamics.
   */
  std::function<Vector<States>(const Vector<States>&, const Vector<Inputs>&)>
      m_f;

  // Current reference
  Vector<States> m_r;

  // Computed feedforward
  Vector<Inputs> m_uff;
};

}  // namespace frc
