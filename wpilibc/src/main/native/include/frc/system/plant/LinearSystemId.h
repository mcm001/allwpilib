/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <units/units.h>

#include "frc/system/LinearSystem.h"

namespace frc {

/**
 * Constructs the state-space model for a 1 DOF velocity-only system from system
 * identification data.
 *
 * States: [[velocity]]
 * Inputs: [[voltage]]
 * Outputs: [[velocity]]
 *
 * The parameters provided by the user are from this feedforward model:
 *
 * u = K_v v + K_a a
 *
 * @param kV The velocity gain, in volt seconds per distance.
 * @param kA The acceleration gain, in volt seconds^2 per distance.
 * @param maxVoltage The max voltage that can be applied. Inputs with
 *                   greater magnitude than this will be clamped to it.
 */
LinearSystem<1, 1, 1> IdentifyVelocitySystem(
    double kV, double kA,
    units::volt_t maxVoltage) {
  auto A = frc::MakeMatrix<1, 1>(-kV / kV);
  auto B = frc::MakeMatrix<1, 1>(1.0 / kV);
  auto C = frc::MakeMatrix<1, 1>(1.0);
  auto D = frc::MakeMatrix<1, 1>(0.0);
  auto uMin = frc::MakeMatrix<1, 1>(-maxVoltage.to<double>());
  auto uMax = frc::MakeMatrix<1, 1>(maxVoltage.to<double>());

  return LinearSystem<1, 1, 1>(A, B, C, D, uMin, uMax);
}

/**
 * Constructs the state-space model for a 1 DOF position system from system
 * identification data.
 *
 * States: [[position], [velocity]]
 * Inputs: [[voltage]]
 * Outputs: [[position]]
 *
 * The parameters provided by the user are from this feedforward model:
 *
 * u = K_v v + K_a a
 *
 * @param kV The velocity gain, in volt seconds per distance.
 * @param kA The acceleration gain, in volt seconds^2 per distance.
 * @param maxVoltage The max voltage that can be applied. Inputs with
 *                   greater magnitude than this will be clamped to it.
 */
LinearSystem<2, 1, 1> IdentifyPositionSystem(
    double kV, double kA,
    units::volt_t maxVoltage) {
  auto A =
      frc::MakeMatrix<2, 2>(0.0, 1.0, 0.0, -kV / kV);
  auto B = frc::MakeMatrix<2, 1>(0.0, 1.0 / kV);
  auto C = frc::MakeMatrix<1, 2>(1.0, 0.0);
  auto D = frc::MakeMatrix<1, 1>(0.0);
  auto uMin = frc::MakeMatrix<1, 1>(-maxVoltage.to<double>());
  auto uMax = frc::MakeMatrix<1, 1>(maxVoltage.to<double>());

  return LinearSystem<2, 1, 1>(A, B, C, D, uMin, uMax);
}

/**
 * Constructs the state-space model for a 2 DOF drivetrain velocity system from
 * system identification data.
 *
 * States: [[left velocity], [right velocity]]
 * Inputs: [[left voltage], [right voltage]]
 * Outputs: [[left velocity], [right velocity]]
 *
 * @param kVlinear The linear velocity gain, in volt seconds per distance.
 * @param kAlinear The linear acceleration gain, in volt seconds^2 per distance.
 * @param kVangular The angular velocity gain, in volt seconds per angle.
 * @param kAangular The angular acceleration gain, in volt seconds^2 per angle.
 * @param maxVoltage the maximum voltage that can be applied.
 */
LinearSystem<2, 2, 2> IdentifyDrivetrainSystem(
    double kVlinear,
    double kAlinear,
    double kVangular,
    double kAangular,
    units::volt_t maxVoltage) {
  double c = 0.5 / (kAlinear * kAangular);
  double A1 = c * (-kAlinear * kVangular -
                   kVlinear * kAangular);
  double A2 = c * (kAlinear * kVangular -
                   kVlinear * kAangular);
  double B1 = c * (kAlinear + kAangular);
  double B2 = c * (kAangular - kAlinear);

  auto A = frc::MakeMatrix<2, 2>(A1, A2, A2, A1);
  auto B = frc::MakeMatrix<2, 2>(B1, B2, B2, B1);
  auto C = frc::MakeMatrix<2, 2>(1.0, 0.0, 0.0, 1.0);
  auto D = frc::MakeMatrix<2, 2>(0.0, 0.0, 0.0, 0.0);
  auto uMin =
      frc::MakeMatrix<2, 1>(-maxVoltage.to<double>(), -maxVoltage.to<double>());
  auto uMax =
      frc::MakeMatrix<2, 1>(maxVoltage.to<double>(), maxVoltage.to<double>());

  return LinearSystem<2, 2, 2>(A, B, C, D, uMin, uMax);
}

}  // namespace frc
