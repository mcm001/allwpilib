/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "frc/controller/LinearQuadraticRegulator.h"

namespace frc {

LinearQuadraticRegulator<1, 1>::LinearQuadraticRegulator(
    const Eigen::Matrix<double, 1, 1>& A, const Eigen::Matrix<double, 1, 1>& B,
    const std::array<double, 1>& Qelems, const std::array<double, 1>& Relems,
    units::second_t dt)
    : LinearQuadraticRegulator(A, B, Qelems, 1.0, Relems, dt) {}

LinearQuadraticRegulator<1, 1>::LinearQuadraticRegulator(
    const Eigen::Matrix<double, 1, 1>& A, const Eigen::Matrix<double, 1, 1>& B,
    const std::array<double, 1>& Qelems, const double rho,
    const std::array<double, 1>& Relems, units::second_t dt)
    : detail::LinearQuadraticRegulatorImpl<1, 1>{A,   B,      Qelems,
                                                 rho, Relems, dt} {}

LinearQuadraticRegulator<2, 1>::LinearQuadraticRegulator(
    const Eigen::Matrix<double, 2, 2>& A, const Eigen::Matrix<double, 2, 1>& B,
    const std::array<double, 2>& Qelems, const std::array<double, 1>& Relems,
    units::second_t dt)
    : LinearQuadraticRegulator(A, B, Qelems, 1.0, Relems, dt) {}

LinearQuadraticRegulator<2, 1>::LinearQuadraticRegulator(
    const Eigen::Matrix<double, 2, 2>& A, const Eigen::Matrix<double, 2, 1>& B,
    const std::array<double, 2>& Qelems, const double rho,
    const std::array<double, 1>& Relems, units::second_t dt)
    : detail::LinearQuadraticRegulatorImpl<2, 1>{A,   B,      Qelems,
                                                 rho, Relems, dt} {}

}  // namespace frc
