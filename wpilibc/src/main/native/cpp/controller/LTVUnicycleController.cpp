/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "frc/controller/LTVUnicycleController.h"

#include <cmath>

#include <wpi/MathExtras.h>

#include "frc/controller/LinearQuadraticRegulator.h"

using namespace frc;

LTVUnicycleController::LTVUnicycleController(
    const std::array<double, 3>& Qelems, const std::array<double, 2>& Relems,
    units::second_t dt)
    : LTVUnicycleController(Qelems, 1.0, Relems, dt) {}

LTVUnicycleController::LTVUnicycleController(
    const std::array<double, 3>& Qelems, const double rho,
    const std::array<double, 2>& Relems, units::second_t dt)
    : m_dt(dt),
    m_Qelms(Qelems),
    m_rho(rho),
    m_Relems(Relems){

  m_B << 1, 0, 0, 0, 0, 1;
}

bool LTVUnicycleController::AtReference() const {
  const auto& eTranslate = m_poseError.Translation();
  const auto& eRotate = m_poseError.Rotation();
  const auto& tolTranslate = m_poseTolerance.Translation();
  const auto& tolRotate = m_poseTolerance.Rotation();
  return units::math::abs(eTranslate.X()) < tolTranslate.X() &&
         units::math::abs(eTranslate.Y()) < tolTranslate.Y() &&
         units::math::abs(eRotate.Radians()) < tolRotate.Radians();
}

void LTVUnicycleController::SetTolerance(const Pose2d& poseTolerance) {
  m_poseTolerance = poseTolerance;
}

ChassisSpeeds LTVUnicycleController::Calculate(
    const Pose2d& currentPose,
    units::meters_per_second_t currentLinearVelocity,
    const Pose2d& poseRef,
    units::meters_per_second_t linearVelocityRef,
    units::radians_per_second_t angularVelocityRef) {
  m_poseError = poseRef.RelativeTo(currentPose);

  if (currentLinearVelocity.to<double>() < 1e-9) {
    currentLinearVelocity = 1e-9_mps;
  }

  Eigen::Matrix<double, 3, 3> A;
  A << 0, 0, 0, 0, 0, currentLinearVelocity.to<double>(), 0, 0, 0;

  Eigen::Matrix<double, 2, 3> K = LinearQuadraticRegulator<3, 2>(A, m_B, m_Qelms, m_rho, m_Relems, m_dt).K();

  Eigen::Matrix<double, 3, 1> error;
  error(0, 0) = m_poseError.Translation().X().to<double>();
  error(1, 0) = m_poseError.Translation().Y().to<double>();
  error(2, 0) = m_poseError.Rotation().Radians().to<double>();

  auto u = K * error;

  return ChassisSpeeds{
      linearVelocityRef + units::meters_per_second_t{u(0, 0)}, 0_mps,
      angularVelocityRef + units::radians_per_second_t{u(1, 0)}};
}

ChassisSpeeds LTVUnicycleController::Calculate(
    const Pose2d& currentPose, units::meters_per_second_t currentLinearVelocity,
    const Trajectory::State& desiredState) {
  return Calculate(currentPose, currentLinearVelocity,
                  desiredState.pose, desiredState.velocity,
                   desiredState.velocity * desiredState.curvature);
}
