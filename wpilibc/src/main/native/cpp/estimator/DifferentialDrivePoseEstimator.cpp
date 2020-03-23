/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "frc/estimator/DifferentialDrivePoseEstimator.h"

#include <array>

#include "frc/StateSpaceUtil.h"
#include "frc2/Timer.h"

using namespace frc;

DifferentialDrivePoseEstimator::DifferentialDrivePoseEstimator(
    const Rotation2d& gyroAngle, const Pose2d& initialPose,
    const Vector<3>& stateStdDevs, const Vector<3>& measurementStdDevs,
    units::second_t nominalDt)
    : m_observer(&DifferentialDrivePoseEstimator::F,
                 [](const Vector<3>& x, const Vector<3>& u) { return x; },
                 StdDevMatrixToArray(stateStdDevs),
                 StdDevMatrixToArray(measurementStdDevs), nominalDt),
      m_nominalDt(nominalDt) {
  m_gyroOffset = initialPose.Rotation() - gyroAngle;
  m_previousAngle = initialPose.Rotation();
  m_observer.SetXhat(PoseToVector(initialPose));
}

void DifferentialDrivePoseEstimator::ResetPosition(
    const Pose2d& pose, const Rotation2d& gyroAngle) {
  m_previousAngle = pose.Rotation();
  m_gyroOffset = GetEstimatedPosition().Rotation() - gyroAngle;

  m_prevLeftDistance = 0_m;
  m_prevRightDistance = 0_m;
}

Pose2d DifferentialDrivePoseEstimator::GetEstimatedPosition() const {
  return Pose2d(units::meter_t(m_observer.Xhat(0)),
                units::meter_t(m_observer.Xhat(1)),
                Rotation2d(units::radian_t(m_observer.Xhat(2))));
}

void DifferentialDrivePoseEstimator::AddVisionMeasurement(
    const Pose2d& visionRobotPose, units::second_t timestamp) {
  m_latencyCompensator.ApplyPastMeasurement(
      m_observer, m_nominalDt, PoseToVector(visionRobotPose), timestamp);
}

Pose2d DifferentialDrivePoseEstimator::Update(const Rotation2d& gyroAngle,
                                              units::meter_t leftDistance,
                                              units::meter_t rightDistance) {
  return UpdateWithTime(frc2::Timer::GetFPGATimestamp(), gyroAngle,
                        leftDistance, rightDistance);
}

Pose2d DifferentialDrivePoseEstimator::UpdateWithTime(
    units::second_t currentTime, const Rotation2d& gyroAngle,
    units::meter_t leftDistance, units::meter_t rightDistance) {
  auto angle = gyroAngle + m_gyroOffset;

  auto deltaLeftDistance = leftDistance - m_prevLeftDistance;
  auto deltaRightDistance = rightDistance - m_prevRightDistance;

  m_prevLeftDistance = leftDistance;
  m_prevRightDistance = rightDistance;

  auto u = frc::MakeMatrix<3, 1>(
      deltaLeftDistance.to<double>(), deltaRightDistance.to<double>(),
      (angle - m_previousAngle).Radians().to<double>());

  m_previousAngle = angle;

  auto dt = m_prevTime >= 0_s ? currentTime - m_prevTime : 0_s;
  m_prevTime = currentTime;

  m_latencyCompensator.AddObserverState(m_observer, u, currentTime);
  m_observer.Predict(u, dt);

  return GetEstimatedPosition();
}

Eigen::Matrix<double, 3, 1> DifferentialDrivePoseEstimator::PoseToVector(
    const Pose2d& pose) {
  return frc::MakeMatrix<3, 1>(pose.Translation().X().to<double>(),
      pose.Translation().Y().to<double>(),
      pose.Rotation().Radians().to<double>());
}

Eigen::Matrix<double, 3, 1> DifferentialDrivePoseEstimator::F(
    const Vector<3>& x, const Vector<3>& u) {
  // Differential drive forward kinematics
  // v_c = (v_l + v_r) / 2
  units::meter_t dx{(u(0, 0) + u(1, 0)) / 2.0};
  auto newPose = Pose2d{units::meter_t{x(0, 0)}, units::meter_t{x(1, 0)},
                        Rotation2d{units::radian_t(x(2, 0))}}
                     .Exp({dx, 0_m, units::radian_t(u(2, 0))});

  return frc::MakeMatrix<3, 1>(newPose.Translation().X().to<double>(),
                               newPose.Translation().Y().to<double>(),
                               x(2, 0) + u(2, 0));
}

std::array<double, 3> DifferentialDrivePoseEstimator::StdDevMatrixToArray(
    const Vector<3>& stdDevs) {
  return std::array<double, 3>{stdDevs(0, 0), stdDevs(1, 0), stdDevs(2, 0)};
}
