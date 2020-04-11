/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "frc/estimator/DifferentialDrivePoseEstimator.h"

#include "frc/StateSpaceUtil.h"
#include "frc2/Timer.h"

using namespace frc;

DifferentialDrivePoseEstimator::DifferentialDrivePoseEstimator(
    const Rotation2d& gyroAngle, const Pose2d& initialPose,
    const Vector<5>& stateStdDevs, const Vector<3>& localMeasurementStdDevs,
    const Vector<3>& visionMeasurmentStdDevs, units::second_t nominalDt)
    : m_observer(
          &DifferentialDrivePoseEstimator::F,
          [](const Vector<5>& x, const Vector<3>& u) {
            return frc::MakeMatrix<3, 1>(x(3, 0), x(4, 0), x(2, 0));
          },
          std::array<double, 5>{stateStdDevs(0, 0), stateStdDevs(1, 0),
                                stateStdDevs(2, 0), stateStdDevs(3, 0),
                                stateStdDevs(4, 0)},
          StdDevMatrixToArray(localMeasurementStdDevs), nominalDt),
      m_nominalDt(nominalDt) {
  Eigen::Matrix<double, 3, 3> visionContR =
      frc::MakeCovMatrix(StdDevMatrixToArray(visionMeasurmentStdDevs));
  m_visionDiscR = frc::DiscretizeR<3>(visionContR, m_nominalDt);
  m_visionCorrect = [&](const Vector<3>& u, const Vector<3>& y) {
    return m_observer.Correct<3>(
        u, y,
        [](const Vector<5>& x, const Vector<3>&) {
          return x.block<3, 1>(0, 0);
        },
        m_visionDiscR);
  };

  m_gyroOffset = initialPose.Rotation() - gyroAngle;
  m_previousAngle = initialPose.Rotation();
  m_observer.SetXhat(frc::MakeMatrix<5, 1>(
      initialPose.Translation().X().to<double>(),
      initialPose.Translation().Y().to<double>(),
      initialPose.Rotation().Radians().to<double>(), 0.0, 0.0));
}

void DifferentialDrivePoseEstimator::ResetPosition(
    const Pose2d& pose, const Rotation2d& gyroAngle) {
  m_previousAngle = pose.Rotation();
  m_gyroOffset = GetEstimatedPosition().Rotation() - gyroAngle;
  m_observer.SetXhat(frc::MakeMatrix<5, 1>(
      pose.Translation().X().to<double>(), pose.Translation().Y().to<double>(),
      pose.Rotation().Radians().to<double>(), 0.0, 0.0));
}

Pose2d DifferentialDrivePoseEstimator::GetEstimatedPosition() const {
  return Pose2d(units::meter_t(m_observer.Xhat(0)),
                units::meter_t(m_observer.Xhat(1)),
                Rotation2d(units::radian_t(m_observer.Xhat(2))));
}

void DifferentialDrivePoseEstimator::AddVisionMeasurement(
    const Pose2d& visionRobotPose, units::second_t timestamp) {
  m_latencyCompensator.ApplyPastMeasurement(&m_observer, m_nominalDt,
                                            PoseToVector(visionRobotPose),
                                            m_visionCorrect, timestamp);
}

Pose2d DifferentialDrivePoseEstimator::Update(
    const Rotation2d& gyroAngle,
    const DifferentialDriveWheelSpeeds& wheelSpeeds,
    units::meter_t leftDistance, units::meter_t rightDistance) {
  return UpdateWithTime(frc2::Timer::GetFPGATimestamp(), gyroAngle, wheelSpeeds,
                        leftDistance, rightDistance);
}

Pose2d DifferentialDrivePoseEstimator::UpdateWithTime(
    units::second_t currentTime, const Rotation2d& gyroAngle,
    const DifferentialDriveWheelSpeeds& wheelSpeeds,
    units::meter_t leftDistance, units::meter_t rightDistance) {
  auto dt = m_prevTime >= 0_s ? currentTime - m_prevTime : m_nominalDt;
  m_prevTime = currentTime;

  auto angle = gyroAngle + m_gyroOffset;
  auto omega = (gyroAngle - m_previousAngle).Radians() / dt;

  auto u = frc::MakeMatrix<3, 1>(
      (wheelSpeeds.left + wheelSpeeds.right).to<double>() / 2.0, 0.0,
      (angle - m_previousAngle).Radians().to<double>() / dt.to<double>());

  m_previousAngle = angle;

  auto localY = frc::MakeMatrix<3, 1>(leftDistance.to<double>(),
                                      rightDistance.to<double>(),
                                      angle.Radians().to<double>());

  m_latencyCompensator.AddObserverState(m_observer, u, localY, currentTime);
  m_observer.Predict(u, dt);
  m_observer.Correct(u, localY);

  return GetEstimatedPosition();
}

Vector<5> DifferentialDrivePoseEstimator::F(const Vector<5>& x,
                                            const Vector<3>& u) {
  // Apply a rotation matrix. Note that we do not add x because Runge-Kutta does
  // that for us.
  auto& theta = x(2, 0);
  Eigen::Matrix<double, 5, 5> toFieldRotation = frc::MakeMatrix<5, 5>(
      // clang-format off
    std::cos(theta), -std::sin(theta), 0.0, 0.0, 0.0,
    std::sin(theta), std::cos(theta), 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0);  // clang-format on
  return toFieldRotation *
         frc::MakeMatrix<5, 1>(u(0, 0), u(1, 0), u(2, 0), u(0, 0), u(1, 0));
}

std::array<double, 3> DifferentialDrivePoseEstimator::StdDevMatrixToArray(
    const Vector<3>& stdDevs) {
  return std::array<double, 3>{stdDevs(0, 0), stdDevs(1, 0), stdDevs(2, 0)};
}
