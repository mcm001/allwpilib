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
    const Eigen::Matrix<double, 5, 1>& stateStdDevs,
    const Eigen::Matrix<double, 3, 1>& localMeasurementStdDevs,
    const Eigen::Matrix<double, 3, 1>& visionMeasurementStdDevs,
    units::second_t nominalDt)
    : m_stateStdDevs(stateStdDevs),
      m_localMeasurementStdDevs(localMeasurementStdDevs),
      m_observer(
          &DifferentialDrivePoseEstimator::F,
          &DifferentialDrivePoseEstimator::LocalMeasurementModel,
          MakeQDiagonals(stateStdDevs, FillStateVector(initialPose, 0_m, 0_m)),
          MakeRDiagonals(localMeasurementStdDevs,
                         FillStateVector(initialPose, 0_m, 0_m)),
          nominalDt),
      m_nominalDt(nominalDt) {
  // Create R (covariances) for vision measurements.
  Eigen::Matrix<double, 3, 3> visionContR =
      frc::MakeCovMatrix(StdDevMatrixToArray<3>(visionMeasurementStdDevs));

  // Create correction mechanism for vision measurements.
  m_visionCorrect = [&](const Eigen::Matrix<double, 3, 1>& u,
                        const Eigen::Matrix<double, 4, 1>& y) {
    m_observer.Correct<4>(
        u, y,
        [](const Eigen::Matrix<double, 6, 1>& x_,
           const Eigen::Matrix<double, 3, 1>& u_) {
          return x_.block<4, 1>(0, 0);
        },
        DiscretizeR<4>(
            MakeCovMatrix<4>(DifferentialDrivePoseEstimator::MakeRDiagonals(
                visionMeasurementStdDevs, m_observer.Xhat())),
            m_nominalDt));
  };

  m_gyroOffset = initialPose.Rotation() - gyroAngle;
  m_previousAngle = initialPose.Rotation();
  m_observer.SetXhat(FillStateVector(initialPose, 0_m, 0_m));
}

void DifferentialDrivePoseEstimator::ResetPosition(
    const Pose2d& pose, const Rotation2d& gyroAngle) {
  m_previousAngle = pose.Rotation();
  m_gyroOffset = GetEstimatedPosition().Rotation() - gyroAngle;
  m_observer.SetXhat(FillStateVector(pose, 0_m, 0_m));
}

Pose2d DifferentialDrivePoseEstimator::GetEstimatedPosition() const {
  return Pose2d(units::meter_t(m_observer.Xhat(0)),
                units::meter_t(m_observer.Xhat(1)),
                Rotation2d(units::radian_t(m_observer.Xhat(2))));
}

void DifferentialDrivePoseEstimator::AddVisionMeasurement(
    const Pose2d& visionRobotPose, units::second_t timestamp) {
  m_latencyCompensator.ApplyPastMeasurement<4>(&m_observer, m_nominalDt,
                                               PoseTo4dVector(visionRobotPose),
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

  Eigen::Matrix<double, 3, 1> u = frc::MakeMatrix<3, 1>(
      (wheelSpeeds.left + wheelSpeeds.right).to<double>() / 2.0, 0.0,
      omega.to<double>());

  m_previousAngle = angle;

  Eigen::Matrix<double, 4, 1> localY = frc::MakeMatrix<4, 1>(
      leftDistance.to<double>(), rightDistance.to<double>(), angle.Cos(),
      angle.Sin());

  m_latencyCompensator.AddObserverState(m_observer, u, localY, currentTime);
  m_observer.Predict(
      u,
      frc::MakeCovMatrix<6>(MakeQDiagonals(m_stateStdDevs, m_observer.Xhat())),
      dt);
  m_observer.Correct<4>(u, localY, &LocalMeasurementModel,
                        frc::MakeCovMatrix<4>(MakeRDiagonals(
                            m_localMeasurementStdDevs, m_observer.Xhat())));

  return GetEstimatedPosition();
}

Eigen::Matrix<double, 6, 1> DifferentialDrivePoseEstimator::F(
    const Eigen::Matrix<double, 6, 1>& x,
    const Eigen::Matrix<double, 3, 1>& u) {
  // Apply a rotation matrix. Note that we do not add x because Runge-Kutta does
  // that for us.
  double cosTheta = x(2);
  double sinTheta = x(3);

  // We want vx and vy to be in the field frame, so we apply a rotation matrix.
  // to u_Field = u = [[vx_field, vy_field, omega]]^T
  Eigen::Matrix<double, 2, 1> chassisVel_local =
      MakeMatrix<2, 1>((u(0, 0) + u(1, 0)) / 2.0, 0.0);
  Eigen::Matrix<double, 2, 2> toFieldRotation =
      MakeMatrix<2, 2>(cosTheta, -sinTheta, sinTheta, cosTheta);
  Eigen::Matrix<double, 2, 1> chassisVel_field =
      toFieldRotation * chassisVel_local;

  // dcos(theta)/dt = -std::sin(theta) * dtheta/dt = -std::sin(theta) * omega
  double dcosTheta = -sinTheta * u(2, 0);
  // dsin(theta)/dt = std::cos(theta) * omega
  double dsinTheta = cosTheta * u(2, 0);

  // As x = [[x_field, y_field, std::cos(theta), std::sin(theta), dist_l,
  // dist_r]]^T, we need to return x-dot = [[vx_field, vy_field, d/dt
  // cos(theta), d/dt sin(theta), vel_left, vel_right]]^T Assuming  no wheel
  // slip, vx = (v_left + v_right) / 2, and vy = 0;

  return MakeMatrix<6, 1>(chassisVel_field(0), chassisVel_field(1), dcosTheta,
                          dsinTheta, u(0), u(1));
}

Eigen::Matrix<double, 4, 1>
DifferentialDrivePoseEstimator::LocalMeasurementModel(
    const Eigen::Matrix<double, 6, 1>& x,
    const Eigen::Matrix<double, 3, 1>& u) {
  return frc::MakeMatrix<4, 1>(x(4), x(5), x(2), x(3));
}

template <int Dim>
std::array<double, Dim> DifferentialDrivePoseEstimator::StdDevMatrixToArray(
    const Eigen::Matrix<double, Dim, 1>& stdDevs) {
  std::array<double, Dim> array;
  for (size_t i = 0; i < Dim; ++i) {
    array[i] = stdDevs(i);
  }
  return array;
}

Eigen::Matrix<double, 6, 1> DifferentialDrivePoseEstimator::FillStateVector(
    const Pose2d& pose, units::meter_t leftDistance,
    units::meter_t rightDistance) {
  return frc::MakeMatrix<6, 1>(
      pose.Translation().X().to<double>(), pose.Translation().Y().to<double>(),
      pose.Rotation().Cos(), pose.Rotation().Sin(), leftDistance.to<double>(),
      rightDistance.to<double>());
}

std::array<double, 6> DifferentialDrivePoseEstimator::MakeQDiagonals(
    const Eigen::Matrix<double, 5, 1>& stdDevs,
    const Eigen::Matrix<double, 6, 1>& x) {
  return {stdDevs(0),        stdDevs(1), stdDevs(2) * x(2),
          stdDevs(2) * x(3), stdDevs(3), stdDevs(4)};
}

std::array<double, 4> DifferentialDrivePoseEstimator::MakeRDiagonals(
    const Eigen::Matrix<double, 3, 1>& stdDevs,
    const Eigen::Matrix<double, 6, 1>& x) {
  return {stdDevs(0), stdDevs(1), stdDevs(2) * x(2), stdDevs(2) * x(3)};
}
