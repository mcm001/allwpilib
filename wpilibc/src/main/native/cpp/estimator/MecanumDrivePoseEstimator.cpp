/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "frc/estimator/MecanumDrivePoseEstimator.h"

#include <limits>

#include "frc/StateSpaceUtil.h"

using namespace frc;

frc::MecanumDrivePoseEstimator::MecanumDrivePoseEstimator(
    const Rotation2d& gyroAngle, const Pose2d& initialPose,
    MecanumDriveKinematics kinematics,
    const std::array<double, 3>& stateStdDevs,
    const std::array<double, 1>& localMeasurementStdDevs,
    const std::array<double, 3>& visionMeasurementStdDevs,
    units::second_t nominalDt)
    : m_stateStdDevs(stateStdDevs),
      m_localMeasurementStdDevs(localMeasurementStdDevs),
      m_visionMeasurementStdDevs(visionMeasurementStdDevs),
      m_observer(
          &MecanumDrivePoseEstimator::F,
          [](const Eigen::Matrix<double, 4, 1>& x,
             const Eigen::Matrix<double, 3, 1>& u) {
            return x.block<2, 1>(2, 0);
          },
          // X and Y here are zero because we only use the cosine and sin
          // states of X in the diagonal.
          MakeQDiagonals(
              stateStdDevs,
              frc::MakeMatrix<4, 1>(0.0, 0.0, initialPose.Rotation().Cos(),
                                    initialPose.Rotation().Sin())),
          MakeRDiagonals(
              localMeasurementStdDevs,
              frc::MakeMatrix<4, 1>(0.0, 0.0, initialPose.Rotation().Cos(),
                                    initialPose.Rotation().Sin())),
          nominalDt),
      m_kinematics(kinematics),
      m_nominalDt(nominalDt) {
  // Create correction mechanism for vision measurements.
  m_visionCorrect = [&](const Eigen::Matrix<double, 3, 1>& u,
                        const Eigen::Matrix<double, 4, 1>& y) {
    m_observer.Correct<4>(
        u, y,
        [](const Eigen::Matrix<double, 4, 1>& x,
           const Eigen::Matrix<double, 3, 1>& u) { return x; },
        DiscretizeR<4>(
            MakeCovMatrix<4>(MakeVisionRDiagonals(visionMeasurementStdDevs, y)),
            nominalDt));
  };

  // Set initial state.
  ResetPosition(initialPose, gyroAngle);

  // Calculate offsets.
  m_gyroOffset = initialPose.Rotation() - gyroAngle;
  m_previousAngle = initialPose.Rotation();
}

void frc::MecanumDrivePoseEstimator::ResetPosition(
    const Pose2d& pose, const Rotation2d& gyroAngle) {
  // Set observer state.
  m_observer.SetXhat(PoseTo4dVector(pose));

  // Calculate offsets.
  m_gyroOffset = pose.Rotation() - gyroAngle;
  m_previousAngle = pose.Rotation();
}

Pose2d frc::MecanumDrivePoseEstimator::GetEstimatedPosition() const {
  return Pose2d(m_observer.Xhat(0) * 1_m, m_observer.Xhat(1) * 1_m,
                Rotation2d(m_observer.Xhat(2), m_observer.Xhat(3)));
}

void frc::MecanumDrivePoseEstimator::AddVisionMeasurement(
    const Pose2d& visionRobotPose, units::second_t timestamp) {
  m_latencyCompensator.ApplyPastMeasurement<4>(&m_observer, m_nominalDt,
                                               PoseTo4dVector(visionRobotPose),
                                               m_visionCorrect, timestamp);
}

Pose2d frc::MecanumDrivePoseEstimator::Update(
    const Rotation2d& gyroAngle, const MecanumDriveWheelSpeeds& wheelSpeeds) {
  return UpdateWithTime(frc2::Timer::GetFPGATimestamp(), gyroAngle,
                        wheelSpeeds);
}

Pose2d frc::MecanumDrivePoseEstimator::UpdateWithTime(
    units::second_t currentTime, const Rotation2d& gyroAngle,
    const MecanumDriveWheelSpeeds& wheelSpeeds) {
  auto dt = m_prevTime >= 0_s ? currentTime - m_prevTime : m_nominalDt;
  m_prevTime = currentTime;

  auto angle = gyroAngle + m_gyroOffset;
  auto omega = (angle - m_previousAngle).Radians() / dt;

  auto chassisSpeeds = m_kinematics.ToChassisSpeeds(wheelSpeeds);
  auto fieldRelativeVelocities =
      Translation2d(chassisSpeeds.vx * 1_s, chassisSpeeds.vy * 1_s)
          .RotateBy(angle);

  auto u = frc::MakeMatrix<3, 1>(fieldRelativeVelocities.X().to<double>(),
                                 fieldRelativeVelocities.Y().to<double>(),
                                 omega.to<double>());

  auto localY = frc::MakeMatrix<2, 1>(angle.Cos(), angle.Sin());
  m_previousAngle = angle;

  m_latencyCompensator.AddObserverState(m_observer, u, localY, currentTime);

  m_observer.Predict(
      u,
      frc::MakeCovMatrix<4>(MakeQDiagonals(m_stateStdDevs, m_observer.Xhat())),
      dt);
  m_observer.Correct<2>(
      u, localY,
      [](const Eigen::Matrix<double, 4, 1>& x,
         const Eigen::Matrix<double, 3, 1>& u) { return x.block<2, 1>(2, 0); },
      frc::MakeCovMatrix<2>(
          MakeRDiagonals(m_localMeasurementStdDevs, m_observer.Xhat())));

  return GetEstimatedPosition();
}

Eigen::Matrix<double, 4, 1> frc::MecanumDrivePoseEstimator::F(
    const Eigen::Matrix<double, 4, 1>& x,
    const Eigen::Matrix<double, 3, 1>& u) {
  return frc::MakeMatrix<4, 1>(u(0), u(1), -x(3) * u(2), x(2) * u(2));
}
