/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <array>
#include <iostream>
#include <limits>

#include <Eigen/Core>
#include <units/time.h>

#include "frc/StateSpaceUtil.h"
#include "frc/estimator/KalmanFilterLatencyCompensator.h"
#include "frc/estimator/UnscentedKalmanFilter.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/kinematics/SwerveDriveKinematics.h"
#include "frc2/Timer.h"

namespace frc {

/**
 * This class wraps an UnscentedKalmanFilter to fuse latency-compensated vision
 * measurements with swerve drive encoder velocity measurements. It will correct
 * for noisy measurements and encoder drift. It is intended to be an easy but
 * more accurate drop-in for SwerveDriveOdometry.
 *
 * Update() should be called every robot loop. If your loops are faster or
 * slower than the default of 0.02s, then you should change the nominal delta
 * time by specifying it in the constructor.
 *
 * AddVisionMeasurement() can be called as infrequently as you want; if you
 * never call it, then this class will behave mostly like regular encoder
 * odometry.
 *
 * Our state-space system is:
 *
 * <strong> x = [[x, y, std::cos(theta), std::sin(theta)]]^T </strong> in the
 * field-coordinate system.
 *
 * <strong> u = [[vx, vy, omega]]^T </strong> in the field-coordinate system.
 *
 * <strong> y = [[x, y, std::cos(theta), std::sin(theta)]]^T </strong> in field
 * coords from vision, or <strong> y = [[cos(theta), std::sin(theta)]]^T
 * </strong> from the gyro.
 */
template <size_t NumModules>
class SwerveDrivePoseEstimator {
 public:
  /**
   * Constructs a SwerveDrivePoseEstimator.
   *
   * @param gyroAngle                The current gyro angle.
   * @param initialPoseMeters        The starting pose estimate.
   * @param kinematics               A correctly-configured kinematics object
   *                                 for your drivetrain.
   * @param stateStdDevs             Standard deviations of model states.
   *                                 Increase these numbers to trust your
   *                                 wheel and gyro velocities less.
   * @param localMeasurementStdDevs  Standard deviations of the gyro
   *                                 measurement. Increase this number to
   *                                 trust gyro angle measurements less.
   * @param visionMeasurementStdDevs Standard deviations of the encoder
   *                                 measurements. Increase these numbers to
   *                                 trust vision less.
   * @param nominalDt                The time in seconds between each robot
   *                                 loop.
   */
  SwerveDrivePoseEstimator(
      const Rotation2d& gyroAngle, const Pose2d& initialPose,
      SwerveDriveKinematics<NumModules>& kinematics,
      const std::array<double, 3>& stateStdDevs,
      const std::array<double, 1>& localMeasurementStdDevs,
      const std::array<double, 3>& visionMeasurementStdDevs,
      units::second_t nominalDt = 0.02_s)
      : m_stateStdDevs(stateStdDevs),
        m_localMeasurementStdDevs(localMeasurementStdDevs),
        m_visionMeasurementStdDevs(visionMeasurementStdDevs),
        m_observer(
            &SwerveDrivePoseEstimator::F,
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
          DiscretizeR<4>(MakeCovMatrix<4>(
                             MakeVisionRDiagonals(visionMeasurementStdDevs, y)),
                         nominalDt));
    };

    // Set initial state.
    ResetPosition(initialPose, gyroAngle);

    // Calculate offsets.
    m_gyroOffset = initialPose.Rotation() - gyroAngle;
    m_previousAngle = initialPose.Rotation();
  }

  /**
   * Resets the robot's position on the field.
   *
   * You NEED to reset your encoders (to zero) when calling this method.
   *
   * The gyroscope angle does not need to be reset in the user's robot code.
   * The library automatically takes care of offsetting the gyro angle.
   *
   * @param pose      The position on the field that your robot is at.
   * @param gyroAngle The angle reported by the gyroscope.
   */
  void ResetPosition(const Pose2d& pose, const Rotation2d& gyroAngle) {
    // Set observer state.
    m_observer.SetXhat(PoseTo4dVector(pose));

    // Calculate offsets.
    m_gyroOffset = pose.Rotation() - gyroAngle;
    m_previousAngle = pose.Rotation();
  }

  /**
   * Gets the pose of the robot at the current time as estimated by the
   * Unscented Kalman Filter.
   *
   * @return The estimated robot pose in meters.
   */
  Pose2d GetEstimatedPosition() const {
    return Pose2d(m_observer.Xhat(0) * 1_m, m_observer.Xhat(1) * 1_m,
                  Rotation2d(m_observer.Xhat(2), m_observer.Xhat(3)));
  }

  /**
   * Add a vision measurement to the Unscented Kalman Filter. This will correct
   * the odometry pose estimate while still accounting for measurement noise.
   *
   * This method can be called as infrequently as you want, as long as you are
   * calling Update() every loop.
   *
   * @param visionRobotPose The pose of the robot as measured by the vision
   *                        camera.
   * @param timestamp       The timestamp of the vision measurement in seconds.
   *                        Note that if you don't use your own time source by
   *                        calling UpdateWithTime() then you must use a
   *                        timestamp with an epoch since FPGA startup
   *                        (i.e. the epoch of this timestamp is the same
   *                        epoch as Timer#GetFPGATimestamp.) This means
   *                        that you should use Timer#GetFPGATimestamp as your
   *                        time source or sync the epochs.
   */
  void AddVisionMeasurement(const Pose2d& visionRobotPose,
                            units::second_t timestamp) {
    m_latencyCompensator.ApplyPastMeasurement<4>(
        &m_observer, m_nominalDt, PoseTo4dVector(visionRobotPose),
        m_visionCorrect, timestamp);
  }

  /**
   * Updates the the Unscented Kalman Filter using only wheel encoder
   * information. This should be called every loop, and the correct loop period
   * must be passed into the constructor of this class.
   *
   * @param gyroAngle    The current gyro angle.
   * @param moduleStates The current velocities and rotations of the swerve
   *                     modules.
   * @return The estimated pose of the robot in meters.
   */
  template <typename... ModuleState>
  Pose2d Update(const Rotation2d& gyroAngle, ModuleState&&... moduleStates) {
    return UpdateWithTime(frc2::Timer::GetFPGATimestamp(), gyroAngle,
                          moduleStates...);
  }

  /**
   * Updates the the Unscented Kalman Filter using only wheel encoder
   * information. This should be called every loop, and the correct loop period
   * must be passed into the constructor of this class.
   *
   * @param currentTime  Time at which this method was called, in seconds.
   * @param gyroAngle    The current gyroscope angle.
   * @param moduleStates The current velocities and rotations of the swerve
   *                     modules.
   * @return The estimated pose of the robot in meters.
   */
  template <typename... ModuleState>
  Pose2d UpdateWithTime(units::second_t currentTime,
                        const Rotation2d& gyroAngle,
                        ModuleState&&... moduleStates) {
    auto dt = m_prevTime >= 0_s ? currentTime - m_prevTime : m_nominalDt;
    m_prevTime = currentTime;

    auto angle = gyroAngle + m_gyroOffset;
    auto omega = (angle - m_previousAngle).Radians() / dt;

    auto chassisSpeeds = m_kinematics.ToChassisSpeeds(moduleStates...);
    auto fieldRelativeSpeeds =
        Translation2d(chassisSpeeds.vx * 1_s, chassisSpeeds.vy * 1_s)
            .RotateBy(angle);

    auto u =
        frc::MakeMatrix<3, 1>(fieldRelativeSpeeds.X().template to<double>(),
                              fieldRelativeSpeeds.Y().template to<double>(),
                              omega.template to<double>());

    auto localY = frc::MakeMatrix<2, 1>(angle.Cos(), angle.Sin());
    m_previousAngle = angle;

    m_latencyCompensator.AddObserverState(m_observer, u, localY, currentTime);

    m_observer.Predict(u,
                       frc::MakeCovMatrix<4>(
                           MakeQDiagonals(m_stateStdDevs, m_observer.Xhat())),
                       dt);
    m_observer.Correct<2>(
        u, localY,
        [](const Eigen::Matrix<double, 4, 1>& x,
           const Eigen::Matrix<double, 3, 1>& u) {
          return x.block<2, 1>(2, 0);
        },
        frc::MakeCovMatrix<2>(
            MakeRDiagonals(m_localMeasurementStdDevs, m_observer.Xhat())));

    return GetEstimatedPosition();
  }

 private:
  UnscentedKalmanFilter<4, 3, 2> m_observer;
  SwerveDriveKinematics<NumModules>& m_kinematics;
  KalmanFilterLatencyCompensator<4, 3, 2, UnscentedKalmanFilter<4, 3, 2>>
      m_latencyCompensator;
  std::function<void(const Eigen::Matrix<double, 3, 1>& u,
                     const Eigen::Matrix<double, 4, 1>& y)>
      m_visionCorrect;

  units::second_t m_nominalDt;
  units::second_t m_prevTime = -1_s;

  Rotation2d m_gyroOffset;
  Rotation2d m_previousAngle;

  /**
   * Get x-dot given the current state and input. Recall that the state is [x,
   * y, std::cos(theta), std::sin(theta)]^T In our case, x-dot will be [dx/dt,
   * dy/dt, d/dt cos(theta), d/dt sin(theta)].
   *
   * @param x The current state.
   * @param u The current input. In our case, u = [vx, vy, d/dt theta]^T
   */
  static Eigen::Matrix<double, 4, 1> F(const Eigen::Matrix<double, 4, 1>& x,
                                       const Eigen::Matrix<double, 3, 1>& u) {
    // Need to return [dx/dt, dy/dt, d/dt cos(theta), d/dt sin(theta)]
    // dx/dt and dy/dt are from u.
    // d/dt cos(theta) = -std::sin(theta) * d/dt(theta) by the chain rule.
    return frc::MakeMatrix<4, 1>(u(0), u(1), -x(3) * u(2), x(2) * u(2));
  }

  template <int Dim>
  static std::array<double, Dim> StdDevMatrixToArray(
      const Eigen::Matrix<double, Dim, 1>& vector) {
    std::array<double, Dim> array;
    for (size_t i = 0; i < Dim; ++i) {
      array[i] = vector(i);
    }
    return array;
  }

  std::array<double, 3> m_stateStdDevs;
  std::array<double, 1> m_localMeasurementStdDevs;
  std::array<double, 3> m_visionMeasurementStdDevs;

  static std::array<double, 4> MakeQDiagonals(
      const std::array<double, 3>& stdDevs,
      const Eigen::Matrix<double, 4, 1>& x) {
    // Std dev in [x, y, std::cos(theta), std::sin(theta)] form.
    return {stdDevs[0], stdDevs[1], stdDevs[2] * x(2), stdDevs[2] * x(3)};
  }

  static std::array<double, 2> MakeRDiagonals(
      const std::array<double, 1>& stdDevs,
      const Eigen::Matrix<double, 4, 1>& x) {
    return {stdDevs[0] * x(2), stdDevs[0] * x(3)};
  }

  static std::array<double, 4> MakeVisionRDiagonals(
      const std::array<double, 3>& stdDevs,
      const Eigen::Matrix<double, 4, 1>& y) {
    return {stdDevs[0], stdDevs[1], stdDevs[2] * y(2), stdDevs[2] * y(3)};
  }
};

}  // namespace frc
