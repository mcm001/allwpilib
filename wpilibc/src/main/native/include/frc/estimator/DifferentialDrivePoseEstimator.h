#pragma once

#include <frc/estimator/ExtendedKalmanFilter.h>
#include <frc/estimator/KalmanFilterLatencyCompensator.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Pose2d.h>
#include <Eigen/Core>
#include <units/units.h>

namespace frc {

class DifferentialDrivePoseEstimator {
 public:
  /**
   * Constructs a DifferentialDrivePose estimator.
   *
   * @param gyroAngle          The current gyro angle.
   * @param initialPose        The starting pose estimate.
   * @param stateStdDevs       Standard deviations of model states. Increase
   *                           these numbers to trust your encoders less.
   * @param measurementStdDevs Standard deviations of the measurements. Increase
   *                           these numbers to trust vision less.
   * @param nominalDtSeconds   The time in seconds between each robot loop.
   */
  DifferentialDrivePoseEstimator(const Rotation2d& gyroAngle,
                                 const Pose2d& initialPose,
                                 Eigen::Matrix<double, 3, 1> stateStdDevs,
                                 Eigen::Matrix<double, 3, 1> measurementStdDevs,
                                 units::second_t nominalDt = 0.02_s);

  void ResetPosition(const Pose2d& pose, const Rotation2d& gyroAngle);

  Pose2d GetEstimatedPosition() const;

  void AddVisionMeasurement(const Pose2d& visionRobotPose,
                            units::second_t timestamp);

  Pose2d Update(const Rotation2d& gyroAngle, units::meter_t leftDistance,
                units::meter_t rightDistance);

  Pose2d UpdateWithTime(units::second_t currentTime,
                        const Rotation2d& gyroAngle,
                        units::meter_t leftDistance,
                        units::meter_t rightDistance);

 private:
  ExtendedKalmanFilter<3, 3, 3> m_observer;
  KalmanFilterLatencyCompensator<3, 3, 3> m_latencyCompensator;

  units::second_t m_nominalDt;
  units::second_t m_prevTime = -1_s;

  Rotation2d m_gyroOffset;
  Rotation2d m_previousAngle;

  static Eigen::Matrix<double, 3, 1> PoseToVector(const Pose2d& pose);
  static Eigen::Matrix<double, 3, 1> F(Eigen::Matrix<double, 3, 1> x, Eigen::Matrix<double, 3, 1> u);
};

}  // namespace frc
