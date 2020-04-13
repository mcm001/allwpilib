/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <array>

#include <Eigen/Core>
#include <units/units.h>

#include "frc/geometry/Pose2d.h"
#include "frc/system/LinearSystem.h"
#include "frc/estimator/ExtendedKalmanFilter.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "frc/kinematics/DifferentialDriveKinematics.h"
#include "frc/system/NumericalJacobian.h"
#include "frc/trajectory/Trajectory.h"

namespace frc {

class LTVDiffDriveController {
 public:
  LTVDiffDriveController(const LinearSystem<2, 2, 2>& plant,
                        const std::array<double, 5>& controllerQ,
                        const std::array<double, 2>& controllerR,
                        const DifferentialDriveKinematics& kinematics,
                        units::second_t dt);

  static Eigen::Matrix<double, 2, 1> GetController(const Eigen::Matrix<double, 10, 1>& x,
    const Eigen::Matrix<double, 5, 1>& r);

  static Eigen::Matrix<double, 10, 1> GetDynamics(const Eigen::Matrix<double, 10, 1>& x,
    const Eigen::Matrix<double, 2, 1>& u);

  /**
   * Returns the current controller reference in the form
   * [X, Y, Heading, LeftVelocity, RightVelocity, LeftPosition,
   * RightPosition, LeftVoltageError, RightVoltageError,
   * AngularVelocityError].
   *
   * @return Matrix[10, 1] The reference.
   */
  const Eigen::Matrix<double, 10, 1>& GetReferences() const;

  /**
   * Returns the states of the controller from the EKF in the form
   * [X, Y, Heading, LeftVelocity, RightVelocity, LeftPosition,
   * RightPosition, LeftVoltageError, RightVoltageError,
   * AngularVelocityError].
   *
   * @return Matrix[10, 1] containing the states.
   */
  const Eigen::Matrix<double, 10, 1>& GetStates() const;

  /**
   * Returns the inputs of the controller in the form [LeftVoltage, RightVoltage].
   *
   * @return Matrix[2, 1] The inputs.
   */
  const Eigen::Matrix<double, 2, 1>& GetInputs() const;

  /**
   * Returns the set measured outputs of the controller in the from
   * [Heading, LeftPosition, RightPosition].
   *
   * @return Matrix[3, 1] The outputs.
   */
  const Eigen::Matrix<double, 3, 1>& GetLocalOutputs() const;

  /**
   * Returns the set measured outputs of the controller in the from
   * [X, Y, Heading, LeftPosition, RightPosition, AngularVelocity].
   *
   * @return Matrix[6, 1] The outputs.
   */
  const Eigen::Matrix<double, 6, 1>& GetGlobalOutputs() const;

  /**
   * Returns the estimated outputs based on the current state estimate.
   * <p>
   * Note: This provides only local measurements.
   * </p>
   *
   * @return Matrix[3, 1] The estimated local outputs.
   */
  const Eigen::Matrix<double, 3, 1>& GetEstimatedLocalOutputs() const;

  /**
   * Returns the estimated outputs based on the current state estimate.
   * <p>
   * Note: This provides global measurements (including pose).
   * </p>
   *
   * @return Matrix[6, 1] The estimated global outputs.
   */
  const Eigen::Matrix<double, 6, 1>& GetEstimatedGlobalOutputs() const;

  /**
   * Returns the input as a {@link DifferentialDriveMotorVoltages} of the 
   * controller after updating it.
   * <p>
   * The reference pose, linear velocity, and angular velocity should come from a
   * {@link Trajectory}.
   * </p>
   * 
   * <p>
   * Note: This MUST be called every loop at the dt specified in the constructor
   * for the controller to update properly.
   * </p>
   *
   * @param poseRef                       The desired pose of the robot.
   * @param linearVelocityRef The desired linear velocity of the
   *                                      robot.
   * @param angularVelocityRef   The desired angular velocity of the
   *                                      robot.
   * @return The control input as a {@link DifferentialDriveMotorVoltages}.
   */
  DifferentialDriveMotorVoltages Calculate(const Pose2d& poseRef,
                          units::meters_per_second_t linearVelocityRef,
                          units::radians_per_second_t angularVelocityRef);

  /**
   * Returns the input as a {@link DifferentialDriveMotorVoltages} of the 
   * controller after updating it.
   * <p>
   * The desired state should come from a {@link Trajectory}.
   * </p>
   * 
   * <p>
   * Note: This MUST be called every loop at the dt specified in the constructor
   * for the controller to update properly.
   * </p>
   * 
   * @param currentPose  The current pose.
   * @param desiredState The desired pose, linear velocity, and angular velocity
   *                     from a trajectory.
   * 
   * @return The control input as a {@link DifferentialDriveMotorVoltages}.
   */
  DifferentialDriveMotorVoltages Calculate(const Pose2d& currentPose,
                          const Trajectory::State& desiredState);

  /**
   * Resets the internal state of the controller.
   */
  void Reset();

  /**
   * Resets the internal state of the controller with a specified pose for
   * the initial state estimate.
   *
   * @param initialPose Initial pose for state estimate.
   */
  void Reset(const Pose2d& initialPose);

  /**
   * Set the tolerance for if the robot is {@link #atReference()} or not.
   *
   * @param poseTolerance The new pose tolerance.
   * @param velocityTolerance The velocity tolerance.
   */
  void SetTolerance(const Pose2d& poseTolerance, units::meters_per_second_t velocityTolerance);

  /**
   * Returns if the controller is at the reference pose on the trajectory.
   * Note that this is different than if the robot has traversed the entire
   * trajectory. The tolerance is set by the {@link #setTolerance(Pose2d, double)}
   * method.
   *
   * @return If the robot is within the specified tolerance of the
   */
  bool AtReference() const;

 private:
  units::meter_t m_rb;
  LinearSystem<2, 2, 2> m_plant;
  units::second_t m_dtSeconds;

  Eigen::Matrix<double, 2, 1> m_appliedU;
  Eigen::Matrix<double, 3, 1> m_localY;
  Eigen::Matrix<double, 6, 1> m_globalY;

  Eigen::Matrix<double, 10, 1> m_r;
  Eigen::Matrix<double, 10, 1> m_nextR;
  Eigen::Matrix<double, 2, 1> m_cappedU;

  Eigen::Matrix<double, 5, 2> m_B;
  Eigen::Matrix<double, 2, 5> m_K0;
  Eigen::Matrix<double, 2, 5> m_K1;

  Eigen::Matrix<double, 6, 6> m_globalR;

  bool m_useLocalMeasurements;

  Pose2d m_poseTolerance;
  units::meters_per_second_t m_velocityTolerance;

  ExtendedKalmanFilter<10, 2, 3> m_observer;
  DifferentialDriveKinematics m_kinematics;

  enum State {
    kX = 0,
    kY = 1,
    kHeading = 2,
    kLeftVelocity = 3,
    kRightVelocity = 4,
    kLeftPosition = 5,
    kRightPosition = 6,
    kLeftVoltageError = 7,
    kRightVoltageError = 8,
    kAngularVelocityError = 8,
  };

  enum Input {
    kLeftVoltage = 0,
    kRightVoltage = 1
  };

  enum LocalOutput {
    kHeading = 0,
    kLeftPosition = 1,
    kRightPosition = 2
  };

  enum GlobalOutput {
    kX = 0,
    kY = 1,
    kHeading = 2,
    kLeftPosition = 3,
    kRightPosition = 4,
    kAngularVelocity = 5
  };

  static void sclaeCappedU(Eigen::Matrix<double, 2, 1>* u);

  static units::radian_t NormalizeAngle(units::radian_t angle);
};

}  // namespace frc
