/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <array>

#include <Eigen/Core>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/velocity.h>

#include "frc/geometry/Pose2d.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "frc/trajectory/Trajectory.h"

namespace frc {

/**
 * A Linear Time-Varying Cascaded Unicycle Controller for differential drive
 * robots. Similar to RAMSETE, this controller combines feedback and feedforward
 * to output ChassisSpeeds to guide a robot along a trajectory. However, this
 * controller utilizes tolerances grounded in reality to pick gains rather than
 * magical Beta and Zeta gains.
 */
class LTVUnicycleController {
 public:
  /**
   * Construct a LTV Unicycle Controller.
   *
   * @param Qelems    The maximum desired error tolerance for the robot's state,
   *                  in the form [X, Y, Heading]^T. Units are meters and
   *                  radians.
   * @param Relems    The maximum desired control effort by the feedback
   *                  controller, in the form [vMax, wMax]^T. Units are meters
   *                  per second and radians per second. Note that this is
   *                  not the maximum speed of the robot, but rather the maximum
   *                  effort the feedback controller should apply on top of the
   *                  trajectory feedforward.
   * @param dtSeconds The nominal dt of this controller. With command based this
   *                  is 0.020.
   */
  LTVUnicycleController(const std::array<double, 3>& Qelems,
                        const std::array<double, 2>& Relems,
                        units::second_t dt);

  /**
   * Construct a LTV Unicycle Controller.
   *
   * @param Qelems    The maximum desired error tolerance for the robot's state,
   *                  in the form [X, Y, Heading]^T. Units are meters and
   *                  radians.
   * @param rho       A weighting factor that balances control effort and state
   *                  excursion. Greater values penalize state excursion
   *                  more heavily. 1 is a good starting value.
   * @param Relems    The maximum desired control effort by the feedback
   *                  controller, in the form [vMax, wMax]^T. Units are meters
   *                  per second and radians per second. Note that this is not
   *                  the maximum speed of the robot, but rather the maximum
   *                  effort the feedback controller should apply on top of the
   *                  trajectory feedforward.
   * @param dtSeconds The nominal dt of this controller. With command based this
   *                  is 0.020.
   */
  LTVUnicycleController(const std::array<double, 3>& Qelems, const double rho,
                        const std::array<double, 2>& Relems,
                        units::second_t dt);

  /**
   * Returns true if the pose error is within tolerance of the reference.
   */
  bool AtReference() const;

  /**
   * Sets the pose error which is considered tolerable for use with
   * AtReference().
   *
   * @param poseTolerance Pose error which is tolerable.
   */
  void SetTolerance(const Pose2d& poseTolerance);

  /**
   * Returns the next output of the controller.
   *
   * The reference pose, linear velocity, and angular velocity should come
   * from a Trajectory.
   *
   * The current linear velocity of the chassis can be found from a
   * DifferentialDriveWheelSpeeds object using
   * DifferentialDriveWheelSpeeds::ToLinearChassisVelocity.
   *
   * @param currentPose                   The current position of the robot.
   * @param currentLinearVelocity         The current linear velocity of the
   *                                      robot. this can be determined by
   *                                      averaging the measured left and
   *                                      right wheel velocities.
   * @param poseRef                       The desired pose of the robot.
   * @param linearVelocityRefMetersPerSec The desired linear velocity of the
   *                                      robot.
   * @param angularVelocityRefRadPerSec   The desired angular velocity of the
   *                                      robot.
   * @return The next calculated output.
   */
  ChassisSpeeds Calculate(const Pose2d& currentPose,
                          units::meters_per_second_t currentLinearVelocity,
                          const Pose2d& poseRef,
                          units::meters_per_second_t linearVelocityRef,
                          units::radians_per_second_t angularVelocityRef);

  /**
   * Returns the next output of the LTV Unicycle Controller.
   *
   * The reference pose, linear velocity, and angular velocity should come from
   * a drivetrain trajectory.
   *
   * The current linear velocity of the chassis can be found from a
   * DifferentialDriveWheelSpeeds object using
   * DifferentialDriveWheelSpeeds::ToLinearChassisVelocity.
   *
   * @param currentPose           The current pose.
   * @param currentLinearVelocity The current linear velocity of the robot.
   *                              this can be determined by averaging the
   *                              measured left and right wheel velocities.
   * @param desiredState          The desired pose, linear velocity, and angular
   *                              velocity from a trajectory.
   */
  ChassisSpeeds Calculate(const Pose2d& currentPose,
                          units::meters_per_second_t currentLinearVelocity,
                          const Trajectory::State& desiredState);

 private:
  Eigen::Matrix<double, 3, 2> m_B;

  std::array<double, 3> m_Qelms;
  double m_rho;
  std::array<double, 2> m_Relems;

  units::second_t m_dt;

  Pose2d m_poseError;
  Pose2d m_poseTolerance;
};

}  // namespace frc
