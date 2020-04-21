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

#include "frc/estimator/ExtendedKalmanFilter.h"
#include "frc/geometry/Pose2d.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "frc/kinematics/DifferentialDriveKinematics.h"
#include "frc/system/LinearSystem.h"
#include "frc/system/NumericalJacobian.h"
#include "frc/trajectory/Trajectory.h"

namespace frc {

template <int N>
using Vector = Eigen::Matrix<double, N, 1>;

class LTVDiffDriveController {
 public:
  LTVDiffDriveController(const LinearSystem<2, 2, 2>& plant,
                         const std::array<double, 5>& controllerQ,
                         const std::array<double, 2>& controllerR,
                         const DifferentialDriveKinematics& kinematics,
                         units::second_t dt);

  /**
   * Returns if the controller is at the reference pose on the trajectory.
   * Note that this is different than if the robot has traversed the entire
   * trajectory. The tolerance is set by the {@link #setTolerance(Pose2d,
   * double)} method.
   *
   * @return If the robot is within the specified tolerances.
   */
  bool AtReference() const;

  /**
   * Set the tolerance for if the robot is {@link #atReference()} or not.
   *
   * @param poseTolerance The new pose tolerance.
   * @param velocityTolerance The velocity tolerance.
   */
  void SetTolerance(const Pose2d& poseTolerance,
                    units::meters_per_second_t velocityTolerance);

  /**
   * Returns the current controller reference in the form
   * [X, Y, Heading, LeftVelocity, RightVelocity, LeftPosition].
   *
   * @return Matrix [5, 1] The reference.
   */
  const Vector<5>& GetReferences() const;

  /**
   * Returns the inputs of the controller in the form [LeftVoltage,
   * RightVoltage].
   *
   * @return Matrix[2, 1] The inputs.
   */
  const Vector<2>& GetInputs() const;

  /**
   * Returns the uncapped control input after updating the controller with the
   * given reference and current states.
   *
   * @param currentState  The current state of the robot as a vector.
   * @param stateRef      The reference state vector.
   * @return The control input as a  pair of motor voltages [left, right].
   */
  const Vector<2>& Calculate(const Vector<5>& currentState,
                             const Vector<5>& stateRef);

  /**
   * Returns the next output of the controller.
   *
   * <p>The desired state should come from a {@link Trajectory}.
   *
   * @param currentState  The current state of the robot as a vector.
   * @param desiredState  The desired pose, linear velocity, and angular
   * velocity from a trajectory.
   * @return The control input as a  pair of motor voltages [left, right].
   */
  const Vector<2>& Calculate(const Vector<5>& currentState,
                             const Trajectory::State& desiredState);

  /**
   * Resets the internal state of the controller.
   */
  void Reset();

  Vector<2> Controller(const Vector<5>& x, const Vector<5>& r);

  Vector<10> Dynamics(const Vector<10>& x, const Vector<2>& u);

 private:
  LinearSystem<2, 2, 2> m_plant;
  units::meter_t m_rb;

  Vector<5> m_nextR;
  Vector<2> m_uncappedU;

  Eigen::Matrix<double, 5, 2> m_B;
  Eigen::Matrix<double, 2, 5> m_K0;
  Eigen::Matrix<double, 2, 5> m_K1;

  Vector<5> m_stateError;

  Pose2d m_poseTolerance;
  units::meters_per_second_t m_velocityTolerance;

  DifferentialDriveKinematics m_kinematics;

  class State {
   public:
    static constexpr int kX = 0;
    static constexpr int kY = 1;
    static constexpr int kHeading = 2;
    static constexpr int kLeftVelocity = 3;
    static constexpr int kRightVelocity = 4;
    static constexpr int kLeftPosition = 5;
    static constexpr int kRightPosition = 6;
    static constexpr int kLeftVoltageError = 7;
    static constexpr int kRightVoltageError = 8;
    static constexpr int kAngularVelocityError = 9;
  };

  class Input {
   public:
    static constexpr int kLeftVoltage = 0;
    static constexpr int kRightVoltage = 1;
  };

  units::radian_t NormalizeAngle(units::radian_t angle);
};

}  // namespace frc
