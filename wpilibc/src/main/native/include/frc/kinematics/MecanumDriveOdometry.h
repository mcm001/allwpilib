/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <mutex>

#include <units/units.h>
#include <wpi/mutex.h>

#include "frc/geometry/Pose2d.h"
#include "frc/kinematics/MecanumDriveKinematics.h"
#include "frc/kinematics/MecanumDriveWheelSpeeds.h"

namespace frc {

/**
 * Class for mecanum drive odometry. Odometry allows you to track the robot's
 * position on the field over a course of a match using readings from your
 * mecanum wheel encoders.
 *
 * Teams can use odometry during the autonomous period for complex tasks like
 * path following. Furthermore, odometry can be used for latency compensation
 * when using computer-vision systems.
 */
class MecanumDriveOdometry {
 public:
  /**
   * Constructs a MecanumDriveOdometry object.
   *
   * @param kinematics The mecanum drive kinematics for your drivetrain.
   * @param initialPose The starting position of the robot on the field.
   */
  explicit MecanumDriveOdometry(MecanumDriveKinematics kinematics,
                                const Pose2d& initialPose = Pose2d());

  /**
   * Resets the robot's position on the field.
   *
   * @param pose The position on the field that your robot is at.
   */
  void ResetPosition(const Pose2d& pose) {
    std::scoped_lock lock(m_mutex);
    m_pose = pose;
  }

  /**
   * Updates the robot's position on the field using forward kinematics and
   * integration of the pose over time. This method takes in the current time as
   * a parameter to calculate period (difference between two timestamps). The
   * period is used to calculate the change in distance from a velocity.
   *
   * @param currentTime The current time.
   * @param wheelSpeeds The current wheel speeds.
   *
   * @return The new pose of the robot.
   */
  const Pose2d& UpdateWithTime(units::second_t currentTime,
                               MecanumDriveWheelSpeeds wheelSpeeds);

  /**
   * Updates the robot's position on the field using forward kinematics and
   * integration of the pose over time. This method takes in the current time as
   * a parameter to calculate period (difference between two timestamps). The
   * period is used to calculate the change in distance from a velocity. This
   * also takes in an angular rate parameter which is used instead of the
   * angular rate that is calculated from forward kinematics.
   *
   * @param currentTime The current time.
   * @param angularVelocity The angular velocity of the robot.
   * @param wheelSpeeds The current wheel speeds.
   *
   * @return The new pose of the robot.
   */
  const Pose2d& UpdateWithTime(units::second_t currentTime,
                               units::radians_per_second_t angularVelocity,
                               MecanumDriveWheelSpeeds wheelSpeeds);

  /**
   * Updates the robot's position on the field using forward kinematics and
   * integration of the pose over time. This method automatically calculates
   * the current time to calculate period (difference between two timestamps).
   * The period is used to calculate the change in distance from a velocity.
   *
   * @param wheelSpeeds The current wheel speeds.
   *
   * @return The new pose of the robot.
   */
  const Pose2d& Update(MecanumDriveWheelSpeeds wheelSpeeds) {
    const auto now = std::chrono::system_clock::now().time_since_epoch();
    units::second_t time{now};
    return UpdateWithTime(time, wheelSpeeds);
  }

  /**
   * Updates the robot's position on the field using forward kinematics and
   * integration of the pose over time. This method automatically calculates
   * the current time to calculate period (difference between two timestamps).
   * The period is used to calculate the change in distance from a velocity.
   * This also takes in an angular rate parameter which is used instead of the
   * angular rate that is calculated from forward kinematics.
   *
   * @param angularVelocity The angular velocity of the robot.
   * @param wheelSpeeds     The current wheel speeds.
   *
   * @return The new pose of the robot.
   */
  const Pose2d& Update(units::radians_per_second_t angularVelocity,
                       MecanumDriveWheelSpeeds wheelSpeeds) {
    const auto now = std::chrono::system_clock::now().time_since_epoch();
    units::second_t time{now};
    return UpdateWithTime(time, angularVelocity, wheelSpeeds);
  }

 private:
  MecanumDriveKinematics m_kinematics;
  Pose2d m_pose;
  wpi::mutex m_mutex;

  units::second_t m_previousTime = -1_s;
};

}  // namespace frc
