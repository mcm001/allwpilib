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

#include "DifferentialDriveKinematics.h"
#include "frc/geometry/Pose2d.h"

namespace frc {
/**
 * Class for differential drive odometry. Odometry allows you to track the
 * robot's position on the field over the course of a match using readings from
 * 2 encoders and a gyroscope.
 *
 * Teams can use odometry during the autonomous period for complex tasks like
 * path following. Furthermore, odometry can be used for latency compensation
 * when using computer-vision systems.
 *
 * Note: It is important to reset both your encoders to zero before you start
 * using this class. Only reset your encoders ONCE. You should not reset your
 * encoders even if you want to reset your robot's pose.
 */
class DifferentialDriveOdometry {
 public:
  /**
   * Constructs a DifferentialDriveOdometry object.
   *
   * @param kinematics The differential drive kinematics for your drivetrain.
   * @param initialPose The starting position of the robot on the field.
   */
  explicit DifferentialDriveOdometry(DifferentialDriveKinematics kinematics,
                                     const Pose2d& initialPose = Pose2d());

  /**
   * Resets the robot's position on the field. Do NOT zero your encoders if you
   * call this function at any other time except initialization.
   *
   * @param pose The position on the field that your robot is at.
   */
  void ResetPosition(const Pose2d& pose) {
    std::scoped_lock lock(m_mutex);
    m_pose = pose;
  }

  /**
   * Updates the robot's position on the field using forward kinematics and
   * integration of the pose over time.
   *
   * @param leftEncoder The value of the left encoder (position). The units for
   * x and y of the returned pose are the same as the units you pass in here.
   * Therefore, it is advised that you convert the raw encoder value into
   * meters, feet, or inches.
   *
   * @param rightEncoder The value of the right encoder (position). The units
   * for x and y of the returned pose are the same as the units you pass in
   * here. Therefore, it is advised that you convert the raw encoder value into
   * meters, feet, or inches.
   *
   * @return The new pose of the robot.
   */
  const Pose2d& Update(units::meter_t leftEncoder, units::meter_t rightEncoder);

  /**
   * Updates the robot's position on the field using forward kinematics and
   * integration of the pose over time. This method uses input from the
   * gyro instead of pure forward kinematics for angular data.
   *
   * @param leftEncoder The value of the left encoder (position). The units for
   * x and y of the returned pose are the same as the units you pass in here.
   * Therefore, it is advised that you convert the raw encoder value into
   * meters, feet, or inches.
   *
   * @param rightEncoder The value of the right encoder (position). The units
   * for x and y of the returned pose are the same as the units you pass in
   * here. Therefore, it is advised that you convert the raw encoder value into
   * meters, feet, or inches.
   *
   * @param gyro The yaw of the robot from a gyroscope.
   *
   * @return The new pose of the robot.
   */
  const Pose2d& Update(units::meter_t leftEncoder, units::meter_t rightEncoder,
                       const Rotation2d& gyro);

 private:
  DifferentialDriveKinematics m_kinematics;
  Pose2d m_pose;
  wpi::mutex m_mutex;

  units::meter_t prevLeftEncoder = 0_m;
  units::meter_t prevRightEncoder = 0_m;
  Rotation2d previousAngle;
};
}  // namespace frc
