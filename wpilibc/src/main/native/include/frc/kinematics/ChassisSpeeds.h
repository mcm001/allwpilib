/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "frc/geometry/Rotation2d.h"

namespace frc {
/**
 * Represents the speed of a robot chassis. Although this struct contains the
 * similar members compared to a Twist2d, they do NOT represent the same thing.
 * Whereas a Twist2d represents a change in pose w.r.t to the robot frame of
 * reference, this ChassisSpeeds struct represents a velocity w.r.t to the robot
 * frame of reference.
 *
 * A strictly non-holonomic drivetrain, such as a differential drive, should
 * never have a dy component because it can never move sideways. Holonomic
 * drivetrains such as swerve and mecanum will often have all three components.
 */
struct ChassisSpeeds {
  /**
   * Represents forward velocity w.r.t the robot frame of reference. (Fwd is +)
   */
  double vx = 0;

  /**
   * Represents strafe velocity w.r.t the robot frame of reference. (Left is +)
   */
  double vy = 0;

  /**
   * Represents the angular velocity of the robot frame. (CCW is +)
   */
  double omega = 0;

  /**
   * Converts a user provided field-relative set of speeds into a robot-relative
   * ChassisSpeeds object.
   *
   * @param vx The component of speed in the x direction relative to the field.
   * Positive x is away from your alliance wall.
   * @param vy The component of speed in the y direction relative to the field.
   * Positive y is to your left when standing behind the alliance wall.
   * @param omega The angular rate of the robot.
   * @param robotAngle The angle of the robot as measured by a gyroscope.
   * Remember that this should be CCW positive.
   *
   * @return ChassisSpeeds object representing the speeds in the robot's frame
   * of reference.
   */
  static ChassisSpeeds FromFieldRelativeSpeeds(double vx, double vy,
                                               double omega,
                                               const Rotation2d& robotAngle) {
    return {vx * robotAngle.Cos() + vy * robotAngle.Sin(),
            -vx * robotAngle.Sin() + vy * robotAngle.Cos(), omega};
  }
};
}  // namespace frc
