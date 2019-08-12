/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.kinematics;


import edu.wpi.first.wpilibj.geometry.Rotation2d;

/**
 * Represents the speed of a robot chassis. Although this struct contains
 * similar members compared to a Twist2d, they do NOT represent the same thing.
 * Whereas a Twist2d represents a change in pose w.r.t to the robot frame of reference,
 * this ChassisSpeeds struct represents a velocity w.r.t to the robot frame of
 * reference.
 *
 * <p>A strictly non-holonomic drivetrain, such as a differential drive, should
 * never have a dy component because it can never move sideways. Holonomic
 * drivetrains such as swerve and mecanum will often have all three components.
 */
@SuppressWarnings("MemberName")
public class ChassisSpeeds {
  /**
   * Represents forward velocity w.r.t the robot frame of reference. (Fwd is +)
   */
  public double vx;

  /**
   * Represents sideways velocity w.r.t the robot frame of reference. (Left is +)
   */
  public double vy;

  /**
   * Represents the angular velocity of the robot frame. (CCW is +)
   */
  public double omega;

  /**
   * Constructs a ChassisSpeeds with zeros for dx, dy, and theta.
   */
  public ChassisSpeeds() {
  }

  /**
   * Constructs a ChassisSpeeds object.
   *
   * @param vx    Forward velocity.
   * @param vy    Sideways velocity.
   * @param omega Angular velocity.
   */
  public ChassisSpeeds(double vx, double vy, double omega) {
    this.vx = vx;
    this.vy = vy;
    this.omega = omega;
  }

  /**
   * Converts a user provided field-relative set of speeds into a robot-relative
   * ChassisSpeeds object.
   *
   * @param vx         The component of speed in the x direction relative to the field.
   *                   Positive x is away from your alliance wall.
   * @param vy         The component of speed in the y direction relative to the field.
   *                   Positive y is to your left when standing behind the alliance wall.
   * @param omega      The angular rate of the robot.
   * @param robotAngle The angle of the robot as measured by a gyroscope. The robot's angle
   *                   is considered to be zero when it is facing directly away from your
   *                   alliance station wall. Remember that this should be CCW positive.
   * @return ChassisSpeeds object representing the speeds in the robot's frame of reference.
   */
  public static ChassisSpeeds fromFieldRelativeSpeeds(
      double vx, double vy, double omega, Rotation2d robotAngle) {
    return new ChassisSpeeds(
        vx * robotAngle.getCos() + vy * robotAngle.getSin(),
        -vx * robotAngle.getSin() + vy * robotAngle.getCos(),
        omega
    );
  }
}
