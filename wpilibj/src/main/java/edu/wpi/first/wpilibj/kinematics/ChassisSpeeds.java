/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.kinematics;


import edu.wpi.first.wpilibj.geometry.Rotation2d;

/**
 * Represents the speed of a robot chassis. Although this struct contains the
 * same members as a Twist2d, they do NOT represent the same thing. Whereas a
 * Twist2d represents a change in pose w.r.t to the robot frame of reference,
 * this ChassisSpeeds struct represents a velocity w.r.t to the robot frame of
 * reference.
 *
 * <p>A strictly non-holonomic drivetrain, such as a differential drive, should
 * never have a dy component because it can never move sideways. Holonomic
 * drivetrains such as swerve and mecanum will often have all three components.
 *
 * <p>dx represents forward velocity w.r.t the robot frame of reference. (Fwd is +)
 *
 * <p>dy represents sideways velocity w.r.t the robot frame of reference. (Left is +)
 *
 * <p>dtheta represents the angular velocity of the robot frame. (CCW is +)
 */
@SuppressWarnings("MemberName")
public class ChassisSpeeds {
  public double dx;
  public double dy;
  public double dtheta;

  /**
   * Constructs a ChassisSpeeds with zeros for dx, dy, and theta.
   */
  public ChassisSpeeds() {
  }

  /**
   * Constructs a ChassisSpeeds object.
   *
   * @param dx     Forward velocity.
   * @param dy     Sideways velocity.
   * @param dtheta Angular velocity.
   */
  public ChassisSpeeds(double dx, double dy, double dtheta) {
    this.dx = dx;
    this.dy = dy;
    this.dtheta = dtheta;
  }

  /**
   * Converts a user provided field-relative set of speeds into a robot-relative
   * ChassisSpeeds object.
   *
   * @param vx         The component of speed in the x direction relative to the field.
   *                   Positive x is away from your alliance wall.
   * @param vy         The component of speed in the y direction relative to the field.
   *                   Positive y is to your left when standing behind the alliance wall.
   * @param vtheta     The angular rate of the robot.
   * @param robotAngle The angle of the robot as measured by a gyroscope.
   *                   Remember that this should be CCW positive.
   *
   * @return ChassisSpeeds object representing the speeds in the robot's frame of reference.
   */
  public static ChassisSpeeds fromFieldRelativeSpeeds(
      double vx, double vy, double vtheta, Rotation2d robotAngle) {
    return new ChassisSpeeds(
        vx * robotAngle.getCos() + vy * robotAngle.getSin(),
        -vx * robotAngle.getSin() + vy * robotAngle.getCos(),
        vtheta
    );
  }
}
