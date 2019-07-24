/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.kinematics;


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
}
