/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.kinematics;

/**
 * Represents the wheel speeds for a differential drive drivetrain.
 */
@SuppressWarnings("MemberName")
public class DifferentialDriveMotorVoltages {
  /**
   * Speed of the left side of the robot.
   */
  public double leftVolts;

  /**
   * Speed of the right side of the robot.
   */
  public double rightVolts;

  /**
   * Constructs a DifferentialDriveWheelSpeeds with zeros for left and right speeds.
   */
  public DifferentialDriveMotorVoltages() {
  }

  /**
   * Constructs a DifferentialDriveWheelSpeeds.
   *
   * @param leftMetersPerSecond  The left speed.
   * @param rightMetersPerSecond The right speed.
   */
  public DifferentialDriveMotorVoltages(double leftVolts, double rightVolts) {
    this.leftVolts = leftVolts;
    this.rightVolts = rightVolts;
  }

  @Override
  public String toString() {
    return String.format("DifferentialDriveMotorVoltages(Left: %.2f volts, Right: %.2f volts)",
        leftVolts, rightVolts);
  }
}
