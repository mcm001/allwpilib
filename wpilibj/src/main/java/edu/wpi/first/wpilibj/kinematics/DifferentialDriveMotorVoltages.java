/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.kinematics;

/**
 * Represents the motor voltages for a differential drive drivetrain.
 */
@SuppressWarnings("MemberName")
public class DifferentialDriveMotorVoltages {
  /**
   * Voltage of the left side of the robot.
   */
  public double leftVolts;

  /**
   * Voltage of the right side of the robot.
   */
  public double rightVolts;

  /**
   * Constructs a DifferentialDriveMotorVoltages with zeros for left and right voltages.
   */
  public DifferentialDriveMotorVoltages() {
  }

  /**
   * Constructs a DifferentialDriveMotorVoltages.
   *
   * @param leftVolts  The left voltage.
   * @param rightVolts The right voltage.
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
