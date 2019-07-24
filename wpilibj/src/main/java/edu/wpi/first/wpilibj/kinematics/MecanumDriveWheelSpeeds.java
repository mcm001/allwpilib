/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.kinematics;

import java.util.stream.DoubleStream;

@SuppressWarnings("MemberName")
public class MecanumDriveWheelSpeeds {
  /**
   * Speed of the front left wheel.
   */
  public double frontLeft;

  /**
   * Speed of the front right wheel.
   */
  public double frontRight;

  /**
   * Speed of the rear left wheel.
   */
  public double rearLeft;

  /**
   * Speed of the rear right wheel.
   */
  public double rearRight;

  /**
   * Constructs a MecanumDriveWheelSpeeds with zeros for all member fields.
   */
  public MecanumDriveWheelSpeeds() {
  }

  /**
   * Constructs a MecanumDriveWheelSpeeds.
   *
   * @param frontLeft  Speed of the front left wheel.
   * @param frontRight Speed of the front right wheel.
   * @param rearLeft   Speed of the rear left wheel.
   * @param rearRight  Speed of the rear right wheel.
   */
  public MecanumDriveWheelSpeeds(double frontLeft, double frontRight,
                                 double rearLeft, double rearRight) {
    this.frontLeft = frontLeft;
    this.frontRight = frontRight;
    this.rearLeft = rearLeft;
    this.rearRight = rearRight;
  }

  /**
   * Normalizes the wheel speeds using some max attainable speed. Sometimes,
   * after inverse kinematics, the requested speed from a/several modules may be
   * above the max attainable speed for the driving motor on that module. To fix
   * this issue, one can "normalize" all the wheel speeds to make sure that all
   * requested module speeds are below the absolute threshold, while maintaining
   * the ratio of speeds between modules.
   *
   * @param attainableMaxSpeed The absolute max speed that a wheel can reach.
   */
  public void normalize(double attainableMaxSpeed) {
    double realMaxSpeed = DoubleStream.of(frontLeft, frontRight, rearLeft, rearRight)
        .max().getAsDouble();

    if (realMaxSpeed > attainableMaxSpeed) {
      frontLeft = frontLeft / realMaxSpeed * attainableMaxSpeed;
      frontRight = frontRight / realMaxSpeed * attainableMaxSpeed;
      rearLeft = rearLeft / realMaxSpeed * attainableMaxSpeed;
      rearRight = rearRight / realMaxSpeed * attainableMaxSpeed;
    }
  }
}
