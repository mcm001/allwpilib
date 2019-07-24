/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.kinematics;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Twist2d;

/**
 * Class for differential drive odometry. Odometry allows you to track the
 * robot's position on the field over the course of a match using readings from
 * 2 encoders and a gyroscope.
 *
 * <p>Teams can use odometry during the autonomous period for complex tasks like
 * path following. Furthermore, odometry can be used for latency compensation
 * when using computer-vision systems.
 *
 * <p>Note: It is important to reset both your encoders to zero before you start
 * using this class. Only reset your encoders ONCE. You should not reset your
 * encoders even if you want to reset your robot's pose.
 */
public class DifferentialDriveOdometry {
  private final DifferentialDriveKinematics m_kinematics;
  private Pose2d m_pose;

  private double m_prevLeftEncoder;
  private double m_prevRightEncoder;
  private Rotation2d m_prevAngle;

  /**
   * Constructs a DifferentialDriveOdometry object.
   *
   * @param kinematics  The differential drive kinematics for your drivetrain.
   * @param initialPose The starting position of the robot on the field.
   */
  public DifferentialDriveOdometry(DifferentialDriveKinematics kinematics, Pose2d initialPose) {
    m_kinematics = kinematics;
    m_pose = initialPose;
    m_prevAngle = initialPose.getRotation();
  }

  /**
   * Constructs a DifferentialDriveOdometry object with the default pose at the origin.
   *
   * @param kinematics The differential drive kinematics for your drivetrain.
   */
  public DifferentialDriveOdometry(DifferentialDriveKinematics kinematics) {
    this(kinematics, new Pose2d());
  }

  /**
   * Resets the robot's position on the field. Do NOT zero your encoders if you
   * call this function at any other time except initialization.
   *
   * @param pose The position on the field that your robot is at.
   */
  public synchronized void resetPosition(Pose2d pose) {
    m_pose = pose;
    m_prevAngle = pose.getRotation();
  }

  /**
   * Updates the robot's position on the field using forward kinematics and
   * integration of the pose over time.
   *
   * @param leftEncoder  The value of the left encoder (position). The units for x and y of
   *                     the returned pose are the same as the units you pass in here. Therefore, it
   *                     is advised that you convert the raw encoder value into meters, feet, or
   *                     inches.
   * @param rightEncoder The value of the right encoder (position). The units for x and y
   *                     of the returned pose are the same as the units you pass in here. Therefore,
   *                     it is advised that you convert the raw encoder value into meters, feet, or
   *                     inches.
   * @return The new pose of the robot.
   */
  public synchronized Pose2d update(double leftEncoder, double rightEncoder) {
    final var deltaLeft = leftEncoder - m_prevLeftEncoder;
    final var deltaRight = rightEncoder - m_prevRightEncoder;

    final var chassisState = m_kinematics.toChassisSpeeds(
        new DifferentialDriveWheelSpeeds(deltaLeft, deltaRight));

    m_pose = m_pose.exp(new Twist2d(chassisState.dx, chassisState.dy, chassisState.dtheta));

    m_prevLeftEncoder = leftEncoder;
    m_prevRightEncoder = rightEncoder;
    m_prevAngle = m_prevAngle.plus(new Rotation2d(chassisState.dtheta));

    return m_pose;
  }

  /**
   * Updates the robot's position on the field using forward kinematics and
   * integration of the pose over time. This method uses input from the
   * gyro instead of pure forward kinematics for angular data.
   *
   * @param leftEncoder  The value of the left encoder (position). The units for x and y of
   *                     the returned pose are the same as the units you pass in here. Therefore, it
   *                     is advised that you convert the raw encoder value into meters, feet, or
   *                     inches.
   * @param rightEncoder The value of the right encoder (position). The units for x and y
   *                     of the returned pose are the same as the units you pass in here. Therefore,
   *                     it is advised that you convert the raw encoder value into meters, feet, or
   *                     inches.
   * @param gyro         The yaw of the robot from a gyroscope.
   * @return The new pose of the robot.
   */
  public synchronized Pose2d update(double leftEncoder, double rightEncoder, Rotation2d gyro) {
    final var deltaLeft = leftEncoder - m_prevLeftEncoder;
    final var deltaRight = rightEncoder - m_prevRightEncoder;

    final var dx = (deltaLeft + deltaRight) / 2;
    final var dtheta = gyro.minus(m_prevAngle);

    m_pose = m_pose.exp(new Twist2d(dx, 0, dtheta.getRadians()));

    m_prevLeftEncoder = leftEncoder;
    m_prevRightEncoder = rightEncoder;
    m_prevAngle = gyro;

    return m_pose;
  }
}
