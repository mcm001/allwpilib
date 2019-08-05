/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.kinematics;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Twist2d;

/**
 * Class for mecnaum drive odometry. Odometry allows you to track the robot's
 * position on the field over a course of a match using readings from your
 * mecanum wheel encoders.
 *
 * <p>Teams can use odometry during the autonomous period for complex tasks like
 * path following. Furthermore, odometry can be used for latency compensation
 * when using computer-vision systems.
 */
public class MecanumDriveOdometry {
  private final MecanumDriveKinematics m_kinematics;
  private Pose2d m_pose;
  private double m_prevTimeSeconds = -1;

  /**
   * Constructs a MecanumDriveOdometry object.
   *
   * @param kinematics  The mecanum drive kinematics for your drivetrain.
   * @param initialPose The starting position of the robot on the field.
   */
  public MecanumDriveOdometry(MecanumDriveKinematics kinematics, Pose2d initialPose) {
    m_kinematics = kinematics;
    m_pose = initialPose;
  }

  /**
   * Constructs a MecanumDriveOdometry object with the default pose at the origin.
   *
   * @param kinematics The mecanum drive kinematics for your drivetrain.
   */
  public MecanumDriveOdometry(MecanumDriveKinematics kinematics) {
    this(kinematics, new Pose2d());
  }

  /**
   * Resets the robot's position on the field.
   *
   * @param pose The position on the field that your robot is at.
   */
  public synchronized void resetPosition(Pose2d pose) {
    m_pose = pose;
  }

  /**
   * Updates the robot's position on the field using forward kinematics and
   * integration of the pose over time. This method takes in the current time as
   * a parameter to calculate period (difference between two timestamps). The
   * period is used to calculate the change in distance from a velocity.
   *
   * @param currentTimeSeconds The current time in seconds.
   * @param wheelSpeeds        The current wheel speeds.
   * @return The new pose of the robot.
   */
  public synchronized Pose2d updateWithTime(double currentTimeSeconds,
                                            MecanumDriveWheelSpeeds wheelSpeeds) {
    double period = m_prevTimeSeconds >= 0 ? currentTimeSeconds - m_prevTimeSeconds : 0.0;
    m_prevTimeSeconds = currentTimeSeconds;

    var chassisState = m_kinematics.toChassisSpeeds(wheelSpeeds);
    m_pose = m_pose.exp(
        new Twist2d(chassisState.vx * period, chassisState.vy * period,
            chassisState.omega * period));

    return m_pose;
  }

  /**
   * Updates the robot's position on the field using forward kinematics and
   * integration of the pose over time. This method takes in the current time as
   * a parameter to calculate period (difference between two timestamps). The
   * period is used to calculate the change in distance from a velocity. This
   * also takes in an angular rate parameter which is used instead of the
   * angular rate that is calculated from forward kinematics.
   *
   * @param currentTimeSeconds The current time in seconds.
   * @param angularRateRadians The angular rate of the robot in radians.
   * @param wheelSpeeds        The current wheel speeds.
   * @return The new pose of the robot.
   */
  public synchronized Pose2d updateWithTime(double currentTimeSeconds, double angularRateRadians,
                                            MecanumDriveWheelSpeeds wheelSpeeds) {
    double period = m_prevTimeSeconds >= 0 ? currentTimeSeconds - m_prevTimeSeconds : 0.0;
    m_prevTimeSeconds = currentTimeSeconds;

    var chassisState = m_kinematics.toChassisSpeeds(wheelSpeeds);
    m_pose = m_pose.exp(
        new Twist2d(chassisState.vx * period, chassisState.vy * period,
            angularRateRadians * period));

    return m_pose;
  }

  /**
   * Updates the robot's position on the field using forward kinematics and
   * integration of the pose over time. This method automatically calculates the
   * current time to calculate period (difference between two timestamps). The
   * period is used to calculate the change in distance from a velocity.
   *
   * @param wheelSpeeds The current wheel speeds.
   * @return The new pose of the robot.
   */
  public synchronized Pose2d update(MecanumDriveWheelSpeeds wheelSpeeds) {
    return updateWithTime(System.currentTimeMillis() / 1000.0, wheelSpeeds);
  }

  /**
   * Updates the robot's position on the field using forward kinematics and
   * integration of the pose over time. This method automatically calculates the
   * current time to calculate period (difference between two timestamps). The
   * period is used to calculate the change in distance from a velocity. This
   * also takes in an angular rate parameter which is used instead of the
   * angular rate that is calculated from forward kinematics.
   *
   * @param angularRateRadians The angular rate of the robot in radians.
   * @param wheelSpeeds        The current wheel speeds.
   * @return The new pose of the robot.
   */
  public synchronized Pose2d update(double angularRateRadians,
                                    MecanumDriveWheelSpeeds wheelSpeeds) {
    return updateWithTime(System.currentTimeMillis() / 1000.0, angularRateRadians, wheelSpeeds);
  }
}
