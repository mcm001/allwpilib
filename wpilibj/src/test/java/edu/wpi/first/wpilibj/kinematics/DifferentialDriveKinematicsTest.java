/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.kinematics;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;

class DifferentialDriveKinematicsTest {
  private static final double kEpsilon = 1E-9;
  private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(0.381);

  @Test
  void testInverseKinematicsForZeros() {
    var chassisSpeeds = new ChassisSpeeds();
    var wheelSpeeds = m_kinematics.toWheelSpeeds(chassisSpeeds);

    assertAll(
        () -> assertEquals(0.0, wheelSpeeds.left, kEpsilon),
        () -> assertEquals(0.0, wheelSpeeds.right, kEpsilon)
    );
  }

  @Test
  void testForwardKinematicsForZeros() {
    var wheelSpeeds = new DifferentialDriveWheelSpeeds();
    var chassisSpeeds = m_kinematics.toChassisSpeeds(wheelSpeeds);

    assertAll(
        () -> assertEquals(0.0, chassisSpeeds.vx, kEpsilon),
        () -> assertEquals(0.0, chassisSpeeds.vy, kEpsilon),
        () -> assertEquals(0.0, chassisSpeeds.omega, kEpsilon)
    );
  }

  @Test
  void testInverseKinematicsForStraightLine() {
    var chassisSpeeds = new ChassisSpeeds(3, 0, 0);
    var wheelSpeeds = m_kinematics.toWheelSpeeds(chassisSpeeds);

    assertAll(
        () -> assertEquals(3.0, wheelSpeeds.left, kEpsilon),
        () -> assertEquals(3.0, wheelSpeeds.right, kEpsilon)
    );
  }

  @Test
  void testForwardKinematicsForStraightLine() {
    var wheelSpeeds = new DifferentialDriveWheelSpeeds(3, 3);
    var chassisSpeeds = m_kinematics.toChassisSpeeds(wheelSpeeds);

    assertAll(
        () -> assertEquals(3.0, chassisSpeeds.vx, kEpsilon),
        () -> assertEquals(0.0, chassisSpeeds.vy, kEpsilon),
        () -> assertEquals(0.0, chassisSpeeds.omega, kEpsilon)
    );
  }

  @Test
  void testInverseKinematicsForRotateInPlace() {
    var chassisSpeeds = new ChassisSpeeds(0, 0, Math.PI);
    var wheelSpeeds = m_kinematics.toWheelSpeeds(chassisSpeeds);

    assertAll(
        () -> assertEquals(-0.381 * Math.PI, wheelSpeeds.left, kEpsilon),
        () -> assertEquals(+0.381 * Math.PI, wheelSpeeds.right, kEpsilon)
    );
  }

  @Test
  void testForwardKinematicsForRotateInPlace() {
    var wheelSpeeds = new DifferentialDriveWheelSpeeds(+0.381 * Math.PI, -0.381 * Math.PI);
    var chassisSpeeds = m_kinematics.toChassisSpeeds(wheelSpeeds);

    assertAll(
        () -> assertEquals(0.0, chassisSpeeds.vx, kEpsilon),
        () -> assertEquals(0.0, chassisSpeeds.vy, kEpsilon),
        () -> assertEquals(-Math.PI, chassisSpeeds.omega, kEpsilon)
    );
  }
}