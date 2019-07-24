/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.kinematics;

import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;

class SwerveDriveKinematicsTest {
  private static final double kEpsilon = 1E-9;

  private final Translation2d m_fl = new Translation2d(12, 12);
  private final Translation2d m_fr = new Translation2d(12, -12);
  private final Translation2d m_bl = new Translation2d(-12, 12);
  private final Translation2d m_br = new Translation2d(-12, -12);

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(m_fl, m_fr, m_bl, m_br);

  @Test
  void testStraightLineInverseKinematics() {

    ChassisSpeeds speeds = new ChassisSpeeds(5, 0, 0);
    var moduleStates = m_kinematics.toSwerveModuleStates(speeds);

    assertAll(
        () -> assertEquals(5.0, moduleStates[0].speed, kEpsilon),
        () -> assertEquals(5.0, moduleStates[1].speed, kEpsilon),
        () -> assertEquals(5.0, moduleStates[2].speed, kEpsilon),
        () -> assertEquals(5.0, moduleStates[3].speed, kEpsilon),
        () -> assertEquals(0.0, moduleStates[0].angle.getRadians(), kEpsilon),
        () -> assertEquals(0.0, moduleStates[1].angle.getRadians(), kEpsilon),
        () -> assertEquals(0.0, moduleStates[2].angle.getRadians(), kEpsilon),
        () -> assertEquals(0.0, moduleStates[3].angle.getRadians(), kEpsilon)
    );
  }

  @Test
  void testStraightLineForwardKinematics() {
    SwerveModuleState state = new SwerveModuleState(5.0, Rotation2d.fromDegrees(90.0));
    var chassisSpeeds = m_kinematics.toChassisSpeeds(state, state, state, state);

    assertAll(
        () -> assertEquals(0.0, chassisSpeeds.dx, kEpsilon),
        () -> assertEquals(5.0, chassisSpeeds.dy, kEpsilon),
        () -> assertEquals(0.0, chassisSpeeds.dtheta, kEpsilon)
    );
  }
}
