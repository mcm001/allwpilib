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
        () -> assertEquals(0.0, chassisSpeeds.vx, kEpsilon),
        () -> assertEquals(5.0, chassisSpeeds.vy, kEpsilon),
        () -> assertEquals(0.0, chassisSpeeds.omega, kEpsilon)
    );
  }

  @Test
  void testStraightStrafeInverseKinematics() {

    ChassisSpeeds speeds = new ChassisSpeeds(0, 5, 0);
    var moduleStates = m_kinematics.toSwerveModuleStates(speeds);

    assertAll(
            () -> assertEquals(5.0, moduleStates[0].speed, kEpsilon),
            () -> assertEquals(5.0, moduleStates[1].speed, kEpsilon),
            () -> assertEquals(5.0, moduleStates[2].speed, kEpsilon),
            () -> assertEquals(5.0, moduleStates[3].speed, kEpsilon),
            () -> assertEquals(90.0, moduleStates[0].angle.getDegrees(), kEpsilon),
            () -> assertEquals(90.0, moduleStates[1].angle.getDegrees(), kEpsilon),
            () -> assertEquals(90.0, moduleStates[2].angle.getDegrees(), kEpsilon),
            () -> assertEquals(90.0, moduleStates[3].angle.getDegrees(), kEpsilon)
    );
  }

    @Test
    void testStraightStrafeForwardKinematics() {
        SwerveModuleState state = new SwerveModuleState(5.0, Rotation2d.fromDegrees(0.0));
        var chassisSpeeds = m_kinematics.toChassisSpeeds(state, state, state, state);

        assertAll(
                () -> assertEquals(5.0, chassisSpeeds.vx, kEpsilon),
                () -> assertEquals(0.0, chassisSpeeds.vy, kEpsilon),
                () -> assertEquals(0.0, chassisSpeeds.omega, kEpsilon)
        );
    }

    @Test
    void testTurnInPlaceInverseKinematics() {

        ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 2*Math.PI);
        var moduleStates = m_kinematics.toSwerveModuleStates(speeds);

        /*
        The circumference of the wheels about the COR is pi * diameter, or 2 * pi * radius
        the radius is the sqrt(12^2in + 12^2in), or 16.9706in, so the circumference the wheels
        trace out is 106.629190516in. since we want our robot to rotate at 1 rotation per second,
        our wheels must trace out 1 rotation (or 106.63 inches) per second.
         */

        assertAll(
                () -> assertEquals(106.63, moduleStates[0].speed, 0.1),
                () -> assertEquals(106.63, moduleStates[1].speed, 0.1),
                () -> assertEquals(106.63, moduleStates[2].speed, 0.1),
                () -> assertEquals(106.63, moduleStates[3].speed, 0.1),
                () -> assertEquals(135.0, moduleStates[0].angle.getDegrees(), kEpsilon),
                () -> assertEquals(45.0, moduleStates[1].angle.getDegrees(), kEpsilon),
                () -> assertEquals(-135.0, moduleStates[2].angle.getDegrees(), kEpsilon),
                () -> assertEquals(-45.0, moduleStates[3].angle.getDegrees(), kEpsilon)
        );
    }

    @Test
    void testTurnInPlaceForwardKinematics() {
        SwerveModuleState fl_state = new SwerveModuleState(106.629, Rotation2d.fromDegrees(135));
        SwerveModuleState fr_state = new SwerveModuleState(106.629, Rotation2d.fromDegrees(45));
        SwerveModuleState bl_state = new SwerveModuleState(106.629, Rotation2d.fromDegrees(-135));
        SwerveModuleState br_state = new SwerveModuleState(106.629, Rotation2d.fromDegrees(-45));

        var chassisSpeeds = m_kinematics.toChassisSpeeds(fl_state, fr_state, bl_state, br_state);

        assertAll(
                () -> assertEquals(0.0, chassisSpeeds.vx, kEpsilon),
                () -> assertEquals(0.0, chassisSpeeds.vy, kEpsilon),
                () -> assertEquals(2 * Math.PI, chassisSpeeds.omega, 0.1)
        );
    }

    @Test
    void testOffCenterCORInverseKinematics() {

        ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 2*Math.PI);
        var moduleStates = m_kinematics.toSwerveModuleStates(speeds, m_fl);

        /*
        This one is a bit trickier. Because we are rotating about the front-left wheel,
        it should be parked at 0 degrees and 0 speed. The front-right and back-left wheels both travel an
        arc with radius 24 (and circumference 150.796), and the back-right wheel travels an arc with
        radius sqrt(24^2 + 24^2) and circumference 213.2584. As for angles, the front-right wheel should be
        pointing straight forward, the back-left wheel should be pointing straight right, and the back-right
        wheel should be at a -45 degree angle
         */

        assertAll(
                () -> assertEquals(0.0, moduleStates[0].speed, 0.1),
                () -> assertEquals(150.796, moduleStates[1].speed, 0.1),
                () -> assertEquals(150.796, moduleStates[2].speed, 0.1),
                () -> assertEquals(213.258, moduleStates[3].speed, 0.1),
                () -> assertEquals(0.0, moduleStates[0].angle.getDegrees(), kEpsilon),
                () -> assertEquals(0.0, moduleStates[1].angle.getDegrees(), kEpsilon),
                () -> assertEquals(-90.0, moduleStates[2].angle.getDegrees(), kEpsilon),
                () -> assertEquals(-45.0, moduleStates[3].angle.getDegrees(), kEpsilon)
        );
    }

    @Test
    void testOffCenterCORForwardKinematics() {
        SwerveModuleState fl_state = new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0));
        SwerveModuleState fr_state = new SwerveModuleState(150.796, Rotation2d.fromDegrees(0.0));
        SwerveModuleState bl_state = new SwerveModuleState(150.796, Rotation2d.fromDegrees(-90));
        SwerveModuleState br_state = new SwerveModuleState(213.258, Rotation2d.fromDegrees(-45));

        var chassisSpeeds = m_kinematics.toChassisSpeeds(fl_state, fr_state, bl_state, br_state);

        /*
        We already know that our omega should be 2pi from the previous test. Next, we need to determine
        the vx and vy of our chassis center. Because our COR is at a 45 degree angle from the center,
        we know that vx and vy must be the same. Furthermore, we know that the center of mass makes
        a full revolution about the center of revolution once every second. Therefore, the center of
        mass must be moving at 106.629in/sec. Recalling that the ratios of a 45/45/90 triagle are
        1:sqrt(2)/2:sqrt(2)/2, we find that the COM vx is -75.398, and vy is 75.398.
        */

        assertAll(
                () -> assertEquals(75.398, chassisSpeeds.vx, 0.1),
                () -> assertEquals(-75.398, chassisSpeeds.vy, 0.1),
                () -> assertEquals(2 * Math.PI, chassisSpeeds.omega, 0.1)
        );
    }

}
