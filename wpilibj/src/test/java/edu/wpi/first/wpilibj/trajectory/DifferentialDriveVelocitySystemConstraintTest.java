/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.trajectory;

import java.util.ArrayList;
import java.util.Collections;

import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVelocitySystemConstraint;
import edu.wpi.first.wpiutil.math.MatBuilder;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.VecBuilder;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertTrue;

class DifferentialDriveVelocitySystemConstraintTest {
  @SuppressWarnings({"LocalVariableName", "PMD.AvoidInstantiatingObjectsInLoops"})
  @Test
  void testDifferentialDriveVelocitySystemConstraint() {
    double maxVoltage = 10;

    // Pick an unreasonably large kA to ensure the constraint has to do some work
    var system = LinearSystemId.identifyDrivetrainSystem(1, 3, 1, 3);
    var kinematics = new DifferentialDriveKinematics(0.5);
    var constraint = new DifferentialDriveVelocitySystemConstraint(system,
          kinematics,
          maxVoltage);

    Trajectory trajectory = TrajectoryGeneratorTest.getTrajectory(
          Collections.singletonList(constraint));

    var duration = trajectory.getTotalTimeSeconds();
    var t = 0.0;
    var dt = 0.02;
    var previousSpeeds = new DifferentialDriveWheelSpeeds(0, 0);

    while (t < duration) {
      var point = trajectory.sample(t);
      var chassisSpeeds = new ChassisSpeeds(
            point.velocityMetersPerSecond, 0,
            point.velocityMetersPerSecond * point.curvatureRadPerMeter
      );

      var wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);

      var x = new MatBuilder<>(Nat.N2(), Nat.N1()).fill(wheelSpeeds.leftMetersPerSecond,
            wheelSpeeds.rightMetersPerSecond);

      var leftAccel = (wheelSpeeds.leftMetersPerSecond - previousSpeeds.leftMetersPerSecond) / dt;
      var rightAccel = (wheelSpeeds.rightMetersPerSecond
            - previousSpeeds.rightMetersPerSecond) / dt;
      var xDot = VecBuilder.fill(leftAccel, rightAccel);

      var u = (system.getB().inv()).times(xDot.minus(system.getA().times(x)));

      double left = u.get(0, 0);
      double right = u.get(1, 0);

      t += dt;
      previousSpeeds = wheelSpeeds;

      System.out.println(left + ", " + right);
      assertTrue((-10 <= left) && (left <= 10));
      assertTrue((-10 <= right) && (right <= 10));

    }
  }

  @Test
  void testEndpointHighCurvature() {
    double maxVoltage = 10;

    var system = LinearSystemId.identifyDrivetrainSystem(1, 3, 1, 3);

    // Large trackwidth - need to test with radius of curvature less than half of trackwidth
    var kinematics = new DifferentialDriveKinematics(3);
    var constraint = new DifferentialDriveVelocitySystemConstraint(system,
          kinematics,
          maxVoltage);

    var config = new TrajectoryConfig(12, 12).addConstraint(constraint);

    // Radius of curvature should be ~1 meter.
    assertDoesNotThrow(() -> TrajectoryGenerator.generateTrajectory(
          new Pose2d(1, 0, Rotation2d.fromDegrees(90)),
          new ArrayList<Translation2d>(),
          new Pose2d(0, 1, Rotation2d.fromDegrees(180)),
          config));

    assertDoesNotThrow(() -> TrajectoryGenerator.generateTrajectory(
          new Pose2d(0, 1, Rotation2d.fromDegrees(180)),
          new ArrayList<Translation2d>(),
          new Pose2d(1, 0, Rotation2d.fromDegrees(90)),
          config.setReversed(true)));

  }
}
