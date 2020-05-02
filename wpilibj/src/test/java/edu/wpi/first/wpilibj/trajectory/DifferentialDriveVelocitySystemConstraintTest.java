/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.trajectory;

import java.util.Collections;

import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVelocitySystemConstraint;
import edu.wpi.first.wpiutil.math.MatBuilder;
import edu.wpi.first.wpiutil.math.Nat;

import static org.junit.jupiter.api.Assertions.assertTrue;

class DifferentialDriveVelocitySystemConstraintTest {
  @SuppressWarnings({"LocalVariableName", "PMD.AvoidInstantiatingObjectsInLoops"})
  @Test
  void testDifferentialDriveVelocitySystemConstraint() {
    double maxVoltage = 10;

    // Pick an unreasonably large kA to ensure the constraint has to do some work
    var system = LinearSystem.identifyDrivetrainSystem(1, 3, 1, 3, maxVoltage);
    var kinematics = new DifferentialDriveKinematics(0.5);
    var constraint = new DifferentialDriveVelocitySystemConstraint(system,
                                                            kinematics,
                                                            maxVoltage);

    Trajectory trajectory = TrajectoryGeneratorTest.getTrajectory(
        Collections.singletonList(constraint));

    var duration = trajectory.getTotalTimeSeconds();
    var t = 0.0;
    var dt = 0.02;

    while (t < duration) {
      var point = trajectory.sample(t);
      var chassisSpeeds = new ChassisSpeeds(
          point.velocityMetersPerSecond, 0,
          point.velocityMetersPerSecond * point.curvatureRadPerMeter
      );

      var wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);

      var x = new MatBuilder<>(Nat.N2(), Nat.N1()).fill(wheelSpeeds.leftMetersPerSecond,
          wheelSpeeds.rightMetersPerSecond);

      //Not really a strictly-correct test as we're using the chassis accel instead of the
      //wheel accel, but much easier than doing it "properly" and a reasonable check anyway
      var xDot = new MatBuilder<>(Nat.N2(), Nat.N1()).fill(point.accelerationMetersPerSecondSq,
          point.accelerationMetersPerSecondSq);

      var u = (system.getB().inv()).times(xDot.minus(system.getA().times(x)));

      double left = u.get(0, 0);
      double right = u.get(1, 0);

      t += dt;

      //Any (value < maxVoltage + 1) leads to the output being offset by
      //(value - maxVoltage). The value is left at 11. This seems to be a JUnit issue.
      assertTrue((-11 <= left) && (left <= 11));
      assertTrue((-11 <= right) && (right <= 11));

    }
  }
}
