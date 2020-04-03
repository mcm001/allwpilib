/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.controller;

import java.util.ArrayList;

import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Twist2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVelocitySystemConstraint;
import edu.wpi.first.wpiutil.math.MatBuilder;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.MatrixUtils;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpilibj.system.RungeKutta;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N2;
import edu.wpi.first.wpiutil.math.numbers.N10;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class LTVDiffDriveControllerTest {
  private static final double kTolerance = 1 / 12.0;
  private static final double kAngularTolerance = Math.toRadians(2);

  private Matrix<N10, N1> x;

  private static double boundRadians(double value) {
    while (value > Math.PI) {
      value -= Math.PI * 2;
    }
    while (value <= -Math.PI) {
      value += Math.PI * 2;
    }
    return value;
  }

  @Test
  @SuppressWarnings("PMD.AvoidInstantiatingObjectsInLoops")
  void yes() {
    final LinearSystem<N2, N2, N2> plant = LinearSystem.identifyDrivetrainSystem(3.02, 0.642, 1.382, 0.08495, 10);

    final var controller = new LTVDiffDriveController(
        plant,
        new MatBuilder<>(Nat.N5(), Nat.N1()).fill(0.0625, 0.125, 2.5, 0.95, 0.95),
        new MatBuilder<>(Nat.N2(), Nat.N1()).fill(12.0, 12.0),
        new MatBuilder<>(Nat.N10(), Nat.N1()).fill(0.002, 0.002, 0.0001, 1.5, 1.5, 0.5, 0.5, 10.0, 10.0, 2.0),
        new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.0001, 0.005, 0.005),
        new MatBuilder<>(Nat.N6(), Nat.N1()).fill(0.005, 0.005, 0.005, 0.0001, 0.005, 0.005),
        0.02,
        1);

    final var waypoints = new ArrayList<Pose2d>();

    controller.setMeasuredLocalOutputs(0.0, 0.0, 0.0);

    waypoints.add(new Pose2d());
    waypoints.add(new Pose2d(4.8768, 2.7432, new Rotation2d(0)));
    var config = new TrajectoryConfig(12 / 3.02, 12 / 0.642);

    var kinematics = new DifferentialDriveKinematics(1);

    var constraint = new DifferentialDriveVelocitySystemConstraint(plant, kinematics, 10);
    config.addConstraint(constraint);

    final var trajectory = TrajectoryGenerator.generateTrajectory(waypoints, config);

    controller.reset(new Pose2d());


    final double kDt = 0.02;
    final var totalTime = trajectory.getTotalTimeSeconds();

    x = MatrixUtils.zeros(Nat.N10(), Nat.N1());

    var u = MatrixUtils.zeros(Nat.N2(), Nat.N1());

    /*for (int i = 0; i < (totalTime / kDt); ++i) {
      var y = controller.getLocalMeasurementModel(x, MatrixUtils.zeros(Nat.N2(), Nat.N1()));

      controller.setMeasuredLocalOutputs(y.get(0, 0), y.get(1, 0), y.get(2, 0));
      controller.setMeasuredInputs(u.get(0, 0), u.get(1, 0));

      var inputs = controller.calculate(trajectory.sample(i));

      u = new MatBuilder<>(Nat.N2(), Nat.N1()).fill(inputs.leftVolts, inputs.rightVolts);

      x = RungeKutta.rungeKutta(controller::getDynamics, x, u, kDt);

      assertTrue(controller.atReference());
    }*/
  }
}
