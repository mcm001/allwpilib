/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.controller;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.estimator.DifferentialDriveStateEstimator;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.math.StateSpaceUtil;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.RungeKutta;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVelocitySystemConstraint;
import edu.wpi.first.wpiutil.math.MatBuilder;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.MatrixUtils;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.SimpleMatrixUtils;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N10;
import edu.wpi.first.wpiutil.math.numbers.N2;
import edu.wpi.first.wpiutil.math.numbers.N3;
import org.knowm.xchart.SwingWrapper;
import org.knowm.xchart.XYChartBuilder;

import static org.junit.jupiter.api.Assertions.assertTrue;

@SuppressWarnings({"LocalVariableName", "PMD.AvoidInstantiatingObjectsInLoops", "MemberName"})
class LTVDiffDriveControllerTest {
  private Matrix<N10, N1> x;
  private Matrix<N2, N1> u;

  private LTVDiffDriveController controller;
  private DifferentialDriveStateEstimator estimator;
  private LinearSystemFeedForward<N2, N2, N2> feedforward;

  private DifferentialDriveKinematics kinematics;

  private Trajectory trajectory;

  private double totalTime;

  @BeforeEach
  void setUp() {
    final LinearSystem<N2, N2, N2> plant = LinearSystem.identifyDrivetrainSystem(
            3.02, 0.642, 1.382, 0.08495, 10);
    kinematics = new DifferentialDriveKinematics(1);

    controller = new LTVDiffDriveController(
            plant,
            new MatBuilder<>(Nat.N5(), Nat.N1()).fill(0.001, 0.002, 20.5, 1.0, 1.0),
            new MatBuilder<>(Nat.N2(), Nat.N1()).fill(12.0, 12.0),
            kinematics,
            0.02);

    estimator = new DifferentialDriveStateEstimator(
      plant,
      MatrixUtils.zeros(Nat.N10()),
      new MatBuilder<>(Nat.N10(), Nat.N1()).fill(0.02, 0.02, 0.01, 0.1, 0.1,
              0.02, 0.02, 0.1, 0.1, 0.01),
      new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01),
      new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.01),
      kinematics);

    feedforward = new LinearSystemFeedForward<>(plant, 0.02);

    final var waypoints = new ArrayList<Pose2d>();
    waypoints.add(new Pose2d());
    waypoints.add(new Pose2d(4.8768, 2.7432, new Rotation2d(0)));

    var config = new TrajectoryConfig(12 / 3.02, 12 / 0.642);

    var constraint = new DifferentialDriveVelocitySystemConstraint(plant, kinematics, 10);
    config.addConstraint(constraint);

    trajectory = TrajectoryGenerator.generateTrajectory(waypoints, config);
    totalTime = trajectory.getTotalTimeSeconds();
  }

  private void scaleCappedU(Matrix<N2, N1> u) {
    boolean isOutputCapped = Math.abs(u.get(0, 0)) > 12.0 || Math.abs(u.get(1, 0)) > 12.0;

    if (isOutputCapped) {
      u.times(12.0 / CommonOps_DDRM.elementMaxAbs(u.getStorage().getDDRM()));
    }
  }

  @Test
  void trackingTest() {
    controller.reset();
    estimator.reset();
    controller.setTolerance(new Pose2d(0.06, 0.06, new Rotation2d(0.06)), 0.3);

    List<Double> trajXs = new ArrayList<>();
    List<Double> trajYs = new ArrayList<>();
    List<Double> observerXs = new ArrayList<>();
    List<Double> observerYs = new ArrayList<>();

    final double kDt = 0.02;

    x = MatrixUtils.zeros(Nat.N10(), Nat.N1());
    u = MatrixUtils.zeros(Nat.N2(), Nat.N1());

    var prevInput = MatrixUtils.zeros(Nat.N2());

    double t = 0.0;

    while (t < totalTime) {
      var y = estimator.getLocalMeasurementModel(x, MatrixUtils.zeros(Nat.N2(), Nat.N1()));

      var currentState = estimator.updateWithTime(y.get(0, 0), y.get(1, 0), y.get(2, 0), prevInput, t);

      var desiredState = trajectory.sample(t);

      observerXs.add(currentState.get(0, 0));
      observerYs.add(currentState.get(1, 0));

      trajXs.add(trajectory.sample(t).poseMeters.getTranslation().getX());
      trajYs.add(trajectory.sample(t).poseMeters.getTranslation().getY());


      System.out.println(currentState.block(Nat.N5(), Nat.N1(), new SimpleMatrixUtils.Pair<>(0, 0)));

      var input = controller.calculate(currentState.block(Nat.N5(), Nat.N1(), new SimpleMatrixUtils.Pair<>(0, 0)), desiredState);

      u = new MatBuilder<>(Nat.N2(), Nat.N1()).fill(input.leftVolts, input.rightVolts);

      var futureWheelVelocities = kinematics.toWheelSpeeds(
              new ChassisSpeeds(trajectory.sample(t + kDt).velocityMetersPerSecond,
                      0,
                      trajectory.sample(t + kDt).velocityMetersPerSecond * trajectory.sample(t + kDt).curvatureRadPerMeter));

      u.plus(feedforward.calculate(new MatBuilder<>(Nat.N2(), Nat.N1()).fill(futureWheelVelocities.leftMetersPerSecond, futureWheelVelocities.rightMetersPerSecond)));

      //System.out.println(feedforward.calculate(new MatBuilder<>(Nat.N2(), Nat.N1()).fill(futureWheelVelocities.leftMetersPerSecond, futureWheelVelocities.rightMetersPerSecond)));

      scaleCappedU(u);

      System.out.println(u);

      prevInput = u;

      x = RungeKutta.rungeKutta(controller::getDynamics, x, u, kDt);

      t += kDt;

      //assertTrue(controller.atReference());
    }

    var chartBuilder = new XYChartBuilder();
    chartBuilder.title = "The Magic of Sensor Fusion";
    var chart = chartBuilder.build();

    chart.addSeries("Trajectory", trajXs, trajYs);
    chart.addSeries("xHat", observerXs, observerYs);

    new SwingWrapper<>(chart).displayChart();
    try {
     Thread.sleep(1000000000);
    } catch (InterruptedException e) {
    }
  }
  /*
  @Test
  void trackingTestGlobal() {
    controller.reset(new Pose2d());
    controller.setMeasuredLocalOutputs(0.0, 0.0, 0.0);
    controller.setTolerance(new Pose2d(0.06, 0.06, new Rotation2d(0.06)), 0.3);

    final double kDt = 0.02;

    x = MatrixUtils.zeros(Nat.N10(), Nat.N1());
    u = MatrixUtils.zeros(Nat.N2(), Nat.N1());

    for (double i = 0; i < totalTime; i = i + kDt) {
      var y = controller.getGlobalMeasurementModel(x, MatrixUtils.zeros(Nat.N2(), Nat.N1()));

      y.plus(StateSpaceUtil.makeWhiteNoiseVector(Nat.N6(),
              new MatBuilder<>(Nat.N6(), Nat.N1()).fill(0.005, 0.005, 0.005, 0.0001, 1.5, 1.5)));

      controller.setMeasuredGlobalOutputs(y.get(0, 0), y.get(1, 0),
              y.get(2, 0), y.get(3, 0), y.get(4, 0), y.get(5, 0));

      //controller.setMeasuredInputs(u.get(0, 0), u.get(1, 0));

      var inputs = controller.calculate(trajectory.sample(i));

      u = new MatBuilder<>(Nat.N2(), Nat.N1()).fill(inputs.leftVolts, inputs.rightVolts);

      x = RungeKutta.rungeKutta(controller::getDynamics, x, u, kDt);

      assertTrue(controller.atReference());
    }
  }

  @Test
  void trackingTestNoise() {
    controller.reset(new Pose2d());
    controller.setMeasuredLocalOutputs(0.0, 0.0, 0.0);
    controller.setTolerance(new Pose2d(1, 1, new Rotation2d(0.5)), 3.0);

    double kDt = 0.02;

    x = MatrixUtils.zeros(Nat.N10(), Nat.N1());
    u = MatrixUtils.zeros(Nat.N2(), Nat.N1());


    for (double i = 0; i < totalTime; i = i + kDt) {
      kDt += StateSpaceUtil.makeWhiteNoiseVector(Nat.N1(),
              new MatBuilder<>(Nat.N1(), Nat.N1()).fill(0.0005)).get(0, 0);

      var y = controller.getLocalMeasurementModel(x, MatrixUtils.zeros(Nat.N2(), Nat.N1()));

      y.plus(StateSpaceUtil.makeWhiteNoiseVector(Nat.N3(),
              new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.0001, 0.005, 0.005)));

      controller.setMeasuredLocalOutputs(y.get(0, 0), y.get(1, 0), y.get(2, 0));
      //controller.setMeasuredInputs(u.get(0, 0), u.get(1, 0));

      var inputs = controller.calculate(trajectory.sample(i));

      u = new MatBuilder<>(Nat.N2(), Nat.N1()).fill(inputs.leftVolts, inputs.rightVolts);

      x = RungeKutta.rungeKutta(controller::getDynamics, x, u, kDt);

      assertTrue(controller.atReference());
    }
  }
  */
}
