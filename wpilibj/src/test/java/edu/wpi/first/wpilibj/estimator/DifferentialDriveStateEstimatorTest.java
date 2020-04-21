/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.estimator;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpiutil.math.MatBuilder;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.MatrixUtils;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N2;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class DifferentialDriveStateEstimatorTest {
  @SuppressWarnings({"LocalVariableName", "PMD.AvoidInstantiatingObjectsInLoops"})
  @Test
  public void testAccuracy() {
    final LinearSystem<N2, N2, N2> plant = LinearSystem.identifyDrivetrainSystem(
        3.02, 0.642, 1.382, 0.08495, 10);
    var kinematics = new DifferentialDriveKinematics(1);

    var estimator = new DifferentialDriveStateEstimator(
            plant,
            MatrixUtils.zeros(Nat.N10()),
            new MatBuilder<>(Nat.N10(), Nat.N1()).fill(0.02, 0.02, 0.01, 0.1, 0.1,
                    0.02, 0.02, 0.1, 0.1, 0.01),
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.01, 0.01, 0.01),
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.01),
            kinematics);

    var traj = TrajectoryGenerator.generateTrajectory(
            List.of(
                    new Pose2d(),
                    new Pose2d(20, 20, Rotation2d.fromDegrees(0)),
                    new Pose2d(23, 23, Rotation2d.fromDegrees(173)),
                    new Pose2d(54, 54, new Rotation2d())
            ),
            new TrajectoryConfig(0.5, 2));

    var rand = new Random(604);

    List<Double> time = new ArrayList<>();
    List<Double> trajXs = new ArrayList<>();
    List<Double> trajYs = new ArrayList<>();
    List<Double> trajLeftVel = new ArrayList<>();
    List<Double> trajRightVel = new ArrayList<>();
    List<Double> observerXs = new ArrayList<>();
    List<Double> observerYs = new ArrayList<>();
    List<Double> observerLeftVelocity = new ArrayList<>();
    List<Double> observerRightVelocity = new ArrayList<>();
    List<Double> observerLeftVoltageError = new ArrayList<>();
    List<Double> observerRightVoltageError = new ArrayList<>();
    List<Double> observerAngularVelocityError = new ArrayList<>();
    List<Double> visionXs = new ArrayList<>();
    List<Double> visionYs = new ArrayList<>();

    final double dt = 0.02;
    double t = 0.0;

    final double visionUpdateRate = 0.1;
    Pose2d lastVisionPose = null;
    double lastVisionUpdateTime = Double.NEGATIVE_INFINITY;

    double distanceLeft = 0.0;
    double distanceRight = 0.0;

    double maxError = Double.NEGATIVE_INFINITY;
    double errorSum = 0;
    Trajectory.State groundtruthState;
    Matrix<N2, N1> input;

    Matrix<N2, N1> prevX = MatrixUtils.zeros(Nat.N2());

    while (t <= traj.getTotalTimeSeconds()) {
      groundtruthState = traj.sample(t);

      var chassisSpeeds = new ChassisSpeeds(
          groundtruthState.velocityMetersPerSecond, 0,
          groundtruthState.velocityMetersPerSecond * groundtruthState.curvatureRadPerMeter
      );
      var wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);

      var x = new MatBuilder<>(Nat.N2(), Nat.N1()).fill(
              wheelSpeeds.leftMetersPerSecond,
              wheelSpeeds.rightMetersPerSecond);

      var xDot = x.minus(prevX).div(dt);

      prevX = x;

      input = (plant.getB().inv()).times(xDot.minus(plant.getA().times(x)));

      if (lastVisionUpdateTime + visionUpdateRate + rand.nextGaussian() * 0.4 < t) {
        if (lastVisionPose != null) {
          estimator.applyPastGlobalMeasurement(
                  lastVisionPose,
                  lastVisionUpdateTime);
        }
        var groundPose = groundtruthState.poseMeters;
        lastVisionPose = new Pose2d(
                new Translation2d(
                        groundPose.getTranslation().getX() + rand.nextGaussian() * 0.0,
                        groundPose.getTranslation().getY() + rand.nextGaussian() * 0.1
                ),
                new Rotation2d(rand.nextGaussian() * 0.01).plus(groundPose.getRotation())
        );
        lastVisionUpdateTime = t;

        visionXs.add(lastVisionPose.getTranslation().getX());
        visionYs.add(lastVisionPose.getTranslation().getY());
      }

      distanceLeft += wheelSpeeds.leftMetersPerSecond * dt;
      distanceRight += wheelSpeeds.rightMetersPerSecond * dt;
      
      distanceLeft += rand.nextGaussian() * 0.01;
      distanceRight += rand.nextGaussian() * 0.01;

      var rotNoise = new Rotation2d(rand.nextGaussian() * 0.01);

      var xHat = estimator.updateWithTime(groundtruthState.poseMeters.getRotation()
                      .plus(rotNoise).getRadians(),
                      distanceLeft,
                      distanceRight,
                      input,
                      t);

      double error =
              groundtruthState.poseMeters.getTranslation().getDistance(
                      new Translation2d(xHat.get(0, 0),
                                        xHat.get(1, 0)));
      if (error > maxError) {
        maxError = error;
      }
      errorSum += error;

      time.add(t);

      trajXs.add(groundtruthState.poseMeters.getTranslation().getX());
      trajYs.add(groundtruthState.poseMeters.getTranslation().getY());
      trajLeftVel.add(wheelSpeeds.leftMetersPerSecond);
      trajRightVel.add(wheelSpeeds.rightMetersPerSecond);

      observerXs.add(xHat.get(0, 0));
      observerYs.add(xHat.get(1, 0));
      observerLeftVelocity.add(xHat.get(3, 0));
      observerRightVelocity.add(xHat.get(4, 0));
      observerLeftVoltageError.add(xHat.get(7, 0));
      observerRightVoltageError.add(xHat.get(8, 0));
      observerAngularVelocityError.add(xHat.get(9, 0));

      t += dt;
    }

    System.out.println(errorSum / (traj.getTotalTimeSeconds() / dt));
    System.out.println(maxError);

    assertEquals(
            0.0, errorSum / (traj.getTotalTimeSeconds() / dt), 0.03,
            "Incorrect mean error"
    );
    assertEquals(
            0.0, maxError, 0.05,
            "Incorrect max error"
    );

    System.out.println("Mean error (meters): " + errorSum / (traj.getTotalTimeSeconds() / dt));
    System.out.println("Max error (meters):  " + maxError);


    /*List<XYChart> charts = new ArrayList<XYChart>();

    var chartBuilder = new XYChartBuilder();
    chartBuilder.title = "The Magic of Sensor Fusion";
    var chart = chartBuilder.build();

    chart.addSeries("Vision", visionXs, visionYs);
    chart.addSeries("Trajectory", trajXs, trajYs);
    chart.addSeries("xHat", observerXs, observerYs);
    charts.add(chart);

    var chartBuilderError = new XYChartBuilder();
    chartBuilderError.title = "Error versus Time";
    var chartError = chartBuilderError.build();

    chartError.addSeries("LeftVoltage", time, observerLeftVoltageError);
    chartError.addSeries("RightVoltage", time, observerRightVoltageError);
    chartError.addSeries("AngularVelocity", time, observerAngularVelocityError);
    charts.add(chartError);

    var chartBuilderLeftVelocity = new XYChartBuilder();
    chartBuilderLeftVelocity.title = "Left Velocity versus Time";
    var chartLeftVelocity = chartBuilderLeftVelocity.build();

    chartLeftVelocity.addSeries("Trajectory", time, trajLeftVel);
    chartLeftVelocity.addSeries("xHat", time, observerLeftVelocity);
    charts.add(chartLeftVelocity);

    var chartBuilderRightVelocity = new XYChartBuilder();
    chartBuilderRightVelocity.title = "Right Velocity versus Time";
    var chartRightVelocity = chartBuilderRightVelocity.build();

    chartRightVelocity.addSeries("Trajectory", time, trajRightVel);
    chartRightVelocity.addSeries("xHat", time, observerRightVelocity);

    charts.add(chartRightVelocity);

    new SwingWrapper<>(charts).displayChartMatrix();
    try {
      Thread.sleep(1000000000);
    } catch (InterruptedException e) {
    }*/
  }
}
