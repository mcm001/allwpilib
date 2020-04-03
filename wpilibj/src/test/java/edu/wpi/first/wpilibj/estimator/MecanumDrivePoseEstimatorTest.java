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
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveOdometry;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpiutil.math.VecBuilder;


import static org.junit.jupiter.api.Assertions.assertEquals;

public class MecanumDrivePoseEstimatorTest {
  @Test
  @SuppressWarnings({"LocalVariableName", "PMD.AvoidInstantiatingObjectsInLoops"})
  public void testAccuracy() {
    var kinematics = new MecanumDriveKinematics(
            new Translation2d(1, 1), new Translation2d(1, -1),
            new Translation2d(-1, -1), new Translation2d(-1, 1));

    var estimator = new MecanumDrivePoseEstimator(
            new Rotation2d(), new Pose2d(), kinematics,
            VecBuilder.fill(0.01, 0.01, 0.01), VecBuilder.fill(0.1, 0.1, 0.1)
    );

    var odometry = new MecanumDriveOdometry(kinematics, new Rotation2d());

    var trajectory = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(), new Pose2d(20, 20, Rotation2d.fromDegrees(0)),
                    new Pose2d(54, 54, new Rotation2d())),
            new TrajectoryConfig(0.5, 2));

    var rand = new Random(5190);

    List<Double> trajXs = new ArrayList<>();
    List<Double> trajYs = new ArrayList<>();
    List<Double> observerXs = new ArrayList<>();
    List<Double> observerYs = new ArrayList<>();
    List<Double> visionXs = new ArrayList<>();
    List<Double> visionYs = new ArrayList<>();

    final double dt = 0.02;
    double t = 0.0;

    final double kVisionUpdateRate = 0.1;
    Pose2d lastVisionPose = null;
    double lastVisionUpdateTime = Double.NEGATIVE_INFINITY;

    double maxError = Double.NEGATIVE_INFINITY;
    double errorSum = 0.0;

    while (t <= trajectory.getTotalTimeSeconds()) {
      Trajectory.State groundTruthState = trajectory.sample(t);

      if (lastVisionUpdateTime + kVisionUpdateRate < t) {
        if (lastVisionPose != null) {
          estimator.addVisionMeasurement(lastVisionPose, lastVisionUpdateTime);
        }
        lastVisionPose = groundTruthState.poseMeters.transformBy(
                new Transform2d(new Translation2d(rand.nextGaussian() * 1.0,
                        rand.nextGaussian() * 1.0),
                        new Rotation2d(rand.nextGaussian() * 0.05))
        );
        lastVisionUpdateTime = t;
        visionXs.add(lastVisionPose.getTranslation().getX());
        visionYs.add(lastVisionPose.getTranslation().getY());
      }

      var wheelSpeeds = kinematics.toWheelSpeeds(new ChassisSpeeds(
              groundTruthState.velocityMetersPerSecond, 0,
              groundTruthState.velocityMetersPerSecond * groundTruthState.curvatureRadPerMeter));

      wheelSpeeds.frontLeftMetersPerSecond += rand.nextGaussian() * 0.1;
      wheelSpeeds.frontRightMetersPerSecond += rand.nextGaussian() * 0.1;
      wheelSpeeds.rearLeftMetersPerSecond += rand.nextGaussian() * 0.1;
      wheelSpeeds.rearRightMetersPerSecond += rand.nextGaussian() * 0.1;

      var xhat = estimator.updateWithTime(t,
              groundTruthState.poseMeters.getRotation()
                      .plus(new Rotation2d(rand.nextGaussian() * 0.05)), wheelSpeeds);

      double error =
              groundTruthState.poseMeters.getTranslation().getDistance(xhat.getTranslation());
      if (error > maxError) {
        maxError = error;
      }
      errorSum += error;

      trajXs.add(groundTruthState.poseMeters.getTranslation().getX());
      trajYs.add(groundTruthState.poseMeters.getTranslation().getY());
      observerXs.add(xhat.getTranslation().getX());
      observerYs.add(xhat.getTranslation().getY());

      t += dt;
    }

    System.out.println(errorSum / (trajectory.getTotalTimeSeconds() / dt));
    System.out.println(maxError);

    assertEquals(0.0, errorSum / (trajectory.getTotalTimeSeconds() / dt), 0.2);
    assertEquals(0.0, maxError, 0.2);


    //    var chartBuilder = new XYChartBuilder();
    //    chartBuilder.title = "The Magic of Sensor Fusion";
    //    var chart = chartBuilder.build();
    //
    //    chart.addSeries("Vision", visionXs, visionYs);
    //    chart.addSeries("Trajectory", trajXs, trajYs);
    //    chart.addSeries("xHat", observerXs, observerYs);
    //
    //    new SwingWrapper<>(chart).displayChart();
    //    try {
    //        Thread.sleep(1000000000);
    //    } catch (InterruptedException e) {
    //    }
  }
}
