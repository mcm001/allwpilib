/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.estimator;

import java.util.Arrays;
import java.util.List;

import org.ejml.simple.SimpleMatrix;
import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.math.StateSpaceUtil;
import edu.wpi.first.wpilibj.system.NumericalJacobian;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpiutil.math.MatBuilder;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.MatrixUtils;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N2;
import edu.wpi.first.wpiutil.math.numbers.N3;
import edu.wpi.first.wpiutil.math.numbers.N5;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;


public class UnscentedKalmanFilterTest {
  @SuppressWarnings({"LocalVariableName", "ParameterName"})
  public static Matrix<N5, N1> getDynamics(Matrix<N5, N1> x, Matrix<N2, N1> u) {
    var motors = DCMotor.getCIM(2);

    var gHigh = 7.08;
    var rb = 0.8382 / 2.0;
    var r = 0.0746125;
    var m = 63.503;
    var J = 5.6;

    var C1 = -Math.pow(gHigh, 2) * motors.m_KtNMPerAmp
          / (motors.m_KvRadPerSecPerVolt * motors.m_rOhms * r * r);
    var C2 = gHigh * motors.m_KtNMPerAmp / (motors.m_rOhms * r);
    var k1 = 1.0 / m + rb * rb / J;
    var k2 = 1.0 / m - rb * rb / J;

    var vl = x.get(3, 0);
    var vr = x.get(4, 0);
    var Vl = u.get(0, 0);
    var Vr = u.get(1, 0);

    Matrix<N5, N1> result = new Matrix<>(new SimpleMatrix(5, 1));
    var v = 0.5 * (vl + vr);
    result.set(0, 0, v * Math.cos(x.get(2, 0)));
    result.set(1, 0, v * Math.sin(x.get(2, 0)));
    result.set(2, 0, (vr - vl) / (2.0 * rb));
    result.set(3, 0,
          k1 * ((C1 * vl) + (C2 * Vl))
                + k2 * ((C1 * vr) + (C2 * Vr)));
    result.set(4, 0,
          k2 * ((C1 * vl) + (C2 * Vl))
                + k1 * ((C1 * vr) + (C2 * Vr)));
    return result;
  }

  @SuppressWarnings("ParameterName")
  public static Matrix<N3, N1> getLocalMeasurementModel(Matrix<N5, N1> x, Matrix<N2, N1> u) {
    return new MatBuilder<>(Nat.N3(), Nat.N1()).fill(x.get(2, 0), x.get(3, 0), x.get(4, 0));
  }

  @SuppressWarnings("ParameterName")
  public static Matrix<N5, N1> getGlobalMeasurementModel(Matrix<N5, N1> x, Matrix<N2, N1> u) {
    return new MatBuilder<>(Nat.N5(), Nat.N1()).fill(x.get(0, 0), x.get(1, 0), x.get(2, 0),
          x.get(3, 0), x.get(4, 0));
  }

  @SuppressWarnings("ParameterName")
  public static Matrix<N5, N1> noOp(Matrix<N5, N1> x, Matrix<N2, N1> u) {
    return x.copy();
  }

  @Test
  @SuppressWarnings("LocalVariableName")
  public void testInit() {
    assertDoesNotThrow(() -> {
      UnscentedKalmanFilter<N5, N2, N3> observer = new UnscentedKalmanFilter<>(
            Nat.N5(), Nat.N3(),
            UnscentedKalmanFilterTest::getDynamics,
            UnscentedKalmanFilterTest::getLocalMeasurementModel,
            VecBuilder.fill(0.5, 0.5, 10.0, 1.0, 1.0),
            VecBuilder.fill(0.0001, 0.01, 0.01),
            0.00505);

      var u = VecBuilder.fill(12.0, 12.0);
      observer.predict(u, 0.00505);

      var localY = getLocalMeasurementModel(observer.getXhat(), u);
      observer.correct(u, localY);
    });
  }

  @SuppressWarnings({"LocalVariableName", "PMD.AvoidInstantiatingObjectsInLoops"})
  @Test
  public void testConvergence() {
    double dtSeconds = 0.00505;
    double rbMeters = 0.8382 / 2.0; // Robot radius

    UnscentedKalmanFilter<N5, N2, N3> observer =
          new UnscentedKalmanFilter<>(Nat.N5(), Nat.N3(),
                UnscentedKalmanFilterTest::getDynamics,
                UnscentedKalmanFilterTest::getLocalMeasurementModel,
                VecBuilder.fill(0.5, 0.5, 10.0, 1.0, 1.0),
                VecBuilder.fill(0.001, 0.01, 0.01), dtSeconds);

    List<Pose2d> waypoints = Arrays.asList(new Pose2d(2.75, 22.521, new Rotation2d()),
          new Pose2d(24.73, 19.68, Rotation2d.fromDegrees(5.846)));
    var trajectory = TrajectoryGenerator.generateTrajectory(
          waypoints,
          new TrajectoryConfig(8.8, 0.1)
    );

    Matrix<N5, N1> r = MatrixUtils.zeros(Nat.N5(), Nat.N1());

    Matrix<N5, N1> nextR = MatrixUtils.zeros(Nat.N5(), Nat.N1());
    Matrix<N2, N1> u = MatrixUtils.zeros(Nat.N2(), Nat.N1());

    var B = NumericalJacobian.numericalJacobianU(Nat.N5(), Nat.N2(),
          UnscentedKalmanFilterTest::getDynamics, MatrixUtils.zeros(Nat.N5(), Nat.N1()), u);

    double totalTime = trajectory.getTotalTimeSeconds();
    for (int i = 0; i < (totalTime / dtSeconds); i++) {
      var ref = trajectory.sample(dtSeconds * i);
      double vl = ref.velocityMetersPerSecond * (1 - (ref.curvatureRadPerMeter * rbMeters));
      double vr = ref.velocityMetersPerSecond * (1 + (ref.curvatureRadPerMeter * rbMeters));

      nextR.set(0, 0, ref.poseMeters.getTranslation().getX());
      nextR.set(1, 0, ref.poseMeters.getTranslation().getY());
      nextR.set(2, 0, ref.poseMeters.getRotation().getRadians());
      nextR.set(3, 0, vl);
      nextR.set(4, 0, vr);

      var localY =
            getLocalMeasurementModel(observer.getXhat(), MatrixUtils.zeros(Nat.N2(), Nat.N1()));
      var whiteNoiseStdDevs = VecBuilder.fill(0.0001, 0.5, 0.5);
      observer.correct(u,
            localY.plus(StateSpaceUtil.makeWhiteNoiseVector(Nat.N3(), whiteNoiseStdDevs)));

      Matrix<N5, N1> rdot = nextR.minus(r).div(dtSeconds);
      u = new Matrix<>(B.getStorage()
            .solve(rdot.minus(getDynamics(r, MatrixUtils.zeros(Nat.N2(), Nat.N1()))).getStorage()));

      observer.predict(u, dtSeconds);

      r = nextR;
    }

    var localY = getLocalMeasurementModel(observer.getXhat(), u);
    observer.correct(u, localY);

    var globalY = getGlobalMeasurementModel(observer.getXhat(), u);
    var R = StateSpaceUtil.makeCostMatrix(
          VecBuilder.fill(0.01, 0.01, 0.0001, 0.5, 0.5));
    observer.correct(Nat.N5(), u, globalY, UnscentedKalmanFilterTest::getGlobalMeasurementModel, R);

    var finalPosition = trajectory.sample(trajectory.getTotalTimeSeconds());
    assertEquals(finalPosition.poseMeters.getTranslation().getX(), observer.getXhat(0), 1.0);
    assertEquals(finalPosition.poseMeters.getTranslation().getY(), observer.getXhat(1), 1.0);
    assertEquals(finalPosition.poseMeters.getRotation().getRadians(), observer.getXhat(2), 1.0);
    assertEquals(0.0, observer.getXhat(3), 1.0);
    assertEquals(0.0, observer.getXhat(4), 1.0);
  }
}
