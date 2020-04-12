/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.estimator;

import java.util.function.BiConsumer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.math.StateSpaceUtil;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpiutil.math.MatBuilder;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.MatrixUtils;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N3;

/**
 * This class wraps a Kalman Filter to fuse latency-compensated vision measurements
 * with mecanum drive encoder velocity measurements. It will correct for noisy
 * measurements and encoder drift. It is intended to be an easy but more accurate drop-in for
 * {@link edu.wpi.first.wpilibj.kinematics.MecanumDriveOdometry}.
 *
 * <p>{@link MecanumDrivePoseEstimator#update} should be called every robot loop (if
 * your loops are faster or slower than the default, then you should change the nominal
 * delta time using the secondary constructor.
 *
 * <p>{@link MecanumDrivePoseEstimator#addVisionMeasurement} can be called as
 * infrequently as you want; if you never call it, then this class will behave mostly like regular
 * encoder odometry.
 *
 * <p>Our state-space system is:
 *
 * <p>x = [[x, y, theta]]^T in the field-coordinate system.
 *
 * <p>u = [[vx, vy, omega]]^T in the field-coordinate system.
 *
 * <p>y = [[x, y, theta]]^T in field coords from vision, or y = [[theta]]^T from the gyro.
 */
public class MecanumDrivePoseEstimator {
  private final KalmanFilter<N3, N3, N1> m_observer;
  private final MecanumDriveKinematics m_kinematics;
  private final BiConsumer<Matrix<N3, N1>, Matrix<N3, N1>> m_visionCorrect;
  private final KalmanFilterLatencyCompensator<N3, N3, N1> m_latencyCompensator;

  private final double m_nominalDt; // Seconds
  private double m_prevTimeSeconds = -1.0;

  private Rotation2d m_gyroOffset;
  private Rotation2d m_previousAngle;

  /**
   * Constructs a MecanumDrivePoseEstimator.
   *
   * @param gyroAngle                The current gyro angle.
   * @param initialPoseMeters        The starting pose estimate.
   * @param kinematics               A correctly-configured kinematics object for your drivetrain.
   * @param stateStdDevs             Standard deviations of model states. Increase these numbers to
   *                                 trust your wheel and gyro velocities less.
   * @param localMeasurementStdDevs  Standard deviations of the gyro measurement. Increase this
   *                                 number to trust gyro angle measurements less.
   * @param visionMeasurementStdDevs Standard deviations of the encoder measurements. Increase
   *                                 these numbers to trust vision less.
   */
  public MecanumDrivePoseEstimator(
          Rotation2d gyroAngle, Pose2d initialPoseMeters, MecanumDriveKinematics kinematics,
          Matrix<N3, N1> stateStdDevs, Matrix<N1, N1> localMeasurementStdDevs,
          Matrix<N3, N1> visionMeasurementStdDevs
  ) {
    this(gyroAngle, initialPoseMeters, kinematics, stateStdDevs, localMeasurementStdDevs,
            visionMeasurementStdDevs, 0.02);
  }

  /**
   * Constructs a MecanumDrivePoseEstimator.
   *
   * @param gyroAngle                The current gyro angle.
   * @param initialPoseMeters        The starting pose estimate.
   * @param stateStdDevs             Standard deviations of model states. Increase these numbers to
   *                                 trust your wheel and gyro velocities less.
   * @param localMeasurementStdDevs  Standard deviations of the gyro measurement. Increase this
   *                                 number to trust gyro angle measurements less.
   * @param visionMeasurementStdDevs Standard deviations of the encoder measurements. Increase
   *                                 these numbers to trust vision less.
   * @param nominalDtSeconds         The time in seconds between each robot loop.
   */
  @SuppressWarnings("ParameterName")
  public MecanumDrivePoseEstimator(
          Rotation2d gyroAngle, Pose2d initialPoseMeters, MecanumDriveKinematics kinematics,
          Matrix<N3, N1> stateStdDevs, Matrix<N1, N1> localMeasurementStdDevs,
          Matrix<N3, N1> visionMeasurementStdDevs, double nominalDtSeconds
  ) {
    m_nominalDt = nominalDtSeconds;

    LinearSystem<N3, N3, N1> observerSystem =
        new LinearSystem<>(
            Nat.N3(), Nat.N3(), Nat.N1(),
            MatrixUtils.zeros(Nat.N3(), Nat.N3()), // A
            MatrixUtils.eye(Nat.N3()), // B
            VecBuilder.fill(0, 0, 1).transpose(), // C
            MatrixUtils.zeros(Nat.N1(), Nat.N3()), // D
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill( // uMin
                    Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY
            ),
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill( // uMax
                    Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY
            )
        );
    m_observer = new KalmanFilter<>(Nat.N3(), Nat.N1(), observerSystem, stateStdDevs,
            localMeasurementStdDevs, nominalDtSeconds);
    m_kinematics = kinematics;
    m_latencyCompensator = new KalmanFilterLatencyCompensator<>();

    var visionContR = StateSpaceUtil.makeCovMatrix(Nat.N3(), visionMeasurementStdDevs);
    var visionDiscR = StateSpaceUtil.discretizeR(visionContR, m_nominalDt);
    m_visionCorrect = (u, y) -> m_observer.correct(u, y,
            MatrixUtils.eye(Nat.N3()), MatrixUtils.zeros(Nat.N3(), Nat.N3()), visionDiscR);

    m_observer.setXhat(StateSpaceUtil.poseToVector(initialPoseMeters));
    m_gyroOffset = initialPoseMeters.getRotation().minus(gyroAngle);
    m_previousAngle = initialPoseMeters.getRotation();
  }

  /**
   * Resets the robot's position on the field.
   *
   * <p>The gyroscope angle does not need to be reset here on the user's robot code.
   * The library automatically takes care of offsetting the gyro angle.
   *
   * @param poseMeters The position on the field that your robot is at.
   * @param gyroAngle  The angle reported by the gyroscope.
   */
  public void resetPosition(Pose2d poseMeters, Rotation2d gyroAngle) {
    m_previousAngle = poseMeters.getRotation();
    m_gyroOffset = getEstimatedPosition().getRotation().minus(gyroAngle);
  }

  /**
   * Returns the robot pose at the current time as estimated by the Kalman Filter.
   *
   * @return The estimated robot pose in meters.
   */
  public Pose2d getEstimatedPosition() {
    return new Pose2d(
            m_observer.getXhat(0), m_observer.getXhat(1), new Rotation2d(m_observer.getXhat(2))
    );
  }

  /**
   * Adds a vision measurement to the Kalman Filter. This will correct the odometry
   * pose estimate while still accounting for measurement noise.
   *
   * <p>This method can be called as infrequently as you want, as long as you are calling
   * {@link MecanumDrivePoseEstimator#update} every loop.
   *
   * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
   * @param timestampSeconds      The timestamp of the vision measurement in seconds. Note
   *                              that if you don't use your own time source by calling
   *                              {@link MecanumDrivePoseEstimator#updateWithTime}, then
   *                              you must use a timestamp with an epoch since FPGA startup
   *                              (i.e. the epoch of this timestamp is the same epoch as
   *                              {@link Timer#getFPGATimestamp()}. This means that you should
   *                              use Timer.getFPGATimestamp() as your time source in this case.
   */
  public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
    m_latencyCompensator.applyPastGlobalMeasurement(
            Nat.N3(),
            m_observer, m_nominalDt,
            StateSpaceUtil.poseToVector(visionRobotPoseMeters),
            m_visionCorrect,
            timestampSeconds
    );
  }

  /**
   * Updates the Kalman Filter using only wheel encoder information. Note that this
   * should be called every loop.
   *
   * @param gyroAngle   The current gyro angle.
   * @param wheelSpeeds The speeds of each wheel of the mecanum drive.
   * @return The estimated pose of the robot in meters.
   */
  public Pose2d update(Rotation2d gyroAngle, MecanumDriveWheelSpeeds wheelSpeeds) {
    return updateWithTime(Timer.getFPGATimestamp(), gyroAngle, wheelSpeeds);
  }

  /**
   * Updates the Kalman Filter using only wheel encoder information. Note that this
   * should be called every loop.
   *
   * @param currentTimeSeconds The current time.
   * @param gyroAngle          The current gyro angle.
   * @param wheelSpeeds        The speeds of each wheel of the mecanum drive.
   * @return The estimated pose of the robot in meters.
   */
  @SuppressWarnings("checkstyle:LocalVariableName")
  public Pose2d updateWithTime(double currentTimeSeconds, Rotation2d gyroAngle,
                               MecanumDriveWheelSpeeds wheelSpeeds) {
    double dt = m_prevTimeSeconds >= 0 ? currentTimeSeconds - m_prevTimeSeconds : m_nominalDt;
    m_prevTimeSeconds = currentTimeSeconds;

    var angle = gyroAngle.plus(m_gyroOffset);
    var omega = angle.minus(m_previousAngle).getRadians() / dt;

    var chassisSpeeds = m_kinematics.toChassisSpeeds(wheelSpeeds);
    var fieldRelativeVelocities =
            new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)
                    .rotateBy(angle);

    var u = new MatBuilder<>(Nat.N3(), Nat.N1()).fill(
            fieldRelativeVelocities.getX(),
            fieldRelativeVelocities.getY(),
            omega
    );
    m_previousAngle = angle;

    var localY = VecBuilder.fill(angle.getRadians());
    m_latencyCompensator.addObserverState(m_observer, u, localY, currentTimeSeconds);
    m_observer.predict(u, dt);
    m_observer.correct(u, localY);

    return getEstimatedPosition();
  }
}
